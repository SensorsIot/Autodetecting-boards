#include <ETH.h>
#include <WiFiUdp.h>
#include <TinyGPSPlus.h>
#include <time.h>
#include <EEPROM.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// Debugging levels
#define DEBUG_STANDARD 1
#define DEBUG_TIME_CRITICAL 0

// Debugging Macros
#if DEBUG_STANDARD
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

#if DEBUG_TIME_CRITICAL
#define DEBUG_CRITICAL_PRINT(x) Serial.print(x)
#define DEBUG_CRITICAL_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_CRITICAL_PRINT(x)
#define DEBUG_CRITICAL_PRINTLN(x)
#endif

// Global Variables
TinyGPSPlus gps;
SemaphoreHandle_t gpsMutex;
WiFiUDP udp;

volatile bool PPSsignal = false;
volatile bool PPSavailable = false;

HardwareSerial gpsSerial(1);

// Baud rate for GPS and correction factor for time update
#define GPSBaud 9600
#define CORRECTION_FACTOR 0

#define EEPROM_SIZE 1

// Board configuration structure including PPS and GPS pins.
struct BoardConfig {
  const char *boardName;
  eth_phy_type_t phyType;
  int ethAddr;
  int powerPin;
  int mdcPin;
  int mdioPin;
  eth_clock_mode_t clkMode;
  int rxPin;   // GPS RX pin
  int txPin;   // GPS TX pin
  int ppsPin;  // PPS signal pin
};

// Define board configurations for different boards.
BoardConfig boardConfigs[] = {
  { "LilyGo PoE", ETH_PHY_LAN8720, 0, -1, 23, 18, ETH_CLOCK_GPIO17_OUT, 15, 4, 14 },
  { "WT32-eth01", ETH_PHY_LAN8720, 1, 16, 23, 18, ETH_CLOCK_GPIO0_IN, 15, 4, 14 }
};
const int numBoards = sizeof(boardConfigs) / sizeof(boardConfigs[0]);

// Global variable to store the selected board configuration.
BoardConfig currentBoardConfig;

int findBoardConfig() {
  if (!EEPROM.begin(EEPROM_SIZE)) {
    DEBUG_PRINTLN("Failed to initialize EEPROM.");
    return -1;
  }
  int lastTested = EEPROM.read(0);
  int currentBoard = (lastTested + 1) % numBoards;

  // Save the current configuration globally.
  currentBoardConfig = boardConfigs[currentBoard];
  DEBUG_PRINT("Initializing Ethernet for ");
  DEBUG_PRINTLN(currentBoardConfig.boardName);

  pinMode(currentBoardConfig.powerPin, OUTPUT);
  digitalWrite(currentBoardConfig.powerPin, LOW);
  delay(100);
  digitalWrite(currentBoardConfig.powerPin, HIGH);
  delay(100);

  bool ethStarted = ETH.begin(currentBoardConfig.phyType,
                              currentBoardConfig.ethAddr,
                              currentBoardConfig.mdcPin,
                              currentBoardConfig.mdioPin,
                              currentBoardConfig.powerPin,
                              currentBoardConfig.clkMode);

  if (!ethStarted) {
    DEBUG_PRINT("Ethernet initialization failed for ");
    DEBUG_PRINTLN(currentBoardConfig.boardName);
    EEPROM.write(0, currentBoard);
    EEPROM.commit();
    ESP.restart();
  }

  DEBUG_PRINT("Ethernet initialized successfully for ");
  DEBUG_PRINTLN(currentBoardConfig.boardName);

  while (ETH.localIP() == IPAddress(0, 0, 0, 0)) {
    DEBUG_PRINTLN("Waiting for IP address...");
    delay(500);
  }

  DEBUG_PRINT("Connected! IP Address: ");
  DEBUG_PRINTLN(ETH.localIP().toString().c_str());
  EEPROM.write(0, 0);
  EEPROM.commit();
  udp.begin(123);  // NTP uses port 123
  return 0;
}

void IRAM_ATTR PPS_ISR() {
  PPSsignal = true;
}

void initPPS() {
  // Use the PPS pin from the current board configuration.
  pinMode(currentBoardConfig.ppsPin, INPUT_PULLDOWN);
  DEBUG_PRINTLN("Checking PPS signal...");
  unsigned long startTime = millis();
  PPSavailable = false;

  // Wait up to 1000 milliseconds for a HIGH signal.
  while (millis() - startTime < 1000) {
    if (digitalRead(currentBoardConfig.ppsPin) == HIGH) {
      PPSavailable = true;
      break;
    }
  }

  if (PPSavailable) {
    DEBUG_PRINTLN("PPS signal detected.");
    attachInterrupt(digitalPinToInterrupt(currentBoardConfig.ppsPin), PPS_ISR, FALLING);
    DEBUG_PRINTLN("PPS initialized. Waiting for signal...");
  } else {
    DEBUG_PRINTLN("PPS signal not detected! Stopping program.");
    // Stop the program by entering an infinite loop.
    while (true) {
      // Optionally, you could add a watchdog reset or low-power sleep here.
    }
  }
}


void initGPS() {
  DEBUG_PRINTLN("Initializing GPS...");
  // Use the RX/TX pins from the current board configuration.
  gpsSerial.begin(GPSBaud, SERIAL_8N1, currentBoardConfig.rxPin, currentBoardConfig.txPin);
  unsigned long startTime = millis();
  while (millis() - startTime < 30000) {
    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
      if (gps.time.isValid()) {
        DEBUG_PRINT("GPS time acquired: ");
        // Format the time as HH:MM:SS, adding a leading zero if needed
        if (gps.time.hour() < 10) DEBUG_PRINT("0");
        DEBUG_PRINT(gps.time.hour());
        DEBUG_PRINT(":");
        if (gps.time.minute() < 10) DEBUG_PRINT("0");
        DEBUG_PRINT(gps.time.minute());
        DEBUG_PRINT(":");
        if (gps.time.second() < 10) DEBUG_PRINT("0");
        DEBUG_PRINTLN(gps.time.second());
        return;
      }
    }
    delay(100);
  }
  DEBUG_PRINTLN("Failed to acquire valid GPS time!");
}

void readGPSTime() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (gps.time.isValid() && gps.date.isValid()) {
    struct tm timeinfo = { 0 };
    timeinfo.tm_year = gps.date.year() - 1900;
    timeinfo.tm_mon = gps.date.month() - 1;
    timeinfo.tm_mday = gps.date.day();
    timeinfo.tm_hour = gps.time.hour();
    timeinfo.tm_min = gps.time.minute();
    timeinfo.tm_sec = gps.time.second();

    time_t unixTime = mktime(&timeinfo) + 2;

    // Wait for the next PPS signal.
    while (!PPSsignal) {}

    struct timeval tv;
    tv.tv_sec = unixTime;
    tv.tv_usec = CORRECTION_FACTOR;
    settimeofday(&tv, NULL);

    DEBUG_CRITICAL_PRINTLN("GPS Time Updated");

    PPSsignal = false;
  }
}

void handleNTPRequest() {
  // Check if a packet has been received
  int packetSize = udp.parsePacket();
  if (packetSize >= 48) {  // NTP packets are 48 bytes long
    uint8_t requestBuffer[48];
    uint8_t responseBuffer[48];

    // Read the incoming packet into requestBuffer
    udp.read(requestBuffer, 48);

    // Record the Receive Timestamp immediately upon packet arrival.
    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint32_t recvSec = tv.tv_sec + 2208988800UL;
    uint32_t recvFrac = (uint32_t)((double)tv.tv_usec * (4294967296.0 / 1000000.0));

    // --- Build the response packet ---
    responseBuffer[0] = 0x24;              // LI=0, VN=4, Mode=4 (server)
    responseBuffer[1] = 1;                 // Stratum 1
    responseBuffer[2] = requestBuffer[2];  // Poll interval copied from request
    responseBuffer[3] = -6;                // Precision (example)

    // Clear Root Delay & Root Dispersion
    for (int i = 4; i < 12; i++) {
      responseBuffer[i] = 0;
    }

    // Reference Identifier ("LOCL")
    responseBuffer[12] = 'L';
    responseBuffer[13] = 'O';
    responseBuffer[14] = 'C';
    responseBuffer[15] = 'L';

    // Reference Timestamp
    responseBuffer[16] = (recvSec >> 24) & 0xFF;
    responseBuffer[17] = (recvSec >> 16) & 0xFF;
    responseBuffer[18] = (recvSec >> 8) & 0xFF;
    responseBuffer[19] = recvSec & 0xFF;
    responseBuffer[20] = (recvFrac >> 24) & 0xFF;
    responseBuffer[21] = (recvFrac >> 16) & 0xFF;
    responseBuffer[22] = (recvFrac >> 8) & 0xFF;
    responseBuffer[23] = recvFrac & 0xFF;

    // Originate Timestamp: copy client's Transmit Timestamp from request (offset 40)
    for (int i = 0; i < 8; i++) {
      responseBuffer[24 + i] = requestBuffer[40 + i];
    }

    // Receive Timestamp
    responseBuffer[32] = (recvSec >> 24) & 0xFF;
    responseBuffer[33] = (recvSec >> 16) & 0xFF;
    responseBuffer[34] = (recvSec >> 8) & 0xFF;
    responseBuffer[35] = recvSec & 0xFF;
    responseBuffer[36] = (recvFrac >> 24) & 0xFF;
    responseBuffer[37] = (recvFrac >> 16) & 0xFF;
    responseBuffer[38] = (recvFrac >> 8) & 0xFF;
    responseBuffer[39] = recvFrac & 0xFF;

    // Transmit Timestamp (recorded just before sending)
    gettimeofday(&tv, NULL);
    uint32_t txSec = tv.tv_sec + 2208988800UL;
    uint32_t txFrac = (uint32_t)((double)tv.tv_usec * (4294967296.0 / 1000000.0));
    responseBuffer[40] = (txSec >> 24) & 0xFF;
    responseBuffer[41] = (txSec >> 16) & 0xFF;
    responseBuffer[42] = (txSec >> 8) & 0xFF;
    responseBuffer[43] = txSec & 0xFF;
    responseBuffer[44] = (txFrac >> 24) & 0xFF;
    responseBuffer[45] = (txFrac >> 16) & 0xFF;
    responseBuffer[46] = (txFrac >> 8) & 0xFF;
    responseBuffer[47] = txFrac & 0xFF;

    // Send the response packet
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.write(responseBuffer, 48);
    udp.endPacket();

    DEBUG_PRINTLN("NTP response sent.");
  }
}

void gpsTask(void *parameter) {
  while (1) {
    readGPSTime();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void ntpTask(void *parameter) {
  while (1) {
    handleNTPRequest();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void setup() {
  Serial.begin(115200);
  findBoardConfig();
  initPPS();
  initGPS();
  gpsMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(gpsTask, "GPSTask", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(ntpTask, "NTPTask", 4096, NULL, 1, NULL, 0);
}

void loop() {
}
