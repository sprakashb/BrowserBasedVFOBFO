//=============================================================================
// uBITX VFO Controller - Professional Edition
//=============================================================================
// Version: 2.0
// Features: WiFi Control, RIT, Band Memory, EEPROM Storage, PTT Control
//=============================================================================

#include <WiFi.h>
#include <WebServer.h>
#include <si5351.h>
#include <Wire.h>
#include <EEPROM.h>
#include <ESPmDNS.h>

//=============================================================================
// CONSTANTS & CONFIGURATION
//=============================================================================

// Hardware Configuration
constexpr int PTT_PIN = 32;
constexpr int PTT_LED_PIN = 2;
constexpr int WIFI_SWITCH_PIN = 27;
constexpr int S_METER_PIN = 36;

// LPF Control Pins
constexpr int TX_LPF_A = 33;
constexpr int TX_LPF_B = 25;
constexpr int TX_LPF_C = 26;

// Frequency Constants
constexpr unsigned long IF_FREQUENCY = 45000000;     // 45 MHz First IF
constexpr unsigned long BFO2_DEFAULT_FREQ = 11998000; // 12 MHz Audio demodulator
constexpr unsigned long BFO1_LSB_FREQ = 32995000;    // 33 MHz for LSB
constexpr unsigned long BFO1_USB_FREQ = 56995000;    // 57 MHz for USB

// Frequency Limits
constexpr unsigned long MIN_VFO_FREQ = 45000000;
constexpr unsigned long MAX_VFO_FREQ = 75000000;
constexpr unsigned long MIN_OPERATING_FREQ = 0;
constexpr unsigned long MAX_OPERATING_FREQ = 30000000;

// RIT Configuration
constexpr long MAX_RIT_OFFSET = 5000;
constexpr long DEFAULT_RIT_STEP = 100;

// Step Sizes
constexpr long STEP_VALUES[] = {10, 100, 1000, 5000, 10000, 100000, 1000000, 10000000};
constexpr const char* STEP_LABELS[] = {"10Hz", "100Hz", "1KHz", "5KHz", "10KHz", "100KHz", "1MHz", "10MHz"};
constexpr int STEP_COUNT = sizeof(STEP_VALUES) / sizeof(STEP_VALUES[0]);

// EEPROM Configuration
constexpr int EEPROM_SIZE = 1024;
constexpr int MAGIC_NUMBER_ADDR = 0;
constexpr int MAGIC_NUMBER = 0xABCD;
constexpr int RIT_ENABLED_ADDR = MAGIC_NUMBER_ADDR + 300;
constexpr int RIT_OFFSET_ADDR = MAGIC_NUMBER_ADDR + 304;
constexpr int BAND_MEMORY_ADDR = MAGIC_NUMBER_ADDR + 200;

// Timing Constants
constexpr unsigned long SAVE_DELAY_MS = 20000;
constexpr unsigned long SERIAL_BAUD = 115200;

//=============================================================================
extern const char MAIN_HTML[] PROGMEM;
//=============================================================================
// DATA STRUCTURES
//=============================================================================

struct BandLimit {
  const char* name;
  unsigned long minFreq;
  unsigned long maxFreq;
};

//ssid/password with your routers' ssid and its password
struct WiFiConfig {
  const char*  ssid;  //"sirius";
  const char* password;  //"c304cret"
  bool isAP;
};

//=============================================================================
// HAM BAND DEFINITIONS
//=============================================================================

constexpr BandLimit BAND_LIMITS[] = {
  {"80m", 3500000, 4000000},
  {"40m", 7000000, 7200000},
  {"20m", 14000000, 14350000},
  {"15m", 21000000, 21450000},
  {"10m", 28000000, 29700000}
};

constexpr int BAND_COUNT = sizeof(BAND_LIMITS) / sizeof(BAND_LIMITS[0]);
constexpr unsigned long BAND_DEFAULT_FREQS[BAND_COUNT] = {3500000, 7000000, 14000000, 21000000, 28000000};

//=============================================================================
// GLOBAL VARIABLES
//=============================================================================

// Core Objects
Si5351 si5351;
WebServer server(80);

// Frequency State
unsigned long vfoFreq = 52000000;
unsigned long bfo1Freq = BFO1_USB_FREQ;
unsigned long bfo2Freq = BFO2_DEFAULT_FREQ;

// Mode State
enum class Mode { LSB, USB, AM, CW };
Mode currentMode = Mode::USB;

// Step Management (0=VFO, 1=BFO1, 2=BFO2)
int stepIndices[3] = {1, 2, 2};

// RIT State
long ritOffset = 0;
bool ritEnabled = false;

// Memory System (0=VFO, 1=BFO1, 2=BFO2)
unsigned long memorySlots[3][10] = {{0}};
unsigned long bandLastFreq[BAND_COUNT];

// PTT State
bool pttActive = false;
bool txLimitEnabled = true;

// EEPROM Management
unsigned long lastChangeTime = 0;
bool saveNeeded = false;

//=============================================================================
// FORWARD DECLARATIONS
//=============================================================================

// Core Functions
void setupSi5351();
void updateAllClocks();
void saveToEEPROM();
void loadFromEEPROM();
void scheduleSave();

// Frequency Management
unsigned long getOperatingFrequency();
void setOperatingFrequency(unsigned long opFreq);
int getCurrentBandIndex(unsigned long freq);
void setBand(int bandIndex);
void updateBFO1ByMode();

// PTT & Filter Management
void setPTT(bool active);
void setTXFilters(unsigned long freq);
bool isWithinHamBand(unsigned long freq);

// Web Handlers
void setupWebServer();
void handleGetConfig();
void handleSetConfig();
void handleGetSetupConfig();
void handleSetSetupConfig();
void handleBandChange();

//=============================================================================
// SETUP & INITIALIZATION
//=============================================================================

void setup() {
  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  EEPROM.begin(EEPROM_SIZE);
  
  initializeHardware();
  loadFromEEPROM();
  initializeBandMemories();
  initializeWiFi();
  initializeSi5351();
  setupWebServer();
  
  Serial.println("uBITX VFO Ready!");
}

void initializeHardware() {
  // PTT Pins
  pinMode(PTT_PIN, OUTPUT);
  pinMode(PTT_LED_PIN, OUTPUT);
  digitalWrite(PTT_PIN, LOW);
  digitalWrite(PTT_LED_PIN, LOW);
  
  // LPF Pins
  const int lpfPins[] = {TX_LPF_A, TX_LPF_B, TX_LPF_C};
  for (int pin : lpfPins) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  
  // S-Meter
  pinMode(S_METER_PIN, INPUT);
  analogReadResolution(12);
  analogSetAttenuation(ADC_6db);
  
  // WiFi Switch
  pinMode(WIFI_SWITCH_PIN, INPUT_PULLUP);
}

void initializeBandMemories() {
  bool hasValidMemories = false;
  for (int i = 0; i < BAND_COUNT; i++) {
    if (bandLastFreq[i] > 0) {
      hasValidMemories = true;
      break;
    }
  }
  
  if (!hasValidMemories) {
    for (int i = 0; i < BAND_COUNT; i++) {
      bandLastFreq[i] = BAND_DEFAULT_FREQS[i];
    }
    Serial.println("Initialized band memories with default frequencies");
  }
}

//=============================================================================
// WIFI MANAGEMENT
//=============================================================================

void initializeWiFi() {
  WiFiConfig config = getWiFiConfig();
  
  if (config.isAP) {
    startAccessPoint(config);
  } else {
    startStationMode(config);
  }
  
  setupMDNS();
}

WiFiConfig getWiFiConfig() {
  constexpr WiFiConfig AP_CONFIG = {"ubitx", "12345678", true};
  constexpr WiFiConfig STA_CONFIG = {"YourSSID", "Password", false};
  
  return digitalRead(WIFI_SWITCH_PIN) ? AP_CONFIG : STA_CONFIG;
}

void startAccessPoint(const WiFiConfig& config) {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(config.ssid, config.password);
  
  Serial.println("\n=================================");
  Serial.println("AP MODE ACTIVE");
  Serial.printf("SSID: %s\n", config.ssid);
  Serial.printf("Password: %s\n", config.password);
  Serial.printf("Web Interface: http://%s\n", WiFi.softAPIP().toString().c_str());
  Serial.println("=================================\n");
}

void startStationMode(const WiFiConfig& config) {
  IPAddress staticIP(192, 168, 1, 200);
  IPAddress gateway(192, 168, 1, 1);
  IPAddress subnet(255, 255, 255, 0);
  IPAddress dns(8, 8, 8, 8);
  
  WiFi.mode(WIFI_STA);
  WiFi.config(staticIP, gateway, subnet, dns);
  WiFi.begin(config.ssid, config.password);
  
  Serial.printf("Connecting to %s", config.ssid);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n=================================");
    Serial.println("STA MODE ACTIVE");
    Serial.printf("Connected to: %s\n", config.ssid);
    Serial.printf("IP Address: http://%s\n", WiFi.localIP().toString().c_str());
    Serial.println("=================================\n");
  } else {
    Serial.println("\nFailed to connect! Falling back to AP mode...");
    startAccessPoint({"ubitx", "12345678", true});
  }
}

void setupMDNS() {
  if (!MDNS.begin("ubitx")) {
    Serial.println("MDNS responder failed!");
  } else {
    MDNS.addService("_http", "_tcp", 80);
    Serial.println("mDNS: http://ubitx.local");
  }
}

//=============================================================================
// SI5351 MANAGEMENT
//=============================================================================

void initializeSi5351() {
  if (!si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0)) {
    Serial.println("Si5351 initialization failed!");
    while (true) {
      delay(1000);
    }
  }
  
  constexpr int CRYSTAL_CORRECTION = 0;
  si5351.set_correction(CRYSTAL_CORRECTION, SI5351_PLL_INPUT_XO);
  
  setupSi5351Outputs();
  updateAllClocks();
  
  Serial.printf("Crystal correction: %d\n", CRYSTAL_CORRECTION);
}

void setupSi5351Outputs() {
  // Disable all outputs first
  si5351.output_enable(SI5351_CLK0, 0);
  si5351.output_enable(SI5351_CLK1, 0);
  si5351.output_enable(SI5351_CLK2, 0);
  
  // Set drive strength
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_8MA);
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_8MA);
}

void updateAllClocks() {
  si5351.set_freq(vfoFreq * 100ULL, SI5351_CLK2);
  si5351.set_freq(bfo1Freq * 100ULL, SI5351_CLK1);
  si5351.set_freq(bfo2Freq * 100ULL, SI5351_CLK0);
  
  si5351.output_enable(SI5351_CLK2, 1);
  si5351.output_enable(SI5351_CLK1, 1);
  si5351.output_enable(SI5351_CLK0, 1);
  
  Serial.println("--- Clock Update ---");
  Serial.printf("VFO: %lu Hz (Operating: %lu Hz)\n", vfoFreq, getOperatingFrequency());
  Serial.printf("BFO1: %lu Hz\n", bfo1Freq);
  Serial.printf("BFO2: %lu Hz\n", bfo2Freq);
  
  if (ritEnabled) {
    Serial.printf("RIT: %s%ld Hz\n", ritOffset >= 0 ? "+" : "", ritOffset);
  }
}

//=============================================================================
// FREQUENCY MANAGEMENT
//=============================================================================

unsigned long getOperatingFrequency() {
  return vfoFreq - IF_FREQUENCY;
}

void setOperatingFrequency(unsigned long opFreq) {
  // Save current band frequency before changing
  unsigned long currentOpFreq = getOperatingFrequency();
  int currentBand = getCurrentBandIndex(currentOpFreq);
  
  if (currentBand >= 0 && currentBand < BAND_COUNT) {
    bandLastFreq[currentBand] = currentOpFreq;
  }
  
  // Apply new frequency
  vfoFreq = constrain(opFreq + IF_FREQUENCY, MIN_VFO_FREQ, MAX_VFO_FREQ);
  scheduleSave();
}

int getCurrentBandIndex(unsigned long freq) {
  for (int i = 0; i < BAND_COUNT; i++) {
    if (freq >= BAND_LIMITS[i].minFreq && freq <= BAND_LIMITS[i].maxFreq) {
      return i;
    }
  }
  return -1;
}

void setBand(int bandIndex) {
  if (bandIndex < 0 || bandIndex >= BAND_COUNT) return;
  
  unsigned long newFreq = constrain(bandLastFreq[bandIndex], 
                                     BAND_LIMITS[bandIndex].minFreq, 
                                     BAND_LIMITS[bandIndex].maxFreq);
  
  setOperatingFrequency(newFreq);
  updateAllClocks();
  
  Serial.printf("Switched to %s band at %lu Hz\n", BAND_LIMITS[bandIndex].name, newFreq);
}

void updateBFO1ByMode() {
  bfo1Freq = (currentMode == Mode::USB) ? BFO1_USB_FREQ : BFO1_LSB_FREQ;
}

//=============================================================================
// PTT & FILTER MANAGEMENT
//=============================================================================

// Modified setPTT function to resetTx filters
void setPTT(bool active) {
  if (active == pttActive) return;
  
  unsigned long txFreq = getOperatingFrequency();
  
  if (active && !isWithinHamBand(txFreq)) {
    Serial.printf("TX BLOCKED: %lu Hz outside ham bands!\n", txFreq);
    flashErrorLED(3);
    return;
  }
  
  pttActive = active;
  digitalWrite(PTT_PIN, active ? HIGH : LOW);
  digitalWrite(PTT_LED_PIN, active ? HIGH : LOW);
  
  if (active) {
    setTXFilters(txFreq);  // Set TX filters
    Serial.printf("TX ON - %lu Hz (%s)\n", txFreq, getBandName(txFreq));
  } else {
    setRXFilters();        // Reset to RX mode
    updateAllClocks();     // Restore RX clocks if needed
    Serial.println("TX OFF");
  }
}
/*void setPTT(bool active) {
  if (active == pttActive) return;
  
  unsigned long txFreq = getOperatingFrequency();
  
  if (active && !isWithinHamBand(txFreq)) {
    Serial.printf("TX BLOCKED: %lu Hz outside ham bands!\n", txFreq);
    flashErrorLED(3);
    return;
  }
  
  pttActive = active;
  digitalWrite(PTT_PIN, active ? HIGH : LOW);
  digitalWrite(PTT_LED_PIN, active ? HIGH : LOW);
  
  if (active) {
    setTXFilters(txFreq);
    Serial.printf("TX ON - %lu Hz (%s)\n", txFreq, getBandName(txFreq));
  } else {
    updateAllClocks();
    Serial.println("TX OFF");
  }
}*/

void flashErrorLED(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(PTT_LED_PIN, HIGH);
    delay(100);
    digitalWrite(PTT_LED_PIN, LOW);
    delay(100);
  }
}

void setTXFilters(unsigned long freq) {
  // LPF selection logic
  if (freq >= 21000000) {
    setLPF(0, 0, 0);  // 10m/12m/15m
  } else if (freq >= 14000000) {
    setLPF(1, 0, 0);  // 17m/20m
  } else if (freq >= 7000000) {
    setLPF(1, 1, 0);  // 30m/40m
  } else {
    setLPF(1, 1, 1);  // 60m/80m/160m
  }
}

// function to set RX filters (all OFF or safe state)
void setRXFilters() {
  // Set all LPF pins to LOW (disabled/through for RX)
  digitalWrite(TX_LPF_A, LOW);
  digitalWrite(TX_LPF_B, LOW);
  digitalWrite(TX_LPF_C, LOW);
  Serial.println("LPF: RX Mode - All filters disabled");
}

void setLPF(bool a, bool b, bool c) {
  digitalWrite(TX_LPF_A, a ? HIGH : LOW);
  digitalWrite(TX_LPF_B, b ? HIGH : LOW);
  digitalWrite(TX_LPF_C, c ? HIGH : LOW);
}

bool isWithinHamBand(unsigned long freq) {
  if (!txLimitEnabled) return true;
  
  for (const auto& band : BAND_LIMITS) {
    if (freq >= band.minFreq && freq <= band.maxFreq) {
      return true;
    }
  }
  return false;
}

const char* getBandName(unsigned long freq) {
  for (const auto& band : BAND_LIMITS) {
    if (freq >= band.minFreq && freq <= band.maxFreq) {
      return band.name;
    }
  }
  return "OUT-OF-BAND";
}

//=============================================================================
// WEB SERVER
//=============================================================================

void setupWebServer() {
  server.on("/", []() { server.send(200, "text/html", MAIN_HTML); });
  server.on("/getConfig", handleGetConfig);
  server.on("/setConfig", HTTP_POST, handleSetConfig);
  server.on("/getSetupConfig", handleGetSetupConfig);
  server.on("/setSetupConfig", HTTP_POST, handleSetSetupConfig);
  server.on("/setBand", handleBandChange);
  server.on("/ptt", handlePTT);
  server.on("/getPTT", []() { server.send(200, "text/plain", pttActive ? "1" : "0"); });
  server.on("/getRIT", handleGetRIT);
  server.on("/setRIT", HTTP_POST, handleSetRIT);
  server.on("/getS", handleGetSMeter);
  server.on("/getTxLimit", []() { server.send(200, "text/plain", txLimitEnabled ? "1" : "0"); });
  
  server.begin();
}

void handleGetConfig() {
  String json = "{";
  json += "\"vfoFreq\":" + String(vfoFreq) + ",";
  json += "\"mode\":\"" + String(currentMode == Mode::USB ? "USB" : "LSB") + "\",";
  json += "\"stepIdx\":" + String(stepIndices[0]) + ",";
  json += "\"bfo1Usb\":" + String(BFO1_USB_FREQ) + ",";
  json += "\"bfo1Lsb\":" + String(BFO1_LSB_FREQ) + ",";
  json += "\"bfo2Default\":" + String(BFO2_DEFAULT_FREQ) + ",";
  json += "\"ifFrequency\":" + String(IF_FREQUENCY) + ",";
  json += "\"memories\":[";
  
  for (int bank = 0; bank < 3; bank++) {
    json += "[";
    for (int slot = 0; slot < 10; slot++) {
      json += String(memorySlots[bank][slot]);
      if (slot < 9) json += ",";
    }
    json += "]";
    if (bank < 2) json += ",";
  }
  
  json += "]}";
  server.send(200, "application/json", json);
}

void handleSetConfig() {
  if (!server.hasArg("plain")) {
    server.send(400, "text/plain", "No data");
    return;
  }
  
  String body = server.arg("plain");
  parseVFOConfig(body);
  parseMemoriesFromJson(body);
  
  updateAllClocks();
  scheduleSave();
  server.send(200, "text/plain", "OK");
}

void handleGetSetupConfig() {
  String json = "{";
  json += "\"bfo1Freq\":" + String(bfo1Freq) + ",";
  json += "\"bfo2Freq\":" + String(bfo2Freq) + ",";
  json += "\"bfo1StepIdx\":" + String(stepIndices[1]) + ",";
  json += "\"bfo2StepIdx\":" + String(stepIndices[2]) + ",";
  json += "\"bfo1Memories\":[";
  
  for (int i = 0; i < 10; i++) {
    json += String(memorySlots[1][i]);
    if (i < 9) json += ",";
  }
  
  json += "],\"bfo2Memories\":[";
  
  for (int i = 0; i < 10; i++) {
    json += String(memorySlots[2][i]);
    if (i < 9) json += ",";
  }
  
  json += "]}";
  server.send(200, "application/json", json);
}

void handleSetSetupConfig() {
  if (!server.hasArg("plain")) {
    server.send(400, "text/plain", "No data");
    return;
  }
  
  String body = server.arg("plain");
  parseBFOConfig(body);
  parseBFO1MemoriesFromJson(body);
  parseBFO2MemoriesFromJson(body);
  
  updateAllClocks();
  saveToEEPROM();  // Immediate save for BFO settings
  server.send(200, "text/plain", "OK");
}

void handleBandChange() {
  int bandIndex = server.arg("band").toInt();
  
  if (bandIndex >= 0 && bandIndex < BAND_COUNT) {
    setBand(bandIndex);
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Invalid band");
  }
}

void handlePTT() {
  String action = server.arg("action");
  
  if (action == "on") {
    setPTT(true);
    server.send(200, "text/plain", "PTT ON");
  } else if (action == "off") {
    setPTT(false);
    server.send(200, "text/plain", "PTT OFF");
  } else {
    server.send(400, "text/plain", "Invalid action");
  }
}

void handleGetRIT() {
  String json = "{";
  json += "\"enabled\":" + String(ritEnabled ? "true" : "false") + ",";
  json += "\"offset\":" + String(ritOffset);
  json += "}";
  server.send(200, "application/json", json);
}

void handleSetRIT() {
  if (!server.hasArg("plain")) {
    server.send(400, "text/plain", "No data");
    return;
  }
  
  String body = server.arg("plain");
  ritEnabled = (body.indexOf("\"enabled\":true") != -1);
  
  int offsetStart = body.indexOf("\"offset\":");
  if (offsetStart != -1) {
    offsetStart += 9;
    int offsetEnd = body.indexOf("}", offsetStart);
    ritOffset = body.substring(offsetStart, offsetEnd).toInt();
  }
  
  scheduleSave();
  server.send(200, "text/plain", "OK");
}

void handleGetSMeter() {
  int val = analogRead(S_METER_PIN);
  int percent = constrain(map(val, 0, 1200, 0, 100), 0, 100);
  server.send(200, "text/plain", String(percent));
}

//=============================================================================
// JSON PARSING HELPERS
//=============================================================================

void parseVFOConfig(const String& json) {
  // Extract VFO frequency
  unsigned long newVFO = extractJsonValue(json, "vfoFreq");
  if (newVFO >= MIN_VFO_FREQ && newVFO <= MAX_VFO_FREQ) {
    vfoFreq = newVFO;
  }
  
  // Extract mode
  int modeStart = json.indexOf("\"mode\":\"");
  if (modeStart != -1) {
    modeStart += 8;
    int modeEnd = json.indexOf("\"", modeStart);
    if (modeEnd != -1) {
      String modeStr = json.substring(modeStart, modeEnd);
      currentMode = (modeStr == "USB") ? Mode::USB : Mode::LSB;
      updateBFO1ByMode();
    }
  }
  
  // Extract step index
  int stepIdx = extractJsonValue(json, "stepIdx");
  if (stepIdx >= 0 && stepIdx < STEP_COUNT) {
    stepIndices[0] = stepIdx;
  }
}

void parseBFOConfig(const String& json) {
  unsigned long newBFO1 = extractJsonValue(json, "bfo1Freq");
  if (newBFO1 >= 1000000 && newBFO1 <= 60000000) {
    bfo1Freq = newBFO1;
  }
  
  unsigned long newBFO2 = extractJsonValue(json, "bfo2Freq");
  if (newBFO2 >= 1000000 && newBFO2 <= 60000000) {
    bfo2Freq = newBFO2;
  }
  
  int bfo1Step = extractJsonValue(json, "bfo1StepIdx");
  if (bfo1Step >= 0 && bfo1Step < STEP_COUNT) {
    stepIndices[1] = bfo1Step;
  }
  
  int bfo2Step = extractJsonValue(json, "bfo2StepIdx");
  if (bfo2Step >= 0 && bfo2Step < STEP_COUNT) {
    stepIndices[2] = bfo2Step;
  }
}

unsigned long extractJsonValue(const String& json, const String& key) {
  String searchKey = "\"" + key + "\":";
  int startPos = json.indexOf(searchKey);
  
  if (startPos == -1) return 0;
  
  startPos += searchKey.length();
  int endPos = json.indexOf(",", startPos);
  if (endPos == -1) endPos = json.indexOf("}", startPos);
  if (endPos == -1) endPos = json.length();
  
  String valueStr = json.substring(startPos, endPos);
  valueStr.trim();
  
  return valueStr.toInt();
}

void parseMemoriesFromJson(const String& json) {
  int memStart = json.indexOf("\"memories\":[[");
  if (memStart == -1) return;
  
  int arrayStart = memStart + 12;
  int arrayEnd = json.indexOf("]]", arrayStart);
  if (arrayEnd == -1) return;
  
  String memData = json.substring(arrayStart, arrayEnd + 1);
  int currentPos = 0;
  
  for (int bank = 0; bank < 3; bank++) {
    int bankStart = memData.indexOf("[", currentPos);
    if (bankStart == -1) break;
    
    int bankEnd = memData.indexOf("]", bankStart);
    if (bankEnd == -1) break;
    
    String bankStr = memData.substring(bankStart + 1, bankEnd);
    parseCommaSeparatedValues(bankStr, memorySlots[bank], 10);
    
    currentPos = bankEnd + 1;
  }
}

void parseBFO1MemoriesFromJson(const String& json) {
  parseArrayFromJson(json, "\"bfo1Memories\":[", memorySlots[1], 10);
}

void parseBFO2MemoriesFromJson(const String& json) {
  parseArrayFromJson(json, "\"bfo2Memories\":[", memorySlots[2], 10);
}

void parseArrayFromJson(const String& json, const String& key, unsigned long* target, int maxSize) {
  int startPos = json.indexOf(key);
  if (startPos == -1) return;
  
  startPos += key.length();
  int endPos = json.indexOf("]", startPos);
  if (endPos == -1) return;
  
  String arrayStr = json.substring(startPos, endPos);
  parseCommaSeparatedValues(arrayStr, target, maxSize);
}

void parseCommaSeparatedValues(const String& str, unsigned long* target, int maxCount) {
  int valStart = 0;
  int count = 0;
  
  while (count < maxCount && valStart < str.length()) {
    int valEnd = str.indexOf(",", valStart);
    if (valEnd == -1) valEnd = str.length();
    
    String valStr = str.substring(valStart, valEnd);
    valStr.trim();
    
    if (valStr.length() > 0 && valStr != "null") {
      target[count] = valStr.toInt();
    }
    
    valStart = valEnd + 1;
    count++;
  }
}

//=============================================================================
// EEPROM MANAGEMENT
//=============================================================================

void saveToEEPROM() {
  Serial.println("Saving configuration to EEPROM...");
  
  // Save header
  EEPROM.put(MAGIC_NUMBER_ADDR, MAGIC_NUMBER);
  EEPROM.put(MAGIC_NUMBER_ADDR + 4, vfoFreq);
  EEPROM.put(MAGIC_NUMBER_ADDR + 8, bfo1Freq);
  EEPROM.put(MAGIC_NUMBER_ADDR + 12, bfo2Freq);
  EEPROM.put(MAGIC_NUMBER_ADDR + 16, static_cast<int>(currentMode));
  
  // Save step indices
  for (int i = 0; i < 3; i++) {
    EEPROM.put(MAGIC_NUMBER_ADDR + 20 + (i * 4), stepIndices[i]);
  }
  
  // Save RIT state
  EEPROM.put(RIT_ENABLED_ADDR, ritEnabled);
  EEPROM.put(RIT_OFFSET_ADDR, ritOffset);
  
  // Save band memories
  for (int i = 0; i < BAND_COUNT; i++) {
    EEPROM.put(BAND_MEMORY_ADDR + (i * 4), bandLastFreq[i]);
  }
  
  // Save memory slots
  int memAddr = MAGIC_NUMBER_ADDR + 40;
  for (int bank = 0; bank < 3; bank++) {
    for (int slot = 0; slot < 10; slot++) {
      EEPROM.put(memAddr + (bank * 40) + (slot * 4), memorySlots[bank][slot]);
    }
  }
  
  if (EEPROM.commit()) {
    Serial.println("EEPROM save successful");
  } else {
    Serial.println("EEPROM save FAILED!");
  }
}

void loadFromEEPROM() {
  int magic;
  EEPROM.get(MAGIC_NUMBER_ADDR, magic);
  
  if (magic != MAGIC_NUMBER) {
    Serial.println("No valid config found, using defaults");
    setDefaultConfiguration();
    return;
  }
  
  Serial.println("Loading saved configuration...");
  
  // Load header
  EEPROM.get(MAGIC_NUMBER_ADDR + 4, vfoFreq);
  EEPROM.get(MAGIC_NUMBER_ADDR + 8, bfo1Freq);
  EEPROM.get(MAGIC_NUMBER_ADDR + 12, bfo2Freq);
  
  int mode;
  EEPROM.get(MAGIC_NUMBER_ADDR + 16, mode);
  currentMode = static_cast<Mode>(mode);
  
  // Load step indices
  for (int i = 0; i < 3; i++) {
    EEPROM.get(MAGIC_NUMBER_ADDR + 20 + (i * 4), stepIndices[i]);
  }
  
  // Load RIT state
  EEPROM.get(RIT_ENABLED_ADDR, ritEnabled);
  EEPROM.get(RIT_OFFSET_ADDR, ritOffset);
  
  // Load band memories
  for (int i = 0; i < BAND_COUNT; i++) {
    unsigned long savedFreq;
    EEPROM.get(BAND_MEMORY_ADDR + (i * 4), savedFreq);
    bandLastFreq[i] = (savedFreq >= MIN_OPERATING_FREQ && savedFreq <= MAX_OPERATING_FREQ) 
                      ? savedFreq : BAND_DEFAULT_FREQS[i];
  }
  
  // Load memory slots
  int memAddr = MAGIC_NUMBER_ADDR + 40;
  for (int bank = 0; bank < 3; bank++) {
    for (int slot = 0; slot < 10; slot++) {
      unsigned long tempFreq;
      EEPROM.get(memAddr + (bank * 40) + (slot * 4), tempFreq);
      memorySlots[bank][slot] = (tempFreq > 0 && tempFreq < 100000000) ? tempFreq : 0;
    }
  }
  
  validateLoadedConfiguration();
  Serial.println("Configuration loaded successfully");
}

void setDefaultConfiguration() {
  vfoFreq = 52000000;
  bfo1Freq = BFO1_USB_FREQ;
  bfo2Freq = BFO2_DEFAULT_FREQ;
  currentMode = Mode::USB;
  
  stepIndices[0] = 1;
  stepIndices[1] = 2;
  stepIndices[2] = 2;
  
  ritEnabled = false;
  ritOffset = 0;
  
  for (int i = 0; i < BAND_COUNT; i++) {
    bandLastFreq[i] = BAND_DEFAULT_FREQS[i];
  }
  
  for (int bank = 0; bank < 3; bank++) {
    for (int slot = 0; slot < 10; slot++) {
      memorySlots[bank][slot] = 0;
    }
  }
}

void validateLoadedConfiguration() {
  vfoFreq = constrain(vfoFreq, MIN_VFO_FREQ, MAX_VFO_FREQ);
  bfo1Freq = constrain(bfo1Freq, 1000000UL, 60000000UL);
  bfo2Freq = constrain(bfo2Freq, 1000000UL, 60000000UL);
  
  stepIndices[1] = constrain(stepIndices[1], 0, STEP_COUNT - 1);
  stepIndices[2] = constrain(stepIndices[2], 0, STEP_COUNT - 1);
  ritOffset = constrain(ritOffset, -MAX_RIT_OFFSET, MAX_RIT_OFFSET);
}

void scheduleSave() {
  lastChangeTime = millis();
  saveNeeded = true;
}

//=============================================================================
// MAIN LOOP
//=============================================================================

void loop() {
  server.handleClient();
  
  if (saveNeeded && (millis() - lastChangeTime >= SAVE_DELAY_MS)) {
    saveToEEPROM();
    saveNeeded = false;
  }
}

//=============================================================================
// HTML CONTENT (Placeholder - your existing MAIN_HTML goes here)
//=============================================================================
// ... (Your existing MAIN_HTML constant here) ...
// ==================== MAIN HTML ====================
const char MAIN_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no, viewport-fit=cover">
<meta charset="UTF-8">
<title>uBITX VFO Controller</title>
<style>
  * {
    -webkit-tap-highlight-color: transparent;
    user-select: none;
    touch-action: manipulation;
    box-sizing: border-box;
  }
  
  body {
    margin: 0;
    padding: 5px;
    background: #0a0a0a;
    font-family: 'Arial Black', Arial, sans-serif;
    touch-action: pan-y pinch-zoom;
  }
  
  .container {
    max-width: 900px;
    margin: 0 auto;
    background: #2c3e50;
    border-radius: 16px;
    padding: 10px;
    box-shadow: 0 5px 20px rgba(0,0,0,0.5);
  }
  
  .tab-container {
    display: flex;
    gap: 6px;
    margin-bottom: 10px;
    background: #1a1a2e;
    border-radius: 12px;
    padding: 4px;
  }
  
  .tab-btn {
    flex: 1;
    padding: 8px;
    background: #34495e;
    border: none;
    border-radius: 10px;
    color: #ecf0f1;
    font-weight: bold;
    cursor: pointer;
    text-align: center;
    font-size: 12px;
  }
  
  .tab-btn.active {
    background: #e74c3c;
    color: white;
  }
  
  .tab-content {
    display: none;
  }
  
  .tab-content.active {
    display: block;
  }
  
  .display {
    background: #2980b9;
    border-radius: 12px;
    padding: 10px;
    margin-bottom: 8px;
    border: 2px solid #f1c40f;
  }
  
  .display.memory-mode {
    background: #e67e22;
  }
  
  .display-info {
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-bottom: 5px;
    font-size: 11px;
  }
  
  #mode-label {
    background: #1a1a2e;
    padding: 4px 10px;
    border-radius: 12px;
    color: #f1c40f;
    font-weight: bold;
  }
  
  #band-status {
    background: #1a1a2e;
    padding: 4px 10px;
    border-radius: 12px;
    font-size: 10px;
    font-weight: bold;
    color: white;
  }
  
  #step-label {
    background: #1a1a2e;
    padding: 4px 10px;
    border-radius: 12px;
    color: #aaa;
  }
  
  #f-display {
    font-size: 32px;
    text-align: center;
    font-family: 'Courier New', monospace;
    font-weight: 900;
    margin: 5px 0;
    color: white;
    text-shadow: 2px 2px 4px #000;
    letter-spacing: 2px;
  }
  
  .mode-band-row {
    display: flex;
    gap: 8px;
    margin: 8px 0;
  }
  
  .mode-selector {
    display: flex;
    gap: 6px;
    flex: 1;
  }
  
  .mode-btn {
    flex: 1;
    padding: 6px 8px;
    background: #34495e;
    border: 2px solid #f1c40f;
    border-radius: 8px;
    text-align: center;
    cursor: pointer;
    color: white;
    font-weight: bold;
    font-size: 12px;
  }
  
  .mode-btn.active {
    background: #e74c3c;
  }
  
  .band-buttons {
    display: flex;
    gap: 5px;
    flex: 2;
  }
  
  .band-btn {
    flex: 1;
    background: #7f0000;
    padding: 6px 0;
    text-align: center;
    border-radius: 8px;
    color: white;
    font-weight: bold;
    cursor: pointer;
    border: 2px solid #f1c40f;
    font-size: 11px;
  }
  
  .band-btn.active-band {
    background: #27ae60;
  }
  
  .knob-section {
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 12px;
    margin: 8px 0;
  }
  
  .vslider-container {
    display: flex;
    flex-direction: column;
    align-items: center;
    background: #1a1a2e;
    border-radius: 16px;
    padding: 8px 4px;
    width: 50px;
  }
  
  .vslider-label {
    font-size: 11px;
    font-weight: bold;
    text-align: center;
  }
  
  .vslider-label.slow { color: #2ecc71; }
  .vslider-label.fast { color: #e74c3c; }
  
  .vslider {
    -webkit-appearance: slider-vertical;
    appearance: slider-vertical;
    width: 20px;
    height: 70px;
    background: linear-gradient(to top, #e74c3c, #f1c40f, #2ecc71);
    border-radius: 10px;
    outline: none;
    cursor: pointer;
  }
  
  .vslider::-webkit-slider-thumb {
    -webkit-appearance: none;
    appearance: none;
    width: 16px;
    height: 16px;
    background: white;
    border-radius: 50%;
    cursor: pointer;
  }
  
  .vslider-value {
    background: #000;
    padding: 2px 5px;
    border-radius: 8px;
    font-size: 9px;
    font-weight: bold;
    color: #f1c40f;
    margin-top: 4px;
  }
  
  .knob {
    width: 110px;
    height: 110px;
    background: conic-gradient(from 0deg, #555, #999 25%, #555 50%, #999 75%, #555);
    border-radius: 50%;
    border: 6px solid #2c3e50;
    cursor: pointer;
    box-shadow: 0 5px 15px rgba(0,0,0,0.3);
    position: relative;
  }
  
  .knob::after {
    content: '';
    position: absolute;
    top: 12px;
    left: 50%;
    width: 10px;
    height: 10px;
    background: #e74c3c;
    border-radius: 50%;
    transform: translateX(-50%);
  }
  
  .btn-group {
    display: flex;
    gap: 5px;
    margin: 6px 0;
  }
  
  .btn-group-ptt {
    display: flex;
    gap: 6px;
    margin: 8px 0;
  }
  
  .btn {
    flex: 1;
    padding: 6px 4px;
    background: #34495e;
    border: 2px solid #f1c40f;
    border-radius: 8px;
    color: white;
    font-weight: bold;
    text-align: center;
    cursor: pointer;
    font-size: 10px;
  }
  
  .btn:active {
    transform: scale(0.96);
  }
  
  .btn-primary {
    background: #27ae60;
  }
  
  .btn-rit {
    background: #e67e22;
  }
  
  .btn-rit.active {
    background: #27ae60;
  }
  
 .btn-ptt {
    background: #c0392b;
    flex: 2;
    padding: 14px 4px;
    font-size: 14px;
    min-height: 56px;    /* Apple's recommended minimum touch target */
    border-radius: 12px;  /* Slightly more rounded */
}
  
  .btn-ptt-active {
    background: #e74c3c;
    animation: pulse 0.8s infinite;
  }
  
  .btn-ptt-locked {
    background: #27ae60;
  }
  
  .btn-memory {
    background: #8e44ad;
  }
  
  .s-meter {
    background: #1a1a2e;
    padding: 6px;
    border-radius: 10px;
    margin: 8px 0;
  }
  
  .s-grid {
    display: flex;
    gap: 3px;
    height: 12px;
  }
  
  .s-seg {
    flex: 1;
    background: #2c3e50;
    border-radius: 2px;
  }
  
  .s-seg.on {
    background: #2ecc71;
  }
  
  .info-text {
    background: #1a1a2e;
    padding: 4px;
    border-radius: 8px;
    font-size: 8px;
    text-align: center;
    margin: 6px 0;
    color: #ffffff;
  }
  
  .vfo-landscape {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 10px;
    align-items: start;
  }
  
  .bfo-grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 12px;
  }
  
  .clock-card {
    background: #34495e;
    border-radius: 12px;
    padding: 10px;
    border: 2px solid #f1c40f;
  }
  
  .clock-title {
    font-size: 12px;
    font-weight: bold;
    color: #f1c40f;
    margin-bottom: 8px;
    text-align: center;
  }
  
  .clock-freq {
    font-size: 18px;
    font-family: 'Courier New', monospace;
    color: white;
    text-align: center;
    margin: 8px 0;
    padding: 8px;
    background: #1a1a2e;
    border-radius: 10px;
  }

  .clock-info {
    font-size: 10px;
    color: #ffffff;  /* ← Change this color */
    text-align: center;
    margin-bottom: 10px;
  }

  .step-label {
    color: white;  /* ← Change this color */
    font-size: 11px;
    margin-bottom: 5px;
    text-align: center;
  }

  .step-value {
    color: #f1c40f;  /* ← Change this color (currently gold/yellow) */
    font-weight: bold;
  }

  .mem-panel {
    background: #1a1a2e;
    border-radius: 12px;
    padding: 10px;
    margin-bottom: 10px;
  }
  
  .mem-header {
    color: #f1c40f;
    font-size: 11px;
    margin-bottom: 8px;
    font-weight: bold;
  }
  
  .mem-buttons {
    display: grid;
    grid-template-columns: repeat(5, 1fr);
    gap: 6px;
  }
  
  .mem-btn {
    background: #8e44ad;
    padding: 6px;
    text-align: center;
    border-radius: 8px;
    color: white;
    font-size: 10px;
    cursor: pointer;
  }
  
  .signature {
    text-align: center;
    color: #666;
    font-size: 9px;
    margin-top: 10px;
  }
  
  @keyframes pulse {
    0%, 100% { opacity: 1; }
    50% { opacity: 0.7; }
  }
  
  @media (max-width: 600px) {
    .vfo-landscape { grid-template-columns: 1fr; }
    .mode-band-row { flex-direction: column; }
    .bfo-grid { grid-template-columns: 1fr; }
    #f-display { font-size: 28px; }
    .knob { width: 100px; height: 100px; }
  }
</style>
</head>
<body>
<div class="container">
  <div class="tab-container">
    <div class="tab-btn active" data-tab="vfo">📻 VFO</div>
    <div class="tab-btn" data-tab="bfo">🔧 BFO</div>
    <div class="tab-btn" data-tab="memory">💾 Memory</div>
  </div>
  
  <!-- VFO TAB -->
  <div id="vfo-tab" class="tab-content active">
    <div class="vfo-landscape">
      <div>
        <div class="display" id="display">
          <div class="display-info">
            <span id="mode-label">USB</span>
<!--            <span id="band-status">40m</span>  -->
            <span id="step-label">100Hz</span>
          </div>
          <div id="f-display">07.000.000</div>
        </div>
        
        <div class="s-meter">
          <div class="s-grid" id="s-grid"></div>
        </div>
        
        <div class="mode-band-row">
          <div class="mode-selector">
            <div class="mode-btn" id="mode-lsb">LSB</div>
            <div class="mode-btn" id="mode-usb">USB</div>
          </div>
          <div class="band-buttons">
            <div class="band-btn" data-band="3500000">80</div>
            <div class="band-btn" data-band="7000000">40</div>
            <div class="band-btn" data-band="14000000">20</div>
            <div class="band-btn" data-band="21000000">15</div>
            <div class="band-btn" data-band="28000000">10</div>
          </div>
        </div>
        
        <div class="btn-group-ptt">
          <div class="btn btn-ptt" id="ptt-momentary">🔴 PTT</div>
<!--          <div class="btn" id="ptt-lock">🔒 Lock</div>  -->
          <div class="info-text" id="band-warning" style="display:none; background:#e74c3c;">
  ⚠️ OUT OF BAND - TX DISABLED ⚠️
          </div>
        </div>
        
        <div class="info-text">▼ Long press display to save VFO</div>
      </div>
      
      <div>
        <div class="knob-section">
          <div class="vslider-container">
            <div class="vslider-label slow">🐢</div>
            <div class="vslider-track">
              <input type="range" id="sensitivity-slider" class="vslider" min="2" max="10" value="6" step="1" orient="vertical">
            </div>
            <div class="vslider-label fast">🐇</div>
            <div class="vslider-value" id="sensitivity-value">6</div>
          </div>
          <div class="knob" id="knob"></div>
        </div>
        
        <div class="btn-group">
          <div class="btn" id="step-down">◀</div>
          <div class="btn btn-primary" id="step-cycle">Step</div>
          <div class="btn" id="step-up">▶</div>
        </div>
        
        <div class="btn-group">
          <div class="btn" id="freq-10">-10</div>
          <div class="btn" id="freq-1">-1</div>
          <div class="btn" id="freq+1">+1</div>
          <div class="btn" id="freq+10">+10</div>
        </div>
        
        <div class="btn-group">
          <div class="btn btn-rit" id="rit-toggle">RIT</div>
          <div class="btn" id="rit-down">-</div>
          <div class="btn" id="rit-up">+</div>
          <div class="btn" id="rit-reset">Rst</div>
        </div>
      </div>
    </div>
  </div>
  
  <!-- BFO TAB - Working version from v2-5 -->
  <div id="bfo-tab" class="tab-content">
    <div class="bfo-grid">
      <div class="clock-card">
        <div class="clock-title">🔧 BFO1 (CLK1) - Second IF Mixer</div>
        <div class="clock-freq" id="bfo1-display-main">33.000000 MHz</div>
        <div class="clock-info">USB: 57 MHz | LSB: 33 MHz</div>
        <div class="step-group">
          <div class="step-label">Step: <span id="bfo1-step-main" class="step-value">1KHz</span></div>
          <div class="btn-group">
            <div class="btn" onclick="bfo1StepDownMain()">◀ Slower</div>
            <div class="btn btn-primary" onclick="bfo1StepCycleMain()">Cycle</div>
            <div class="btn" onclick="bfo1StepUpMain()">Faster ▶</div>
          </div>
          <div class="btn-group">
            <div class="btn" onclick="adjustBFO1Main(-10)">-10</div>
            <div class="btn" onclick="adjustBFO1Main(-1)">-1</div>
            <div class="btn" onclick="adjustBFO1Main(1)">+1</div>
            <div class="btn" onclick="adjustBFO1Main(10)">+10</div>
          </div>
        </div>
        <div class="btn-group">
          <div class="btn" onclick="setBFO1toUSBMain()">USB (57MHz)</div>
          <div class="btn" onclick="setBFO1toLSBMain()">LSB (33MHz)</div>
          <div class="btn btn-memory" onclick="saveBFO1ToMemory()">💾 Save</div>
        </div>
        <div class="info-text">Tap Save to store</div>
      </div>
      
      <div class="clock-card">
        <div class="clock-title">🔧 BFO2 (CLK0) - Audio Demodulator</div>
        <div class="clock-freq" id="bfo2-display-main">12.000000 MHz</div>
        <div class="clock-info">Fixed 12 MHz Carrier</div>
        <div class="step-group">
          <div class="step-label">Step: <span id="bfo2-step-main" class="step-value">1KHz</span></div>
          <div class="btn-group">
            <div class="btn" onclick="bfo2StepDownMain()">◀ Slower</div>
            <div class="btn btn-primary" onclick="bfo2StepCycleMain()">Cycle</div>
            <div class="btn" onclick="bfo2StepUpMain()">Faster ▶</div>
          </div>
          <div class="btn-group">
            <div class="btn" onclick="adjustBFO2Main(-10)">-10</div>
            <div class="btn" onclick="adjustBFO2Main(-1)">-1</div>
            <div class="btn" onclick="adjustBFO2Main(1)">+1</div>
            <div class="btn" onclick="adjustBFO2Main(10)">+10</div>
          </div>
        </div>
        <div class="btn-group">
          <div class="btn" onclick="resetBFO2Main()">Reset to 12MHz</div>
          <div class="btn btn-memory" onclick="saveBFO2ToMemory()">💾 Save</div>
        </div>
        <div class="info-text">Tap Save to store</div>
      </div>
    </div>
  </div>
  
  <!-- MEMORY TAB -->
  <div id="memory-tab" class="tab-content">
    <div class="mem-panel">
      <div class="mem-header">📀 VFO Memories (10 slots)</div>
      <div class="mem-buttons" id="mem-buttons-tab"></div>
      <div class="memory-hint">Tap to recall | Long press VFO display to save</div>
    </div>
    
    <div class="mem-panel">
      <div class="mem-header">📀 BFO1 Memories (10 slots)</div>
      <div class="mem-buttons" id="bfo1-mem-buttons-tab"></div>
      <div class="memory-hint">Tap to recall | Tap Save button in BFO tab to store</div>
    </div>
    
    <div class="mem-panel">
      <div class="mem-header">📀 BFO2 Memories (10 slots)</div>
      <div class="mem-buttons" id="bfo2-mem-buttons-tab"></div>
      <div class="memory-hint">Tap to recall | Tap Save button in BFO tab to store</div>
    </div>
  </div>
  
  <div class="signature">uBITX VFO | RIT: RX only</div>
</div>

<script>
// ==================== CONSTANTS ====================
const IF_FREQ = 45000000;
const MIN_OP = 0;
const MAX_OP = 30000000;
const MAX_RIT = 5000;

const steps = [10, 100, 1000, 5000, 10000, 100000, 1000000, 10000000];
const stepLabels = ["10Hz", "100Hz", "1KHz", "5KHz", "10KHz", "100KHz", "1MHz", "10MHz"];

const bfoSteps = [10, 100, 1000, 5000, 10000, 100000, 1000000];
const bfoStepLabels = ["10Hz", "100Hz", "1KHz", "5KHz", "10KHz", "100KHz", "1MHz"];

// ==================== GLOBAL STATE ====================
let vfoFreq = 52000000;
let mode = "USB";
let stepIdx = 1;
let ritEnabled = false;
let ritOffset = 0;
let memories = [[],[],[]];
let memoryMode = false;

let BFO1_USB = 56995000;
let BFO1_LSB = 32995000;
let BFO2_DEFAULT = 11998000;

let bfo1Freq = 56995000;
let bfo2Freq = 11998000;
let bfo1StepIdx = 2;
let bfo2StepIdx = 2;
let bfo1Memories = new Array(10).fill(0);
let bfo2Memories = new Array(10).fill(0);

// Knob state
let isDragging = false;
let lastAngle = 0;
let currentRotation = 0;
let lastSendTime = 0;
let knobSensitivity = 6;

let pttActive = false;
let pttLocked = false;
let pttTimer = null;

// Initialize memory arrays
for(let i = 0; i < 3; i++) {
  for(let j = 0; j < 10; j++) {
    memories[i][j] = 0;
  }
}

// ==================== HELPER FUNCTIONS ====================
function getOperatingFreq() {
  return vfoFreq - IF_FREQ;
}

function setOperatingFreq(opFreq) {
  let newVFO = opFreq + IF_FREQ;
  if(newVFO >= 45000000 && newVFO <= 75000000) {
    vfoFreq = newVFO;
  }
}

function getRXFreq() {
  let opFreq = getOperatingFreq();
  if(ritEnabled) {
    return (mode === "LSB") ? opFreq - ritOffset : opFreq + ritOffset;
  }
  return opFreq;
}

function getDisplayFreq() {
  if(pttActive) {
    return getOperatingFreq();
  }
  return getRXFreq();
}

function formatFrequency() {
  let displayFreq = getDisplayFreq();
  if(displayFreq >= 1000000) {
    return (displayFreq/1000000).toFixed(6) + " MHz";
  } else {
    return (displayFreq/1000).toFixed(3) + " kHz";
  }
}

function formatBFOfreq(freq) {
  if (freq >= 1000000) {
    return (freq/1000000).toFixed(6) + " MHz";
  } else {
    return (freq/1000).toFixed(3) + " kHz";
  }
}

// Get band from operating frequency
function getBandInfo(freq) {
  if (freq >= 28000000) return { name: '10m', lpf: '35MHz', color: '#e74c3c' };
  else if (freq >= 21000000) return { name: '15m', lpf: '30MHz', color: '#e67e22' };
  else if (freq >= 14000000) return { name: '20m', lpf: '30MHz', color: '#e67e22' };
  else if (freq >= 7000000) return { name: '40m', lpf: '14MHz', color: '#2ecc71' };
  else if (freq >= 3500000) return { name: '80m', lpf: '7MHz', color: '#3498db' };
  else return { name: '160m', lpf: '3MHz', color: '#9b59b6' };
}

// Update band display
function updateBandDisplay() {
  let opFreq = getOperatingFreq();
  let band = getBandInfo(opFreq);
  let bandStatus = document.getElementById('band-status');
  if (bandStatus) {
    bandStatus.innerHTML = `📡 ${band.name} (${band.lpf})`;
    bandStatus.style.background = band.color;
  }
  
  // Highlight active band button
  let activeBand = null;
  if (opFreq >= 24000000) activeBand = 28000000;
  else if (opFreq >= 18000000) activeBand = 21000000;
  else if (opFreq >= 14000000) activeBand = 14000000;
  else if (opFreq >= 7000000) activeBand = 7000000;
  else if (opFreq >= 3500000) activeBand = 3500000;
  
  document.querySelectorAll('.band-btn').forEach(btn => {
    let bandFreq = parseInt(btn.getAttribute('data-band'));
    if (activeBand && bandFreq === activeBand) {
      btn.classList.add('active-band');
    } else {
      btn.classList.remove('active-band');
    }
  });
}
//===========Band Limit Chk ==================
// Check if frequency is within ham bands
function isWithinHamBand(freq) {
  const hamBands = [
    { min: 3500000, max: 4000000, name: '80m' },
    { min: 7000000, max: 7200000, name: '40m' },
    { min: 14000000, max: 14350000, name: '20m' },
    { min: 21000000, max: 21450000, name: '15m' },
    { min: 28000000, max: 29700000, name: '10m' }
  ];
  
  for (let band of hamBands) {
    if (freq >= band.min && freq <= band.max) {
      return true;
    }
  }
  return false;
}

// Update band warning display
function updateBandWarning() {
  let opFreq = getOperatingFreq();
  let inBand = isWithinHamBand(opFreq);
  let warningDiv = document.getElementById('band-warning');
  let pttBtn = document.getElementById('ptt-momentary');
  
  if (!inBand) {
    if (warningDiv) warningDiv.style.display = 'block';
    if (pttBtn) {
      pttBtn.style.opacity = '0.5';
      pttBtn.style.backgroundColor = '#555';
    }
  } else {
    if (warningDiv) warningDiv.style.display = 'none';
    if (pttBtn) {
      pttBtn.style.opacity = '1';
      pttBtn.style.backgroundColor = '#c0392b';
    }
  }
}


// ==================== UI FUNCTIONS ====================
function updateUI() {
  let fDisplay = document.getElementById('f-display');
  if(fDisplay) fDisplay.innerText = formatFrequency();
  
  let stepLabel = document.getElementById('step-label');
  if(stepLabel) stepLabel.innerText = "Step: " + stepLabels[stepIdx];
  
  // Update RIT button
  let ritBtn = document.getElementById('rit-toggle');
  if(ritBtn) {
    if(ritEnabled) {
      ritBtn.innerHTML = `RIT ${ritOffset > 0 ? '+' : ''}${(ritOffset/1000).toFixed(1)}K`;
      ritBtn.classList.add('active');
    } else {
      ritBtn.innerHTML = 'RIT';
      ritBtn.classList.remove('active');
    }
  }
  // Call this in updateUI()
  updateBandWarning();
  // Update band indicator with TX limit warning
  let opFreq = getOperatingFreq();
  let band = getBandInfo(opFreq);
  let bandStatus = document.getElementById('band-status');
  if (bandStatus) {
    bandStatus.innerHTML = `📡 ${band.name} (${band.lpf})`;
  // Check if frequency is within ham band
    if (isWithinHamBand) {
    bandStatus.style.background = band.color;
    } else {
    bandStatus.style.background = '#e74c3c';  // Red for out-of-band
    bandStatus.innerHTML = `⚠️ ${band.name} (${band.lpf})`;
    } 
  } 
  // Update mode buttons
  let modeLsb = document.getElementById('mode-lsb');
  let modeUsb = document.getElementById('mode-usb');
  if(modeLsb) modeLsb.classList.remove('active');
  if(modeUsb) modeUsb.classList.remove('active');
  if(mode === "LSB" && modeLsb) modeLsb.classList.add('active');
  else if(mode === "USB" && modeUsb) modeUsb.classList.add('active');
  
  let modeLabel = document.getElementById('mode-label');
  if(modeLabel) modeLabel.innerText = mode;
  
  let displayDiv = document.getElementById('display');
  if(displayDiv) {
    if(memoryMode) displayDiv.classList.add('memory-mode');
    else displayDiv.classList.remove('memory-mode');
  }
  
  updateBandDisplay();
  updateVFOMemoryButtons();
}

function updateVFOMemoryButtons() {
  let container = document.getElementById('mem-buttons-tab');
  if(container) {
    container.innerHTML = '';
    for(let i = 0; i < 10; i++) {
      let btn = document.createElement('div');
      btn.className = 'mem-btn';
      let memVal = memories[0][i];
      if(memVal && memVal > 0) {
        let opMem = memVal - IF_FREQ;
        btn.innerHTML = `M${i+1}<br>${(opMem/1000000).toFixed(2)}M`;
      } else {
        btn.innerHTML = `M${i+1}<br>Empty`;
      }
      btn.onclick = (function(slot) {
        return function() { handleVFOMemory(slot); };
      })(i);
      container.appendChild(btn);
    }
  }
}

function handleVFOMemory(slot) {
  if(memoryMode) {
    memories[0][slot] = vfoFreq;
    updateVFOMemoryButtons();
    exitMemoryMode();
    sendConfig();
    showMessage('VFO saved to memory ' + (slot + 1), '#27ae60');
  } else {
    let saved = memories[0][slot];
    if(saved && saved > 0) {
      vfoFreq = saved;
      updateUI();
      sendConfig();
      showMessage('VFO recalled from memory ' + (slot + 1), '#e67e22');
    } else {
      showMessage('Memory ' + (slot + 1) + ' is empty!', '#e74c3c');
    }
  }
}

function enterMemoryMode() {
  memoryMode = true;
  updateUI();
  switchTab('memory');
  showMessage('Tap any VFO memory button to save', '#f1c40f');
  setTimeout(() => { if(memoryMode) exitMemoryMode(); }, 5000);
}

function exitMemoryMode() {
  memoryMode = false;
  updateUI();
}

// ==================== VFO ACTIONS ====================
function setMode(newMode) {
  mode = newMode;
  if(mode === "USB") bfo1Freq = BFO1_USB;
  else if(mode === "LSB") bfo1Freq = BFO1_LSB;
  updateBFO1UI();
  sendSetupConfig();
  updateUI();
  sendConfig();
}

function toggleRIT() { 
  ritEnabled = !ritEnabled; 
  updateUI(); 
  sendRITConfig(); 
}

function adjustRIT(delta) { 
  ritOffset += delta; 
  if(ritOffset > MAX_RIT) ritOffset = MAX_RIT; 
  if(ritOffset < -MAX_RIT) ritOffset = -MAX_RIT; 
  updateUI(); 
  sendRITConfig(); 
}

function resetRIT() { 
  ritOffset = 0; 
  ritEnabled = false; 
  updateUI(); 
  sendRITConfig(); 
}

function adjustFrequency(delta) {
  let step = steps[stepIdx];
  let newOpFreq = getOperatingFreq() + (delta * step);
  if(newOpFreq >= 0 && newOpFreq <= MAX_OP) {
    setOperatingFreq(newOpFreq);
    updateUI();
    sendConfig();
  }
}

function changeStep(delta) {
  let newIdx = stepIdx + delta;
  if(newIdx >= 0 && newIdx < steps.length) {
    stepIdx = newIdx;
    updateUI();
    sendConfig();
  }
}

function cycleStep() {
  stepIdx = (stepIdx + 1) % steps.length;
  updateUI();
  sendConfig();
}

function setBand(bandFreq) {
  // Find band index from frequency
  let bandIndex = -1;
  const bandFreqs = [3500000, 7000000, 14000000, 21000000, 28000000];
  bandIndex = bandFreqs.indexOf(bandFreq);
  
  if (bandIndex >= 0) {
    fetch('/setBand?band=' + bandIndex)
      .then(() => {
        // Refresh config after band change
        loadConfig();
      })
      .catch(e => console.log('Band change error:', e));
  }
}
/*
function setBand(freq) {
  setOperatingFreq(freq);
  updateUI();
  sendConfig();
}*/

// ==================== BFO UI FUNCTIONS ====================
function updateBFO1UI() {
  let elem = document.getElementById('bfo1-display-main');
  if(elem) elem.innerHTML = formatBFOfreq(bfo1Freq);
  let stepElem = document.getElementById('bfo1-step-main');
  if(stepElem) stepElem.innerText = bfoStepLabels[bfo1StepIdx];
}

function updateBFO2UI() {
  let elem = document.getElementById('bfo2-display-main');
  if(elem) elem.innerHTML = formatBFOfreq(bfo2Freq);
  let stepElem = document.getElementById('bfo2-step-main');
  if(stepElem) stepElem.innerText = bfoStepLabels[bfo2StepIdx];
}

// BFO adjustment functions
function adjustBFO1Main(delta) {
  let step = bfoSteps[bfo1StepIdx];
  let newFreq = bfo1Freq + (delta * step);
  if(newFreq >= 1000000 && newFreq <= 60000000) {
    bfo1Freq = newFreq;
    updateBFO1UI();
    sendSetupConfig();
    showMessage(`BFO1: ${(bfo1Freq/1000000).toFixed(3)} MHz`, '#27ae60');
  }
}

function adjustBFO2Main(delta) {
  let step = bfoSteps[bfo2StepIdx];
  let newFreq = bfo2Freq + (delta * step);
  if(newFreq >= 1000000 && newFreq <= 60000000) {
    bfo2Freq = newFreq;
    updateBFO2UI();
    sendSetupConfig();
    showMessage(`BFO2: ${(bfo2Freq/1000000).toFixed(3)} MHz`, '#27ae60');
  }
}

// BFO Step control functions
function bfo1StepDownMain() {
  if(bfo1StepIdx > 0) {
    bfo1StepIdx--;
    updateBFO1UI();
    sendSetupConfig();
    showMessage(`BFO1 Step: ${bfoStepLabels[bfo1StepIdx]}`, '#f1c40f');
  }
}

function bfo1StepUpMain() {
  if(bfo1StepIdx < bfoSteps.length - 1) {
    bfo1StepIdx++;
    updateBFO1UI();
    sendSetupConfig();
    showMessage(`BFO1 Step: ${bfoStepLabels[bfo1StepIdx]}`, '#f1c40f');
  }
}

function bfo1StepCycleMain() {
  bfo1StepIdx = (bfo1StepIdx + 1) % bfoSteps.length;
  updateBFO1UI();
  sendSetupConfig();
  showMessage(`BFO1 Step: ${bfoStepLabels[bfo1StepIdx]}`, '#f1c40f');
}

function bfo2StepDownMain() {
  if(bfo2StepIdx > 0) {
    bfo2StepIdx--;
    updateBFO2UI();
    sendSetupConfig();
    showMessage(`BFO2 Step: ${bfoStepLabels[bfo2StepIdx]}`, '#f1c40f');
  }
}

function bfo2StepUpMain() {
  if(bfo2StepIdx < bfoSteps.length - 1) {
    bfo2StepIdx++;
    updateBFO2UI();
    sendSetupConfig();
    showMessage(`BFO2 Step: ${bfoStepLabels[bfo2StepIdx]}`, '#f1c40f');
  }
}

function bfo2StepCycleMain() {
  bfo2StepIdx = (bfo2StepIdx + 1) % bfoSteps.length;
  updateBFO2UI();
  sendSetupConfig();
  showMessage(`BFO2 Step: ${bfoStepLabels[bfo2StepIdx]}`, '#f1c40f');
}

function setBFO1toUSBMain() {
  bfo1Freq = BFO1_USB;
  updateBFO1UI();
  sendSetupConfig();
  showMessage('BFO1 set to USB (57MHz)', '#27ae60');
}

function setBFO1toLSBMain() {
  bfo1Freq = BFO1_LSB;
  updateBFO1UI();
  sendSetupConfig();
  showMessage('BFO1 set to LSB (33MHz)', '#27ae60');
}

function resetBFO2Main() {
  bfo2Freq = BFO2_DEFAULT;
  updateBFO2UI();
  sendSetupConfig();
  showMessage('BFO2 reset to 12MHz', '#27ae60');
}

// ==================== MEMORY FUNCTIONS ====================
function updateAllMemoryButtons() {
  // Update VFO memory buttons
  let container = document.getElementById('mem-buttons-tab');
  if(container) {
    container.innerHTML = '';
    for(let i = 0; i < 10; i++) {
      let btn = document.createElement('div');
      btn.className = 'mem-btn';
      let memVal = memories[0] ? memories[0][i] : 0;
      if(memVal && memVal > 0) {
        let opMem = memVal - IF_FREQ;
        btn.innerHTML = `M${i+1}<br>${(opMem/1000000).toFixed(2)}M`;
      } else {
        btn.innerHTML = `M${i+1}<br>Empty`;
      }
      btn.onclick = (function(slot) {
        return function() { handleVFOMemory(slot); };
      })(i);
      container.appendChild(btn);
    }
  }
  
  // BFO1 Memory Buttons
  let container1 = document.getElementById('bfo1-mem-buttons-tab');
  if(container1) {
    container1.innerHTML = '';
    for(let i = 0; i < 10; i++) {
      let btn = document.createElement('div');
      btn.className = 'mem-btn';
      // Use bfo1Memories array which should now be populated
      let memVal = bfo1Memories[i];
      if(memVal && memVal > 0) {
        btn.innerHTML = `M${i+1}<br>${(memVal/1000000).toFixed(3)}M`;
      } else {
        btn.innerHTML = `M${i+1}<br>Empty`;
      }
      btn.onclick = (function(slot) {
        return function() { handleBFO1Memory(slot); };
      })(i);
      container1.appendChild(btn);
    }
  }
  
  // BFO2 Memory Buttons
  let container2 = document.getElementById('bfo2-mem-buttons-tab');
  if(container2) {
    container2.innerHTML = '';
    for(let i = 0; i < 10; i++) {
      let btn = document.createElement('div');
      btn.className = 'mem-btn';
      let memVal = bfo2Memories[i];
      if(memVal && memVal > 0) {
        btn.innerHTML = `M${i+1}<br>${(memVal/1000000).toFixed(3)}M`;
      } else {
        btn.innerHTML = `M${i+1}<br>Empty`;
      }
      btn.onclick = (function(slot) {
        return function() { handleBFO2Memory(slot); };
      })(i);
      container2.appendChild(btn);
    }
  }
}
function saveBFO1ToMemory() {
  window.pendingSave = { type: 'bfo1', frequency: bfo1Freq };
  switchTab('memory');
  showMessage('Tap any BFO1 memory button to save', '#f1c40f');
  setTimeout(() => {
    if(window.pendingSave && window.pendingSave.type === 'bfo1') {
      window.pendingSave = null;
    }
  }, 10000);
}

function saveBFO2ToMemory() {
  window.pendingSave = { type: 'bfo2', frequency: bfo2Freq };
  switchTab('memory');
  showMessage('Tap any BFO2 memory button to save', '#f1c40f');
  setTimeout(() => {
    if(window.pendingSave && window.pendingSave.type === 'bfo2') {
      window.pendingSave = null;
    }
  }, 10000);
}

function handleBFO1Memory(slot) {
  if(window.pendingSave && window.pendingSave.type === 'bfo1') {
    bfo1Memories[slot] = window.pendingSave.frequency;
    updateAllMemoryButtons();
    sendSetupConfig();
    window.pendingSave = null;
    showMessage('BFO1 saved to memory ' + (slot + 1), '#27ae60');
  } else {
    let saved = bfo1Memories[slot];
    if(saved && saved > 0) {
      bfo1Freq = saved;
      updateBFO1UI();
      sendSetupConfig();
      showMessage('BFO1 recalled from memory ' + (slot + 1), '#e67e22');
    } else {
      showMessage('Memory ' + (slot + 1) + ' is empty!', '#e74c3c');
    }
  }
}

function handleBFO2Memory(slot) {
  if(window.pendingSave && window.pendingSave.type === 'bfo2') {
    bfo2Memories[slot] = window.pendingSave.frequency;
    updateAllMemoryButtons();
    sendSetupConfig();
    window.pendingSave = null;
    showMessage('BFO2 saved to memory ' + (slot + 1), '#27ae60');
  } else {
    let saved = bfo2Memories[slot];
    if(saved && saved > 0) {
      bfo2Freq = saved;
      updateBFO2UI();
      sendSetupConfig();
      showMessage('BFO2 recalled from memory ' + (slot + 1), '#e67e22');
    } else {
      showMessage('Memory ' + (slot + 1) + ' is empty!', '#e74c3c');
    }
  }
}

// ==================== PTT FUNCTIONS ====================
async function setPTT(active) {
  if (active === pttActive) return;
  try {
    await fetch('/ptt?action=' + (active ? 'on' : 'off'));
    pttActive = active;
    let pttBtn = document.getElementById('ptt-momentary');
    if(pttBtn) {
      if(active) {
        pttBtn.classList.add('btn-ptt-active');
        pttBtn.innerHTML = '🔴 TX';
      } else {
        pttBtn.classList.remove('btn-ptt-active');
        pttBtn.innerHTML = '🔴 PTT';
      }
    }
    updateUI();
  } catch(e) {}
}

function togglePTT() {
  if (pttLocked) {
    setPTT(!pttActive);
  } else if (!pttActive) {
    setPTT(true);
  }
}

function pttStart() {
  if (!pttLocked && !pttActive) setPTT(true);
}

function pttEnd() {
  if (!pttLocked && pttActive) setPTT(false);
}

function togglePTTLock() { 
  pttLocked = !pttLocked; 
  let btn = document.getElementById('ptt-lock');
  if(btn) {
    if(pttLocked) { 
      btn.classList.add('btn-ptt-locked'); 
      btn.innerHTML = '🔓 Unlock'; 
    } else { 
      btn.classList.remove('btn-ptt-locked'); 
      btn.innerHTML = '🔒 Lock'; 
    }
  }
  if (!pttLocked && pttActive) setPTT(false);
  showMessage(pttLocked ? 'Lock ON - Tap to toggle TX' : 'Lock OFF - Press & hold for TX', '#27ae60');
}

function updatePTTUI() {
  let pttBtn = document.getElementById('ptt-momentary');
  let bandBtns = document.querySelectorAll('.band-btn');
  if(pttBtn) {
    if(pttActive) { 
      pttBtn.classList.add('btn-ptt-active'); 
      pttBtn.innerHTML = '🔴 TX ACTIVE'; 
    } else { 
      pttBtn.classList.remove('btn-ptt-active'); 
      pttBtn.innerHTML = '🔴 PTT'; 
    }
  }
  bandBtns.forEach(btn => {
    if(pttActive) btn.classList.add('ptt-active');
    else btn.classList.remove('ptt-active');
  });
}

async function updatePTTStatus() {
  try {
    let response = await fetch('/getPTT');
    let status = await response.text();
    pttActive = (status === '1');
    updatePTTUI();
    updateUI();
  } catch(e) {}
}

// ==================== SERVER FUNCTIONS ====================
function sendConfig() {
  let now = Date.now();
  if(now - lastSendTime > 50) {
    fetch('/setConfig', { 
      method: 'POST', 
      headers: {'Content-Type': 'application/json'}, 
      body: JSON.stringify({ vfoFreq, mode, stepIdx, memories: memories }) 
    }).catch(e => console.log('Send error:', e));
    lastSendTime = now;
  }
}

function sendRITConfig() { 
  fetch('/setRIT', { 
    method: 'POST', 
    headers: {'Content-Type': 'application/json'}, 
    body: JSON.stringify({ ritEnabled, ritOffset }) 
  }).catch(e => console.log('RIT error:', e));
}

function sendSetupConfig() { 
  fetch('/setSetupConfig', { 
    method: 'POST', 
    headers: {'Content-Type': 'application/json'}, 
    body: JSON.stringify({ bfo1Freq, bfo2Freq, bfo1StepIdx, bfo2StepIdx, bfo1Memories, bfo2Memories }) 
  }).catch(e => console.log('Setup error:', e));
}



// ==================== TAB FUNCTIONS ====================
function switchTab(tabName) {
  document.querySelectorAll('.tab-content').forEach(tab => tab.classList.remove('active'));
  document.querySelectorAll('.tab-btn').forEach(btn => btn.classList.remove('active'));
  let activeTab = document.getElementById(tabName + '-tab');
  if(activeTab) activeTab.classList.add('active');
  let activeBtn = document.querySelector(`.tab-btn[data-tab="${tabName}"]`);
  if(activeBtn) activeBtn.classList.add('active');
  
  if(tabName === 'memory') {
    updateAllMemoryButtons();
  }
}
async function loadConfig() {
  try {
    let response = await fetch('/getConfig');
    let config = await response.json();
    vfoFreq = config.vfoFreq;
    mode = config.mode;
    stepIdx = config.stepIdx;
    
    // CRITICAL FIX: Load all memories from the response
    if (config.memories) {
      memories = config.memories;
      console.log('Loaded VFO memories:', memories[0]);
      console.log('Loaded BFO1 memories:', memories[1]);
      console.log('Loaded BFO2 memories:', memories[2]);
      
      // Also update bfoMemories arrays
      if (memories[1]) bfo1Memories = memories[1];
      if (memories[2]) bfo2Memories = memories[2];
    }
    
    if(config.bfo1Usb) BFO1_USB = config.bfo1Usb;
    if(config.bfo1Lsb) BFO1_LSB = config.bfo1Lsb;
    if(config.bfo2Default) BFO2_DEFAULT = config.bfo2Default;
    
    bfo1Freq = (mode === "USB") ? BFO1_USB : BFO1_LSB;
    bfo2Freq = BFO2_DEFAULT;
    
    updateUI();
    updateBFO1UI();
    updateBFO2UI();
    updateAllMemoryButtons();  // This will now show loaded memories
  } catch(e) { console.log('Load error:', e); }
  
  // Load RIT config
  try {
    let response = await fetch('/getRIT');
    let rit = await response.json();
    ritEnabled = rit.enabled;
    ritOffset = rit.offset;
    updateUI();
  } catch(e) {}
}
// ==================== KNOB FUNCTIONS ====================
function getAngle(x, y) {
  let r = document.getElementById('knob');
  if(!r) return 0;
  let rect = r.getBoundingClientRect();
  return Math.atan2(y - (rect.top + rect.height/2), x - (rect.left + rect.width/2)) * 180 / Math.PI;
}

function move(e) {
  if (!isDragging) return;
  e.preventDefault();
  
  let ev = e.touches ? e.touches[0] : e;
  let ang = getAngle(ev.clientX, ev.clientY);
  let d = ang - lastAngle;
  if (d > 180) d -= 360;
  if (d < -180) d += 360;
  currentRotation += d;
  
  let step = steps[stepIdx];
  let steps_change = Math.round(d / knobSensitivity);
  if (steps_change > 3) steps_change = 3;
  if (steps_change < -3) steps_change = -3;
  
  let delta = steps_change * step;
  let newOpFreq = getOperatingFreq() + delta;
  
  if(newOpFreq >= 0 && newOpFreq <= MAX_OP) {
    setOperatingFreq(newOpFreq);
    updateUI();
    sendConfig();
  }
  
  let knob = document.getElementById('knob');
  if(knob) knob.style.transform = 'rotate(' + currentRotation + 'deg)';
  lastAngle = ang;
}

function startDrag(e) {
  isDragging = true;
  let ev = e.touches ? e.touches[0] : e;
  lastAngle = getAngle(ev.clientX, ev.clientY);
  e.preventDefault();
}

function stopDrag() {
  isDragging = false;
}

// ==================== MESSAGE FUNCTIONS ====================
function showMessage(message, color) {
  let msg = document.createElement('div');
  msg.innerHTML = message;
  msg.style.position = 'fixed';
  msg.style.bottom = '20px';
  msg.style.left = '50%';
  msg.style.transform = 'translateX(-50%)';
  msg.style.background = color;
  msg.style.color = 'white';
  msg.style.padding = '8px 16px';
  msg.style.borderRadius = '20px';
  msg.style.fontSize = '12px';
  msg.style.fontWeight = 'bold';
  msg.style.zIndex = '1000';
  document.body.appendChild(msg);
  setTimeout(() => msg.remove(), 2000);
}

// ==================== S-METER ====================
async function updateSMeter() {
  try {
    let response = await fetch('/getS');
    let val = await response.text();
    let percent = parseInt(val);
    let activeSegs = Math.floor(percent / 5);
    if(activeSegs > 20) activeSegs = 20;
    let segs = document.querySelectorAll('.s-seg');
    for(let i = 0; i < segs.length; i++) {
      if(i < activeSegs) segs[i].classList.add('on');
      else segs[i].classList.remove('on');
    }
  } catch(e) {}
}

// ==================== EVENT LISTENERS ====================
function setupEventListeners() {
  // Mode buttons
  document.getElementById('mode-lsb').onclick = () => setMode("LSB");
  document.getElementById('mode-usb').onclick = () => setMode("USB");
  
  // Step buttons
  document.getElementById('step-down').onclick = () => changeStep(-1);
  document.getElementById('step-up').onclick = () => changeStep(1);
  document.getElementById('step-cycle').onclick = () => cycleStep();
  
  // Frequency buttons
  document.getElementById('freq-10').onclick = () => adjustFrequency(-10);
  document.getElementById('freq-1').onclick = () => adjustFrequency(-1);
  document.getElementById('freq+1').onclick = () => adjustFrequency(1);
  document.getElementById('freq+10').onclick = () => adjustFrequency(10);
  
  // RIT buttons
  document.getElementById('rit-toggle').onclick = () => toggleRIT();
  document.getElementById('rit-down').onclick = () => adjustRIT(-100);
  document.getElementById('rit-up').onclick = () => adjustRIT(100);
  document.getElementById('rit-reset').onclick = () => resetRIT();
  
  // Sensitivity slider
  let sensSlider = document.getElementById('sensitivity-slider');
  if(sensSlider) {
    sensSlider.oninput = function() {
      knobSensitivity = parseInt(this.value);
      document.getElementById('sensitivity-value').innerText = knobSensitivity;
    };
  }
  
  // Band buttons
  document.querySelectorAll('.band-btn').forEach(btn => {
    btn.onclick = () => setBand(parseInt(btn.getAttribute('data-band')));
  });
  
  // Tab buttons
  document.querySelectorAll('.tab-btn').forEach(btn => {
    btn.onclick = () => switchTab(btn.getAttribute('data-tab'));
  });
  
  // PTT buttons
// In setupEventListeners add:
let pttBtn = document.getElementById('ptt-momentary');
if(pttBtn) {
  pttBtn.onmousedown = () => setPTT(true);
  pttBtn.onmouseup = () => setPTT(false);
  pttBtn.ontouchstart = (e) => { e.preventDefault(); setPTT(true); };
  pttBtn.ontouchend = (e) => { e.preventDefault(); setPTT(false); };
}
  
  let pttMomentary = document.getElementById('ptt-momentary');
  let pttLock = document.getElementById('ptt-lock');
  
  if(pttMomentary) {
    pttMomentary.onclick = (e) => { e.stopPropagation(); togglePTT(); };
    pttMomentary.addEventListener('touchstart', (e) => { if (!pttLocked) { e.preventDefault(); pttStart(); } });
    pttMomentary.addEventListener('touchend', (e) => { if (!pttLocked) { e.preventDefault(); pttEnd(); } });
    pttMomentary.addEventListener('mousedown', () => { if (!pttLocked) pttStart(); });
    pttMomentary.addEventListener('mouseup', () => { if (!pttLocked) pttEnd(); });
    pttMomentary.addEventListener('mouseleave', () => { if (!pttLocked && pttActive) setPTT(false); });
  }
  if(pttLock) pttLock.onclick = () => togglePTTLock();
  
  // VFO long press for memory mode
  let display = document.getElementById('display');
  let pressTimer;
  if(display) {
    display.addEventListener('touchstart', () => { pressTimer = setTimeout(() => enterMemoryMode(), 500); });
    display.addEventListener('touchend', () => clearTimeout(pressTimer));
    display.addEventListener('mousedown', () => { pressTimer = setTimeout(() => enterMemoryMode(), 500); });
    display.addEventListener('mouseup', () => clearTimeout(pressTimer));
  }
  
  // Knob events
  let knob = document.getElementById('knob');
  if(knob) {
    knob.addEventListener('mousedown', startDrag);
    knob.addEventListener('touchstart', startDrag);
    window.addEventListener('mouseup', stopDrag);
    window.addEventListener('touchend', stopDrag);
    window.addEventListener('mousemove', move);
    window.addEventListener('touchmove', move);
  }
}

// ==================== INITIALIZATION ====================
let sGrid = document.getElementById('s-grid');
if(sGrid) {
  for(let i = 0; i < 20; i++) {
    let seg = document.createElement('div');
    seg.className = 's-seg';
    sGrid.appendChild(seg);
  }
}

setupEventListeners();
loadConfig();
setInterval(updateSMeter, 250);
setInterval(updatePTTStatus, 500);
</script>
</body>
</html>
)rawliteral";
