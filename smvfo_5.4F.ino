//=============================================================================
// uBITX VFO Controller - Professional Edition
// Version: 5.4F - Corrected VFO/BFO Relationship with Working Band Selection
//
// Changes since 5.3E:
//   - Fixed mode-change requests being silently dropped by the tuning
//     knob's network throttle. Mode button, band switch, memory recall,
//     VFO swap, and direct frequency entry now always reach the firmware
//     immediately instead of competing with knob traffic for the same
//     50ms window.
//   - Fixed BFO1 not being retuned to match VFO B's mode during split TX
//     when VFO A and VFO B are set to different sidebands.
//   - The on-screen BFO1 reading now refreshes immediately after any
//     automatic mode change (Mode button, band switch, memory recall,
//     direct frequency entry, VFO swap) instead of needing a page reload
//     to catch up with what the radio actually retuned to.
//=============================================================================
// ACCORDING TO uBITX SIGNAL PATH:
//
// First Mixer: VFO (CLK2) mixes with RF to produce 45 MHz IF
// Second Mixer: 45 MHz IF mixes with BFO1 (CLK1) to produce 12 MHz IF
// Third Mixer: 12 MHz IF mixes with BFO2 (CLK0) to produce Audio
//
// FORMULAS:
// For LSB: VFO = Operating_Freq + (BFO1 + BFO2)
// For USB: VFO = Operating_Freq + (BFO1 - BFO2)
//=============================================================================
// Credits: Please see the User Manual for Details.
// Original idea and implimentation by Mirko Pavleski
//==========================================================================

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
constexpr int TX_LPF_A = 4;  //33;
constexpr int TX_LPF_B = 13: //25;
constexpr int TX_LPF_C = 14; //26;

// Frequency Constants
constexpr unsigned long IF_FREQUENCY = 45000000;     // 45 MHz First IF
constexpr unsigned long BFO2_DEFAULT_FREQ = 11997700; // 12 MHz Audio demodulator
constexpr unsigned long BFO1_LSB_FREQ = 32997200;    //32995000;    // 33 MHz for LSB
constexpr unsigned long BFO1_USB_FREQ = 56995000;    // 57 MHz for USB

// Frequency Limits
constexpr unsigned long MIN_VFO_FREQ = 25000000;      // 25 MHz minimum VFO
constexpr unsigned long MAX_VFO_FREQ = 100000000;     // 100 MHz maximum VFO
constexpr unsigned long MIN_OPERATING_FREQ = 3500000;  //**dbg
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
//constexpr int MAGIC_NUMBER = 0xABCD;
constexpr uint32_t MAGIC_NUMBER   = 0xDEAD1234UL;
constexpr uint8_t  CONFIG_VERSION = 1;   // increment ONLY when EEPROM layout changes

// Address map (never change these without incrementing CONFIG_VERSION):
constexpr int ADDR_MAGIC      = 0;    // 4 bytes  uint32_t
constexpr int ADDR_VERSION    = 4;    // 1 byte   uint8_t
constexpr int ADDR_OP_FREQ    = 5;    // 4 bytes  unsigned long
constexpr int ADDR_BFO1       = 9;    // 4 bytes
constexpr int ADDR_BFO2       = 13;   // 4 bytes
constexpr int ADDR_MODE       = 17;   // 4 bytes  int
constexpr int ADDR_STEPS      = 21;   // 12 bytes (3 × int)
constexpr int ADDR_MEMORIES   = 33;   // 120 bytes (3 banks × 10 × 4)
constexpr int ADDR_BAND_MEM   = 153;  // 20 bytes (5 × 4)
constexpr int ADDR_RIT_EN     = 173;  // 1 byte   bool
constexpr int ADDR_RIT_OFF    = 174;  // 4 bytes  long
// Total used: 178 bytes of 1024
// A B VFO Mod adds
// Add these two addresses after ADDR_RIT_OFF in the address map:
constexpr int ADDR_VFOB_FREQ  = 178;   // 4 bytes  unsigned long
constexpr int ADDR_VFOB_MODE  = 182;   // 4 bytes  int
constexpr int ADDR_SPLIT      = 186;   // 1 byte   bool
// Total used: 187 bytes

//constexpr int RIT_ENABLED_ADDR = MAGIC_NUMBER_ADDR + 300;
//constexpr int RIT_OFFSET_ADDR = MAGIC_NUMBER_ADDR + 304;
//constexpr int BAND_MEMORY_ADDR = MAGIC_NUMBER_ADDR + 200;

// Timing Constants
constexpr unsigned long SAVE_DELAY_MS = 20000;  // was 20000 20s
constexpr unsigned long SERIAL_BAUD = 115200;

//=============================================================================
// DATA STRUCTURES
//=============================================================================

struct BandLimit {
  const char* name;
  unsigned long minFreq;
  unsigned long maxFreq;
};

struct WiFiConfig {
  const char* ssid;
  const char* password;
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
unsigned long operatingFreq = 7000000;      // Displayed frequency (0-30MHz)
unsigned long vfoFreq = 0;                  // Calculated VFO for CLK2
unsigned long bfo1Freq = BFO1_LSB_FREQ;     // CLK1 - Second IF mixer (33/57 MHz)
unsigned long bfo2Freq = BFO2_DEFAULT_FREQ; // CLK0 - Audio demodulator (12 MHz)

//AB MODE ADD ON  VFO A/B State
unsigned long vfoBFreq       = 7000000;   // VFO B operating frequency
bool          vfoBActive      = false;     // false = VFO A selected, true = VFO B
bool          splitMode       = false;     // true = RX on A, TX on B

// Mode State
enum class Mode { LSB, USB,};  // AM, CW };
Mode currentMode = Mode::LSB;
Mode vfoBMode = Mode::LSB; // VFO B mode

// Step Management (0=Operating Freq, 1=BFO1, 2=BFO2)
int stepIndices[3] = {1, 2, 2};

// RIT State
long ritOffset = 0;
bool ritEnabled = false;

// Memory System (0=Operating Freq, 1=BFO1, 2=BFO2)
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
extern const char MAIN_HTML[] PROGMEM;

void initializeSi5351();
void updateAllClocks();
void saveToEEPROM();
void loadFromEEPROM();
void scheduleSave();
unsigned long calculateVFOfreq();
unsigned long getOperatingFrequency();
void setOperatingFrequency(unsigned long opFreq);
int getCurrentBandIndex(unsigned long freq);
void setBand(int bandIndex);
void updateBFO1ByMode();
void setPTT(bool active);
void setTXFilters(unsigned long freq);
bool isWithinHamBand(unsigned long freq);
const char* getBandName(unsigned long freq);
void setupWebServer();
void handleGetConfig();
void handleSetConfig();
void handleGetSetupConfig();
void handleSetSetupConfig();
void handleBandChange();
void handlePTT();
void handleGetRIT();
void handleSetRIT();
void handleGetSMeter();
void parseVFOConfig(const String& json);
void parseBFOConfig(const String& json);
unsigned long extractJsonValue(const String& json, const String& key);//**dbg
void parseMemoriesFromJson(const String& json);
void parseBFO1MemoriesFromJson(const String& json);
void parseBFO2MemoriesFromJson(const String& json);
void parseArrayFromJson(const String& json, const String& key, unsigned long* target, int maxSize);
void parseCommaSeparatedValues(const String& str, unsigned long* target, int maxCount);
void flashErrorLED(int times);
void setRXFilters();
void setLPF(bool a, bool b, bool c);
//ABMode
void handleGetVFOB();
void handleSetVFOB();
void handleSetSplit();
void handleSwapVFO();
void handleCopyAtoB();
//=============================================================================
// CORRECT VFO CALCULATION BASED ON uBITX SIGNAL PATH
//=============================================================================
unsigned long calculateVFOfreq() {
  Mode          txMode;
  // On TX in split mode, use VFO B frequency and mode
  unsigned long txOpFreq = (splitMode && pttActive) ? vfoBFreq     : operatingFreq;
  txMode   = (splitMode && pttActive) ? vfoBMode      : currentMode;
  unsigned long result;

  if (txMode == Mode::LSB) {
    result = txOpFreq + (long)bfo1Freq + (long)bfo2Freq;
  } else {
    result = (long)txOpFreq + ((long)bfo1Freq - (long)bfo2Freq);
  }

  unsigned long txFreq = result;

  // Apply RIT only on RX (never on TX, never in split TX)
  if (ritEnabled && !pttActive) {
    result = (long)result + ritOffset;
  }

  Serial.printf("[%s] OpFreq:%lu BFO1:%lu BFO2:%lu | TX:%lu RIT:%s CLK2:%lu\n",
                (splitMode && pttActive) ? "SPLIT-TX" : (pttActive ? "TX" : "RX"),
                txOpFreq, bfo1Freq, bfo2Freq, txFreq,
                (ritEnabled && !pttActive) ? "ON" : "OFF", result);

  return constrain(result, MIN_VFO_FREQ, MAX_VFO_FREQ);
}

//=============================================================================
// SI5351 MANAGEMENT
//=============================================================================

void updateAllClocks() {
  vfoFreq = calculateVFOfreq();

  si5351.set_freq(vfoFreq * 100ULL, SI5351_CLK2);
  si5351.set_freq(bfo1Freq * 100ULL, SI5351_CLK1);
  si5351.set_freq(bfo2Freq * 100ULL, SI5351_CLK0);

  si5351.output_enable(SI5351_CLK2, 1);
  si5351.output_enable(SI5351_CLK1, 1);
  si5351.output_enable(SI5351_CLK0, 1);

  /*  Serial.println("\n========== CLOCK UPDATE ==========");
    Serial.printf("Mode: %s\n", currentMode == Mode::USB ? "USB" : "LSB");
    Serial.printf("Operating Freq: %lu Hz (%.3f MHz)\n", operatingFreq, operatingFreq / 1000000.0);
    Serial.printf("VFO (CLK2): %lu Hz (%.3f MHz)\n", vfoFreq, vfoFreq / 1000000.0);
    Serial.printf("BFO1 (CLK1): %lu Hz (%.3f MHz)\n", bfo1Freq, bfo1Freq / 1000000.0);
    Serial.printf("BFO2 (CLK0): %lu Hz (%.3f MHz)\n", bfo2Freq, bfo2Freq / 1000000.0);
    Serial.println("=====================================\n");*/
}

void initializeSi5351() {
  if (!si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0)) {
    Serial.println("Si5351 initialization failed!");
    while (true) {
      delay(1000);
    }
  }

  constexpr int CRYSTAL_CORRECTION = 162600;
  si5351.set_correction(CRYSTAL_CORRECTION, SI5351_PLL_INPUT_XO);

  si5351.output_enable(SI5351_CLK0, 0);
  si5351.output_enable(SI5351_CLK1, 0);
  si5351.output_enable(SI5351_CLK2, 0);

  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_8MA);
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_8MA);

  updateAllClocks();

  Serial.printf("Crystal correction: %d\n", CRYSTAL_CORRECTION);
}

//=============================================================================
// FREQUENCY MANAGEMENT
//=============================================================================
unsigned long getOperatingFrequency() {
  return operatingFreq;
}

void setOperatingFrequency(unsigned long opFreq) {
  // Save current band frequency before changing
  int currentBand = getCurrentBandIndex(operatingFreq);

  if (currentBand >= 0 && currentBand < BAND_COUNT) {
    bandLastFreq[currentBand] = operatingFreq;
  }

  operatingFreq = constrain(opFreq, MIN_OPERATING_FREQ, MAX_OPERATING_FREQ);
  updateAllClocks();
  scheduleSave();

  Serial.printf("Operating frequency set to: %lu Hz\n", operatingFreq);
}

void setBFO1Frequency(unsigned long newFreq) {
  bfo1Freq = constrain(newFreq, 1000000UL, 60000000UL);
  updateAllClocks();
  scheduleSave();

  Serial.printf("BFO1 set to: %lu Hz\n", bfo1Freq);
}

void setBFO2Frequency(unsigned long newFreq) {
  bfo2Freq = constrain(newFreq, 1000000UL, 60000000UL);
  updateAllClocks();
  scheduleSave();

  Serial.printf("BFO2 set to: %lu Hz\n", bfo2Freq);
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

  Serial.printf("Switched to %s band at %lu Hz\n", BAND_LIMITS[bandIndex].name, newFreq);
}

void updateBFO1ByMode() {
  if (currentMode == Mode::USB) {
    setBFO1Frequency(BFO1_USB_FREQ);
  } else {
    setBFO1Frequency(BFO1_LSB_FREQ);
  }
}

//=============================================================================
// PTT & FILTER MANAGEMENT
//=============================================================================
void setPTT(bool active) {
  if (active == pttActive) return;

  // Determine actual TX frequency and mode:
  // In split mode TX uses VFO B's frequency and mode, otherwise always VFO A
  unsigned long txFreq = (splitMode && active) ? vfoBFreq : operatingFreq;
  Mode          txMode = (splitMode && active) ? vfoBMode : currentMode;

  // Safety: check whichever frequency will actually be transmitted
  if (active && !isWithinHamBand(txFreq)) {
    Serial.printf("TX BLOCKED: %lu Hz outside ham bands!\n", txFreq);
    flashErrorLED(3);
    return;
  }

  // Also check VFO B safety if split is enabled
  if (active && splitMode && !isWithinHamBand(vfoBFreq)) {
    Serial.printf("TX BLOCKED: VFO B %lu Hz outside ham bands!\n", vfoBFreq);
    flashErrorLED(3);
    return;
  }

  pttActive = active;
  digitalWrite(PTT_PIN, active ? HIGH : LOW);
  digitalWrite(PTT_LED_PIN, active ? HIGH : LOW);

  // BFO1 (CLK1) is a single physical register that calculateVFOfreq() assumes
  // matches whichever mode it's calculating for. Normally that's safe because
  // updateBFO1ByMode() keeps it in sync with currentMode. But in split TX,
  // calculateVFOfreq() switches its formula to vfoBMode (see line ~222) — if
  // VFO B's mode differs from VFO A's, BFO1 must be retuned to match the TX
  // mode here, otherwise the injection frequency and the math disagree and
  // the transmitted sideband comes out wrong.
  static unsigned long bfo1FreqRXSaved  = 0;
  static bool          bfo1TempSwitched = false;

  if (active) {
    if (splitMode && txMode != currentMode) {
      bfo1FreqRXSaved  = bfo1Freq;
      bfo1TempSwitched = true;
      bfo1Freq = (txMode == Mode::USB) ? BFO1_USB_FREQ : BFO1_LSB_FREQ;
      Serial.printf("Split TX mode (%s) differs from VFO-A (%s) — retuning BFO1 to %lu Hz for TX\n",
                    txMode == Mode::USB ? "USB" : "LSB",
                    currentMode == Mode::USB ? "USB" : "LSB",
                    bfo1Freq);
    }
    setTXFilters(txFreq);      // LPF based on actual TX freq
    Serial.printf("TX ON  | VFO-A:%lu Hz | VFO-B:%lu Hz | Split:%s | TXon:%lu Hz (%s)\n",
                  operatingFreq, vfoBFreq,
                  splitMode ? "YES" : "NO",
                  txFreq, getBandName(txFreq));
    updateAllClocks();         // CLK2 set here — calculateVFOfreq() prints actual value
  } else {
    if (bfo1TempSwitched) {
      bfo1Freq = bfo1FreqRXSaved;
      bfo1TempSwitched = false;
      Serial.printf("Restoring RX BFO1 to %lu Hz\n", bfo1Freq);
    }
    setRXFilters();
    Serial.printf("TX OFF | RIT:%s offset:%ld Hz\n",
                  ritEnabled ? "ON" : "OFF", ritOffset);
    updateAllClocks();         // CLK2 back to RX freq
  }
}

void flashErrorLED(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(PTT_LED_PIN, HIGH);
    delay(100);
    digitalWrite(PTT_LED_PIN, LOW);
    delay(100);
  }
}

void setTXFilters(unsigned long freq) {
  if (freq >= 21000000) {
    setLPF(0, 0, 0);
  } else if (freq >= 14000000) {
    setLPF(1, 0, 0);
  } else if (freq >= 7000000) {
    setLPF(1, 1, 0);
  } else {
    setLPF(1, 1, 1);
  }
}

void setRXFilters() {
  digitalWrite(TX_LPF_A, LOW);
  digitalWrite(TX_LPF_B, LOW);
  digitalWrite(TX_LPF_C, LOW);
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
// SETUP & INITIALIZATION
//=============================================================================

void setup() {
  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  EEPROM.begin(EEPROM_SIZE);

  // Initialize hardware pins
  pinMode(PTT_PIN, OUTPUT);
  pinMode(PTT_LED_PIN, OUTPUT);
  digitalWrite(PTT_PIN, LOW);
  digitalWrite(PTT_LED_PIN, LOW);

  const int lpfPins[] = {TX_LPF_A, TX_LPF_B, TX_LPF_C};
  for (int pin : lpfPins) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  pinMode(S_METER_PIN, INPUT);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pinMode(WIFI_SWITCH_PIN, INPUT_PULLUP);

  loadFromEEPROM();

  // Initialize band memories
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
  WiFiConfig config;
  // WiFi initialization
  if (digitalRead(WIFI_SWITCH_PIN)) {
    config = {"ubitx", "12345678", true};
    WiFi.mode(WIFI_AP);
    WiFi.softAP(config.ssid, config.password);
    Serial.println("\nAP MODE ACTIVE");
    Serial.printf("SSID: %s\nPassword: %s\nWeb: http://%s\n",
                  config.ssid, config.password, WiFi.softAPIP().toString().c_str());
  } else {
    config = {"YourSSID", "Password", false}; // Enter your Wifi SSID and Password for Station Mode  
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
      Serial.println("\nSTA MODE ACTIVE");
      Serial.printf("IP: http://%s\n", WiFi.localIP().toString().c_str());
    } else {
      Serial.println("\nFailed! Falling back to AP mode");
      WiFi.mode(WIFI_AP);
      WiFi.softAP("ubitx", "12345678");
    }
  }

  if (!MDNS.begin("ubitx")) {
    Serial.println("MDNS failed!");
  } else {
    MDNS.addService("_http", "_tcp", 80);
    Serial.println("mDNS: http://ubitx.local");
  }

  initializeSi5351();
  setupWebServer();

  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║     uBITX VFO Controller v2.1        ║");
  Serial.println("║   Corrected VFO/BFO Relationship     ║");
  Serial.println("╚════════════════════════════════════════╝\n");
}

//=============================================================================
// WEB SERVER
//=============================================================================

void setupWebServer() {
  server.on("/", []() {
    server.send(200, "text/html", MAIN_HTML);
  });
  server.on("/getConfig", handleGetConfig);
  server.on("/setConfig", HTTP_POST, handleSetConfig);
  server.on("/getSetupConfig", handleGetSetupConfig);
  server.on("/setSetupConfig", HTTP_POST, handleSetSetupConfig);
  server.on("/setBand", handleBandChange);
  server.on("/ptt", handlePTT);
  server.on("/getPTT", []() {
    server.send(200, "text/plain", pttActive ? "1" : "0");
  });
  server.on("/getRIT", handleGetRIT);
  server.on("/setRIT", HTTP_POST, handleSetRIT);
  server.on("/getS", handleGetSMeter);
  server.on("/getTxLimit", []() {
    server.send(200, "text/plain", txLimitEnabled ? "1" : "0");
  });
  server.on("/setVFOB",    HTTP_POST, handleSetVFOB);
  server.on("/getVFOB",    HTTP_GET,  handleGetVFOB);
  server.on("/setSplit",   HTTP_POST, handleSetSplit);
  server.on("/swapVFO",    HTTP_POST, handleSwapVFO);
  server.on("/copyAtoB",   HTTP_POST, handleCopyAtoB);
  server.begin();
}

void handleGetConfig() {
  String json = "{";
  json += "\"operatingFreq\":" + String(operatingFreq) + ",";
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

  // ADD THIS:
  bool immediateSave = (body.indexOf("\"immediateSave\":true") != -1);
  if (immediateSave) {
    saveToEEPROM();
  } else {
    scheduleSave();
  }

  //  scheduleSave();
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

  // CRITICAL FIX: Also ensure operating freq memories are preserved
  // The memories array should already contain all three banks

  // IMPORTANT: Update the clocks immediately so VFO recalculates
  updateAllClocks();

  // Check if this is an immediate save request (from memory operations)
  bool immediateSave = (body.indexOf("\"immediateSave\":true") != -1);

  // hardwareOnly: just update clocks, no EEPROM activity at all
  bool hardwareOnly = (body.indexOf("\"hardwareOnly\":true") != -1);
  if (hardwareOnly) {
    server.send(200, "text/plain", "OK");
    return;
  }

  if (immediateSave) {
    saveToEEPROM();
    Serial.println("EEPROM saved (BFO memory operation)");
  } else {
    scheduleSave();
    // No serial print here — was flooding terminal on every knob step
  }

  server.send(200, "text/plain", "OK");
}

void handleBandChange() {
  if (server.hasArg("band")) {
    int bandIndex = server.arg("band").toInt();

    if (bandIndex >= 0 && bandIndex < BAND_COUNT) {
      setBand(bandIndex);
      server.send(200, "text/plain", "OK");
    } else {
      server.send(400, "text/plain", "Invalid band");
    }
  } else {
    server.send(400, "text/plain", "Missing band parameter");
  }
}

void handlePTT() {
  if (server.hasArg("action")) {
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
  } else {
    server.send(400, "text/plain", "Missing action parameter");
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
    if (offsetEnd == -1) offsetEnd = body.length();
    ritOffset = body.substring(offsetStart, offsetEnd).toInt();
    ritOffset = constrain(ritOffset, -MAX_RIT_OFFSET, MAX_RIT_OFFSET);
  }
  updateAllClocks();
  scheduleSave();
  server.send(200, "text/plain", "OK");
}

void handleGetVFOB() {
  String json = "{";
  json += "\"vfoBFreq\":"   + String(vfoBFreq) + ",";
  json += "\"vfoBMode\":\""  + String(vfoBMode == Mode::USB ? "USB" : "LSB") + "\",";
  json += "\"vfoBActive\":"  + String(vfoBActive  ? "true" : "false") + ",";
  json += "\"splitMode\":"   + String(splitMode   ? "true" : "false");
  json += "}";
  server.send(200, "application/json", json);
}

void handleSetVFOB() {
  if (!server.hasArg("plain")) {
    server.send(400, "text/plain", "No data");
    return;
  }
  String body = server.arg("plain");

  unsigned long newFreq = extractJsonValue(body, "vfoBFreq");
  if (newFreq >= MIN_OPERATING_FREQ && newFreq <= MAX_OPERATING_FREQ)
    vfoBFreq = newFreq;

  if (body.indexOf("\"vfoBMode\":\"USB\"") != -1) vfoBMode = Mode::USB;
  else if (body.indexOf("\"vfoBMode\":\"LSB\"") != -1) vfoBMode = Mode::LSB;

  if (body.indexOf("\"vfoBActive\":true") != -1)  vfoBActive = true;
  else if (body.indexOf("\"vfoBActive\":false") != -1) vfoBActive = false;

  // If VFO B is now selected, swap which one drives the display
  // (hardware clocks don't change — VFO selection only affects what
  //  the UI tunes and what calculateVFOfreq uses for non-split RX)
  updateAllClocks();
  scheduleSave();
  server.send(200, "text/plain", "OK");
}

void handleSetSplit() {
  if (!server.hasArg("plain")) {
    server.send(400, "text/plain", "No data");
    return;
  }
  String body = server.arg("plain");
  splitMode = (body.indexOf("\"split\":true") != -1);
  Serial.printf("Split mode: %s\n", splitMode ? "ON" : "OFF");
  updateAllClocks();
  server.send(200, "text/plain", "OK");
}

void handleSwapVFO() {
  // Swap A and B completely
  unsigned long tmpFreq = operatingFreq; operatingFreq = vfoBFreq; vfoBFreq = tmpFreq;
  Mode tmpMode = currentMode;            currentMode   = vfoBMode;  vfoBMode  = tmpMode;
  vfoBActive = !vfoBActive;
  updateBFO1ByMode();
  updateAllClocks();
  scheduleSave();
  server.send(200, "text/plain", "OK");
}

void handleCopyAtoB() {
  vfoBFreq = operatingFreq;
  vfoBMode = currentMode;
  Serial.printf("VFO B set to A: %lu Hz %s\n", vfoBFreq,
                vfoBMode == Mode::USB ? "USB" : "LSB");
  server.send(200, "text/plain", "OK");
}
void handleGetSMeter() {
  int val = analogRead(S_METER_PIN);
  int percent = constrain(map(val, 0, 4095, 0, 100), 0, 100);
  server.send(200, "text/plain", String(percent));
}

//=============================================================================
// JSON PARSING HELPERS
//=============================================================================

void parseVFOConfig(const String& json) {
  unsigned long newOpFreq = extractJsonValue(json, "operatingFreq");
  if (newOpFreq >= MIN_OPERATING_FREQ && newOpFreq <= MAX_OPERATING_FREQ) {
    operatingFreq = newOpFreq;
  }

  int modeStart = json.indexOf("\"mode\":\"");
  if (modeStart != -1) {
    modeStart += 8;
    int modeEnd = json.indexOf("\"", modeStart);
    if (modeEnd != -1) {
      String modeStr = json.substring(modeStart, modeEnd);
      Mode newMode = (modeStr == "USB") ? Mode::USB : Mode::LSB;
      if (currentMode != newMode) {
        currentMode = newMode;
        updateBFO1ByMode();
      }
    }
  }

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

void parseMemoriesFromJson(const String& json) {
  int searchFrom = json.indexOf("\"memories\":[[");
  if (searchFrom == -1) return;
  searchFrom += 12; // skip past "memories":[[  to start of first bank

  for (int bank = 0; bank < 3 && searchFrom < json.length(); bank++) {
    int bankStart = json.indexOf("[", searchFrom);
    if (bankStart == -1) break;
    int bankEnd = json.indexOf("]", bankStart + 1);
    if (bankEnd == -1) break;
    String bankStr = json.substring(bankStart + 1, bankEnd);
    parseCommaSeparatedValues(bankStr, memorySlots[bank], 10);
    searchFrom = bankEnd + 1;
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

unsigned long extractJsonValue(const String& json, const String& key) {  //*dbg unsigned long
  String searchKey = "\"" + key + "\":";
  int startPos = json.indexOf(searchKey);
  if (startPos == -1) return 0;

  startPos += searchKey.length();
  // Skip whitespace
  while (startPos < json.length() && json[startPos] == ' ') startPos++;

  int endPos = startPos;
  // Read sign + digits
  if (json[endPos] == '-') endPos++;
  while (endPos < json.length() && isDigit(json[endPos])) endPos++;

  if (endPos == startPos) return 0;
  return (unsigned long)json.substring(startPos, endPos).toInt();
}
//=============================================================================
// EEPROM MANAGEMENT
//=============================================================================
void saveToEEPROM() {
  EEPROM.put(ADDR_MAGIC,    (uint32_t)MAGIC_NUMBER);
  EEPROM.put(ADDR_VERSION,  (uint8_t)CONFIG_VERSION);
  EEPROM.put(ADDR_OP_FREQ,  operatingFreq);
  EEPROM.put(ADDR_BFO1,     bfo1Freq);
  EEPROM.put(ADDR_BFO2,     bfo2Freq);
  EEPROM.put(ADDR_MODE,     (int)currentMode);
  for (int i = 0; i < 3; i++)
    EEPROM.put(ADDR_STEPS + i * 4, stepIndices[i]);
  for (int b = 0; b < 3; b++)
    for (int s = 0; s < 10; s++)
      EEPROM.put(ADDR_MEMORIES + (b * 40) + (s * 4), memorySlots[b][s]);
  for (int i = 0; i < BAND_COUNT; i++)
    EEPROM.put(ADDR_BAND_MEM + i * 4, bandLastFreq[i]);
  EEPROM.put(ADDR_RIT_EN,  ritEnabled);
  EEPROM.put(ADDR_RIT_OFF, ritOffset);
  EEPROM.put(ADDR_VFOB_FREQ, vfoBFreq);
  EEPROM.put(ADDR_VFOB_MODE, (int)vfoBMode);
  EEPROM.put(ADDR_SPLIT,     splitMode);
  if (EEPROM.commit())
    Serial.println("EEPROM saved OK");
  else
    Serial.println("EEPROM commit FAILED");
}


void loadFromEEPROM() {
  uint32_t magic;  uint8_t version;
  EEPROM.get(ADDR_MAGIC,   magic);
  EEPROM.get(ADDR_VERSION, version);

  if (magic != MAGIC_NUMBER || version != CONFIG_VERSION) {
    Serial.printf("EEPROM invalid (magic=0x%08X ver=%d) — defaults loaded\n", magic, version);
    // Set defaults
    operatingFreq = 7000000;
    bfo1Freq      = BFO1_LSB_FREQ;
    bfo2Freq      = BFO2_DEFAULT_FREQ;
    currentMode   = Mode::LSB;
    stepIndices[0] = 1; stepIndices[1] = 2; stepIndices[2] = 2;
    ritEnabled = false; ritOffset = 0;
    for (int i = 0; i < BAND_COUNT; i++) bandLastFreq[i] = BAND_DEFAULT_FREQS[i];
    for (int b = 0; b < 3; b++)
      for (int s = 0; s < 10; s++) memorySlots[b][s] = 0;
    saveToEEPROM();   // write defaults + valid magic so next boot loads cleanly
    return;
  }

  EEPROM.get(ADDR_OP_FREQ, operatingFreq);
  EEPROM.get(ADDR_BFO1,    bfo1Freq);
  EEPROM.get(ADDR_BFO2,    bfo2Freq);
  int mode; EEPROM.get(ADDR_MODE, mode);
  currentMode = static_cast<Mode>(mode);
  for (int i = 0; i < 3; i++) EEPROM.get(ADDR_STEPS + i * 4, stepIndices[i]);
  for (int b = 0; b < 3; b++)
    for (int s = 0; s < 10; s++)
      EEPROM.get(ADDR_MEMORIES + (b * 40) + (s * 4), memorySlots[b][s]);
  for (int i = 0; i < BAND_COUNT; i++) EEPROM.get(ADDR_BAND_MEM + i * 4, bandLastFreq[i]);
  EEPROM.get(ADDR_RIT_EN,  ritEnabled);
  EEPROM.get(ADDR_RIT_OFF, ritOffset);
  //AB VFO Mode
  EEPROM.get(ADDR_VFOB_FREQ, vfoBFreq);
  int vfoBModeInt; EEPROM.get(ADDR_VFOB_MODE, vfoBModeInt);
  vfoBMode = static_cast<Mode>(vfoBModeInt);
  EEPROM.get(ADDR_SPLIT, splitMode);
  vfoBFreq = constrain(vfoBFreq, MIN_OPERATING_FREQ, MAX_OPERATING_FREQ);

  // Validate
  operatingFreq = constrain(operatingFreq, MIN_OPERATING_FREQ, MAX_OPERATING_FREQ);
  bfo1Freq      = constrain(bfo1Freq,  1000000UL, 60000000UL);
  bfo2Freq      = constrain(bfo2Freq,  1000000UL, 60000000UL);
  ritOffset     = constrain(ritOffset, -MAX_RIT_OFFSET, MAX_RIT_OFFSET);
  for (int i = 0; i < 3; i++)
    if (stepIndices[i] < 0 || stepIndices[i] >= STEP_COUNT) stepIndices[i] = 1;
  for (int b = 0; b < 3; b++)
    for (int s = 0; s < 10; s++)
      if (memorySlots[b][s] > MAX_OPERATING_FREQ) memorySlots[b][s] = 0;

  Serial.println("EEPROM loaded OK");
}


void scheduleSave() {
  lastChangeTime = millis();
  saveNeeded = true;
}

/*void forceSaveToEEPROM() {
  Serial.println("FORCE SAVING to EEPROM...");
  saveToEEPROM();
  saveNeeded = false;
  lastChangeTime = millis();
  }*/

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
// MAIN HTML - COMPLETELY CORRECTED VERSION
//=============================================================================
const char MAIN_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1.0,maximum-scale=1.0,user-scalable=no,viewport-fit=cover">
<meta charset="UTF-8">
<title>uBITX VFO</title>
<style>
* { -webkit-tap-highlight-color:transparent; box-sizing:border-box; margin:0; padding:0; }
/* touch-action:manipulation on buttons only — fast tap, no 300ms delay */
.btn, .btn-ptt, .band-btn, .mode-btn, .vstrip-btn, .bc-btn, .mem-btn, .tab-btn,
.numpad-btn, .freq-go, .freq-cancel { touch-action:manipulation; }
/* touch-action:none on knob — JS owns all touch events, browser must not intercept */
.knob { touch-action:none; }

body { background:#0a0a0a; font-family:'Arial Black',Arial,sans-serif; color:white; padding:4px; }

.container { max-width:900px; margin:0 auto; background:#2c3e50; border-radius:12px; padding:7px; box-shadow:0 4px 16px rgba(0,0,0,0.6); }

/* ── TABS ── */
.tab-container { display:flex; gap:4px; margin-bottom:7px; background:#1a1a2e; border-radius:9px; padding:3px; }
.tab-btn { flex:1; padding:7px 2px; background:#34495e; border:none; border-radius:7px; color:#ecf0f1; font-weight:bold; cursor:pointer; font-size:11px; text-align:center; }
.tab-btn.active { background:#e74c3c; }
.tab-content { display:none; }
.tab-content.active { display:block; }

/* ── TWO-COLUMN GRID ── */
.vfo-landscape { display:grid; grid-template-columns:1fr 1fr; gap:7px; align-items:start; }

/* ════ LEFT COLUMN ════ */

/* VFO A/B label row */
.vfo-ab-row { display:flex; gap:10px; margin-bottom:6px; }
.vfo-label  { flex:1; cursor:pointer; padding:3px 2px; border-radius:5px; font-family:'Courier New',monospace; font-size:14px; font-weight:bold; text-align:center; white-space:nowrap; overflow:hidden; text-overflow:ellipsis; transition:all 0.15s; }
.vfo-active   { background:#1a3a5c; color:#00d4ff; border:2px solid #00d4ff; }
.vfo-inactive { background:#1a1a2e; color:#8c8c8c;    border:2px solid #595959; }
.vfo-inactive:hover { color:#aaa; border-color:#555; }
.split-tag { background:#c0392b; color:white; font-size:0.7em; padding:1px 3px; border-radius:3px; margin-left:3px; }

/* Frequency display box */
.display { background:#1a3a5c; border-radius:9px; padding:6px 8px 5px 8px; border:2px solid #f1c40f; margin-bottom:5px; cursor:pointer; user-select:none; }
.display.memory-mode { background:#7a3a00; border-color:#e67e22; }
.display-top { display:flex; justify-content:space-between; align-items:center; margin-bottom:2px; }
#mode-label { background:#1a1a2e; padding:2px 8px; border-radius:9px; color:#f1c40f; font-weight:bold; font-size:11px; }
#step-label { background:#1a1a2e; padding:2px 7px; border-radius:9px; color:#f1c40f; font-size:11px; }
#f-display  { font-size:36px; text-align:center; font-family:'Courier New',monospace; font-weight:900; margin:3px 0 4px 0; color:white; text-shadow:2px 2px 4px #000; letter-spacing:2px; }
.s-meter-inline { display:flex; align-items:center; gap:4px; }
.s-grid  { display:flex; gap:2px; height:8px; flex:1; }
.s-seg   { flex:1; background:#2c3e50; border-radius:1px; }
.s-seg.on{ background:#2ecc71; }
.s-label { font-size:9px; color:#aaa; min-width:24px; text-align:right; font-family:monospace; }

/* Mode + Band row */
.mode-band-row { display:flex; gap:4px; margin:8px 0; }
.mode-selector { display:flex; gap:4px; }
.mode-btn { padding:4px 4px; background:#34495e; border:2px solid #f1c40f; border-radius:7px; cursor:pointer; color:white; font-weight:bold; font-size:14px; text-align:center; }
.mode-btn.active { background:#e74c3c; }
.band-buttons { display:flex; gap:3px; flex:1; }
.band-btn { flex:1; background:#7f0000; padding:4px 0; text-align:center; border-radius:7px; color:white; font-weight:bold; cursor:pointer; border:2px solid #f1c40f; font-size:14px; }
.band-btn.active-band { background:#27ae60; }

/* Split indicator */
#split-indicator { display:none; color:#e74c3c; font-weight:bold; font-size:10px; text-align:center; margin:2px 0; }

/* PTT */
.btn-ptt { width:100%; background:#c0392b; padding:10px 8px; font-size:16px; min-height:40px; border-radius:9px; border:2px solid #f1c40f; text-align:center; cursor:pointer; font-weight:bold; margin-top:10px; }
.btn-ptt-active { background:#00b300 !important; box-shadow:0 0 12px #00ff00; }
#band-warning { background:#e74c3c; padding:3px; border-radius:6px; font-size:9px; text-align:center; margin-top:3px; display:none; }

/* Right column top row: VFO AB buttons + SAVE corner */
/* Top row — dummy buttons + SAVE, all equal width */
.rc-top { display:flex; gap:4px; margin-bottom:6px; }
.rc-top .vstrip-btn { flex:1; }
.vstrip-btn.save { background:#6a1a9a; border:2px solid #a855f7; border-radius:7px; color:#d8b4fe; font-size:13px; font-weight:bold; padding:5px 4px; cursor:pointer; white-space:nowrap; text-align:center; }
.vstrip-btn.save:active { transform:scale(0.95); }
.vstrip-btn.dummy { background:#1a1a2e; border:2px solid #595959; border-radius:7px; color:#8c8c8c; font-size:13px; font-weight:bold; padding:5px 4px; cursor:default; text-align:center; }

/* Dummy stack right of knob — identical width to split-stack */
.dummy-stack { display:flex; flex-direction:column; gap:12px; flex:0 0 auto;width:calc(25% - 3px); margin-left:12px; }

/* Knob area — split stack left, then gap, slider, knob */
.knob-area { display:flex; align-items:flex-start; gap:4px; margin-bottom:8px; }

/* Split button stack — fixed width matching .btn flex:1 in a 4-item row
   In the freq row below: 4 buttons each ~25% of column width.
   Step row has 3 items so ◀ = ~33%. We want split ≈ step arrow width.
   Use flex:0 0 auto with same padding as .btn so they size like a single .btn */
.split-stack { display:flex; flex-direction:column; gap:12px; flex:0 0 auto; width:calc(25% - 3px); margin-right:12px; }
.vstrip-btn { background:#1a3a5c; border:2px solid #00d4ff; border-radius:7px; color:#00d4ff; font-size:20px; font-weight:bold; text-align:center; padding:10px 2px; cursor:pointer; width:100%; box-sizing:border-box; }
.vstrip-btn:active { transform:scale(0.95); }
.vstrip-btn.split { background:#1a1a2e; border-color:#e74c3c; color:#e74c3c; }
.vstrip-btn.split.active { background:#c0392b; color:white; border-color:#ff6b6b; }

/* Slider */
.vslider-container { display:flex; flex-direction:column; align-items:center; background:#1a1a2e; border-radius:12px; padding:5px 3px; }
.vslider-lbl { font-size:10px; font-weight:bold; }
.vslider-lbl.slow { color:#2ecc71; }
.vslider-lbl.fast { color:#e74c3c; }
.vslider { -webkit-appearance:slider-vertical; appearance:slider-vertical; width:18px; height:85px; background:linear-gradient(to top,#e74c3c,#f1c40f,#2ecc71); border-radius:9px; outline:none; cursor:pointer; }
.vslider::-webkit-slider-thumb { -webkit-appearance:none; appearance:none; width:14px; height:14px; background:white; border-radius:50%; cursor:pointer; }
.vslider-val { background:#000; padding:2px 3px; border-radius:5px; font-size:9px; font-weight:bold; color:#f1c40f; margin-top:3px; }

/* Knob — enlarged */
.knob { width:135px; height:135px; background:conic-gradient(from 0deg,#555,#999 25%,#555 50%,#999 75%,#555); border-radius:50%; border:5px solid #2c3e50; cursor:pointer; box-shadow:0 4px 12px rgba(0,0,0,0.5); position:relative; flex-shrink:0; touch-action:none; }
.knob::after { content:''; position:absolute; top:12px; left:50%; width:10px; height:10px; background:#e74c3c; border-radius:50%; transform:translateX(-50%); }

/* Step row */
.step-row { display:flex; align-items:center; gap:4px; margin:4px 4px; font-size:16px }
.step-display { flex:1; background:#1a1a2e; border:1px solid #f1c40f; border-radius:6px; padding:5px 2px; font-family:'Courier New',monospace; font-size:14px; color:#f1c40f; font-weight:bold; text-align:center; pointer-events:none; }

/* Shared button styles */
.btn { flex:1; padding:6px 3px; background:#34495e; border:2px solid #f1c40f; border-radius:7px; color:white; font-weight:bold; text-align:center; cursor:pointer; font-size:14px; }
.btn:active { transform:scale(0.96); }
.btn-primary { background:#27ae60; }
.btn-rit     { background:#e67e22; }
.btn-rit.active { background:#27ae60; }
.btn-memory  { background:#8e44ad; }
.btn-group   { display:flex; gap:4px; margin:4px 0; }

/* BC indicator — passive lamp, no touch */
.bc-btn { flex:1; padding:6px 3px; background:#2c3e50; border:2px solid #444; border-radius:7px; color:#444; font-weight:bold; text-align:center; font-size:10px; pointer-events:none; user-select:none; transition:all 0.3s; }
.bc-btn.bc-active { background:#e67e22; color:white; border-color:#f39c12; box-shadow:0 0 8px #e67e22; animation:bcPulse 2s infinite; }
@keyframes bcPulse { 0%,100%{box-shadow:0 0 8px #e67e22;} 50%{box-shadow:0 0 16px #f1c40f;} }

/* BFO tab */
.bfo-grid   { display:grid; grid-template-columns:1fr 1fr; gap:9px; }
.clock-card { background:#34495e; border-radius:9px; padding:9px; border:2px solid #f1c40f; }
.clock-title{ font-size:11px; font-weight:bold; color:#f1c40f; margin-bottom:6px; text-align:center; }
.clock-freq { font-size:17px; font-family:'Courier New',monospace; color:white; text-align:center; margin:6px 0; padding:6px; background:#1a1a2e; border-radius:8px; }
.clock-info { font-size:10px; color:#aaa; text-align:center; margin-bottom:7px; }
.step-label { color:white; font-size:10px; margin-bottom:4px; text-align:center; }
.step-value { color:#f1c40f; font-weight:bold; }

/* Memory tab */
.mem-panel  { background:#1a1a2e; border-radius:9px; padding:8px; margin-bottom:7px; }
.mem-header { color:#f1c40f; font-size:11px; margin-bottom:6px; font-weight:bold; }
.mem-buttons{ display:grid; grid-template-columns:repeat(5,1fr); gap:5px; }
.mem-btn    { background:#8e44ad; padding:5px; text-align:center; border-radius:7px; color:white; font-size:10px; cursor:pointer; border:1px solid #a55fc1; }
.mem-btn:active { transform:scale(0.95); }
.mem-btn.pending { background:#e67e22; border-color:#f39c12; animation:pulse 0.8s infinite; }

.signature  { text-align:center; color:#444; font-size:9px; margin-top:7px; }

@keyframes pulse { 0%,100%{opacity:1} 50%{opacity:0.65} }

/* Numpad overlay */
.numpad-btn { padding:10px; background:#34495e; border:1px solid #555; border-radius:8px; color:white; font-size:14px; font-weight:bold; text-align:center; cursor:pointer; }
.numpad-btn:active { background:#4a6274; transform:scale(0.95); }
#freq-entry-overlay.visible { display:flex !important; }
#freq-entry-input.error { border-color:#e74c3c !important; color:#e74c3c !important; }
#freq-entry-hint.error  { color:#e74c3c !important; }

/* Landscape phone tweaks */
@media (orientation:landscape) and (max-height:500px) {
  #f-display { font-size:24px; }
  .knob { width:100px; height:100px; touch-action:none; }
  .vslider { height:65px; }
  .btn-ptt { padding:7px; min-height:34px; }
  .vstrip-btn { padding:6px 4px; font-size:10px; }
}
@media (max-width:480px) {
  .vfo-landscape { grid-template-columns:1fr; }
  .bfo-grid { grid-template-columns:1fr; }
}
</style>
</head>
<body>
<div class="container">

  <!-- TAB BAR -->
  <div class="tab-container">
    <div class="tab-btn active" data-tab="vfo">📻 VFO</div>
    <div class="tab-btn" data-tab="bfo">🔧 BFO</div>
    <div class="tab-btn" data-tab="memory">💾 Memory</div>
  </div>

  <!-- ══════════ VFO TAB ══════════ -->
  <div id="vfo-tab" class="tab-content active">
    <div class="vfo-landscape">

      <!-- ── LEFT COLUMN ── -->
      <div>
        <!-- VFO A/B labels — outside display box, tap to select active VFO -->
        <div class="vfo-ab-row">
          <div id="vfo-a-freq" class="vfo-label vfo-active" onclick="selectVFOA()">VFO A: 7.000000 LSB</div>
          <div id="vfo-b-freq" class="vfo-label vfo-inactive" onclick="selectVFOB()">VFO B: 7.000000 LSB</div>
        </div>

        <!-- Frequency display — long press for direct entry -->
        <div class="display" id="display">
          <div class="display-top">
            <span id="mode-label">LSB</span>
            <span id="step-label">100Hz</span>
          </div>
          <div id="f-display">07.000.000</div>
          <div class="s-meter-inline">
            <div class="s-grid" id="s-grid"></div>
            <div class="s-label" id="s-label">S0</div>
          </div>
        </div>

        <!-- Mode + Band -->
        <div class="mode-band-row">
          <div class="mode-selector">
            <div class="mode-btn" id="mode-lsb">LSB</div>
            <div class="mode-btn" id="mode-usb">USB</div>
          </div>
          <div class="band-buttons">
            <div class="band-btn" data-band="0">80</div>
            <div class="band-btn" data-band="1">40</div>
            <div class="band-btn" data-band="2">20</div>
            <div class="band-btn" data-band="3">15</div>
            <div class="band-btn" data-band="4">10</div>
          </div>
        </div>

        <!-- Split indicator -->
        <div id="split-indicator">⚡ SPLIT TX — transmitting on VFO B</div>

        <!-- RIT row + BC indicator -->
        <div class="btn-group">
          <div class="btn btn-rit" id="rit-toggle">RIT</div>
          <div class="btn" id="rit-down">-</div>
          <div class="btn" id="rit-up">+</div>
          <div class="btn" id="rit-reset">Rst</div>
          <div class="bc-btn" id="bc-indicator">BC</div>
        </div>

        <!-- PTT -->
        <div class="btn-ptt" id="ptt-momentary">🔴 PTT</div>
        <div id="band-warning">⚠️ OUT OF BAND — TX DISABLED ⚠️</div>
      </div>

      <!-- ── RIGHT COLUMN ── -->
      <div>

        <!-- Top row: 2 dummy buttons + SAVE, equal thirds -->
        <div class="rc-top">
          <div class="vstrip-btn dummy">···</div>
          <div class="vstrip-btn dummy">···</div>
          <div class="vstrip-btn save" onclick="enterMemoryMode()" title="Save frequency to memory">💾 SAVE</div>
        </div>

        <!-- Knob area: split stack | gap | slider | knob | gap | dummy stack -->
        <div class="knob-area">
          <div class="split-stack">
            <div class="vstrip-btn" onclick="copyAtoB()" title="Copy VFO A to B">A=B</div>
            <div class="vstrip-btn" onclick="swapVFO()" title="Swap VFO A and B">A↔B</div>
            <div class="vstrip-btn split" id="split-btn" onclick="toggleSplit()" title="Split TX on VFO B">SPLT</div>
          </div>
          <div class="vslider-container">
            <div class="vslider-lbl slow">🐢</div>
            <input type="range" id="sensitivity-slider" class="vslider" min="2" max="10" value="6" step="1" orient="vertical">
            <div class="vslider-lbl fast">🐇</div>
            <div class="vslider-val" id="sensitivity-value">6</div>
          </div>
          <div class="knob" id="knob"></div>
          <div class="dummy-stack">
            <div class="vstrip-btn dummy" style="background:#1a1a2e;border-color:#595959;color:#8c8c8c;cursor:default;">···</div>
            <div class="vstrip-btn dummy" style="background:#1a1a2e;border-color:#595959;color:#8c8c8c;cursor:default;">···</div>
            <div class="vstrip-btn dummy" style="background:#1a1a2e;border-color:#595959;color:#8c8c8c;cursor:default;">···</div>
          </div>
        </div>

        <!-- Step display + arrows — untouched -->
        <div class="step-row">
          <div class="btn" id="step-down">◀</div>
          <div class="step-display" id="step-display">100 Hz</div>
          <div class="btn" id="step-up">▶</div>
        </div>

        <!-- Frequency nudge buttons — untouched -->
        <div class="btn-group">
          <div class="btn" id="freq-10">-10</div>
          <div class="btn" id="freq-1">-1</div>
          <div class="btn" id="freq+1">+1</div>
          <div class="btn" id="freq+10">+10</div>
        </div>

      </div><!-- end right column -->
    </div><!-- end vfo-landscape -->
  </div><!-- end vfo-tab -->
  
  <!-- ══════════ BFO TAB ══════════ -->
  <div id="bfo-tab" class="tab-content">
    <div class="bfo-grid">
      <div class="clock-card">
        <div class="clock-title">🔧 BFO1 (CLK1) — Second IF Mixer</div>
        <div class="clock-freq" id="bfo1-display-main">33.000000 MHz</div>
        <div class="clock-info">USB: 57 MHz | LSB: 33 MHz</div>
        <div class="step-label">Step: <span id="bfo1-step-main" class="step-value">1KHz</span></div>
        <div class="btn-group">
          <div class="btn" id="bfo1-step-down">◀ Slower</div>
          <div class="btn btn-primary" id="bfo1-step-cycle">Cycle</div>
          <div class="btn" id="bfo1-step-up">Faster ▶</div>
        </div>
        <div class="btn-group">
          <div class="btn" id="bfo1-minus-10">-10</div>
          <div class="btn" id="bfo1-minus-1">-1</div>
          <div class="btn" id="bfo1-plus-1">+1</div>
          <div class="btn" id="bfo1-plus-10">+10</div>
        </div>
        <div class="btn-group">
          <div class="btn" id="bfo1-usb">USB (57MHz)</div>
          <div class="btn" id="bfo1-lsb">LSB (33MHz)</div>
          <div class="btn btn-memory" id="bfo1-save">💾 Save</div>
        </div>
      </div>
      <div class="clock-card">
        <div class="clock-title">🔧 BFO2 (CLK0) — Audio Demodulator</div>
        <div class="clock-freq" id="bfo2-display-main">12.000000 MHz</div>
        <div class="clock-info">Fixed 12 MHz Carrier</div>
        <div class="step-label">Step: <span id="bfo2-step-main" class="step-value">1KHz</span></div>
        <div class="btn-group">
          <div class="btn" id="bfo2-step-down">◀ Slower</div>
          <div class="btn btn-primary" id="bfo2-step-cycle">Cycle</div>
          <div class="btn" id="bfo2-step-up">Faster ▶</div>
        </div>
        <div class="btn-group">
          <div class="btn" id="bfo2-minus-10">-10</div>
          <div class="btn" id="bfo2-minus-1">-1</div>
          <div class="btn" id="bfo2-plus-1">+1</div>
          <div class="btn" id="bfo2-plus-10">+10</div>
        </div>
        <div class="btn-group">
          <div class="btn" id="bfo2-reset">Reset 12MHz</div>
          <div class="btn btn-memory" id="bfo2-save">💾 Save</div>
        </div>
      </div>
    </div>
  </div>

  <!-- ══════════ MEMORY TAB ══════════ -->
  <div id="memory-tab" class="tab-content">
    <div class="mem-panel">
      <div class="mem-header">📀 Operating Freq Memories (10 slots)</div>
      <div class="mem-buttons" id="mem-buttons-tab"></div>
    </div>
    <div class="mem-panel">
      <div class="mem-header">📀 BFO1 Memories (10 slots)</div>
      <div class="mem-buttons" id="bfo1-mem-buttons-tab"></div>
    </div>
    <div class="mem-panel">
      <div class="mem-header">📀 BFO2 Memories (10 slots)</div>
      <div class="mem-buttons" id="bfo2-mem-buttons-tab"></div>
    </div>
  </div>

  <div class="signature">uBITX VFO 5.4F | VFO = OpFreq + (BFO1 ± BFO2) | Long-press display = direct freq entry</div>
  <!-- ══ DIRECT FREQUENCY ENTRY OVERLAY ══ -->
  <div id="freq-entry-overlay" style="display:none;position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(0,0,0,0.82);z-index:500;align-items:center;justify-content:center;">
    <div style="background:#1a3a5c;border:2px solid #f1c40f;border-radius:14px;padding:14px;width:240px;max-width:90vw;">
      <div style="color:#f1c40f;font-size:12px;font-weight:bold;margin-bottom:8px;text-align:center;">📻 Enter Frequency</div>
      <div id="freq-entry-target" style="color:#aaa;font-size:10px;text-align:center;margin-bottom:6px;">Tuning VFO A</div>
      <input id="freq-entry-input" type="text" readonly
        style="width:100%;background:#0a1a2e;border:2px solid #f1c40f;border-radius:8px;color:white;font-family:'Courier New',monospace;font-size:20px;font-weight:bold;padding:7px 10px;text-align:right;margin-bottom:4px;outline:none;box-sizing:border-box;">
      <div id="freq-entry-hint" style="font-size:9px;color:#aaa;text-align:center;min-height:14px;margin-bottom:8px;">Enter MHz  e.g. 14.225 or 7.010</div>
      <div style="display:grid;grid-template-columns:repeat(3,1fr);gap:5px;margin-bottom:8px;">
        <div class="numpad-btn" data-digit="1">1</div><div class="numpad-btn" data-digit="2">2</div><div class="numpad-btn" data-digit="3">3</div>
        <div class="numpad-btn" data-digit="4">4</div><div class="numpad-btn" data-digit="5">5</div><div class="numpad-btn" data-digit="6">6</div>
        <div class="numpad-btn" data-digit="7">7</div><div class="numpad-btn" data-digit="8">8</div><div class="numpad-btn" data-digit="9">9</div>
        <div class="numpad-btn" data-digit="." style="background:#2c3e50;">.</div>
        <div class="numpad-btn" data-digit="0">0</div>
        <div class="numpad-btn" id="numpad-del" style="background:#7f0000;">⌫</div>
      </div>
      <div style="display:flex;gap:8px;">
        <button id="freq-entry-cancel" style="flex:1;padding:9px;background:#555;border:none;border-radius:8px;color:white;font-weight:bold;font-size:13px;cursor:pointer;">CANCEL</button>
        <button id="freq-entry-go"     style="flex:1;padding:9px;background:#27ae60;border:none;border-radius:8px;color:white;font-weight:bold;font-size:13px;cursor:pointer;">GO ▶</button>
      </div>
    </div>
  </div>

</div><!-- end container -->

<script>
// ==================== CONSTANTS ====================
const MIN_OP  = 3500000;
const MAX_OP  = 30000000;
const MAX_RIT = 5000;
const steps      = [10,100,1000,5000,10000,100000,1000000,10000000];
const stepLabels = ["10Hz","100Hz","1KHz","5KHz","10KHz","100KHz","1MHz","10MHz"];
const bfoSteps      = [10,100,1000,5000,10000,100000,1000000];
const bfoStepLabels = ["10Hz","100Hz","1KHz","5KHz","10KHz","100KHz","1MHz"];

// Single source of truth for all band logic
const BAND_LIMITS = [
  {min:3500000,  max:4000000 },
  {min:7000000,  max:7200000 },
  {min:14000000, max:14350000},
  {min:21000000, max:21450000},
  {min:28000000, max:29700000}
];
const BAND_DEFAULT_MODES = ["LSB","LSB","USB","USB","USB"];
const BAND_DEFAULT_FREQS = [3500000,7000000,14000000,21000000,28000000];
const BAND_NAMES         = ["80m","40m","20m","15m","10m"];

// Known non-ham stations for BC indicator label
const KNOWN_STATIONS = [
  {freq:2500000,  tol:2000, label:"WWV"},
  {freq:3330000,  tol:2000, label:"CHU"},
  {freq:5000000,  tol:2000, label:"WWV"},
  {freq:7850000,  tol:2000, label:"CHU"},
  {freq:10000000, tol:2000, label:"WWV"},
  {freq:14670000, tol:2000, label:"CHU"},
  {freq:15000000, tol:2000, label:"WWV"},
  {freq:20000000, tol:2000, label:"WWV"}
];

// ==================== STATE ====================
let operatingFreq  = 7000000;
let mode           = "LSB";
let stepIdx        = 1;
let ritEnabled     = false;
let ritOffset      = 0;
let memories       = [[],[],[]];
let memoryMode     = false;
let pttActive      = false;
let pttAutoOffTimer= null;

let BFO1_USB = 56995000;
let BFO1_LSB = 32995000;
let BFO2_DEFAULT = 11997500;
let bfo1Freq    = 32995000;
let bfo2Freq    = 11997500;
let bfo1StepIdx = 2;
let bfo2StepIdx = 2;
let bfo1Memories = new Array(10).fill(0);
let bfo2Memories = new Array(10).fill(0);

// VFO A/B
let vfoBFreq   = 7000000;
let vfoBMode   = "LSB";
let vfoBActive = false;
let splitMode  = false;

// Per-band last tuned frequency — kept in sync as operator tunes
let bandLastFreq = [...BAND_DEFAULT_FREQS];

// Knob
let isDragging = false, lastAngle = 0, currentRotation = 0;
let lastKnobSendTime = 0, knobSensitivity = 6;

// BFO debounce
let bfoSaveTimer = null;

// Initialise memories
for(let i=0;i<3;i++) for(let j=0;j<10;j++) memories[i][j]=0;

// ==================== BAND / FREQ HELPERS ====================
function getBandIndexFromFreq(freq) {
  for(let i=0;i<BAND_LIMITS.length;i++)
    if(freq>=BAND_LIMITS[i].min && freq<=BAND_LIMITS[i].max) return i;
  return -1;
}
function isWithinHamBand(freq) { return getBandIndexFromFreq(freq)>=0; }

function getBCLabel(freq) {
  if(isWithinHamBand(freq)) return "BC";
  for(let s of KNOWN_STATIONS)
    if(Math.abs(freq-s.freq)<=s.tol) return s.label;
  return "BC";
}

function getRXFreq() {
  if(ritEnabled && !pttActive) return operatingFreq + ritOffset;
  return operatingFreq;
}

function formatFrequency(freq) {
  let f = (freq!==undefined) ? freq : getRXFreq();
  return f>=1000000 ? (f/1000000).toFixed(6)+" MHz" : (f/1000).toFixed(3)+" kHz";
}
function formatBFOfreq(freq) {
  return freq>=1000000 ? (freq/1000000).toFixed(6)+" MHz" : (freq/1000).toFixed(3)+" kHz";
}

// ==================== TAB SWITCHING ====================
function switchTab(name) {
  document.querySelectorAll('.tab-content').forEach(function(t){ t.classList.remove('active'); });
  document.querySelectorAll('.tab-btn').forEach(function(b){ b.classList.remove('active'); });
  let tab=document.getElementById(name+'-tab');   if(tab) tab.classList.add('active');
  let btn=document.querySelector('.tab-btn[data-tab="'+name+'"]'); if(btn) btn.classList.add('active');
}

// ==================== DISPLAY UPDATES ====================
function updateVFODisplay() {
  let rxFreq = vfoBActive ? vfoBFreq : getRXFreq();
  document.getElementById('f-display').innerText = formatFrequency(rxFreq);

  let aLabel=document.getElementById('vfo-a-freq');
  if(aLabel) {
    aLabel.className = vfoBActive ? 'vfo-label vfo-inactive' : 'vfo-label vfo-active';
    aLabel.innerText = (operatingFreq/1000000).toFixed(6)+' '+mode;
  }
  let bLabel=document.getElementById('vfo-b-freq');
  if(bLabel) {
    bLabel.className = vfoBActive ? 'vfo-label vfo-active' : 'vfo-label vfo-inactive';
    bLabel.innerHTML = (vfoBFreq/1000000).toFixed(6)+' '+vfoBMode
      +(splitMode?' <span class="split-tag">TX</span>':'');
  }
  let si=document.getElementById('split-indicator');
  if(si) si.style.display = splitMode ? 'block' : 'none';
}

function updateBandDisplay() {
  let freq = vfoBActive ? vfoBFreq : operatingFreq;
  let bi   = getBandIndexFromFreq(freq);
  document.querySelectorAll('.band-btn').forEach(function(b){ b.classList.remove('active-band'); });
  if(bi>=0) {
    let ab=document.querySelector('.band-btn[data-band="'+bi+'"]');
    if(ab) ab.classList.add('active-band');
  }
}

function updateBandWarning() {
  // Check frequency that will actually be transmitted
  let txFreq  = splitMode ? vfoBFreq : operatingFreq;
  let inBand  = isWithinHamBand(txFreq);
  let warnDiv = document.getElementById('band-warning');
  let pttBtn  = document.getElementById('ptt-momentary');
  if(!inBand) {
    if(warnDiv) warnDiv.style.display='block';
    if(pttBtn&&!pttActive) {
      pttBtn.style.opacity='0.4'; pttBtn.style.backgroundColor='#555';
      pttBtn.style.pointerEvents='none';
    }
  } else {
    if(warnDiv) warnDiv.style.display='none';
    if(pttBtn&&!pttActive) {
      pttBtn.style.opacity='1'; pttBtn.style.backgroundColor='#c0392b';
      pttBtn.style.pointerEvents='auto';
    }
  }
}

function updateBCIndicator() {
  let freq  = vfoBActive ? vfoBFreq : operatingFreq;
  let label = getBCLabel(freq);
  let btn   = document.getElementById('bc-indicator');
  if(!btn) return;
  btn.innerText = label;
  if(!isWithinHamBand(freq)) btn.classList.add('bc-active');
  else                        btn.classList.remove('bc-active');
}

function updateUI() {
  updateVFODisplay();

  // Step — write to both display locations
  let sd=document.getElementById('step-display'); if(sd) sd.innerText=stepLabels[stepIdx];
  let sl=document.getElementById('step-label');   if(sl) sl.innerText=stepLabels[stepIdx];

  // RIT button
  let ritBtn=document.getElementById('rit-toggle');
  if(ritBtn) {
    if(ritEnabled) {
      ritBtn.innerHTML='RIT '+(ritOffset>0?'+':'')+(ritOffset/1000).toFixed(1)+'K';
      ritBtn.classList.add('active');
    } else {
      ritBtn.innerHTML='RIT';
      ritBtn.classList.remove('active');
    }
  }

  // Mode buttons
  let ml=document.getElementById('mode-lsb'); if(ml) ml.classList.toggle('active',mode==="LSB");
  let mu=document.getElementById('mode-usb'); if(mu) mu.classList.toggle('active',mode==="USB");
  let lbl=document.getElementById('mode-label'); if(lbl) lbl.innerText=mode;

  // Memory mode highlight on display box
  let disp=document.getElementById('display');
  if(disp) disp.classList.toggle('memory-mode',memoryMode);

  updateBandDisplay();
  updateBandWarning();
  updateBCIndicator();
  updateVFOMemoryButtons();
}

// ==================== S-METER ====================
async function updateSMeter() {
  try {
    let r=await fetch('/getS');
    let val=parseInt(await r.text());
    let active=Math.min(20,Math.floor(val/5));
    document.querySelectorAll('.s-seg').forEach(function(seg,i){
      seg.classList.toggle('on',i<active);
    });
    let sl=document.getElementById('s-label');
    if(sl) sl.innerText = active>=18 ? 'S9+' : 'S'+Math.round(active/2.22);
  } catch(e){}
}

// ==================== MEMORY ====================
function updateVFOMemoryButtons() {
  let c=document.getElementById('mem-buttons-tab'); if(!c) return;
  c.innerHTML='';
  for(let i=0;i<10;i++) {
    let btn=document.createElement('div');
    btn.className='mem-btn'+(memoryMode?' pending':'');
    let v=memories[0][i];
    btn.innerHTML=(v&&v>0)?'M'+(i+1)+'<br>'+(v/1000000).toFixed(3)+'M':'M'+(i+1)+'<br>Empty';
    btn.onclick=(function(slot){ return function(){ handleVFOMemory(slot); }; })(i);
    c.appendChild(btn);
  }
}

function updateAllMemoryButtons() {
  updateVFOMemoryButtons();
  function buildPanel(id, arr, handler) {
    let c=document.getElementById(id); if(!c) return;
    c.innerHTML='';
    for(let i=0;i<10;i++) {
      let btn=document.createElement('div'); btn.className='mem-btn';
      let v=arr[i];
      btn.innerHTML=(v&&v>0)?'M'+(i+1)+'<br>'+(v/1000000).toFixed(3)+'M':'M'+(i+1)+'<br>Empty';
      btn.onclick=(function(slot){ return function(){ handler(slot); }; })(i);
      c.appendChild(btn);
    }
  }
  buildPanel('bfo1-mem-buttons-tab', bfo1Memories, handleBFO1Memory);
  buildPanel('bfo2-mem-buttons-tab', bfo2Memories, handleBFO2Memory);
}

function handleVFOMemory(slot) {
  if(memoryMode) {
    memories[0][slot]=operatingFreq;
    exitMemoryMode();
    sendConfigWithSave();
    showMessage('✓ M'+(slot+1)+' saved — '+(operatingFreq/1000000).toFixed(3)+' MHz','#27ae60');
    setTimeout(function(){ switchTab('vfo'); },3000);
  } else {
    let saved=memories[0][slot];
    if(saved&&saved>0) {
      operatingFreq=saved;
      let bi=getBandIndexFromFreq(saved);
      if(bi>=0){ mode=BAND_DEFAULT_MODES[bi]; bandLastFreq[bi]=saved; }
      updateUI(); sendConfig(true); refreshBFO1ForMode();
      showMessage('◀ M'+(slot+1)+' recalled — '+(saved/1000000).toFixed(3)+' MHz','#e67e22');
      setTimeout(function(){ switchTab('vfo'); },3000);
    } else { showMessage('M'+(slot+1)+' is empty','#e74c3c'); }
  }
}

function enterMemoryMode() {
  memoryMode=true; updateUI(); switchTab('memory');
  showMessage('Tap a slot to save current frequency','#f1c40f');
  setTimeout(function(){ if(memoryMode) exitMemoryMode(); },10000);
}
function exitMemoryMode() { memoryMode=false; updateUI(); }

function saveBFO1ToMemory() {
  window.pendingSave={type:'bfo1',frequency:bfo1Freq}; switchTab('memory');
  showMessage('Tap a BFO1 slot to save','#f1c40f');
  setTimeout(function(){ if(window.pendingSave&&window.pendingSave.type==='bfo1') window.pendingSave=null; },10000);
}
function saveBFO2ToMemory() {
  window.pendingSave={type:'bfo2',frequency:bfo2Freq}; switchTab('memory');
  showMessage('Tap a BFO2 slot to save','#f1c40f');
  setTimeout(function(){ if(window.pendingSave&&window.pendingSave.type==='bfo2') window.pendingSave=null; },10000);
}

function handleBFO1Memory(slot) {
  if(window.pendingSave&&window.pendingSave.type==='bfo1') {
    bfo1Memories[slot]=window.pendingSave.frequency;
    if(memories[1]) memories[1][slot]=window.pendingSave.frequency;
    updateAllMemoryButtons();
    fetch('/setSetupConfig',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({bfo1Freq,bfo2Freq,bfo1StepIdx,bfo2StepIdx,bfo1Memories,bfo2Memories,immediateSave:true})});
    window.pendingSave=null;
    showMessage('✓ BFO1 M'+(slot+1)+' saved','#27ae60');
    setTimeout(function(){ switchTab('bfo'); },3000);
  } else {
    let saved=bfo1Memories[slot];
    if(saved&&saved>0){ bfo1Freq=saved; updateBFO1UI(); applyBFOChanges(); showMessage('◀ BFO1 M'+(slot+1)+' recalled','#e67e22'); }
    else showMessage('M'+(slot+1)+' empty','#e74c3c');
  }
}

function handleBFO2Memory(slot) {
  if(window.pendingSave&&window.pendingSave.type==='bfo2') {
    bfo2Memories[slot]=window.pendingSave.frequency;
    if(memories[2]) memories[2][slot]=window.pendingSave.frequency;
    updateAllMemoryButtons();
    fetch('/setSetupConfig',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({bfo1Freq,bfo2Freq,bfo1StepIdx,bfo2StepIdx,bfo1Memories,bfo2Memories,immediateSave:true})});
    window.pendingSave=null;
    showMessage('✓ BFO2 M'+(slot+1)+' saved','#27ae60');
    setTimeout(function(){ switchTab('bfo'); },3000);
  } else {
    let saved=bfo2Memories[slot];
    if(saved&&saved>0){ bfo2Freq=saved; updateBFO2UI(); applyBFOChanges(); showMessage('◀ BFO2 M'+(slot+1)+' recalled','#e67e22'); }
    else showMessage('M'+(slot+1)+' empty','#e74c3c');
  }
}

// ==================== VFO ACTIONS ====================
function setMode(newMode) {
  mode=newMode; updateUI(); sendConfig(true); refreshBFO1ForMode();
  // No loadConfig() here — races with sendConfig
}
function toggleRIT()      { ritEnabled=!ritEnabled; updateUI(); sendRITConfig(); }
function adjustRIT(delta) { ritOffset=Math.max(-MAX_RIT,Math.min(MAX_RIT,ritOffset+delta)); updateUI(); sendRITConfig(); }
function resetRIT()       { ritOffset=0; ritEnabled=false; updateUI(); sendRITConfig(); }

function adjustFrequency(delta) {
  let step=steps[stepIdx];
  if(vfoBActive) {
    let nf=vfoBFreq+delta*step;
    if(nf>=MIN_OP&&nf<=MAX_OP){ vfoBFreq=nf; sendVFOB(); updateVFODisplay(); updateBCIndicator(); }
  } else {
    let nf=operatingFreq+delta*step;
    if(nf>=MIN_OP&&nf<=MAX_OP){
      operatingFreq=nf;
      let bi=getBandIndexFromFreq(nf); if(bi>=0) bandLastFreq[bi]=nf;
      updateUI(); sendConfig();
    }
  }
}

function changeStep(delta) {
  let ni=stepIdx+delta;
  if(ni>=0&&ni<steps.length){ stepIdx=ni; updateUI(); sendConfig(true); }
}

async function setBand(bandIndex) {
  if(bandIndex<0||bandIndex>=BAND_LIMITS.length) return;
  let nf=bandLastFreq[bandIndex];
  nf=Math.max(BAND_LIMITS[bandIndex].min,Math.min(BAND_LIMITS[bandIndex].max,nf));
  operatingFreq=nf; mode=BAND_DEFAULT_MODES[bandIndex];
  updateUI(); refreshBFO1ForMode();
  // Await /setBand first (it updates the firmware's own per-band frequency
  // memory) so it can't land AFTER /setConfig and clobber the frequency we
  // just sent. /setConfig always goes last and is the authoritative value.
  try { await fetch('/setBand?band='+bandIndex); } catch(e){ console.log('Band err:',e); }
  sendConfig(true);
  showMessage(BAND_NAMES[bandIndex]+' | '+mode+' | '+(nf/1000000).toFixed(3)+' MHz','#27ae60');
}

// ==================== VFO A/B ====================
function selectVFOA() { vfoBActive=false; updateUI(); }
function selectVFOB() { vfoBActive=true; updateVFODisplay(); updateBCIndicator(); }

function swapVFO() {
  fetch('/swapVFO',{method:'POST'}).then(function(){
    let tf=operatingFreq; operatingFreq=vfoBFreq; vfoBFreq=tf;
    let tm=mode;          mode=vfoBMode;           vfoBMode=tm;
    vfoBActive=!vfoBActive; updateUI(); refreshBFO1ForMode();
  });
}

function copyAtoB() {
  vfoBFreq=operatingFreq; vfoBMode=mode;
  fetch('/copyAtoB',{method:'POST'}).then(function(){
    updateVFODisplay();
    showMessage('VFO B = VFO A — '+(vfoBFreq/1000000).toFixed(3)+' MHz','#27ae60');
  });
}

function toggleSplit() {
  splitMode=!splitMode;
  fetch('/setSplit',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({split:splitMode})}).then(function(){
    let sb=document.getElementById('split-btn');
    if(sb) sb.classList.toggle('active',splitMode);
    updateUI();
    showMessage(splitMode?'⚡ SPLIT ON — TX on VFO B':'Split OFF',splitMode?'#e74c3c':'#27ae60');
  });
}

function sendVFOB() {
  fetch('/setVFOB',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({vfoBFreq,vfoBMode,vfoBActive})});
}

// ==================== BFO ====================
function updateBFO1UI() {
  let e=document.getElementById('bfo1-display-main'); if(e) e.innerHTML=formatBFOfreq(bfo1Freq);
  let s=document.getElementById('bfo1-step-main');    if(s) s.innerText=bfoStepLabels[bfo1StepIdx];
}
function updateBFO2UI() {
  let e=document.getElementById('bfo2-display-main'); if(e) e.innerHTML=formatBFOfreq(bfo2Freq);
  let s=document.getElementById('bfo2-step-main');    if(s) s.innerText=bfoStepLabels[bfo2StepIdx];
}

// Every place that sets `mode` (Mode button, band switch, memory recall, direct
// freq entry, VFO swap) makes the firmware retune BFO1 via updateBFO1ByMode() —
// but nothing told the JS-side bfo1Freq / BFO display about it, so the BFO tab
// kept showing the old value even though the radio had switched. This mirrors
// the firmware's own logic so the display stays truthful without a page reload.
function refreshBFO1ForMode() {
  bfo1Freq = (mode === "USB") ? BFO1_USB : BFO1_LSB;
  updateBFO1UI();
}

// Hardware-only update — no EEPROM activity
function applyBFOChanges() {
  fetch('/setSetupConfig',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({bfo1Freq,bfo2Freq,bfo1StepIdx,bfo2StepIdx,
      bfo1Memories,bfo2Memories,hardwareOnly:true})});
}

// Delayed EEPROM write — fires 30s after last BFO adjustment
function debouncedEEPROMSave() {
  if(bfoSaveTimer) clearTimeout(bfoSaveTimer);
  bfoSaveTimer=setTimeout(function(){
    fetch('/setSetupConfig',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({bfo1Freq,bfo2Freq,bfo1StepIdx,bfo2StepIdx,
        bfo1Memories,bfo2Memories,immediateSave:true})});
    bfoSaveTimer=null;
  },30000);
}

function adjustBFO1(delta) {
  let nf=bfo1Freq+delta*bfoSteps[bfo1StepIdx];
  if(nf>=1000000&&nf<=60000000){
    bfo1Freq=nf; updateBFO1UI(); applyBFOChanges(); debouncedEEPROMSave();
    showMessage('BFO1: '+(bfo1Freq/1000000).toFixed(3)+' MHz','#27ae60');
  }
}
function adjustBFO2(delta) {
  let nf=bfo2Freq+delta*bfoSteps[bfo2StepIdx];
  if(nf>=1000000&&nf<=60000000){
    bfo2Freq=nf; updateBFO2UI(); applyBFOChanges(); debouncedEEPROMSave();
    showMessage('BFO2: '+(bfo2Freq/1000000).toFixed(3)+' MHz','#27ae60');
  }
}

function doBFO1StepDown()  { if(bfo1StepIdx>0){ bfo1StepIdx--; updateBFO1UI(); applyBFOChanges(); debouncedEEPROMSave(); } }
function doBFO1StepUp()    { if(bfo1StepIdx<bfoSteps.length-1){ bfo1StepIdx++; updateBFO1UI(); applyBFOChanges(); debouncedEEPROMSave(); } }
function doBFO1StepCycle() { bfo1StepIdx=(bfo1StepIdx+1)%bfoSteps.length; updateBFO1UI(); applyBFOChanges(); debouncedEEPROMSave(); }
function doBFO2StepDown()  { if(bfo2StepIdx>0){ bfo2StepIdx--; updateBFO2UI(); applyBFOChanges(); debouncedEEPROMSave(); } }
function doBFO2StepUp()    { if(bfo2StepIdx<bfoSteps.length-1){ bfo2StepIdx++; updateBFO2UI(); applyBFOChanges(); debouncedEEPROMSave(); } }
function doBFO2StepCycle() { bfo2StepIdx=(bfo2StepIdx+1)%bfoSteps.length; updateBFO2UI(); applyBFOChanges(); debouncedEEPROMSave(); }

function doSetBFO1toUSB() { bfo1Freq=BFO1_USB; updateBFO1UI(); applyBFOChanges(); debouncedEEPROMSave(); showMessage('BFO1 → USB 57MHz','#27ae60'); }
function doSetBFO1toLSB() { bfo1Freq=BFO1_LSB; updateBFO1UI(); applyBFOChanges(); debouncedEEPROMSave(); showMessage('BFO1 → LSB 33MHz','#27ae60'); }
function doResetBFO2()    { bfo2Freq=BFO2_DEFAULT; updateBFO2UI(); applyBFOChanges(); debouncedEEPROMSave(); showMessage('BFO2 reset to 12MHz','#27ae60'); }

// ==================== PTT ====================
async function setPTT(active) {
  if(active===pttActive) return;
  let txFreq=splitMode?vfoBFreq:operatingFreq;
  if(active&&!isWithinHamBand(txFreq)){ showMessage('⛔ TX BLOCKED — out of band!','#e74c3c'); return; }
  if(active&&splitMode&&!isWithinHamBand(vfoBFreq)){ showMessage('⛔ SPLIT TX BLOCKED — VFO B out of band!','#e74c3c'); return; }
  try {
    await fetch('/ptt?action='+(active?'on':'off'));
    pttActive=active;
    let btn=document.getElementById('ptt-momentary');
    if(btn) {
      if(active){
        btn.classList.add('btn-ptt-active'); btn.innerHTML='🟢 TX ON — tap to stop';
        btn.style.animation='pulse 1s infinite';
        pttAutoOffTimer=setTimeout(function(){ showMessage('⚠️ Auto TX-OFF 3min limit','#e67e22'); setPTT(false); },180000);
        if(splitMode) showMessage('📡 TX on VFO B: '+(vfoBFreq/1000000).toFixed(6)+' MHz','#e67e22');
      } else {
        btn.classList.remove('btn-ptt-active'); btn.innerHTML='🔴 PTT';
        btn.style.animation='';
        if(pttAutoOffTimer){ clearTimeout(pttAutoOffTimer); pttAutoOffTimer=null; }
      }
    }
    updateUI();
  } catch(e){ showMessage('PTT error — check connection','#e74c3c'); }
}
function togglePTT() { setPTT(!pttActive); }

// ==================== SERVER ====================
function sendConfig(force) {
  let now=Date.now();
  if(force || now-lastKnobSendTime>50){
    fetch('/setConfig',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({operatingFreq,mode,stepIdx,memories})})
      .catch(function(e){ console.log('setConfig err:',e); showMessage('⚠ Config send failed','#e74c3c'); });
    lastKnobSendTime=now;
  }
}
function sendConfigWithSave() {
  fetch('/setConfig',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({operatingFreq,mode,stepIdx,memories,immediateSave:true})});
}
function sendRITConfig() {
  fetch('/setRIT',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({enabled:ritEnabled,offset:ritOffset})});
}

async function loadConfig() {
  try {
    let r=await fetch('/getConfig'); let c=await r.json();
    operatingFreq=c.operatingFreq||7000000; mode=c.mode||"LSB"; stepIdx=c.stepIdx||1;
    if(c.memories) for(let b=0;b<3;b++) if(c.memories[b]) memories[b]=[...c.memories[b]];
    if(c.bfo1Usb)     BFO1_USB    =c.bfo1Usb;
    if(c.bfo1Lsb)     BFO1_LSB    =c.bfo1Lsb;
    if(c.bfo2Default) BFO2_DEFAULT=c.bfo2Default;
  } catch(e){ console.log('getConfig err:',e); }
  try {
    let r=await fetch('/getSetupConfig'); let c=await r.json();
    bfo1Freq=c.bfo1Freq||bfo1Freq; bfo2Freq=c.bfo2Freq||bfo2Freq;
    bfo1StepIdx=c.bfo1StepIdx||2;  bfo2StepIdx=c.bfo2StepIdx||2;
    if(c.bfo1Memories) bfo1Memories=[...c.bfo1Memories];
    if(c.bfo2Memories) bfo2Memories=[...c.bfo2Memories];
    updateBFO1UI(); updateBFO2UI();
  } catch(e){ console.log('getSetupConfig err:',e); }
  try {
    let r=await fetch('/getRIT'); let c=await r.json();
    ritEnabled=c.enabled||false; ritOffset=c.offset||0;
  } catch(e){ console.log('getRIT err:',e); }
  try {
    let r=await fetch('/getVFOB'); let c=await r.json();
    vfoBFreq=c.vfoBFreq||7000000; vfoBMode=c.vfoBMode||"LSB";
    vfoBActive=c.vfoBActive||false; splitMode=c.splitMode||false;
    let sb=document.getElementById('split-btn');
    if(sb) sb.classList.toggle('active',splitMode);
  } catch(e){ console.log('getVFOB err:',e); }
  updateUI(); updateAllMemoryButtons();
}

// ==================== MESSAGES ====================
function showMessage(msg,color) {
  let el=document.createElement('div');
  el.innerHTML=msg;
  Object.assign(el.style,{position:'fixed',bottom:'20px',left:'50%',
    transform:'translateX(-50%)',background:color,color:'white',
    padding:'8px 16px',borderRadius:'20px',fontSize:'12px',
    fontWeight:'bold',zIndex:'1000'});
  document.body.appendChild(el);
  setTimeout(function(){ el.remove(); },2500);
}

// ==================== KNOB ====================
function getAngle(x,y) {
  let k=document.getElementById('knob'); let r=k.getBoundingClientRect();
  return Math.atan2(y-(r.top+r.height/2),x-(r.left+r.width/2))*180/Math.PI;
}
function startDrag(e) {
  isDragging=true; let ev=e.touches?e.touches[0]:e;
  lastAngle=getAngle(ev.clientX,ev.clientY); e.preventDefault();
}
function stopDrag()  { isDragging=false; }
function move(e) {
  if(!isDragging) return; e.preventDefault();
  let ev=e.touches?e.touches[0]:e;
  let ang=getAngle(ev.clientX,ev.clientY);
  let d=ang-lastAngle;
  if(d>180) d-=360; if(d<-180) d+=360;
  currentRotation+=d;
  let sc=Math.round(d/knobSensitivity);
  if(sc>3) sc=3; if(sc<-3) sc=-3;
  if(sc!==0) adjustFrequency(sc);
  let knob=document.getElementById('knob');
  if(knob) knob.style.transform='rotate('+currentRotation+'deg)';
  lastAngle=ang;
}

// ==================== DIRECT FREQ ENTRY ====================
let freqEntryValue='';
const LONG_PRESS_MS=600;

function openFreqEntry() {
  if(memoryMode) return;
  freqEntryValue='';
  let inp=document.getElementById('freq-entry-input');
  let hint=document.getElementById('freq-entry-hint');
  let tgt=document.getElementById('freq-entry-target');
  if(inp){ inp.value=''; inp.classList.remove('error'); }
  if(hint){ hint.innerText='Enter MHz  e.g. 14.225 or 7.010'; hint.classList.remove('error'); }
  if(tgt) tgt.innerText=vfoBActive?'Tuning VFO B':'Tuning VFO A';
  let ov=document.getElementById('freq-entry-overlay'); if(ov) ov.classList.add('visible');
}
function closeFreqEntry() {
  let ov=document.getElementById('freq-entry-overlay'); if(ov) ov.classList.remove('visible');
  freqEntryValue='';
}
function freqEntryDigit(d) {
  let inp=document.getElementById('freq-entry-input');
  let hint=document.getElementById('freq-entry-hint');
  if(inp) inp.classList.remove('error');
  if(hint){ hint.classList.remove('error'); hint.innerText='Enter MHz  e.g. 14.225 or 7.010'; }
  if(d==='.'&&freqEntryValue.indexOf('.')!==-1) return;
  if(freqEntryValue.replace('.','').length>=8) return;
  freqEntryValue+=d; if(inp) inp.value=freqEntryValue;
}
function freqEntryDelete() {
  freqEntryValue=freqEntryValue.slice(0,-1);
  let inp=document.getElementById('freq-entry-input');
  if(inp){ inp.value=freqEntryValue; inp.classList.remove('error'); }
}
function freqEntryError(msg) {
  let inp=document.getElementById('freq-entry-input');
  let hint=document.getElementById('freq-entry-hint');
  if(inp) inp.classList.add('error');
  if(hint){ hint.innerText='⚠ '+msg; hint.classList.add('error'); }
}
function freqEntryGo() {
  if(!freqEntryValue) return;
  let mhz=parseFloat(freqEntryValue);
  if(isNaN(mhz)){ freqEntryError('Invalid number'); return; }
  let hz=Math.round(mhz*1000000);
  if(!isWithinHamBand(hz)){
    freqEntryError('Out of band — 3.5-4 / 7-7.2 / 14-14.35 / 21-21.45 / 28-29.7 MHz'); return;
  }
  let bi=getBandIndexFromFreq(hz);
  if(vfoBActive){
    vfoBFreq=hz; if(bi>=0) vfoBMode=BAND_DEFAULT_MODES[bi]; sendVFOB();
  } else {
    operatingFreq=hz; if(bi>=0){ mode=BAND_DEFAULT_MODES[bi]; bandLastFreq[bi]=hz; } sendConfig(true);
    if(bi>=0) refreshBFO1ForMode();
  }
  updateUI(); closeFreqEntry();
  showMessage('✓ '+(hz/1000000).toFixed(6)+' MHz'+(bi>=0?' | '+BAND_DEFAULT_MODES[bi]:''),'#27ae60');
}

// ==================== EVENT LISTENERS ====================
function setupEventListeners() {
  // Mode
  let ml=document.getElementById('mode-lsb'); if(ml) ml.onclick=function(){ setMode("LSB"); };
  let mu=document.getElementById('mode-usb'); if(mu) mu.onclick=function(){ setMode("USB"); };

  // Step arrows
  let sd=document.getElementById('step-down'); if(sd) sd.onclick=function(){ changeStep(-1); };
  let su=document.getElementById('step-up');   if(su) su.onclick=function(){ changeStep(1);  };

  // Freq nudge
  let fm10=document.getElementById('freq-10');  if(fm10) fm10.onclick=function(){ adjustFrequency(-10); };
  let fm1 =document.getElementById('freq-1');   if(fm1)  fm1.onclick =function(){ adjustFrequency(-1);  };
  let fp1 =document.getElementById('freq+1');   if(fp1)  fp1.onclick =function(){ adjustFrequency(1);   };
  let fp10=document.getElementById('freq+10');  if(fp10) fp10.onclick=function(){ adjustFrequency(10);  };

  // RIT
  let rt=document.getElementById('rit-toggle'); if(rt) rt.onclick=function(){ toggleRIT(); };
  let rd=document.getElementById('rit-down');   if(rd) rd.onclick=function(){ adjustRIT(-100); };
  let ru=document.getElementById('rit-up');     if(ru) ru.onclick=function(){ adjustRIT(100);  };
  let rr=document.getElementById('rit-reset');  if(rr) rr.onclick=function(){ resetRIT(); };

  // Sensitivity slider
  let ss=document.getElementById('sensitivity-slider');
  if(ss) ss.oninput=function(){
    knobSensitivity=parseInt(this.value);
    let sv=document.getElementById('sensitivity-value'); if(sv) sv.innerText=knobSensitivity;
  };

  // Band buttons
  document.querySelectorAll('.band-btn').forEach(function(btn){
    btn.onclick=function(){ setBand(parseInt(btn.getAttribute('data-band'))); };
  });

  // Tab buttons
  document.querySelectorAll('.tab-btn').forEach(function(btn){
    btn.onclick=function(){ switchTab(btn.getAttribute('data-tab')); };
  });

  // BFO1
  let b1=[['bfo1-step-down',doBFO1StepDown],['bfo1-step-up',doBFO1StepUp],
    ['bfo1-step-cycle',doBFO1StepCycle],['bfo1-minus-10',function(){ adjustBFO1(-10); }],
    ['bfo1-minus-1',function(){ adjustBFO1(-1); }],['bfo1-plus-1',function(){ adjustBFO1(1); }],
    ['bfo1-plus-10',function(){ adjustBFO1(10); }],['bfo1-usb',doSetBFO1toUSB],
    ['bfo1-lsb',doSetBFO1toLSB],['bfo1-save',saveBFO1ToMemory]];
  b1.forEach(function(p){ let e=document.getElementById(p[0]); if(e) e.onclick=p[1]; });

  // BFO2
  let b2=[['bfo2-step-down',doBFO2StepDown],['bfo2-step-up',doBFO2StepUp],
    ['bfo2-step-cycle',doBFO2StepCycle],['bfo2-minus-10',function(){ adjustBFO2(-10); }],
    ['bfo2-minus-1',function(){ adjustBFO2(-1); }],['bfo2-plus-1',function(){ adjustBFO2(1); }],
    ['bfo2-plus-10',function(){ adjustBFO2(10); }],['bfo2-reset',doResetBFO2],
    ['bfo2-save',saveBFO2ToMemory]];
  b2.forEach(function(p){ let e=document.getElementById(p[0]); if(e) e.onclick=p[1]; });

  // PTT — latching single click, no mousedown/mouseup
  let pttBtn=document.getElementById('ptt-momentary');
  if(pttBtn){
    pttBtn.addEventListener('click',function(e){ e.preventDefault(); e.stopPropagation(); togglePTT(); });
    pttBtn.addEventListener('contextmenu',function(e){ e.preventDefault(); });
  }

  // Long-press display → direct freq entry
  let disp=document.getElementById('display'); let pressTimer;
  if(disp){
    disp.addEventListener('touchstart',function(){ pressTimer=setTimeout(openFreqEntry,LONG_PRESS_MS); },{passive:true});
    disp.addEventListener('touchend',  function(){ clearTimeout(pressTimer); });
    disp.addEventListener('touchmove', function(){ clearTimeout(pressTimer); });
    disp.addEventListener('mousedown', function(){ pressTimer=setTimeout(openFreqEntry,LONG_PRESS_MS); });
    disp.addEventListener('mouseup',   function(){ clearTimeout(pressTimer); });
  }

  // Knob
  let knob=document.getElementById('knob');
  if(knob){
    knob.addEventListener('mousedown',startDrag);
    knob.addEventListener('touchstart',startDrag,{passive:false});
    window.addEventListener('mouseup',stopDrag);
    window.addEventListener('touchend',stopDrag);
    window.addEventListener('mousemove',move);
    window.addEventListener('touchmove',move,{passive:false});
  }

  // Numpad for direct freq entry
  document.querySelectorAll('.numpad-btn[data-digit]').forEach(function(btn){
    btn.addEventListener('click',function(){ freqEntryDigit(btn.getAttribute('data-digit')); });
  });
  let nd=document.getElementById('numpad-del');         if(nd)  nd.addEventListener('click',freqEntryDelete);
  let ng=document.getElementById('freq-entry-go');      if(ng)  ng.addEventListener('click',freqEntryGo);
  let nc=document.getElementById('freq-entry-cancel');  if(nc)  nc.addEventListener('click',closeFreqEntry);
  let nov=document.getElementById('freq-entry-overlay');
  if(nov) nov.addEventListener('click',function(e){ if(e.target===nov) closeFreqEntry(); });
}

// ==================== INIT ====================
let sg=document.getElementById('s-grid');
if(sg){ for(let i=0;i<20;i++){ let s=document.createElement('div'); s.className='s-seg'; sg.appendChild(s); } }

setupEventListeners();
loadConfig();
setInterval(updateSMeter,250);
</script>
</body>
</html>
)rawliteral";
