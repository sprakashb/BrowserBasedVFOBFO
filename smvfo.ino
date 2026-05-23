// smartfone based Controller for Hamradio Experimenters
// Original idea and implimentation by Mirko Pavleski and detailed at
// https://www.hackstaer.io/mircemk/the-ultimate-smartphone-vfo-esp32-si5351-wireless-control-bc1fcd
// It was updated with the help of AI to provide 3 clocks which may be used as VFO/BFO etc
// Basically it provides control over all 3 clocks of Si5351 from the interface on Smartphone or any web browser .
// The starting values, limits etc can be contolled from within the program (some comments below with vu2spf tag may help)
// v1.0 14/5/2026 smctl4ubitx.ino
// v2.0 17/5/2026  tabbed display smctl4tabed.ino
// v3.0 19/5/2026  many updates and modifications in display and operating
// v4.0-5 Landscape display variety
// v4.6 21/5/2026  included a switch on GPIO 21, Hi-local AP, Lo -  Home Wifi
// v4.7 22/5/2026  mDNS problemin Android use fixedIP
// HOW to Use : upload the sketch after setting proper parameters as appropriate to ESP32 which is connected to Si5351 Module with I2C
// This program creates an accesspoint called ubitx (you may change it as per your liking in WIFI section below)
// Connect to this wifi from phone and open 192.168.4.1 to access the control of Si5351.
// Select  appropriate step size and change the VFO (Operating Fequency) using dial knob.( or + / - Buttons). VFO is output on Clock2 of Si5351
// There is provision to store current VFO on one of the 10 memory spaces. Long touch the frequency display to activate memory mode and
// touch the appropriate memory button
// For setting the other two clocks of Si5351 Use the Setup button. In ubitx terminology we may call them BFO1 and BFO2 (on Clock1 and Clock0 of Si53351)
// Both the frequencies are adjustable using step size and + / - buttons. To store the values in memory long touch the frequency display and
// touch one of the memory buttons.
// use one of 3 ways for Connection to ESP from web browser -
// method 1- wifi_switch connected to ESP32 is high - use local Accesspoint 
// AP mode : ESP32 is always 192.168.4.1  - bookmark http://192.168.4.1
// b. wifi_switch connected to ESP32 is Low - either mDNS or Fixed IP
// method 2 - setting FixedIP - uncomment "#define fixedIP" below - IP is 192.168.1.200
// method 3 - using mDNS - comment (put // before) "#define fixedIP" below

// uBITX v4/5/6 Style VFO - FIXED VFO MEMORY
// CLK2: VFO (45-75 MHz) - Main tuning
// CLK1: BFO1 (33 MHz LSB / 57 MHz USB) - Second IF mixer
// CLK0: BFO2 (12 MHz) - Audio demodulator

#include <WiFi.h>
#include <WebServer.h>
#include <si5351.h>
#include <Wire.h>
#include <EEPROM.h>
#include <ESPmDNS.h>

Si5351 si5351;

//=========== Attention - Match with your rig frequencies =============================================
// uBITX  Configuration
const unsigned long IF_FREQUENCY = 45000000;  // 45 MHz First IF  // vu2spf adjust as needed
const unsigned long BFO2_FREQ = 11998000 ;   //12000000;     // 12 MHz Audio demodulator (fixed)  // vu2spf  adjust as needed

// BFO1 frequencies for second IF mixer
const unsigned long BFO1_LSB = 32995000;  //33000000;      // 33 MHz for LSB
const unsigned long BFO1_USB = 56995000;  //57000000;      // 57 MHz for USB

// Frequency limits  //vu2spf- adjust these limits as needed
const unsigned long MIN_VFO_FREQ = 45000000;   // 45 MHz minimum VFO
const unsigned long MAX_VFO_FREQ = 75000000;   // 75 MHz maximum VFO
const unsigned long MIN_OPERATING_FREQ = 0;    // Operating = VFO - IF
const unsigned long MAX_OPERATING_FREQ = 30000000; // 30 MHz

// Clock configurations   //vu2spf adjust as needed
unsigned long vfoFreq = 52000000;        // VFO (CLK2) starts at 52MHz (7MHz operating)
unsigned long bfo1Freq = BFO1_USB;       // BFO1 (CLK1) starts at 33MHz USB
unsigned long bfo2Freq = BFO2_FREQ;      // BFO2 (CLK0) fixed at 12MHz

// WiFi =============== USER Credential for Home WiFi to be filled ======================
IPAddress IP;
int wifi_switch = 27;  // High= Local AP Low = Home router Wifi
const char* Local_ssid = "ubitx";                 //vu2spf  LOcal ESP32 Access Point Name and Passwd
const char* Local_password = "12345678";
const char* Home_ssid = "sirius";                 //vu2spf  Home Access Point Name and Passwd 
const char* Home_password = "c304cret";
//It was found that mDNS does not work in Android (cannot find ubitx.local)
//as its substitute we can use fixed IP when the wifi_switch is Low 
// ==================== FIXED IP ADDRESS CONFIGURATION ====================
#define fixedIP    // to use fixed ip address, commentit out if using mDNS
// AP mode : ESP32 is always 192.168.4.1  - bookmark http://192.168.4.1
// STA mode: uncomment and set static IP below to get a fixed address
//           then bookmark that IP - no mDNS/ubitx.local needed
//
#ifdef fixedIP
 IPAddress staticIP(192, 168, 1, 200);   // choose a free IP on your router
 IPAddress gateway(192, 168, 1, 1);      // your router's IP
 IPAddress subnet(255, 255, 255, 0);
 IPAddress dns(8, 8, 8, 8);
// -- in STA setup() call before WiFi.begin():
//    WiFi.config(staticIP, gateway, subnet, dns);
#endif
// ==================================================================
WebServer server(80);

// Step sizes
const long steps[] = {10, 100, 1000, 5000, 10000, 100000, 1000000, 10000000};
const char* stepLabels[] = {"10Hz", "100Hz", "1KHz", "5KHz", "10KHz", "100KHz", "1MHz", "10MHz"};
const int stepCount = 8;

// Mode enumeration
enum Mode {
  MODE_LSB,
  MODE_USB,
  MODE_AM,
  MODE_CW
};
Mode currentMode = MODE_USB;

// Step indices for each clock
int stepIndices[3] = {1, 2, 2};  // VFO=100Hz, BFO1=1KHz, BFO2=1KHz

// Memory slots [clock][slot] where clock: 0=VFO, 1=BFO1, 2=BFO2
// VFO stores the actual VFO frequency (45-75 MHz), not operating frequency
unsigned long memorySlots[3][10] = {{0}};

// EEPROM
const int EEPROM_SIZE = 1024;
const int MAGIC_NUMBER_ADDR = 0;
const int MAGIC_NUMBER = 0xABCD;    // change this to any other Hexadecimal numbe (like ABCE, ABCF etc) to Factory reset

//DELAYED EPROM WRITE
unsigned long lastChangeTime = 0;
bool saveNeeded = false;
const unsigned long SAVE_DELAY_MS = 20000; // save 20 seconds "after last change", To Reduce number of EPROM Writes

const int SMETER_PIN = 32; //ESP32 ADC pin - Limit the voltage to 1.2 V

// Function declarations
unsigned long getOperatingFrequency();
void setOperatingFrequency(unsigned long opFreq);
void updateAllClocks();
void saveToEEPROM();
void scheduleSave();
void loadFromEEPROM();
void setupSi5351Outputs();
int extractJsonValue(String json, String key, int index);

// HTML content as PROGMEM strings - // vu2spf This is for the main screen for VFO control
const char MAIN_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
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
    background: #000;
    font-family: 'Arial Black', Arial, sans-serif;
  }
  
  .container {
    max-width: 900px;
    margin: 0 auto;
    background: #E0AB07;
    border-radius: 16px;
    padding: 8px 10px;
    box-shadow: 0 5px 20px rgba(0,0,0,0.5);
  }
  
  /* Header */
  .header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-bottom: 6px;
  }
  
  .title {
    font-size: 16px;
    font-weight: bold;
    color: #fff;
    text-shadow: 2px 2px 4px #000;
  }

  .ip-display {
    background: rgba(0,0,0,0.4);
    color: #2ecc71;
    font-size: 11px;
    font-weight: bold;
    padding: 3px 8px;
    border-radius: 8px;
    border: 1px solid #2ecc71;
    font-family: 'Courier New', monospace;
  }
  .setup-btn {
    background: #8e44ad;
    padding: 4px 10px;
    border-radius: 8px;
    color: white;
    font-weight: bold;
    cursor: pointer;
    border: 2px solid #f1c40f;
    font-size: 11px;
  }
  
  .setup-btn:active {
    transform: scale(0.95);
  }
  
  /* Tab Bar */
  .tab-container {
    display: flex;
    gap: 5px;
    margin-bottom: 8px;
    background: rgba(0,0,0,0.3);
    border-radius: 10px;
    padding: 3px;
  }
  
  .tab-btn {
    flex: 1;
    padding: 6px;
    background: #555;
    border: none;
    border-radius: 8px;
    color: white;
    font-weight: bold;
    cursor: pointer;
    text-align: center;
    font-size: 11px;
    transition: all 0.2s;
  }
  
  .tab-btn.active {
    background: #e74c3c;
    transform: scale(0.98);
  }
  
  .tab-btn:active {
    transform: scale(0.95);
  }
  
  /* Tab Content */
  .tab-content {
    display: none;
  }
  
  .tab-content.active {
    display: block;
  }
  
  /* Display */
  .display {
    background: #0077c2;
    border: 3px solid #111;
    border-radius: 12px;
    padding: 8px 10px;
    margin-bottom: 6px;
    box-shadow: inset 0 0 15px rgba(0,0,0,0.5);
  }
  
  .display.memory-mode {
    background: #e67e22;
  }
  
  .display-info {
    display: flex;
    justify-content: space-between;
    margin-bottom: 4px;
    font-size: 11px;
    color: white;
  }
  
  #f-display {
    font-size: 34px;
    text-align: center;
    font-family: 'Courier New', monospace;
    font-weight: 900;
    margin: 4px 0;
    color: white;
    text-shadow: 2px 2px 4px #000;
    letter-spacing: 2px;
  }
  
  .display-footer {
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-top: 4px;
    font-size: 11px;
    color: white;
  }
  
  /* Mode Selector */
  .mode-selector {
    display: flex;
    gap: 5px;
    margin: 6px 0;
  }
  
  .mode-btn {
    flex: 1;
    padding: 6px;
    background: #2980b9;
    border: 2px solid #f1c40f;
    border-radius: 8px;
    text-align: center;
    cursor: pointer;
    color: white;
    font-weight: bold;
    font-size: 11px;
  }
  
  .mode-btn.active {
    background: #e74c3c;
    transform: scale(0.98);
  }
  
  /* Knob */
  .knob-container {
    display: flex;
    justify-content: center;
    margin: 8px 0;
  }
  
  .knob {
    width: 140px;
    height: 140px;
    background: conic-gradient(from 0deg, #444, #888 25%, #444 50%, #888 75%, #444);
    border-radius: 50%;
    border: 8px solid #1a1a1a;
    cursor: pointer;
    box-shadow: 0 10px 20px rgba(0,0,0,0.3);
    position: relative;
    will-change: transform;
  }
  
  .knob::after {
    content: '';
    position: absolute;
    top: 16px;
    left: 50%;
    width: 14px;
    height: 14px;
    background: #111;
    border-radius: 50%;
    transform: translateX(-50%);
  }
 /* Vertical Slider Styles */
.knob-wrapper {
  display: flex;
  gap: 12px;
  align-items: center;
  justify-content: center;
  margin: 8px 0;
}

.vslider-container {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  background: rgba(0, 0, 0, 0.3);
  border-radius: 16px;
  padding: 8px 6px;
  width: 55px;
  height: 140px;
}

.vslider-label {
  font-size: 9px;
  font-weight: bold;
  text-align: center;
}

.vslider-label.slow {
  color: #2ecc71;
}

.vslider-label.fast {
  color: #e74c3c;
}

.vslider-track {
  flex: 1;
  display: flex;
  justify-content: center;
  align-items: center;
  margin: 4px 0;
}

/* Actual vertical slider */
.vslider {
  -webkit-appearance: slider-vertical;
  appearance: slider-vertical;
  width: 22px;
  height: 80px;
  background: linear-gradient(to top, #e74c3c, #f1c40f, #2ecc71);
  border-radius: 12px;
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
  box-shadow: 0 2px 5px rgba(0,0,0,0.3);
}

.vslider::-webkit-slider-thumb:hover {
  transform: scale(1.1);
}

.vslider-value {
  background: #000;
  padding: 2px 6px;
  border-radius: 10px;
  font-size: 10px;
  font-weight: bold;
  color: #f1c40f;
  text-align: center;
  min-width: 30px;
}

/* For Firefox */
.vslider {
  writing-mode: vertical-lr;
  direction: rtl;
}

  /* Buttons */
  .btn-group {
    display: flex;
    gap: 5px;
    margin: 5px 0;
    flex-wrap: wrap;
  }
  
  .btn {
    flex: 1;
    padding: 6px;
    background: #555;
    border: 2px solid #f1c40f;
    border-radius: 8px;
    color: white;
    font-weight: bold;
    text-align: center;
    cursor: pointer;
    transition: transform 0.05s;
    min-width: 50px;
    font-size: 11px;
  }
  
  .btn:active {
    transform: translateY(2px);
  }
  
  .btn-primary {
    background: #27ae60;
  }
  
  .info-text {
    background: rgba(0,0,0,0.5);
    padding: 4px;
    border-radius: 5px;
    font-size: 9px;
    text-align: center;
    margin: 4px 0;
    color: #ddd;
  }
  
  /* Band Buttons */
  .band-buttons {
    display: grid;
    grid-template-columns: repeat(4, 1fr);
    gap: 5px;
    margin: 6px 0;
  }
  
  .band-btn {
    background: #7f0000;
    padding: 6px;
    text-align: center;
    border-radius: 6px;
    color: white;
    font-weight: bold;
    cursor: pointer;
    border: 2px solid #f1c40f;
    font-size: 11px;
  }
  
  .band-btn:active {
    transform: scale(0.95);
  }
  
  /* BFO Cards */
  .clock-card {
    background: #34495e;
    border-radius: 10px;
    padding: 8px 10px;
    margin-bottom: 8px;
    border: 2px solid #f1c40f;
  }
  
  .clock-title {
    font-size: 12px;
    font-weight: bold;
    color: #f1c40f;
    margin-bottom: 6px;
    text-align: center;
  }
  
  .clock-freq {
    font-size: 22px;
    font-family: 'Courier New', monospace;
    color: white;
    text-align: center;
    margin: 5px 0;
    padding: 6px;
    background: rgba(0,0,0,0.3);
    border-radius: 8px;
    cursor: pointer;
    transition: background 0.2s;
  }
  
  .clock-freq:active {
    background: rgba(0,0,0,0.5);
  }
  
  .clock-freq.memory-mode {
    background: #e67e22;
  }
  
  .clock-info {
    font-size: 9px;
    color: #bdc3c7;
    text-align: center;
    margin-bottom: 6px;
  }
  
  /* Memory Panel */
  .mem-panel {
    background: rgba(0,0,0,0.7);
    border-radius: 8px;
    padding: 8px;
    margin: 6px 0;
  }
  
  .mem-header {
    display: flex;
    justify-content: space-between;
    margin-bottom: 6px;
    color: white;
    font-size: 11px;
  }
  
  .mem-buttons {
    display: grid;
    grid-template-columns: repeat(5, 1fr);
    gap: 4px;
  }
  
  .mem-btn {
    background: #8e44ad;
    padding: 4px;
    text-align: center;
    border-radius: 5px;
    color: white;
    font-size: 9px;
    cursor: pointer;
    border: 1px solid #f1c40f;
  }
  
  .mem-btn:active {
    transform: translateY(2px);
  }
  
  .memory-hint {
    font-size: 8px;
    color: #f1c40f;
    text-align: center;
    margin-top: 4px;
  }
  
  /* S-Meter */
  .s-meter {
    background: #000;
    padding: 4px;
    border-radius: 6px;
    margin: 5px 0;
  }
  
  .s-grid {
    display: flex;
    gap: 2px;
    height: 12px;
  }
  
  .s-seg {
    flex: 1;
    background: #222;
    transition: background 0.05s;
  }
  
  .s-seg.on {
    background: #00ff00;
    box-shadow: 0 0 4px #00ff00;
  }
  
  .signature {
    text-align: center;
    color: #888;
    font-size: 8px;
    margin-top: 6px;
  }
  
  /* Landscape two-column VFO layout */
  .vfo-landscape {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 10px;
    align-items: start;
  }
  
  /* BFO two-column layout */
  .bfo-landscape {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 10px;
    align-items: start;
  }
  
  /* Memory three-column layout */
  .memory-landscape {
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
    gap: 8px;
  }
  
  @media (max-width: 600px) {
    #f-display { font-size: 26px; }
    .knob { width: 110px; height: 110px; }
    .clock-freq { font-size: 18px; }
    .vfo-landscape { grid-template-columns: 1fr; }
    .bfo-landscape { grid-template-columns: 1fr; }
    .memory-landscape {
      grid-template-columns: 1fr;
    }
    .memory-landscape .mem-buttons {
      grid-template-columns: repeat(10, 1fr);
      gap: 3px;
    }
    .memory-landscape .mem-btn {
      padding: 6px 2px;
      font-size: 9px;
    }
  }
</style>
</head>
<body>
<div class="container">
  <div class="header">
    <div class="title">uBITX VFO</div>
        <div class="ip-display">📶 %%IP%%</div>
<!--    <div class="setup-btn" onclick="showSetup()">⚙️</div> -->
  </div>
  
  <!-- Tab Bar -->
  <div class="tab-container">
    <div class="tab-btn active" data-tab="vfo">📻 VFO</div>
    <div class="tab-btn" data-tab="bfo">🔧 BFO</div>
    <div class="tab-btn" data-tab="memory">💾 Memory</div>
  </div>
  
  <!-- ==================== VFO TAB ==================== -->
  <div id="vfo-tab" class="tab-content active">
    <div class="vfo-landscape">
      <!-- LEFT COLUMN: Display, S-Meter, Mode, Band -->
      <div>
        <div class="display" id="display">
          <div class="display-info">
            <span id="mode-label">USB</span>
            <span id="memory-mode-label" style="background:#ff0;color:#000;padding:2px 6px;border-radius:4px;display:none;">MEMORY MODE</span>
            <span id="step-label">Step: 100Hz</span>
          </div>
          <div id="f-display">07.000.000</div>
        </div>
        
        <div class="s-meter">
          <div class="s-grid" id="s-grid"></div>
        </div>
        
        <div class="mode-selector">
          <div class="mode-btn" id="mode-lsb">LSB</div>
          <div class="mode-btn" id="mode-usb">USB</div>
          <div class="mode-btn" id="mode-am">AM</div>
          <div class="mode-btn" id="mode-cw">CW</div>
        </div>
        
        <div class="band-buttons">
          <div class="band-btn" data-band="7000000">40M</div>
          <div class="band-btn" data-band="14000000">20M</div>
          <div class="band-btn" data-band="21000000">15M</div>
          <div class="band-btn" data-band="28000000">10M</div>
        </div>
        
        <div class="info-text">Long press frequency display to save VFO to memory</div>
      </div>
      
      <!-- RIGHT COLUMN: Knob, Step, Freq buttons -->
      <div>
        <div class="knob-wrapper" style="display: flex; gap: 12px; align-items: center; justify-content: center;">
          <!-- Vertical Slider Container -->
          <div class="vslider-container">
            <div class="vslider-label slow">SLOW</div>
            <div class="vslider-track">
              <input type="range" id="sensitivity-slider" class="vslider" min="2" max="10" value="6" step="1" orient="vertical">
            </div>
            <div class="vslider-label fast">FAST</div>
            <div class="vslider-value" id="sensitivity-value">6</div>
          </div>
          <!-- Knob -->
          <div class="knob" id="knob"></div>
        </div>
        
        <div class="btn-group">
          <div class="btn btn-primary" id="step-down">◀ Step</div>
          <div class="btn btn-primary" id="step-cycle">Step Cycle</div>
          <div class="btn btn-primary" id="step-up">Step ▶</div>
        </div>
        
        <div class="btn-group">
          <div class="btn" id="freq-10">-10</div>
          <div class="btn" id="freq-1">-1</div>
          <div class="btn" id="freq+1">+1</div>
          <div class="btn" id="freq+10">+10</div>
        </div>
      </div>
    </div>
  </div>
  
 <!-- ==================== BFO TAB ==================== -->
<div id="bfo-tab" class="tab-content">
  <div class="bfo-landscape">
  <div class="clock-card">
    <div class="clock-title">🔧 BFO1 (CLK1) - Second IF Mixer</div>
    <div class="clock-freq" id="bfo1-display-main">BFO1_LSB</div>
    <div class="clock-info">LSB: 33 MHz | USB: 57 MHz</div>
    <div class="step-group">
      <div class="step-label" style="color:white; font-size:11px; text-align:center;">Step: <span id="bfo1-step-main" style="color:#f1c40f;">1KHz</span></div>
      <div class="btn-group">
        <div class="btn" onclick="bfo1StepDownMain()">◀</div>
        <div class="btn btn-primary" onclick="bfo1StepCycleMain()">Cycle</div>
        <div class="btn" onclick="bfo1StepUpMain()">▶</div>
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
      <div class="btn" style="background:#8e44ad;" onclick="saveBFO1ToMemory()">💾 Save</div>
    </div>
    <div class="info-text" style="margin-top:5px;">Tap Save to store current BFO1 frequency</div>
  </div>
  
  <div class="clock-card">
    <div class="clock-title">🔧 BFO2 (CLK0) - Audio Demodulator</div>
    <div class="clock-freq" id="bfo2-display-main">12.000000</div>
    <div class="clock-info">Fixed 12 MHz Carrier</div>
    <div class="step-group">
      <div class="step-label" style="color:white; font-size:11px; text-align:center;">Step: <span id="bfo2-step-main" style="color:#f1c40f;">1KHz</span></div>
      <div class="btn-group">
        <div class="btn" onclick="bfo2StepDownMain()">◀</div>
        <div class="btn btn-primary" onclick="bfo2StepCycleMain()">Cycle</div>
        <div class="btn" onclick="bfo2StepUpMain()">▶</div>
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
      <div class="btn" style="background:#8e44ad;" onclick="saveBFO2ToMemory()">💾 Save</div>
    </div>
    <div class="info-text" style="margin-top:5px;">Tap Save to store current BFO2 frequency</div>
  </div>
  </div>
</div> 
  <!-- ==================== MEMORY TAB ==================== -->
  <div id="memory-tab" class="tab-content">
    <div class="memory-landscape">
    <div class="mem-panel" style="margin-top:0">
      <div class="mem-header">📀 VFO Memories</div>
      <div class="mem-buttons" id="mem-buttons-tab"></div>
      <div class="memory-hint">Tap to recall | Long press VFO display to save</div>
    </div>
    
    <div class="mem-panel">
      <div class="mem-header">📀 BFO1 Memories</div>
      <div class="mem-buttons" id="bfo1-mem-buttons-tab"></div>
      <div class="memory-hint">Tap to recall | Long press BFO1 frequency to save</div>
    </div>
    
    <div class="mem-panel">
      <div class="mem-header">📀 BFO2 Memories</div>
      <div class="mem-buttons" id="bfo2-mem-buttons-tab"></div>
      <div class="memory-hint">Tap to recall | Long press BFO2 frequency to save</div>
    </div>
    </div>
  </div>
  
  <div class="signature">uBITX VFO Controller | 3-Clock Control</div>
</div>

<script>
  // ==================== CONSTANTS ====================
  const IF_FREQ = 45000000;
  const MIN_VFO = 45000000;
  const MAX_VFO = 75000000;
  const MIN_OP = 0;
  const MAX_OP = 30000000;
  
  const steps = [10, 100, 1000, 5000, 10000, 100000, 1000000, 10000000];
  const stepLabels = ["10Hz", "100Hz", "1KHz", "5KHz", "10KHz", "100KHz", "1MHz", "10MHz"];
  
  const bfoSteps = [10, 100, 1000, 5000, 10000, 100000, 1000000];
  const bfoStepLabels = ["10Hz", "100Hz", "1KHz", "5KHz", "10KHz", "100KHz", "1MHz"];
  
  // ==================== GLOBAL STATE ====================
  let vfoFreq = 52000000;
  let mode = "USB";
  let stepIdx = 1;
  let memories = [[],[],[]];
  let memoryMode = false;

  let BFO1_USB = 33000000;     // Will be updated from Arduino
  let BFO1_LSB = 57000000;     // Will be updated from Arduino    
  let BFO2_DEFAULT = 12000000; // Will be updated from Arduino
  let IF_FREQ_CONFIG = 45000000; // Will be updated from Arduino

  let bfo1Freq = 33000000;
  let bfo2Freq = 12000000;
  let bfo1StepIdx = 2;
  let bfo2StepIdx = 2;
  let bfo1Memories = new Array(10).fill(0);
  let bfo2Memories = new Array(10).fill(0);
  let bfo1MemoryMode = false;
  let bfo2MemoryMode = false;
  
  // Knob state
  let isDragging = false;
  let lastAngle = 0;
  let currentRotation = 0;
  let lastSendTime = 0;

  // Sensitivity variable (default 4)
  let knobSensitivity = 4;
  let KNOB_MAX_STEPS = 3;        // Maximum steps per event (prevents huge jumps)

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
    if(newVFO >= MIN_VFO && newVFO <= MAX_VFO) {
      vfoFreq = newVFO;
    }
  }
  
  function formatFrequency(freq) {
    let opFreq = getOperatingFreq();
    if(opFreq >= 1000000) {
      return (opFreq/1000000).toFixed(6) + " MHz";
    } else {
      return (opFreq/1000).toFixed(3) + " kHz";
    }
  }
  
  function formatBFOfreq(freq) {
    if (freq >= 1000000) {
        return (freq/1000000).toFixed(6);
    } else {
        return (freq/1000).toFixed(3) + " kHz";
    }
}
  // ==================== TAB SWITCHING ====================
  function switchTab(tabName) {
    document.querySelectorAll('.tab-content').forEach(tab => {
      tab.classList.remove('active');
    });
    document.querySelectorAll('.tab-btn').forEach(btn => {
      btn.classList.remove('active');
    });
    
    document.getElementById(tabName + '-tab').classList.add('active');
    document.querySelector(`.tab-btn[data-tab="${tabName}"]`).classList.add('active');
    
    // Refresh memory buttons when switching to memory tab
    if(tabName === 'memory') {
      updateAllMemoryButtons();
    }
  }
  
  document.querySelectorAll('.tab-btn').forEach(btn => {
    btn.onclick = () => switchTab(btn.getAttribute('data-tab'));
  });
  
  // ==================== VFO UI FUNCTIONS ====================
  function updateUI() {
    document.getElementById('f-display').innerText = formatFrequency();
    document.getElementById('step-label').innerText = "Step: " + stepLabels[stepIdx];
    
    document.querySelectorAll('.mode-btn').forEach(btn => btn.classList.remove('active'));
    if(mode === "LSB") document.getElementById('mode-lsb').classList.add('active');
    else if(mode === "USB") document.getElementById('mode-usb').classList.add('active');
    else if(mode === "AM") document.getElementById('mode-am').classList.add('active');
    else if(mode === "CW") document.getElementById('mode-cw').classList.add('active');
    
    document.getElementById('mode-label').innerText = mode;
    
    let displayDiv = document.getElementById('display');
    if(memoryMode) {
      displayDiv.classList.add('memory-mode');
    } else {
      displayDiv.classList.remove('memory-mode');
    }
    document.getElementById('memory-mode-label').style.display = memoryMode ? "inline-block" : "none";
    
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
          let displayVal = (opMem/1000000).toFixed(3) + "M";
          btn.innerHTML = `M${i+1}<br><span style="font-size:8px;">${displayVal}</span>`;
        } else {
          btn.innerHTML = `M${i+1}<br><span style="font-size:8px;">Empty</span>`;
        }
        btn.onclick = (function(slot) {
          return function() { handleVFOMemory(slot); };
        })(i);
        container.appendChild(btn);
      }
    }
  }
  
  function saveVFOMemory() {
    window.pendingSave = {
        type: 'vfo',
        frequency: vfoFreq
    };
    switchTab('memory');
    
    let vfoPanel = document.querySelector('#memory-tab .mem-panel:first-child');
    if (vfoPanel) {
        vfoPanel.style.border = '2px solid #f1c40f';
        vfoPanel.style.boxShadow = '0 0 10px #f1c40f';
        setTimeout(() => {
            if (vfoPanel) {
                vfoPanel.style.border = '';
                vfoPanel.style.boxShadow = '';
            }
        }, 2000);
    }
    
    showMemorySaveHint('VFO');
    
    setTimeout(() => {
        if (window.pendingSave && window.pendingSave.type === 'vfo') {
            window.pendingSave = null;
            showMessage('VFO save cancelled', '#e74c3c');
        }
    }, 10000);
}
  function handleVFOMemory(slot) {
    if(memoryMode) {
      memories[0][slot] = vfoFreq;
      updateVFOMemoryButtons();
      exitMemoryMode();
      sendConfig();
      let display = document.getElementById('display');
      display.style.backgroundColor = '#27ae60';
      setTimeout(() => display.style.backgroundColor = '', 500);
    } else {
      let saved = memories[0][slot];
      if(saved && saved > 0) {
        vfoFreq = saved;
        updateUI();
        sendConfig();
        let display = document.getElementById('display');
        display.style.backgroundColor = '#e67e22';
        setTimeout(() => display.style.backgroundColor = '', 200);
      }
    }
  }
  
function enterMemoryMode() {
    memoryMode = true;
    updateUI();
    
    // Automatically switch to Memory tab
    switchTab('memory');
    
    // Auto-exit after 5 seconds
    setTimeout(() => {
        if(memoryMode) {
            exitMemoryMode();
        }
    }, 5000);
}
  
  function exitMemoryMode() {
    memoryMode = false;
    updateUI();
  }
  
  // ==================== BFO UI FUNCTIONS ====================
  function updateBFO1MainUI() {
    let displayFreq = formatBFOfreq(bfo1Freq);
    let unit = (bfo1Freq >= 1000000) ? " MHz" : " kHz";
    document.getElementById('bfo1-display-main').innerHTML = displayFreq + unit;
    document.getElementById('bfo1-step-main').innerText = bfoStepLabels[bfo1StepIdx];
}
   
  function updateBFO2MainUI() {
    document.getElementById('bfo2-display-main').innerText = formatBFOfreq(bfo2Freq);
    document.getElementById('bfo2-step-main').innerText = bfoStepLabels[bfo2StepIdx];
    if(bfo2MemoryMode) {
      document.getElementById('bfo2-display-main').classList.add('memory-mode');
    } else {
      document.getElementById('bfo2-display-main').classList.remove('memory-mode');
    }
  }
  
  function updateAllMemoryButtons() {
    // BFO1 Memory Buttons
    let container1 = document.getElementById('bfo1-mem-buttons-tab');
    if(container1) {
      container1.innerHTML = '';
      for(let i = 0; i < 10; i++) {
        let btn = document.createElement('div');
        btn.className = 'mem-btn';
        let memVal = bfo1Memories[i];
        if(memVal && memVal > 0) {
          btn.innerHTML = `M${i+1}<br><span style="font-size:8px;">${(memVal/1000000).toFixed(3)}M</span>`;
        } else {
          btn.innerHTML = `M${i+1}<br><span style="font-size:8px;">Empty</span>`;
        }
        btn.onclick = (function(slot) {
          return function() { handleBFO1MemoryRecall(slot); };
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
          btn.innerHTML = `M${i+1}<br><span style="font-size:8px;">${(memVal/1000000).toFixed(3)}M</span>`;
        } else {
          btn.innerHTML = `M${i+1}<br><span style="font-size:8px;">Empty</span>`;
        }
        btn.onclick = (function(slot) {
          return function() { handleBFO2MemoryRecall(slot); };
        })(i);
        container2.appendChild(btn);
      }
    }
  }
  
  function handleBFO1MemoryRecall(slot) {
    let saved = bfo1Memories[slot];
    if(saved && saved > 0) {
      bfo1Freq = saved;
      updateBFO1MainUI();
      sendSetupConfig();
    }
  }
  
  function handleBFO2MemoryRecall(slot) {
    let saved = bfo2Memories[slot];
    if(saved && saved > 0) {
      bfo2Freq = saved;
      updateBFO2MainUI();
      sendSetupConfig();
    }
  }
  
  function enterBFO1MemoryMode() {
    bfo1MemoryMode = true;
    updateBFO1MainUI();
    setTimeout(() => {
      if(bfo1MemoryMode) {
        bfo1MemoryMode = false;
        updateBFO1MainUI();
      }
    }, 5000);
  }
  
  function enterBFO2MemoryMode() {
    bfo2MemoryMode = true;
    updateBFO2MainUI();
    setTimeout(() => {
      if(bfo2MemoryMode) {
        bfo2MemoryMode = false;
        updateBFO2MainUI();
      }
    }, 5000);
  }
  
  function adjustBFO1Main(delta) {
    let step = bfoSteps[bfo1StepIdx];
    let newFreq = bfo1Freq + (delta * step);
    if(newFreq >= 1000000 && newFreq <= 60000000) {
      bfo1Freq = newFreq;
      updateBFO1MainUI();
      sendSetupConfig();
    }
  }
  
  function adjustBFO2Main(delta) {
    let step = bfoSteps[bfo2StepIdx];
    let newFreq = bfo2Freq + (delta * step);
    if(newFreq >= 1000000 && newFreq <= 60000000) {
      bfo2Freq = newFreq;
      updateBFO2MainUI();
      sendSetupConfig();
    }
  }
  
  function bfo1StepDownMain() {
    if(bfo1StepIdx > 0) {
      bfo1StepIdx--;
      updateBFO1MainUI();
      sendSetupConfig();
    }
  }
  
  function bfo1StepUpMain() {
    if(bfo1StepIdx < bfoSteps.length - 1) {
      bfo1StepIdx++;
      updateBFO1MainUI();
      sendSetupConfig();
    }
  }
  
  function bfo1StepCycleMain() {
    bfo1StepIdx = (bfo1StepIdx + 1) % bfoSteps.length;
    updateBFO1MainUI();
    sendSetupConfig();
  }
  
  function bfo2StepDownMain() {
    if(bfo2StepIdx > 0) {
      bfo2StepIdx--;
      updateBFO2MainUI();
      sendSetupConfig();
    }
  }
  
  function bfo2StepUpMain() {
    if(bfo2StepIdx < bfoSteps.length - 1) {
      bfo2StepIdx++;
      updateBFO2MainUI();
      sendSetupConfig();
    }
  }
  
  function bfo2StepCycleMain() {
    bfo2StepIdx = (bfo2StepIdx + 1) % bfoSteps.length;
    updateBFO2MainUI();
    sendSetupConfig();
  }

function setBFO1toUSBMain() {
    bfo1Freq = BFO1_USB;
    updateBFO1MainUI();
    sendSetupConfig();
    showMessage('BFO1 set to ' + (BFO1_USB/1000000).toFixed(3) + ' MHz (USB)', '#27ae60');
}

function setBFO1toLSBMain() {
    bfo1Freq = BFO1_LSB;
    updateBFO1MainUI();
    sendSetupConfig();
    showMessage('BFO1 set to ' + (BFO1_LSB/1000000).toFixed(3) + ' MHz (LSB)', '#27ae60');
}

function resetBFO2Main() {
    bfo2Freq = BFO2_DEFAULT;
    updateBFO2MainUI();
    sendSetupConfig();
    showMessage('BFO2 reset to ' + (BFO2_DEFAULT/1000000).toFixed(3) + ' MHz', '#27ae60');
}


  //spb1
  // ==================== BFO MEMORY SAVING FUNCTIONS ====================
function saveBFO1ToMemory() {
    // First, store which BFO we're saving (for highlighting)
    window.savingBFO = 'bfo1';
    
    // Switch to Memory tab
    switchTab('memory');
    
    // Highlight BFO1 memory panel
    let bfo1Panel = document.querySelector('#memory-tab .mem-panel:nth-child(2)');
    if (bfo1Panel) {
        bfo1Panel.style.transition = 'all 0.3s';
        bfo1Panel.style.border = '2px solid #f1c40f';
        bfo1Panel.style.boxShadow = '0 0 10px #f1c40f';
        setTimeout(() => {
            if (bfo1Panel) {
                bfo1Panel.style.border = '';
                bfo1Panel.style.boxShadow = '';
            }
        }, 2000);
    }
    
    // Show instruction message
    showMemorySaveHint('BFO1');
    
    // Store the frequency to save when user taps a memory button
    window.pendingSave = {
        type: 'bfo1',
        frequency: bfo1Freq
    };
    
    // Auto-cancel after 10 seconds
    setTimeout(() => {
        if (window.pendingSave && window.pendingSave.type === 'bfo1') {
            window.pendingSave = null;
            showMessage('BFO1 save cancelled', '#e74c3c');
        }
    }, 10000);
}

function saveBFO2ToMemory() {
    // First, store which BFO we're saving
    window.savingBFO = 'bfo2';
    
    // Switch to Memory tab
    switchTab('memory');
    
    // Highlight BFO2 memory panel
    let bfo2Panel = document.querySelector('#memory-tab .mem-panel:nth-child(3)');
    if (bfo2Panel) {
        bfo2Panel.style.transition = 'all 0.3s';
        bfo2Panel.style.border = '2px solid #f1c40f';
        bfo2Panel.style.boxShadow = '0 0 10px #f1c40f';
        setTimeout(() => {
            if (bfo2Panel) {
                bfo2Panel.style.border = '';
                bfo2Panel.style.boxShadow = '';
            }
        }, 2000);
    }
    
    // Show instruction message
    showMemorySaveHint('BFO2');
    
    // Store the frequency to save when user taps a memory button
    window.pendingSave = {
        type: 'bfo2',
        frequency: bfo2Freq
    };
    
    // Auto-cancel after 10 seconds
    setTimeout(() => {
        if (window.pendingSave && window.pendingSave.type === 'bfo2') {
            window.pendingSave = null;
            showMessage('BFO2 save cancelled', '#e74c3c');
        }
    }, 10000);
}
function showMemorySaveHint(bfoname) {
    let hint = document.createElement('div');
    hint.innerHTML = `📀 Tap any ${bfoname} memory button to save current frequency`;
    hint.style.position = 'fixed';
    hint.style.bottom = '20px';
    hint.style.left = '50%';
    hint.style.transform = 'translateX(-50%)';
    hint.style.background = '#f1c40f';
    hint.style.color = '#000';
    hint.style.padding = '10px 20px';
    hint.style.borderRadius = '25px';
    hint.style.fontSize = '12px';
    hint.style.fontWeight = 'bold';
    hint.style.zIndex = '1000';
    hint.style.whiteSpace = 'nowrap';
    hint.style.boxShadow = '0 2px 10px rgba(0,0,0,0.3)';
    document.body.appendChild(hint);
    
    setTimeout(() => {
        if (hint) hint.remove();
    }, 4000);
}

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
    msg.style.whiteSpace = 'nowrap';
    msg.style.boxShadow = '0 2px 10px rgba(0,0,0,0.3)';
    document.body.appendChild(msg);
    
    setTimeout(() => {
        if (msg) msg.remove();
    }, 2000);
}


// Updated BFO memory recall with better feedback
function handleBFO1MemoryRecall(slot) {
    // Check if we're in save mode (pending save from BFO1 button)
    if (window.pendingSave && window.pendingSave.type === 'bfo1') {
        // Save mode - store the pending frequency
        bfo1Memories[slot] = window.pendingSave.frequency;
        updateAllMemoryButtons();
        sendSetupConfig();
        
        // Clear pending save
        window.pendingSave = null;
        
        // Show success feedback
        showMessage('BFO1 saved to memory ' + (slot + 1), '#27ae60');
        
        // Flash the saved button
        let buttons = document.querySelectorAll('#bfo1-mem-buttons-tab .mem-btn');
        if (buttons[slot]) {
            buttons[slot].style.backgroundColor = '#27ae60';
            setTimeout(() => {
                if (buttons[slot]) buttons[slot].style.backgroundColor = '';
            }, 500);
        }
    } else {
        // Normal recall mode
        let saved = bfo1Memories[slot];
        if (saved && saved > 0) {
            bfo1Freq = saved;
            updateBFO1MainUI();
            sendSetupConfig();
            showMessage('BFO1 recalled from memory ' + (slot + 1), '#e67e22');
        } else {
            showMessage('Memory ' + (slot + 1) + ' is empty!', '#e74c3c');
        }
    }
}

function handleBFO2MemoryRecall(slot) {
    // Check if we're in save mode (pending save from BFO2 button)
    if (window.pendingSave && window.pendingSave.type === 'bfo2') {
        // Save mode - store the pending frequency
        bfo2Memories[slot] = window.pendingSave.frequency;
        updateAllMemoryButtons();
        sendSetupConfig();
        
        // Clear pending save
        window.pendingSave = null;
        
        // Show success feedback
        showMessage('BFO2 saved to memory ' + (slot + 1), '#27ae60');
        
        // Flash the saved button
        let buttons = document.querySelectorAll('#bfo2-mem-buttons-tab .mem-btn');
        if (buttons[slot]) {
            buttons[slot].style.backgroundColor = '#27ae60';
            setTimeout(() => {
                if (buttons[slot]) buttons[slot].style.backgroundColor = '';
            }, 500);
        }
    } else {
        // Normal recall mode
        let saved = bfo2Memories[slot];
        if (saved && saved > 0) {
            bfo2Freq = saved;
            updateBFO2MainUI();
            sendSetupConfig();
            showMessage('BFO2 recalled from memory ' + (slot + 1), '#e67e22');
        } else {
            showMessage('Memory ' + (slot + 1) + ' is empty!', '#e74c3c');
        }
    }
}

// Add Save buttons to BFO tab - update the BFO tab HTML
// Replace the BFO tab HTML section with this:
  // ==================== VFO ACTIONS ====================
function setMode(newMode) {
    mode = newMode;
    
    // Update BFO1 based on mode using values from Arduino
    if (mode === "USB") {
        bfo1Freq = BFO1_USB;
        updateBFO1MainUI();
        sendSetupConfig();
    } else if (mode === "LSB") {
        bfo1Freq = BFO1_LSB;
        updateBFO1MainUI();
        sendSetupConfig();
    }
    // AM and CW keep current BFO1 setting
    
    updateUI();
    sendConfig();
}

  function adjustFrequency(delta) {
    let step = steps[stepIdx];
    let newOpFreq = getOperatingFreq() + (delta * step);
    if(newOpFreq >= MIN_OP && newOpFreq <= MAX_OP) {
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
  
  function setBand(freq) {
    setOperatingFreq(freq);
    updateUI();
    sendConfig();
  }
  
  function showSetup() {
    window.location.href = '/setup';
  }
  
  // ==================== SERVER COMMUNICATION ====================
  function sendConfig() {
    let now = Date.now();
    if(now - lastSendTime > 50) {
      let data = {
        vfoFreq: vfoFreq,
        mode: mode,
        stepIdx: stepIdx,
        memories: memories
      };
      fetch('/setConfig', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(data)
      }).catch(e => console.log('Send error:', e));
      lastSendTime = now;
    }
  }
  
  function sendSetupConfig() {
    let data = {
      bfo1Freq: bfo1Freq,
      bfo2Freq: bfo2Freq,
      bfo1StepIdx: bfo1StepIdx,
      bfo2StepIdx: bfo2StepIdx,
      bfo1Memories: bfo1Memories,
      bfo2Memories: bfo2Memories
    };
    fetch('/setSetupConfig', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify(data)
    }).catch(e => console.log('Send error:', e));
  }
  
 async function loadConfig() {
    try {
        let response = await fetch('/getConfig');
        let config = await response.json();
        vfoFreq = config.vfoFreq;
        mode = config.mode;
        stepIdx = config.stepIdx;
        memories = config.memories;
        
        // Get BFO values from Arduino
        if (config.bfo1Usb) BFO1_USB = config.bfo1Usb;
        if (config.bfo1Lsb) BFO1_LSB = config.bfo1Lsb;
        if (config.bfo2Default) BFO2_DEFAULT = config.bfo2Default;
        if (config.ifFrequency) IF_FREQ_CONFIG = config.ifFrequency;
        
        // Set BFO frequencies based on received values
        bfo1Freq = (mode === "USB") ? BFO1_USB : BFO1_LSB;
        bfo2Freq = BFO2_DEFAULT;
        
        updateUI();
        updateBFO1MainUI();
        updateBFO2MainUI();
    } catch(e) {
        console.log('Load error:', e);
    }
}
  
  // ==================== KNOB HANDLING ====================
  function getAngle(x, y) {
    let r = document.getElementById('knob').getBoundingClientRect();
    return Math.atan2(y - (r.top + r.height/2), x - (r.left + r.width/2)) * 180 / Math.PI;
  }
  
  function move(e) {
    if (!isDragging) return;
    let ev = e.touches ? e.touches[0] : e;
    let ang = getAngle(ev.clientX, ev.clientY);
    let d = ang - lastAngle;
    if (d > 180) d -= 360;
    if (d < -180) d += 360;
    currentRotation += d;

    let step = steps[stepIdx];
    
    // Calculate steps with sensitivity
    let steps_change = Math.round(d / knobSensitivity);
    
    // Limit maximum steps per event (removes velocity sensitivity)
    if (steps_change > KNOB_MAX_STEPS) steps_change = KNOB_MAX_STEPS;
    if (steps_change < -KNOB_MAX_STEPS) steps_change = -KNOB_MAX_STEPS;
    
    let delta = steps_change * step;
    
    let newOpFreq = getOperatingFreq() + delta;
    
    if(newOpFreq >= MIN_OP && newOpFreq <= MAX_OP) {
        setOperatingFreq(newOpFreq);
        updateUI();
        sendConfig();


    }
    
    document.getElementById('knob').style.transform = 'rotate(' + currentRotation + 'deg)';
    lastAngle = ang;
  }
  
  function startDrag(e) {
    isDragging = true;
    let ev = e.touches ? e.touches[0] : e;
    lastAngle = getAngle(ev.clientX, ev.clientY);
    e.preventDefault();
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
    document.getElementById('mode-lsb').onclick = () => setMode("LSB");
    document.getElementById('mode-usb').onclick = () => setMode("USB");
    document.getElementById('mode-am').onclick = () => setMode("AM");
    document.getElementById('mode-cw').onclick = () => setMode("CW");

    document.getElementById('sensitivity-slider').oninput = function() {
    knobSensitivity = parseInt(this.value);
    document.getElementById('sensitivity-value').innerText = knobSensitivity;}

    document.getElementById('step-down').onclick = () => changeStep(-1);
    document.getElementById('step-up').onclick = () => changeStep(1);
    document.getElementById('step-cycle').onclick = () => cycleStep();
    
    document.getElementById('freq-10').onclick = () => adjustFrequency(-10);
    document.getElementById('freq-1').onclick = () => adjustFrequency(-1);
    document.getElementById('freq+1').onclick = () => adjustFrequency(1);
    document.getElementById('freq+10').onclick = () => adjustFrequency(10);
    
    document.querySelectorAll('.band-btn').forEach(btn => {
      btn.onclick = () => setBand(parseInt(btn.getAttribute('data-band')));
    });
    
    // VFO long press for memory mode
    let display = document.getElementById('display');
    let pressTimer;
    display.addEventListener('touchstart', () => {
      pressTimer = setTimeout(() => enterMemoryMode(), 500);
    });
    display.addEventListener('touchend', () => clearTimeout(pressTimer));
    display.addEventListener('mousedown', () => {
      pressTimer = setTimeout(() => enterMemoryMode(), 500);
    });
    display.addEventListener('mouseup', () => clearTimeout(pressTimer));
    

    // Knob events
    let knob = document.getElementById('knob');
    knob.addEventListener('mousedown', startDrag);
    knob.addEventListener('touchstart', startDrag, {passive: false});
    window.addEventListener('mouseup', () => isDragging = false);
    window.addEventListener('touchend', () => isDragging = false);
    window.addEventListener('mousemove', move);
    window.addEventListener('touchmove', move, {passive: false});
  }
  
  // ==================== INITIALIZATION ====================
  let sGrid = document.getElementById('s-grid');
  for(let i = 0; i < 20; i++) {
    let seg = document.createElement('div');
    seg.className = 's-seg';
    sGrid.appendChild(seg);
  }
  
  setupEventListeners();
  loadConfig();
  setInterval(updateSMeter, 250);
</script>
</body>
</html>
)rawliteral";

const char SETUP_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<meta charset="UTF-8">
<title>uBITX Setup</title>
<style>
  * {
    -webkit-tap-highlight-color: transparent;
    user-select: none;
    touch-action: manipulation;
    box-sizing: border-box;
  }
  
  body {
    margin: 0;
    padding: 10px;
    background: #000;
    font-family: 'Arial Black', Arial, sans-serif;
  }
  
  .container {
    max-width: 900px;
    margin: 0 auto;
    background: #2c3e50;
    border-radius: 20px;
    padding: 15px;
    box-shadow: 0 5px 20px rgba(0,0,0,0.5);
  }
  
  .header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-bottom: 20px;
  }
  
  .title {
    font-size: 24px;
    font-weight: bold;
    color: #f1c40f;
  }
  
  .back-btn {
    background: #e74c3c;
    padding: 8px 15px;
    border-radius: 8px;
    color: white;
    font-weight: bold;
    cursor: pointer;
    border: none;
  }
  
  .clock-card {
    background: #34495e;
    border-radius: 15px;
    padding: 15px;
    margin-bottom: 20px;
    border: 2px solid #f1c40f;
  }
  
  .clock-title {
    font-size: 18px;
    font-weight: bold;
    color: #f1c40f;
    margin-bottom: 10px;
  }
  
  .clock-freq {
    font-size: 32px;
    font-family: 'Courier New', monospace;
    color: white;
    text-align: center;
    margin: 10px 0;
    padding: 10px;
    background: rgba(0,0,0,0.3);
    border-radius: 10px;
    cursor: pointer;
    transition: background 0.2s;
  }
  
  .clock-freq:active {
    background: rgba(0,0,0,0.5);
  }
  
  .clock-freq.memory-mode {
    background: #e67e22;
  }
  
  .clock-info {
    font-size: 12px;
    color: #bdc3c7;
    text-align: center;
    margin-bottom: 15px;
  }
  
  .btn-group {
    display: flex;
    gap: 8px;
    margin: 10px 0;
    flex-wrap: wrap;
  }
  
  .btn {
    flex: 1;
    padding: 10px;
    background: #555;
    border: 2px solid #f1c40f;
    border-radius: 8px;
    color: white;
    font-weight: bold;
    text-align: center;
    cursor: pointer;
    min-width: 60px;
  }
  
  .btn:active {
    transform: scale(0.95);
  }
  
  .btn-primary {
    background: #27ae60;
  }
  
  .step-group {
    margin: 10px 0;
    padding: 10px;
    background: rgba(0,0,0,0.2);
    border-radius: 8px;
  }
  
  .step-label {
    color: white;
    font-size: 14px;
    margin-bottom: 8px;
    text-align: center;
  }
  
  .step-value {
    color: #f1c40f;
    font-weight: bold;
    font-size: 18px;
  }
  
  .reset-btn {
    background: #e67e22;
    margin-top: 10px;
  }
  
  .mem-section {
    margin-top: 15px;
    padding-top: 10px;
    border-top: 1px solid #f1c40f;
  }
  
  .mem-buttons {
    display: grid;
    grid-template-columns: repeat(5, 1fr);
    gap: 8px;
    margin-top: 10px;
  }
  
  .mem-btn {
    background: #8e44ad;
    padding: 8px;
    text-align: center;
    border-radius: 6px;
    color: white;
    font-size: 11px;
    cursor: pointer;
    border: 1px solid #f1c40f;
  }
  
  .mem-btn:active {
    transform: scale(0.95);
  }
  
  .memory-hint {
    font-size: 10px;
    color: #f1c40f;
    text-align: center;
    margin-top: 5px;
  }
  
  .signature {
    text-align: center;
    color: #888;
    font-size: 10px;
    margin-top: 20px;
  }
</style>
</head>
<body>
<div class="container">
  <div class="header">
    <div class="title">⚙️ Setup</div>
    <div class="back-btn" onclick="goBack()">← Back</div>
  </div>
  
  <div class="clock-card">
    <div class="clock-title">🔧 BFO1 (CLK1) - Second IF Mixer</div>
    <div class="clock-freq" id="bfo1-display-main">33.000000 MHz</div>
    <div class="clock-info">33 MHz USB / 57 MHz LSB (Long press frequency to save)</div>
    <div class="step-group">
      <div class="step-label">Current Step: <span class="step-value" id="bfo1-step">1KHz</span></div>
      <div class="btn-group">
        <div class="btn" onclick="bfo1StepDown()">◀ Slower</div>
        <div class="btn btn-primary" onclick="bfo1StepCycle()">Cycle Step</div>
        <div class="btn" onclick="bfo1StepUp()">Faster ▶</div>
      </div>
      <div class="btn-group">
        <div class="btn" onclick="adjustBFO1(-1)">-1 Step</div>
        <div class="btn" onclick="adjustBFO1(1)">+1 Step</div>
      </div>  
      <div class="btn-group">
        <div class="btn" onclick="adjustBFO1(-10)">-10 Step</div>
        <div class="btn" onclick="adjustBFO1(10)">+10 Step</div>
      </div>
    </div>
    <div class="btn-group">
      <div class="btn" onclick="setBFO1toUSB()">Set USB (33MHz)</div>
      <div class="btn" onclick="setBFO1toLSB()">Set LSB (57MHz)</div>
    </div>
    
    <div class="mem-section">
      <div style="color:white; font-size:12px;">📀 BFO1 Memory</div>
      <div class="mem-buttons" id="bfo1-mem-buttons"></div>
      <div class="memory-hint">Tap memory to recall | Long press frequency to save</div>
    </div>
  </div>
  
  <div class="clock-card">
    <div class="clock-title">🔧 BFO2 (CLK0) - Audio Demodulator</div>
    <div class="clock-freq" id="bfo2-display">12.000000 MHz</div>
    <div class="clock-info">12 MHz Carrier (Long press frequency to save)</div>
    <div class="step-group">
      <div class="step-label">Current Step: <span class="step-value" id="bfo2-step">1KHz</span></div>
      <div class="btn-group">
        <div class="btn" onclick="bfo2StepDown()">◀ Slower</div>
        <div class="btn btn-primary" onclick="bfo2StepCycle()">Cycle Step</div>
        <div class="btn" onclick="bfo2StepUp()">Faster ▶</div>
      </div>
      <div class="btn-group">
        <div class="btn" onclick="adjustBFO2(-1)">-1 Step</div>
        <div class="btn" onclick="adjustBFO2(1)">+1 Step</div>
      </div>
      <div class="btn-group">
        <div class="btn" onclick="adjustBFO2(-10)">-10 Step</div>
        <div class="btn" onclick="adjustBFO2(10)">+10 Step</div>
      </div>
    </div>
    <div class="btn-group">
      <div class="btn reset-btn" onclick="resetBFO2()">Reset to 12MHz</div>
    </div>
    
    <div class="mem-section">
      <div style="color:white; font-size:12px;">📀 BFO2 Memory</div>
      <div class="mem-buttons" id="bfo2-mem-buttons"></div>
      <div class="memory-hint">Tap memory to recall | Long press frequency to save</div>
    </div>
  </div>
  
  <div class="signature">uBITX v3/4/5/6 VFO Setup - Adjust BFOs</div>
</div>

<script>
  let bfo1Freq = 33000000;
  let bfo2Freq = 12000000;
  let bfo1StepIdx = 2;
  let bfo2StepIdx = 2;
  let bfo1Memories = new Array(10).fill(0);
  let bfo2Memories = new Array(10).fill(0);
  let bfo1MemoryMode = false;
  let bfo2MemoryMode = false;
  
  const steps = [10, 100, 1000, 5000, 10000, 100000, 1000000];
  const stepLabels = ["10Hz", "100Hz", "1KHz", "5KHz", "10KHz", "100KHz", "1MHz"];
  
  function formatFrequency(freq) {
    return (freq/1000000).toFixed(6) + " MHz";
  }
  
  function updateBFO1UI() {
    document.getElementById('bfo1-display').innerText = formatFrequency(bfo1Freq);
    document.getElementById('bfo1-step').innerText = stepLabels[bfo1StepIdx];
    if(bfo1MemoryMode) {
      document.getElementById('bfo1-display').classList.add('memory-mode');
    } else {
      document.getElementById('bfo1-display').classList.remove('memory-mode');
    }
    updateBFO1MemoryButtons();
  }
  
  function updateBFO2UI() {
    document.getElementById('bfo2-display').innerText = formatFrequency(bfo2Freq);
    document.getElementById('bfo2-step').innerText = stepLabels[bfo2StepIdx];
    if(bfo2MemoryMode) {
      document.getElementById('bfo2-display').classList.add('memory-mode');
    } else {
      document.getElementById('bfo2-display').classList.remove('memory-mode');
    }
    updateBFO2MemoryButtons();
  }
  
  function updateBFO1MemoryButtons() {
    let container = document.getElementById('bfo1-mem-buttons');
    container.innerHTML = '';
    for(let i = 0; i < 10; i++) {
      let btn = document.createElement('div');
      btn.className = 'mem-btn';
      let memVal = bfo1Memories[i];
      if(memVal && memVal > 0) {
        btn.innerHTML = `M${i+1}<br><span style="font-size:9px;">${(memVal/1000000).toFixed(3)}M</span>`;
      } else {
        btn.innerHTML = `M${i+1}<br><span style="font-size:9px;">Empty</span>`;
      }
      btn.onclick = (function(slot) {
        return function() { handleBFO1Memory(slot); };
      })(i);
      container.appendChild(btn);
    }
  }
  
  function updateBFO2MemoryButtons() {
    let container = document.getElementById('bfo2-mem-buttons');
    container.innerHTML = '';
    for(let i = 0; i < 10; i++) {
      let btn = document.createElement('div');
      btn.className = 'mem-btn';
      let memVal = bfo2Memories[i];
      if(memVal && memVal > 0) {
        btn.innerHTML = `M${i+1}<br><span style="font-size:9px;">${(memVal/1000000).toFixed(3)}M</span>`;
      } else {
        btn.innerHTML = `M${i+1}<br><span style="font-size:9px;">Empty</span>`;
      }
      btn.onclick = (function(slot) {
        return function() { handleBFO2Memory(slot); };
      })(i);
      container.appendChild(btn);
    }
  }
  
  function handleBFO1Memory(slot) {
    if(bfo1MemoryMode) {
      bfo1Memories[slot] = bfo1Freq;
      updateBFO1MemoryButtons();
      exitBFO1MemoryMode();
      sendConfig();
    } else {
      let saved = bfo1Memories[slot];
      if(saved && saved > 0) {
        bfo1Freq = saved;
        updateBFO1UI();
        sendConfig();
      }
    }
  }
  
  function handleBFO2Memory(slot) {
    if(bfo2MemoryMode) {
      bfo2Memories[slot] = bfo2Freq;
      updateBFO2MemoryButtons();
      exitBFO2MemoryMode();
      sendConfig();
    } else {
      let saved = bfo2Memories[slot];
      if(saved && saved > 0) {
        bfo2Freq = saved;
        updateBFO2UI();
        sendConfig();
      }
    }
  }
  
  function enterBFO1MemoryMode() {
    bfo1MemoryMode = true;
    updateBFO1UI();
    setTimeout(() => {
      if(bfo1MemoryMode) {
        exitBFO1MemoryMode();
      }
    }, 5000);
  }
  
  function exitBFO1MemoryMode() {
    bfo1MemoryMode = false;
    updateBFO1UI();
  }
  
  function enterBFO2MemoryMode() {
    bfo2MemoryMode = true;
    updateBFO2UI();
    setTimeout(() => {
      if(bfo2MemoryMode) {
        exitBFO2MemoryMode();
      }
    }, 5000);
  }
  
  function exitBFO2MemoryMode() {
    bfo2MemoryMode = false;
    updateBFO2UI();
  }
  
  function adjustBFO1(delta) {
    let step = steps[bfo1StepIdx];
    let newFreq = bfo1Freq + (delta * step);
    if(newFreq >= 1000000 && newFreq <= 60000000) {
      bfo1Freq = newFreq;
      updateBFO1UI();
      sendConfig();
    }
  }
  
  function adjustBFO2(delta) {
    let step = steps[bfo2StepIdx];
    let newFreq = bfo2Freq + (delta * step);
    if(newFreq >= 1000000 && newFreq <= 60000000) {
      bfo2Freq = newFreq;
      updateBFO2UI();
      sendConfig();
    }
  }
  
  function bfo1StepDown() {
    if(bfo1StepIdx > 0) {
      bfo1StepIdx--;
      updateBFO1UI();
      sendConfig();
    }
  }
  
  function bfo1StepUp() {
    if(bfo1StepIdx < steps.length - 1) {
      bfo1StepIdx++;
      updateBFO1UI();
      sendConfig();
    }
  }
  
  function bfo1StepCycle() {
    bfo1StepIdx = (bfo1StepIdx + 1) % steps.length;
    updateBFO1UI();
    sendConfig();
  }
  
  function bfo2StepDown() {
    if(bfo2StepIdx > 0) {
      bfo2StepIdx--;
      updateBFO2UI();
      sendConfig();
    }
  }
  
  function bfo2StepUp() {
    if(bfo2StepIdx < steps.length - 1) {
      bfo2StepIdx++;
      updateBFO2UI();
      sendConfig();
    }
  }
  
  function bfo2StepCycle() {
    bfo2StepIdx = (bfo2StepIdx + 1) % steps.length;
    updateBFO2UI();
    sendConfig();
  }
  
  function setBFO1toUSB() {
    bfo1Freq = 33000000;
    updateBFO1UI();
    sendConfig();
  }
  
  function setBFO1toLSB() {
    bfo1Freq = 57000000;
    updateBFO1UI();
    sendConfig();
  }
  
  function resetBFO2() {
    bfo2Freq = 12000000;
    updateBFO2UI();
    sendConfig();
  }
  
  function sendConfig() {
    let data = {
      bfo1Freq: bfo1Freq,
      bfo2Freq: bfo2Freq,
      bfo1StepIdx: bfo1StepIdx,
      bfo2StepIdx: bfo2StepIdx,
      bfo1Memories: bfo1Memories,
      bfo2Memories: bfo2Memories
    };
    fetch('/setSetupConfig', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify(data)
    }).catch(e => console.log('Send error:', e));
  }
  
  async function loadConfig() {
    try {
      let response = await fetch('/getSetupConfig');
      let config = await response.json();
      bfo1Freq = config.bfo1Freq;
      bfo2Freq = config.bfo2Freq;
      bfo1StepIdx = config.bfo1StepIdx;
      bfo2StepIdx = config.bfo2StepIdx;
      bfo1Memories = config.bfo1Memories;
      bfo2Memories = config.bfo2Memories;
      updateBFO1UI();
      updateBFO2UI();
    } catch(e) {
      console.log('Load error:', e);
    }
  }
  
  function goBack() {
    window.location.href = '/';
  }
  
  let bfo1Display = document.getElementById('bfo1-display');
  let bfo1Timer;
  bfo1Display.addEventListener('touchstart', (e) => {
    bfo1Timer = setTimeout(() => { enterBFO1MemoryMode(); }, 500);
    e.preventDefault();
  });
  bfo1Display.addEventListener('touchend', () => {
    clearTimeout(bfo1Timer);
  });
  bfo1Display.addEventListener('mousedown', (e) => {
    bfo1Timer = setTimeout(() => { enterBFO1MemoryMode(); }, 500);
    e.preventDefault();
  });
  bfo1Display.addEventListener('mouseup', () => {
    clearTimeout(bfo1Timer);
  });
  
  let bfo2Display = document.getElementById('bfo2-display');
  let bfo2Timer;
  bfo2Display.addEventListener('touchstart', (e) => {
    bfo2Timer = setTimeout(() => { enterBFO2MemoryMode(); }, 500);
    e.preventDefault();
  });
  bfo2Display.addEventListener('touchend', () => {
    clearTimeout(bfo2Timer);
  });
  bfo2Display.addEventListener('mousedown', (e) => {
    bfo2Timer = setTimeout(() => { enterBFO2MemoryMode(); }, 500);
    e.preventDefault();
  });
  bfo2Display.addEventListener('mouseup', () => {
    clearTimeout(bfo2Timer);
  });
  
  loadConfig();
</script>
</body>
</html>
)rawliteral";

// Calculate operating frequency from VFO
unsigned long getOperatingFrequency() {
  return vfoFreq - IF_FREQUENCY;
}

// Set VFO from operating frequency
void setOperatingFrequency(unsigned long opFreq) {
  vfoFreq = opFreq + IF_FREQUENCY;
  if (vfoFreq < MIN_VFO_FREQ) vfoFreq = MIN_VFO_FREQ;
  if (vfoFreq > MAX_VFO_FREQ) vfoFreq = MAX_VFO_FREQ;
}

// Update BFO1 based on mode
void updateBFO1ByMode() {
  if (currentMode == MODE_USB) {
    bfo1Freq = BFO1_USB;
  } else if (currentMode == MODE_LSB) {
    bfo1Freq = BFO1_LSB;
  }
}

void updateAllClocks() {
  // CLK2 = VFO (45-75 MHz)
  si5351.set_freq(vfoFreq * 100ULL, SI5351_CLK2);
  si5351.output_enable(SI5351_CLK2, 1);
  
  // CLK1 = BFO1 (33 or 57 MHz)
  si5351.set_freq(bfo1Freq * 100ULL, SI5351_CLK1);
  si5351.output_enable(SI5351_CLK1, 1);

    Serial.printf(">>> Setting CLK0 to: %lu Hz (centi-Hz: %llu)\n", bfo2Freq, bfo2Freq * 100ULL);
  // CLK0 = BFO2 (12 MHz) - Using standard set_freq, the library handles low frequencies internally
  // The Si5351 library automatically uses the R divider when needed
  si5351.set_freq(bfo2Freq * 100ULL, SI5351_CLK0);
  si5351.output_enable(SI5351_CLK0, 1);
  
  Serial.println("--- Clock Update ---");
  Serial.printf("CLK2 (VFO): %lu Hz (Operating: %lu Hz)\n", vfoFreq, getOperatingFrequency());
  Serial.printf("CLK1 (BFO1): %lu Hz\n", bfo1Freq);
  Serial.printf("CLK0 (BFO2): %lu Hz\n", bfo2Freq);
}


void setup() {
  Serial.begin(115200);
  Wire.begin();  //21, 22);
  
  EEPROM.begin(EEPROM_SIZE);
  loadFromEEPROM();
  
  pinMode(SMETER_PIN, INPUT);
  analogReadResolution(12);
  analogSetAttenuation(ADC_6db);

  pinMode(wifi_switch, INPUT_PULLUP); //High = Local AP ,  Low = Home Wifi
  
  if(digitalRead(wifi_switch)){  //high so use fixed local IP in AP mode
  WiFi.mode(WIFI_AP);
  WiFi.softAP(Local_ssid, Local_password);
  IP = WiFi.softAPIP();
  Serial.print("Connect to AP IP address: ");
  Serial.println(IP);
  }
  else{                 //Low so either mDNS or fixed IP
    WiFi.mode(WIFI_STA);
#ifdef fixedIP
       WiFi.config(staticIP, gateway, subnet, dns);
       WiFi.begin();
#else
    WiFi.begin(Home_ssid, Home_password);
#endif
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
      }
    Serial.println();
    Serial.print("Connected! IP: ");
    Serial.println(WiFi.localIP());
    IP = WiFi.localIP();
  }
  
  if (!si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0)) {
    Serial.println("Si5351 initialization failed!");
    while(1);
  }
  
// ===== CRYSTAL CALIBRATION =====
  // The Si5351 uses a 25MHz crystal reference
//  1. **Measure actual output** with a frequency counter
//2. **Calculate ppm error**: `(measured - expected) / expected * 1,000,000`
//3. **Calculate correction**: `correction = ppm * (-100)`

//For example:
//- Expected: 10,000,000 Hz
//- Measured: 9,999,900 Hz
//- Error: -10 ppm
//- Correction: +1000 (since -10 ppm × -100 = +1000)

  // Correction formula: correction = ppm_error * (-100)
  // 
  // Examples:
  //   If frequency is 10 Hz low at 10 MHz (-1 ppm): correction = +100
  //   If frequency is 10 Hz high at 10 MHz (+1 ppm): correction = -100
  //   If frequency is 100 Hz low at 10 MHz (-10 ppm): correction = +1000
  //   If frequency is 100 Hz high at 10 MHz (+10 ppm): correction = -1000
  //
  // Typical correction values range from -5000 to +5000
  // You can also store this in EEPROM for persistence
  
  #define CRYSTAL_CORRECTION 0 ///148000  // Replace with your measured value, 0 = no correction
   si5351.set_correction(CRYSTAL_CORRECTION, SI5351_PLL_INPUT_XO);
  
  // Optional: Read correction from EEPROM for user calibration
  // int correction;
  // EEPROM.get(CORRECTION_ADDR, correction);
  // si5351.set_correction(correction);
  
  Serial.print("Crystal correction set to: ");
  Serial.println(CRYSTAL_CORRECTION);
  // =================================
    
  setupSi5351Outputs();
  updateAllClocks();
  
  // Main page
   server.on("/", []() {
    String page = MAIN_HTML;
    String ip = WiFi.getMode() == WIFI_AP ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
    page.replace("%%IP%%", ip);
    server.send(200, "text/html", page);
  });
  
  // Setup page
  server.on("/setup", []() { 
    server.send(200, "text/html", SETUP_HTML); 
  });
  
  // Get full configuration
  server.on("/getConfig", []() {
    String json = "{";
    json += "\"vfoFreq\":" + String(vfoFreq) + ",";
    json += "\"mode\":\"" + String(currentMode == MODE_USB ? "USB" : (currentMode == MODE_LSB ? "LSB" : (currentMode == MODE_AM ? "AM" : "CW"))) + "\",";
    json += "\"stepIdx\":" + String(stepIndices[0]) + ",";
    // Add BFO values from Arduino definitions
    json += "\"bfo1Usb\":" + String(BFO1_USB) + ",";
    json += "\"bfo1Lsb\":" + String(BFO1_LSB) + ",";
    json += "\"bfo2Default\":" + String(BFO2_FREQ) + ",";
    json += "\"ifFrequency\":" + String(IF_FREQUENCY) + ",";
    json += "\"memories\":[";
    
    for(int c = 0; c < 3; c++) {
        json += "[";
        for(int i = 0; i < 10; i++) {
            json += String(memorySlots[c][i]);
            if(i < 9) json += ",";
        }
        json += "]";
        if(c < 2) json += ",";
    }
    json += "]}";
    server.send(200, "application/json", json);
});
  
  // Set main configuration - handles VFO memories correctly
  server.on("/setConfig", HTTP_POST, []() {
    if (server.hasArg("plain")) {
      String body = server.arg("plain");
      
      unsigned long newVFO = extractJsonValue(body, "vfoFreq", 0);
      if (newVFO >= MIN_VFO_FREQ && newVFO <= MAX_VFO_FREQ) {
        vfoFreq = newVFO;
      }
      
      // Extract mode
      int modeStart = body.indexOf("\"mode\":\"");
      if (modeStart != -1) {
        modeStart += 8;
        int modeEnd = body.indexOf("\"", modeStart);
        if (modeEnd != -1) {
          String modeStr = body.substring(modeStart, modeEnd);
          if (modeStr == "USB") {
            currentMode = MODE_USB;
            updateBFO1ByMode();
          } else if (modeStr == "LSB") {
            currentMode = MODE_LSB;
            updateBFO1ByMode();
          } else if (modeStr == "AM") currentMode = MODE_AM;
          else if (modeStr == "CW") currentMode = MODE_CW;
        }
      }
      
      stepIndices[0] = extractJsonValue(body, "stepIdx", 0);
      if (stepIndices[0] < 0 || stepIndices[0] >= stepCount) stepIndices[0] = 1;
      
      // Extract memories from the config
      String memoriesStr = body.substring(body.indexOf("\"memories\":[") + 12);
      // Parse VFO memories (first array)
      int vfoMemStart = memoriesStr.indexOf("[");
      int vfoMemEnd = memoriesStr.indexOf("]", vfoMemStart);
      if (vfoMemStart != -1 && vfoMemEnd != -1) {
        String vfoMemArray = memoriesStr.substring(vfoMemStart + 1, vfoMemEnd);
        int commaPos = 0;
        for (int i = 0; i < 10; i++) {
          int nextComma = vfoMemArray.indexOf(",", commaPos);
          if (nextComma == -1) nextComma = vfoMemArray.length();
          String valStr = vfoMemArray.substring(commaPos, nextComma);
          valStr.trim();
          unsigned long val = valStr.toInt();
          // Store VFO frequencies directly (no conversion needed)
          if (val >= MIN_VFO_FREQ && val <= MAX_VFO_FREQ) {
            memorySlots[0][i] = val;
          }
          commaPos = nextComma + 1;
          if (commaPos > vfoMemArray.length()) break;
        }
      }
      
      updateAllClocks();
      scheduleSave();
      server.send(200, "text/plain", "OK");
    }
  });
  
  // Get setup configuration
  server.on("/getSetupConfig", []() {
    String json = "{";
    json += "\"bfo1Freq\":" + String(bfo1Freq) + ",";
    json += "\"bfo2Freq\":" + String(bfo2Freq) + ",";
    json += "\"bfo1StepIdx\":" + String(stepIndices[1]) + ",";
    json += "\"bfo2StepIdx\":" + String(stepIndices[2]) + ",";
    json += "\"bfo1Memories\":[";
    for(int i = 0; i < 10; i++) {
      json += String(memorySlots[1][i]);
      if(i < 9) json += ",";
    }
    json += "],";
    json += "\"bfo2Memories\":[";
    for(int i = 0; i < 10; i++) {
      json += String(memorySlots[2][i]);
      if(i < 9) json += ",";
    }
    json += "]}";
    server.send(200, "application/json", json);
  });
  
  // Set setup configuration
  server.on("/setSetupConfig", HTTP_POST, []() {
    if (server.hasArg("plain")) {
      String body = server.arg("plain");
      
      unsigned long newBFO1 = extractJsonValue(body, "bfo1Freq", 0);
      if (newBFO1 >= 1000000 && newBFO1 <= 60000000) {
        bfo1Freq = newBFO1;
      }
      
      unsigned long newBFO2 = extractJsonValue(body, "bfo2Freq", 0);
      if (newBFO2 >= 1000000 && newBFO2 <= 60000000) {
        bfo2Freq = newBFO2;
      }
      
      int newBFO1Step = extractJsonValue(body, "bfo1StepIdx", 0);
      int newBFO2Step = extractJsonValue(body, "bfo2StepIdx", 0);
      
      if (newBFO1Step >= 0 && newBFO1Step < stepCount) stepIndices[1] = newBFO1Step;
      if (newBFO2Step >= 0 && newBFO2Step < stepCount) stepIndices[2] = newBFO2Step;
      
      updateAllClocks();
      scheduleSave();
      server.send(200, "text/plain", "OK");
    }
  });
  
  server.on("/getS", []() { 
    int val = analogRead(SMETER_PIN);
    int percent = map(val, 0, 1200, 0, 100);
    if(percent > 100) percent = 100;
    server.send(200, "text/plain", String(percent)); 
  });
  
  server.begin();
  Serial.println("uBITX v4 VFO Ready!");
  if(wifi_switch)                       // Onboard WiFi Access Point
    Serial.println("Connect to WiFi: ubitx / 12345678");
  else {
    Serial.println("Connect to : ");    // external Home Wifi
    Serial.print("AP IP: ");
    Serial.println(IP);
  }
//  MDNS.begin("ubitx");
  if (!MDNS.begin("ubitx")) {
  Serial.println("Error setting up MDNS responder!");
  //while(1) {
    delay(1000);
  }

  MDNS.addService("_http", "_tcp", 80);
}

void loop() {
  server.handleClient();
  
  // Deferred EEPROM save - write only after 5 seconds of no changes
  if (saveNeeded && (millis() - lastChangeTime >= SAVE_DELAY_MS)) {
    saveToEEPROM();
    saveNeeded = false;
    Serial.println("Deferred EEPROM save done");
  }
}

int extractJsonValue(String json, String key, int index) {
  String searchKey = "\"" + key + "\":[";
  int startPos = json.indexOf(searchKey);
  if (startPos == -1) {
    searchKey = "\"" + key + "\":";
    startPos = json.indexOf(searchKey);
    if (startPos == -1) return -1;
    startPos += searchKey.length();
    int endPos = json.indexOf(",", startPos);
    if (endPos == -1) endPos = json.indexOf("}", startPos);
    if (endPos == -1) endPos = json.length();
    String valueStr = json.substring(startPos, endPos);
    valueStr.trim();
    return valueStr.toInt();
  }
  
  startPos += searchKey.length();
  int endPos = json.indexOf("]", startPos);
  if (endPos == -1) return -1;
  
  String arrayStr = json.substring(startPos, endPos);
  int valueStart = 0;
  
  for (int i = 0; i <= index; i++) {
    if (i > 0) {
      valueStart = arrayStr.indexOf(",", valueStart) + 1;
      if (valueStart == 0) return -1;
    }
    int valueEnd = arrayStr.indexOf(",", valueStart);
    if (valueEnd == -1) valueEnd = arrayStr.length();
    
    if (i == index) {
      String valueStr = arrayStr.substring(valueStart, valueEnd);
      valueStr.trim();
      return valueStr.toInt();
    }
  }
  return -1;
}

void setupSi5351Outputs() {
  // Disable all outputs first
  si5351.output_enable(SI5351_CLK0, 0);
  si5351.output_enable(SI5351_CLK1, 0);
  si5351.output_enable(SI5351_CLK2, 0);
  
  // Set drive strength for all outputs (8mA is good for most applications)
//  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA); // was causing problem in Clock0 frequency (doubling the freq)
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_8MA);
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_8MA);
  
  // Optional: If you need to disable the clock outputs when not in use
  // si5351.set_clock_pwr(SI5351_CLK0, 0);
  // si5351.set_clock_pwr(SI5351_CLK1, 0);
  // si5351.set_clock_pwr(SI5351_CLK2, 0);
  
  Serial.println("Si5351 outputs configured");
}

// ==================== DEFERRED EEPROM SAVE ====================

void scheduleSave() {
  lastChangeTime = millis();
  saveNeeded = true;
}

void saveToEEPROM() {
  EEPROM.put(MAGIC_NUMBER_ADDR, MAGIC_NUMBER);
  EEPROM.put(MAGIC_NUMBER_ADDR + 4, vfoFreq);
  EEPROM.put(MAGIC_NUMBER_ADDR + 8, bfo1Freq);
  EEPROM.put(MAGIC_NUMBER_ADDR + 12, bfo2Freq);
  EEPROM.put(MAGIC_NUMBER_ADDR + 16, (int)currentMode);
  for (int i = 0; i < 3; i++) {
    EEPROM.put(MAGIC_NUMBER_ADDR + 20 + (i * 4), stepIndices[i]);
  }
  int memAddr = MAGIC_NUMBER_ADDR + 40;
  for (int c = 0; c < 3; c++) {
    for (int i = 0; i < 10; i++) {
      EEPROM.put(memAddr + (c * 40) + (i * 4), memorySlots[c][i]);
    }
  }
  EEPROM.commit();
  Serial.println("Configuration saved");
}

void loadFromEEPROM() {
  int magic;
  EEPROM.get(MAGIC_NUMBER_ADDR, magic);
  
  if (magic == MAGIC_NUMBER) {
    EEPROM.get(MAGIC_NUMBER_ADDR + 4, vfoFreq);
    EEPROM.get(MAGIC_NUMBER_ADDR + 8, bfo1Freq);
    EEPROM.get(MAGIC_NUMBER_ADDR + 12, bfo2Freq);
    int mode;
    EEPROM.get(MAGIC_NUMBER_ADDR + 16, mode);
    currentMode = (Mode)mode;
    for (int i = 0; i < 3; i++) {
      EEPROM.get(MAGIC_NUMBER_ADDR + 20 + (i * 4), stepIndices[i]);
    }
    
    int memAddr = MAGIC_NUMBER_ADDR + 40;
    for (int c = 0; c < 3; c++) {
      for (int i = 0; i < 10; i++) {
        EEPROM.get(memAddr + (c * 40) + (i * 4), memorySlots[c][i]);
      }
    }
    
    if (vfoFreq < MIN_VFO_FREQ || vfoFreq > MAX_VFO_FREQ) vfoFreq = 52000000;
    if (bfo1Freq < 1000000 || bfo1Freq > 60000000) bfo1Freq = (currentMode == MODE_USB) ? BFO1_USB : BFO1_LSB;
    if (bfo2Freq < 1000000 || bfo2Freq > 60000000) bfo2Freq = BFO2_FREQ;
    if (stepIndices[1] < 0 || stepIndices[1] >= stepCount) stepIndices[1] = 2;
    if (stepIndices[2] < 0 || stepIndices[2] >= stepCount) stepIndices[2] = 2;
    
    Serial.println("Configuration loaded");
  } else {
    vfoFreq = 52000000;
    bfo1Freq = BFO1_USB;
    bfo2Freq = BFO2_FREQ;
    currentMode = MODE_USB;
    stepIndices[0] = 1;
    stepIndices[1] = 2;
    stepIndices[2] = 2;
    memset(memorySlots, 0, sizeof(memorySlots));
    Serial.println("Using defaults");
  }
}
