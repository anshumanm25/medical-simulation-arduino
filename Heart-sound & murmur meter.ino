#include <Arduino.h>
#include <HardwareSerial.h>
#include <DFRobotDFPlayerMini.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h> 

// ─────────────────────────────────────────────
//  PIN DEFINITIONS
// ─────────────────────────────────────────────
#define DF_RX_PIN   16   // ESP32 GPIO16 ← DFPlayer TX
#define DF_TX_PIN   17   // ESP32 GPIO17 → DFPlayer RX (via R1)

#define SENSOR_1    27   // Aortic
#define SENSOR_2    15   // Pulmonary
#define SENSOR_3    14   // Tricuspid
#define SENSOR_4    4    // Mitral

// ─────────────────────────────────────────────
//  SD CARD FILE MAP
// ─────────────────────────────────────────────
#define FOLDER           1
#define FILE_NORMAL      1
#define FILE_COND_A      2  // PDA
#define FILE_COND_B      3  // ASD
#define FILE_COND_C      4  // VSD
#define FILE_COND_D      5  // AS
#define FILE_COND_E      6  // AR
#define FILE_COND_F      7  // AS + AR
#define FILE_COND_G      8  // MS
#define FILE_COND_H      9  // MR
#define FILE_COND_I      10 // MS + MR
#define FILE_COND_J      11 // TR
#define FILE_COND_K      12 // Pericardial Rub
#define FILE_COND_L      13 // PS + PR
#define FILE_COND_M      14 // PS

// ─────────────────────────────────────────────
//  CONDITION TABLE
// ─────────────────────────────────────────────
struct Condition {
  uint8_t   abnormalSensor; // 1=Aortic, 2=Pulmonary, 3=Tricuspid, 4=Mitral
  uint8_t   abnormalFile;
  char      label;
  const char* fullName; 
};

const Condition CONDITIONS[] = {
  { 2, FILE_COND_A, 'A', "PDA" },
  { 2, FILE_COND_B, 'B', "ASD" },
  { 3, FILE_COND_C, 'C', "VSD" },
  { 1, FILE_COND_D, 'D', "AS" },
  { 1, FILE_COND_E, 'E', "AR" },
  { 1, FILE_COND_F, 'F', "AS + AR" },
  { 4, FILE_COND_G, 'G', "MS" },
  { 4, FILE_COND_H, 'H', "MR" },
  { 4, FILE_COND_I, 'I', "MS + MR" },
  { 3, FILE_COND_J, 'J', "TR" },
  { 0, FILE_COND_K, 'K', "Pericardial Rub" }, // 0 indicates multi-sensor global sound
  { 2, FILE_COND_L, 'L', "PS + PR" },         // Fixed missing comma syntax here
  { 2, FILE_COND_M, 'M', "PS" }
};
const uint8_t NUM_CONDITIONS = sizeof(CONDITIONS) / sizeof(CONDITIONS[0]);

// ─────────────────────────────────────────────
//  WI-FI, WEBSERVER & DNS CONFIGURATION
// ─────────────────────────────────────────────
const char* ssid = "Heart_Simulator";
const char* password = "12345678";
WebServer server(80);
DNSServer dnsServer; 
const byte DNS_PORT = 53;

// Modern, Mobile-Responsive Dashboard UI (Updated with Condition M)
const char HTML_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>Simulator Controller</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; }
        body { background-color: #F8F9FC; color: #1E293B; display: flex; justify-content: center; padding: 8px; }
        .phone-container { width: 100%; max-width: 412px; background: #F8F9FC; min-height: calc(100vh - 16px); display: flex; flex-direction: column; gap: 12px; }
        .header-card { background: linear-gradient(135deg, #4F46E5 0%, #3B82F6 100%); border-radius: 16px; padding: 16px; color: white; box-shadow: 0 8px 20px rgba(59, 130, 246, 0.15); }
        .header-main { display: flex; align-items: center; gap: 10px; margin-bottom: 2px; }
        .header-icon svg { width: 22px; height: 22px; fill: currentColor; }
        .header-card h1 { font-size: 18px; font-weight: 600; letter-spacing: -0.3px; }
        .header-card p { font-size: 11px; color: rgba(255, 255, 255, 0.8); margin-bottom: 12px; margin-left: 32px; }
        .active-banner { background: white; border-radius: 12px; padding: 10px 14px; display: flex; align-items: center; box-shadow: 0 4px 12px rgba(0,0,0,0.03); }
        .active-left { display: flex; align-items: center; gap: 10px; }
        .status-dot { width: 10px; height: 10px; background-color: #22C55E; border-radius: 50%; box-shadow: 0 0 0 3px rgba(34, 197, 94, 0.2); }
        .active-title { font-size: 9px; color: #64748B; text-transform: uppercase; font-weight: 600; letter-spacing: 0.5px; }
        .active-value { font-size: 14px; color: #0F172A; font-weight: 700; margin-top: 1px; }
        .section-title-wrap { margin-top: 2px; display: flex; gap: 6px; align-items: center; }
        .lightning-icon svg { width: 16px; height: 16px; fill: none; stroke: #4F46E5; stroke-width: 2; }
        .section-title { font-size: 14px; font-weight: 600; color: #0F172A; }
        .grid-container { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
        .full-width-card { grid-column: span 2; }
        .action-card { background: white; border: 1.5px solid #E2E8F0; border-radius: 12px; padding: 12px; text-align: left; cursor: pointer; transition: all 0.15s ease; display: flex; flex-direction: column; position: relative; -webkit-tap-highlight-color: transparent; }
        .action-card:active { transform: scale(0.97); }
        .card-icon-circle { width: 32px; height: 32px; border-radius: 50%; display: flex; align-items: center; justify-content: center; margin-bottom: 12px; }
        .card-icon-circle svg { width: 16px; height: 16px; }
        .card-title { font-size: 13px; font-weight: 700; color: #0F172A; }
        .card-desc { font-size: 10px; color: #94A3B8; margin-top: 1px; }
        .arrow-pill { position: absolute; bottom: 12px; right: 12px; width: 22px; height: 22px; border-radius: 50%; border: 1px solid #E2E8F0; display: flex; align-items: center; justify-content: center; background: white; }
        .arrow-pill svg { width: 10px; height: 10px; stroke-width: 2.5; stroke: currentColor; fill: none; }
        
        .mode-normal { --accent: #22C55E; }
        .mode-normal .card-icon-circle { background: #DCFCE7; }
        .mode-normal .card-icon-circle svg { fill: none; stroke: #22C55E; stroke-width: 2; }
        .mode-normal .arrow-pill { color: #22C55E; }
        .mode-normal.action-card { flex-direction: row; align-items: center; justify-content: space-between; padding: 12px; }
        .mode-normal .n-left-block { display: flex; align-items: center; gap: 10px; }
        .mode-normal .card-icon-circle { margin-bottom: 0; }
        .mode-normal .n-text-block { display: flex; flex-direction: column; }
        .mode-normal .arrow-pill { position: static; }

        .mode-abnormal { --accent: #EF4444; }
        .mode-abnormal .card-icon-circle { background: #FEF2F2; }
        .mode-abnormal .card-icon-circle svg { fill: none; stroke: #EF4444; stroke-width: 2; }
        .mode-abnormal .arrow-pill { color: #EF4444; }

        .action-card.active-card { border-color: var(--accent); background-color: white; box-shadow: 0 4px 12px rgba(0, 0, 0, 0.04); }
        .action-card.active-card .card-title { color: var(--accent); }
    </style>
    <script>
        function updateCondition(endpoint, cardElement) {
            fetch('/set?mode=' + endpoint)
            .then(response => response.text())
            .then(text => { 
                document.getElementById('status-text').innerText = text.toUpperCase(); 
                const cards = document.querySelectorAll('.action-card');
                cards.forEach(c => c.classList.remove('active-card'));
                cardElement.classList.add('active-card');
            }).catch(err => console.error("Communication error", err));
        }
    </script>
</head>
<body>
    <div class="phone-container">
        <div class="header-card">
            <div class="header-main">
                <span class="header-icon">
                    <svg viewBox="0 0 24 24"><path d="M12 21.35l-1.45-1.32C5.4 15.36 2 12.28 2 8.5 2 5.42 4.42 3 7.5 3c1.74 0 3.41.81 4.5 2.09C13.09 3.81 14.76 3 16.5 3 19.58 3 22 5.42 22 8.5c0 3.78-3.4 6.86-8.55 11.54L12 21.35z"/></svg>
                </span>
                <h1>Simulator Controller</h1>
            </div>
            <p>Control and manage simulator modes</p>
            <div class="active-banner">
                <div class="active-left">
                    <div class="status-dot"></div>
                    <div>
                        <div class="active-title">Active Mode</div>
                        <div id="status-text" class="active-value">NORMAL MODE</div>
                    </div>
                </div>
            </div>
        </div>

        <div class="section-title-wrap">
            <span class="lightning-icon">
                <svg viewBox="0 0 24 24"><polygon points="13 2 3 14 12 14 11 22 21 10 12 10 13 2"/></svg>
            </span>
            <div class="section-title">Select Medical Condition</div>
        </div>

        <div class="grid-container">
            <div class="action-card mode-normal full-width-card active-card" onclick="updateCondition('N', this)">
                <div class="n-left-block">
                    <div class="card-icon-circle"><svg viewBox="0 0 24 24"><path d="M12 22s8-4 8-10V5l-8-3-8 3v7c0 6 8 10 8 10z"/></svg></div>
                    <div class="n-text-block">
                        <div class="card-title">NORMAL MODE</div>
                        <div class="card-desc">Healthy baseline cardiac profile</div>
                    </div>
                </div>
                <div class="arrow-pill"><svg viewBox="0 0 24 24"><path d="M9 5l7 7-7 7"/></svg></div>
            </div>

            <div class="action-card mode-abnormal" onclick="updateCondition('A', this)">
                <div class="card-icon-circle"><svg viewBox="0 0 24 24"><path d="M22 12h-4l-3 9L9 3l-3 9H2"/></svg></div>
                <div class="card-title">PDA</div>
                <div class="card-desc">Patent Ductus Arteriosus</div>
                <div class="arrow-pill"><svg viewBox="0 0 24 24"><path d="M9 5l7 7-7 7"/></svg></div>
            </div>
            <div class="action-card mode-abnormal" onclick="updateCondition('B', this)">
                <div class="card-icon-circle"><svg viewBox="0 0 24 24"><path d="M22 12h-4l-3 9L9 3l-3 9H2"/></svg></div>
                <div class="card-title">ASD</div>
                <div class="card-desc">Atrial Septal Defect</div>
                <div class="arrow-pill"><svg viewBox="0 0 24 24"><path d="M9 5l7 7-7 7"/></svg></div>
            </div>
            <div class="action-card mode-abnormal" onclick="updateCondition('C', this)">
                <div class="card-icon-circle"><svg viewBox="0 0 24 24"><path d="M22 12h-4l-3 9L9 3l-3 9H2"/></svg></div>
                <div class="card-title">VSD</div>
                <div class="card-desc">Ventricular Septal Defect</div>
                <div class="arrow-pill"><svg viewBox="0 0 24 24"><path d="M9 5l7 7-7 7"/></svg></div>
            </div>
            <div class="action-card mode-abnormal" onclick="updateCondition('D', this)">
                <div class="card-icon-circle"><svg viewBox="0 0 24 24"><path d="M22 12h-4l-3 9L9 3l-3 9H2"/></svg></div>
                <div class="card-title">AS</div>
                <div class="card-desc">Aortic Stenosis</div>
                <div class="arrow-pill"><svg viewBox="0 0 24 24"><path d="M9 5l7 7-7 7"/></svg></div>
            </div>
            <div class="action-card mode-abnormal" onclick="updateCondition('E', this)">
                <div class="card-icon-circle"><svg viewBox="0 0 24 24"><path d="M22 12h-4l-3 9L9 3l-3 9H2"/></svg></div>
                <div class="card-title">AR</div>
                <div class="card-desc">Aortic Regurgitation</div>
                <div class="arrow-pill"><svg viewBox="0 0 24 24"><path d="M9 5l7 7-7 7"/></svg></div>
            </div>
            <div class="action-card mode-abnormal" onclick="updateCondition('F', this)">
                <div class="card-icon-circle"><svg viewBox="0 0 24 24"><path d="M22 12h-4l-3 9L9 3l-3 9H2"/></svg></div>
                <div class="card-title">AS + AR</div>
                <div class="card-desc">Combined Aortic Disease</div>
                <div class="arrow-pill"><svg viewBox="0 0 24 24"><path d="M9 5l7 7-7 7"/></svg></div>
            </div>
            <div class="action-card mode-abnormal" onclick="updateCondition('G', this)">
                <div class="card-icon-circle"><svg viewBox="0 0 24 24"><path d="M22 12h-4l-3 9L9 3l-3 9H2"/></svg></div>
                <div class="card-title">MS</div>
                <div class="card-desc">Mitral Stenosis</div>
                <div class="arrow-pill"><svg viewBox="0 0 24 24"><path d="M9 5l7 7-7 7"/></svg></div>
            </div>
            <div class="action-card mode-abnormal" onclick="updateCondition('H', this)">
                <div class="card-icon-circle"><svg viewBox="0 0 24 24"><path d="M22 12h-4l-3 9L9 3l-3 9H2"/></svg></div>
                <div class="card-title">MR</div>
                <div class="card-desc">Mitral Regurgitation</div>
                <div class="arrow-pill"><svg viewBox="0 0 24 24"><path d="M9 5l7 7-7 7"/></svg></div>
            </div>
            <div class="action-card mode-abnormal" onclick="updateCondition('I', this)">
                <div class="card-icon-circle"><svg viewBox="0 0 24 24"><path d="M22 12h-4l-3 9L9 3l-3 9H2"/></svg></div>
                <div class="card-title">MS + MR</div>
                <div class="card-desc">Combined Mitral Disease</div>
                <div class="arrow-pill"><svg viewBox="0 0 24 24"><path d="M9 5l7 7-7 7"/></svg></div>
            </div>
            <div class="action-card mode-abnormal" onclick="updateCondition('J', this)">
                <div class="card-icon-circle"><svg viewBox="0 0 24 24"><path d="M22 12h-4l-3 9L9 3l-3 9H2"/></svg></div>
                <div class="card-title">TR</div>
                <div class="card-desc">Tricuspid Regurgitation</div>
                <div class="arrow-pill"><svg viewBox="0 0 24 24"><path d="M9 5l7 7-7 7"/></svg></div>
            </div>
            <div class="action-card mode-abnormal" onclick="updateCondition('L', this)">
                <div class="card-icon-circle"><svg viewBox="0 0 24 24"><path d="M22 12h-4l-3 9L9 3l-3 9H2"/></svg></div>
                <div class="card-title">PS + PR</div>
                <div class="card-desc">Pulmonary Stenosis & Regurgitation</div>
                <div class="arrow-pill"><svg viewBox="0 0 24 24"><path d="M9 5l7 7-7 7"/></svg></div>
            </div>
            <div class="action-card mode-abnormal" onclick="updateCondition('M', this)">
                <div class="card-icon-circle"><svg viewBox="0 0 24 24"><path d="M22 12h-4l-3 9L9 3l-3 9H2"/></svg></div>
                <div class="card-title">PS</div>
                <div class="card-desc">Pulmonary Stenosis</div>
                <div class="arrow-pill"><svg viewBox="0 0 24 24"><path d="M9 5l7 7-7 7"/></svg></div>
            </div>
            <div class="action-card mode-abnormal full-width-card" onclick="updateCondition('K', this)">
                <div class="card-icon-circle"><svg viewBox="0 0 24 24"><path d="M22 12h-4l-3 9L9 3l-3 9H2"/></svg></div>
                <div class="card-title">Pericardial Rub</div>
                <div class="card-desc">Pericardial Inflammation Friction Sound</div>
                <div class="arrow-pill"><svg viewBox="0 0 24 24"><path d="M9 5l7 7-7 7"/></svg></div>
            </div>
        </div>
    </div>
</body>
</html>
)rawliteral";

// ─────────────────────────────────────────────
//  GLOBALS
// ─────────────────────────────────────────────
HardwareSerial dfSerial(2);        
DFRobotDFPlayerMini player;

int8_t  activeCondition  = -1;
int8_t  lastSensorPlayed = -1;
uint8_t lastFileQueued   =  0;
bool    isPlaying        = false;

// ─────────────────────────────────────────────
//  HELPER FUNCTIONS
// ─────────────────────────────────────────────
uint8_t fileForSensor(uint8_t sensorNum) {
  if (activeCondition < 0) return FILE_NORMAL;
  
  const Condition& c = CONDITIONS[activeCondition];
  
  // Custom Multi-Sensor Check for Pericardial Rub (Condition K)
  if (c.abnormalSensor == 0) {
    return c.abnormalFile; 
  }
  
  return (sensorNum == c.abnormalSensor) ? c.abnormalFile : FILE_NORMAL;
}

bool isTouched(uint8_t pin) {
  return digitalRead(pin) == HIGH;
}

void playFile(uint8_t fileNum) {
  if (isPlaying && lastFileQueued == fileNum) return;
  player.playFolder(FOLDER, fileNum);
  lastFileQueued = fileNum;
  isPlaying      = true;
  Serial.printf("  Playing folder %d / file %d\n", FOLDER, fileNum);
}

void printMenu() {
  Serial.println("\n================================================");
  Serial.println("  Cardiac Simulation System - 13 Condition Matrix");
  Serial.println("  Select an operating variant:");
  for (uint8_t i = 0; i < NUM_CONDITIONS; i++) {
    if (CONDITIONS[i].abnormalSensor == 0) {
      Serial.printf("   [%c]  %-16s (abnormal at ALL sensors)\n", CONDITIONS[i].label, CONDITIONS[i].fullName);
    } else {
      Serial.printf("   [%c]  %-16s (abnormal at sensor %d)\n", CONDITIONS[i].label, CONDITIONS[i].fullName, CONDITIONS[i].abnormalSensor);
    }
  }
  Serial.println("   [N]  Normal Mode Configuration");
  Serial.println("   [?]  Re-print Dashboard Layout Menu");
  Serial.println("================================================\n");
}

void handleSerialMenu() {
  if (!Serial.available()) return;
  char ch = toupper(Serial.read());

  while (Serial.available()) Serial.read(); 
  if (ch < 32) return;

  if (ch >= 'A' && ch < 'A' + NUM_CONDITIONS) {
    activeCondition  = ch - 'A';
    lastSensorPlayed = -1;
    isPlaying        = false;
    player.stop();                      
    Serial.printf("\n Condition %s [%c] selected via Serial.\n", CONDITIONS[activeCondition].fullName, ch);
  } else if (ch == 'N') {
    activeCondition  = -1;
    lastSensorPlayed = -1;
    isPlaying        = false;
    player.stop();                      
    Serial.println("\n Normal mode selected via Serial.");
  } else if (ch == '?') {
    printMenu();
  }
}

// ─────────────────────────────────────────────
//  WEB SERVER ROUTINES
// ─────────────────────────────────────────────
void handleRoot() {
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "-1");
  server.send_P(200, "text/html", HTML_PAGE);
}

void handleWiFiSelection() {
  if (server.hasArg("mode")) {
    char ch = toupper(server.arg("mode")[0]);
    
    lastSensorPlayed = -1;
    isPlaying = false;
    player.stop();

    if (ch >= 'A' && ch < 'A' + NUM_CONDITIONS) {
      activeCondition = ch - 'A';
      Serial.printf("\n Condition %c selected via Wi-Fi.\n", ch);
      server.send(200, "text/plain", CONDITIONS[activeCondition].fullName);
      return;
    } else if (ch == 'N') {
      activeCondition = -1;
      Serial.println("\n Normal mode selected via Wi-Fi.");
      server.send(200, "text/plain", "Normal Mode");
      return;
    }
  }
  server.send(400, "text/plain", "Invalid Parameter");
}

void handleNotFound() {
  server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString(), true);
  server.send(302, "text/plain", ""); 
}

// ─────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────
void setup() {
  delay(2000);
  Serial.begin(115200);
  
  pinMode(SENSOR_1, INPUT);
  pinMode(SENSOR_2, INPUT);
  pinMode(SENSOR_3, INPUT);
  pinMode(SENSOR_4, INPUT);
  
  dfSerial.begin(9600, SERIAL_8N1, DF_RX_PIN, DF_TX_PIN);

  Serial.println("\nInitialising DFPlayer Mini...");
  if (!player.begin(dfSerial, true, true)) {
    Serial.println("DFPlayer error! Verify hardware linkages / file tracks.");
    while (true) delay(1000);
  }
  Serial.println("DFPlayer ready.");

  // Hardware Volume Maximum Limit (30/30)
  player.volume(30);
  player.EQ(DFPLAYER_EQ_NORMAL);

  Serial.println("Setting up Wi-Fi Access Point...");
  IPAddress apIP(192, 168, 4, 1);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(ssid, password);
  
  Serial.print("Access Point Live! SSID: ");
  Serial.println(ssid);

  dnsServer.start(DNS_PORT, "*", apIP);
  Serial.println("DNS Captive Server Started.");

  server.on("/", handleRoot);
  server.on("/set", handleWiFiSelection);
  server.on("/generate_204", handleRoot);  
  server.on("/fwlink", handleRoot);        
  server.onNotFound(handleNotFound);       
  
  server.begin();
  Serial.println("HTTP Web Server Started.");

  printMenu();
}

// ─────────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────────
void loop() {
  dnsServer.processNextRequest(); 
  handleSerialMenu();
  server.handleClient(); 

  bool t1 = isTouched(SENSOR_1);
  bool t2 = isTouched(SENSOR_2);
  bool t3 = isTouched(SENSOR_3);
  bool t4 = isTouched(SENSOR_4);

  int8_t touchedSensor = -1;
  if      (t1) touchedSensor = 1;
  else if (t2) touchedSensor = 2;
  else if (t3) touchedSensor = 3;
  else if (t4) touchedSensor = 4;

  if (touchedSensor == -1) {
    if (lastSensorPlayed != -1) {
      Serial.println("  Sensor released - stopping playback.");
      player.stop();
      isPlaying        = false;
      lastSensorPlayed = -1;
      lastFileQueued   = 0;
    }
    delay(20);
    return;
  }

  if (touchedSensor != lastSensorPlayed) {
    lastSensorPlayed = touchedSensor;
    isPlaying        = false;
    Serial.printf("\n  Sensor %d touched", touchedSensor);

    uint8_t f = fileForSensor(touchedSensor);
    
    bool isAbnormal = false;
    if (activeCondition >= 0) {
      if (CONDITIONS[activeCondition].abnormalSensor == 0 || touchedSensor == CONDITIONS[activeCondition].abnormalSensor) {
        isAbnormal = true;
      }
    }
    
    Serial.printf("  [%s]\n", isAbnormal ? "ABNORMAL" : "normal");
    playFile(f);
  }

  if (isPlaying && player.available()) {
    uint8_t type = player.readType();
    if (type == DFPlayerPlayFinished) {
      playFile(lastFileQueued); 
    }
  }

  delay(10); 
}
