#include <Wire.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <WebServer.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define TRIG_PIN 5
#define ECHO_PIN 18

const byte DNS_PORT = 53;
DNSServer dnsServer;
WebServer server(80);

const char* apSSID = "RespMonitor-Pro";
const char* apPassword = "12345678";

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

float baselineDistance = 12.0;
float filteredDistance = 12.0;
float deltaDistance = 0.0;
float lastValidDistance = 12.0;

float calibrationFactor = 50.0;
float tidalVolume = 0.0;
float displayedTidalVolume = 0.0;
float maxTidalVolume = 0.0;

// RR from distance threshold based on half baseline
float rrTriggerDistance = 6.0;
float rrReleaseDistance = 6.6;

int respirationRate = 0;
bool breathDetected = false;
bool sensorHealthy = true;

unsigned long lastBreathTime = 0;

const int intervalCount = 4;
unsigned long breathIntervals[intervalCount] = {0, 0, 0, 0};
int intervalIndex = 0;
int validIntervals = 0;

int failCount = 0;
const int maxFailCount = 5;

const unsigned long minBreathInterval = 1000;
const unsigned long maxBreathInterval = 10000;
const unsigned long apneaTimeout = 12000;

float getDistanceRaw();
float getFilteredDistance();
void drawRespScreen();
void drawNoEchoScreen();
void updateRespirationRate(unsigned long currentBreathTime);
void resetSensor();
void handleCaptiveRedirect();

const char index_html[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>RespMonitor-Pro</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif; }
        body { background-color: #030a16; color: #ffffff; padding: 15px; display: flex; flex-direction: column; align-items: center; min-height: 100vh; }
        .header { width: 100%; max-width: 500px; display: flex; justify-content: space-between; align-items: center; margin-bottom: 20px; border-bottom: 1px solid #112240; padding-bottom: 10px; }
        .header h1 { font-size: 1.1rem; font-weight: 500; letter-spacing: 0.5px; }
        .container { width: 100%; max-width: 500px; display: grid; grid-template-columns: 1fr; gap: 15px; }
        .card { background: #0a1931; border: 1px solid #172a45; border-radius: 12px; padding: 20px; display: flex; align-items: center; position: relative; }
        .card-icon { width: 45px; height: 45px; margin-right: 15px; display: flex; align-items: center; justify-content: center; }
        .card-info { display: flex; flex-direction: column; }
        .card-label { font-size: 0.75rem; text-transform: uppercase; color: #8892b0; letter-spacing: 1px; font-weight: bold; margin-bottom: 2px; }
        .card-sublabel { font-size: 0.7rem; color: #64748b; margin-top: -2px; }
        .card-value { font-size: 3rem; font-weight: 700; color: #ffffff; line-height: 1; margin-top: 5px; }
        .card-unit { font-size: 1.2rem; font-weight: 400; color: #8892b0; margin-left: 4px; }
        .chart-card { background: #0a1931; border: 1px solid #172a45; border-radius: 12px; padding: 15px; max-width: 500px; width: 100%; }
        canvas { width: 100%; height: 180px; display: block; background: #051124; border-radius: 8px; border: 1px solid #112240; }
    </style>
</head>
<body>

    <div class="header">
        <h1>☰ &nbsp; RespMonitor-Pro</h1>
        <span style="color:#64748b; font-size:1.2rem;">ⓘ</span>
    </div>

    <div class="container">
        <div class="card">
            <div class="card-icon">
                <svg width="40" height="40" viewBox="0 0 24 24" fill="none" stroke="#00b4d8" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round">
                    <path d="M12 4v7M12 11l3.5 2M12 11l-3.5 2" />
                    <path d="M15.5 13c1 0.8 2.5 1.5 3.5 0.5s1.2-3.5 0.5-5.5c-.5-1.5-1.5-3.5-3.5-4.5s-3.5-.5-4 .5M8.5 13c-1 0.8-2.5 1.5-3.5 0.5s-1.2-3.5-0.5-5.5c.5-1.5 1.5-3.5 3.5-4.5s3.5-.5 4 .5" />
                    <path d="M17 11.5c.8.6 1.8 1 2.5.2s.5-2.2 0-3.2c-.3-1-.8-2.2-2.2-2.8M7 11.5c-.8.6-1.8 1-2.5.2s-.5-2.2 0-3.2c.3-1 .8-2.2 2.2-2.8" />
                </svg>
            </div>
            <div class="card-info">
                <span class="card-label">RR</span>
                <span class="card-sublabel">Respiration Rate</span>
                <div class="card-value"><span id="rr-val">0</span></div>
            </div>
        </div>

        <div class="card">
            <div class="card-icon">
                <svg width="40" height="40" viewBox="0 0 24 24" fill="none" stroke="#00b4d8" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><polyline points="22 12 18 12 15 21 9 3 6 12 2 12"></polyline></svg>
            </div>
            <div class="card-info">
                <span class="card-label">VT</span>
                <span class="card-sublabel">Tidal Volume</span>
                <div class="card-value"><span id="vt-val">0</span><span class="card-unit">mL</span></div>
            </div>
        </div>

        <div class="chart-card">
            <canvas id="waveCanvas"></canvas>
        </div>
    </div>

    <script>
        const canvas = document.getElementById('waveCanvas');
        const ctx = canvas.getContext('2d');

        const dpr = window.devicePixelRatio || 1;
        canvas.width = canvas.offsetWidth * dpr;
        canvas.height = canvas.offsetHeight * dpr;
        ctx.scale(dpr, dpr);

        let dataPoints = [];
        const maxPoints = 60;
        for (let i = 0; i < maxPoints; i++) dataPoints.push(0);

        function drawChart() {
            const w = canvas.width / dpr;
            const h = canvas.height / dpr;
            ctx.clearRect(0, 0, w, h);

            ctx.strokeStyle = '#112240';
            ctx.lineWidth = 0.5;
            for (let i = 1; i < 4; i++) {
                let y = (h / 4) * i;
                ctx.beginPath();
                ctx.moveTo(0, y);
                ctx.lineTo(w, y);
                ctx.stroke();
            }

            ctx.beginPath();
            ctx.lineWidth = 3;
            ctx.strokeStyle = '#00b4d8';
            ctx.shadowBlur = 8;
            ctx.shadowColor = '#00b4d8';

            for (let i = 0; i < dataPoints.length; i++) {
                let x = (w / (maxPoints - 1)) * i;
                let normY = dataPoints[i] / 800.0;
                if (normY > 1) normY = 1;
                let y = h - (normY * (h - 20)) - 10;

                if (i == 0) ctx.moveTo(x, y);
                else ctx.lineTo(x, y);
            }

            ctx.stroke();
            ctx.shadowBlur = 0;
            requestAnimationFrame(drawChart);
        }

        setInterval(() => {
            fetch('/data')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('rr-val').innerText = data.rr;
                    document.getElementById('vt-val').innerText = data.vt;

                    dataPoints.push(data.vt_live);
                    if (dataPoints.length > maxPoints) dataPoints.shift();
                })
                .catch(err => console.log(err));
        }, 150);

        drawChart();
    </script>
</body>
</html>
)rawhtml";

void handleCaptiveRedirect() {
  server.sendHeader("Location", "http://192.168.4.1/", true);
  server.send(302, "text/plain", "");
}

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  u8g2.begin();

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(20, 22, "Resp");
  u8g2.drawStr(20, 42, "Monitor");
  u8g2.sendBuffer();
  delay(1500);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(apSSID, apPassword);
  Serial.print("Access Point started. SSID: ");
  Serial.println(apSSID);
  Serial.print("AP IP Address: ");
  Serial.println(WiFi.softAPIP());

  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());

  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html", index_html);
  });

  server.on("/data", HTTP_GET, []() {
    String json = "{\"rr\":" + String(respirationRate) +
                  ",\"vt\":" + String((int)tidalVolume) +
                  ",\"vt_live\":" + String((int)tidalVolume) + "}";
    server.send(200, "application/json", json);
  });

  server.onNotFound([]() {
    if (server.hostHeader() != "192.168.4.1") {
      handleCaptiveRedirect();
    } else {
      server.send_P(200, "text/html", index_html);
    }
  });

  server.begin();

  u8g2.clearBuffer();
  u8g2.drawStr(0, 18, "Keep chamber still");
  u8g2.drawStr(0, 36, "Finding baseline...");
  u8g2.sendBuffer();
  delay(2000);

  float sum = 0.0;
  int valid = 0;

  for (int i = 0; i < 20; i++) {
    float d = getDistanceRaw();
    if (d > 0) {
      sum += d;
      valid++;
    }
    delay(70);
  }

  if (valid > 0) {
    baselineDistance = sum / valid;
    filteredDistance = baselineDistance;
    lastValidDistance = baselineDistance;

    rrTriggerDistance = baselineDistance / 2.0;
    rrReleaseDistance = rrTriggerDistance + 0.6;

    sensorHealthy = true;
  } else {
    sensorHealthy = false;
  }

  u8g2.clearBuffer();
  u8g2.drawStr(0, 18, "Baseline ready");
  u8g2.setCursor(0, 38);
  u8g2.print("Base: ");
  u8g2.print(baselineDistance, 2);
  u8g2.print(" cm");
  u8g2.sendBuffer();
  delay(1500);
}

void loop() {
  dnsServer.processNextRequest();
  server.handleClient();

  filteredDistance = getFilteredDistance();

  if (!sensorHealthy) {
    drawNoEchoScreen();
    Serial.println("Sensor error: no echo");
    delay(100);
    return;
  }

  if (lastBreathTime > 0 && (millis() - lastBreathTime > apneaTimeout)) {
    respirationRate = 0;
    validIntervals = 0;
    intervalIndex = 0;
    for (int i = 0; i < intervalCount; i++) breathIntervals[i] = 0;
  }

  deltaDistance = baselineDistance - filteredDistance;
  if (deltaDistance < 0) deltaDistance = 0;
  if (deltaDistance > 20) deltaDistance = 20;

  tidalVolume = deltaDistance * calibrationFactor;

  unsigned long currentTime = millis();

  if (!breathDetected && filteredDistance <= rrTriggerDistance) {
    if (lastBreathTime == 0 || (currentTime - lastBreathTime) >= minBreathInterval) {
      breathDetected = true;
      updateRespirationRate(currentTime);
      lastBreathTime = currentTime;
      maxTidalVolume = tidalVolume;
      Serial.println("BREATH DETECTED");
    }
  }

  if (breathDetected) {
    if (tidalVolume > maxTidalVolume) {
      maxTidalVolume = tidalVolume;
    }

    if (filteredDistance >= rrReleaseDistance) {
      breathDetected = false;
      displayedTidalVolume = maxTidalVolume;
      maxTidalVolume = 0;
    }
  }

  drawRespScreen();

  Serial.print("Base: ");
  Serial.print(baselineDistance, 2);
  Serial.print(" cm | Distance: ");
  Serial.print(filteredDistance, 2);
  Serial.print(" cm | RR_Trig: ");
  Serial.print(rrTriggerDistance, 2);
  Serial.print(" cm | RR_Rel: ");
  Serial.print(rrReleaseDistance, 2);
  Serial.print(" cm | Delta: ");
  Serial.print(deltaDistance, 2);
  Serial.print(" cm | VT_live: ");
  Serial.print(tidalVolume, 1);
  Serial.print(" mL | RR: ");
  Serial.println(respirationRate);

  delay(10);
}

void updateRespirationRate(unsigned long currentBreathTime) {
  if (lastBreathTime > 0) {
    unsigned long breathInterval = currentBreathTime - lastBreathTime;

    if (breathInterval >= minBreathInterval && breathInterval <= maxBreathInterval) {
      breathIntervals[intervalIndex] = breathInterval;
      intervalIndex = (intervalIndex + 1) % intervalCount;

      if (validIntervals < intervalCount) {
        validIntervals++;
      }

      unsigned long totalInterval = 0;
      for (int i = 0; i < validIntervals; i++) {
        totalInterval += breathIntervals[i];
      }

      float averageInterval = (float)totalInterval / (float)validIntervals;
      respirationRate = (int)round(60000.0 / averageInterval);
    }
  }
}

void resetSensor() {
  pinMode(ECHO_PIN, OUTPUT);
  digitalWrite(ECHO_PIN, LOW);
  delay(50);
  pinMode(ECHO_PIN, INPUT);
  delay(50);
}

float getDistanceRaw() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000);

  if (duration == 0) {
    failCount++;
    if (failCount >= maxFailCount) {
      resetSensor();
      failCount = 0;
      sensorHealthy = false;
    }
    return -1;
  }

  float distance = duration * 0.0343 / 2.0;

  if (distance < 2.0 || distance > 400.0) {
    failCount++;
    if (failCount >= maxFailCount) {
      sensorHealthy = false;
    }
    return -1;
  }

  failCount = 0;
  sensorHealthy = true;
  lastValidDistance = distance;
  return distance;
}

float getFilteredDistance() {
  const int samples = 5;
  float vals[samples];
  int valid = 0;

  for (int i = 0; i < samples; i++) {
    float d = getDistanceRaw();
    if (d > 0) {
      vals[valid] = d;
      valid++;
    }
    delay(15);
  }

  if (valid == 0) {
    return lastValidDistance;
  }

  for (int i = 0; i < valid - 1; i++) {
    for (int j = i + 1; j < valid; j++) {
      if (vals[j] < vals[i]) {
        float t = vals[i];
        vals[i] = vals[j];
        vals[j] = t;
      }
    }
  }

  float median = vals[valid / 2];
  static float ema = 0.0;

  if (ema == 0.0) ema = median;
  ema = 0.75 * ema + 0.25 * median;

  return ema;
}

void drawRespScreen() {
  u8g2.clearBuffer();

  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(8, 12, "RR");
  u8g2.drawStr(74, 12, "VT");
  u8g2.drawLine(0, 16, 127, 16);

  u8g2.setFont(u8g2_font_logisoso18_tr);
  u8g2.setCursor(0, 42);
  u8g2.print(respirationRate);

  u8g2.setCursor(64, 42);
  u8g2.print(tidalVolume, 0);

  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.setCursor(92, 60);
  u8g2.print("mL");

  u8g2.sendBuffer();
}

void drawNoEchoScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(0, 16, "Sensor error");
  u8g2.drawStr(0, 32, "No Echo");
  u8g2.drawStr(0, 48, "Check wiring/power");
  u8g2.sendBuffer();
}
