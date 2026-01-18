/*
  Kompletny szkic dla ESP32:
  - pH (DFRobot_ESP_PH_WITH_ADC + ADS1115)
  - Kalibracja przez Serial: wpisuj polecenia "enterph", "calph", "exitph"
  - TFT_eSPI: bieżące pH + min/max + wykres (ostatnie 24h na wyświetlaczu)
  - Silnik krokowy (AccelStepper) – tylko przód albo STOP względem zadanego pH
  - Wi-Fi: jednocześnie AP + (opcjonalnie) STA (zapis SSID/hasła w NVS)
  - HTTP: wbudowana strona z wykresem ostatnich 7 dni (dane w SPIFFS)
  - OTA: aktualizacja po sieci (działa także w trybie AP)

  Płytka: ESP32 Dev Module
*/

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <SPIFFS.h>
#include <Preferences.h>
#include <time.h>
#include <sys/time.h>

#include <AccelStepper.h>
#include "DFRobot_ESP_PH_WITH_ADC.h"
#include <Adafruit_ADS1X15.h>
#include "EEPROM.h"
#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>

// ===================== KONFIG SIECIOWY =====================
const char* AP_SSID     = "ESP32-AP";
const char* AP_PASSWORD = "esp32pass";     // min. 8 znaków

const char* OTA_HOST    = "esp32-ph";
const char* OTA_PASS    = "otapass";

// Hardcoded Wi-Fi STA credentials (optional; overrides NVS if non-empty)
const char* WIFI_SSID = "PLAY_Swiatlowodowy_24";
const char* WIFI_PASS = "E1T8C&ZdAbwx";

// ===================== DANE / OKNO CZASOWE =====================
// I2C piny (domyślne dla ESP32) – GPIO22 = SCL, GPIO21 = SDA
#define I2C_SDA 21
#define I2C_SCL 22

static const time_t   WINDOW_SECONDS   = 7 * 24 * 60 * 60; // 7 dni (HTTP)
static const uint32_t SAMPLE_EVERY_MS  = 600000;           // 10 minut

// TFT
#define TFT_W 160
#define TFT_H 128
#define DATA_POINTS_TFT 144 // 24h przy próbkowaniu co 10 min

#define DATA_POINTS_HTTP 1008 // 7*24*6
const char* DATA_FILE = "/data.csv";      // CSV: epoch,value\n

// ===================== PH / CZUJNIKI =====================
DFRobot_ESP_PH_WITH_ADC ph;
Adafruit_ADS1115 ads;
bool    gADSOK   = false;
uint8_t gADSAddr = 0;

float voltage     = 0.0f;
float phValue     = 0.0f;
float temperature = 25.0f;

static const int PH_AVG_N = 30;
float phReadings[PH_AVG_N] = {0};
int   phReadingsIndex = 0;
int   phSamples       = 0;       // ile realnych próbek mamy w buforze (<=PH_AVG_N)
bool  phPrimed        = false;

float phAverage     = 0.0f;
float maxPhAverage  = 0.0f;
float minPhAverage  = 10.0f;

// Bufor wykresu 24h (TFT)
float phDataPointsTFT[DATA_POINTS_TFT] = {0};
int   dataIndexTFT = 0;

// ===================== SILNIK =====================
AccelStepper stepper(AccelStepper::DRIVER, 19, 18); // STEP=19, DIR=18
const float targetPH = 8.15f;

#define MOTOR_UPDATE_INTERVAL 1000
bool motorRunning = false;
int  motorDirection = 0; // 1=przód, 0=stop (brak wstecz)

// ===================== Piny dodatkowe =====================
// UWAGA: NIE używamy GPIO21/22 jako zwykłych wyjść, bo to I2C (SDA/SCL).
#define PIN_AUX 25     // przeniesione z 22 -> 25 (wybierz wolny GPIO jeśli 25 zajęty)
#define PIN_23  23

// ===================== GRAFIKA / TFT =====================
TFT_eSPI myGLCD = TFT_eSPI();

unsigned long lastUpdateTime      = 0;     // 1s – uśrednianie
const unsigned long updateInterval = 1000;

unsigned long lastDataUpdateTime  = 0;     // 10 min – zapis próbki
unsigned long lastMotorUpdateTime = 0;

// ===================== SIEC / OTA / HTTP =====================
WebServer server(80);
Preferences prefs;

// ===================== PROTOTYPY =====================
void i2cScan();
bool initI2CAndADS();

void setupWiFi();
void setupOTA();
void setupWeb();
void syncNTP();
bool haveValidTime();

void updatePHReading();
void calculatePHAverage();

void controlStepper(float phAvg);

void updateDataTFT(float v);
void drawGraphTFT();

void appendDataToFS(float value);
void pruneOldDataFS();
bool readDataJSON(String &outJson, time_t nowTs);

void handleRoot();
void handleData();
void handleAdd();
void handleWiFiGet();
void handleWiFiPost();
void handleTimeGet();
void handleTimePost();
void handleReboot();

// ===================== HTML =====================
static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="pl">
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width,initial-scale=1"/>
  <title>ESP32 pH – wykres 7 dni</title>
  <style>
    body{font-family:system-ui,Segoe UI,Roboto,Arial,sans-serif;margin:0;padding:16px;background:#0b0c10;color:#e6e7e8}
    .card{background:#15171c;border:1px solid #22252b;border-radius:16px;padding:16px;max-width:940px;margin:0 auto 24px;box-shadow:0 10px 20px rgba(0,0,0,.25)}
    h1{margin:0 0 8px;font-size:22px}
    .grid{display:grid;gap:12px;grid-template-columns:repeat(auto-fit,minmax(240px,1fr))}
    .muted{opacity:.75}
    canvas{width:100%;height:300px;display:block;background:#0f1116;border-radius:12px}
    button,input{background:#0f1116;color:#e6e7e8;border:1px solid #2a2f37;border-radius:10px;padding:8px 10px}
    a{color:#9ad}
    .row{display:flex;gap:8px;flex-wrap:wrap;align-items:center}
    .badge{display:inline-block;background:#0f1116;border:1px solid #2a2f37;border-radius:999px;padding:4px 10px}
    .small{font-size:12px;opacity:.8}
  </style>
</head>
<body>
  <div class="card">
    <h1>ESP32 – pH</h1>
    <div class="grid">
      <div>
        <div class="muted">Bieżąca wartość</div>
        <div id="current" style="font-size:28px;font-weight:700">—</div>
      </div>
      <div>
        <div class="muted">Okno</div>
        <div class="badge">ostatnie 7 dni</div>
      </div>
      <div>
        <div class="muted">Min / Max (7d)</div>
        <div id="minmax">—</div>
      </div>
    </div>
    <div class="small">Kalibracja przez Serial: <code>enterph</code> → <code>calph</code> → <code>exitph</code></div>
  </div>

  <div class="card">
    <div class="row" style="justify-content:space-between">
      <div class="muted">Wykres (ostatnie 7 dni)</div>
      <div class="row">
        <form id="addForm" class="row" onsubmit="return addSample(event)">
          <input id="val" type="number" step="any" placeholder="Nowa próbka"/>
          <button type="submit">Dodaj</button>
        </form>
        <a href="/wifi">Wi-Fi</a>
        <a href="/time">Czas</a>
      </div>
    </div>
    <canvas id="chart" width="1000" height="300"></canvas>
  </div>

<script>
async function fetchData(){
  const r = await fetch('/api/data');
  const j = await r.json();
  drawChart(j.points||[]);
  if(j.points && j.points.length){
    const last = j.points[j.points.length-1];
    const vv = Number(last.v).toFixed(2);
    document.getElementById('current').textContent = vv;
    const vals = j.points.map(p=>Number(p.v));
    const mn = Math.min(...vals), mx = Math.max(...vals);
    document.getElementById('minmax').textContent = mn.toFixed(2)+" / "+mx.toFixed(2);
  } else {
    document.getElementById('current').textContent = '—';
    document.getElementById('minmax').textContent = '—';
  }
}

function drawChart(points){
  const c = document.getElementById('chart');
  const ctx = c.getContext('2d');
  ctx.clearRect(0,0,c.width,c.height);
  if(!points.length){
    ctx.fillStyle = '#9aa';
    ctx.fillText('Brak danych', 20, 30);
    return;
  }
  const m = {l:50,r:10,t:10,b:30};
  const W = c.width - m.l - m.r;
  const H = c.height - m.t - m.b;
  const xs = points.map(p=>p.t);
  const ys = points.map(p=>Number(p.v));
  const tmin = Math.min(...xs), tmax = Math.max(...xs);
  const ymin = Math.min(...ys), ymax = Math.max(...ys);
  const xscale = x => m.l + ( (x - tmin) / Math.max(1,(tmax - tmin)) ) * W;
  const yscale = y => m.t + H - ( (y - ymin) / Math.max(1e-9,(ymax - ymin||1)) ) * H;

  // siatka X – co 1 dzień
  const day = 86400;
  const startDay = Math.floor(tmin/day)*day;
  ctx.strokeStyle = '#22252b';
  ctx.fillStyle = '#9aa';
  ctx.beginPath();
  for(let t = startDay; t <= tmax + day; t += day){
    const x = xscale(t);
    ctx.moveTo(x, m.t);
    ctx.lineTo(x, m.t+H);
  }
  ctx.stroke();
  ctx.font = '12px system-ui';
  ctx.textAlign = 'center';
  for(let t = startDay; t <= tmax + day; t += day){
    const x = xscale(t);
    const d = new Date(t*1000);
    const lab = (d.getMonth()+1)+"/"+d.getDate();
    ctx.fillText(lab, x, m.t+H+20);
  }

  // Y – min/max
  ctx.textAlign = 'right';
  ctx.fillText(ymax.toFixed(2), m.l-6, m.t+10);
  ctx.fillText(ymin.toFixed(2), m.l-6, m.t+H);

  // linia
  ctx.beginPath();
  points.forEach((p,i)=>{
    const x = xscale(p.t), y = yscale(Number(p.v));
    if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
  });
  ctx.strokeStyle = '#8ab4f8';
  ctx.lineWidth = 2;
  ctx.stroke();

  // punkty
  ctx.fillStyle = '#c7d1e0';
  points.forEach(p=>{
    const x = xscale(p.t), y = yscale(Number(p.v));
    ctx.beginPath();
    ctx.arc(x,y,2.5,0,Math.PI*2);
    ctx.fill();
  });
}

async function addSample(e){
  e.preventDefault();
  const v = document.getElementById('val').value;
  if(!v) return false;
  await fetch('/api/add?value='+encodeURIComponent(v));
  document.getElementById('val').value='';
  fetchData();
  return false;
}

fetchData();
setInterval(fetchData, 15000);
</script>
</body>
</html>
)HTML";

// ===================== POMOC / CZAS =====================
bool haveValidTime(){
  time_t nowTs; time(&nowTs);
  return nowTs > 1720000000; // ~2024-07 – proste sprawdzenie czy NTP ustawione
}

void syncNTP(){
  setenv("TZ","CET-1CEST,M3.5.0/2,M10.5.0/3",1); // Europa/Warszawa
  tzset();
  configTime(0,0, "pool.ntp.org", "time.nist.gov");
  for (int i=0;i<20;i++) {
    if (haveValidTime()) break;
    delay(500);
  }
}

// ===================== I2C / ADS =====================
void i2cScan(){
  Serial.println("I2C scan start");
  byte count = 0;
  for (byte addr=1; addr<127; addr++){
    Wire.beginTransmission(addr);
    byte err = Wire.endTransmission();
    if (err == 0){
      Serial.printf("  -> I2C device @ 0x%02X\n", addr);
      count++;
    }
  }
  if (!count) Serial.println("  (brak urz.)");
  Serial.println("I2C scan end");
}

bool initI2CAndADS(){
  struct PinSet { int sda; int scl; };
  const PinSet sets[] = { {I2C_SDA, I2C_SCL}, {16,17}, {32,33} };
  const uint32_t speeds[] = {100000, 50000, 10000};
  const uint8_t addrs[] = {0x48, 0x49, 0x4A, 0x4B};

  for (auto &ps : sets){
    for (auto spd : speeds){
      Wire.end();
      Wire.begin(ps.sda, ps.scl);
      Wire.setClock(spd);
      delay(5);

      Serial.println();
      Serial.printf("[I2C] Init on SDA=%d SCL=%d @ %lu Hz\n", ps.sda, ps.scl, (unsigned long)spd);

      i2cScan();
      for (auto a : addrs){
        Serial.printf("[ADS] Proba pod adresem 0x%02X... ", a);
        if (ads.begin(a)){
          Serial.println("OK");
          ads.setGain(GAIN_ONE);
          gADSOK = true;
          gADSAddr = a;
          return true;
        } else {
          Serial.println("brak");
        }
      }
    }
  }
  return false;
}

// ===================== FS: zapis/odczyt =====================
void appendDataToFS(float value){
  if(!SPIFFS.exists(DATA_FILE)){
    File f = SPIFFS.open(DATA_FILE, FILE_WRITE);
    if(!f){ Serial.println("! Nie mogę utworzyć pliku danych"); return; }
    f.println("epoch,value");
    f.close();
  }

  time_t nowTs; time(&nowTs);
  File f = SPIFFS.open(DATA_FILE, FILE_APPEND);
  if(!f){ Serial.println("! Nie mogę otworzyć pliku do zapisu"); return; }
  f.printf("%ld,%.6f\n", (long)nowTs, value);
  f.close();
  pruneOldDataFS();
}

void pruneOldDataFS(){
  if(!SPIFFS.exists(DATA_FILE)) return;

  File in = SPIFFS.open(DATA_FILE, FILE_READ);
  if(!in) return;

  (void)in.readStringUntil('\n'); // header

  time_t nowTs; time(&nowTs);
  const time_t minTs = nowTs - WINDOW_SECONDS - 3600; // bufor

  File out = SPIFFS.open("/tmp.csv", FILE_WRITE);
  if(!out){ in.close(); return; }
  out.println("epoch,value");

  while(in.available()){
    String line = in.readStringUntil('\n');
    line.trim();
    if(!line.length()) continue;

    int c = line.indexOf(',');
    if(c<0) continue;

    time_t ts = strtol(line.substring(0,c).c_str(), nullptr, 10);
    if(ts >= minTs) out.println(line);
  }

  in.close();
  out.close();

  SPIFFS.remove(DATA_FILE);
  SPIFFS.rename("/tmp.csv", DATA_FILE);
}

bool readDataJSON(String &outJson, time_t nowTs){
  if(!SPIFFS.exists(DATA_FILE)){
    outJson = "{\"points\":[]}";
    return true;
  }

  File f = SPIFFS.open(DATA_FILE, FILE_READ);
  if(!f) return false;

  (void)f.readStringUntil('\n'); // header

  outJson = "{\"points\":[";
  bool first = true;
  const time_t minTs = nowTs - WINDOW_SECONDS;

  while(f.available()){
    String line = f.readStringUntil('\n');
    line.trim();
    if(!line.length()) continue;

    int c = line.indexOf(',');
    if(c<0) continue;

    time_t ts = strtol(line.substring(0,c).c_str(), nullptr, 10);
    if(ts < minTs) continue;

    String val = line.substring(c+1);
    val.trim();

    if(!first) outJson += ",";
    first = false;

    outJson += "{\"t\":" + String((long)ts) + ",\"v\":" + val + "}";
  }

  outJson += "]}";
  f.close();
  return true;
}

// ===================== HTTP HANDLERY =====================
void handleRoot(){ server.send(200, "text/html; charset=utf-8", INDEX_HTML); }

void handleData(){
  time_t nowTs; time(&nowTs);
  String json;
  if(!readDataJSON(json, nowTs)){
    server.send(500, "application/json", "{\"error\":\"read fail\"}");
    return;
  }
  server.send(200, "application/json", json);
}

void handleAdd(){
  if(!server.hasArg("value")){
    server.send(400, "application/json", "{\"error\":\"missing value\"}");
    return;
  }
  float v = server.arg("value").toFloat();
  appendDataToFS(v);
  updateDataTFT(v);
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleWiFiGet(){
  String html = F(
    "<html><head><meta name='viewport' content='width=device-width,initial-scale=1'><title>Wi-Fi</title></head>"
    "<body style='font-family:sans-serif;padding:16px'>"
    "<h2>Konfiguracja Wi-Fi (STA)</h2>"
    "<form method='POST'>SSID:<br><input name='ssid'/><br>"
    "Hasło:<br><input name='pass' type='password'/><br><br>"
    "<button>Zapisz i zrestartuj</button></form>"
    "<p><a href='/'>&larr; Wróć</a></p>"
    "</body></html>"
  );
  server.send(200, "text/html; charset=utf-8", html);
}

void handleWiFiPost(){
  if(server.hasArg("ssid") && server.hasArg("pass")){
    prefs.begin("wifi", false);
    prefs.putString("ssid", server.arg("ssid"));
    prefs.putString("pass", server.arg("pass"));
    prefs.end();
    server.send(200, "text/html; charset=utf-8", "Zapisano. Restart za 2s...");
    delay(2000);
    ESP.restart();
  } else {
    server.send(400, "text/plain", "Brak ssid/pass");
  }
}

void handleTimeGet(){
  time_t nowTs; time(&nowTs);
  String html = "<html><head><meta name='viewport' content='width=device-width,initial-scale=1'><title>Czas</title></head>"
                "<body style='font-family:sans-serif;padding:16px'>";
  html += "<h2>Ustaw czas</h2>";
  html += "<p>Aktualny epoch: <b>" + String((long)nowTs) + "</b></p>";
  html += "<form method='POST'>Epoch (sekundy):<br><input name='epoch' type='number'/><br><br><button>Ustaw</button></form>";
  html += "<p><a href='/'>&larr; Wróć</a></p></body></html>";
  server.send(200, "text/html; charset=utf-8", html);
}

void handleTimePost(){
  if(server.hasArg("epoch")){
    time_t e = strtoul(server.arg("epoch").c_str(), nullptr, 10);
    struct timeval tv; tv.tv_sec = e; tv.tv_usec = 0;
    settimeofday(&tv, nullptr);
    server.send(200, "text/plain", "Czas ustawiony");
  } else {
    server.send(400, "text/plain", "Brak epoch");
  }
}

void handleReboot(){
  server.send(200, "text/plain", "Restart...");
  delay(500);
  ESP.restart();
}

// ===================== SIEC/OTA =====================
void setupWiFi(){
  prefs.begin("wifi", true);
  String ssid = prefs.getString("ssid", "");
  String pass = prefs.getString("pass", "");
  prefs.end();

  // Override z wpisanych na stałe danych i zapisz do NVS
  if (WIFI_SSID && WIFI_SSID[0]) {
    ssid = WIFI_SSID;
    pass = WIFI_PASS ? WIFI_PASS : "";
    prefs.begin("wifi", false);
    prefs.putString("ssid", ssid);
    prefs.putString("pass", pass);
    prefs.end();
    Serial.println("Używam wpisanych na stałe danych Wi-Fi (STA) i zapisuję do NVS.");
  }

  WiFi.persistent(false);
  WiFi.setSleep(false);
  WiFi.mode(WIFI_AP_STA);

  IPAddress apIP(192,168,4,1), apGW(192,168,4,1), apNM(255,255,255,0);
  WiFi.softAPConfig(apIP, apGW, apNM);

  bool apok = WiFi.softAP(AP_SSID, AP_PASSWORD, 6, false, 4);
  delay(200);
  Serial.printf("AP start: %s\n", apok ? "OK" : "FAIL");
  Serial.printf("AP SSID: %s\n", AP_SSID);
  Serial.print("AP IP:   "); Serial.println(WiFi.softAPIP());

  if(ssid.length()){
    Serial.print("Łączenie STA: "); Serial.println(ssid);
    WiFi.begin(ssid.c_str(), pass.c_str());
    uint32_t t0 = millis();
    while(WiFi.status()!=WL_CONNECTED && millis()-t0 < 15000){
      delay(250);
      Serial.print('.');
    }
    Serial.println();
    if(WiFi.status()==WL_CONNECTED){
      Serial.print("STA IP: "); Serial.println(WiFi.localIP());
      syncNTP();
    } else {
      Serial.println("Nie połączono z STA (pozostaje AP)");
    }
  }
}

void setupOTA(){
  ArduinoOTA.setHostname(OTA_HOST);
  if(String(OTA_PASS).length()) ArduinoOTA.setPassword(OTA_PASS);

  ArduinoOTA.onStart([](){ Serial.println("OTA Start"); });
  ArduinoOTA.onEnd([](){ Serial.println("\nOTA End"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total){
    unsigned int pct = (total == 0) ? 0 : (progress * 100U) / total;
    Serial.printf("OTA: %u%%\r", pct);
  });
  ArduinoOTA.onError([](ota_error_t error){
    Serial.printf("OTA Error[%u]\n", error);
  });

  ArduinoOTA.begin();
  Serial.println("OTA gotowe");
}

void setupWeb(){
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/data", HTTP_GET, handleData);
  server.on("/api/add", HTTP_GET, handleAdd);

  server.on("/wifi", HTTP_GET, handleWiFiGet);
  server.on("/wifi", HTTP_POST, handleWiFiPost);

  server.on("/time", HTTP_GET, handleTimeGet);
  server.on("/time", HTTP_POST, handleTimePost);

  server.on("/reboot", HTTP_GET, handleReboot);

  server.on("/ping", HTTP_GET, [](){ server.send(200, "text/plain", "pong"); });
  server.on("/generate_204", HTTP_GET, [](){ server.send(204); });
  server.on("/hotspot-detect.html", HTTP_GET, [](){ server.send(200, "text/html", "<html><body>OK</body></html>"); });

  server.onNotFound([](){
    String msg = String("404: ") + server.uri();
    server.send(404, "text/plain", msg);
  });

  server.begin();
  Serial.println("HTTP serwer start");
}

// ===================== LOGIKA PH =====================
void calculatePHAverage(){
  if(phSamples <= 0){
    phAverage = 0.0f;
    return;
  }

  float sum = 0.0f;
  for (int i=0;i<phSamples;i++) sum += phReadings[i];
  phAverage = sum / (float)phSamples;

  // Aktualizuj min/max dopiero gdy bufor pełny i zrobiliśmy pełny cykl 30 próbek
  if (phSamples == PH_AVG_N && phReadingsIndex == 0){
    if (phAverage > maxPhAverage) maxPhAverage = phAverage;
    if (phAverage < minPhAverage) minPhAverage = phAverage;
  }
}

void updatePHReading(){
  if(!gADSOK){
    myGLCD.setCursor(0, 0);
    myGLCD.setTextColor(TFT_YELLOW, TFT_BLACK);
    myGLCD.setTextSize(1);
    myGLCD.print("ADS: N/A  ");
    return;
  }

  // Zostawiamy Twoje przeliczenie, by nie rozjechać kalibracji biblioteki DFRobot
  voltage = ads.readADC_SingleEnded(1) / 10.0f;
  phValue = ph.readPH(voltage, temperature);

  // Prime bufora (brak zer na starcie)
  if(!phPrimed){
    for(int i=0;i<PH_AVG_N;i++) phReadings[i] = phValue;
    phPrimed = true;
    phSamples = PH_AVG_N;
    phReadingsIndex = 0;
  } else {
    phReadings[phReadingsIndex++] = phValue;
    if (phReadingsIndex >= PH_AVG_N) phReadingsIndex = 0;
    if (phSamples < PH_AVG_N) phSamples++;
  }

  calculatePHAverage();

  // Pasek tekstowy
  myGLCD.setCursor(0, 0);
  myGLCD.setTextColor(TFT_WHITE, TFT_BLACK);
  myGLCD.setTextSize(1);
  myGLCD.printf("P: %.2f  ", phAverage);
  myGLCD.printf("MA: %.2f  ", maxPhAverage);
  myGLCD.printf("MI: %.2f  ", minPhAverage);
}

// ===================== SILNIK: tylko przód albo STOP =====================
void controlStepper(float phAvg){
  const float deadband = 0.01f;     // histereza
  const float runSpeed = 250.0f;    // tylko do przodu

  if(!gADSOK || !phPrimed){
    stepper.setSpeed(0);
    motorRunning = false;
    motorDirection = 0;
    return;
  }

  if (phAvg < (targetPH - deadband)) {
    stepper.setSpeed(runSpeed);     // przód
    motorRunning = true;
    motorDirection = 1;
  } else {
    stepper.setSpeed(0);            // STOP (również gdy phAvg > targetPH)
    motorRunning = false;
    motorDirection = 0;
  }
}

// ===================== TFT: bufor 24h =====================
void updateDataTFT(float v){
  phDataPointsTFT[dataIndexTFT++] = v;

  if (dataIndexTFT >= DATA_POINTS_TFT) {
    dataIndexTFT = 0;

    // Reset do nowej doby (zgodnie z Twoją intencją)
    maxPhAverage = 0.0f;
    minPhAverage = 10.0f;
  }

  drawGraphTFT();
}

void drawGraphTFT(){
  const int graphWidth  = 144;
  const int graphHeight = 100;
  const int yOffset     = 18;
  const int xOffset     = 15;

  const float maxPH = 8.5f;
  const float minPH = 7.5f;

  // Czyść obszar wykresu (bez paska tekstowego)
  myGLCD.fillRect(0, yOffset, TFT_W, graphHeight + 14, TFT_BLACK);

  // Siatka co 0.10 pH
  myGLCD.setTextSize(1);
  for (int i=0;i<=10;i++){
    float currentPH = minPH + i*0.10f;
    int y = yOffset + graphHeight - (int)((currentPH - minPH)/(maxPH-minPH)*graphHeight);
    myGLCD.drawFastHLine(xOffset, y, graphWidth, TFT_DARKGREY);

    myGLCD.setCursor(xOffset + graphWidth + 1, y - 6);
    myGLCD.setTextColor(TFT_WHITE, TFT_BLACK);
    myGLCD.printf("%d", (int)(currentPH*10)); // etykieta *10
  }

  if (dataIndexTFT <= 1) return;

  // Skala X: od 0..(dataIndexTFT-1) rozciągnięte na graphWidth
  const float xScale = (dataIndexTFT <= 1) ? 0.0f : (float)(graphWidth - 1) / (float)(dataIndexTFT - 1);

  auto yMap = [&](float pHval)->int {
    float clamped = pHval;
    if(clamped < minPH) clamped = minPH;
    if(clamped > maxPH) clamped = maxPH;
    return yOffset + graphHeight - (int)((clamped - minPH)/(maxPH-minPH)*graphHeight);
  };

  int prevX = xOffset;
  int prevY = yMap(phDataPointsTFT[0]);

  for (int i=1;i<dataIndexTFT;i++){
    int x = xOffset + (int)roundf((float)i * xScale);
    int y = yMap(phDataPointsTFT[i]);
    myGLCD.drawLine(prevX, prevY, x, y, TFT_WHITE);
    prevX = x;
    prevY = y;
  }
}

// ===================== SETUP / LOOP =====================
void setup(){
  Serial.begin(115200);
  delay(200);
  Serial.println("Boot...");

  EEPROM.begin(32);

  if(!SPIFFS.begin(true)) Serial.println("! Błąd SPIFFS");

  myGLCD.init();
  myGLCD.setRotation(3);
  myGLCD.fillScreen(TFT_BLACK);

  // Piny dodatkowe (NIE używamy 21/22)
  pinMode(PIN_AUX, OUTPUT);
  pinMode(PIN_23, OUTPUT);
  digitalWrite(PIN_23, HIGH);
  digitalWrite(PIN_AUX, HIGH);

  // Silnik
  stepper.setMaxSpeed(2000);
  stepper.setSpeed(0); // start STOP

  // pH
  ph.begin();

  // I2C/ADS
  if(!initI2CAndADS()){
    Serial.println("! ADS1115 nie wykryty (sprawdź 3.3V/GND/SDA/SCL/ADDR)");
    myGLCD.setCursor(0, 10);
    myGLCD.setTextColor(TFT_RED, TFT_BLACK);
    myGLCD.print("ADS FAIL");
  }

  // Sieć + serwer + OTA
  setupWiFi();
  setupWeb();
  setupOTA();

  myGLCD.setCursor(0, 0);
  myGLCD.setTextColor(TFT_WHITE, TFT_BLACK);
  myGLCD.print("Start OK");
}

void loop(){
  unsigned long nowMs = millis();

  // Odczyt/średnia co 1s
  if (nowMs - lastUpdateTime >= updateInterval){
    lastUpdateTime = nowMs;
    updatePHReading();
  }

  // Co 10 min: próbka do wykresu + FS
  if (nowMs - lastDataUpdateTime >= SAMPLE_EVERY_MS){
    lastDataUpdateTime = nowMs;
    if (gADSOK && phPrimed){
      updateDataTFT(phAverage);
      appendDataToFS(phAverage);
    }
  }

  // Silnik co MOTOR_UPDATE_INTERVAL
  if (nowMs - lastMotorUpdateTime >= MOTOR_UPDATE_INTERVAL){
    lastMotorUpdateTime = nowMs;
    controlStepper(phAverage);
  }

  // Stepper (runSpeed działa dla speed=0 => nic nie robi)
  stepper.runSpeed();

  // HTTP/OTA
  server.handleClient();
  ArduinoOTA.handle();

  // Kalibracja – komendy z Serial
  ph.calibration(voltage, temperature);
}
