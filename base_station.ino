/*
 *  base_station.ino  —  Raccoglitore statistiche badge Pamali
 *
 *  Hardware: Heltec WiFi LoRa 32 V3 (ESP32-S3 + OLED SSD1306 128x64)
 *
 *  Ascolta i beacon BLE di tutti i badge, tiene una TABELLA per-ID
 *  persistente su LittleFS (sopravvive ai riavvii). Se un badge ripassa,
 *  riconosce l'ID e aggiorna le sue statistiche (visite, persone conosciute...).
 *  Mostra lo stato sull'OLED e i dettagli sul monitor seriale.
 *
 *  Librerie (Library Manager):
 *    NimBLE-Arduino  ≥ 2.0
 *    U8g2            (oleddisplay)
 *
 *  Comandi seriali: d=dump tabella CSV   s=salva ora   r=RESET (cancella tutto)
 */

#include <NimBLEDevice.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <WebServer.h>

// ── Access Point WiFi (il telefono si collega qui) ──────
const char* AP_SSID = "PAMALI-STATS";
const char* AP_PASS = "pamali2026";     // min 8 caratteri (vuoto "" = aperto)
WebServer   server(80);

// ── OLED Heltec V3 ──────────────────────────────────────
#define VEXT_PIN   36     // alimentazione periferiche (LOW = ON)
#define OLED_RST   21
#define OLED_SDA   17
#define OLED_SCL   18
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, OLED_RST, OLED_SCL, OLED_SDA);

// ── Protocollo badge (DEVE combaciare col badge!) ───────
static const uint16_t COMPANY_ID = 0xFF01;
static const uint8_t  PROTO_VER  = 1;

struct __attribute__((packed)) BeaconPayload {
  uint16_t companyId;
  uint8_t  proto;
  uint8_t  type;        // bit0 = cuore/sole, bit7 = creatore
  uint8_t  id[3];
  uint8_t  hue;
  uint8_t  mood;
  uint8_t  phase;
  uint32_t tConsensus;
  uint16_t peopleMet;
};

// ── Tabella statistiche per badge ───────────────────────
#define MAX_BADGES   200
#define VISIT_GAP_MS 60000UL     // assenza oltre 60s = nuova "visita"
#define PRESENT_MS   8000UL      // visto negli ultimi 8s = presente ora

struct __attribute__((packed)) BadgeStat {
  uint32_t id;
  uint8_t  type;
  uint8_t  lastMood;
  uint8_t  lastHue;
  uint16_t peopleMet;
  uint16_t visits;
  uint32_t firstSeenMs;
  uint32_t lastSeenMs;
  int8_t   lastRssi;
};

BadgeStat badges[MAX_BADGES];
uint16_t  badgeCount = 0;
volatile bool dirty   = false;        // dati da salvare
SemaphoreHandle_t tblMutex = nullptr;

const char* STATS_FILE = "/stats.bin";

// ═══════════════════════════════════════════════════════
//  PERSISTENZA (LittleFS)
// ═══════════════════════════════════════════════════════
void saveStats() {
  if (xSemaphoreTake(tblMutex, pdMS_TO_TICKS(50)) != pdTRUE) return;
  File f = LittleFS.open(STATS_FILE, "w");
  if (f) {
    f.write((uint8_t*)&badgeCount, sizeof(badgeCount));
    f.write((uint8_t*)badges, sizeof(BadgeStat) * badgeCount);
    f.close();
    Serial.printf("[FS] salvati %u badge\n", badgeCount);
  }
  dirty = false;
  xSemaphoreGive(tblMutex);
}

void loadStats() {
  if (!LittleFS.exists(STATS_FILE)) { Serial.println("[FS] nessun dato precedente"); return; }
  File f = LittleFS.open(STATS_FILE, "r");
  if (!f) return;
  f.read((uint8_t*)&badgeCount, sizeof(badgeCount));
  if (badgeCount > MAX_BADGES) badgeCount = MAX_BADGES;
  f.read((uint8_t*)badges, sizeof(BadgeStat) * badgeCount);
  f.close();
  Serial.printf("[FS] caricati %u badge dal riavvio precedente\n", badgeCount);
}

// ═══════════════════════════════════════════════════════
//  AGGIORNAMENTO TABELLA
// ═══════════════════════════════════════════════════════
int findBadge(uint32_t id) {
  for (uint16_t i = 0; i < badgeCount; i++)
    if (badges[i].id == id) return i;
  return -1;
}

void updateBadge(uint32_t id, const BeaconPayload* p, int8_t rssi) {
  if (xSemaphoreTake(tblMutex, pdMS_TO_TICKS(10)) != pdTRUE) return;
  uint32_t now = millis();
  int idx = findBadge(id);
  if (idx < 0) {
    if (badgeCount >= MAX_BADGES) { xSemaphoreGive(tblMutex); return; }
    idx = badgeCount++;
    badges[idx] = {};
    badges[idx].id          = id;
    badges[idx].firstSeenMs = now;
    badges[idx].visits      = 1;
    Serial.printf("[NEW] badge %06X (totale %u)\n", id, badgeCount);
  } else {
    // nuova visita se era assente da un po'
    if (now - badges[idx].lastSeenMs > VISIT_GAP_MS) badges[idx].visits++;
  }
  badges[idx].type       = p->type;
  badges[idx].lastMood   = p->mood;
  badges[idx].lastHue    = p->hue;
  badges[idx].peopleMet  = p->peopleMet;
  badges[idx].lastSeenMs = now;
  badges[idx].lastRssi   = rssi;
  dirty = true;
  xSemaphoreGive(tblMutex);
}

// ═══════════════════════════════════════════════════════
//  BLE SCAN
// ═══════════════════════════════════════════════════════
class CollectorCB : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* dev) override {
    if (!dev->haveManufacturerData()) return;
    std::string mfg = dev->getManufacturerData();
    if (mfg.size() < sizeof(BeaconPayload)) return;
    const BeaconPayload* p = (const BeaconPayload*)mfg.data();
    if (p->companyId != COMPANY_ID || p->proto != PROTO_VER) return;
    uint32_t id = ((uint32_t)p->id[0] << 16) | ((uint32_t)p->id[1] << 8) | p->id[2];
    updateBadge(id, p, (int8_t)dev->getRSSI());
  }
};

// ═══════════════════════════════════════════════════════
//  STATISTICHE AGGREGATE
// ═══════════════════════════════════════════════════════
struct Agg {
  uint16_t total, present;
  float    avgMet;
  uint16_t maxMet; uint32_t maxId;
  uint16_t cuori, soli, creatori;
  uint16_t moodCnt[3];
};

Agg computeAgg() {
  Agg a = {};
  if (xSemaphoreTake(tblMutex, pdMS_TO_TICKS(20)) != pdTRUE) return a;
  uint32_t now = millis();
  uint32_t sumMet = 0;
  for (uint16_t i = 0; i < badgeCount; i++) {
    a.total++;
    if (now - badges[i].lastSeenMs < PRESENT_MS) a.present++;
    sumMet += badges[i].peopleMet;
    if (badges[i].peopleMet > a.maxMet) { a.maxMet = badges[i].peopleMet; a.maxId = badges[i].id; }
    if (badges[i].type & 0x01) a.soli++; else a.cuori++;
    if (badges[i].type & 0x80) a.creatori++;
    uint8_t m = badges[i].lastMood; if (m < 3) a.moodCnt[m]++;
  }
  a.avgMet = a.total ? (float)sumMet / a.total : 0.0f;
  xSemaphoreGive(tblMutex);
  return a;
}

// istogramma incontri: bucket 0, 1-2, 3-5, 6-10, 11+
void computeHist(uint16_t h[5]) {
  for (int i = 0; i < 5; i++) h[i] = 0;
  if (xSemaphoreTake(tblMutex, pdMS_TO_TICKS(20)) != pdTRUE) return;
  for (uint16_t i = 0; i < badgeCount; i++) {
    uint16_t m = badges[i].peopleMet;
    if      (m == 0) h[0]++;
    else if (m <= 2) h[1]++;
    else if (m <= 5) h[2]++;
    else if (m <= 10) h[3]++;
    else h[4]++;
  }
  xSemaphoreGive(tblMutex);
}

// ═══════════════════════════════════════════════════════
//  OLED — pagine a rotazione
// ═══════════════════════════════════════════════════════
void drawHistogram() {
  uint16_t h[5]; computeHist(h);
  uint16_t mx = 1;
  for (int i = 0; i < 5; i++) if (h[i] > mx) mx = h[i];
  const char* lbl[5] = {"0", "1-2", "3-5", "6-10", "11+"};
  int x0 = 6, bw = 20, gap = 4, baseY = 56, maxH = 38;
  u8g2.setFont(u8g2_font_5x7_tf);
  for (int i = 0; i < 5; i++) {
    int bh = (int)((long)h[i] * maxH / mx);
    int x  = x0 + i * (bw + gap);
    u8g2.drawBox(x, baseY - bh, bw, bh);
    char n[6]; snprintf(n, sizeof(n), "%u", h[i]);
    u8g2.drawStr(x + 4, baseY - bh - 1, n);     // valore sopra la barra
    u8g2.drawStr(x + 1, 63, lbl[i]);            // etichetta sotto
  }
}

void drawDisplay() {
  Agg a = computeAgg();
  uint8_t page = (millis() / 4000) % 6;   // cambia pagina ogni 4s (6 pagine)

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 10, "PAMALI  -  stats");
  u8g2.drawHLine(0, 13, 128);

  char buf[40];
  switch (page) {
    case 0:
      u8g2.setFont(u8g2_font_logisoso16_tf);
      snprintf(buf, sizeof(buf), "%u", a.total);
      u8g2.drawStr(0, 40, buf);
      u8g2.setFont(u8g2_font_6x12_tf);
      u8g2.drawStr(0, 52, "badge totali");
      snprintf(buf, sizeof(buf), "presenti ora: %u", a.present);
      u8g2.drawStr(0, 63, buf);
      break;
    case 1:
      u8g2.setFont(u8g2_font_logisoso16_tf);
      snprintf(buf, sizeof(buf), "%.1f", a.avgMet);
      u8g2.drawStr(0, 40, buf);
      u8g2.setFont(u8g2_font_6x12_tf);
      u8g2.drawStr(0, 52, "media incontri");
      snprintf(buf, sizeof(buf), "record: %u (%06X)", a.maxMet, a.maxId);
      u8g2.drawStr(0, 63, buf);
      break;
    case 2:
      u8g2.drawStr(0, 28, "MOOD ora:");
      snprintf(buf, sizeof(buf), "chill  %u", a.moodCnt[0]); u8g2.drawStr(4, 40, buf);
      snprintf(buf, sizeof(buf), "social %u", a.moodCnt[1]); u8g2.drawStr(4, 51, buf);
      snprintf(buf, sizeof(buf), "party  %u", a.moodCnt[2]); u8g2.drawStr(4, 62, buf);
      break;
    case 3:
      snprintf(buf, sizeof(buf), "Cuori:  %u", a.cuori);    u8g2.drawStr(0, 30, buf);
      snprintf(buf, sizeof(buf), "Soli:   %u", a.soli);     u8g2.drawStr(0, 44, buf);
      snprintf(buf, sizeof(buf), "Creatori: %u", a.creatori); u8g2.drawStr(0, 58, buf);
      break;
    case 4:
      u8g2.setFont(u8g2_font_5x7_tf);
      u8g2.drawStr(28, 22, "incontri/badge");
      drawHistogram();
      break;
    case 5:
      u8g2.drawStr(0, 28, "Vedi le statistiche:");
      u8g2.drawStr(0, 42, "WiFi: PAMALI-STATS");
      u8g2.drawStr(0, 54, "http://192.168.4.1");
      break;
  }
  u8g2.sendBuffer();
}

// ═══════════════════════════════════════════════════════
//  SERIALE
// ═══════════════════════════════════════════════════════
void dumpCSV() {
  Serial.println("\nid,tipo,creatore,mood,hue,peopleMet,visite,ultimoRSSI,present");
  uint32_t now = millis();
  for (uint16_t i = 0; i < badgeCount; i++) {
    BadgeStat& b = badges[i];
    Serial.printf("%06X,%s,%d,%u,%u,%u,%u,%d,%d\n",
      b.id, (b.type & 1) ? "sole" : "cuore", (b.type & 0x80) ? 1 : 0,
      b.lastMood, b.lastHue, b.peopleMet, b.visits, b.lastRssi,
      (now - b.lastSeenMs < PRESENT_MS) ? 1 : 0);
  }
  Agg a = computeAgg();
  Serial.printf("--- TOT %u | presenti %u | media incontri %.2f | record %u | cuori %u soli %u creatori %u\n",
    a.total, a.present, a.avgMet, a.maxMet, a.cuori, a.soli, a.creatori);
}

void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == 'd') dumpCSV();
  else if (c == 's') saveStats();
  else if (c == 'r') {
    badgeCount = 0; LittleFS.remove(STATS_FILE);
    Serial.println("[RESET] statistiche azzerate");
  }
}

// ═══════════════════════════════════════════════════════
//  WEB — JSON, CSV e pagina mobile
// ═══════════════════════════════════════════════════════
const char* moodName(uint8_t m) { return m == 0 ? "chill" : m == 1 ? "social" : "party"; }

void handleData() {
  Agg a = computeAgg();
  String j; j.reserve(16000);
  j  = "{\"agg\":{";
  j += "\"total\":" + String(a.total);
  j += ",\"present\":" + String(a.present);
  j += ",\"avgMet\":" + String(a.avgMet, 2);
  j += ",\"maxMet\":" + String(a.maxMet);
  char idbuf[8]; snprintf(idbuf, sizeof(idbuf), "%06X", a.maxId);
  j += ",\"maxId\":\"" + String(idbuf) + "\"";
  j += ",\"cuori\":" + String(a.cuori);
  j += ",\"soli\":" + String(a.soli);
  j += ",\"creatori\":" + String(a.creatori);
  j += ",\"mood\":[" + String(a.moodCnt[0]) + "," + String(a.moodCnt[1]) + "," + String(a.moodCnt[2]) + "]";
  j += "},\"badges\":[";
  uint32_t now = millis();
  if (xSemaphoreTake(tblMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    for (uint16_t i = 0; i < badgeCount; i++) {
      BadgeStat& b = badges[i];
      char id[8]; snprintf(id, sizeof(id), "%06X", b.id);
      if (i) j += ",";
      j += "{\"id\":\"" + String(id) + "\"";
      j += ",\"type\":\"" + String((b.type & 1) ? "sole" : "cuore") + "\"";
      j += ",\"cr\":" + String((b.type & 0x80) ? 1 : 0);
      j += ",\"mood\":\"" + String(moodName(b.lastMood)) + "\"";
      j += ",\"hue\":" + String(b.lastHue);
      j += ",\"met\":" + String(b.peopleMet);
      j += ",\"vis\":" + String(b.visits);
      j += ",\"rssi\":" + String(b.lastRssi);
      j += ",\"pre\":" + String((now - b.lastSeenMs < PRESENT_MS) ? 1 : 0);
      j += "}";
    }
    xSemaphoreGive(tblMutex);
  }
  j += "]}";
  server.send(200, "application/json", j);
}

void handleCSVweb() {
  String s = "id,tipo,creatore,mood,hue,peopleMet,visite,rssi,presente\n";
  uint32_t now = millis();
  if (xSemaphoreTake(tblMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    for (uint16_t i = 0; i < badgeCount; i++) {
      BadgeStat& b = badges[i];
      char line[96];
      snprintf(line, sizeof(line), "%06X,%s,%d,%s,%u,%u,%u,%d,%d\n",
        b.id, (b.type & 1) ? "sole" : "cuore", (b.type & 0x80) ? 1 : 0,
        moodName(b.lastMood), b.lastHue, b.peopleMet, b.visits, b.lastRssi,
        (now - b.lastSeenMs < PRESENT_MS) ? 1 : 0);
      s += line;
    }
    xSemaphoreGive(tblMutex);
  }
  server.sendHeader("Content-Disposition", "attachment; filename=pamali_stats.csv");
  server.send(200, "text/csv", s);
}

const char INDEX_HTML[] PROGMEM = R"HTML(<!DOCTYPE html><html lang="it"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Pamali Stats</title><style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#0d1530;color:#dce;font-family:system-ui,sans-serif;padding:12px;-webkit-tap-highlight-color:transparent}
h1{font-size:18px;color:#ffd24d;text-align:center;margin-bottom:10px;letter-spacing:1px}
.cards{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-bottom:12px}
.card{background:#16203f;border:1px solid #2a3a66;border-radius:10px;padding:10px;text-align:center}
.card .n{font-size:26px;font-weight:700;color:#fff}
.card .l{font-size:11px;color:#8fa;opacity:.8;text-transform:uppercase}
.sec{font-size:12px;color:#8aa;text-transform:uppercase;margin:14px 0 6px;letter-spacing:1px}
.bar{height:22px;background:#16203f;border-radius:5px;margin:4px 0;position:relative;overflow:hidden}
.bar>span{position:absolute;left:0;top:0;bottom:0;background:linear-gradient(90deg,#ffb33a,#ff5e9c);border-radius:5px}
.bar>b{position:absolute;left:8px;top:3px;font-size:12px;color:#fff;font-weight:600}
.bar>i{position:absolute;right:8px;top:3px;font-size:12px;color:#fff;font-style:normal}
table{width:100%;border-collapse:collapse;font-size:12px;margin-top:4px}
th,td{padding:5px 4px;border-bottom:1px solid #223;text-align:left}
th{color:#8aa;cursor:pointer}
.dot{display:inline-block;width:9px;height:9px;border-radius:50%}
.cr{color:#ffd24d}
a.dl{display:block;text-align:center;margin-top:14px;color:#8af;font-size:13px}
small{color:#567;display:block;text-align:center;margin-top:6px}
</style></head><body>
<h1>PAMALI &mdash; statistiche</h1>
<div class="cards">
 <div class="card"><div class="n" id="total">0</div><div class="l">badge totali</div></div>
 <div class="card"><div class="n" id="present">0</div><div class="l">presenti ora</div></div>
 <div class="card"><div class="n" id="avg">0</div><div class="l">media incontri</div></div>
 <div class="card"><div class="n" id="max">0</div><div class="l">record incontri</div></div>
</div>
<div class="sec">distribuzione incontri</div>
<div id="hist"></div>
<div class="sec">mood</div>
<div id="moods"></div>
<div class="sec">tutti i badge (<span id="cnt">0</span>)</div>
<table><thead><tr>
 <th data-k="id">ID</th><th data-k="type">tipo</th><th data-k="mood">mood</th>
 <th data-k="met">incontri</th><th data-k="vis">visite</th><th data-k="pre">qui</th>
</tr></thead><tbody id="rows"></tbody></table>
<a class="dl" href="/data.csv">scarica CSV</a>
<small id="ts"></small>
<script>
let sortK='met',sortDir=-1;
function hsv(h){return 'hsl('+(h/255*360)+',80%,55%)'}
function bars(el,items,max){el.innerHTML=items.map(it=>{
 let w=max?Math.round(it.v/max*100):0;
 return '<div class="bar"><span style="width:'+w+'%"></span><b>'+it.l+'</b><i>'+it.v+'</i></div>'}).join('')}
function render(d){
 document.getElementById('total').textContent=d.agg.total;
 document.getElementById('present').textContent=d.agg.present;
 document.getElementById('avg').textContent=d.agg.avgMet;
 document.getElementById('max').textContent=d.agg.maxMet;
 document.getElementById('cnt').textContent=d.badges.length;
 // istogramma incontri
 let bk=[0,0,0,0,0];
 d.badges.forEach(b=>{let m=b.met; bk[m==0?0:m<=2?1:m<=5?2:m<=10?3:4]++});
 let mx=Math.max(1,...bk);
 bars(document.getElementById('hist'),
   [['0',bk[0]],['1-2',bk[1]],['3-5',bk[2]],['6-10',bk[3]],['11+',bk[4]]].map(x=>({l:x[0],v:x[1]})),mx);
 // mood
 let mm=Math.max(1,...d.agg.mood);
 bars(document.getElementById('moods'),
   [['chill',d.agg.mood[0]],['social',d.agg.mood[1]],['party',d.agg.mood[2]]].map(x=>({l:x[0],v:x[1]})),mm);
 // tabella
 let bs=d.badges.slice().sort((a,b)=>{let x=a[sortK],y=b[sortK];return (x<y?-1:x>y?1:0)*sortDir});
 document.getElementById('rows').innerHTML=bs.map(b=>
  '<tr><td>'+(b.cr?'<span class="cr">&#9733;</span> ':'')+b.id+'</td>'+
  '<td><span class="dot" style="background:'+hsv(b.hue)+'"></span> '+b.type+'</td>'+
  '<td>'+b.mood+'</td><td>'+b.met+'</td><td>'+b.vis+'</td>'+
  '<td>'+(b.pre?'&#9679;':'')+'</td></tr>').join('');
 document.getElementById('ts').textContent='aggiornato '+new Date().toLocaleTimeString();
}
document.querySelectorAll('th').forEach(th=>th.onclick=()=>{
 let k=th.dataset.k; if(k==sortK)sortDir*=-1; else{sortK=k;sortDir=-1} load()});
function load(){fetch('/data').then(r=>r.json()).then(render).catch(()=>{})}
load(); setInterval(load,3000);
</script></body></html>)HTML";

void handleRoot() { server.send_P(200, "text/html", INDEX_HTML); }

// ═══════════════════════════════════════════════════════
//  SETUP / LOOP
// ═══════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== PAMALI base station ===");
  Serial.println("comandi: d=dump  s=salva  r=reset");

  tblMutex = xSemaphoreCreateMutex();

  // accendi periferiche (Vext) + OLED
  pinMode(VEXT_PIN, OUTPUT); digitalWrite(VEXT_PIN, LOW); delay(50);
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW); delay(20); digitalWrite(OLED_RST, HIGH); delay(20);
  Wire.begin(OLED_SDA, OLED_SCL);
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 30, "PAMALI base...");
  u8g2.sendBuffer();

  // filesystem persistente
  if (!LittleFS.begin(true)) Serial.println("[FS] errore LittleFS");
  else loadStats();

  // BLE scan continuo (alimentato, niente risparmio)
  NimBLEDevice::init("PAMALI-BASE");
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(new CollectorCB(), true);  // wantDuplicates=true
  scan->setActiveScan(false);
  scan->setInterval(100);
  scan->setWindow(100);
  scan->start(0, false);

  // WiFi Access Point + web server (il telefono si collega a PAMALI-STATS)
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  IPAddress ip = WiFi.softAPIP();
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/data.csv", handleCSVweb);
  server.begin();
  Serial.printf("WiFi AP '%s' (pw '%s')  →  http://%s\n", AP_SSID, AP_PASS, ip.toString().c_str());

  Serial.println("scan avviato.\n");
}

uint32_t lastSave = 0, lastSummary = 0, lastDraw = 0;

void loop() {
  handleSerial();
  server.handleClient();          // serve le richieste del telefono
  uint32_t now = millis();

  if (now - lastDraw >= 500) { drawDisplay(); lastDraw = now; }

  // salva su flash ogni 30s se ci sono novita'
  if (dirty && now - lastSave >= 30000) { saveStats(); lastSave = now; }

  // riepilogo seriale ogni 15s
  if (now - lastSummary >= 15000) {
    Agg a = computeAgg();
    Serial.printf("[%lus] badge=%u presenti=%u mediaIncontri=%.2f record=%u (cuori=%u soli=%u creatori=%u)\n",
      now / 1000, a.total, a.present, a.avgMet, a.maxMet, a.cuori, a.soli, a.creatori);
    lastSummary = now;
  }
  delay(20);
}
