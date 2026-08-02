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
#include <RadioLib.h>      // sync grafo via LoRa tra piu' Heltec (installa "RadioLib")

// ── LoRa SX1262 (Heltec WiFi LoRa 32 V3) ────────────────
#define LORA_NSS   8
#define LORA_SCK   9
#define LORA_MOSI  10
#define LORA_MISO  11
#define LORA_RST   12
#define LORA_BUSY  13
#define LORA_DIO1  14
#define LORA_FREQ  868.0    // MHz — EU868 (Italia). USA: 915.0
SPIClass loraSPI(HSPI);
SX1262   radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY, loraSPI);
bool     loraOK = false;
uint16_t stationId = 0;                 // id univoco di questo Heltec (dal MAC)
static const uint32_t GOSSIP_MS = 10000; // intervallo broadcast archi (occhio al duty-cycle!)

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
  uint8_t  cPage;       // pagina contatti (grafo)
  uint8_t  cId[2][3];   // ID di 2 contatti
  uint8_t  cW[2];       // peso in unità da 4s
};

// ── Tabella statistiche per badge ───────────────────────
#define MAX_BADGES   200
#define VISIT_GAP_MS 60000UL     // assenza oltre 60s = nuova "visita"
#define PRESENT_MS   8000UL      // visto negli ultimi 8s = presente ora (zona locale)
#define SYNC_PRESENT_MS 150000UL // multi-Heltec: finestra "presente" piu' ampia (ritardo LoRa)

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

// statistiche aggregate (definita in alto: evita errori di auto-prototipo Arduino)
struct Agg {
  uint16_t total, present;
  float    avgMet;
  uint16_t maxMet; uint32_t maxId;
  uint16_t cuori, soli, creatori;
  uint16_t moodCnt[4];   // chill, social, party, rasta
};

// PUBBLICO: dispositivi BLE (telefoni) nei dintorni (anche queste struct in alto)
struct PhoneSeen { uint32_t h; uint32_t last; int8_t rssi; bool apple; };
struct Crowd     { uint16_t now, peak, apple; uint32_t total; int avgRssi; };

// ANDAMENTO SERATA: composizione di mood e colori dei badge presenti, campionata
#define HUE_BINS 8
struct MoodSample { uint8_t mood[4]; uint8_t hue[HUE_BINS]; };

// ── GRAFO: archi (chi ha incontrato chi, con peso = secondi insieme) ─────
#define MAX_EDGES 2000
struct __attribute__((packed)) Edge {
  uint32_t a;      // id minore (24-bit)
  uint32_t b;      // id maggiore
  uint16_t w;      // peso = secondi di contatto (massimo osservato)
};
Edge      edges[MAX_EDGES];
uint16_t  edgeCount = 0;
uint16_t  gossipCursor = 0;           // round-robin per il broadcast LoRa
volatile bool edgesDirty = false;
SemaphoreHandle_t edgeMutex = nullptr;
const char* EDGES_FILE = "/edges.bin";

int findEdge(uint32_t a, uint32_t b) {
  for (uint16_t i = 0; i < edgeCount; i++)
    if (edges[i].a == a && edges[i].b == b) return i;
  return -1;
}

// inserisce/aggiorna un arco (non-orientato) tenendo il peso massimo.
// ritorna true se qualcosa è cambiato (nuovo arco o peso aumentato).
bool upsertEdge(uint32_t x, uint32_t y, uint16_t w) {
  if (x == y || x == 0 || y == 0) return false;
  uint32_t a = x < y ? x : y, b = x < y ? y : x;
  bool changed = false;
  if (xSemaphoreTake(edgeMutex, pdMS_TO_TICKS(10)) != pdTRUE) return false;
  int idx = findEdge(a, b);
  if (idx >= 0) {
    if (w > edges[idx].w) { edges[idx].w = w; changed = true; }
  } else if (edgeCount < MAX_EDGES) {
    edges[edgeCount++] = { a, b, w };
    changed = true;
  }
  if (changed) edgesDirty = true;
  xSemaphoreGive(edgeMutex);
  return changed;
}

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

void saveEdges() {
  if (xSemaphoreTake(edgeMutex, pdMS_TO_TICKS(50)) != pdTRUE) return;
  File f = LittleFS.open(EDGES_FILE, "w");
  if (f) {
    f.write((uint8_t*)&edgeCount, sizeof(edgeCount));
    f.write((uint8_t*)edges, sizeof(Edge) * edgeCount);
    f.close();
    Serial.printf("[FS] salvati %u archi\n", edgeCount);
  }
  edgesDirty = false;
  xSemaphoreGive(edgeMutex);
}

void loadEdges() {
  if (!LittleFS.exists(EDGES_FILE)) return;
  File f = LittleFS.open(EDGES_FILE, "r");
  if (!f) return;
  f.read((uint8_t*)&edgeCount, sizeof(edgeCount));
  if (edgeCount > MAX_EDGES) edgeCount = MAX_EDGES;
  f.read((uint8_t*)edges, sizeof(Edge) * edgeCount);
  f.close();
  Serial.printf("[FS] caricati %u archi del grafo\n", edgeCount);
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

// Fonde un badge ricevuto via LoRa da un altro Heltec (dedup per id).
// age = da quanti ms l'altro Heltec l'ha visto. Tiene la vista PIU' recente.
void mergeBadge(uint32_t id, uint8_t type, uint8_t mood, uint8_t hue,
                uint16_t met, uint16_t vis, uint16_t age) {
  if (xSemaphoreTake(tblMutex, pdMS_TO_TICKS(10)) != pdTRUE) return;
  uint32_t now = millis();
  int idx = findBadge(id);
  if (idx < 0) {
    if (badgeCount >= MAX_BADGES) { xSemaphoreGive(tblMutex); return; }
    idx = badgeCount++;
    badges[idx] = {};
    badges[idx].id = id; badges[idx].firstSeenMs = now; badges[idx].visits = vis ? vis : 1;
  }
  uint32_t seen = now - age;                       // quando l'ha visto l'altro (mio clock)
  if (seen > badges[idx].lastSeenMs) {             // piu' recente di cio' che so
    badges[idx].lastSeenMs = seen;
    badges[idx].type = type; badges[idx].lastMood = mood; badges[idx].lastHue = hue;
  }
  if (met > badges[idx].peopleMet) badges[idx].peopleMet = met;
  if (vis > badges[idx].visits)    badges[idx].visits    = vis;
  dirty = true;
  xSemaphoreGive(tblMutex);
}

// ═══════════════════════════════════════════════════════
//  PUBBLICO — dispositivi BLE (telefoni) nei dintorni
// ═══════════════════════════════════════════════════════
#define MAX_PHONES   500
#define PHONE_TTL_MS 45000UL          // visto negli ultimi 45s = "presente ora"
PhoneSeen phones[MAX_PHONES];
uint16_t  phoneCount = 0;
uint16_t  phonePeak  = 0;
uint32_t  phoneTotal = 0;             // indirizzi distinti cumulativi (gonfiato dal MAC casuale)
SemaphoreHandle_t phoneMutex = nullptr;

uint32_t addrHash(const std::string& s) {        // FNV-1a 32 bit dell'indirizzo BLE
  uint32_t h = 2166136261u;
  for (size_t i = 0; i < s.size(); i++) { h ^= (uint8_t)s[i]; h *= 16777619u; }
  return h;
}
void notePhone(uint32_t h, int8_t rssi, bool apple) {
  if (xSemaphoreTake(phoneMutex, pdMS_TO_TICKS(5)) != pdTRUE) return;
  uint32_t now = millis();
  for (uint16_t i = 0; i < phoneCount; i++)
    if (phones[i].h == h) { phones[i].last = now; phones[i].rssi = rssi; phones[i].apple = apple; xSemaphoreGive(phoneMutex); return; }
  phoneTotal++;
  if (phoneCount < MAX_PHONES) phones[phoneCount++] = { h, now, rssi, apple };
  else {                                          // pieno: sostituisci il piu' vecchio
    uint16_t o = 0;
    for (uint16_t i = 1; i < phoneCount; i++) if (phones[i].last < phones[o].last) o = i;
    phones[o] = { h, now, rssi, apple };
  }
  xSemaphoreGive(phoneMutex);
}
void compactPhones() {                            // scarta i dispositivi non piu' presenti
  if (xSemaphoreTake(phoneMutex, pdMS_TO_TICKS(10)) != pdTRUE) return;
  uint32_t now = millis(); uint16_t w = 0;
  for (uint16_t i = 0; i < phoneCount; i++)
    if (now - phones[i].last < PHONE_TTL_MS) phones[w++] = phones[i];
  phoneCount = w;
  xSemaphoreGive(phoneMutex);
}
Crowd crowdStats() {
  Crowd c = {};
  if (xSemaphoreTake(phoneMutex, pdMS_TO_TICKS(10)) != pdTRUE) return c;
  uint32_t t = millis(), sum = 0;
  for (uint16_t i = 0; i < phoneCount; i++)
    if (t - phones[i].last < PHONE_TTL_MS) { c.now++; if (phones[i].apple) c.apple++; sum += (uint32_t)(-phones[i].rssi); }
  c.avgRssi = c.now ? -(int)(sum / c.now) : 0;
  if (c.now > phonePeak) phonePeak = c.now;
  c.peak = phonePeak; c.total = phoneTotal;
  xSemaphoreGive(phoneMutex);
  return c;
}

// storico affluenza della zona di QUESTO Heltec
#define CROWD_HIST       96        // 96 campioni × 30s ≈ 48 min
#define CROWD_SAMPLE_MS  30000UL
uint8_t crowdHist[CROWD_HIST];
uint8_t crowdHistN = 0;
void pushCrowdHist(uint16_t v) {
  uint8_t b = v > 255 ? 255 : (uint8_t)v;
  if (crowdHistN < CROWD_HIST) crowdHist[crowdHistN++] = b;
  else { memmove(crowdHist, crowdHist + 1, CROWD_HIST - 1); crowdHist[CROWD_HIST - 1] = b; }
}

// ═══════════════════════════════════════════════════════
//  BLE SCAN
// ═══════════════════════════════════════════════════════
class CollectorCB : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* dev) override {
    int8_t rssi = (int8_t)dev->getRSSI();
    bool isBadge = false, apple = false;
    if (dev->haveManufacturerData()) {
      std::string mfg = dev->getManufacturerData();
      if (mfg.size() >= 2) {
        uint16_t cid = (uint8_t)mfg[0] | ((uint8_t)mfg[1] << 8);
        if (cid == 0x004C) apple = true;          // Apple (iPhone/Watch/AirPods…)
      }
      if (mfg.size() >= sizeof(BeaconPayload)) {
        const BeaconPayload* p = (const BeaconPayload*)mfg.data();
        if (p->companyId == COMPANY_ID && p->proto == PROTO_VER) {
          isBadge = true;
          uint32_t id = ((uint32_t)p->id[0] << 16) | ((uint32_t)p->id[1] << 8) | p->id[2];
          updateBadge(id, p, rssi);
          for (uint8_t k = 0; k < 2; k++) {       // archi del grafo
            uint32_t cid = ((uint32_t)p->cId[k][0] << 16) | ((uint32_t)p->cId[k][1] << 8) | p->cId[k][2];
            if (cid == 0) continue;
            upsertEdge(id, cid, (uint16_t)p->cW[k] * 4);
          }
        }
      }
    }
    if (!isBadge)                                 // pubblico: telefono/gadget BLE
      notePhone(addrHash(dev->getAddress().toString()), rssi, apple);
  }
};

// ═══════════════════════════════════════════════════════
//  LoRa — sync del grafo tra piu' Heltec (gossip + merge)
// ═══════════════════════════════════════════════════════
#define EDGES_PER_PACKET  24
#define BADGES_PER_PACKET 18
uint8_t  gossipPhase = 0;       // alterna invio archi / badge
uint16_t badgeCursor = 0;       // round-robin sui badge
volatile bool loraRxFlag = false;
void IRAM_ATTR onLoRaIRQ() { loraRxFlag = true; }

// altri Heltec sentiti di recente sulla rete LoRa (per "Heltec attivi")
#define MAX_STATIONS   16
#define STATION_TTL_MS 60000UL
struct Station { uint16_t id; uint32_t lastSeen; uint16_t cNow; uint16_t cPeak; };
Station  stations[MAX_STATIONS];
uint8_t  stationCount = 0;
float    lastLoraRssi = 0;     // RSSI dell'ultimo pacchetto LoRa valido ricevuto
uint32_t lastLoraMs   = 0;
void noteStation(uint16_t id, uint16_t cNow, uint16_t cPeak) {
  uint32_t now = millis();
  for (uint8_t i = 0; i < stationCount; i++)
    if (stations[i].id == id) { stations[i].lastSeen = now; stations[i].cNow = cNow; stations[i].cPeak = cPeak; return; }
  if (stationCount < MAX_STATIONS) { stations[stationCount] = { id, now, cNow, cPeak }; stationCount++; }
}
uint8_t activeStations() {     // quanti ALTRI Heltec sentiti negli ultimi 60s
  uint32_t now = millis(); uint8_t n = 0;
  for (uint8_t i = 0; i < stationCount; i++)
    if (now - stations[i].lastSeen < STATION_TTL_MS) n++;
  return n;
}
uint16_t stationsCrowdSum() {  // somma dei telefoni nelle ALTRE zone (Heltec attivi)
  uint32_t now = millis(); uint16_t s = 0;
  for (uint8_t i = 0; i < stationCount; i++)
    if (now - stations[i].lastSeen < STATION_TTL_MS) s += stations[i].cNow;
  return s;
}

void initLoRa() {
  // ID dai byte ALTI del MAC: i 3 bassi sono l'OUI Espressif, UGUALE su tutti
  // i chip (stesso bug corretto sui badge) → gli Heltec si sarebbero ignorati.
  stationId = (uint16_t)((ESP.getEfuseMac() >> 32) & 0xFFFF);
  loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  int st = radio.begin(LORA_FREQ);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.printf("[LoRa] init err %d → sync grafo DISATTIVO (singolo Heltec ok)\n", st);
    loraOK = false;
    return;
  }
  radio.setOutputPower(14);     // dBm
  radio.setBandwidth(125.0);
  radio.setSpreadingFactor(7);  // SF7: airtime basso (~duty 1% a 10s). Alza per piu' portata
  radio.setCodingRate(7);
  radio.setSyncWord(0x34);
  radio.setDio1Action(onLoRaIRQ);
  radio.startReceive();
  loraOK = true;
  Serial.printf("[LoRa] pronto a %.1fMHz, stazione #%u\n", LORA_FREQ, stationId);
}

// pacchetto ricevuto → merge nella tabella locale (archi o badge, secondo il tipo)
// formato: 'P','A', type(1), stationId(2), cNow(2), cPeak(2), count(1), payload...
void processLoRaRx() {
  uint8_t buf[256];
  size_t len = radio.getPacketLength();
  int st = radio.readData(buf, len);
  float rssi = radio.getRSSI();                   // RSSI di questo pacchetto
  radio.startReceive();
  if (st != RADIOLIB_ERR_NONE || len < 10) return;  // header ora 10 byte
  if (buf[0] != 'P' || buf[1] != 'A') return;
  uint8_t  ptype = buf[2];                         // 0=archi 1=badge
  uint16_t from  = buf[3] | (buf[4] << 8);
  if (from == stationId) return;                  // eco mio, ignora
  uint16_t cNow  = buf[5] | (buf[6] << 8);
  uint16_t cPeak = buf[7] | (buf[8] << 8);
  lastLoraRssi = rssi; lastLoraMs = millis();
  noteStation(from, cNow, cPeak);                 // un altro Heltec è vivo + il suo pubblico
  uint8_t n = buf[9];
  size_t off = 10;
  uint16_t merged = 0;
  if (ptype == 0) {                               // ── ARCHI ──
    for (uint8_t k = 0; k < n && off + 8 <= len; k++) {
      uint32_t a = ((uint32_t)buf[off] << 16) | ((uint32_t)buf[off+1] << 8) | buf[off+2]; off += 3;
      uint32_t b = ((uint32_t)buf[off] << 16) | ((uint32_t)buf[off+1] << 8) | buf[off+2]; off += 3;
      uint16_t w = buf[off] | (buf[off+1] << 8); off += 2;
      if (upsertEdge(a, b, w)) merged++;
    }
    if (merged) Serial.printf("[LoRa] da #%u: %u archi\n", from, merged);
  } else {                                        // ── BADGE ──
    for (uint8_t k = 0; k < n && off + 12 <= len; k++) {
      uint32_t id  = ((uint32_t)buf[off] << 16) | ((uint32_t)buf[off+1] << 8) | buf[off+2]; off += 3;
      uint8_t  ty  = buf[off++], mo = buf[off++], hu = buf[off++];
      uint16_t met = buf[off] | (buf[off+1] << 8); off += 2;
      uint16_t vis = buf[off] | (buf[off+1] << 8); off += 2;
      uint16_t age = buf[off] | (buf[off+1] << 8); off += 2;
      mergeBadge(id, ty, mo, hu, met, vis, age);
      merged++;
    }
    if (merged) Serial.printf("[LoRa] da #%u: %u badge sync\n", from, merged);
  }
}

// broadcast: alterna un lotto di ARCHI e uno di BADGE (round-robin), + pubblico
void sendGossip() {
  if (!loraOK) return;
  Crowd c = crowdStats();
  uint8_t buf[256];
  buf[0] = 'P'; buf[1] = 'A';
  buf[3] = stationId & 0xFF; buf[4] = stationId >> 8;
  buf[5] = c.now  & 0xFF; buf[6] = c.now  >> 8;   // pubblico della MIA zona
  buf[7] = c.peak & 0xFF; buf[8] = c.peak >> 8;
  // scegli cosa inviare (salta i vuoti, altrimenti alterna)
  bool sendBadges;
  if      (edgeCount == 0)  sendBadges = true;
  else if (badgeCount == 0) sendBadges = false;
  else { gossipPhase ^= 1; sendBadges = gossipPhase; }
  uint8_t  n = 0;
  size_t   off = 10;
  if (!sendBadges) {                              // ── ARCHI ──
    buf[2] = 0;
    if (edgeCount > 0 && xSemaphoreTake(edgeMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      for (uint8_t k = 0; k < EDGES_PER_PACKET && k < edgeCount; k++) {
        uint16_t i = (gossipCursor + k) % edgeCount;
        buf[off++] = edges[i].a >> 16; buf[off++] = edges[i].a >> 8; buf[off++] = edges[i].a;
        buf[off++] = edges[i].b >> 16; buf[off++] = edges[i].b >> 8; buf[off++] = edges[i].b;
        buf[off++] = edges[i].w & 0xFF; buf[off++] = edges[i].w >> 8;
        n++;
      }
      gossipCursor = (gossipCursor + n) % edgeCount;
      xSemaphoreGive(edgeMutex);
    }
  } else {                                        // ── BADGE ──
    buf[2] = 1;
    if (badgeCount > 0 && xSemaphoreTake(tblMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      uint32_t now = millis();
      for (uint8_t k = 0; k < BADGES_PER_PACKET && k < badgeCount; k++) {
        uint16_t i = (badgeCursor + k) % badgeCount;
        BadgeStat& b = badges[i];
        uint32_t age = now - b.lastSeenMs; if (age > 60000) age = 60000;
        buf[off++] = b.id >> 16; buf[off++] = b.id >> 8; buf[off++] = b.id;
        buf[off++] = b.type; buf[off++] = b.lastMood; buf[off++] = b.lastHue;
        buf[off++] = b.peopleMet & 0xFF; buf[off++] = b.peopleMet >> 8;
        buf[off++] = b.visits & 0xFF; buf[off++] = b.visits >> 8;
        buf[off++] = age & 0xFF; buf[off++] = (age >> 8) & 0xFF;
        n++;
      }
      badgeCursor = (badgeCursor + n) % badgeCount;
      xSemaphoreGive(tblMutex);
    }
  }
  buf[9] = n;
  int st = radio.transmit(buf, off);
  loraRxFlag = false;            // scarta eventuale flag spurio del TxDone
  radio.startReceive();
  if (st != RADIOLIB_ERR_NONE) Serial.printf("[LoRa] TX err %d\n", st);
}

// ═══════════════════════════════════════════════════════
//  STATISTICHE AGGREGATE
// ═══════════════════════════════════════════════════════
// finestra "presente": stretta da soli (8s), piu' larga in rete LoRa per
// assorbire il ritardo di sync dei badge delle altre zone.
uint32_t presentWin() { return activeStations() > 0 ? SYNC_PRESENT_MS : PRESENT_MS; }

Agg computeAgg() {
  Agg a = {};
  if (xSemaphoreTake(tblMutex, pdMS_TO_TICKS(20)) != pdTRUE) return a;
  uint32_t now = millis();
  uint32_t pw = presentWin();
  uint32_t sumMet = 0;
  for (uint16_t i = 0; i < badgeCount; i++) {
    a.total++;
    if (now - badges[i].lastSeenMs < pw) a.present++;
    sumMet += badges[i].peopleMet;
    if (badges[i].peopleMet > a.maxMet) { a.maxMet = badges[i].peopleMet; a.maxId = badges[i].id; }
    if (badges[i].type & 0x01) a.soli++; else a.cuori++;
    if (badges[i].type & 0x80) a.creatori++;
    uint8_t m = badges[i].lastMood; if (m < 4) a.moodCnt[m]++;
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

// ── ANDAMENTO SERATA: storico composizione mood + colori (badge presenti) ──
#define MOODHIST     120
#define MOODHIST_MS  120000UL          // 1 campione ogni 2 min → 120 = ~4 ore
MoodSample moodHist[MOODHIST];
uint8_t    moodHistN = 0;
void pushMoodHist(const MoodSample& s) {
  if (moodHistN < MOODHIST) moodHist[moodHistN++] = s;
  else { memmove(moodHist, moodHist + 1, sizeof(MoodSample) * (MOODHIST - 1)); moodHist[MOODHIST - 1] = s; }
}
void sampleMoodHist() {
  MoodSample s = {};
  uint32_t now = millis();
  uint32_t pw = presentWin();
  if (xSemaphoreTake(tblMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    for (uint16_t i = 0; i < badgeCount; i++) {
      if (now - badges[i].lastSeenMs >= pw) continue;          // solo i presenti
      uint8_t m = badges[i].lastMood; if (m < 4 && s.mood[m] < 255) s.mood[m]++;
      uint8_t b = (uint16_t)badges[i].lastHue * HUE_BINS / 256; if (b >= HUE_BINS) b = HUE_BINS - 1;
      if (s.hue[b] < 255) s.hue[b]++;
    }
    xSemaphoreGive(tblMutex);
  }
  pushMoodHist(s);
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
  uint8_t page = (millis() / 4000) % 8;   // cambia pagina ogni 4s (8 pagine)

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
      snprintf(buf, sizeof(buf), "chill %u  social %u", a.moodCnt[0], a.moodCnt[1]); u8g2.drawStr(4, 44, buf);
      snprintf(buf, sizeof(buf), "party %u  rasta %u",  a.moodCnt[2], a.moodCnt[3]); u8g2.drawStr(4, 58, buf);
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
    case 6:
      u8g2.drawStr(0, 27, "RETE / GRAFO");
      snprintf(buf, sizeof(buf), "archi: %u", edgeCount);
      u8g2.drawStr(0, 39, buf);
      if (loraOK) {
        snprintf(buf, sizeof(buf), "Heltec in rete: %u", activeStations() + 1);
        u8g2.drawStr(0, 51, buf);
        if (lastLoraMs && millis() - lastLoraMs < STATION_TTL_MS)
          snprintf(buf, sizeof(buf), "LoRa rx: %d dBm", (int)lastLoraRssi);
        else
          snprintf(buf, sizeof(buf), "LoRa rx: --");
        u8g2.drawStr(0, 63, buf);
      } else {
        u8g2.drawStr(0, 51, "LoRa: off (solo)");
      }
      break;
    case 7: {
      Crowd c = crowdStats();
      u8g2.setFont(u8g2_font_logisoso16_tf);
      snprintf(buf, sizeof(buf), "%u", c.now);
      u8g2.drawStr(0, 40, buf);
      u8g2.setFont(u8g2_font_6x12_tf);
      u8g2.drawStr(0, 52, "telefoni vicini");
      if (activeStations() > 0)
        snprintf(buf, sizeof(buf), "rete %u in %u zone", c.now + stationsCrowdSum(), activeStations() + 1);
      else
        snprintf(buf, sizeof(buf), "picco %u  Apple %u", c.peak, c.apple);
      u8g2.drawStr(0, 63, buf);
      break;
    }
  }
  u8g2.sendBuffer();
}

// ═══════════════════════════════════════════════════════
//  SERIALE
// ═══════════════════════════════════════════════════════
void dumpCSV() {
  Serial.println("\nid,tipo,creatore,mood,hue,peopleMet,visite,ultimoRSSI,present");
  uint32_t now = millis(); uint32_t pw = presentWin();
  for (uint16_t i = 0; i < badgeCount; i++) {
    BadgeStat& b = badges[i];
    Serial.printf("%06X,%s,%d,%u,%u,%u,%u,%d,%d\n",
      b.id, (b.type & 1) ? "sole" : "cuore", (b.type & 0x80) ? 1 : 0,
      b.lastMood, b.lastHue, b.peopleMet, b.visits, b.lastRssi,
      (now - b.lastSeenMs < pw) ? 1 : 0);
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
    edgeCount = 0;  gossipCursor = 0; LittleFS.remove(EDGES_FILE);
    Serial.println("[RESET] statistiche e grafo azzerati");
  }
  else if (c == 'g') {
    if (loraOK) { sendGossip(); Serial.println("[LoRa] gossip forzato"); }
    else Serial.println("[LoRa] non disponibile");
  }
}

// ═══════════════════════════════════════════════════════
//  WEB — JSON, CSV e pagina mobile
// ═══════════════════════════════════════════════════════
const char* moodName(uint8_t m) { return m == 0 ? "chill" : m == 1 ? "social" : m == 2 ? "party" : "rasta"; }

void handleData() {
  Agg a = computeAgg();
  Crowd cr = crowdStats();
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
  j += ",\"mood\":[" + String(a.moodCnt[0]) + "," + String(a.moodCnt[1]) + ","
     + String(a.moodCnt[2]) + "," + String(a.moodCnt[3]) + "]";
  j += "},\"crowd\":{\"now\":" + String(cr.now) + ",\"peak\":" + String(cr.peak)
     + ",\"apple\":" + String(cr.apple) + ",\"total\":" + String(cr.total)
     + ",\"rssi\":" + String(cr.avgRssi)
     + ",\"net\":" + String(cr.now + stationsCrowdSum())
     + ",\"zones\":" + String(activeStations() + 1)
     + ",\"hist\":[";
  for (uint8_t i = 0; i < crowdHistN; i++) { if (i) j += ","; j += String(crowdHist[i]); }
  j += "]}";
  j += ",\"badges\":[";
  uint32_t now = millis(); uint32_t pw = presentWin();
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
      j += ",\"pre\":" + String((now - b.lastSeenMs < pw) ? 1 : 0);
      j += "}";
    }
    xSemaphoreGive(tblMutex);
  }
  j += "]}";
  server.send(200, "application/json", j);
}

void handleCSVweb() {
  String s = "id,tipo,creatore,mood,hue,peopleMet,visite,rssi,presente\n";
  uint32_t now = millis(); uint32_t pw = presentWin();
  if (xSemaphoreTake(tblMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    for (uint16_t i = 0; i < badgeCount; i++) {
      BadgeStat& b = badges[i];
      char line[96];
      snprintf(line, sizeof(line), "%06X,%s,%d,%s,%u,%u,%u,%d,%d\n",
        b.id, (b.type & 1) ? "sole" : "cuore", (b.type & 0x80) ? 1 : 0,
        moodName(b.lastMood), b.lastHue, b.peopleMet, b.visits, b.lastRssi,
        (now - b.lastSeenMs < pw) ? 1 : 0);
      s += line;
    }
    xSemaphoreGive(tblMutex);
  }
  server.sendHeader("Content-Disposition", "attachment; filename=pamali_stats.csv");
  server.send(200, "text/csv", s);
}

// ── ANDAMENTO SERATA: serie storica mood + colori (per i grafici stacked) ──
void handleTimeline() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");
  String j; j.reserve(2048);
  j = "{\"bins\":" + String(HUE_BINS) + ",\"moods\":[";
  for (uint8_t i = 0; i < moodHistN; i++) {
    if (i) j += ",";
    j += "[" + String(moodHist[i].mood[0]) + "," + String(moodHist[i].mood[1]) + ","
       + String(moodHist[i].mood[2]) + "," + String(moodHist[i].mood[3]) + "]";
    if (j.length() > 1500) { server.sendContent(j); j = ""; }
  }
  j += "],\"colors\":[";
  for (uint8_t i = 0; i < moodHistN; i++) {
    if (i) j += ",";
    j += "[";
    for (uint8_t b = 0; b < HUE_BINS; b++) { if (b) j += ","; j += String(moodHist[i].hue[b]); }
    j += "]";
    if (j.length() > 1500) { server.sendContent(j); j = ""; }
  }
  j += "]}";
  server.sendContent(j);
  server.sendContent("");
}

// ── GRAFO: nodi (badge) + archi (incontri pesati), in streaming chunked ──
void handleGraphData() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");
  uint32_t now = millis(); uint32_t pw = presentWin();
  String j; j.reserve(2048);
  j = "{\"nodes\":[";
  if (xSemaphoreTake(tblMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    for (uint16_t i = 0; i < badgeCount; i++) {
      BadgeStat& b = badges[i];
      char id[8]; snprintf(id, sizeof(id), "%06X", b.id);
      if (i) j += ",";
      uint8_t t = (b.type & 0x80) ? 2 : (b.type & 1);   // 2=creatore 1=sole 0=cuore
      j += "{\"id\":\""; j += id; j += "\",\"h\":"; j += b.lastHue;
      j += ",\"t\":"; j += t; j += ",\"m\":"; j += b.peopleMet;
      j += ",\"p\":"; j += (now - b.lastSeenMs < pw) ? 1 : 0; j += "}";
      if (j.length() > 1500) { server.sendContent(j); j = ""; }
    }
    xSemaphoreGive(tblMutex);
  }
  j += "],\"edges\":[";
  if (xSemaphoreTake(edgeMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    for (uint16_t i = 0; i < edgeCount; i++) {
      char a[8], b2[8];
      snprintf(a, sizeof(a), "%06X", edges[i].a);
      snprintf(b2, sizeof(b2), "%06X", edges[i].b);
      if (i) j += ",";
      j += "[\""; j += a; j += "\",\""; j += b2; j += "\","; j += edges[i].w; j += "]";
      if (j.length() > 1500) { server.sendContent(j); j = ""; }
    }
    xSemaphoreGive(edgeMutex);
  }
  j += "]}";
  server.sendContent(j);
  server.sendContent("");
}

void handleEdgesCSV() {
  server.sendHeader("Content-Disposition", "attachment; filename=pamali_grafo.csv");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv", "");
  server.sendContent("a,b,secondi_insieme\n");
  if (xSemaphoreTake(edgeMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    String s; s.reserve(1700);
    for (uint16_t i = 0; i < edgeCount; i++) {
      char line[40];
      snprintf(line, sizeof(line), "%06X,%06X,%u\n", edges[i].a, edges[i].b, edges[i].w);
      s += line;
      if (s.length() > 1500) { server.sendContent(s); s = ""; }
    }
    server.sendContent(s);
    xSemaphoreGive(edgeMutex);
  }
  server.sendContent("");
}

const char GRAPH_HTML[] PROGMEM = R"GRF(<!DOCTYPE html><html lang="it"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Pamali — rete</title><style>
*{margin:0;padding:0;box-sizing:border-box}
html,body{height:100%;background:#0d1530;overflow:hidden;font-family:system-ui,sans-serif}
#c{display:block;width:100vw;height:100vh;touch-action:none}
#h{position:fixed;top:8px;left:0;right:0;text-align:center;color:#ffd24d;font-size:15px;pointer-events:none}
#i{position:fixed;bottom:8px;left:0;right:0;text-align:center;color:#7890b8;font-size:11px;pointer-events:none}
a{position:fixed;top:7px;right:10px;color:#8af;font-size:12px;text-decoration:none}
</style></head><body>
<canvas id="c"></canvas>
<div id="h">PAMALI — rete degli incontri</div>
<a href="/">‹ stats</a>
<div id="i">nodi = persone · linee = tempo passato insieme · oro = creatore · trascina/zoom</div>
<script>
const cv=document.getElementById('c'),cx=cv.getContext('2d');
let W,H,DPR;function rs(){DPR=devicePixelRatio||1;W=cv.clientWidth;H=cv.clientHeight;cv.width=W*DPR;cv.height=H*DPR;cx.setTransform(DPR,0,0,DPR,0,0)}
addEventListener('resize',rs);rs();
let N=new Map(),E=[];let view={x:W/2,y:H/2,s:1};let auto=true;
function col(h){return 'hsl('+(h/255*360)+',75%,55%)'}
function fit(){let ns=[...N.values()];if(!ns.length)return;let xn=1e9,xx=-1e9,yn=1e9,yx=-1e9;ns.forEach(n=>{xn=Math.min(xn,n.x);xx=Math.max(xx,n.x);yn=Math.min(yn,n.y);yx=Math.max(yx,n.y)});let w=(xx-xn)||1,h=(yx-yn)||1,s=Math.min(W/(w+90),H/(h+90),2.2);view.s=view.s*.9+s*.1;view.x=view.x*.9+(W/2-(xn+xx)/2*view.s)*.1;view.y=view.y*.9+(H/2-(yn+yx)/2*view.s)*.1}
function load(){fetch('/graphdata').then(r=>r.json()).then(d=>{
 d.nodes.forEach(n=>{let o=N.get(n.id);if(!o){o={id:n.id,x:(Math.random()-.5)*240,y:(Math.random()-.5)*240,vx:0,vy:0};N.set(n.id,o)}o.h=n.h;o.t=n.t;o.m=n.m;o.p=n.p;o.gray=0});
 d.edges.forEach(e=>{[e[0],e[1]].forEach(id=>{if(!N.has(id))N.set(id,{id,x:(Math.random()-.5)*240,y:(Math.random()-.5)*240,vx:0,vy:0,h:150,t:0,m:0,p:0,gray:1})})});
 E=d.edges.map(e=>({a:N.get(e[0]),b:N.get(e[1]),w:e[2]})).filter(e=>e.a&&e.b);
}).catch(()=>{})}
load();setInterval(load,10000);
function step(){let ns=[...N.values()];
 for(let i=0;i<ns.length;i++){let a=ns[i];for(let j=i+1;j<ns.length;j++){let b=ns[j];let dx=a.x-b.x,dy=a.y-b.y,d2=dx*dx+dy*dy+.01,d=Math.sqrt(d2),f=700/d2,fx=f*dx/d,fy=f*dy/d;a.vx+=fx;a.vy+=fy;b.vx-=fx;b.vy-=fy}}
 E.forEach(e=>{let dx=e.b.x-e.a.x,dy=e.b.y-e.a.y,d=Math.sqrt(dx*dx+dy*dy)+.01,L=45+130/(1+e.w/40),f=(d-L)*.01,fx=f*dx/d,fy=f*dy/d;e.a.vx+=fx;e.a.vy+=fy;e.b.vx-=fx;e.b.vy-=fy});
 ns.forEach(n=>{n.vx-=n.x*.002;n.vy-=n.y*.002;n.vx*=.85;n.vy*=.85;n.x+=n.vx;n.y+=n.vy})}
function draw(){cx.clearRect(0,0,W,H);cx.save();cx.translate(view.x,view.y);cx.scale(view.s,view.s);
 E.forEach(e=>{cx.strokeStyle='rgba(130,160,220,'+Math.min(.75,.1+e.w/500)+')';cx.lineWidth=Math.min(5,.5+e.w/100);cx.beginPath();cx.moveTo(e.a.x,e.a.y);cx.lineTo(e.b.x,e.b.y);cx.stroke()});
 N.forEach(n=>{let r=4+Math.sqrt(n.m||0)*2.2;cx.globalAlpha=n.p?1:.55;cx.beginPath();cx.arc(n.x,n.y,r,0,7);cx.fillStyle=n.gray?'#3a4a6a':col(n.h);cx.fill();cx.globalAlpha=1;if(n.t==2){cx.strokeStyle='#ffd24d';cx.lineWidth=2.5;cx.stroke()}});
 cx.restore()}
function loop(){step();step();if(auto)fit();draw();requestAnimationFrame(loop)}loop();
let drag=null;
cv.addEventListener('pointerdown',e=>{auto=false;drag={x:e.clientX,y:e.clientY,vx:view.x,vy:view.y}});
addEventListener('pointermove',e=>{if(drag){view.x=drag.vx+(e.clientX-drag.x);view.y=drag.vy+(e.clientY-drag.y)}});
addEventListener('pointerup',()=>drag=null);
cv.addEventListener('wheel',e=>{e.preventDefault();auto=false;view.s*=e.deltaY<0?1.1:.9},{passive:false});
</script></body></html>)GRF";

void handleGraph()    { server.send_P(200, "text/html", GRAPH_HTML); }

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
<div class="sec">pubblico nei paraggi (BLE)</div>
<div class="cards">
 <div class="card"><div class="n" id="cnow">0</div><div class="l">telefoni vicini</div></div>
 <div class="card"><div class="n" id="cpeak">0</div><div class="l">picco</div></div>
</div>
<canvas id="spark" style="width:100%;height:44px;display:block;margin-top:8px"></canvas>
<small id="cnote"></small>
<div class="sec">andamento serata &mdash; mood</div>
<canvas id="tmood" style="width:100%;height:80px;display:block"></canvas>
<small style="margin-top:3px">chill &middot; social &middot; party &middot; rasta</small>
<div class="sec">andamento serata &mdash; colori</div>
<canvas id="tcol" style="width:100%;height:80px;display:block"></canvas>
<div class="sec">tutti i badge (<span id="cnt">0</span>)</div>
<table><thead><tr>
 <th data-k="id">ID</th><th data-k="type">tipo</th><th data-k="mood">mood</th>
 <th data-k="met">incontri</th><th data-k="vis">visite</th><th data-k="pre">qui</th>
</tr></thead><tbody id="rows"></tbody></table>
<a class="dl" href="/graph">&#128376;&#65039; rete degli incontri (grafo)</a>
<a class="dl" href="/data.csv">scarica CSV</a>
<small id="ts"></small>
<script>
let sortK='met',sortDir=-1;
function hsv(h){return 'hsl('+(h/255*360)+',80%,55%)'}
function bars(el,items,max){el.innerHTML=items.map(it=>{
 let w=max?Math.round(it.v/max*100):0;
 return '<div class="bar"><span style="width:'+w+'%"></span><b>'+it.l+'</b><i>'+it.v+'</i></div>'}).join('')}
function spark(a){let cv=document.getElementById('spark');if(!cv||!a)return;let W=cv.width=cv.clientWidth||300,H=cv.height=44,x=cv.getContext('2d');x.clearRect(0,0,W,H);if(a.length<2)return;let mx=Math.max(1,...a),pt=i=>[i/(a.length-1)*W,H-a[i]/mx*(H-3)-2];x.beginPath();x.moveTo(0,H);for(let i=0;i<a.length;i++){let p=pt(i);x.lineTo(p[0],p[1])}x.lineTo(W,H);x.closePath();x.fillStyle='rgba(255,94,156,.22)';x.fill();x.beginPath();for(let i=0;i<a.length;i++){let p=pt(i);i?x.lineTo(p[0],p[1]):x.moveTo(p[0],p[1])}x.strokeStyle='#ff5e9c';x.lineWidth=1.6;x.stroke()}
function stacked(cid,rows,colorFn){let cv=document.getElementById(cid);if(!cv)return;let W=cv.width=cv.clientWidth||300,H=cv.height=80,x=cv.getContext('2d');x.clearRect(0,0,W,H);if(!rows||!rows.length)return;let mx=1;rows.forEach(r=>{let t=r.reduce((a,b)=>a+b,0);if(t>mx)mx=t});let bw=W/rows.length;for(let i=0;i<rows.length;i++){let r=rows[i],y=H;for(let k=0;k<r.length;k++){let h=r[k]/mx*(H-1);if(h<=0)continue;x.fillStyle=colorFn(k,r.length);x.fillRect(i*bw,y-h,bw+0.6,h);y-=h}}}
const MOODCOL=['#3aa0ff','#3ad17a','#ff5e9c','#ffd24d'];
function loadTimeline(){fetch('/timeline').then(r=>r.json()).then(t=>{stacked('tmood',t.moods,k=>MOODCOL[k]);stacked('tcol',t.colors,(k,n)=>'hsl('+((k+0.5)/n*360)+',75%,55%)')}).catch(()=>{})}
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
   [['chill',d.agg.mood[0]],['social',d.agg.mood[1]],['party',d.agg.mood[2]],['rasta',d.agg.mood[3]||0]].map(x=>({l:x[0],v:x[1]})),mm);
 // pubblico (dispositivi BLE nei paraggi)
 if(d.crowd){document.getElementById('cnow').textContent=d.crowd.now;
  document.getElementById('cpeak').textContent=d.crowd.peak;
  let z=d.crowd.zones>1?(' · rete '+d.crowd.net+' in '+d.crowd.zones+' zone'):'';
  document.getElementById('cnote').textContent=d.crowd.apple+' Apple · ~'+d.crowd.rssi+' dBm · '+d.crowd.total+' passaggi'+z;
  spark(d.crowd.hist);}
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
loadTimeline(); setInterval(loadTimeline,30000);
</script></body></html>)HTML";

void handleRoot() { server.send_P(200, "text/html", INDEX_HTML); }

// ═══════════════════════════════════════════════════════
//  SETUP / LOOP
// ═══════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== PAMALI base station ===");
  Serial.println("comandi: d=dump  s=salva  r=reset  g=gossip LoRa");

  tblMutex   = xSemaphoreCreateMutex();
  edgeMutex  = xSemaphoreCreateMutex();
  phoneMutex = xSemaphoreCreateMutex();

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
  else { loadStats(); loadEdges(); }

  // LoRa per il sync del grafo tra piu' Heltec (opzionale: se assente, prosegue)
  initLoRa();

  // ── WiFi Access Point PRIMA dello scan BLE, cosi' l'AP si stabilizza ──
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);                                  // AP sempre sveglio
  bool apOk = WiFi.softAP(AP_SSID, AP_PASS, 1, 0, 4);    // canale 1, max 4 client
  IPAddress ip = WiFi.softAPIP();
  Serial.printf("WiFi AP '%s' (pw '%s') %s  →  http://%s\n",
    AP_SSID, AP_PASS, apOk ? "OK" : "FALLITO", ip.toString().c_str());
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/data.csv", handleCSVweb);
  server.on("/graph", handleGraph);
  server.on("/graphdata", handleGraphData);
  server.on("/edges.csv", handleEdgesCSV);
  server.on("/timeline", handleTimeline);
  server.begin();

  // ── BLE scan in DUTY-CYCLE (NON 100%! la finestra continua soffoca il WiFi
  //    e l'access point non riesce ad autenticare il telefono) ──
  NimBLEDevice::init("PAMALI-BASE");
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(new CollectorCB(), true);      // wantDuplicates=true
  scan->setActiveScan(false);
  scan->setInterval(160);   // periodo 100ms
  scan->setWindow(64);      // acceso 40ms → 40% duty: lascia tempo radio al WiFi
  scan->start(0, false);

  Serial.println("scan avviato.\n");
}

uint32_t lastSave = 0, lastSummary = 0, lastDraw = 0, lastGossip = 0, lastEdgeSave = 0, lastCompact = 0, lastCrowdSample = 0, lastMoodSample = 0;

void loop() {
  handleSerial();
  server.handleClient();          // serve le richieste del telefono
  uint32_t now = millis();

  // LoRa: ricevi archi da altri Heltec e fai broadcast dei tuoi (gossip)
  if (loraRxFlag) { loraRxFlag = false; processLoRaRx(); }
  if (loraOK && now - lastGossip >= GOSSIP_MS) { sendGossip(); lastGossip = now; }

  if (now - lastDraw >= 500) { drawDisplay(); lastDraw = now; }

  // salva su flash ogni 30s se ci sono novita'
  if (dirty      && now - lastSave     >= 30000) { saveStats(); lastSave     = now; }
  if (edgesDirty && now - lastEdgeSave >= 30000) { saveEdges(); lastEdgeSave = now; }

  // pubblico: ripulisci i dispositivi spariti ogni 20s e campiona lo storico
  if (now - lastCompact >= 20000) { compactPhones(); lastCompact = now; }
  if (now - lastCrowdSample >= CROWD_SAMPLE_MS) { pushCrowdHist(crowdStats().now); lastCrowdSample = now; }
  if (now - lastMoodSample  >= MOODHIST_MS)     { sampleMoodHist();                 lastMoodSample  = now; }

  // riepilogo seriale ogni 15s
  if (now - lastSummary >= 15000) {
    Agg a = computeAgg();
    Crowd cr = crowdStats();
    Serial.printf("[%lus] badge=%u presenti=%u mediaIncontri=%.2f record=%u (cuori=%u soli=%u creatori=%u) archi=%u | pubblico: %u ora (picco %u, %u Apple)\n",
      now / 1000, a.total, a.present, a.avgMet, a.maxMet, a.cuori, a.soli, a.creatori, edgeCount,
      cr.now, cr.peak, cr.apple);
    lastSummary = now;
  }
  delay(20);
}
