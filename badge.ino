/*
 *  badge_festival.ino
 *
 *  Festival badge — BLE beacon + LED RGB + Kuramoto sync
 *
 *  Hardware:
 *    ESP32-WROOM-32E-N8
 *    12× WS2812B-2020  → GPIO 16
 *    Tactile switch    → GPIO 14  (pull-up interno)
 *    LDR GL5528        → GPIO 34  (+ 10kΩ verso GND)
 *
 *  Librerie (Library Manager):
 *    NimBLE-Arduino  ≥ 2.0   (h2zero)   <-- API 2.x
 *    FastLED         ≥ 3.6
 */

#include <NimBLEDevice.h>
#include <FastLED.h>
#include <Preferences.h>
#include <math.h>

// ═══════════════════════════════════════════════════════
//  HARDWARE
// ═══════════════════════════════════════════════════════
#define LED_PIN       16
#define BUTTON_PIN    14
#define LDR_PIN       34

// ── TIPO DI BADGE: imposta QUI quale stai programmando ──
// (build_all.sh li sovrascrive da riga di comando con -DBADGE_TYPE=... )
#define BADGE_CUORE   0
#define BADGE_SOLE    1
#define BADGE_FUNGO   2       // badge di test/debug (7 LED)
#ifndef BADGE_TYPE
  #define BADGE_TYPE  BADGE_CUORE
#endif

#if   BADGE_TYPE == BADGE_CUORE
  #define NUM_LEDS    22
#elif BADGE_TYPE == BADGE_SOLE
  #define NUM_LEDS    22
#else
  #define NUM_LEDS    7        // FUNGO (test)
#endif

#ifndef IS_CREATOR
  #define IS_CREATOR  0      // 1 SOLO sul TUO badge (il creatore). Gli altri 0.
#endif
#define CREATOR_HUE   38      // tonalita' ORO della "firma" del creatore (FastLED hue)

// ═══════════════════════════════════════════════════════
//  BLE
// ═══════════════════════════════════════════════════════
static const uint16_t COMPANY_ID  = 0xFF01;
static const uint8_t  PROTO_VER   = 1;

// ═══════════════════════════════════════════════════════
//  RSSI / DISTANZA
// ═══════════════════════════════════════════════════════
// Tarati su misure REALI dei badge NUOVI @+9dBm, indossati:
//   20cm→-67   70cm→-79   1.5m→-84   3m→-92   (modello ≈ -82 - 21·log10 d)
static const int8_t  RSSI_IN    = -82;  // entra PAIRED / fusione piena  (~1m)
static const int8_t  RSSI_OUT   = -88;  // coupling fasi + tribù; USCITA (~2m)
static const int8_t  RSSI_SENSE = -90;  // entra SENSING / rilevamento (~7m sul campo)
static const float   EMA_ALPHA  = 0.2f;  // piu' filtrato: RSSI debole/rumoroso (antenna reale)

// ═══════════════════════════════════════════════════════
//  KURAMOTO
// ═══════════════════════════════════════════════════════
static const float   OMEGA          = 2.0f * PI / 1.2f; // periodo battito 1.2s
static const float   K_COUPLING     = 6.0f;  // forza aggancio fasi (2K=12 > max Δω≈2.6)
static const float   MOOD_SPEED[4]  = { 0.4f, 1.0f, 1.3f, 1.0f }; // chill, social, party, rasta
static const uint8_t MOOD_BRIGHT[4] = {  80,   170,  220,  180 }; // valore FastLED max
// Mood 3 "RASTA": gruppo di LED sempre accesi che cicla rosso/oro/verde + resto che batte
static const uint8_t  RASTA_STEADY[]   = { 4,5,6,7,8,9,10,18,19,20 }; // LED fissi (numeraz. 1-based)
static const uint32_t RASTA_CYCLE_MS   = 4000;  // cambio colore del gruppo fisso
static const uint32_t RASTA_BEAT_MS    = 6000;  // periodo battito degli altri LED (lento)
static const uint8_t  RASTA_HUES[3]    = { 0, 38, 96 };  // rosso, oro, verde
// Quando due o piu' badge sono vicini battono ALLO STESSO RITMO COMUNE,
// indipendente dal mood (Δω=0 → le fasi si agganciano a sfasamento ZERO).
// Allontanandosi ciascuno torna al ritmo + luminosita' del proprio mood.
static const float    TOGETHER_SPEED  = 1.0f; // ritmo comune da vicini (pace "social")
static const uint8_t  TOGETHER_BRIGHT = 200;  // luminosita' comune da vicini
static const uint32_t CHILL_BREATH_MS = 10000;// durata di un respiro completo in chill

// ── MAPPA "SOCIAL": livello a cui si accende ogni LED (indice 0-based) ──
// Riempimento secondo il cablaggio FISICO del badge; svuotamento al contrario.
#if   BADGE_TYPE == BADGE_CUORE
// Sequenza: 15 → 16,14 → 1,22,13 → 2,17,12 → 3,21,11 → 20 → 4,18,19,10 → 5,9 → 6,8 → 7
static const uint8_t  SOCIAL_LEVEL[NUM_LEDS] =
  { 2,3,4,6,7,8,9,8,7,6,4,3,2,1,0,1,3,6,6,5,4,2 };
static const uint8_t  SOCIAL_MAXLEVEL = 9;
#elif BADGE_TYPE == BADGE_SOLE
static const uint8_t  SOCIAL_LEVEL[NUM_LEDS] =
  { 6,8,6,9,10,9,11,7,8,6,5,4,4,3,1,2,0,2,1,4,2,5 };
static const uint8_t  SOCIAL_MAXLEVEL = 11;
#else  // FUNGO (test)
static const uint8_t  SOCIAL_LEVEL[NUM_LEDS] = { 0,1,1,0,1,1,2 };
static const uint8_t  SOCIAL_MAXLEVEL = 2;
#endif
static const uint32_t SOCIAL_FILL_MS = 6000;  // riempimento+svuotamento completo (piu' lento)

// ═══════════════════════════════════════════════════════
//  TIMING
// ═══════════════════════════════════════════════════════
static const uint32_t NB_TIMEOUT_MS     =   4000;
static const uint32_t TRIBE_TIME_MS     =   3000; // >2 badge per 3s → entra in TRIBE
static const uint32_t GROUP_FUSION_MS   = 240000; // gruppo: fusione piena. 2400000ms = 4 MINUTI
                                                   // (40s = 40000, 4min = 240000, 10min = 600000)
static const uint32_t HB_INTERVAL_MS   = 600000; // heartbeat ogni 10 min
static const uint32_t HB_DUR_MS        =   5000;
static const uint32_t ADV_INTERVAL_MS  =    160;  // ripubblica payload piu' spesso (fase fresca)
static const uint32_t NVS_SAVE_MS      =  30000;
static const uint32_t MATURITY_FULL_MS = 1800000; // 30 MIN di vicinanza totale = fusione piena
static const uint32_t FUSION_GRACE_MS  =   5000; // ad ogni avvicinamento: 5s prima di (ri)fondere
static const uint32_t FUSION_RESUME_MS =   2000; // dissolvenza nel riprendere dalla soglia salvata
static const uint32_t FAREWELL_DUR_MS  =   1500;
static const uint32_t RECOG_DUR_MS     =    600;

static const uint16_t NIGHT_THRESH     =    800; // ADC 0-4095: sotto = notte

// ═══════════════════════════════════════════════════════
//  STRUTTURE DATI
// ═══════════════════════════════════════════════════════
struct __attribute__((packed)) BeaconPayload {
  uint16_t companyId;
  uint8_t  proto;
  uint8_t  type;
  uint8_t  id[3];
  uint8_t  hue;
  uint8_t  mood;
  uint8_t  phase;       // 0-255 → 0-2π
  uint32_t tConsensus;  // ms festival time
  uint16_t peopleMet;   // persone distinte conosciute (per le statistiche)
  // ── GRAFO: 2 contatti a rotazione (ID + peso) per costruire la rete ──
  uint8_t  cPage;       // indice pagina contatti che sto trasmettendo
  uint8_t  cId[2][3];   // ID di 2 miei contatti (0,0,0 = slot vuoto)
  uint8_t  cW[2];       // peso = tempo di contatto in unità da 4s (cap 255 = ~17min)
};  // 25 byte: stanno in un beacon BLE (31 max)

// ── Lista contatti del badge (per il grafo), persistente e trasmessa ──
#define MAX_CONTACTS 100
struct __attribute__((packed)) Contact {
  uint8_t  id[3];       // ID del contatto
  uint16_t secs;        // tempo di contatto cumulativo in secondi
};

#define MAX_NB 10

struct Neighbor {
  uint32_t idHash;
  int8_t   rssiFilt;
  uint8_t  hue;
  uint8_t  mood;
  float    phase;
  uint32_t lastSeen;
  uint32_t firstPaired;
  uint32_t sessionMs;        // durata dell'incontro IN CORSO con questo vicino (rodaggio 5s)
  uint32_t totalContactMs;
  bool     knownBefore;
  bool     isCreator;        // il vicino e' il badge del CREATORE
};

enum State : uint8_t { IDLE, SENSING, PAIRED, TRIBE, FAREWELL };

// ═══════════════════════════════════════════════════════
//  VARIABILI GLOBALI
// ═══════════════════════════════════════════════════════
CRGB        leds[NUM_LEDS];
Preferences prefs;

uint8_t  myHue        = 0;
uint8_t  myMood       = 1;
float    myPhase      = 0.0f;
State    myState      = IDLE;
uint32_t t0Offset     = 0;
uint32_t myId         = 0;   // 3 byte UNIVOCI (NIC del MAC), calcolato in setup

Neighbor neighbors[MAX_NB];
uint8_t  nbCount      = 0;

Contact  contacts[MAX_CONTACTS];   // tutte le persone incontrate (per il grafo)
uint8_t  contactCount = 0;
uint8_t  contactPage  = 0;         // pagina corrente trasmessa nel beacon

int8_t   partnerId    = -1;
uint32_t groupMs      = 0;   // da quanto il gruppo (>2 badge) è insieme → fusione gruppo
uint32_t farewellStart= 0;

// (la sessione d'incontro e' ora per-vicino: Neighbor.sessionMs)

bool     recognitionPending = false;
uint32_t recognitionStart   = 0;

bool     creatorSparkPending = false;   // scintilla dorata "hai incontrato il creatore"
uint32_t creatorSparkStart   = 0;
uint16_t peopleMet           = 0;       // persone distinte conosciute (persistente NVS)

int      ldrFiltered  = 2048;
bool     isNight      = false;

// Debug fusione colori + modalita' LED (aggiornati in renderLEDs)
uint8_t  dbgShowHue   = 0;      // tono effettivamente mostrato (dopo fusione)
bool     dbgTogether  = false;  // true = LED pulsano ASSIEME, false = onda

NimBLEAdvertising* pAdv = nullptr;

uint32_t lastAdvMs   = 0;
uint32_t lastNvsMs   = 0;
uint32_t lastLDRms   = 0;
uint32_t lastLoopMs  = 0;
uint32_t bootMs      = 0;

SemaphoreHandle_t nbMutex = nullptr;

// ═══════════════════════════════════════════════════════
//  UTILITY
// ═══════════════════════════════════════════════════════

uint32_t festivalTime() {
  return millis() - bootMs + t0Offset;
}

// Eta' di un timestamp, SENZA underflow. Il callback BLE gira su un altro task e
// puo' aggiornare lastSeen DOPO che il loop ha letto 'now': in quel caso
// (now - lastSeen) va in underflow a ~4.29e9 e il vicino risulta "scaduto"
// → sessioni azzerate, vicini esclusi dal coupling, sync che salta.
static inline uint32_t ageMs(uint32_t now, uint32_t stamp) {
  return (now >= stamp) ? (now - stamp) : 0;
}

// Blend circolare hue FastLED (0-255). w=0→a, w=1→b
uint8_t blendHue(uint8_t a, uint8_t b, float w) {
  int diff = (int)b - (int)a;
  if (diff >  128) diff -= 256;
  if (diff < -128) diff += 256;
  return (uint8_t)((int)a + (int)roundf(diff * w));
}

// Media circolare hue (FastLED 0-255)
uint8_t circularMeanHue(uint8_t* hues, uint8_t n) {
  if (n == 0) return 0;
  float sx = 0.0f, sy = 0.0f;
  for (uint8_t i = 0; i < n; i++) {
    float a = (float)hues[i] / 255.0f * 2.0f * PI;
    sx += cosf(a);
    sy += sinf(a);
  }
  float mean = atan2f(sy, sx);
  if (mean < 0) mean += 2.0f * PI;
  return (uint8_t)(mean / (2.0f * PI) * 255.0f);
}

// ═══════════════════════════════════════════════════════
//  LISTA CONTATTI (per il grafo)  —  chiamare con mutex preso
// ═══════════════════════════════════════════════════════
int findContactByHash(uint32_t h) {
  uint8_t a = h >> 16, b = h >> 8, c = h;
  for (uint8_t i = 0; i < contactCount; i++)
    if (contacts[i].id[0] == a && contacts[i].id[1] == b && contacts[i].id[2] == c) return i;
  return -1;
}

// Inserisce/aggiorna un contatto col suo peso (secondi). Tiene il valore piu' alto.
void upsertContact(uint32_t h, uint32_t secs) {
  if (secs > 65535UL) secs = 65535UL;
  int idx = findContactByHash(h);
  if (idx >= 0) {
    if ((uint16_t)secs > contacts[idx].secs) contacts[idx].secs = (uint16_t)secs;
    return;
  }
  if (contactCount < MAX_CONTACTS) {
    idx = contactCount++;
  } else {                                  // pieno: sostituisci il piu' debole
    idx = 0;
    for (uint8_t i = 1; i < contactCount; i++)
      if (contacts[i].secs < contacts[idx].secs) idx = i;
    if (contacts[idx].secs >= (uint16_t)secs) return;   // il nuovo non batte il minimo
  }
  contacts[idx].id[0] = h >> 16;
  contacts[idx].id[1] = h >> 8;
  contacts[idx].id[2] = h;
  contacts[idx].secs  = (uint16_t)secs;
}

// ═══════════════════════════════════════════════════════
//  NEIGHBOR TABLE
// ═══════════════════════════════════════════════════════

// Cerca o crea un neighbor. Ritorna indice nel array, -1 se impossibile.
// CHIAMARE con mutex preso.
int8_t findOrCreate(uint32_t hash) {
  // cerca esistente
  for (uint8_t i = 0; i < nbCount; i++)
    if (neighbors[i].idHash == hash) return i;

  int8_t slot;
  if (nbCount < MAX_NB) {
    slot = (int8_t)nbCount++;
  } else {
    // sostituisci il neighbor più vecchio tra quelli non attivi
    uint32_t oldest = millis();
    slot = 0;
    for (uint8_t i = 0; i < MAX_NB; i++) {
      if (neighbors[i].lastSeen < oldest) {
        oldest = neighbors[i].lastSeen;
        slot   = i;
      }
    }
    // riversa il vecchio nella lista contatti (grafo + persistenza)
    if (neighbors[slot].totalContactMs > 0)
      upsertContact(neighbors[slot].idHash, neighbors[slot].totalContactMs / 1000);
  }

  // carica il tempo gia' accumulato con questa persona (dalla lista contatti)
  int ci = findContactByHash(hash);
  uint32_t saved = (ci >= 0) ? (uint32_t)contacts[ci].secs * 1000UL : 0;

  neighbors[slot] = {
    .idHash       = hash,
    .rssiFilt     = -128,   // sentinella "mai misurato": la 1a lettura aggancia diretta
    .hue          = 0,
    .mood         = 1,
    .phase        = 0.0f,
    .lastSeen     = millis(),
    .firstPaired  = 0,
    .sessionMs    = 0,
    .totalContactMs = saved,
    .knownBefore  = (saved > 0)
  };
  return slot;
}

// ═══════════════════════════════════════════════════════
//  BLE — SCAN CALLBACK
// ═══════════════════════════════════════════════════════
class ScanCB : public NimBLEScanCallbacks {              // NimBLE 2.x
  void onResult(const NimBLEAdvertisedDevice* dev) override {
    if (!dev->haveManufacturerData()) return;
    std::string mfg = dev->getManufacturerData();
    if (mfg.size() < sizeof(BeaconPayload)) return;

    const BeaconPayload* p = (const BeaconPayload*)mfg.data();
    if (p->companyId != COMPANY_ID || p->proto != PROTO_VER) return;

    uint32_t hash = ((uint32_t)p->id[0] << 16) |
                    ((uint32_t)p->id[1] << 8)  |
                     (uint32_t)p->id[2];
    if (hash == myId) return;   // ignora un eventuale eco del proprio beacon

    // consensus T0: adotta il valore più alto
    uint32_t theirFT = p->tConsensus;
    uint32_t myFT    = festivalTime();
    if (theirFT > myFT + 500) {
      t0Offset += (theirFT - myFT);
    }

    if (xSemaphoreTake(nbMutex, 0) == pdTRUE) {
      int8_t slot = findOrCreate(hash);
      if (slot >= 0) {
        float raw = (float)dev->getRSSI();
        if (neighbors[slot].rssiFilt == -128) {
          neighbors[slot].rssiFilt = (int8_t)raw;            // 1a misura: aggancio diretto
        } else {
          float prev = (float)neighbors[slot].rssiFilt;
          neighbors[slot].rssiFilt = (int8_t)(EMA_ALPHA * raw + (1.0f - EMA_ALPHA) * prev);
        }
        neighbors[slot].hue      = p->hue;
        neighbors[slot].mood     = p->mood;
        neighbors[slot].phase    = (float)p->phase / 255.0f * 2.0f * PI;
        neighbors[slot].lastSeen = millis();
        neighbors[slot].isCreator = (p->type & 0x80) != 0;   // bit 7 = creatore
      }
      xSemaphoreGive(nbMutex);
    }
  }
};

// ═══════════════════════════════════════════════════════
//  BLE — ADVERTISING
// ═══════════════════════════════════════════════════════
void publishBeacon() {
  BeaconPayload p{};
  p.companyId  = COMPANY_ID;
  p.proto      = PROTO_VER;
  p.type       = BADGE_TYPE | (IS_CREATOR ? 0x80 : 0);   // bit 7 = creatore
  p.id[0]      = (uint8_t)(myId >> 16);
  p.id[1]      = (uint8_t)(myId >>  8);
  p.id[2]      = (uint8_t)(myId);
  p.hue        = myHue;
  p.mood       = myMood;
  p.phase      = (uint8_t)(myPhase / (2.0f * PI) * 255.0f);
  p.tConsensus = festivalTime();
  p.peopleMet  = peopleMet;

  // ── pagina contatti a rotazione (2 archi per beacon) per costruire il grafo ──
  if (xSemaphoreTake(nbMutex, pdMS_TO_TICKS(3)) == pdTRUE) {
    uint8_t nPages = (contactCount + 1) / 2;
    if (nPages == 0) nPages = 1;
    uint8_t pg = contactPage % nPages;
    p.cPage = pg;
    for (uint8_t k = 0; k < 2; k++) {
      uint8_t ci = pg * 2 + k;
      if (ci < contactCount) {
        p.cId[k][0] = contacts[ci].id[0];
        p.cId[k][1] = contacts[ci].id[1];
        p.cId[k][2] = contacts[ci].id[2];
        uint32_t q  = contacts[ci].secs / 4;       // peso in unità da 4s
        p.cW[k]     = (q > 255) ? 255 : (uint8_t)q;
      }
    }
    contactPage++;
    xSemaphoreGive(nbMutex);
  }

  NimBLEAdvertisementData data;
  data.setManufacturerData((const uint8_t*)&p, sizeof(p));   // NimBLE 2.x

  pAdv->stop();
  pAdv->setAdvertisementData(data);
  pAdv->start();
}

// ═══════════════════════════════════════════════════════
//  KURAMOTO PHASE UPDATE
// ═══════════════════════════════════════════════════════
void updatePhase(float dt) {
  float    pull = 0.0f;
  uint8_t  n    = 0;
  uint32_t now  = millis();

  // Da vicini (PAIRED/TRIBE) si batte a un RITMO COMUNE mood-indipendente, cosi'
  // le fasi si agganciano a sfasamento zero. Da soli, ritmo del proprio mood.
  bool  together = (myState == PAIRED || myState == TRIBE);
  float mySpeed  = together ? TOGETHER_SPEED : MOOD_SPEED[myMood];

  if (xSemaphoreTake(nbMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    for (uint8_t i = 0; i < nbCount; i++) {
      uint32_t age = ageMs(now, neighbors[i].lastSeen);
      if (age > NB_TIMEOUT_MS)               continue;
      if (neighbors[i].rssiFilt < RSSI_OUT)  continue;
      // La fase ricevuta e' "vecchia" di 'age' ms: nel frattempo il vicino e'
      // andato avanti. La estrapoliamo in avanti, altrimenti con beacon ogni
      // ~300-1000ms il dato e' scorrelato e il pull si media a zero. Se siamo
      // "together" anche lui batte al ritmo comune → uso TOGETHER_SPEED.
      uint8_t nm    = (neighbors[i].mood < 4) ? neighbors[i].mood : 1;
      float   nbSpd = together ? TOGETHER_SPEED : MOOD_SPEED[nm];
      float   est   = neighbors[i].phase
                    + OMEGA * nbSpd * ((float)age / 1000.0f);
      pull += sinf(est - myPhase);
      n++;
    }
    xSemaphoreGive(nbMutex);
  }

  if (n > 0) pull /= n;
  myPhase += (OMEGA * mySpeed + K_COUPLING * pull) * dt;
  while (myPhase >= 2.0f * PI) myPhase -= 2.0f * PI;
  while (myPhase <  0.0f)      myPhase += 2.0f * PI;
}

// ═══════════════════════════════════════════════════════
//  STATE MACHINE
// ═══════════════════════════════════════════════════════
void updateState(float dt) {
  uint32_t now = millis();

  // Farewell in corso: aspetta fine
  if (myState == FAREWELL) {
    if (now - farewellStart >= FAREWELL_DUR_MS) {
      myState   = IDLE;
      partnerId = -1;
    }
    return;
  }

  if (xSemaphoreTake(nbMutex, pdMS_TO_TICKS(2)) != pdTRUE) return;

  // Trova vicino migliore e conta i paired-grade
  int8_t  bestSlot  = -1;
  int8_t  bestRssi  = RSSI_SENSE;
  uint8_t pairedGrade = 0;

  for (uint8_t i = 0; i < nbCount; i++) {
    if (ageMs(now, neighbors[i].lastSeen) > NB_TIMEOUT_MS) continue;
    if (neighbors[i].rssiFilt > bestRssi) {
      bestRssi = neighbors[i].rssiFilt;
      bestSlot = i;
    }
    if (neighbors[i].rssiFilt > RSSI_OUT) pairedGrade++;
  }

  // Transizioni
  if (bestSlot >= 0 && bestRssi > RSSI_IN) {

    // Primo ingresso in PAIRED (da stato non-paired)
    if (myState != PAIRED && myState != TRIBE) {
      recognitionPending = true;
      recognitionStart   = now;
      bool firstTime  = (neighbors[bestSlot].firstPaired == 0);
      bool newPerson  = firstTime && !neighbors[bestSlot].knownBefore;
      if (firstTime)
        neighbors[bestSlot].firstPaired = now;
      // contatore persone DISTINTE conosciute (persistente)
      if (newPerson) {
        peopleMet++;
        prefs.putUShort("met", peopleMet);
      }
      // assicura che questo contatto sia nella lista del grafo da subito
      upsertContact(neighbors[bestSlot].idHash, neighbors[bestSlot].totalContactMs / 1000);
      // ── SCINTILLA DEL CREATORE ──
      // se incontro il creatore (e non lo sono io), oppure se IO sono il creatore
      // e conosco qualcuno di nuovo → scintilla dorata su entrambi i badge.
      if ((neighbors[bestSlot].isCreator && !IS_CREATOR) ||
          (IS_CREATOR && firstTime)) {
        creatorSparkPending = true;
        creatorSparkStart   = now;
      }
    }
    myState   = PAIRED;
    partnerId = bestSlot;

  } else if (bestSlot >= 0 && bestRssi > RSSI_SENSE) {
    if (myState == PAIRED || myState == TRIBE) {
      myState       = FAREWELL;
      farewellStart = now;
    } else {
      myState   = SENSING;
      partnerId = -1;
    }
  } else {
    if (myState == PAIRED || myState == TRIBE) {
      myState       = FAREWELL;
      farewellStart = now;
    } else {
      myState   = IDLE;
      partnerId = -1;
    }
  }

  // Gruppo (>2 badge): conta da quanto è insieme. Dopo TRIBE_TIME_MS entra in
  // TRIBE; groupMs serve poi per la fusione di gruppo (~40s). Si azzera se il
  // gruppo si rompe (scende a 2 badge o meno).
  if (myState == PAIRED && pairedGrade >= 2) {
    groupMs += (uint32_t)(dt * 1000.0f);
    if (groupMs >= TRIBE_TIME_MS) myState = TRIBE;
  } else {
    groupMs = 0;
  }

  // ── Sessione d'incontro, PER OGNI VICINO (non solo il "migliore"): ad ogni
  //    avvicinamento la fusione riprende dopo FUSION_GRACE_MS (5s) e il livello
  //    riparte dalla soglia gia' raggiunta (totalContactMs persistente).
  //    NB: era una sola sessione legata al partner; in un gruppo il partner
  //    rimbalza tra i badge e il rodaggio ripartiva in continuazione, perdendo
  //    meta' del tempo di contatto. Ora ognuno ha la sua sessione e in una
  //    tribu' si accumula contatto con TUTTI quelli vicini, non solo con uno.
  uint32_t dtMs = (uint32_t)(dt * 1000.0f);
  for (uint8_t i = 0; i < nbCount; i++) {
    bool together = (ageMs(now, neighbors[i].lastSeen) <= NB_TIMEOUT_MS) &&
                    (neighbors[i].rssiFilt > RSSI_OUT);   // stessa isteresi del pairing
    if (together) {
      neighbors[i].sessionMs += dtMs;
      if (neighbors[i].sessionMs >= FUSION_GRACE_MS)
        neighbors[i].totalContactMs += dtMs;
    } else {
      neighbors[i].sessionMs = 0;      // separati → al prossimo incontro rodaggio da capo
    }
  }

  xSemaphoreGive(nbMutex);
}

// ═══════════════════════════════════════════════════════
//  NIGHT MODE
// ═══════════════════════════════════════════════════════
void updateLDR() {
  int raw   = analogRead(LDR_PIN);
  ldrFiltered = (ldrFiltered * 7 + raw) / 8;
  isNight   = (ldrFiltered < NIGHT_THRESH);
}

// ═══════════════════════════════════════════════════════
//  LED RENDERING
// ═══════════════════════════════════════════════════════
void renderLEDs() {
  uint32_t now = millis();

  // Brightness: da vicini = comune (mood-indipendente), da soli = del proprio mood
  bool    together = (myState == PAIRED || myState == TRIBE);
  uint8_t maxV     = together ? TOGETHER_BRIGHT : MOOD_BRIGHT[myMood];
  if (isNight) maxV /= 2;
  uint8_t showBright = 255;   // brightness globale: il chill la abbassa per il dithering

  // Hue da mostrare + maturity
  uint8_t showHue  = myHue;
  float   maturity = 0.0f;

  if (xSemaphoreTake(nbMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    if (partnerId >= 0 && partnerId < nbCount) {
      maturity = min(1.0f,
        (float)neighbors[partnerId].totalContactMs / (float)MATURITY_FULL_MS);

      if (myState == PAIRED) {
        // gate dei 5s: la fusione (ri)compare solo dopo il rodaggio, con una
        // breve dissolvenza che riprende dalla soglia gia' raggiunta.
        float act = constrain(((float)neighbors[partnerId].sessionMs - (float)FUSION_GRACE_MS)
                              / (float)FUSION_RESUME_MS, 0.0f, 1.0f);
        showHue = blendHue(myHue, neighbors[partnerId].hue, 0.5f * maturity * act);
      }
      else if (myState == TRIBE) {
        uint8_t tribeHues[MAX_NB + 1];
        uint8_t tn = 0;
        tribeHues[tn++] = myHue;
        for (uint8_t i = 0; i < nbCount; i++) {
          if (ageMs(now, neighbors[i].lastSeen) > NB_TIMEOUT_MS) continue;
          if (neighbors[i].rssiFilt > RSSI_OUT)
            tribeHues[tn++] = neighbors[i].hue;
        }
        // fusione di GRUPPO graduale: piena in ~40s (molto piu' veloce della
        // coppia). A gMat=1 tutti mostrano la media → stesso colore condiviso.
        float gMat = min(1.0f, (float)groupMs / (float)GROUP_FUSION_MS);
        showHue = blendHue(myHue, circularMeanHue(tribeHues, tn), gMat);
      }
    }
    xSemaphoreGive(nbMutex);
  }

  // Esponi per il debug seriale: tono fuso + modalita' animazione
  dbgShowHue  = showHue;
  dbgTogether = together;

  // Pulse base: sin(phase) → 0..1  (battito ASSIEME: fondo 0%, si spegne del tutto)
  float pulseFrac = sinf(myPhase) * 0.5f + 0.5f;
  uint8_t baseV   = (uint8_t)(pulseFrac * maxV);

  // Farewell: fade out
  if (myState == FAREWELL) {
    float prog = min(1.0f, (float)(now - farewellStart) / (float)FAREWELL_DUR_MS);
    baseV = (uint8_t)(baseV * (1.0f - prog));
  }

  // ── Riempi LED per stato ───────────────────────────
  switch (myState) {

    case IDLE: {
      // Da SOLO: ogni mood ha la sua animazione "personale".
      switch (myMood) {

        case 0: {  // CHILL — respiro lento 10s a PIENA risoluzione (0-255), con
                   // brightness globale FISSA a maxV → dithering sempre attivo
                   // (niente scatti) e gli overlay (aura creatore) non vengono
                   // trascinati giu' dal respiro. Curva sin(ph/2): tocca lo 0
                   // e risale subito, cima morbida.
          float   ph     = (float)(now % CHILL_BREATH_MS) / (float)CHILL_BREATH_MS * 2.0f * PI;
          float   breath = sinf(ph * 0.5f);                 // 0 → 1 → 0
          uint8_t lin    = (uint8_t)(breath * 255.0f);
          fill_solid(leds, NUM_LEDS, CHSV(myHue, 255, lin));
          showBright     = maxV;
          break;
        }

        case 1: {  // SOCIAL — riempie i LED nell'ordine della mappa fisica del
                   // badge, poi li svuota AL CONTRARIO (vedi SOCIAL_LEVEL[]).
          float u     = (float)(now % SOCIAL_FILL_MS) / (float)SOCIAL_FILL_MS;
          float tri   = 1.0f - fabsf(2.0f * u - 1.0f);          // 0 → 1 → 0
          float front = tri * (float)(SOCIAL_MAXLEVEL + 1);     // fronte d'onda
          for (int i = 0; i < NUM_LEDS; i++) {
            float b = constrain(front - (float)SOCIAL_LEVEL[i], 0.0f, 1.0f);
            leds[i] = CHSV(myHue, 255, (uint8_t)(b * maxV));
          }
          break;
        }

        case 2: {  // PARTY — arcobaleno che ruota veloce + scintillio, ignora myHue
          uint8_t spin = (uint8_t)(now / 6);            // rotazione rapida dei colori
          for (int i = 0; i < NUM_LEDS; i++) {
            uint8_t h  = (uint8_t)(spin + i * 256 / NUM_LEDS);   // ogni LED un colore
            uint8_t tw = sin8((uint8_t)(now + i * 36));          // lampeggio veloce sfasato
            uint8_t v  = map(tw, 0, 255, maxV / 4, maxV);
            leds[i] = CHSV(h, 255, v);
          }
          break;
        }

        default: {  // RASTA — i LED in RASTA_STEADY sempre accesi, colore che
                    // cicla rosso→oro→verde; gli ALTRI pulsano tutti assieme.
                    // Pixel a PIENA risoluzione (0-255) + luminosita' GLOBALE:
                    // il dithering FastLED rende continuo il passaggio
                    // minimo↔spento (niente scatto), come nel chill.
          bool steady[NUM_LEDS] = { false };
          for (uint8_t k = 0; k < sizeof(RASTA_STEADY); k++)
            if (RASTA_STEADY[k] >= 1 && RASTA_STEADY[k] <= NUM_LEDS)
              steady[RASTA_STEADY[k] - 1] = true;
          uint8_t ch  = RASTA_HUES[(now / RASTA_CYCLE_MS) % 3];
          float   bph = (float)(now % RASTA_BEAT_MS) / (float)RASTA_BEAT_MS * 2.0f * PI;
          uint8_t bv  = (uint8_t)((0.5f - 0.5f * cosf(bph)) * 255.0f); // 0→255→0 continuo
          for (int i = 0; i < NUM_LEDS; i++)
            leds[i] = steady[i] ? CHSV(ch, 255, 255) : CHSV(myHue, 255, bv);
          showBright = maxV;   // scala globale → dithering attivo, fissi restano a maxV
          break;
        }
      }
      break;
    }

    case SENSING: {
      // LED "scanner": uno più luminoso ruota lentamente
      uint8_t dimV    = baseV * 25 / 100;
      uint32_t period = 2000; // ms per giro completo
      uint8_t  active = (uint8_t)((now / (period / NUM_LEDS)) % NUM_LEDS);
      for (int i = 0; i < NUM_LEDS; i++) {
        uint8_t v = (i == (int)active) ? baseV : dimV;
        leds[i] = CHSV(showHue, 255, v);
      }
      break;
    }

    case PAIRED: {
      // VICINO A QUALCUNO → tutti i LED pulsano ASSIEME (battito sincronizzato
      // Kuramoto). L'ampiezza del battito cresce con la maturita' del legame.
      uint8_t pv = (uint8_t)(baseV * (0.7f + 0.3f * maturity));
      fill_solid(leds, NUM_LEDS, CHSV(showHue, 255, pv));
      break;
    }

    case TRIBE: {
      // TRIBU' (>=3 vicini) → tutti pulsano ASSIEME a piena luminosita',
      // colore = media circolare del gruppo (tutti convergono allo stesso tono).
      fill_solid(leds, NUM_LEDS, CHSV(showHue, 255, baseV));
      break;
    }

    case FAREWELL:
      fill_solid(leds, NUM_LEDS, CHSV(showHue, 200, baseV));
      break;
  }

  // ── Overlay: festival heartbeat ───────────────────
  uint32_t ft = festivalTime();
  if ((ft % HB_INTERVAL_MS) < HB_DUR_MS) {
    float   prog  = (float)(ft % HB_INTERVAL_MS) / (float)HB_DUR_MS;
    uint8_t flash = (uint8_t)(sinf(prog * PI) * 60);
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i].r = min(255, (int)leds[i].r + flash);
      leds[i].g = min(255, (int)leds[i].g + flash);
      leds[i].b = min(255, (int)leds[i].b + flash);
    }
  }

  // ── Overlay: flash di riconoscimento/incontro ─────
  // Due flash bianchi brevi
  if (recognitionPending) {
    uint32_t elapsed = now - recognitionStart;
    if (elapsed < RECOG_DUR_MS) {
      bool on = (elapsed < 150) || (elapsed >= 250 && elapsed < 400);
      if (on) fill_solid(leds, NUM_LEDS, CRGB(180, 180, 180));
    } else {
      recognitionPending = false;
    }
  }

  // ── FIRMA DEL CREATORE: aura dorata che percorre il contorno del badge ──
  // Attiva in TUTTE le modalita' (overlay additivo dopo ogni animazione).
  if (IS_CREATOR) {
    static const uint8_t AURA_PATH[] = { 15,16,1,2,3,21,11,12,13,14 };  // giro (1-based)
    const uint8_t  AURA_N      = sizeof(AURA_PATH);
    const uint32_t AURA_LAP_MS = 2000;  // un giro completo in 2 secondi
    const uint8_t  AURA_TRAIL  = 2;     // lunghezza scia (LED)
    uint8_t head = (uint8_t)(((uint64_t)(now % AURA_LAP_MS)) * AURA_N / AURA_LAP_MS);
    // Luminosita' finale dell'aura = MASSIMO della modalita' corrente (maxV),
    // indipendente dall'animazione sotto. Se la brightness globale e' gia' la
    // scala di modalita' (chill/rasta: showBright=maxV) il pixel va a 255;
    // negli altri stati (showBright=255) il pixel va direttamente a maxV.
    uint8_t auraV = (showBright == 255) ? maxV : 255;
    for (uint8_t d = 0; d < AURA_TRAIL; d++) {
      uint8_t p = AURA_PATH[(head + AURA_N - d) % AURA_N];
      if (p < 1 || p > NUM_LEDS) continue;         // guardia per badge con meno LED
      uint8_t v = (uint8_t)((uint16_t)auraV * (AURA_TRAIL - d) / AURA_TRAIL);
      leds[p - 1] = CHSV(CREATOR_HUE, 230, v);     // SOSTITUISCE il colore del LED
    }
  }

  // ── SCINTILLA DORATA: "hai incontrato il creatore" (su entrambi i badge) ──
  if (creatorSparkPending) {
    uint32_t el = now - creatorSparkStart;
    const uint32_t SPARK_DUR = 2200;
    if (el < SPARK_DUR) {
      float fade = 1.0f - (float)el / SPARK_DUR;
      for (int i = 0; i < NUM_LEDS; i++) {
        uint8_t r = (uint8_t)((i * 73u + now / 40u) & 0xFF);   // scintillio sparso
        if (r < 95) {
          CRGB g = CHSV(CREATOR_HUE, 215, (uint8_t)(225 * fade));
          leds[i].r = min(255, (int)leds[i].r + g.r);
          leds[i].g = min(255, (int)leds[i].g + g.g);
          leds[i].b = min(255, (int)leds[i].b + g.b);
        }
      }
    } else {
      creatorSparkPending = false;
    }
  }

  FastLED.setBrightness(showBright);   // 255 per tutti gli stati; chill la modula (dithering)
  FastLED.show();
}

// ═══════════════════════════════════════════════════════
//  BUTTON
// ═══════════════════════════════════════════════════════
namespace Btn {
  bool     prev       = false;
  uint32_t pressStart = 0;
  bool     longFired  = false;
  bool     superFired = false;

  void onTap() {
    myHue += 15;               // 17 step sulla ruota colori
    prefs.putUChar("hue", myHue);
  }
  void onLong() {
    myMood = (myMood + 1) % 4;   // chill → social → party → rasta → …
    prefs.putUChar("mood", myMood);
    Serial.printf("Mood → %d\n", myMood);
  }
  void onSuperLong() {
    Serial.println("Factory reset!");
    prefs.clear();
    delay(200);
    ESP.restart();
  }

  void update() {
    bool pressed = (digitalRead(BUTTON_PIN) == LOW);
    uint32_t t   = millis();

    // DIAGNOSTICA: stampa quando il pulsante cambia stato
    if (pressed != prev)
      Serial.printf("[BTN] GPIO%d = %s\n", BUTTON_PIN, pressed ? "PREMUTO (LOW)" : "rilasciato (HIGH)");

    if (pressed && !prev) {
      pressStart  = t;
      longFired   = false;
      superFired  = false;
    }
    if (pressed && prev) {
      uint32_t held = t - pressStart;
      if (held >= 12000 && !superFired) { onSuperLong(); superFired = true; }  // reset a 12s
      else if (held >= 800 && !longFired) { onLong(); longFired = true; }
    }
    if (!pressed && prev) {
      uint32_t held = t - pressStart;
      if (held >= 30 && held < 800 && !longFired) onTap();
    }
    prev = pressed;
  }
}

// ═══════════════════════════════════════════════════════
//  NVS — SALVATAGGIO
// ═══════════════════════════════════════════════════════
void saveNVS() {
  if (xSemaphoreTake(nbMutex, pdMS_TO_TICKS(10)) != pdTRUE) return;
  // riversa i vicini attivi nella lista contatti, poi salva la lista come blob
  for (uint8_t i = 0; i < nbCount; i++)
    if (neighbors[i].totalContactMs > 0)
      upsertContact(neighbors[i].idHash, neighbors[i].totalContactMs / 1000);
  prefs.putUChar("ncont", contactCount);
  prefs.putBytes("contacts", contacts, (size_t)contactCount * sizeof(Contact));
  xSemaphoreGive(nbMutex);
  Serial.printf("[NVS] saved %u contatti\n", contactCount);
}

// ═══════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(100);
  bootMs = millis();

  // ID UNIVOCO dai 3 byte ALTI del MAC (i 3 bassi sono l'OUI Espressif, uguale per tutti!)
  uint64_t mac = ESP.getEfuseMac();
  myId = (((uint32_t)(uint8_t)(mac >> 24)) << 16) |
         (((uint32_t)(uint8_t)(mac >> 32)) <<  8) |
          ((uint32_t)(uint8_t)(mac >> 40));
  Serial.printf("myId = %06X\n", myId);

  // Mutex per la neighbor table
  nbMutex = xSemaphoreCreateMutex();

  // GPIO
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // FastLED — limita corrente a 400mA @4V (sicuro con LiPo 500mAh)
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setMaxPowerInVoltsAndMilliamps(4, 400);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();

  // NVS
  prefs.begin("badge", false);
  myHue   = prefs.getUChar("hue",  (uint8_t)random(0, 255));
  myMood  = prefs.getUChar("mood", 1);
  peopleMet = prefs.getUShort("met", 0);   // persone conosciute (persistente)
  // lista contatti (grafo) salvata
  contactCount = prefs.getUChar("ncont", 0);
  if (contactCount > MAX_CONTACTS) contactCount = MAX_CONTACTS;
  if (contactCount > 0)
    prefs.getBytes("contacts", contacts, (size_t)contactCount * sizeof(Contact));
  myPhase = (float)(random(0, 628)) / 100.0f;  // fase iniziale casuale

  const char* typeName = (BADGE_TYPE == BADGE_CUORE) ? "CUORE"
                       : (BADGE_TYPE == BADGE_SOLE)  ? "SOLE" : "FUNGO";
  Serial.printf("Badge %s type=%d NUM_LEDS=%d hue=%d mood=%d\n",
    typeName, BADGE_TYPE, NUM_LEDS, myHue, myMood);

  // NimBLE
  NimBLEDevice::init("BADGE");
  NimBLEDevice::setPower(9);  // +9 dBm = MASSIMO ESP32 (antenna debole dei cuore/sole reali)

  // Advertising
  pAdv = NimBLEDevice::getAdvertising();
  pAdv->setMinInterval(100);  // ×0.625ms = 62.5ms (piu' frequente: piu' chance a distanza)
  pAdv->setMaxInterval(160);  // ×0.625ms = 100ms

  // Scan passivo con DUTY-CYCLE ~30% (radio accesa 30ms ogni 100ms)
  // → ESP32 da ~100mA (scan continuo) a ~50mA, +1h circa di autonomia.
  // I badge avvertono ogni ~250ms: su piu' cicli i pacchetti vengono comunque
  // raccolti, e l'EMA dell'RSSI integra nel tempo → prossimita' affidabile.
  NimBLEScan* scan = NimBLEDevice::getScan();
  // 2° arg = wantDuplicates. DEVE essere true: vogliamo un report ad OGNI
  // advertisement cosi' l'RSSI resta fresco. Con 'false' NimBLE attiva il
  // duplicate-filter (setScanCallbacks chiama setDuplicateFilter(!wantDuplicates))
  // e ogni badge veniva riportato UNA SOLA volta → i vicini invecchiavano e
  // non avveniva mai il pairing. ERA QUESTO IL BUG.
  scan->setScanCallbacks(new ScanCB(), true);    // NimBLE 2.x
  scan->setMaxResults(0);                          // solo callback, niente accumulo in RAM
  scan->setActiveScan(false);
  scan->setInterval(160);   // ×0.625ms = 100ms (periodo)
  scan->setWindow(112);     // ×0.625ms = 70ms → 70% duty: buon compromesso
                            // reattivita'/batteria (~20mA risparmiati vs 90%)
  scan->start(0, false);    // NimBLE 2.x: start(duration, restart)

  publishBeacon();

  // LED boot: tre flash bianchi rapidi
  for (int i = 0; i < 3; i++) {
    fill_solid(leds, NUM_LEDS, CRGB(80, 80, 80));
    FastLED.show(); delay(80);
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show(); delay(80);
  }

  lastLoopMs = millis();
  Serial.println("Ready.");
}

// ═══════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════
void loop() {
  uint32_t now = millis();
  float dt = (float)(now - lastLoopMs) / 1000.0f;
  dt = min(dt, 0.1f);  // clamp: evita salti grandi dopo sleep/blocchi
  lastLoopMs = now;

  // Pulsante
  Btn::update();

  // LDR ogni 200ms
  if (now - lastLDRms >= 200) {
    updateLDR();
    lastLDRms = now;
  }

  // Kuramoto
  updatePhase(dt);

  // State machine
  updateState(dt);

  // LED
  renderLEDs();

  // Re-advertise
  if (now - lastAdvMs >= ADV_INTERVAL_MS) {
    publishBeacon();
    lastAdvMs = now;
  }

  // Salva NVS ogni 30s
  if (now - lastNvsMs >= NVS_SAVE_MS) {
    saveNVS();
    lastNvsMs = now;
  }

  // Debug seriale ogni 3s (rimuovi in produzione)
  static uint32_t lastDbg = 0;
  if (now - lastDbg >= 3000) {
    lastDbg = now;
    static const char* stNames[]  = {"IDLE","SENS","PAIR","TRIBE","FARE"};
    static const char* moodNames[]= {"chill","social","party","rasta"};
    Serial.printf("[%lus] %s LED=%-7s mood=%s beat=%-6s hue=%d phase=%.2f night=%d(ldr=%d) btn=%d nb=%d",
      festivalTime() / 1000,
      stNames[myState], dbgTogether ? "ASSIEME" : "onda", moodNames[myMood],
      dbgTogether ? "COMUNE" : moodNames[myMood],
      myHue, myPhase, (int)isNight, ldrFiltered,
      digitalRead(BUTTON_PIN), nbCount);
    // Diagnostica prossimita': vicino piu' forte e da quanto non lo si sente.
    // Da vicino deve dare rssi alto (es. -45) e age basso (<500ms).
    if (nbCount > 0) {
      int8_t   bRssi = -127;
      uint32_t bAge  = 0;
      for (uint8_t i = 0; i < nbCount; i++)
        if (neighbors[i].rssiFilt > bRssi) {
          bRssi = neighbors[i].rssiFilt;
          bAge  = ageMs(now, neighbors[i].lastSeen);
        }
      Serial.printf("  bestRssi=%d age=%lums", bRssi, (unsigned long)bAge);
    }
    if (partnerId >= 0 && partnerId < nbCount) {
      float mat = min(1.0f,
        (float)neighbors[partnerId].totalContactMs / (float)MATURITY_FULL_MS);
      // fase del partner estrapolata (come nel coupling) + differenza di fase
      uint8_t pnm  = (neighbors[partnerId].mood < 4) ? neighbors[partnerId].mood : 1;
      float   pAge = (float)ageMs(now, neighbors[partnerId].lastSeen) / 1000.0f;
      float   pSpd = dbgTogether ? TOGETHER_SPEED : MOOD_SPEED[pnm];
      float   nbPh = neighbors[partnerId].phase + OMEGA * pSpd * pAge;
      float   dPh  = nbPh - myPhase;
      while (dPh >  PI) dPh -= 2.0f * PI;
      while (dPh < -PI) dPh += 2.0f * PI;
      nbPh = fmodf(nbPh, 2.0f * PI); if (nbPh < 0) nbPh += 2.0f * PI;
      uint32_t sess = neighbors[partnerId].sessionMs;
      Serial.printf("  → %08lx rssi=%d sess=%lus%s mat=%.1f%%  SYNC dPh=%+.2f  FUSIONE %d+%d=%d",
        (unsigned long)neighbors[partnerId].idHash,
        neighbors[partnerId].rssiFilt,
        (unsigned long)(sess / 1000), (sess >= FUSION_GRACE_MS) ? ">fonde" : ">rodaggio",
        mat * 100.0f,
        dPh,
        myHue, neighbors[partnerId].hue, dbgShowHue);
    }
    if (groupMs > 0) {
      float gMat = min(1.0f, (float)groupMs / (float)GROUP_FUSION_MS);
      Serial.printf("  GRUPPO %lus fus=%.0f%%", (unsigned long)(groupMs / 1000), gMat * 100.0f);
    }
    Serial.printf("  met=%u%s", peopleMet, IS_CREATOR ? " [CREATORE]" : "");
    Serial.println();
  }
}
