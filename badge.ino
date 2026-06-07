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
#define NUM_LEDS      7
#define BUTTON_PIN    14
#define LDR_PIN       34
#define BADGE_TYPE    0       // 0 = cuore, 1 = sole
#define IS_CREATOR    1       // 1 SOLO sul TUO badge (il creatore). Gli altri 0.
#define CREATOR_HUE   38      // tonalita' ORO della "firma" del creatore (FastLED hue)

// ═══════════════════════════════════════════════════════
//  BLE
// ═══════════════════════════════════════════════════════
static const uint16_t COMPANY_ID  = 0xFF01;
static const uint8_t  PROTO_VER   = 1;

// ═══════════════════════════════════════════════════════
//  RSSI / DISTANZA
// ═══════════════════════════════════════════════════════
static const int8_t  RSSI_IN    = -60;  // dBm: entra PAIRED  (~1m)
static const int8_t  RSSI_OUT   = -68;  // dBm: esce PAIRED (isteresi ~2m)
static const int8_t  RSSI_SENSE = -75;  // dBm: entra SENSING (~3-4m)
static const float   EMA_ALPHA  = 0.3f;

// ═══════════════════════════════════════════════════════
//  KURAMOTO
// ═══════════════════════════════════════════════════════
static const float   OMEGA          = 2.0f * PI / 1.2f; // periodo battito 1.2s
static const float   K_COUPLING     = 4.0f;
static const float   MOOD_SPEED[3]  = { 0.8f, 1.0f, 1.3f };
static const uint8_t MOOD_BRIGHT[3] = {  80,   170,  220 };  // valore FastLED max
static const float   LED_SPREAD     = 1.0f;  // onda sui LED: 1.0 = un'onda intera sul giro

// ═══════════════════════════════════════════════════════
//  TIMING
// ═══════════════════════════════════════════════════════
static const uint32_t NB_TIMEOUT_MS     =   4000;
static const uint32_t TRIBE_TIME_MS     =  30000;
static const uint32_t HB_INTERVAL_MS   = 600000; // heartbeat ogni 10 min
static const uint32_t HB_DUR_MS        =   5000;
static const uint32_t ADV_INTERVAL_MS  =    250;
static const uint32_t NVS_SAVE_MS      =  30000;
static const uint32_t MATURITY_FULL_MS =  30000; // contatto totale per maturità piena
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

Neighbor neighbors[MAX_NB];
uint8_t  nbCount      = 0;

int8_t   partnerId    = -1;
uint32_t tribeEnterMs = 0;
uint32_t farewellStart= 0;

bool     recognitionPending = false;
uint32_t recognitionStart   = 0;

bool     creatorSparkPending = false;   // scintilla dorata "hai incontrato il creatore"
uint32_t creatorSparkStart   = 0;
uint16_t creatorMetCount     = 0;       // quante persone nuove ha conosciuto il creatore

int      ldrFiltered  = 2048;
bool     isNight      = false;

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
    // salva NVS del vecchio
    char key[9];
    snprintf(key, sizeof(key), "%08lx", (unsigned long)neighbors[slot].idHash);
    prefs.putUInt(key, neighbors[slot].totalContactMs);
  }

  // carica totalContactMs del nuovo (se già incontrato in precedenza)
  char key[9];
  snprintf(key, sizeof(key), "%08lx", (unsigned long)hash);
  uint32_t saved = prefs.getUInt(key, 0);

  neighbors[slot] = {
    .idHash       = hash,
    .rssiFilt     = -90,
    .hue          = 0,
    .mood         = 1,
    .phase        = 0.0f,
    .lastSeen     = millis(),
    .firstPaired  = 0,
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

    // consensus T0: adotta il valore più alto
    uint32_t theirFT = p->tConsensus;
    uint32_t myFT    = festivalTime();
    if (theirFT > myFT + 500) {
      t0Offset += (theirFT - myFT);
    }

    if (xSemaphoreTake(nbMutex, 0) == pdTRUE) {
      int8_t slot = findOrCreate(hash);
      if (slot >= 0) {
        float raw  = (float)dev->getRSSI();
        float prev = (float)neighbors[slot].rssiFilt;
        neighbors[slot].rssiFilt = (int8_t)(EMA_ALPHA * raw + (1.0f - EMA_ALPHA) * prev);
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
  uint64_t mac = ESP.getEfuseMac();
  p.id[0]      = (uint8_t)(mac >>  0);
  p.id[1]      = (uint8_t)(mac >>  8);
  p.id[2]      = (uint8_t)(mac >> 16);
  p.hue        = myHue;
  p.mood       = myMood;
  p.phase      = (uint8_t)(myPhase / (2.0f * PI) * 255.0f);
  p.tConsensus = festivalTime();

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

  if (xSemaphoreTake(nbMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    for (uint8_t i = 0; i < nbCount; i++) {
      if (now - neighbors[i].lastSeen > NB_TIMEOUT_MS)  continue;
      if (neighbors[i].rssiFilt < RSSI_OUT)             continue;
      pull += sinf(neighbors[i].phase - myPhase);
      n++;
    }
    xSemaphoreGive(nbMutex);
  }

  if (n > 0) pull /= n;
  myPhase += (OMEGA * MOOD_SPEED[myMood] + K_COUPLING * pull) * dt;
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
    if (now - neighbors[i].lastSeen > NB_TIMEOUT_MS) continue;
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
      bool firstTime = (neighbors[bestSlot].firstPaired == 0);
      if (firstTime)
        neighbors[bestSlot].firstPaired = now;
      // ── SCINTILLA DEL CREATORE ──
      // se incontro il creatore (e non lo sono io), oppure se IO sono il creatore
      // e conosco qualcuno di nuovo → scintilla dorata su entrambi i badge.
      if ((neighbors[bestSlot].isCreator && !IS_CREATOR) ||
          (IS_CREATOR && firstTime)) {
        creatorSparkPending = true;
        creatorSparkStart   = now;
        if (IS_CREATOR && firstTime) creatorMetCount++;
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

  // Tribe: se ≥2 vicini a portata PAIRED per TRIBE_TIME_MS
  if (myState == PAIRED && pairedGrade >= 2) {
    if (tribeEnterMs == 0) tribeEnterMs = now;
    if (now - tribeEnterMs >= TRIBE_TIME_MS) myState = TRIBE;
  } else {
    tribeEnterMs = 0;
  }

  // Accumulo contatto con il partner corrente
  if ((myState == PAIRED || myState == TRIBE) && partnerId >= 0) {
    neighbors[partnerId].totalContactMs += (uint32_t)(dt * 1000.0f);
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

  // Brightness massima per mood + notte
  uint8_t maxV = MOOD_BRIGHT[myMood];
  if (isNight) maxV /= 2;

  // Hue da mostrare + maturity
  uint8_t showHue  = myHue;
  float   maturity = 0.0f;

  if (xSemaphoreTake(nbMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    if (partnerId >= 0 && partnerId < nbCount) {
      maturity = min(1.0f,
        (float)neighbors[partnerId].totalContactMs / (float)MATURITY_FULL_MS);

      if (myState == PAIRED) {
        showHue = blendHue(myHue, neighbors[partnerId].hue, 0.5f * maturity);
      }
      else if (myState == TRIBE) {
        uint8_t tribeHues[MAX_NB + 1];
        uint8_t tn = 0;
        tribeHues[tn++] = myHue;
        for (uint8_t i = 0; i < nbCount; i++) {
          if (now - neighbors[i].lastSeen > NB_TIMEOUT_MS) continue;
          if (neighbors[i].rssiFilt > RSSI_OUT)
            tribeHues[tn++] = neighbors[i].hue;
        }
        showHue = circularMeanHue(tribeHues, tn);
      }
    }
    xSemaphoreGive(nbMutex);
  }

  // Pulse base: sin(phase) → 0..1
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
      // Onda allegra a bassa luminosità: ogni LED sfasato
      uint8_t pv = maxV / 2;
      for (int i = 0; i < NUM_LEDS; i++) {
        float ledPh = myPhase + (float)i * LED_SPREAD * (2.0f * PI / (float)NUM_LEDS);
        uint8_t v   = (uint8_t)((sinf(ledPh) * 0.5f + 0.5f) * pv);
        leds[i] = CHSV(showHue, 255, v);
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
      // Onda sfasata, piu' luminosa con la maturita' del legame
      uint8_t pv = (uint8_t)(maxV * (0.7f + 0.3f * maturity));
      for (int i = 0; i < NUM_LEDS; i++) {
        float ledPh = myPhase + (float)i * LED_SPREAD * (2.0f * PI / (float)NUM_LEDS);
        uint8_t v   = (uint8_t)((sinf(ledPh) * 0.5f + 0.5f) * pv);
        leds[i] = CHSV(showHue, 255, v);
      }
      break;
    }

    case TRIBE:
      // Onda rotante: ogni LED sfasato di 2π/NUM_LEDS
      for (int i = 0; i < NUM_LEDS; i++) {
        float ledPh = myPhase + (float)i * (2.0f * PI / (float)NUM_LEDS);
        uint8_t v   = (uint8_t)((sinf(ledPh) * 0.5f + 0.5f) * maxV);
        leds[i] = CHSV(showHue, 255, v);
      }
      break;

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

  // ── FIRMA DEL CREATORE: aura dorata che ruota lenta (solo il tuo badge) ──
  if (IS_CREATOR) {
    const uint32_t CREATOR_SPIN_MS = 400;   // ms per passo (alza = piu' lento)
    const uint8_t  CREATOR_TRAIL   = 2;     // lunghezza scia (LED)
    const uint8_t  CREATOR_PEAK    = 76;    // intensita' di picco (era 152, -50%)
    uint8_t head = (uint8_t)((now / CREATOR_SPIN_MS) % NUM_LEDS);   // cometa dorata lenta
    for (int i = 0; i < NUM_LEDS; i++) {
      uint8_t d = (uint8_t)((i - head + NUM_LEDS) % NUM_LEDS);
      uint8_t glow = (d < CREATOR_TRAIL)
                   ? (uint8_t)(CREATOR_PEAK * (CREATOR_TRAIL - d) / CREATOR_TRAIL) : 0;
      if (glow) {
        CRGB g = CHSV(CREATOR_HUE, 230, glow);
        leds[i].r = min(255, (int)leds[i].r + g.r);
        leds[i].g = min(255, (int)leds[i].g + g.g);
        leds[i].b = min(255, (int)leds[i].b + g.b);
      }
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
    myMood = (myMood + 1) % 3;
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
      if (held >= 5000 && !superFired) { onSuperLong(); superFired = true; }
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
  for (uint8_t i = 0; i < nbCount; i++) {
    if (neighbors[i].totalContactMs == 0) continue;
    char key[9];
    snprintf(key, sizeof(key), "%08lx", (unsigned long)neighbors[i].idHash);
    prefs.putUInt(key, neighbors[i].totalContactMs);
  }
  xSemaphoreGive(nbMutex);
  Serial.println("[NVS] saved");
}

// ═══════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(100);
  bootMs = millis();

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
  myPhase = (float)(random(0, 628)) / 100.0f;  // fase iniziale casuale

  Serial.printf("Badge %s type=%d hue=%d mood=%d\n",
    BADGE_TYPE == 0 ? "CUORE" : "SOLE",
    BADGE_TYPE, myHue, myMood);

  // NimBLE
  NimBLEDevice::init("BADGE");
  NimBLEDevice::setPower(0);  // NimBLE 2.x: dBm diretti (0 dBm), calibrato ~1m PAIRED

  // Advertising
  pAdv = NimBLEDevice::getAdvertising();
  pAdv->setMinInterval(160);  // ×0.625ms = 100ms
  pAdv->setMaxInterval(400);  // ×0.625ms = 250ms

  // Scan passivo con DUTY-CYCLE ~30% (radio accesa 30ms ogni 100ms)
  // → ESP32 da ~100mA (scan continuo) a ~50mA, +1h circa di autonomia.
  // I badge avvertono ogni ~250ms: su piu' cicli i pacchetti vengono comunque
  // raccolti, e l'EMA dell'RSSI integra nel tempo → prossimita' affidabile.
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(new ScanCB(), false);   // NimBLE 2.x (era setAdvertisedDeviceCallbacks)
  scan->setActiveScan(false);
  scan->setInterval(160);   // ×0.625ms = 100ms (periodo)
  scan->setWindow(48);      // ×0.625ms = 30ms  (finestra accesa) → 30% duty
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
    static const char* moodNames[]= {"chill","social","party"};
    Serial.printf("[%lus] %s mood=%s hue=%d phase=%.2f night=%d(ldr=%d) btn=%d nb=%d",
      festivalTime() / 1000,
      stNames[myState], moodNames[myMood],
      myHue, myPhase, (int)isNight, ldrFiltered,
      digitalRead(BUTTON_PIN), nbCount);
    if (partnerId >= 0 && partnerId < nbCount) {
      float mat = min(1.0f,
        (float)neighbors[partnerId].totalContactMs / (float)MATURITY_FULL_MS);
      Serial.printf("  → %08lx rssi=%d contact=%lus mat=%.0f%%",
        (unsigned long)neighbors[partnerId].idHash,
        neighbors[partnerId].rssiFilt,
        neighbors[partnerId].totalContactMs / 1000,
        mat * 100.0f);
    }
    if (IS_CREATOR) Serial.printf("  [CREATORE] conosciute=%u", creatorMetCount);
    Serial.println();
  }
}
static const uint8_t  PROTO_VER   = 1;

// ═══════════════════════════════════════════════════════
//  RSSI / DISTANZA
// ═══════════════════════════════════════════════════════
static const int8_t  RSSI_IN    = -60;  // dBm: entra PAIRED  (~1m)
static const int8_t  RSSI_OUT   = -68;  // dBm: esce PAIRED (isteresi ~2m)
static const int8_t  RSSI_SENSE = -75;  // dBm: entra SENSING (~3-4m)
static const float   EMA_ALPHA  = 0.3f;

// ═══════════════════════════════════════════════════════
//  KURAMOTO
// ═══════════════════════════════════════════════════════
static const float   OMEGA          = 2.0f * PI / 1.2f; // periodo battito 1.2s
static const float   K_COUPLING     = 4.0f;
static const float   MOOD_SPEED[3]  = { 0.8f, 1.0f, 1.3f };
static const uint8_t MOOD_BRIGHT[3] = {  80,   170,  220 };  // valore FastLED max

// ═══════════════════════════════════════════════════════
//  TIMING
// ═══════════════════════════════════════════════════════
static const uint32_t NB_TIMEOUT_MS     =   4000;
static const uint32_t TRIBE_TIME_MS     =  30000;
static const uint32_t HB_INTERVAL_MS   = 600000; // heartbeat ogni 10 min
static const uint32_t HB_DUR_MS        =   5000;
static const uint32_t ADV_INTERVAL_MS  =    250;
static const uint32_t NVS_SAVE_MS      =  30000;
static const uint32_t MATURITY_FULL_MS =  30000; // contatto totale per maturità piena
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

Neighbor neighbors[MAX_NB];
uint8_t  nbCount      = 0;

int8_t   partnerId    = -1;
uint32_t tribeEnterMs = 0;
uint32_t farewellStart= 0;

bool     recognitionPending = false;
uint32_t recognitionStart   = 0;

bool     creatorSparkPending = false;   // scintilla dorata "hai incontrato il creatore"
uint32_t creatorSparkStart   = 0;
uint16_t creatorMetCount     = 0;       // quante persone nuove ha conosciuto il creatore

int      ldrFiltered  = 2048;
bool     isNight      = false;

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
    // salva NVS del vecchio
    char key[9];
    snprintf(key, sizeof(key), "%08lx", (unsigned long)neighbors[slot].idHash);
    prefs.putUInt(key, neighbors[slot].totalContactMs);
  }

  // carica totalContactMs del nuovo (se già incontrato in precedenza)
  char key[9];
  snprintf(key, sizeof(key), "%08lx", (unsigned long)hash);
  uint32_t saved = prefs.getUInt(key, 0);

  neighbors[slot] = {
    .idHash       = hash,
    .rssiFilt     = -90,
    .hue          = 0,
    .mood         = 1,
    .phase        = 0.0f,
    .lastSeen     = millis(),
    .firstPaired  = 0,
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

    // consensus T0: adotta il valore più alto
    uint32_t theirFT = p->tConsensus;
    uint32_t myFT    = festivalTime();
    if (theirFT > myFT + 500) {
      t0Offset += (theirFT - myFT);
    }

    if (xSemaphoreTake(nbMutex, 0) == pdTRUE) {
      int8_t slot = findOrCreate(hash);
      if (slot >= 0) {
        float raw  = (float)dev->getRSSI();
        float prev = (float)neighbors[slot].rssiFilt;
        neighbors[slot].rssiFilt = (int8_t)(EMA_ALPHA * raw + (1.0f - EMA_ALPHA) * prev);
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
  uint64_t mac = ESP.getEfuseMac();
  p.id[0]      = (uint8_t)(mac >>  0);
  p.id[1]      = (uint8_t)(mac >>  8);
  p.id[2]      = (uint8_t)(mac >> 16);
  p.hue        = myHue;
  p.mood       = myMood;
  p.phase      = (uint8_t)(myPhase / (2.0f * PI) * 255.0f);
  p.tConsensus = festivalTime();

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

  if (xSemaphoreTake(nbMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    for (uint8_t i = 0; i < nbCount; i++) {
      if (now - neighbors[i].lastSeen > NB_TIMEOUT_MS)  continue;
      if (neighbors[i].rssiFilt < RSSI_OUT)             continue;
      pull += sinf(neighbors[i].phase - myPhase);
      n++;
    }
    xSemaphoreGive(nbMutex);
  }

  if (n > 0) pull /= n;
  myPhase += (OMEGA * MOOD_SPEED[myMood] + K_COUPLING * pull) * dt;
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
    if (now - neighbors[i].lastSeen > NB_TIMEOUT_MS) continue;
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
      bool firstTime = (neighbors[bestSlot].firstPaired == 0);
      if (firstTime)
        neighbors[bestSlot].firstPaired = now;
      // ── SCINTILLA DEL CREATORE ──
      // se incontro il creatore (e non lo sono io), oppure se IO sono il creatore
      // e conosco qualcuno di nuovo → scintilla dorata su entrambi i badge.
      if ((neighbors[bestSlot].isCreator && !IS_CREATOR) ||
          (IS_CREATOR && firstTime)) {
        creatorSparkPending = true;
        creatorSparkStart   = now;
        if (IS_CREATOR && firstTime) creatorMetCount++;
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

  // Tribe: se ≥2 vicini a portata PAIRED per TRIBE_TIME_MS
  if (myState == PAIRED && pairedGrade >= 2) {
    if (tribeEnterMs == 0) tribeEnterMs = now;
    if (now - tribeEnterMs >= TRIBE_TIME_MS) myState = TRIBE;
  } else {
    tribeEnterMs = 0;
  }

  // Accumulo contatto con il partner corrente
  if ((myState == PAIRED || myState == TRIBE) && partnerId >= 0) {
    neighbors[partnerId].totalContactMs += (uint32_t)(dt * 1000.0f);
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

  // Brightness massima per mood + notte
  uint8_t maxV = MOOD_BRIGHT[myMood];
  if (isNight) maxV /= 2;

  // Hue da mostrare + maturity
  uint8_t showHue  = myHue;
  float   maturity = 0.0f;

  if (xSemaphoreTake(nbMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    if (partnerId >= 0 && partnerId < nbCount) {
      maturity = min(1.0f,
        (float)neighbors[partnerId].totalContactMs / (float)MATURITY_FULL_MS);

      if (myState == PAIRED) {
        showHue = blendHue(myHue, neighbors[partnerId].hue, 0.5f * maturity);
      }
      else if (myState == TRIBE) {
        uint8_t tribeHues[MAX_NB + 1];
        uint8_t tn = 0;
        tribeHues[tn++] = myHue;
        for (uint8_t i = 0; i < nbCount; i++) {
          if (now - neighbors[i].lastSeen > NB_TIMEOUT_MS) continue;
          if (neighbors[i].rssiFilt > RSSI_OUT)
            tribeHues[tn++] = neighbors[i].hue;
        }
        showHue = circularMeanHue(tribeHues, tn);
      }
    }
    xSemaphoreGive(nbMutex);
  }

  // Pulse base: sin(phase) → 0..1
  float pulseFrac = sinf(myPhase) * 0.5f + 0.5f;
  uint8_t baseV   = (uint8_t)(pulseFrac * maxV);

  // Farewell: fade out
  if (myState == FAREWELL) {
    float prog = min(1.0f, (float)(now - farewellStart) / (float)FAREWELL_DUR_MS);
    baseV = (uint8_t)(baseV * (1.0f - prog));
  }

  // ── Riempi LED per stato ───────────────────────────
  switch (myState) {

    case IDLE:
      // Respiro lento a bassa luminosità
      fill_solid(leds, NUM_LEDS, CHSV(showHue, 255, baseV / 2));
      break;

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

    case PAIRED:
      // Fill uniforme, brighness aumenta con maturity
      baseV = (uint8_t)(baseV * (0.7f + 0.3f * maturity));
      fill_solid(leds, NUM_LEDS, CHSV(showHue, 255, baseV));
      break;

    case TRIBE:
      // Onda rotante: ogni LED sfasato di 2π/NUM_LEDS
      for (int i = 0; i < NUM_LEDS; i++) {
        float ledPh = myPhase + (float)i * (2.0f * PI / (float)NUM_LEDS);
        uint8_t v   = (uint8_t)((sinf(ledPh) * 0.5f + 0.5f) * maxV);
        leds[i] = CHSV(showHue, 255, v);
      }
      break;

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

  // ── FIRMA DEL CREATORE: aura dorata che ruota lenta (solo il tuo badge) ──
  if (IS_CREATOR) {
    uint8_t head = (uint8_t)((now / 60) % NUM_LEDS);   // cometa dorata
    for (int i = 0; i < NUM_LEDS; i++) {
      uint8_t d = (uint8_t)((i - head + NUM_LEDS) % NUM_LEDS);
      uint8_t glow = (d < 4) ? (uint8_t)(38 * (4 - d)) : 0;  // scia di 4 LED
      if (glow) {
        CRGB g = CHSV(CREATOR_HUE, 230, glow);
        leds[i].r = min(255, (int)leds[i].r + g.r);
        leds[i].g = min(255, (int)leds[i].g + g.g);
        leds[i].b = min(255, (int)leds[i].b + g.b);
      }
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
    myMood = (myMood + 1) % 3;
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

    if (pressed && !prev) {
      pressStart  = t;
      longFired   = false;
      superFired  = false;
    }
    if (pressed && prev) {
      uint32_t held = t - pressStart;
      if (held >= 5000 && !superFired) { onSuperLong(); superFired = true; }
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
  for (uint8_t i = 0; i < nbCount; i++) {
    if (neighbors[i].totalContactMs == 0) continue;
    char key[9];
    snprintf(key, sizeof(key), "%08lx", (unsigned long)neighbors[i].idHash);
    prefs.putUInt(key, neighbors[i].totalContactMs);
  }
  xSemaphoreGive(nbMutex);
  Serial.println("[NVS] saved");
}

// ═══════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(100);
  bootMs = millis();

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
  myPhase = (float)(random(0, 628)) / 100.0f;  // fase iniziale casuale

  Serial.printf("Badge %s type=%d hue=%d mood=%d\n",
    BADGE_TYPE == 0 ? "CUORE" : "SOLE",
    BADGE_TYPE, myHue, myMood);

  // NimBLE
  NimBLEDevice::init("BADGE");
  NimBLEDevice::setPower(0);  // NimBLE 2.x: dBm diretti (0 dBm), calibrato ~1m PAIRED

  // Advertising
  pAdv = NimBLEDevice::getAdvertising();
  pAdv->setMinInterval(160);  // ×0.625ms = 100ms
  pAdv->setMaxInterval(400);  // ×0.625ms = 250ms

  // Scan passivo con DUTY-CYCLE ~30% (radio accesa 30ms ogni 100ms)
  // → ESP32 da ~100mA (scan continuo) a ~50mA, +1h circa di autonomia.
  // I badge avvertono ogni ~250ms: su piu' cicli i pacchetti vengono comunque
  // raccolti, e l'EMA dell'RSSI integra nel tempo → prossimita' affidabile.
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(new ScanCB(), false);   // NimBLE 2.x (era setAdvertisedDeviceCallbacks)
  scan->setActiveScan(false);
  scan->setInterval(160);   // ×0.625ms = 100ms (periodo)
  scan->setWindow(48);      // ×0.625ms = 30ms  (finestra accesa) → 30% duty
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
    static const char* moodNames[]= {"chill","social","party"};
    Serial.printf("[%lus] %s mood=%s hue=%d phase=%.2f night=%d nb=%d",
      festivalTime() / 1000,
      stNames[myState], moodNames[myMood],
      myHue, myPhase, (int)isNight, nbCount);
    if (partnerId >= 0 && partnerId < nbCount) {
      float mat = min(1.0f,
        (float)neighbors[partnerId].totalContactMs / (float)MATURITY_FULL_MS);
      Serial.printf("  → %08lx rssi=%d contact=%lus mat=%.0f%%",
        (unsigned long)neighbors[partnerId].idHash,
        neighbors[partnerId].rssiFilt,
        neighbors[partnerId].totalContactMs / 1000,
        mat * 100.0f);
    }
    if (IS_CREATOR) Serial.printf("  [CREATORE] conosciute=%u", creatorMetCount);
    Serial.println();
  }
}
