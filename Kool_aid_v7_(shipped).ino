/*
  6 strips × (8 bars × 24 px) = 6 × 192 = 1152 LEDs total
  24 sensors: one per section (6 strips × 4 sections)
  Each strip’s 4 sections map to bar pairs: (0,1), (2,3), (4,5), (6,7)
  Progress per section: 0..24. Unfilled LEDs are OFF by default.
*/

#include <Adafruit_NeoPixel.h>

// -------------------------- USER CONFIG --------------------------
#define NUM_STRIPS        6
#define NUM_BARS          8
#define PIXELS_PER_BAR    24
#define LEDS_PER_STRIP    (NUM_BARS * PIXELS_PER_BAR)

// NeoPixel type
#define NEO_TYPE          (NEO_RGB + NEO_KHZ800)

// Strip pins
uint8_t STRIP_PINS[NUM_STRIPS] = {2, 3, 4, 5, 6, 7};

// Sensor pins
uint8_t COIN_PINS[24] = {
  22,23,24,25,   // strip 0 sections 0..3
  26,27,28,29,   // strip 1
  30,31,32,33,   // strip 2
  34,35,36,37,   // strip 3
  38,39,40,41,   // strip 4
  42,43,44,45    // strip 5
};

// --- Balanced, reliable settings ---
const uint16_t SENSOR_POLL_MS    = 1;    // poll sensors every 1 ms
const uint16_t FRAME_MS          = 33;   // ~30 Hz global LED cadence

const uint8_t  ACTIVE_LEVEL      = LOW;  // sensors pull LOW when broken
const uint16_t DEBOUNCE_MS       = 3;    // raw->stable debounce
const uint16_t MIN_ACTIVE_MS     = 4;    // LOW must last at least this long
const uint16_t QUIET_BEFORE_MS   = 10;   // stable time before new event

// Extra robustness
const uint16_t MIN_INACTIVE_MS   = 10;   // require >=10ms HIGH before next LOW
const uint16_t REFRACTORY_MS     = 120;  // lockout after a valid count
const uint16_t SENSOR_STARTUP_MS = 3000; // ignore activity first 3s after boot

static uint32_t lastStableMs[24];
static uint32_t activeStartMs[24];
static uint32_t lastCountMs[24];

// ---- PIR motion settings ----
#define NUM_PIRS 3
uint8_t PIR_PINS[NUM_PIRS] = {8, 9, 10};
const uint8_t PIR_ACTIVE_LEVEL = HIGH;

// Inactivity timeout (15 minutes)
const uint32_t INACTIVITY_MS = 15UL * 60UL * 1000UL;

static volatile uint32_t lastMotionMs = 0;
static bool lightsOff = false;

// ------------------------- LED OBJECTS ---------------------------
Adafruit_NeoPixel strips[NUM_STRIPS] = {
  Adafruit_NeoPixel(LEDS_PER_STRIP, STRIP_PINS[0], NEO_TYPE),
  Adafruit_NeoPixel(LEDS_PER_STRIP, STRIP_PINS[1], NEO_TYPE),
  Adafruit_NeoPixel(LEDS_PER_STRIP, STRIP_PINS[2], NEO_TYPE),
  Adafruit_NeoPixel(LEDS_PER_STRIP, STRIP_PINS[3], NEO_TYPE),
  Adafruit_NeoPixel(LEDS_PER_STRIP, STRIP_PINS[4], NEO_TYPE),
  Adafruit_NeoPixel(LEDS_PER_STRIP, STRIP_PINS[5], NEO_TYPE),
};

// ---------------------- GRADIENT / RENDER ------------------------
struct RGB { uint8_t r,g,b; };
struct Gradient { RGB c0, c1; };

static inline RGB lerpRGB(const RGB& a, const RGB& b, float t) {
  RGB o;
  o.r = (uint8_t)(a.r + (b.r - a.r) * t + 0.5f);
  o.g = (uint8_t)(a.g + (b.g - a.g) * t + 0.5f);
  o.b = (uint8_t)(a.b + (b.b - a.b) * t + 0.5f);
  return o;
}

Gradient sectionGrad[NUM_STRIPS][4];
bool     sectionLeftToRight[NUM_STRIPS][4];

static inline int barFromSection(int section, bool secondInPair) {
  return section * 2 + (secondInPair ? 1 : 0);
}
static inline int ledIndex(int bar, int pixel) {
  return bar * PIXELS_PER_BAR + pixel;
}

void renderSection(int si, int section, int value) {
  if (value < 0) value = 0;
  if (value > PIXELS_PER_BAR) value = PIXELS_PER_BAR;

  const Gradient& grad = sectionGrad[si][section];
  Adafruit_NeoPixel& strip = strips[si];

  RGB buf[PIXELS_PER_BAR];
  for (int i = 0; i < PIXELS_PER_BAR; ++i) {
    if (i < value) {
      float t = (PIXELS_PER_BAR == 1) ? 0.0f : (float)i / (float)(PIXELS_PER_BAR - 1);
      buf[i] = lerpRGB(grad.c0, grad.c1, t);
    } else {
      buf[i] = {0,0,0};
    }
  }

  int barA = barFromSection(section, false);
  int barB = barFromSection(section, true);
  bool ltr = sectionLeftToRight[si][section];

  for (int i = 0; i < PIXELS_PER_BAR; ++i) {
    int pix = ltr ? i : (PIXELS_PER_BAR - 1 - i);
    uint32_t c = strip.Color(buf[pix].r, buf[pix].g, buf[pix].b);
    strip.setPixelColor(ledIndex(barA, i), c);
    strip.setPixelColor(ledIndex(barB, i), c);
  }
}

void renderStrip(int si, const uint8_t sectionMask) {
  for (int s = 0; s < 4; ++s) {
    if (sectionMask & (1u << s)) {
      extern int progressVals[NUM_STRIPS][4];
      renderSection(si, s, progressVals[si][s]);
    }
  }
}

// ---------------------- PROGRESS / DIRTY -------------------------
int progressVals[NUM_STRIPS][4];
int counts[24];
static int lastCounts[24];
static uint8_t dirtyStrip[NUM_STRIPS];
static uint8_t sectionDirty[NUM_STRIPS];

inline void mapCountsAndMarkDirty() {
  for (int si = 0; si < NUM_STRIPS; ++si) {
    for (int s = 0; s < 4; ++s) {
      int i = 4*si + s;
      int v = counts[i];
      if (v != lastCounts[i]) {
        lastCounts[i] = v;
        progressVals[si][s] = v;
        dirtyStrip[si] = 1;
        sectionDirty[si] |= (1u << s);
      }
    }
  }
}

// -------------------------- SENSORS ------------------------------
static uint8_t rawState[24], stableState[24], lastStable[24];
static uint32_t lastChangeMs[24];
static volatile uint8_t brokeLatch[24];  // legacy (unused)

// NEW: extra state for robustness
static uint32_t refractoryUntil[24];   // when the channel can count again
static uint32_t lastHighStartMs[24];   // when line last went HIGH (unbroken)

// External pull-ups: use INPUT (no internal pull-up)
void sensorsBegin() {
  uint32_t now = millis();
  for (int i = 0; i < 24; ++i) {
    pinMode(COIN_PINS[i], INPUT);      // external pull-up in hardware
    uint8_t r = digitalRead(COIN_PINS[i]);

    rawState[i]       = r;
    stableState[i]    = r;
    lastStable[i]     = r;
    lastChangeMs[i]   = now;
    lastStableMs[i]   = now;
    activeStartMs[i]  = 0;
    lastCountMs[i]    = 0;

    refractoryUntil[i] = now + SENSOR_STARTUP_MS;          // startup ignore window
    lastHighStartMs[i] = (r != ACTIVE_LEVEL) ? now : 0;    // track HIGH period
  }
}

// Throttled polling + robust edge handling
inline void pollSensors() {
  static uint32_t lastPoll = 0;
  uint32_t now = millis();
  if (now - lastPoll < SENSOR_POLL_MS) return;   // throttle reads
  lastPoll = now;

  for (int i = 0; i < 24; ++i) {
    uint8_t r = digitalRead(COIN_PINS[i]);

    // Debounce raw -> stable
    if (r != rawState[i]) {
      rawState[i] = r;
      lastChangeMs[i] = now;
      continue;
    }
    if (now - lastChangeMs[i] < DEBOUNCE_MS) continue;

    // Stable edge?
    if (stableState[i] != r) {
      uint8_t  prev         = stableState[i];
      uint32_t prevStableAt = lastStableMs[i];

      stableState[i] = r;
      lastStableMs[i] = now;

      // Transition to ACTIVE (broken = LOW)
      if (r == ACTIVE_LEVEL && prev != ACTIVE_LEVEL) {
        bool quietOK = (now - prevStableAt >= QUIET_BEFORE_MS);
        bool refrOK  = (now >= refractoryUntil[i]);
        bool highOK  = (lastHighStartMs[i] == 0) || (now - lastHighStartMs[i] >= MIN_INACTIVE_MS);
        if (quietOK && refrOK && highOK) {
          activeStartMs[i] = now;     // candidate break begins
        } else {
          activeStartMs[i] = 0;       // reject noisy edge
        }
      }

      // Transition to INACTIVE (unbroken = HIGH): evaluate duration
      else if (r != ACTIVE_LEVEL && prev == ACTIVE_LEVEL) {
        lastHighStartMs[i] = now;     // start of HIGH window for next break

        if (activeStartMs[i]) {
          uint32_t dur = now - activeStartMs[i];
          if (dur >= MIN_ACTIVE_MS && now >= refractoryUntil[i]) {
            // Accept the coin
            counts[i] += 1;
            if (counts[i] > PIXELS_PER_BAR) counts[i] = 0;  // wrap 24->0
            lastCountMs[i]     = now;
            refractoryUntil[i] = now + REFRACTORY_MS;       // brief lockout
          }
          activeStartMs[i] = 0;
        }
      }
    }
  }
}

// ---------------------- STAGGERED REFRESH ------------------------
static uint32_t lastFrameMs = 0;
static int nextStrip = 0;
static uint32_t lastAnyShowMs = 0;
const uint32_t KEEPALIVE_MS = 600;

inline void staggeredRefresh() {
  uint32_t now = millis();

  if (now - lastAnyShowMs >= KEEPALIVE_MS) {
    dirtyStrip[nextStrip] = 1;
  }
  if (now - lastFrameMs < FRAME_MS) return;
  lastFrameMs = now;

  for (int attempts = 0; attempts < NUM_STRIPS; ++attempts) {
    int si = nextStrip;
    nextStrip = (nextStrip + 1) % NUM_STRIPS;

    if (dirtyStrip[si]) {
      uint8_t mask = sectionDirty[si] ? sectionDirty[si] : 0x0F; // if unknown, render all 4
      renderStrip(si, mask);
      sectionDirty[si] = 0;

      strips[si].show();
      dirtyStrip[si] = 0;
      lastAnyShowMs = now;
      break;
    }
  }
}

// --------------------- WAKE WIPE ANIMATION ----------------------
// (current white wipe; say the word if you want gradient wipe again)
inline void fillBar(int si, int bar, uint32_t color) {
  for (int i = 0; i < PIXELS_PER_BAR; ++i) {
    strips[si].setPixelColor(ledIndex(bar, i), color);
  }
}

void wakeWipe() {
  uint32_t c = strips[0].Color(12, 12, 12);
  for (int b = 0; b < NUM_BARS; ++b) {
    for (int si = 0; si < NUM_STRIPS; ++si) {
      fillBar(si, b, c);
      strips[si].show();
    }
    delay(60);
  }
  for (int si = 0; si < NUM_STRIPS; ++si) {
    for (int p = 0; p < LEDS_PER_STRIP; ++p) strips[si].setPixelColor(p, 0);
    strips[si].show();
  }
  for (int si = 0; si < NUM_STRIPS; ++si) {
    sectionDirty[si] = 0x0F;
    dirtyStrip[si]   = 1;
  }
}

// PIR helpers
inline bool anyMotionNow() {
  for (int i = 0; i < NUM_PIRS; ++i) {
    if (digitalRead(PIR_PINS[i]) == PIR_ACTIVE_LEVEL) return true;
  }
  return false;
}

inline void turnOffAllLights() {
  for (int si = 0; si < NUM_STRIPS; ++si) {
    for (int p = 0; p < LEDS_PER_STRIP; ++p) {
      strips[si].setPixelColor(p, 0);
    }
    strips[si].show();
  }
}

inline void wakeDisplay() {
  for (int si = 0; si < NUM_STRIPS; ++si) {
    sectionDirty[si] = 0x0F;
    dirtyStrip[si]   = 1;
  }
}

// --------------------------- SETUP -------------------------------
void setup() {
  Serial.begin(115200);

  // Init LEDs
  for (int si = 0; si < NUM_STRIPS; ++si) {
    strips[si].begin();
    strips[si].setBrightness(255);
    strips[si].show();
    for (int s = 0; s < 4; ++s) sectionLeftToRight[si][s] = true;
  }
  sectionGrad[0][0] = { {255,  10,  10}, {255,  10, 10} }; // strawberry
  sectionGrad[0][1] = { {255,   0,   0}, {255,   0,  0} }; // cherry
  sectionGrad[0][2] = { {255,   0,   0}, {255,  10,   0} }; // Tropical Punch
  sectionGrad[0][3] = { {255,  5,   0}, {255,   25,   0} }; // peach mango
  sectionGrad[1][0] = { {255,   10,   0}, {255, 10,   0} }; // orange
  sectionGrad[1][1] = { {255, 5,   0}, {255, 5, 0} }; // tangerine
  sectionGrad[1][2] = { {255, 5,   0}, {255, 5, 0} }; // root beer
  sectionGrad[1][3] = { {255, 25,   0}, {255,   80, 0} }; // pina pineapple
  sectionGrad[2][0] = { {255, 25,   0}, {255,   80, 0} }; // sunshine punch
  sectionGrad[2][1] = { {255, 80,   0}, {255, 80, 00} }; // lemonade
  sectionGrad[2][2] = { {255, 120,   0}, {255, 120, 0} }; // lemon-lime
  sectionGrad[2][3] = { {0, 100, 0} , {255,  0, 0} }; // strawberry kiwi
  sectionGrad[3][0] = { {0, 100, 0}, {0, 100, 0} }; // jamaica
  sectionGrad[3][1] = { {  0, 100,   10}, {  0, 100, 10} }; // green apple
  sectionGrad[3][2] = { {  0,  100, 100}, {255,   0, 50} }; // sharkleberry fin
  sectionGrad[3][3] = { {0, 100, 100}, {0,   100, 100} }; // great bluedini
  sectionGrad[4][0] = { {0,   0,   100}, {255,80, 0} }; // blue raspberry lemonade
  sectionGrad[4][1] = { {  0, 0,   100}, {  0, 0, 100} }; // blast off blue moon berry
  sectionGrad[4][2] = { {  0,   0, 10}, {0,   0, 100} }; // ghoul aid blackberry
  sectionGrad[4][3] = { {0, 0, 100}, {255,   0, 50} }; // magic twists switchin secret
  sectionGrad[5][0] = { {0,   0, 50}, {255, 0, 30} }; // grape
  sectionGrad[5][1] = { {255, 0,   50}, { 255, 0, 50} }; // purplesaurus rex
  sectionGrad[5][2] = { {255,   0, 80}, {255,   0, 10} }; // fruit ts wildberry tea
  sectionGrad[5][3] = { {255, 0, 40}, {255,   0, 40} }; // raspberry

  // Init counters / dirties
  for (int i = 0; i < 24; ++i) { counts[i] = 0; lastCounts[i] = -1; } // -1 forces initial paint
  for (int si = 0; si < NUM_STRIPS; ++si) { dirtyStrip[si] = 1; sectionDirty[si] = 0x0F; }

  sensorsBegin();

  // Initial full render
  for (int si = 0; si < NUM_STRIPS; ++si) {
    for (int s = 0; s < 4; ++s) progressVals[si][s] = counts[4*si + s];
    renderStrip(si, 0x0F);
    strips[si].show();
  }
  lastAnyShowMs = millis();
  lastFrameMs = 0;

  // PIR init
  for (int i = 0; i < NUM_PIRS; ++i) {
    pinMode(PIR_PINS[i], INPUT);
  }
  lastMotionMs = millis();
}

// ---------------------------- LOOP -------------------------------
void loop() {
  uint32_t now = millis();
  if (anyMotionNow()) {
    lastMotionMs = now;
    if (lightsOff) {
      lightsOff = false;
      wakeWipe();
    }
  } else {
    if (!lightsOff && (now - lastMotionMs >= INACTIVITY_MS)) {
      lightsOff = true;
      turnOffAllLights();
    }
  }

  pollSensors();
  mapCountsAndMarkDirty();
  if (!lightsOff) staggeredRefresh();
}
