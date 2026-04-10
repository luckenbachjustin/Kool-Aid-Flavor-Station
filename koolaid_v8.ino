/*
  6 strips × (8 bars × 24 px) = 6 × 192 = 1152 LEDs total
  24 sensors: one per section (6 strips × 4 sections)
  Each strip's 4 sections map to bar pairs: (0,1), (2,3), (4,5), (6,7)
  Progress per section: 0..24. Unfilled LEDs are OFF by default.

  PER-BAR COLOR CORRECTION:
  Each section has two independent gradients — one for barA, one for barB.
  Use this to correct for color variation between the two physical LED strips
  that make up each bar pair (e.g. from different manufacturing batches).
  barA = the first bar in the pair (even bar index)
  barB = the second bar in the pair (odd bar index)
*/

#include <Adafruit_NeoPixel.h>

// -------------------------- USER CONFIG --------------------------
#define NUM_STRIPS        6
#define NUM_BARS          8
#define PIXELS_PER_BAR    24
#define LEDS_PER_STRIP    (NUM_BARS * PIXELS_PER_BAR)

#define NEO_TYPE          (NEO_RGB + NEO_KHZ800)

uint8_t STRIP_PINS[NUM_STRIPS] = {2, 3, 4, 5, 6, 7};

uint8_t COIN_PINS[24] = {
  22,23,24,25,   // strip 0 sections 0..3
  26,27,28,29,   // strip 1
  30,31,32,33,   // strip 2
  34,35,36,37,   // strip 3
  38,39,40,41,   // strip 4
  42,43,44,45    // strip 5
};

const uint16_t SENSOR_POLL_MS    = 1;
const uint16_t FRAME_MS          = 33;
const uint8_t  ACTIVE_LEVEL      = LOW;
const uint16_t DEBOUNCE_MS       = 3;
const uint16_t MIN_ACTIVE_MS     = 4;
const uint16_t QUIET_BEFORE_MS   = 10;
const uint16_t MIN_INACTIVE_MS   = 10;
const uint16_t REFRACTORY_MS     = 120;
const uint16_t SENSOR_STARTUP_MS = 3000;

static uint32_t lastStableMs[24];
static uint32_t activeStartMs[24];
static uint32_t lastCountMs[24];

#define NUM_PIRS 3
uint8_t PIR_PINS[NUM_PIRS] = {8, 9, 10};
const uint8_t PIR_ACTIVE_LEVEL = HIGH;

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
struct RGB { uint8_t r, g, b; };

// Each section now has TWO gradients: one for barA, one for barB.
// If your two bars look identical, just set gradA == gradB (the default below).
// To correct a batch mismatch, tweak gradB's c0/c1 independently.
struct SectionGrad {
  RGB gradA_c0, gradA_c1;   // bar A (even bar index: 0, 2, 4, 6)
  RGB gradB_c0, gradB_c1;   // bar B (odd bar index:  1, 3, 5, 7)
};

static inline RGB lerpRGB(const RGB& a, const RGB& b, float t) {
  return {
    (uint8_t)(a.r + (b.r - a.r) * t + 0.5f),
    (uint8_t)(a.g + (b.g - a.g) * t + 0.5f),
    (uint8_t)(a.b + (b.b - a.b) * t + 0.5f)
  };
}

// sectionGrad[strip 0..5][section 0..3]
// For each section: first two RGB pairs = bar A gradient, last two = bar B gradient.
// Start with gradA == gradB; adjust gradB when you spot a batch mismatch.
SectionGrad sectionGrad[NUM_STRIPS][4] = {

  { // ---- Strip 0 ----
    // section 0 – strawberry
    { {255,10,10},{255,10,10},  {255,10,10},{255,10,10} },
    // section 1 – cherry
    { {255,0,0},{255,0,0},      {255,0,0},{255,0,0} },
    // section 2 – Tropical Punch
    { {255,0,0},{255,10,0},     {255,0,0},{255,10,0} },
    // section 3 – peach mango
    { {255,5,0},{255,25,0},     {255,5,0},{255,25,0} },
  },

  { // ---- Strip 1 ----
    // section 0 – orange
    { {255,10,0},{255,10,0},    {255,10,0},{255,10,0} },
    // section 1 – tangerine
    { {255,5,0},{255,5,0},      {255,5,0},{255,5,0} },
    // section 2 – root beer
    { {255,5,0},{255,5,0},      {255,5,0},{255,5,0} },
    // section 3 – pina pineapple
    { {255,25,0},{255,80,0},    {255,25,0},{255,80,0} },
  },

  { // ---- Strip 2 ----
    // section 0 – sunshine punch
    { {255,25,0},{255,80,0},    {255,25,0},{255,80,0} },
    // section 1 – lemonade
    { {255,80,0},{255,80,0},    {255,80,0},{255,80,0} },
    // section 2 – lemon-lime
    { {255,120,0},{255,120,0},  {255,120,0},{255,120,0} },
    // section 3 – strawberry kiwi
    { {0,100,0},{255,0,0},      {0,100,0},{255,0,0} },
  },

  { // ---- Strip 3 ----
    // section 0 – jamaica
    { {0,100,0},{0,100,0},      {0,100,0},{0,100,0} },
    // section 1 – green apple
    { {0,100,10},{0,100,10},    {0,100,10},{0,100,10} },
    // section 2 – sharkleberry fin
    { {0,100,100},{255,0,50},   {0,100,100},{255,0,50} },
    // section 3 – great bluedini
    { {0,100,100},{0,100,100},  {0,100,100},{0,100,100} },
  },

  { // ---- Strip 4 ----
    // section 0 – blue raspberry lemonade
    { {0,0,100},{255,80,0},     {0,0,100},{255,80,0} },
    // section 1 – blast off blue moon berry
    { {0,0,100},{0,0,100},      {0,0,100},{0,0,100} },
    // section 2 – ghoul aid blackberry
    { {0,0,100},{0,0,100},      {0,0,100},{0,0,100} },
    // section 3 – magic twists switchin secret
    { {0,0,100},{255,0,50},     {0,0,100},{255,0,50} },
  },

  { // ---- Strip 5 ----
    // section 0 – grape
    { {0,0,50},{255,0,30},      {0,0,50},{255,0,30} },
    // section 1 – purplesaurus rex
    { {255,0,50},{255,0,50},    {255,0,50},{255,0,50} },
    // section 2 – fruit punch wildberry tea
    { {255,0,80},{255,0,10},    {255,0,80},{255,0,10} },
    // section 3 – raspberry
    { {255,0,40},{255,0,40},    {255,0,40},{255,0,40} },
  },
};

bool sectionLeftToRight[NUM_STRIPS][4];

// ---------------------- LED INDEX HELPERS ------------------------
static inline int barFromSection(int section, bool secondInPair) {
  return section * 2 + (secondInPair ? 1 : 0);
}
static inline int ledIndex(int bar, int pixel) {
  return bar * PIXELS_PER_BAR + pixel;
}

// ---------------------- RENDER -----------------------------------
// Each bar in the pair uses its own gradient for color, but they share
// the same progress value so they fill in lockstep.
void renderSection(int si, int section, int value) {
  if (value < 0) value = 0;
  if (value > PIXELS_PER_BAR) value = PIXELS_PER_BAR;

  const SectionGrad& sg = sectionGrad[si][section];
  Adafruit_NeoPixel& strip = strips[si];

  int  barA = barFromSection(section, false);
  int  barB = barFromSection(section, true);
  bool ltr  = sectionLeftToRight[si][section];

  for (int i = 0; i < PIXELS_PER_BAR; ++i) {
    int pix = ltr ? i : (PIXELS_PER_BAR - 1 - i);
    float t = (PIXELS_PER_BAR == 1) ? 0.0f
                                    : (float)pix / (float)(PIXELS_PER_BAR - 1);

    if (i < value) {
      RGB ca = lerpRGB(sg.gradA_c0, sg.gradA_c1, t);
      RGB cb = lerpRGB(sg.gradB_c0, sg.gradB_c1, t);
      strip.setPixelColor(ledIndex(barA, i), strip.Color(ca.r, ca.g, ca.b));
      strip.setPixelColor(ledIndex(barB, i), strip.Color(cb.r, cb.g, cb.b));
    } else {
      strip.setPixelColor(ledIndex(barA, i), 0);
      strip.setPixelColor(ledIndex(barB, i), 0);
    }
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
static volatile uint8_t brokeLatch[24];

static uint32_t refractoryUntil[24];
static uint32_t lastHighStartMs[24];

void sensorsBegin() {
  uint32_t now = millis();
  for (int i = 0; i < 24; ++i) {
    pinMode(COIN_PINS[i], INPUT);
    uint8_t r = digitalRead(COIN_PINS[i]);

    rawState[i]        = r;
    stableState[i]     = r;
    lastStable[i]      = r;
    lastChangeMs[i]    = now;
    lastStableMs[i]    = now;
    activeStartMs[i]   = 0;
    lastCountMs[i]     = 0;
    refractoryUntil[i] = now + SENSOR_STARTUP_MS;
    lastHighStartMs[i] = (r != ACTIVE_LEVEL) ? now : 0;
  }
}

inline void pollSensors() {
  static uint32_t lastPoll = 0;
  uint32_t now = millis();
  if (now - lastPoll < SENSOR_POLL_MS) return;
  lastPoll = now;

  for (int i = 0; i < 24; ++i) {
    uint8_t r = digitalRead(COIN_PINS[i]);

    if (r != rawState[i]) {
      rawState[i] = r;
      lastChangeMs[i] = now;
      continue;
    }
    if (now - lastChangeMs[i] < DEBOUNCE_MS) continue;

    if (stableState[i] != r) {
      uint8_t  prev         = stableState[i];
      uint32_t prevStableAt = lastStableMs[i];

      stableState[i]  = r;
      lastStableMs[i] = now;

      if (r == ACTIVE_LEVEL && prev != ACTIVE_LEVEL) {
        bool quietOK = (now - prevStableAt >= QUIET_BEFORE_MS);
        bool refrOK  = (now >= refractoryUntil[i]);
        bool highOK  = (lastHighStartMs[i] == 0) || (now - lastHighStartMs[i] >= MIN_INACTIVE_MS);
        activeStartMs[i] = (quietOK && refrOK && highOK) ? now : 0;
      } else if (r != ACTIVE_LEVEL && prev == ACTIVE_LEVEL) {
        lastHighStartMs[i] = now;
        if (activeStartMs[i]) {
          uint32_t dur = now - activeStartMs[i];
          if (dur >= MIN_ACTIVE_MS && now >= refractoryUntil[i]) {
            counts[i] += 1;
            if (counts[i] > PIXELS_PER_BAR) counts[i] = 0;
            lastCountMs[i]     = now;
            refractoryUntil[i] = now + REFRACTORY_MS;
          }
          activeStartMs[i] = 0;
        }
      }
    }
  }
}

// ---------------------- STAGGERED REFRESH ------------------------
static uint32_t lastFrameMs  = 0;
static int      nextStrip     = 0;
static uint32_t lastAnyShowMs = 0;
const  uint32_t KEEPALIVE_MS  = 600;

inline void staggeredRefresh() {
  uint32_t now = millis();

  if (now - lastAnyShowMs >= KEEPALIVE_MS) dirtyStrip[nextStrip] = 1;
  if (now - lastFrameMs < FRAME_MS) return;
  lastFrameMs = now;

  for (int attempts = 0; attempts < NUM_STRIPS; ++attempts) {
    int si = nextStrip;
    nextStrip = (nextStrip + 1) % NUM_STRIPS;

    if (dirtyStrip[si]) {
      uint8_t mask = sectionDirty[si] ? sectionDirty[si] : 0x0F;
      renderStrip(si, mask);
      sectionDirty[si] = 0;
      strips[si].show();
      dirtyStrip[si]  = 0;
      lastAnyShowMs   = now;
      break;
    }
  }
}

// --------------------- WAKE WIPE ANIMATION ----------------------
inline void fillBar(int si, int bar, uint32_t color) {
  for (int i = 0; i < PIXELS_PER_BAR; ++i)
    strips[si].setPixelColor(ledIndex(bar, i), color);
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
  for (int i = 0; i < NUM_PIRS; ++i)
    if (digitalRead(PIR_PINS[i]) == PIR_ACTIVE_LEVEL) return true;
  return false;
}

inline void turnOffAllLights() {
  for (int si = 0; si < NUM_STRIPS; ++si) {
    for (int p = 0; p < LEDS_PER_STRIP; ++p) strips[si].setPixelColor(p, 0);
    strips[si].show();
  }
}

// --------------------------- SETUP -------------------------------
void setup() {
  Serial.begin(115200);

  for (int si = 0; si < NUM_STRIPS; ++si) {
    strips[si].begin();
    strips[si].setBrightness(255);
    strips[si].show();
    for (int s = 0; s < 4; ++s) sectionLeftToRight[si][s] = true;
  }

  for (int i = 0; i < 24; ++i)  { counts[i] = 0; lastCounts[i] = -1; }
  for (int si = 0; si < NUM_STRIPS; ++si) { dirtyStrip[si] = 1; sectionDirty[si] = 0x0F; }

  sensorsBegin();

  for (int si = 0; si < NUM_STRIPS; ++si) {
    for (int s = 0; s < 4; ++s) progressVals[si][s] = counts[4*si + s];
    renderStrip(si, 0x0F);
    strips[si].show();
  }
  lastAnyShowMs = millis();
  lastFrameMs   = 0;

  for (int i = 0; i < NUM_PIRS; ++i) pinMode(PIR_PINS[i], INPUT);
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
