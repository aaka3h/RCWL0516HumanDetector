/*
  RCWL-0516 Human Detector - Heltec V3 (WiFi Kit 32 V3 / WiFi LoRa 32 V3)

  The RCWL-0516 is a microwave (Doppler radar) motion sensor. Its OUT pin
  goes HIGH (3.3 V) for about 2 s whenever something moves within ~5-7 m,
  and it sees through thin plastic, wood and glass. This sketch reads OUT,
  shows MOTION / Clear plus event stats on the OLED, lights the white LED
  and logs events.

  Screen
    MOTION / Clear        uptime
    Events: total    in last 10 min
    Now: how long the current motion has lasted (or Idle: time since last)
    Last: length of the previous event and how long ago it ended
    Max: longest event    Sum: total time with motion
    Sig: % of the last 32 ms OUT was HIGH (live signal strength)
    Noise: rejected single spikes
    timeline, one column per second (~2 min)

  Wiring (RCWL-0516 -> Heltec V3)
    VIN  -> 5V    (the sensor needs 4-28 V; do NOT use 3V3 here)
    GND  -> GND
    OUT  -> GPIO 7
    3V3  -> not connected (it's an output from the sensor's regulator)
    CDS  -> not connected

  How detection works: OUT is not a clean on/off signal. Every detection
  starts with a burst of chatter, and distant movement gives short, choppy
  pulses (1-900 ms). So instead of waiting for a clean HIGH, a 1 kHz timer
  samples OUT and keeps a count of HIGH samples over the last 32 ms. When
  that count reaches CONFIRM_SAMPLES it's motion: the timer turns the white
  LED on right there (a few ms after the sensor fires) even while the
  screen is redrawing. The LED goes off HOLD_MS after activity stops.
  Lower CONFIRM_SAMPLES = more sensitive (farther), higher = fewer false
  alarms. The sensor's own power-up pulse is ignored.

  PRG button (GPIO0)
    tap  = reset all stats
    hold = blank / wake the screen

  Tips
    - Keep the sensor's component side facing the area to watch and at
      least a few cm away from the ESP32 antenna and USB cable.
    - Metal behind the sensor reflects the radar and causes false triggers.
    - WiFi/Bluetooth are switched off: their bursts on the supply rail are
      the #1 cause of RCWL-0516 false alarms.

  Serial (115200): "motion:<0|1> signal:<0-100>" lines for Tools > Serial
  Plotter, plus a text line on every detection.

  Board:     Heltec WiFi LoRa 32(V3) (or WiFi Kit 32(V3)), esp32 core 3.x
  Libraries: Adafruit SSD1306, Adafruit GFX
*/

#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <soc/gpio_reg.h>

// ---------- Heltec V3 pins ----------
const uint8_t PIN_OLED_SDA = 17;
const uint8_t PIN_OLED_SCL = 18;
const uint8_t PIN_OLED_RST = 21;
const uint8_t PIN_VEXT     = 36;  // drive LOW to power the OLED
const uint8_t PIN_BUTTON   = 0;   // PRG button, active LOW
const uint8_t PIN_LED      = 35;  // white onboard LED, lights on motion
const uint8_t OLED_ADDR    = 0x3C;

// ---------- Sensor ----------
const uint8_t PIN_RADAR    = 7;   // RCWL-0516 OUT

// ---------- Tuning ----------
const uint32_t WARMUP_MS      = 5000;  // sensor settles after power-up; ignore it meanwhile
const int      CONFIRM_SAMPLES = 4;    // HIGH samples (1 ms each) in the last 32 ms to count as motion
const uint32_t HOLD_MS        = 1000;  // MOTION stays on this long after OUT goes LOW
const uint32_t LONG_PRESS_MS  = 600;
const uint32_t DEBOUNCE_MS    = 30;
const uint32_t REDRAW_MS      = 100;
const uint32_t SERIAL_MS      = 200;
const uint32_t HIST_STEP_MS   = 1000;  // one graph column per second (~2 min on screen)

const uint32_t RATE_WINDOW_MS = 600000;  // "last 10 min" event count

const int SCREEN_W  = 128;
const int SCREEN_H  = 64;
const int GRAPH_TOP = 56;
const int GRAPH_H   = 8;
const int MAX_EVENT_TIMES = 64;

Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, PIN_OLED_RST);

// Owned by the 1 kHz sampler (timer interrupt)
const int WINDOW = 32;               // samples in the sliding window (bits of `window`)
volatile bool armed = false;         // true once warmed up and OUT has been LOW
volatile bool isrMotion = false;
volatile uint32_t isrDetectedAt = 0; // millis() when motion started
volatile uint32_t isrActiveAt = 0;   // millis() of the last confirmed activity
volatile uint32_t isrEvents = 0;
volatile uint32_t isrNoise = 0;
volatile uint8_t highCount = 0;      // HIGH samples in the window
uint32_t window = 0;
bool candidate = false;
hw_timer_t *sampler = nullptr;

// Detection (main loop copy)
bool rawHigh = false;
uint32_t handledEvents = 0;
uint32_t glitches = 0, glitchBase = 0;
bool motion = false;
uint32_t lastActiveAt = 0;
uint32_t detectedAt = 0;
uint32_t detections = 0;
bool everDetected = false;
uint32_t lastEventMs = 0;     // length of the previous event
uint32_t longestEventMs = 0;
uint32_t totalActiveMs = 0;   // finished events only

// Start times of recent events, for the "last 10 min" count
uint32_t eventTimes[MAX_EVENT_TIMES];
int eventPos = 0, eventCount = 0;

// History graph: 1 = detected at any time during that second
uint8_t history[SCREEN_W];
int histPos = 0, histCount = 0;
bool histActive = false;
uint32_t histStepAt = 0;

bool screenOn = true;

// ---------- Button ----------

enum Press { PRESS_NONE, PRESS_SHORT, PRESS_LONG };

Press readButton() {
  static bool down = false, longSent = false;
  static uint32_t downAt = 0, changeAt = 0;
  bool pressed = digitalRead(PIN_BUTTON) == LOW;
  uint32_t now = millis();
  if (pressed != down && now - changeAt > DEBOUNCE_MS) {
    changeAt = now;
    down = pressed;
    if (down) {
      downAt = now;
      longSent = false;
    } else if (!longSent) {
      return PRESS_SHORT;
    }
  }
  if (down && !longSent && now - downAt >= LONG_PRESS_MS) {
    longSent = true;
    return PRESS_LONG;
  }
  return PRESS_NONE;
}

// ---------- Helpers ----------

// Short duration: "4.1s", "42s", "2m03s", "1h02m"
void formatDur(char *out, size_t len, uint32_t ms) {
  uint32_t s = ms / 1000;
  if (ms < 10000)    snprintf(out, len, "%.1fs", ms / 1000.0f);
  else if (s < 60)   snprintf(out, len, "%lus", (unsigned long)s);
  else if (s < 3600) snprintf(out, len, "%lum%02lus", (unsigned long)(s / 60), (unsigned long)(s % 60));
  else               snprintf(out, len, "%luh%02lum", (unsigned long)(s / 3600), (unsigned long)(s / 60 % 60));
}

int eventsInWindow(uint32_t now) {
  int n = 0;
  for (int i = 0; i < eventCount; i++) {
    if (now - eventTimes[i] <= RATE_WINDOW_MS) n++;
  }
  return n;
}

void printCentered(const char *text, int y, int size) {
  display.setTextSize(size);
  int w = strlen(text) * 6 * size;
  display.setCursor((SCREEN_W - w) / 2, y);
  display.print(text);
}

// ---------- Drawing ----------

void drawWarmup(uint32_t now) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  printCentered("RCWL-0516", 4, 1);
  if (now < WARMUP_MS) {
    printCentered("Warming up", 24, 1);
    char buf[8];
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)((WARMUP_MS - now + 999) / 1000));
    printCentered(buf, 38, 2);
  } else {
    printCentered("Waiting for sensor", 24, 1);
    printCentered("to settle...", 36, 1);
  }
  display.display();
}

void drawMonitor(uint32_t now) {
  char a[12], b[12];
  display.clearDisplay();
  display.setTextSize(1);

  // Status bar: inverted while motion is detected, uptime on the right
  formatDur(a, sizeof(a), now);
  if (motion) {
    display.fillRect(0, 0, SCREEN_W, 10, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
  } else {
    display.setTextColor(SSD1306_WHITE);
    display.drawFastHLine(0, 9, SCREEN_W, SSD1306_WHITE);
  }
  display.setCursor(1, 1);
  display.print(motion ? "MOTION" : "Clear");
  display.setCursor(SCREEN_W - 6 * (strlen(a) + 3) - 1, 1);
  display.printf("up %s", a);
  display.setTextColor(SSD1306_WHITE);

  // Event counts
  display.setCursor(0, 12);
  display.printf("Events:%lu", (unsigned long)detections);
  display.setCursor(78, 12);
  display.printf("10m:%d", eventsInWindow(now));

  // Current state
  display.setCursor(0, 21);
  if (motion) {
    formatDur(a, sizeof(a), now - detectedAt);
    display.printf("Now: moving %s", a);
  } else if (everDetected) {
    formatDur(a, sizeof(a), now - lastActiveAt);
    display.printf("Idle: %s", a);
  } else {
    display.print("Idle: no events yet");
  }

  // Previous event
  display.setCursor(0, 30);
  if (lastEventMs > 0) {
    formatDur(a, sizeof(a), lastEventMs);
    if (motion) {
      display.printf("Last: %s long", a);
    } else {
      formatDur(b, sizeof(b), now - lastActiveAt);
      display.printf("Last:%s, %s ago", a, b);
    }
  } else {
    display.print("Last: -");
  }

  // Longest and total
  formatDur(a, sizeof(a), longestEventMs);
  formatDur(b, sizeof(b), totalActiveMs + (motion ? now - detectedAt : 0));
  display.setCursor(0, 39);
  display.printf("Max:%s", a);
  display.setCursor(66, 39);
  display.printf("Sum:%s", b);

  // Raw sensor
  display.setCursor(0, 48);
  display.printf("Sig:%d%%", highCount * 100 / WINDOW);
  display.setCursor(66, 48);
  display.printf("Noise:%lu", (unsigned long)glitches);

  // Timeline: newest on the right, one column per second
  display.drawFastHLine(0, GRAPH_TOP + GRAPH_H - 1, SCREEN_W, SSD1306_WHITE);
  for (int i = 0; i < histCount; i++) {
    int idx = (histPos - histCount + i + SCREEN_W) % SCREEN_W;
    if (history[idx]) {
      int x = SCREEN_W - histCount + i;
      display.drawFastVLine(x, GRAPH_TOP, GRAPH_H - 1, SSD1306_WHITE);
    }
  }

  display.display();
}

// ---------- Sensor sampler (1 kHz timer interrupt) ----------

// GPIO35 is in the second GPIO bank; write the register directly so the
// LED switches from inside the interrupt without going through the driver.
inline void IRAM_ATTR ledOn()  { REG_WRITE(GPIO_OUT1_W1TS_REG, 1UL << (PIN_LED - 32)); }
inline void IRAM_ATTR ledOff() { REG_WRITE(GPIO_OUT1_W1TC_REG, 1UL << (PIN_LED - 32)); }

void IRAM_ATTR onSample() {
  uint32_t bit = (REG_READ(GPIO_IN_REG) >> PIN_RADAR) & 1;
  uint32_t out = window >> (WINDOW - 1);
  window = (window << 1) | bit;
  highCount += bit - out;
  if (!armed) return;

  uint32_t now = millis();
  if (highCount >= CONFIRM_SAMPLES) {
    isrActiveAt = now;
    candidate = false;
    if (!isrMotion) {
      isrMotion = true;
      isrDetectedAt = now;
      isrEvents++;
      ledOn();
    }
  } else if (isrMotion) {
    if (now - isrActiveAt >= HOLD_MS) {
      isrMotion = false;
      ledOff();
    }
  } else if (highCount > 0) {
    candidate = true;
  } else if (candidate) {
    candidate = false;  // a few HIGH samples that never added up: noise
    isrNoise++;
  }
}

// ---------- Setup / loop ----------

void setup() {
  Serial.begin(115200);

  // Radio off: less supply noise for the radar
  WiFi.mode(WIFI_OFF);
  btStop();

  pinMode(PIN_RADAR, INPUT_PULLDOWN);  // pull-down keeps a disconnected pin reading LOW
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  pinMode(PIN_VEXT, OUTPUT);
  digitalWrite(PIN_VEXT, LOW);
  delay(50);
  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED init failed");
  }
  display.clearDisplay();
  display.display();

  sampler = timerBegin(1000000);           // 1 MHz tick
  timerAttachInterrupt(sampler, &onSample);
  timerAlarm(sampler, 1000, true, 0);      // every 1 ms

  Serial.println("RCWL-0516 motion detector: warming up");
}

void loop() {
  uint32_t now = millis();
  bool warm = now >= WARMUP_MS;

  // ----- Sensor -----
  bool wasMotion = motion;
  rawHigh = digitalRead(PIN_RADAR) == HIGH;

  // Arm only after warm-up AND the sensor's own power-up pulse has ended
  if (!armed && warm && !rawHigh) {
    armed = true;
    Serial.println("Armed");
  }

  if (armed) {
    noInterrupts();
    bool isrM = isrMotion;
    uint32_t events = isrEvents, startAt = isrDetectedAt, activeAt = isrActiveAt, noise = isrNoise;
    interrupts();
    glitches = noise - glitchBase;

    if (events != handledEvents) {
      // Started (maybe also ended already, if the loop was busy)
      handledEvents = events;
      motion = true;
      everDetected = true;
      detectedAt = startAt;
      detections++;
      eventTimes[eventPos] = detectedAt;
      eventPos = (eventPos + 1) % MAX_EVENT_TIMES;
      if (eventCount < MAX_EVENT_TIMES) eventCount++;
      Serial.printf("[%lu.%03lu s] MOTION (#%lu)\n", (unsigned long)(detectedAt / 1000),
                    (unsigned long)(detectedAt % 1000), (unsigned long)detections);
    }
    lastActiveAt = activeAt;
    if (motion) histActive = true;

    if (motion && !isrM) {
      motion = false;
      // Event length = first to last sensor activity (the hold time isn't counted)
      lastEventMs = lastActiveAt - detectedAt;
      if (lastEventMs > longestEventMs) longestEventMs = lastEventMs;
      totalActiveMs += lastEventMs;
      Serial.printf("[%lu s] Clear (lasted %.1f s)\n", (unsigned long)(now / 1000), lastEventMs / 1000.0f);
    }

    if (now - histStepAt >= HIST_STEP_MS) {
      histStepAt = now;
      history[histPos] = histActive || motion;
      histPos = (histPos + 1) % SCREEN_W;
      if (histCount < SCREEN_W) histCount++;
      histActive = false;
    }
  }

  // ----- Button -----
  Press press = readButton();
  if (press == PRESS_SHORT) {
    detections = motion ? 1 : 0;
    everDetected = motion;
    glitchBase += glitches;
    glitches = 0;
    lastEventMs = longestEventMs = totalActiveMs = 0;
    eventCount = eventPos = 0;
    if (motion) {
      eventTimes[eventPos++] = detectedAt;
      eventCount = 1;
    }
    histCount = 0;
    Serial.println("Stats reset");
  } else if (press == PRESS_LONG) {
    screenOn = !screenOn;
    display.ssd1306_command(screenOn ? SSD1306_DISPLAYON : SSD1306_DISPLAYOFF);
  }

  // ----- Output -----
  static uint32_t lastRedraw = 0, lastSerial = 0;
  if (screenOn && (now - lastRedraw >= REDRAW_MS || motion != wasMotion)) {
    lastRedraw = now;
    if (armed) drawMonitor(now);
    else      drawWarmup(now);
  }
  if (armed && now - lastSerial >= SERIAL_MS) {
    lastSerial = now;
    Serial.printf("motion:%d signal:%d\n", motion ? 1 : 0, highCount * 100 / WINDOW);
  }

  delay(1);
}
