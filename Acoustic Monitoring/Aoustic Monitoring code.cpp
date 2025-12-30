#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OneWire.h>
#include <DallasTemperature.h>

/* ---------------- WIFI ---------------- */
const char* ssid = "NAVEEN_17 6175";
const char* password = "12345678";

/* ----------- FIREBASE ----------------- */
#define FIREBASE_URL "https://sound-monitoring-6a0d1-default-rtdb.firebaseio.com"
#define DEVICE_ID "lift-01"

/* ------------------ PIN SETUP ------------------ */
#define MIC_PIN        34
#define BUZZER_PIN     23
#define ONE_WIRE_BUS    4     // DS18B20
#define OLED_ADDR     0x3C

Adafruit_SSD1306 display(128, 64, &Wire, -1);

/* ----------------- DS18B20 TEMP ---------------- */
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

/* ------------------ FFT CONFIG ------------------ */
#define SAMPLES       512
#define SAMPLE_RATE 10000.0

double vReal[SAMPLES];
double vImag[SAMPLES];

/* ----------------- WAVEFORM BUFFER ------------- */
#define WAVE_SAMPLES 128
int16_t waveBuf[WAVE_SAMPLES];

/* ------------- High-Pass Filter (DC Remove) ---- */
static inline int hpfDC(int x) {
  static int x1 = 0;
  static float y1 = 0;
  const float a = 0.995f;

  float y = (x - x1) + a * y1;
  x1 = x;
  y1 = y;

  return (int)y;
}

/* ------------------ Simple FFT ------------------ */
class SimpleFFT {
public:
  double *vr, *vi;
  uint16_t N;
  double fs;

  SimpleFFT(double *r, double *i, uint16_t n, double fs_) :
    vr(r), vi(i), N(n), fs(fs_) {}

  void compute() {
    uint16_t j = 0;

    for (uint16_t i = 1; i < N - 1; i++) {
      uint16_t bit = N >> 1;
      while (j & bit) { j ^= bit; bit >>= 1; }
      j ^= bit;
      if (i < j) {
        double tr = vr[i]; vr[i] = vr[j]; vr[j] = tr;
        tr = vi[i]; vi[i] = vi[j]; vi[j] = tr;
      }
    }

    for (uint16_t len = 2; len <= N; len <<= 1) {
      double ang = -2 * PI / len;
      double wlen_r = cos(ang);
      double wlen_i = sin(ang);

      for (uint16_t i = 0; i < N; i += len) {
        double w_r = 1;
        double w_i = 0;

        for (uint16_t j = 0; j < len / 2; j++) {
          uint16_t u = i + j;
          uint16_t v = i + j + len / 2;

          double ur = vr[u];
          double ui = vi[u];

          double vr2 = vr[v] * w_r - vi[v] * w_i;
          double vi2 = vr[v] * w_i + vi[v] * w_r;

          vr[u] = ur + vr2;
          vi[u] = ui + vi2;

          vr[v] = ur - vr2;
          vi[v] = ui - vi2;

          double w_r2 = w_r * wlen_r - w_i * wlen_i;
          w_i = w_r * wlen_i + w_i * wlen_r;
          w_r = w_r2;
        }
      }
    }
  }

  void magnitude() {
    for (int i = 0; i < N; i++) {
      vr[i] = sqrt(vr[i] * vr[i] + vi[i] * vi[i]);
      vi[i] = 0;
    }
  }
};

SimpleFFT FFT(vReal, vImag, SAMPLES, SAMPLE_RATE);

/* ----------------- WAVEFORM CAPTURE ------------- */
void captureWaveform() {
  const unsigned long Ts = 1000000UL / 20000UL;
  unsigned long t0 = micros();

  for (int i = 0; i < WAVE_SAMPLES; i++) {
    while ((micros() - t0) < (unsigned long)i * Ts) {}
    int v = hpfDC(analogRead(MIC_PIN));
    waveBuf[i] = v;
  }
}

/* ------------------- DRAW OLED ------------------ */
void drawOLED(float freq, float temp, const char* state) {

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  display.setCursor(0, 0);
  display.print("F:");
  display.print(freq, 0);
  display.print("Hz");

  display.setCursor(70, 0);
  display.print(state);

  display.setCursor(0, 10);
  display.print("T:");
  display.print(temp, 1);
  display.print("C");

  int midY = 42;
  display.drawLine(0, midY, 127, midY, WHITE);

  for (int x = 1; x < WAVE_SAMPLES; x++) {
    int y1 = midY - waveBuf[x - 1] / 40;
    int y2 = midY - waveBuf[x] / 40;

    y1 = constrain(y1, 0, 63);
    y2 = constrain(y2, 0, 63);

    display.drawLine(x - 1, y1, x, y2, WHITE);
  }

  display.display();
}

/* ------------------- FIREBASE UPLOAD ------------------ */
void firebaseUpload(float freq, float temp, const char* state) {

  HTTPClient http;

  String pathCurrent = String(FIREBASE_URL) + "/devices/" + DEVICE_ID + "/current.json";
  String pathHistory = String(FIREBASE_URL) + "/devices/" + DEVICE_ID + "/history.json";

  String json = "{";
  json += "\"timestamp\":" + String(millis()) + ",";
  json += "\"freq_hz\":" + String(freq) + ",";
  json += "\"temp_c\":" + String(temp) + ",";
  json += "\"state\":\"" + String(state) + "\"";
  json += "}";

  // Snapshot overwrite
  http.begin(pathCurrent);
  http.addHeader("Content-Type", "application/json");
  http.PUT(json);
  http.end();

  // History push
  http.begin(pathHistory);
  http.addHeader("Content-Type", "application/json");
  http.POST(json);
  http.end();
}

/* -------------------- SETUP --------------------- */
void setup() {
  Serial.begin(115200);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  sensors.begin();

  Wire.begin(21, 22);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);

  Serial.println("Connecting WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }

  Serial.println("\nWiFi Connected.");
  display.clearDisplay();
}

/* --------------------- LOOP --------------------- */
void loop() {

  sensors.requestTemperatures();
  float temp = sensors.getTempCByIndex(0);

  captureWaveform();

  const unsigned long Ts = 1000000UL / SAMPLE_RATE;
  unsigned long t0 = micros();
  const double TWO_PI_VAL = 2.0 * PI;

  for (int i = 0; i < SAMPLES; i++) {
    while ((micros() - t0) < (unsigned long)i * Ts) {}
    int hp = hpfDC(analogRead(MIC_PIN));

    double w = 0.5 * (1 - cos(TWO_PI_VAL * i / (SAMPLES - 1)));

    vReal[i] = hp * w;
    vImag[i] = 0;
  }

  FFT.compute();
  FFT.magnitude();

  int peakBin = 1;
  double maxMag = 0;

  for (int i = 1; i < SAMPLES / 2; i++) {
    if (vReal[i] > maxMag) {
      maxMag = vReal[i];
      peakBin = i;
    }
  }

  float freq = (peakBin * SAMPLE_RATE) / SAMPLES;

  const char* state = "IDLE";
  bool buzzer = false;

  if (freq > 700 && freq < 950 && temp > 45) {
    state = "BEARING ISSUE";
    buzzer = true;
  }
  else if (freq > 1000 && temp > 47) {
    state = "BRAKING";
    buzzer = true;
  }
  else {
    state = "IDLE";
    buzzer = false;
  }

  digitalWrite(BUZZER_PIN, buzzer ? HIGH : LOW);

  drawOLED(freq, temp, state);

  Serial.printf("F=%.0f Hz | T=%.1f C | %s\n", freq, temp, state);

  firebaseUpload(freq, temp, state);   // ✅ FIREBASE ADDED HERE

  delay(50);
}