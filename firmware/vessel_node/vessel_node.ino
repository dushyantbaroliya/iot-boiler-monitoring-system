/*
  Vessel node firmware: one ESP32 per pressure vessel.

  Reads water temperature (DS18B20 in a thermowell) and gauge pressure
  (0-10 bar, 4-20 mA transducer across a 150 ohm sense resistor), sounds a
  local buzzer + LED alarm, and uploads 15 s averages to the vessel's own
  ThingSpeak channel (field 1 = temperature in degC, field 2 = pressure in bar g).

  The same sketch runs on all three boards. Only secrets.h differs
  (VESSEL_ID, channel ID, write API key). Copy secrets.example.h to secrets.h.

  Timing (1 s main tick, nothing blocks):
    - pressure read every 1 s
    - temperature conversion started every 5 s, read 800 ms later
    - alarm checked every tick, so it fires within about 1 s (requirement: 2 s)
    - upload every 15 s (ThingSpeak free-channel limit)
    - Wi-Fi reconnect attempted every 2 s while disconnected; measuring and
      alarming carry on without the network.

  Wiring (see report, Figure 5.3):
    DS18B20 VDD -> 3V3, DQ -> GPIO 4 (4.7k pull-up to 3V3), GND -> GND
    +24 V -> transducer(+); transducer(-) -> 150 ohm -> GND
    sense node -> 1k -> GPIO 34 (100 nF from GPIO-side to GND)
    GPIO 26 -> 1k -> BC547 base; buzzer between +5 V and collector
    GPIO 25 -> 220 ohm -> red LED -> GND

  Libraries: OneWire, DallasTemperature, ThingSpeak (Arduino Library Manager).
  Board: ESP32 Dev Module (Espressif ESP32 core 2.x or 3.x).
*/

#include <WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ThingSpeak.h>
#include "secrets.h"

// ---------------------------------------------------------------- pins
const int PIN_ONEWIRE  = 4;
const int PIN_PRESSURE = 34;   // ADC1 channel: usable while Wi-Fi is on
const int PIN_BUZZER   = 26;
const int PIN_LED      = 25;

// ---------------------------------------------------------------- limits
const float P_ALARM_BAR = 3.5;    // 70 % of the 5 bar PRV setting
const float T_ALARM_C   = 120.0;  // end-of-run limit (DS18B20 max is 125 C)
const float P_CLEAR_BAR = 3.3;    // hysteresis so the alarm does not chatter
const float T_CLEAR_C   = 118.0;

// ---------------------------------------------------------------- pressure calibration
// 4 mA * 150 ohm = 600 mV -> 0 bar, 20 mA * 150 ohm = 3000 mV -> 10 bar.
// Replace these two numbers with the millivolt readings you get at 0 bar and at
// the Bourdon-gauge reference point (two-point calibration, report Section 5.5).
const float CAL_ZERO_MV  = 600.0;   // reading at 0 bar
const float CAL_SPAN_MV  = 3000.0;  // reading at CAL_SPAN_BAR
const float CAL_SPAN_BAR = 10.0;
const float LOOP_FAULT_LOW_MV  = 540.0;   // below 3.6 mA: broken wire or no 24 V
const float LOOP_FAULT_HIGH_MV = 3150.0;  // above 21 mA: shorted or over-range

// ---------------------------------------------------------------- timing (ms)
const unsigned long TICK_MS        = 1000;
const unsigned long TEMP_PERIOD_MS = 5000;
const unsigned long TEMP_CONV_MS   = 800;    // 12-bit conversion takes 750 ms
const unsigned long UPLOAD_MS      = 15000;
const unsigned long WIFI_RETRY_MS  = 2000;

OneWire oneWire(PIN_ONEWIRE);
DallasTemperature ds(&oneWire);
WiFiClient client;

// latest values and their state
float tempC = NAN, pressBar = NAN;
bool tempFault = false, pressFault = false, alarmOn = false;   // set true only by a failed read
bool tempPending = false;

// 15 s accumulators
float tSum = 0, pSum = 0;
int tCount = 0, pCount = 0;

unsigned long lastTick = 0, lastTempReq = 0, lastUpload = 0, lastWifiTry = 0;
unsigned long uploadsOk = 0, uploadsTried = 0;

// ---------------------------------------------------------------- helpers
float readPressure() {
  // average 16 samples to reduce ADC noise; analogReadMilliVolts uses the
  // chip's factory calibration, which the raw analogRead does not.
  uint32_t mv = 0;
  for (int i = 0; i < 16; i++) mv += analogReadMilliVolts(PIN_PRESSURE);
  float v = mv / 16.0;
  pressFault = (v < LOOP_FAULT_LOW_MV) || (v > LOOP_FAULT_HIGH_MV);
  float p = (v - CAL_ZERO_MV) * CAL_SPAN_BAR / (CAL_SPAN_MV - CAL_ZERO_MV);
  return p < 0 ? 0 : p;
}

void serviceWifi(unsigned long now) {
  if (WiFi.status() == WL_CONNECTED) return;
  if (now - lastWifiTry < WIFI_RETRY_MS) return;
  lastWifiTry = now;
  Serial.println("[wifi] not connected, retrying");
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void serviceTemperature(unsigned long now) {
  if (!tempPending && now - lastTempReq >= TEMP_PERIOD_MS) {
    ds.requestTemperatures();          // returns immediately (non-blocking mode)
    lastTempReq = now;
    tempPending = true;
  }
  if (tempPending && now - lastTempReq >= TEMP_CONV_MS) {
    tempPending = false;
    float t = ds.getTempCByIndex(0);
    if (t == DEVICE_DISCONNECTED_C) {   // -127: probe missing or wire broken
      tempFault = true;
    } else {
      tempFault = false;
      tempC = t;
      tSum += t;
      tCount++;
    }
  }
}

void updateAlarm() {
  bool trip = pressFault || tempFault ||
              (!isnan(pressBar) && pressBar > P_ALARM_BAR) ||
              (!isnan(tempC) && tempC > T_ALARM_C);
  bool clear = !pressFault && !tempFault &&
               pressBar < P_CLEAR_BAR && tempC < T_CLEAR_C;
  if (trip && !alarmOn) {
    alarmOn = true;
    Serial.printf("[ALARM] V%d  P=%.2f bar  T=%.1f C  pressFault=%d tempFault=%d\n",
                  VESSEL_ID, pressBar, tempC, pressFault, tempFault);
  } else if (alarmOn && clear) {
    alarmOn = false;
    Serial.printf("[alarm cleared] V%d\n", VESSEL_ID);
  }
  digitalWrite(PIN_BUZZER, alarmOn ? HIGH : LOW);
  digitalWrite(PIN_LED, alarmOn ? HIGH : LOW);
}

void serviceUpload(unsigned long now) {
  if (now - lastUpload < UPLOAD_MS) return;
  lastUpload = now;
  if (tCount == 0 && pCount == 0) return;

  float tAvg = tCount ? tSum / tCount : NAN;
  float pAvg = pCount ? pSum / pCount : NAN;
  tSum = pSum = 0;
  tCount = pCount = 0;

  Serial.printf("V%d  T=%.2f C  P=%.3f bar g  alarm=%d  wifi=%d  uploads=%lu/%lu\n",
                VESSEL_ID, tAvg, pAvg, alarmOn, WiFi.status() == WL_CONNECTED,
                uploadsOk, uploadsTried);

  if (WiFi.status() != WL_CONNECTED) return;   // keep measuring; data for this window is lost
  if (!isnan(tAvg)) ThingSpeak.setField(1, tAvg);
  if (!isnan(pAvg)) ThingSpeak.setField(2, pAvg);
  uploadsTried++;
  int code = ThingSpeak.writeFields(TS_CHANNEL_ID, TS_WRITE_KEY);
  if (code == 200) uploadsOk++;
  else Serial.printf("[thingspeak] write failed, code %d\n", code);
}

// ---------------------------------------------------------------- setup / loop
void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(PIN_LED, LOW);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_PRESSURE, ADC_11db);   // about 0.15-3.1 V usable

  ds.begin();
  ds.setResolution(12);
  ds.setWaitForConversion(false);
  if (ds.getDeviceCount() == 0) Serial.println("[ds18b20] no sensor found on GPIO 4");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);   // connection completes in the background
  ThingSpeak.begin(client);

  Serial.printf("Vessel node %d started (channel %lu)\n", VESSEL_ID, (unsigned long)TS_CHANNEL_ID);
  unsigned long now = millis();
  lastWifiTry = lastUpload = now;
  lastTempReq = now - TEMP_PERIOD_MS;   // first temperature request straight away
}

void loop() {
  unsigned long now = millis();
  serviceTemperature(now);             // polled every pass so the 800 ms read is on time

  if (now - lastTick >= TICK_MS) {
    lastTick = now;
    pressBar = readPressure();
    if (!pressFault) { pSum += pressBar; pCount++; }
    updateAlarm();
    serviceWifi(now);
    serviceUpload(now);
  }
}
