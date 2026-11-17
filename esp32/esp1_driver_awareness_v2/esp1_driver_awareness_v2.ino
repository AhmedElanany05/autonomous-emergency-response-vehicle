#include <Arduino.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

const char*    WIFI_SSID   = "YOUR_WIFI_NAME";
const char*    WIFI_PASS   = "YOUR_WIFI_PASSWORD";
const char*    JETSON_IP   = "192.168.1.100";
const uint16_t JETSON_PORT = 5005;

#define ECG_OUTPUT        34
#define LO_PLUS_PIN       33
#define LO_MINUS_PIN      25

#define SIM800_RX         27
#define SIM800_TX         26

#define RESPONSE_BTN_PIN  13
#define CANCEL_BTN_PIN    15
#define STATUS_LED_PIN    14
#define BUZZER_PIN        32

#define OLED_SDA_PIN      21
#define OLED_SCL_PIN      22
#define OLED_I2C_ADDR   0x3C
#define OLED_WIDTH      128
#define OLED_HEIGHT      64

const char phoneNumber[] = "+20XXXXXXXXXX";

#define PEAK_THRESHOLD   2500
#define REFRACTORY_MS     300
#define BPM_WINDOW          8
const uint8_t BPM_MIN_SAMPLES_FOR_DISPLAY = 3;
const uint8_t BPM_MIN_SAMPLES_FOR_TRIGGER = 4;
const uint8_t BPM_MEDIAN_WINDOW          = 5;
const float   BPM_SMOOTH_ALPHA           = 0.30f;
const float   BPM_MIN_VALID              = 35.0f;
const float   BPM_MAX_VALID              = 220.0f;

uint32_t rrIntervals[BPM_WINDOW];
uint8_t  rrIndex      = 0;
bool     rrBufferFull = false;
float    currentBPM   = 0.0f;
uint32_t lastPeakMs   = 0;
bool     leadsOff     = false;
bool     aboveThresh  = false;

const uint8_t ECG_GRAPH_SAMPLES = 96;
uint16_t      ecgGraph[ECG_GRAPH_SAMPLES];
uint8_t       ecgGraphHead      = 0;
bool          ecgGraphWrapped   = false;
unsigned long lastDisplayDrawMs = 0;
const unsigned long DISPLAY_REFRESH_MS = 60;

const unsigned long RESPONSE_TEST_MS    = 5000UL;
const unsigned long BUZZ_TOGGLE_MS      = 400UL;
const unsigned long TRIGGER_COOLDOWN_MS = 20000UL;

bool          responseTestActive   = false;
unsigned long responseTestStart    = 0;
unsigned long lastBuzzToggle       = 0;
bool          buzzState            = false;
bool          bpmConfirmedOnce     = false;
unsigned long triggerCooldownUntil = 0;

const unsigned long BTN_DEBOUNCE_MS = 50;

struct ButtonDebounce {
  uint8_t       pin;
  int           lastRaw;
  int           stable;
  unsigned long lastChangeMs;
};

ButtonDebounce responseBtn = { RESPONSE_BTN_PIN, HIGH, HIGH, 0 };
ButtonDebounce cancelBtn   = { CANCEL_BTN_PIN,   HIGH, HIGH, 0 };

const unsigned long RECONNECT_INTERVAL = 3000;
WiFiClient    client;
unsigned long lastReconnectAttempt = 0;

bool emergencyInProgress = false;

unsigned long lastSecondTick = 0;

String statusLineTop    = "ESP1 booting";
String statusLineBottom = "...";

HardwareSerial  sim800(1);
Adafruit_SH1106G display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

void emit(const String& line) {
  Serial.println(line);
  if (client.connected()) {
    client.print(line);
    client.print('\n');
  }
}

void setStatus(const String& top, const String& bottom) {
  statusLineTop = top;
  statusLineBottom = bottom;
}

void pushECGSample(uint16_t raw) {
  ecgGraph[ecgGraphHead] = raw;
  ecgGraphHead = (ecgGraphHead + 1) % ECG_GRAPH_SAMPLES;
  if (ecgGraphHead == 0) ecgGraphWrapped = true;
}

uint8_t graphCount() {
  return ecgGraphWrapped ? ECG_GRAPH_SAMPLES : ecgGraphHead;
}

uint16_t graphSampleAt(uint8_t index) {
  if (!ecgGraphWrapped) return ecgGraph[index];
  return ecgGraph[(ecgGraphHead + index) % ECG_GRAPH_SAMPLES];
}

uint8_t rrSampleCount() {
  return rrBufferFull ? BPM_WINDOW : rrIndex;
}

bool heartRateReady() {
  return rrSampleCount() >= BPM_MIN_SAMPLES_FOR_DISPLAY && currentBPM >= BPM_MIN_VALID;
}

uint32_t recentRRAt(uint8_t newestOffset) {
  uint8_t count = rrSampleCount();
  if (count == 0 || newestOffset >= count) return 0;

  int index = (int)(rrIndex - 1 - newestOffset) % BPM_WINDOW;
  if (index < 0) index += BPM_WINDOW;
  return rrIntervals[index];
}

uint32_t medianRecentRR() {
  uint8_t count = rrSampleCount();
  uint8_t windowCount = min(count, BPM_MEDIAN_WINDOW);
  uint32_t values[BPM_MEDIAN_WINDOW];

  for (uint8_t i = 0; i < windowCount; i++) values[i] = recentRRAt(i);

  for (uint8_t i = 1; i < windowCount; i++) {
    uint32_t key = values[i];
    int8_t j = i - 1;
    while (j >= 0 && values[j] > key) {
      values[j + 1] = values[j];
      j--;
    }
    values[j + 1] = key;
  }

  if (windowCount == 0) return 0;
  if (windowCount % 2 == 1) return values[windowCount / 2];
  return (values[windowCount / 2] + values[(windowCount / 2) - 1]) / 2;
}

void drawECGGraph(int graphY) {
  const int graphX = 0;
  const int graphW = OLED_WIDTH;
  const int graphH = OLED_HEIGHT - graphY;

  display.drawRect(graphX, graphY, graphW, graphH, SH110X_WHITE);

  uint8_t count = graphCount();
  if (count < 2 || leadsOff) {
    display.setCursor(28, graphY + 14);
    display.print(leadsOff ? "Leads off" : "Waiting ECG");
    return;
  }

  uint16_t minVal = 4095;
  uint16_t maxVal = 0;
  for (uint8_t i = 0; i < count; i++) {
    uint16_t v = graphSampleAt(i);
    if (v < minVal) minVal = v;
    if (v > maxVal) maxVal = v;
  }

  if (maxVal - minVal < 600) {
    uint16_t center = (maxVal + minVal) / 2;
    minVal = (center > 300) ? center - 300 : 0;
    maxVal = (center < 3795) ? center + 300 : 4095;
  }

  int prevX = graphX + 1;
  int prevY = graphY + graphH / 2;
  for (uint8_t i = 0; i < count; i++) {
    uint16_t sample = graphSampleAt(i);
    int x = graphX + 1 + ((long)i * (graphW - 3)) / (count - 1);
    int y = graphY + graphH - 2 - ((long)(sample - minVal) * (graphH - 4)) / (maxVal - minVal);
    y = constrain(y, graphY + 1, graphY + graphH - 2);
    if (i > 0) display.drawLine(prevX, prevY, x, y, SH110X_WHITE);
    prevX = x;
    prevY = y;
  }
}

void drawOLED() {
  unsigned long now = millis();
  if (now - lastDisplayDrawMs < DISPLAY_REFRESH_MS) return;
  lastDisplayDrawMs = now;

  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  bool measuringView = !emergencyInProgress && !responseTestActive && !leadsOff;

  if (measuringView) {
    display.setTextSize(2);
    display.setCursor(0, 0);
    if (heartRateReady()) display.print((int)round(currentBPM));
    else display.print("--");
    display.setTextSize(1);
    display.setCursor(42, 6);
    display.print("BPM");
    drawECGGraph(18);
  } else {
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("BPM:");
    if (heartRateReady()) display.print((int)round(currentBPM));
    else display.print("--");

    if (responseTestActive) {
      int remain = (int)((RESPONSE_TEST_MS - min(RESPONSE_TEST_MS, now - responseTestStart) + 999UL) / 1000UL);
      display.setCursor(78, 0);
      display.print("ACK ");
      display.print(remain);
      display.print("s");
    } else if (now < triggerCooldownUntil) {
      unsigned long remain = ((triggerCooldownUntil - now) / 1000UL) + 1;
      display.setCursor(72, 0);
      display.print("CD ");
      display.print(remain);
      display.print("s");
    }

    display.setCursor(0, 10);
    display.print(statusLineTop);
    if (statusLineBottom.length() > 0) {
      display.setCursor(0, 18);
      display.print(statusLineBottom);
    }

    drawECGGraph(22);
  }

  display.display();
}

void updateIdleStatus() {
  if (leadsOff) setStatus("ECG leads off", "Check electrodes");
  else if (millis() < triggerCooldownUntil) setStatus("Cooldown active", "Monitoring paused");
  else setStatus("Monitoring ECG", "Btn13/15 ready");
}

void wifiConnect() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  setStatus("Connecting WiFi", "Please wait");
  Serial.print("WiFi connecting");

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    Serial.print('.');
    drawOLED();
    delay(500);
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi OK IP=");
    Serial.println(WiFi.localIP());
    setStatus("WiFi connected", WiFi.localIP().toString());
  } else {
    Serial.println("WiFi failed");
    setStatus("WiFi failed", "Retrying...");
  }
}

void ensureJetsonConnection() {
  if (client.connected()) return;

  if (WiFi.status() != WL_CONNECTED) {
    if (millis() - lastReconnectAttempt > RECONNECT_INTERVAL) {
      lastReconnectAttempt = millis();
      WiFi.reconnect();
    }
    return;
  }

  if (millis() - lastReconnectAttempt < RECONNECT_INTERVAL) return;
  lastReconnectAttempt = millis();

  if (client.connect(JETSON_IP, JETSON_PORT, 3000)) {
    client.setNoDelay(true);
    emit("EVT:READY");
  }
}

void sendAT(const char* cmd, unsigned long waitMs = 300) {
  sim800.println(cmd);
  delay(waitMs);
  while (sim800.available()) Serial.write(sim800.read());
}

void initGSM() {
  Serial.println("Init GSM...");
  sendAT("AT");
  sendAT("ATE0");
  sendAT("AT+CMGF=1");
  sendAT("AT+CSCS=\"GSM\"");
  Serial.println("GSM ready.");
}

bool sendSMS(const char* number, const char* message) {
  Serial.print("Sending SMS to ");
  Serial.println(number);
  while (sim800.available()) sim800.read();
  sim800.print("AT+CMGS=\"");
  sim800.print(number);
  sim800.println("\"");

  String prompt = "";
  unsigned long t = millis();
  while (millis() - t < 5000) {
    while (sim800.available()) prompt += (char)sim800.read();
    if (prompt.indexOf('>') != -1) break;
    delay(50);
  }
  if (prompt.indexOf('>') == -1) {
    Serial.println("SMS: no prompt");
    return false;
  }

  sim800.print(message);
  delay(100);
  sim800.write(26);

  String resp = "";
  t = millis();
  while (millis() - t < 15000) {
    while (sim800.available()) resp += (char)sim800.read();
    if (resp.indexOf("+CMGS:") != -1) break;
    delay(100);
  }

  bool ok = resp.indexOf("+CMGS:") != -1;
  Serial.println(ok ? "SMS OK" : "SMS FAIL");
  return ok;
}

bool isCallAnswered() {
  while (sim800.available()) sim800.read();
  sim800.println("AT+CLCC");
  delay(1000);
  String resp = "";
  while (sim800.available()) {
    char c = sim800.read();
    resp += c;
    Serial.write(c);
  }
  return (resp.indexOf("+CLCC:") != -1 && resp.indexOf(",0,0,0,0") != -1);
}

bool waitForAnswer(unsigned long timeoutMs) {
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    if (isCallAnswered()) {
      Serial.println("Call answered!");
      return true;
    }
    delay(500);
  }
  Serial.println("No answer.");
  return false;
}

void playBeeps() {
  sendAT("AT+DTAM=1", 3000);
  sendAT("AT+CLDTMF=5,\"1,1,1,1,1,1,1,1\",100", 12000);
}

void makeCall() {
  Serial.print("Calling ");
  Serial.println(phoneNumber);
  while (sim800.available()) sim800.read();
  sim800.print("ATD");
  sim800.print(phoneNumber);
  sim800.println(";");
  delay(1000);

  digitalWrite(BUZZER_PIN, HIGH);
  delay(300);
  digitalWrite(BUZZER_PIN, LOW);

  emit("EVT:CALL_OK");
  bool answered = waitForAnswer(30000);
  if (answered) {
    emit("EVT:CALL_ANSWERED");
    playBeeps();
    delay(3000);
  } else {
    emit("EVT:CALL_NOANSWER");
  }
  sendAT("ATH");
  Serial.println("Call ended.");
}

void runAlertSequence() {
  emergencyInProgress = true;
  emit("EVT:ALERT_START");
  setStatus("EMERGENCY", "Sending SMS...");

  const char* smsText =
    "THE DRIVER IS FACING A MEDICAL EMERGENCY AT THIS LOCATION: "
    "LATITUDE, LONGITUDE";

  bool smsOk = sendSMS(phoneNumber, smsText);
  emit(smsOk ? "EVT:SMS_OK" : "EVT:SMS_FAIL");
  setStatus("EMERGENCY", smsOk ? "SMS OK. Calling" : "SMS FAIL. Call");
  drawOLED();

  delay(2000);
  makeCall();

  emit("EVT:DONE");
  setStatus("Alert sent", "Stay calm");
  emergencyInProgress = false;
}

void sampleECG() {
  bool nowLeadsOff = digitalRead(LO_PLUS_PIN) || digitalRead(LO_MINUS_PIN);
  if (nowLeadsOff != leadsOff) {
    leadsOff = nowLeadsOff;
    emit(leadsOff ? "EVT:LEADS_OFF" : "EVT:LEADS_OK");
    if (leadsOff) aboveThresh = false;
  }

  if (leadsOff) return;

  int raw = analogRead(ECG_OUTPUT);
  pushECGSample((uint16_t)raw);
  uint32_t now = millis();

  if (!aboveThresh && raw > PEAK_THRESHOLD) {
    aboveThresh = true;
    uint32_t rr = now - lastPeakMs;
    float instantBPM = (rr > 0) ? (60000.0f / rr) : 0.0f;
    if (lastPeakMs != 0 && rr > REFRACTORY_MS
        && instantBPM >= BPM_MIN_VALID && instantBPM <= BPM_MAX_VALID) {
      rrIntervals[rrIndex % BPM_WINDOW] = rr;
      rrIndex++;
      if (rrIndex >= BPM_WINDOW) rrBufferFull = true;

      uint32_t medianRR = medianRecentRR();
      if (medianRR > 0) {
        float filteredBPM = 60000.0f / medianRR;
        if (currentBPM <= 0.0f) currentBPM = filteredBPM;
        else currentBPM += (filteredBPM - currentBPM) * BPM_SMOOTH_ALPHA;
      }
    }
    lastPeakMs = now;
  }

  if (aboveThresh && raw < (PEAK_THRESHOLD - 200)) aboveThresh = false;
}

void startResponseTest() {
  responseTestActive = true;
  responseTestStart = millis();
  lastBuzzToggle = millis();
  buzzState = true;
  digitalWrite(BUZZER_PIN, HIGH);
  digitalWrite(STATUS_LED_PIN, HIGH);
  setStatus("Heart rate found", "Press 13/15 in 5s");
  emit("EVT:RESPONSE_TEST_START");
}

void stopResponseOutputs() {
  buzzState = false;
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(STATUS_LED_PIN, LOW);
}

void cancelResponseTest(const char* sourceLabel) {
  responseTestActive = false;
  bpmConfirmedOnce = false;
  triggerCooldownUntil = millis() + TRIGGER_COOLDOWN_MS;
  stopResponseOutputs();
  emit("EVT:RESPONSE_OK");
  setStatus(String(sourceLabel) + " pressed", "Alert canceled");
}

void failResponseTest() {
  responseTestActive = false;
  bpmConfirmedOnce = false;
  triggerCooldownUntil = millis() + TRIGGER_COOLDOWN_MS;
  stopResponseOutputs();
  emit("EVT:RESPONSE_FAIL");
  setStatus("No response", "Waiting for park");
}

void serviceResponseTest() {
  if (!responseTestActive) return;

  unsigned long now = millis();
  unsigned long elapsed = now - responseTestStart;

  if (now - lastBuzzToggle >= BUZZ_TOGGLE_MS) {
    lastBuzzToggle = now;
    buzzState = !buzzState;
    digitalWrite(BUZZER_PIN, buzzState ? HIGH : LOW);
  }

  if (elapsed >= RESPONSE_TEST_MS) failResponseTest();
}

bool buttonPressed(ButtonDebounce& btn) {
  int raw = digitalRead(btn.pin);
  unsigned long now = millis();

  if (raw != btn.lastRaw) {
    btn.lastRaw = raw;
    btn.lastChangeMs = now;
  }

  if ((now - btn.lastChangeMs) >= BTN_DEBOUNCE_MS && raw != btn.stable) {
    btn.stable = raw;
    if (btn.stable == LOW) return true;
  }

  return false;
}

void serviceButtons() {
  bool responsePressed = buttonPressed(responseBtn);
  bool cancelPressed = buttonPressed(cancelBtn);

  if (!responseTestActive) return;

  if (responsePressed) {
    cancelResponseTest("Btn13");
  } else if (cancelPressed) {
    cancelResponseTest("Btn15");
  }
}

void handleJetsonCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd.equalsIgnoreCase("PARKED")) {
    if (!emergencyInProgress) {
      if (responseTestActive) {
        responseTestActive = false;
        stopResponseOutputs();
      }
      runAlertSequence();
    }
    return;
  }

  if (cmd.equalsIgnoreCase("RESET")) {
    triggerCooldownUntil = millis();
    bpmConfirmedOnce = false;
    rrIndex = 0;
    rrBufferFull = false;
    currentBPM = 0.0f;
    emit("EVT:RESET_OK");
    updateIdleStatus();
    return;
  }

  if (cmd.equalsIgnoreCase("PROMPT")) {
    if (!emergencyInProgress && !responseTestActive) {
      triggerCooldownUntil = millis();
      bpmConfirmedOnce = true;
      startResponseTest();
    }
    return;
  }

  if (cmd.equalsIgnoreCase("STATUS")) {
    String s = "EVT:STATUS BPM=";
    s += (int)round(currentBPM);
    s += " LEADS=";
    s += leadsOff ? "off" : "ok";
    emit(s);
    return;
  }
}

void readTcpCommands() {
  static String inbuf = "";
  while (client.connected() && client.available()) {
    char c = client.read();
    if (c == '\n' || c == '\r') {
      if (inbuf.length() > 0) {
        handleJetsonCommand(inbuf);
        inbuf = "";
      }
    } else {
      inbuf += c;
      if (inbuf.length() > 64) inbuf = "";
    }
  }
}

void setup() {
  Serial.begin(115200);
  sim800.begin(9600, SERIAL_8N1, SIM800_RX, SIM800_TX);

  pinMode(LO_PLUS_PIN, INPUT);
  pinMode(LO_MINUS_PIN, INPUT);
  pinMode(RESPONSE_BTN_PIN, INPUT_PULLUP);
  pinMode(CANCEL_BTN_PIN, INPUT_PULLUP);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  display.begin(OLED_I2C_ADDR, true);
  display.clearDisplay();
  display.display();
  drawOLED();

  delay(3000);
  initGSM();
  wifiConnect();
  updateIdleStatus();
}

void loop() {
  ensureJetsonConnection();
  readTcpCommands();
  sampleECG();
  serviceButtons();
  serviceResponseTest();

  unsigned long now = millis();

  if (!responseTestActive && !emergencyInProgress
      && now >= triggerCooldownUntil && !bpmConfirmedOnce
      && rrSampleCount() >= BPM_MIN_SAMPLES_FOR_TRIGGER
      && currentBPM >= BPM_MIN_VALID && currentBPM <= BPM_MAX_VALID) {
    bpmConfirmedOnce = true;
    startResponseTest();
  }

  if (now - lastSecondTick >= 1000) {
    lastSecondTick = now;
    String s = "EVT:BPM ";
    s += (int)round(currentBPM);
    emit(s);

    if (!responseTestActive && !emergencyInProgress) updateIdleStatus();
  }

  drawOLED();
  delay(4);
}
