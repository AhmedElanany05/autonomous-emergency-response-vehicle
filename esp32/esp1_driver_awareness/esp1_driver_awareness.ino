#include <Arduino.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <U8g2lib.h>

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

const char phoneNumber[] = "+20XXXXXXXXXX";

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

enum OledMode {
  OLED_ECG = 0,
  OLED_COUNTDOWN = 1,
  OLED_STATUS = 2
};

uint8_t oledMode = OLED_ECG;
String oledLine1 = "";
String oledLine2 = "";
int countdown = 0;
unsigned long lastDrawMs = 0;

HardwareSerial sim800(2);

WiFiClient client;
unsigned long lastReconnectAttempt = 0;
const unsigned long RECONNECT_INTERVAL = 3000;

#define WAVE_LEN 128
int16_t waveBuffer[WAVE_LEN];
uint8_t waveIndex = 0;

#define PEAK_THRESHOLD  2200
#define REFRACTORY_MS    300
#define BPM_WINDOW         8

uint32_t lastPeakMs = 0;
uint32_t rrIntervals[BPM_WINDOW];
uint8_t rrIndex = 0;
bool rrBufferFull = false;
float currentBPM = 0.0f;
bool aboveThresh = false;
bool leadsOff = false;
unsigned long bootTime = 0;

const unsigned long READING_WINDOW_MS = 5000UL;
const unsigned long CANCEL_WINDOW_MS = 5000UL;
const unsigned long TRIGGER_COOLDOWN_MS = 30000UL;

bool responseSequenceActive = false;
bool alertArmed = false;
bool emergencyInProgress = false;
unsigned long triggerCooldownUntil = 0;

const unsigned long BTN_DEBOUNCE_MS = 50;

struct ButtonDebounce {
  uint8_t pin;
  int lastRaw;
  int stable;
  unsigned long lastChangeMs;
};

ButtonDebounce responseBtn = { RESPONSE_BTN_PIN, HIGH, HIGH, 0 };
ButtonDebounce cancelBtn = { CANCEL_BTN_PIN, HIGH, HIGH, 0 };

unsigned long lastSecondTick = 0;

void emit(const String& line) {
  Serial.println(line);
  if (client.connected()) {
    client.print(line);
    client.print('\n');
  }
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
    return btn.stable == LOW;
  }

  return false;
}

bool anyCancelButtonPressed() {
  bool responsePressed = buttonPressed(responseBtn);
  bool cancelPressed = buttonPressed(cancelBtn);
  return responsePressed || cancelPressed;
}

void drawOLED() {
  u8g2.clearBuffer();

  if (oledMode == OLED_STATUS) {
    u8g2.drawHLine(0, 0, 128);
    u8g2.drawHLine(0, 63, 128);
    u8g2.setFont(u8g2_font_ncenB10_tr);
    u8g2.drawStr(0, 22, oledLine1.c_str());
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(0, 44, oledLine2.c_str());
  } else if (oledMode == OLED_COUNTDOWN) {
    u8g2.drawHLine(0, 13, 128);
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(0, 10, "BPM");

    u8g2.setFont(u8g2_font_logisoso16_tr);
    char bStr[6];
    snprintf(bStr, sizeof(bStr), "%3d", currentBPM > 0 ? (int)round(currentBPM) : 0);
    u8g2.drawStr(28, 11, bStr);

    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(0, 26, "ALERT IN:");
    u8g2.drawStr(70, 26, "BTN=CANCEL");

    u8g2.setFont(u8g2_font_logisoso32_tr);
    char cStr[4];
    snprintf(cStr, sizeof(cStr), "%d", countdown);
    int w = u8g2.getStrWidth(cStr);
    u8g2.drawStr((128 - w) / 2, 62, cStr);
  } else {
    u8g2.setFont(u8g2_font_04b_03_tr);
    u8g2.drawStr(0, 7, "ECG MONITOR");

    static bool dotOn = false;
    static unsigned long lastDotMs = 0;
    if (millis() - lastDotMs > 300) {
      dotOn = !dotOn;
      lastDotMs = millis();
    }
    if (dotOn) u8g2.drawDisc(110, 4, 3);
    else u8g2.drawCircle(110, 4, 3);

    u8g2.drawHLine(0, 9, 128);
    u8g2.drawVLine(90, 9, 55);

    u8g2.setFont(u8g2_font_04b_03_tr);
    u8g2.drawStr(93, 20, "HEART");
    u8g2.drawStr(93, 28, "RATE");

    u8g2.setFont(u8g2_font_logisoso16_tr);
    char bpmStr[5];
    if (currentBPM > 0) snprintf(bpmStr, sizeof(bpmStr), "%3d", (int)round(currentBPM));
    else strcpy(bpmStr, "---");
    u8g2.drawStr(92, 50, bpmStr);

    u8g2.setFont(u8g2_font_04b_03_tr);
    u8g2.drawStr(95, 60, "bpm");

    const int PLOT_X = 0;
    const int PLOT_W = 89;
    const int PLOT_Y = 11;
    const int PLOT_H = 52;
    const int Y_MID = PLOT_Y + PLOT_H / 2;

    for (int x = PLOT_X; x < PLOT_X + PLOT_W; x += 4) {
      u8g2.drawPixel(x, Y_MID);
      u8g2.drawPixel(x + 1, Y_MID);
    }

    for (int x = 0; x < PLOT_W - 1; x++) {
      int idx0 = (waveIndex + x) % WAVE_LEN;
      int idx1 = (waveIndex + x + 1) % WAVE_LEN;

      int y0 = Y_MID - map(waveBuffer[idx0], 0, 4095, -(PLOT_H / 2 - 2), (PLOT_H / 2 - 2));
      int y1 = Y_MID - map(waveBuffer[idx1], 0, 4095, -(PLOT_H / 2 - 2), (PLOT_H / 2 - 2));
      y0 = constrain(y0, PLOT_Y, 63);
      y1 = constrain(y1, PLOT_Y, 63);

      u8g2.drawLine(PLOT_X + x, y0, PLOT_X + x + 1, y1);
    }

    int cursorX = PLOT_X + (waveIndex % PLOT_W);
    for (int y = PLOT_Y; y <= 63; y += 2) u8g2.drawPixel(cursorX, y);
  }

  u8g2.sendBuffer();
}

void buzzerOn() {
  digitalWrite(BUZZER_PIN, HIGH);
}

void buzzerOff() {
  digitalWrite(BUZZER_PIN, LOW);
}

void buzzer(int ms) {
  buzzerOn();
  delay(ms);
  buzzerOff();
}

void wifiConnect() {
  WiFi.disconnect(true);
  delay(500);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  oledMode = OLED_STATUS;
  oledLine1 = "Connecting WiFi";
  oledLine2 = "";
  drawOLED();

  Serial.print("WiFi connecting");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    Serial.print('.');
    delay(500);
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    oledLine1 = "WiFi OK";
    oledLine2 = WiFi.localIP().toString();
  } else {
    oledLine1 = "WiFi FAILED";
    oledLine2 = "retrying...";
  }

  drawOLED();
  delay(1500);
  oledMode = OLED_ECG;
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

String readResp(unsigned long timeoutMs) {
  String resp = "";
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    while (sim800.available()) resp += (char)sim800.read();
    delay(1);
  }
  return resp;
}

String sendCmdGetResp(const char* cmd, unsigned long waitMs = 3000) {
  while (sim800.available()) sim800.read();
  sim800.println(cmd);
  return readResp(waitMs);
}

bool sendCmd(const char* cmd, unsigned long waitMs = 3000, const char* mustContain = "OK") {
  String resp = sendCmdGetResp(cmd, waitMs);
  if (mustContain == nullptr) return true;
  return resp.indexOf(mustContain) != -1;
}

bool modemAlive() {
  for (int i = 0; i < 3; i++) {
    if (sendCmd("AT", 2000, "OK")) return true;
    delay(1000);
  }
  return false;
}

bool modemReady() {
  if (!modemAlive()) return false;
  if (sendCmdGetResp("AT+CPIN?", 3000).indexOf("READY") == -1) return false;
  String reg = sendCmdGetResp("AT+CREG?", 3000);
  return (reg.indexOf("+CREG: 0,1") != -1 || reg.indexOf("+CREG: 0,5") != -1);
}

bool sendSMS(const String& message) {
  while (sim800.available()) sim800.read();
  if (!modemReady()) return false;
  if (!sendCmd("AT+CMEE=2", 2000, "OK")) return false;
  if (!sendCmd("AT+CMGF=1", 2000, "OK")) return false;

  sim800.print("AT+CMGS=\"");
  sim800.print(phoneNumber);
  sim800.print("\"\r");
  if (readResp(5000).indexOf('>') == -1) return false;

  sim800.print(message);
  delay(300);
  sim800.write(26);
  String finalResp = readResp(20000);
  return (finalResp.indexOf("+CMGS:") != -1 && finalResp.indexOf("OK") != -1);
}

bool makeCall() {
  while (sim800.available()) sim800.read();
  if (!modemReady()) return false;

  sim800.print("ATD");
  sim800.print(phoneNumber);
  sim800.println(";");
  String resp = readResp(10000);
  return (resp.indexOf("OK") != -1 || resp.indexOf("CONNECT") != -1);
}

bool isCallAnswered() {
  String resp = sendCmdGetResp("AT+CLCC", 3000);
  return (resp.indexOf("+CLCC:") != -1 && resp.indexOf(",0,0,0,0") != -1);
}

bool waitForAnswer(unsigned long timeoutMs) {
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    if (isCallAnswered()) return true;
    delay(500);
  }
  return false;
}

void playBeeps() {
  sendCmd("AT+DTAM=1", 3000, nullptr);
  sendCmd("AT+CLDTMF=5,\"1,1,1,1,1,1,1,1\",100", 12000, nullptr);
}

void hangUp() {
  while (sim800.available()) sim800.read();
  sim800.println("ATH");
  readResp(3000);
}

void sampleWaveOnly() {
  int raw = leadsOff ? 2048 : analogRead(ECG_OUTPUT);
  waveBuffer[waveIndex] = (int16_t)raw;
  waveIndex = (waveIndex + 1) % WAVE_LEN;
}

void resetResponseState() {
  alertArmed = false;
  responseSequenceActive = false;
  triggerCooldownUntil = millis() + TRIGGER_COOLDOWN_MS;
}

bool waitForCancelWindow() {
  responseSequenceActive = true;
  emit("EVT:RESPONSE_TEST_START");

  oledMode = OLED_ECG;
  unsigned long readStart = millis();
  while (millis() - readStart < READING_WINDOW_MS) {
    ensureJetsonConnection();
    sampleWaveOnly();
    if (anyCancelButtonPressed()) {
      emit("EVT:RESPONSE_OK");
      resetResponseState();
      return false;
    }
    drawOLED();
    delay(8);
  }

  oledMode = OLED_COUNTDOWN;
  buzzerOn();
  digitalWrite(STATUS_LED_PIN, HIGH);

  unsigned long countdownStart = millis();
  while (millis() - countdownStart < CANCEL_WINDOW_MS) {
    ensureJetsonConnection();

    unsigned long elapsed = millis() - countdownStart;
    countdown = (int)((CANCEL_WINDOW_MS - elapsed + 999UL) / 1000UL);
    drawOLED();

    if (anyCancelButtonPressed()) {
      buzzerOff();
      digitalWrite(STATUS_LED_PIN, LOW);
      emit("EVT:RESPONSE_OK");

      oledMode = OLED_STATUS;
      oledLine1 = "CANCELLED";
      oledLine2 = "Alert stopped";
      drawOLED();
      delay(1500);

      resetResponseState();
      oledMode = OLED_ECG;
      return false;
    }

    delay(50);
  }

  buzzerOff();
  digitalWrite(STATUS_LED_PIN, LOW);
  resetResponseState();
  return true;
}

void startDetectedHeartRateResponseTest() {
  if (responseSequenceActive || emergencyInProgress || millis() < triggerCooldownUntil) return;

  alertArmed = true;
  bool proceed = waitForCancelWindow();
  if (!proceed) {
    oledMode = OLED_ECG;
    return;
  }

  emit("EVT:RESPONSE_FAIL");

  oledMode = OLED_STATUS;
  oledLine1 = "Alert Triggered";
  oledLine2 = "Car parking...";
  drawOLED();
}

void sampleECG() {
  if (millis() - bootTime < 5000) return;

  bool nowLeadsOff = (digitalRead(LO_PLUS_PIN) == HIGH) || (digitalRead(LO_MINUS_PIN) == HIGH);
  if (nowLeadsOff != leadsOff) {
    leadsOff = nowLeadsOff;
    emit(leadsOff ? "EVT:LEADS_OFF" : "EVT:LEADS_OK");
    if (leadsOff) {
      alertArmed = false;
      responseSequenceActive = false;
    }
  }

  int rawSample = leadsOff ? 2048 : analogRead(ECG_OUTPUT);
  waveBuffer[waveIndex] = (int16_t)rawSample;
  waveIndex = (waveIndex + 1) % WAVE_LEN;

  if (leadsOff) {
    aboveThresh = false;
    return;
  }

  unsigned long now = millis();
  if (!aboveThresh && rawSample > PEAK_THRESHOLD) {
    aboveThresh = true;

    if (now - lastPeakMs > REFRACTORY_MS) {
      if (lastPeakMs > 0) {
        uint32_t rr = now - lastPeakMs;
        if (rr > 300 && rr < 2000) {
          rrIntervals[rrIndex % BPM_WINDOW] = rr;
          rrIndex++;
          if (rrIndex >= BPM_WINDOW) rrBufferFull = true;

          uint8_t count = rrBufferFull ? BPM_WINDOW : rrIndex;
          uint32_t sum = 0;
          for (uint8_t i = 0; i < count; i++) sum += rrIntervals[i];
          currentBPM = 60000.0f / ((float)sum / count);
        }
      }

      lastPeakMs = now;

      if (!alertArmed && !responseSequenceActive && !emergencyInProgress
          && now >= triggerCooldownUntil && rrIndex >= 3
          && currentBPM > 30.0f && currentBPM < 220.0f) {
        startDetectedHeartRateResponseTest();
      }
    }
  }

  if (aboveThresh && rawSample < (PEAK_THRESHOLD - 150)) aboveThresh = false;
}

void runAlertSequence() {
  emergencyInProgress = true;
  responseSequenceActive = false;
  buzzerOff();
  digitalWrite(STATUS_LED_PIN, LOW);
  emit("EVT:ALERT_START");

  oledMode = OLED_STATUS;
  oledLine1 = "EMERGENCY";
  oledLine2 = "Sending SMS...";
  drawOLED();

  String smsText = "THE DRIVER IS FACING A MEDICAL EMERGENCY AT THIS LOCATION: LATITUDE,LONGITUDE";
  bool smsOk = sendSMS(smsText);
  emit(smsOk ? "EVT:SMS_OK" : "EVT:SMS_FAIL");
  buzzer(3000);

  oledLine2 = smsOk ? "SMS OK. Calling..." : "SMS FAIL. Calling...";
  drawOLED();
  delay(2000);

  if (makeCall()) {
    emit("EVT:CALL_OK");
    buzzer(3000);
    oledLine1 = "CALLING...";
    oledLine2 = "Waiting answer";
    drawOLED();

    if (waitForAnswer(30000)) {
      emit("EVT:CALL_ANSWERED");
      oledLine1 = "CALL ANSWERED";
      oledLine2 = "Playing beeps...";
      drawOLED();
      playBeeps();
      delay(5000);
    } else {
      emit("EVT:CALL_NOANSWER");
      oledLine1 = "NO ANSWER";
      oledLine2 = "Hanging up...";
      drawOLED();
    }

    hangUp();
  } else {
    emit("EVT:CALL_FAIL");
    oledLine1 = "CALL FAILED";
    oledLine2 = "";
    drawOLED();
  }

  emit("EVT:DONE");
  oledLine1 = "ALERT SENT";
  oledLine2 = "Stay calm";
  drawOLED();
  delay(3000);

  oledMode = OLED_ECG;
  emergencyInProgress = false;
}

void handleJetsonCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd.equalsIgnoreCase("PARKED")) {
    if (!emergencyInProgress) runAlertSequence();
    return;
  }

  if (cmd.equalsIgnoreCase("RESET")) {
    triggerCooldownUntil = millis();
    alertArmed = false;
    responseSequenceActive = false;
    rrIndex = 0;
    rrBufferFull = false;
    currentBPM = 0.0f;
    emit("EVT:RESET_OK");
    oledMode = OLED_ECG;
    return;
  }

  if (cmd.equalsIgnoreCase("PROMPT")) {
    if (!emergencyInProgress && !responseSequenceActive) {
      triggerCooldownUntil = millis();
      startDetectedHeartRateResponseTest();
    }
    return;
  }

  if (cmd.equalsIgnoreCase("STATUS")) {
    emit("EVT:STATUS BPM=" + String((int)round(currentBPM)) + " LEADS=" + (leadsOff ? "off" : "ok"));
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

  pinMode(LO_PLUS_PIN, INPUT);
  pinMode(LO_MINUS_PIN, INPUT);
  pinMode(RESPONSE_BTN_PIN, INPUT_PULLUP);
  pinMode(CANCEL_BTN_PIN, INPUT_PULLUP);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  buzzerOff();

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  sim800.begin(9600, SERIAL_8N1, SIM800_RX, SIM800_TX);

  for (int i = 0; i < WAVE_LEN; i++) waveBuffer[i] = 2048;

  u8g2.begin();

  for (int x = 0; x < 128; x += 4) {
    u8g2.clearBuffer();
    u8g2.drawVLine(x, 0, 64);
    u8g2.setFont(u8g2_font_04b_03_tr);
    u8g2.drawStr(30, 38, "ECG MONITOR");
    u8g2.sendBuffer();
    delay(10);
  }

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB10_tr);
  u8g2.drawStr(10, 30, "ECG MONITOR");
  u8g2.setFont(u8g2_font_04b_03_tr);
  u8g2.drawStr(25, 46, "Initializing...");
  u8g2.sendBuffer();

  bootTime = millis();
  delay(2000);

  sendCmd("AT", 2000, nullptr);
  sendCmd("AT+CMEE=2", 2000, nullptr);
  sendCmd("AT+CPIN?", 3000, nullptr);
  sendCmd("AT+CREG?", 3000, nullptr);
  sendCmd("AT+CSQ", 3000, nullptr);

  wifiConnect();
  Serial.println("System ready.");
}

void loop() {
  ensureJetsonConnection();
  readTcpCommands();
  sampleECG();

  unsigned long now = millis();

  if (now - lastSecondTick >= 1000) {
    lastSecondTick = now;
    emit("EVT:BPM " + String((int)round(currentBPM)));
  }

  if (oledMode == OLED_ECG && now - lastDrawMs >= 8) {
    lastDrawMs = now;
    drawOLED();
  }

  delay(4);
}
