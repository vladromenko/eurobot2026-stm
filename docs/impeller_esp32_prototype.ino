/*
 * Управление турбинкой 12 В 120 000 об/мин (вариант с потенциометром и кнопкой)
 * Плата: ESP32 DevKit (классический ESP32, ЦАП на GPIO25)
 * Arduino IDE: Tools -> Board -> "ESP32 Dev Module", Upload Speed 115200
 *
 * ПОДКЛЮЧЕНИЕ
 *   Красный провод турбинки   -> +12 В (аккумулятор 3S через предохранитель 15–20 А)
 *   Чёрный провод турбинки    -> −12 В  И  GND ESP32 (общая земля обязательна!)
 *   Потенциометр отпаян:
 *     средний провод          -> GPIO25 через резистор 1 кОм   (сигнал скорости)
 *     земляной провод         -> GND
 *     опорный провод (3,3–5В) -> НИКУДА, заизолировать
 *   Кнопка (через MOSFET 2N7000, ножки S-G-D слева направо, плоской стороной к себе):
 *     D (сток)                -> сигнальный провод кнопки
 *     S (исток)               -> GND
 *     G (затвор)              -> GPIO27 через 220 Ом, плюс 10–100 кОм с затвора на GND
 *     земляной провод кнопки  -> GND
 *
 * КОМАНДЫ (Serial Monitor, 115200 бод, окончание строки "New Line"):
 *   on            — включить (длинное нажатие кнопки), после этого можно задавать скорость
 *   off           — плавно остановить и выключить
 *   s <0..100>    — скорость в процентах, например: s 40
 *   v <0..3.3>    — напрямую задать напряжение сигнала (калибровка), сбрасывается командой s/off
 *   press <мс>    — «нажать» кнопку на заданное время, например: press 1500
 *   status        — текущее состояние
 */

#include <Arduino.h>

// ======================= НАСТРОЙКИ =======================

// Сигнал скорости
#define USE_DAC          1      // 1 = ЦАП на GPIO25 (ровное напряжение 0–3,3 В)
                                // 0 = ШИМ на GPIO25 (если контроллер ждёт ШИМ или у вас ESP32-S3/C3)
const int   SPEED_PIN      = 25;
const int   PWM_FREQ       = 1000;  // для USE_DAC 0: пробуйте 1000, затем 20000
const int   PWM_BITS       = 8;

// Калибровка: подставьте значения из своих замеров (команда v)
const float V_STOP         = 0.0;   // напряжение «стоп»
const float V_MIN          = 0.5;   // напряжение, при котором мотор начинает крутиться
const float V_MAX          = 3.3;   // напряжение для максимума (не больше 3.3)
const bool  INVERT_SPEED   = false; // true, если у контроллера 0 В = максимум

// Кнопка через 2N7000 (HIGH на GPIO27 = кнопка нажата)
const bool  USE_BUTTON     = true;  // false, если включаете кнопкой руками
const int   BUTTON_PIN     = 27;
const int   BUTTON_LONG_MS = 200;   // короткое нажатие (нажал-отпустил); подберите командой press
const int   BOOT_DELAY_MS  = 1000;  // пауза после включения перед разгоном

// Ограничения
const int   MAX_SPEED_PCT  = 30;    // для первых тестов 30; потом можно 100
const float RAMP_PCT_PER_S = 40.0;  // скорость плавного разгона/торможения, %/с

// =========================================================

bool  fanOn          = false;
bool  manualVoltage  = false;  // режим калибровки (команда v)
float manualVolts    = 0;
float targetPct      = 0;
float currentPct     = 0;
unsigned long lastRampMs = 0;
String rx;

// ---------- низкий уровень ----------

void writeSignalVolts(float v) {
  v = constrain(v, 0.0f, 3.3f);
  if (INVERT_SPEED) v = 3.3f - v;
#if USE_DAC
  dacWrite(SPEED_PIN, (uint8_t)(v / 3.3f * 255.0f + 0.5f));
#else
  ledcWrite(SPEED_PIN, (uint32_t)(v / 3.3f * ((1 << PWM_BITS) - 1) + 0.5f));
#endif
}

void applySpeedPct(float pct) {
  if (pct <= 0.0f) writeSignalVolts(V_STOP);
  else             writeSignalVolts(V_MIN + (V_MAX - V_MIN) * pct / 100.0f);
}

void pressButton(int ms) {
  if (!USE_BUTTON) {
    Serial.println("Кнопка отключена (USE_BUTTON = false)");
    return;
  }
  digitalWrite(BUTTON_PIN, HIGH);   // MOSFET открыт = кнопка нажата
  delay(ms);
  digitalWrite(BUTTON_PIN, LOW);    // отпущена
}

// ---------- высокий уровень ----------

void impellerOn() {
  if (fanOn) { Serial.println("Уже включена"); return; }
  manualVoltage = false;
  currentPct = targetPct = 0;
  applySpeedPct(0);
  pressButton(BUTTON_LONG_MS);
  delay(BOOT_DELAY_MS);
  fanOn = true;
  Serial.println("Турбинка: ВКЛ. Задайте скорость: s <0..100>");
}

void impellerOff() {
  manualVoltage = false;
  targetPct = 0;
  while (currentPct > 0) {          // плавное торможение
    currentPct = max(0.0f, currentPct - RAMP_PCT_PER_S * 0.02f);
    applySpeedPct(currentPct);
    delay(20);
  }
  applySpeedPct(0);
  if (fanOn) {
    delay(300);
    pressButton(BUTTON_LONG_MS);
  }
  fanOn = false;
  Serial.println("Турбинка: ВЫКЛ");
}

void setSpeed(float pct) {
  manualVoltage = false;
  targetPct = constrain(pct, 0.0f, (float)MAX_SPEED_PCT);
  if (pct > MAX_SPEED_PCT)
    Serial.printf("Ограничено MAX_SPEED_PCT = %d%%\n", MAX_SPEED_PCT);
  Serial.printf("Скорость -> %.0f%%\n", targetPct);
}

void rampUpdate() {
  unsigned long now = millis();
  float dt = (now - lastRampMs) / 1000.0f;
  lastRampMs = now;

  if (manualVoltage) { writeSignalVolts(manualVolts); return; }

  float goal = fanOn ? targetPct : 0;
  float step = RAMP_PCT_PER_S * dt;
  if (currentPct < goal)      currentPct = min(goal, currentPct + step);
  else if (currentPct > goal) currentPct = max(goal, currentPct - step);
  applySpeedPct(currentPct);
}

// ---------- команды ----------

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();
  if (cmd.length() == 0) return;

  if (cmd == "on") impellerOn();
  else if (cmd == "off") impellerOff();
  else if (cmd.startsWith("s ")) {
    if (!fanOn) { Serial.println("Сначала введите: on"); return; }
    setSpeed(cmd.substring(2).toFloat());
  }
  else if (cmd.startsWith("v ")) {
    manualVolts = constrain(cmd.substring(2).toFloat(), 0.0f, 3.3f);
    manualVoltage = true;
    currentPct = targetPct = 0;
    Serial.printf("Калибровка: сигнал = %.2f В (выход: s <..> или off)\n", manualVolts);
  }
  else if (cmd.startsWith("press ")) {
    int ms = cmd.substring(6).toInt();
    Serial.printf("Нажатие кнопки %d мс\n", ms);
    pressButton(ms);
    Serial.println("Отпущена");
  }
  else if (cmd == "status") {
    Serial.printf("fanOn=%d, скорость: цель %.0f%%, сейчас %.0f%%, калибровка=%d (%.2f В)\n",
                  fanOn, targetPct, currentPct, manualVoltage, manualVolts);
  }
  else Serial.println("Команды: on | off | s <0..100> | v <0..3.3> | press <мс> | status");
}

// ---------- setup / loop ----------

void setup() {
  // кнопку отпускаем как можно раньше, чтобы при старте ничего не «нажалось»
  pinMode(BUTTON_PIN, OUTPUT);
  digitalWrite(BUTTON_PIN, LOW);

  Serial.begin(115200);

#if !USE_DAC
  ledcAttach(SPEED_PIN, PWM_FREQ, PWM_BITS);   // Arduino-ESP32 core 3.x
#endif
  writeSignalVolts(V_STOP);

  lastRampMs = millis();
  delay(300);
  Serial.println("\nУправление турбинкой готово.");
  Serial.println("Команды: on | off | s <0..100> | v <0..3.3> | press <мс> | status");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { handleCommand(rx); rx = ""; }
    else rx += c;
  }
  rampUpdate();
  delay(10);
}
