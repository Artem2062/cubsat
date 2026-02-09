// BaseStation_Compact_NoRL.ino
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define JOY_X_PIN A0
#define JOY_Y_PIN A1
#define JOY_BTN_PIN 3
#define CE_PIN 7
#define CSN_PIN 8

LiquidCrystal_I2C lcd(0x27, 16, 2);

RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "CUBE1";

struct CommandPacket {
  uint8_t header = 0x37;
  uint8_t sat_id = 0x25;
  uint8_t packet_num = 0;
  uint8_t script = 0;
  uint8_t relay_state = 0xFF;
  uint8_t laser_state = 0xFF;
  int8_t target_x = 0;
  int8_t target_y = 0;
};

struct TelemetryPacket {
  uint8_t header;
  uint8_t sat_id;
  uint8_t packet_num;
  uint8_t mode;
  uint8_t scan_step;
  uint8_t relay_state;
  uint8_t laser_state;
  int8_t pos_x;
  int8_t pos_y;
  uint8_t target_reached;
};

CommandPacket cmd;
TelemetryPacket telemetry;
uint8_t packetCounter = 0;

bool autoMode = false;
bool telemetryReceived = false;
unsigned long lastTelemetryTime = 0;

int16_t joyOffsetX = 0;
int16_t joyOffsetY = 0;
int8_t lastSentX = 0;
int8_t lastSentY = 0;

int8_t displayX = 0;
int8_t displayY = 0;
uint8_t displayScanStep = 0;

void setup() {
  Serial.begin(9600);
  
  pinMode(JOY_BTN_PIN, INPUT_PULLUP);
  
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("CubeSat Control");
  delay(1000);
  
  radio.begin();
  radio.openWritingPipe(address);
  radio.openReadingPipe(1, address);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.startListening();
  
  cmd.header = 0x37;
  cmd.sat_id = 0x25;
  cmd.script = 0;
  cmd.relay_state = 0xFF;
  cmd.laser_state = 0xFF;
  cmd.target_x = 0;
  cmd.target_y = 0;
  
  delay(1000);
  lcd.clear();
  
  Serial.println("CubeSat Control Ready");
  Serial.println("Commands: HELP, RELAY ON/OFF, LASER ON/OFF, POS X Y, CENTER, MANUAL, AUTO");
}

void loop() {
  unsigned long currentTime = millis();
  
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    processSerialCommand(input);
  }
  
  int joyX = analogRead(JOY_X_PIN);
  int joyY = analogRead(JOY_Y_PIN);
  bool joyBtn = !digitalRead(JOY_BTN_PIN);
  
  static bool lastBtnState = false;
  static unsigned long lastSendTime = 0;
  
  if (joyBtn && !lastBtnState) {
    toggleMode();
    delay(300);
  }
  lastBtnState = joyBtn;
  
  if (!autoMode && currentTime - lastSendTime > 100) {
    processJoystick(joyX, joyY);
    lastSendTime = currentTime;
  }
  
  receiveTelemetry();
  
  static unsigned long lastLCDUpdate = 0;
  if (currentTime - lastLCDUpdate > 300) {
    updateLCD();
    lastLCDUpdate = currentTime;
  }
  
  delay(20);
}

void toggleMode() {
  if (!autoMode) {
    autoMode = true;
    sendCommand(1, 0xFF, 0xFF, 0, 0);
    Serial.println("Mode: AUTO");
  } else {
    autoMode = false;
    sendCommand(2, 0xFF, 0xFF, 0, 0);
    joyOffsetX = 0;
    joyOffsetY = 0;
    lastSentX = 0;
    lastSentY = 0;
    Serial.println("Mode: MANUAL");
  }
}

void processJoystick(int joyX, int joyY) {
  int deadzone = 30;
  int center = 512;
  
  int deltaX = 0;
  int deltaY = 0;
  
  if (abs(joyX - center) > deadzone) {
    deltaY = map(joyX, 0, 1023, 8, -8);
  }
  
  if (abs(joyY - center) > deadzone) {
    deltaX = map(joyY, 0, 1023, 8, -8);
  }
  
  joyOffsetX += deltaX;
  joyOffsetY += deltaY;
  
  joyOffsetX = constrain(joyOffsetX, -40, 40);
  joyOffsetY = constrain(joyOffsetY, -40, 40);
  
  int8_t targetX = joyOffsetX;
  int8_t targetY = joyOffsetY;
  
  targetX = constrain(targetX, -40, 40);
  targetY = constrain(targetY, -40, 40);
  
  if (targetX != lastSentX || targetY != lastSentY) {
    // Отправляем координаты как есть (джойстик уже инвертирован в маппинге)
    sendCommand(0, 0xFF, 1, targetX, targetY);
    lastSentX = targetX;
    lastSentY = targetY;
    // На дисплей выводим инвертированные координаты
    displayX = -targetX;
    displayY = -targetY;
  }
}

void sendCommand(uint8_t script, uint8_t relay, uint8_t laser, int8_t x, int8_t y) {
  cmd.packet_num = packetCounter++;
  cmd.script = script;
  
  if (relay != 0xFF) {
    cmd.relay_state = relay;
  }
  
  if (laser != 0xFF) {
    cmd.laser_state = laser;
  }
  
  cmd.target_x = x;
  cmd.target_y = y;
  
  radio.stopListening();
  radio.write(&cmd, sizeof(cmd));
  radio.startListening();
  
  logCommand(script, relay, laser, x, y);
}

void logCommand(uint8_t script, uint8_t relay, uint8_t laser, int8_t x, int8_t y) {
  Serial.print("CMD #");
  Serial.print(cmd.packet_num);
  Serial.print(": ");
  
  switch(script) {
    case 0: Serial.print("MANUAL"); break;
    case 1: Serial.print("START"); break;
    case 2: Serial.print("STOP"); break;
    default: Serial.print("SYS"); break;
  }
  
  if (relay != 0xFF) {
    Serial.print(" R:");
    Serial.print(relay ? "ON" : "OFF");
  }
  
  if (laser != 0xFF) {
    Serial.print(" L:");
    Serial.print(laser ? "ON" : "OFF");
  }
  
  if (script == 0) {
    Serial.print(" X=");
    Serial.print(x);
    Serial.print(" Y=");
    Serial.print(y);
  }
  
  Serial.println();
}

void receiveTelemetry() {
  if (radio.available()) {
    TelemetryPacket received;
    radio.read(&received, sizeof(received));
    
    if (received.header == 0x38 && received.sat_id == 0x25) {
      memcpy(&telemetry, &received, sizeof(received));
      telemetryReceived = true;
      lastTelemetryTime = millis();
      
      if (autoMode) {
        // В авторежиме выводим инвертированные координаты с телеметрии
        displayX = -telemetry.pos_x;
        displayY = -telemetry.pos_y;
        displayScanStep = telemetry.scan_step;
      }
    }
  }
  
  if (telemetryReceived && millis() - lastTelemetryTime > 2000) {
    telemetryReceived = false;
  }
}

void updateLCD() {
  lcd.clear();
  
  lcd.setCursor(0, 0);
  
  if (autoMode) {
    lcd.print("AUTO");
    lcd.setCursor(5, 0);
    if (telemetryReceived) {
      lcd.print("S:");
      if (displayScanStep < 10) lcd.print("0");
      lcd.print(displayScanStep);
      lcd.print("/36");
    } else {
      lcd.print("NO SIG");
    }
  } else {
    lcd.print("MANUAL");
  }
  
  lcd.setCursor(0, 1);
  lcd.print("X:");
  
  // Форматированный вывод координат со знаком и ведущим нулем
  if (displayX >= 0) {
    lcd.print("+");
    if (displayX < 10) lcd.print("0");
    lcd.print(displayX);
  } else {
    lcd.print("-");
    if (displayX > -10) lcd.print("0");
    lcd.print(abs(displayX));
  }
  
  lcd.print(" Y:");
  
  if (displayY >= 0) {
    lcd.print("+");
    if (displayY < 10) lcd.print("0");
    lcd.print(displayY);
  } else {
    lcd.print("-");
    if (displayY > -10) lcd.print("0");
    lcd.print(abs(displayY));
  }
}

void processSerialCommand(String input) {
  input.trim();
  input.toUpperCase();
  
  if (input == "HELP") {
    Serial.println("Commands:");
    Serial.println("  RELAY ON/OFF");
    Serial.println("  LASER ON/OFF");
    Serial.println("  POS X Y     (X,Y = -40..40, инвертируются)");
    Serial.println("  CENTER");
    Serial.println("  MANUAL");
    Serial.println("  AUTO");
    Serial.println("  HELP");
  }
  else if (input == "RELAY ON") {
    sendCommand(0xFF, 1, 0xFF, 0, 0);
    Serial.println("Relay ON");
  }
  else if (input == "RELAY OFF") {
    sendCommand(0xFF, 0, 0xFF, 0, 0);
    Serial.println("Relay OFF");
  }
  else if (input == "LASER ON") {
    sendCommand(0xFF, 0xFF, 1, 0, 0);
    Serial.println("Laser ON");
  }
  else if (input == "LASER OFF") {
    sendCommand(0xFF, 0xFF, 0, 0, 0);
    Serial.println("Laser OFF");
  }
  else if (input == "MANUAL") {
    autoMode = false;
    sendCommand(2, 0xFF, 0xFF, 0, 0);
    joyOffsetX = 0;
    joyOffsetY = 0;
    lastSentX = 0;
    lastSentY = 0;
    displayX = 0;
    displayY = 0;
    Serial.println("Manual mode");
  }
  else if (input == "AUTO") {
    autoMode = true;
    sendCommand(1, 0xFF, 0xFF, 0, 0);
    Serial.println("Auto mode");
  }
  else if (input == "CENTER") {
    autoMode = false;
    sendCommand(0, 0xFF, 1, 0, 0);
    joyOffsetX = 0;
    joyOffsetY = 0;
    lastSentX = 0;
    lastSentY = 0;
    displayX = 0;
    displayY = 0;
    Serial.println("Centering");
  }
  else if (input.startsWith("POS")) {
    int space1 = input.indexOf(' ');
    int space2 = input.indexOf(' ', space1 + 1);
    if (space1 > 0 && space2 > 0) {
      int x = input.substring(space1 + 1, space2).toInt();
      int y = input.substring(space2 + 1).toInt();
      
      x = constrain(x, -40, 40);
      y = constrain(y, -40, 40);
      
      autoMode = false;
      // ИНВЕРСИЯ: отправляем инвертированные координаты
      sendCommand(0, 0xFF, 1, -x, -y);
      
      joyOffsetX = -x;
      joyOffsetY = -y;
      // На дисплей выводим прямые координаты (как вводил пользователь)
      displayX = x;
      displayY = y;
      
      Serial.print("Move to X=");
      Serial.print(x);
      Serial.print(" Y=");
      Serial.println(y);
      Serial.print("(Sent: X=");
      Serial.print(-x);
      Serial.print(" Y=");
      Serial.print(-y);
      Serial.println(")");
    } else {
      Serial.println("Invalid POS command format. Use: POS X Y");
    }
  }
  else if (input != "") {
    Serial.println("Unknown command. Type HELP for commands list.");
  }
}