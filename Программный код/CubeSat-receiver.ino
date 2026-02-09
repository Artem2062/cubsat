// CubeSat_Radio_Only.ino
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

#define EMERGENCY_PIN 2
#define RELAY_PIN     3
#define LASER_PIN     6
#define CE_PIN        7
#define CSN_PIN       8
#define SERVO_X_PIN   9
#define SERVO_Y_PIN   5

RF24 radio(CE_PIN, CSN_PIN);
Servo servoX, servoY;
const byte address[6] = "CUBE1";

// Расширенная структура команды с управлением реле и лазером
struct CommandPacket {
  uint8_t header;
  uint8_t sat_id;
  uint8_t packet_num;
  uint8_t script;
  uint8_t relay_state;  // 0=OFF, 1=ON, 0xFF=не менять
  uint8_t laser_state;  // 0=OFF, 1=ON, 0xFF=не менять
  int8_t target_x;
  int8_t target_y;
};

struct TelemetryPacket {
  uint8_t header = 0x38;
  uint8_t sat_id = 0x25;
  uint8_t packet_num = 0;
  uint8_t mode = 0;
  uint8_t scan_step = 0;
  uint8_t relay_state = 1;  // Состояние реле (1=ON по умолчанию)
  uint8_t laser_state = 1;  // Состояние лазера (1=ON по умолчанию)
  int8_t pos_x = 0;
  int8_t pos_y = 0;
  uint8_t target_reached = 1;
};

CommandPacket cmd;
TelemetryPacket telemetry;
uint8_t telemetryCounter = 0;


bool emergencyStop = false;
uint8_t currentMode = 0;
uint8_t scanStep = 0;
unsigned long lastStepTime = 0;
unsigned long lastMoveTime = 0;
unsigned long lastTelemetryTime = 0;

float currentX = 0;
float currentY = 0;
float targetX = 0;
float targetY = 0;
bool targetReached = true;
bool lastPointDelayDone = false;
unsigned long lastPointTime = 0;

const float MANUAL_SPEED = 0.4;
const float AUTO_SPEED = 0.99;
const unsigned long MOVE_INTERVAL = 20;

const int8_t scanPattern[36][2] = {
  {-43,0},{-33,0},{-23,0},{-13,0},{-2,0},{8,0},{18,0},{28,0},{38,0},
  {0,-40},{0,-30},{0,-20},{-0,-10},{0,0},{0,10},{0,20},{0,30},{0,40},
  {-40,-40},{-34,-30},{-24,-20},{-14,-10},{0,0},{10,10},{20,20},{30,30},{40,40},
  {-40,40},{-30,30},{-20,20},{-10,10},{0,0},{10,-10},{20,-20},{30,-30},{40,-40}
};

void setup() {
  pinMode(EMERGENCY_PIN, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LASER_PIN, OUTPUT);
  
  // Включение по умолчанию
  digitalWrite(RELAY_PIN, HIGH);
  digitalWrite(LASER_PIN, HIGH);
  
  servoX.attach(SERVO_X_PIN);
  servoY.attach(SERVO_Y_PIN);
  
  moveToPosition(0, 0);
  delay(1000);
  
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.startListening();
  
  telemetry.sat_id = 0x25;
  telemetry.relay_state = 1;
  telemetry.laser_state = 1;
  telemetry.target_reached = 1;
}

void loop() {
  unsigned long currentTime = millis();
  
  if (digitalRead(EMERGENCY_PIN) == LOW) {
    emergencyStop = true;
    digitalWrite(RELAY_PIN, LOW);
    digitalWrite(LASER_PIN, LOW);
    currentMode = 0;
    scanStep = 0;
    lastPointDelayDone = false;
    delay(1000);
    return;
  }
  
  if (radio.available()) {
    radio.read(&cmd, sizeof(cmd));
    
    if (cmd.header == 0x37 && cmd.sat_id == 0x25) {
      processCommand();
    }
  }
  
  if (currentTime - lastMoveTime >= MOVE_INTERVAL) {
    smoothMove();
    lastMoveTime = currentTime;
    
    float tolerance = (currentMode == 1) ? 0.3 : 0.5;
    if (fabs(currentX - targetX) < tolerance && fabs(currentY - targetY) < tolerance) {
      targetReached = true;
    } else {
      targetReached = false;
    }
  }
  
  if (currentMode == 1) {
    doScanning();
  }
  
  if (currentTime - lastTelemetryTime >= 300) {
    sendTelemetry();
    lastTelemetryTime = currentTime;
  }
  
  delay(10);
}

void processCommand() {
  // Управление реле
  if (cmd.relay_state == 0) {
    digitalWrite(RELAY_PIN, LOW);
    telemetry.relay_state = 0;
  } else if (cmd.relay_state == 1) {
    digitalWrite(RELAY_PIN, HIGH);
    telemetry.relay_state = 1;
  }
  
  // Управление лазером
  if (cmd.laser_state == 0) {
    digitalWrite(LASER_PIN, LOW);
    telemetry.laser_state = 0;
  } else if (cmd.laser_state == 1) {
    digitalWrite(LASER_PIN, HIGH);
    telemetry.laser_state = 1;
  }
  
  // Обработка скриптов
  if (cmd.script == 1) { // START
    if (currentMode != 1) {
      currentMode = 1;
      scanStep = 0;
      lastStepTime = millis();
      targetReached = true;
      lastPointDelayDone = false;
    }
  }
  else if (cmd.script == 2) { // STOP
    if (currentMode == 1) {
      currentMode = 0;
      scanStep = 0;
      targetX = 0;
      targetY = 0;
      lastPointDelayDone = false;
    }
  }
  else if (cmd.script == 0) { // MANUAL
    currentMode = 0;
    lastPointDelayDone = false;
    
    if (cmd.target_x >= -40 && cmd.target_x <= 40) {
      targetX = cmd.target_x;
    }
    if (cmd.target_y >= -40 && cmd.target_y <= 40) {
      targetY = cmd.target_y;
    }
  }
}

void smoothMove() {
  float speed = (currentMode == 1) ? AUTO_SPEED : MANUAL_SPEED;
  
  if (currentX < targetX) {
    currentX += speed;
    if (currentX > targetX) currentX = targetX;
    updateServos();
  } else if (currentX > targetX) {
    currentX -= speed;
    if (currentX < targetX) currentX = targetX;
    updateServos();
  }
  
  if (currentY < targetY) {
    currentY += speed;
    if (currentY > targetY) currentY = targetY;
    updateServos();
  } else if (currentY > targetY) {
    currentY -= speed;
    if (currentY < targetY) currentY = targetY;
    updateServos();
  }
}

void moveToPosition(int8_t x, int8_t y) {
  currentX = x;
  currentY = y;
  targetX = x;
  targetY = y;
  targetReached = true;
  updateServos();
}

void updateServos() {
  int16_t pwmX = 1500 + (int16_t)(currentX * 12);
  int16_t pwmY = 1500 + (int16_t)(currentY * 12);
  
  pwmX = constrain(pwmX, 500, 2500);
  pwmY = constrain(pwmY, 500, 2500);
  
  servoX.writeMicroseconds(pwmX);
  servoY.writeMicroseconds(pwmY);
}

void doScanning() {
  if (scanStep >= 36) {
    // Достигли последней точки сканирования
    if (!targetReached) {
      // Ждем, пока сервоприводы достигнут последней точки
      float tolerance = 0.3;
      if (fabs(currentX - targetX) < tolerance && fabs(currentY - targetY) < tolerance) {
        targetReached = true;
      }
    } else {
      // Ждем 3 секунды на последней точке
      if (!lastPointDelayDone) {
        lastPointTime = millis();
        lastPointDelayDone = true;
      }
      
      if (millis() - lastPointTime >= 3000) {
        // Завершаем сканирование и возвращаемся в центр
        currentMode = 0;
        scanStep = 0;
        targetX = 0;
        targetY = 0;
        lastPointDelayDone = false;
      }
    }
    return;
  }
  
  // Переход к следующей точке сканирования
  if (targetReached && (millis() - lastStepTime >= 3000)) {
    targetX = scanPattern[scanStep][0];
    targetY = scanPattern[scanStep][1];
    targetReached = false;
    scanStep++;
    lastStepTime = millis();
  }
}

void sendTelemetry() {
  telemetry.packet_num = telemetryCounter++;
  telemetry.mode = currentMode;
  telemetry.scan_step = scanStep + 1;
  telemetry.pos_x = (int8_t)currentX;
  telemetry.pos_y = (int8_t)currentY;
  telemetry.target_reached = targetReached ? 1 : 0;
  
  radio.stopListening();
  radio.write(&telemetry, sizeof(telemetry));
  radio.startListening();
}