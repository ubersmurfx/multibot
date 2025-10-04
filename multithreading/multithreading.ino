#include <PS2X_lib.h>
// library for support PS2 controller and esp32 https://github.com/MyArduinoLib/Arduino-PS2X-ESP32.git

#define PS2_DAT 19
#define PS2_CMD 23
#define PS2_SEL 5
#define PS2_CLK 18
#define R_UP 32
#define R_GND 33
#define L_UP 26
#define L_GND 25
#define ledPin 2
#define ledPin2 4
#define ledPin3 16
#define ledPin4 17
#define BT1 27
#define pressures true
#define rumble false

uint8_t speedR, speedL;
uint8_t valueR, valueL;
bool navR, navL;
byte vibrate = 0;
byte type = 0;
int8_t error = -1;
uint8_t tryNum = 1;
boolean button1WasUp = true;
int state = 0; 

// <------------------>
unsigned long timer1, timer2;
// <------------------>
PS2X ps2x;

// Буфер для обработки прерываний
volatile int joystickValueL = 128;
volatile int joystickValueR = 128;
volatile bool padUp = false;
volatile bool padDown = false;
volatile bool padLeft = false;
volatile bool padRight = false;
volatile int padUpSpeed = 0;
volatile int padDownSpeed = 0;
volatile int padLeftSpeed = 0;
volatile int padRightSpeed = 0;

// Семафор
SemaphoreHandle_t xDataSemaphore;

// Переменные для обработки автономного движения
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
bool ledState = false;
bool autonomousMode = false; // Флаг автономного движения
const int autonomousSpeed = 150; // Скорость автономного движения

static bool crossWasPressed = false;
static bool circleWasPressed = false;


class Motor {
private:
  int pinUp;
  int pinGnd;
  
public:
  Motor(int upPin, int gndPin) : pinUp(upPin), pinGnd(gndPin) {
    pinMode(pinUp, OUTPUT);
    pinMode(pinGnd, OUTPUT);
    stop();
  }

  void forward(int speed) {
    speed = constrain(speed, 0, 255);
    analogWrite(pinUp, speed);
    analogWrite(pinGnd, 0);
    Serial.print("вперед");
    Serial.println(speed);
  }
  
  void backward(int speed) {
    speed = constrain(speed, 0, 255);
    analogWrite(pinGnd, speed);
    analogWrite(pinUp, 0);
    Serial.print("назад");
    Serial.println(speed);
  }
  
  void stop() {
    analogWrite(pinUp, 0);
    analogWrite(pinGnd, 0);
  }
};

Motor leftMotor(L_UP, L_GND);
Motor rightMotor(R_UP, R_GND);

void joystickTask(void *parameter) {
  while (1) {
    if (type == 1) {
      ps2x.read_gamepad(false, vibrate);
      if (xSemaphoreTake(xDataSemaphore, portMAX_DELAY) == pdTRUE) {
        joystickValueL = ps2x.Analog(PSS_LY);  
        joystickValueR = ps2x.Analog(PSS_RY);

        padUp = ps2x.Button(PSB_PAD_UP);
        padDown = ps2x.Button(PSB_PAD_DOWN);
        padLeft = ps2x.Button(PSB_PAD_LEFT);
        padRight = ps2x.Button(PSB_PAD_RIGHT);
        
        if (padUp) padUpSpeed = ps2x.Analog(PSAB_PAD_UP);
        if (padDown) padDownSpeed = ps2x.Analog(PSAB_PAD_DOWN);
        if (padLeft) padLeftSpeed = ps2x.Analog(PSAB_PAD_LEFT);
        if (padRight) padRightSpeed = ps2x.Analog(PSAB_PAD_RIGHT);
        
        xSemaphoreGive(xDataSemaphore);
      }
    }
    
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void activate_profilimetr() {
  digitalWrite(ledPin, HIGH);
  digitalWrite(ledPin2, HIGH);
  digitalWrite(ledPin3, HIGH);
  digitalWrite(ledPin4, HIGH);
}

void deactivate_profilimetr() {
  digitalWrite(ledPin, LOW);
  digitalWrite(ledPin2, LOW);
  digitalWrite(ledPin3, LOW);
  digitalWrite(ledPin4, LOW);
}

void task_cross() {
  if (ps2x.Button(PSB_CROSS)) {
    if (!crossWasPressed) {
      crossWasPressed = true;
      Serial.println("cross");
      state = 0;
      Serial.print("current state");
      Serial.print(state);
    }
  } else {
    crossWasPressed = false;
  }
}

void task_circle() {
  if (ps2x.Button(PSB_CIRCLE)) {
    if (!circleWasPressed) {
      circleWasPressed = true;
      Serial.println("circle");
      autonomousMode = false;
      leftMotor.forward(0);
      rightMotor.forward(0);
      deactivate_profilimetr();
      Serial.print("current mode auto: false");
    }
  } else {
    circleWasPressed = false;
  }
}

// 1. Нажали кнопку
// 2. Ждём 2 секунды
// 3. Включаем профилемер (лазеры)
// 4. Едем 160 см.

void auto_task() {
    switch(state) {
    case 0: // получить время
      if (autonomousMode == false) {
        return;
      }
      Serial.print("state0");
      timer1 = millis();
      state = 1;
      break;
    case 1: // выполнить движение
      if (autonomousMode == false) {
        return;
      }
      Serial.print("state1");
      if (nonblockdelay(2000, timer1)) {
        activate_profilimetr();
        leftMotor.forward(255);
        rightMotor.forward(255);
        timer2 = millis();
        state = 2;
      }
      break;
    case 2: // остановка
      if (autonomousMode == false) {
        return;
      }
        Serial.print("state2");
      if (nonblockdelay(5000, timer2)) {
        deactivate_profilimetr();
        leftMotor.forward(0);
        rightMotor.forward(0);
        state = 3;
      }
      break;
  }
}

bool nonblockdelay(const long interval, unsigned long &previousTime) {
  unsigned long currentMillis = millis();
  if (currentMillis - previousTime >= interval) {
    previousTime = currentMillis;
    return true;
  }
  return false;
}

void setup() {
  pinMode(BT1, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);
  pinMode(ledPin4, OUTPUT);
  
  pinMode(R_UP, OUTPUT);
  pinMode(L_UP, OUTPUT);
  pinMode(L_GND, OUTPUT);
  pinMode(R_GND, OUTPUT);
  
  Serial.begin(9600);
  
  while (error != 0) {
    delay(1000);
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
    
    Serial.print("# Попытка конфигурации: ");
    Serial.println(tryNum);
    tryNum++;
  }
  
  Serial.print("Analog(1): ");
  Serial.println(ps2x.Analog(1), HEX);
  
  type = ps2x.readType();
  
  switch (type) {
    case 0:
      Serial.println("Обнаружен неизвестный контроллер");
      break;
      
    case 1:
      Serial.println("Обнаружен контроллер DualShock");
      break;
      
    case 2:
      Serial.println("Обнаружен контроллер GuitarHero");
      break;
      
    case 3:
      Serial.println("Обнаружен беспроводной контроллер Sony DualShock");
      break;
  }
  
  // семафор
  xDataSemaphore = xSemaphoreCreateMutex();
  // Пока реализация на задачах
  xTaskCreatePinnedToCore(
    joystickTask,    // Функция задачи
    "JoystickTask",  // Имя задачи
    10000,           // Размер стека
    NULL,            // Параметры
    1,               // Приоритет
    NULL,            // Handle задачи
    0                // Ядро (0 или 1)
  );

  Serial.println("Задача джойстика создана");
}

void loop() {
  task_cross();
  task_circle();
  int buttonState = digitalRead(BT1);

  if (buttonState == LOW and ) {
    autonomousMode = true;
    auto_task();
  } else {
    Serial.println("Auto Off");
  }

  if (type == 1) {
    Serial.print("JOY:");
    Serial.println(joystickValueL);
    if (xSemaphoreTake(xDataSemaphore, portMAX_DELAY) == pdTRUE) {
      if (joystickValueL==128) {
      }
      else if (joystickValueL > 127) {
        leftMotor.backward(abs(128 - joystickValueL) * 2 + 1);
      } else {
        leftMotor.forward((127 - joystickValueL) * 2 + 1);
      }

      // Управление правым мотором через правый джойстик
      if (joystickValueR==128) {
      }
      else if (joystickValueR > 127) {
        rightMotor.backward(abs(128 - joystickValueR) * 2 + 1);
      } else {
        rightMotor.forward((127 - joystickValueR) * 2 + 1);
      }
      
      // Управление через кнопки крестовины (приоритет над джойстиками)
      if (padUp) { // движение вперед
        leftMotor.forward(padUpSpeed);
        rightMotor.forward(padUpSpeed);
      }
      
      if (padRight) { // поворот направо
        leftMotor.forward(padRightSpeed);
        rightMotor.backward(padRightSpeed);
      }
      
      if (padLeft) { // поворот налево
        leftMotor.backward(padLeftSpeed);
        rightMotor.forward(padLeftSpeed);
      }
      
      if (padDown) { // движение назад
        leftMotor.backward(padDownSpeed);
        rightMotor.backward(padDownSpeed);
      }
      xSemaphoreGive(xDataSemaphore);
    }
  }
  delay(10);
}

