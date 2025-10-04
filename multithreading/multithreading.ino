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

// <------------------>
uint8_t timestamp1 = 250;
uint8_t timestamp2 = 1500;

// <------------------>
PS2X ps2x;

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
  
  // Движение вперед
  void forward(int speed) {
    speed = constrain(speed, 0, 255);
    analogWrite(pinUp, speed);
    analogWrite(pinGnd, 0);
  }
  
  // Движение назад
  void backward(int speed) {
    speed = constrain(speed, 0, 255);
    analogWrite(pinGnd, speed);
    analogWrite(pinUp, 0);
  }
  
  // Остановка
  void stop() {
    analogWrite(pinUp, 0);
    analogWrite(pinGnd, 0);
  }
};

Motor leftMotor(L_UP, L_GND);
Motor rightMotor(R_UP, R_GND);

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
}

void loop() {
  boolean button1IsUp = digitalRead(BT1);
  
  // Читаем состояние кнопки
  int buttonState = digitalRead(BT1);
  
  // Если кнопка нажата (INPUT_PULLUP - нажатие дает LOW)
  if (buttonState == LOW) {
    digitalWrite(ledPin, HIGH);  // Включаем светодиод
    delay(1000);
    Serial.println("Кнопка нажата - светодиод включен");
  } else {
    digitalWrite(ledPin, LOW);   // Выключаем светодиод
  }
  
  button1WasUp = button1IsUp;

  if (type == 1) {
    ps2x.read_gamepad(false, vibrate);
    int valueL = ps2x.Analog(PSS_LY);  
    int valueR = ps2x.Analog(PSS_RY);

    if (valueL > 127) {
      leftMotor.forward(abs(128 - valueL) * 2 + 1);
    } else {
      leftMotor.backward((127 - valueL) * 2 + 1);
    }

    if (valueR > 127) {
      rightMotor.forward(abs(128 - valueR) * 2 + 1);
    } else {
      rightMotor.backward((127 - valueR) * 2 + 1);
    }
    
    if (ps2x.Button(PSB_PAD_UP)) { // движение вперед
      int speed = ps2x.Analog(PSAB_PAD_UP);
      Serial.println("вперед");
      leftMotor.forward(speed);
      rightMotor.forward(speed);
    }
    
    if (ps2x.Button(PSB_PAD_RIGHT)) { // поворот направо
      int speed = ps2x.Analog(PSAB_PAD_RIGHT);
      Serial.println("направо");
      leftMotor.forward(speed);
      rightMotor.backward(speed);
    }
    
    if (ps2x.Button(PSB_PAD_LEFT)) { // поворот налево
      int speed = ps2x.Analog(PSAB_PAD_LEFT);
      Serial.println("налево");
      leftMotor.backward(speed);
      rightMotor.forward(speed);
    }
    
    if (ps2x.Button(PSB_PAD_DOWN)) { // движение назад
      int speed = ps2x.Analog(PSAB_PAD_DOWN);
      Serial.println("назад");
      leftMotor.backward(speed);
      rightMotor.backward(speed);
    }
  }
  
  delay(100); // Задержка для стабильной работы
}