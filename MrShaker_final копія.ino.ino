#include <AccelStepper.h>
#include <Servo.h>
#include "HX711.h"
#include <avr/wdt.h>

HX711 scale;
uint8_t dataPin = 30;
uint8_t clockPin = 31;

#define LIMIT_SWITCH_PIN1 2
#define LIMIT_SWITCH_PIN2 3
#define LIMIT_SWITCH_PIN 46
// Піни двигунів
#define DIR_PIN1 16
#define STEP_PIN1 17
#define DIR_PIN2 18
#define STEP_PIN2 19
#define DIR_PIN3 20
#define STEP_PIN3 21
#define DIR_PIN4 22
#define STEP_PIN4 23
#define DIR_PIN5 24
#define STEP_PIN5 25
#define DIR_PIN6 26
#define STEP_PIN6 27
#define DIR_PIN7 28
#define STEP_PIN7 29

#define POMP_PIN 6
#define SHAKERR_PIN1 8
#define SHAKERR_PIN2 9
#define SERVO_PIN 11
#define DOOR_SERVO_PIN_1 5
#define DOOR_SERVO_PIN_2 7
#define GRAY_WATER_SERVO_PIN 4

#define MILK_WEIGHT 25

#define SHAKE_POSITION -20500

AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN1, DIR_PIN1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN2, DIR_PIN2);
AccelStepper stepper3(AccelStepper::DRIVER, STEP_PIN3, DIR_PIN3);
AccelStepper stepper4(AccelStepper::DRIVER, STEP_PIN4, DIR_PIN4);
AccelStepper stepper5(AccelStepper::DRIVER, STEP_PIN5, DIR_PIN5);
AccelStepper stepper6(AccelStepper::DRIVER, STEP_PIN6, DIR_PIN6);
AccelStepper stepper7(AccelStepper::DRIVER, STEP_PIN7, DIR_PIN7);
int stepDelay = 25;
Servo myservo;
Servo door_servo_1;
Servo door_servo_2;
Servo gray_water_servo;

void setup() {
  Serial.begin(9600);
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_PIN1, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_PIN2, INPUT_PULLUP);
  pinMode(SHAKERR_PIN1, OUTPUT);
  pinMode(SHAKERR_PIN2, OUTPUT);
  pinMode(POMP_PIN, OUTPUT);

  myservo.attach(SERVO_PIN);
  myservo.write(0);

  door_servo_1.attach(DOOR_SERVO_PIN_1);

  door_servo_1.write(125);

  door_servo_2.attach(DOOR_SERVO_PIN_2);

  door_servo_2.write(60);

  gray_water_servo.attach(GRAY_WATER_SERVO_PIN);
  gray_water_servo.write(0);

  stepper1.setMaxSpeed(8000);
  stepper1.setAcceleration(10000);
  stepper2.setMaxSpeed(8000);
  stepper2.setAcceleration(10000);
  stepper3.setMaxSpeed(8000);
  stepper3.setAcceleration(10000);
  stepper4.setMaxSpeed(7000);
  stepper4.setAcceleration(10000);
  stepper5.setMaxSpeed(10000);
  stepper5.setAcceleration(20000);
  stepper6.setMaxSpeed(80000);
  stepper6.setAcceleration(5000);
  stepper7.setMaxSpeed(40000);
  stepper7.setAcceleration(5000);

  scale.begin(dataPin, clockPin);
  scale.set_offset(-301296);
  scale.set_scale(425.748565);
  findZeroPositionStepper7();
  findZeroPosition(); 
  
}

// Гомінг stepper7
void findZeroPositionStepper7() {
  while (digitalRead(LIMIT_SWITCH_PIN2) == HIGH) {
    stepper7.setSpeed(-3200);
    stepper7.runSpeed();
  }
  stepper7.stop();
  stepper7.setCurrentPosition(0);
}
void goToPositionWithEndstop(AccelStepper &stepper, long targetPosition) {
  // 1. Їдемо швидко на задану позицію (наприклад, -19000)
  stepper.moveTo(targetPosition);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }

  // 2. Потім повільно рухаємось поки кінцевик не спрацює
  stepper.setMaxSpeed(2000);       // повільна швидкість, налаштуй під себе

  // Рух у напрямку, де кінцевик замкнеться
  while (digitalRead(LIMIT_SWITCH_PIN) == HIGH) {
    stepper.move(-10);    // рухаємося вперед по 1 кроку
    stepper.run();
  }

  // 3. Коли кінцевик спрацював — зупиняємось і встановлюємо позицію як нову нульову
  stepper.setCurrentPosition(-20500);
}

void pourWater(int grams, int stepper7EndPos) {
  stepper7.moveTo(15000);
  while (stepper7.distanceToGo() != 0) {
    stepper7.run();
  }
  scale.tare();
  while (scale.get_units(5) < grams) {
    digitalWrite(POMP_PIN, HIGH);
    delay(1000);
    digitalWrite(POMP_PIN, LOW);
  }
  stepper7.moveTo(stepper7EndPos);
  while (stepper7.distanceToGo() != 0) {
    stepper7.run();
  }
}

void findZeroPosition() {
  while (digitalRead(LIMIT_SWITCH_PIN1) == HIGH) {
    stepper6.setSpeed(2000);
    stepper6.runSpeed();
  }
  stepper6.stop();
  stepper6.setCurrentPosition(0);
}
void manualMoveCommand(String cmd) {
  int space1 = cmd.indexOf(' ');
  int space2 = cmd.indexOf(' ', space1 + 1);
  if (space2 == -1) {
    return;
  }

  int motorNum = cmd.substring(space1 + 1, space2).toInt();
  int steps = cmd.substring(space2 + 1).toInt();

  AccelStepper* motor = nullptr;
  switch (motorNum) {
    case 1: motor = &stepper1; break;
    case 2: motor = &stepper2; break;
    case 3: motor = &stepper3; break;
    case 4: motor = &stepper4; break;
    case 5: motor = &stepper5; break;
    case 6: motor = &stepper6; break;
    case 7: motor = &stepper7; break;
    default:
      return;
  }

  motor->move(steps);
  while (motor->distanceToGo() != 0) {
    motor->run();
  }
}
void manualMoveSpeed(String cmd) {
  int space1 = cmd.indexOf(' ');
  int space2 = cmd.indexOf(' ', space1 + 1);
  if (space2 == -1) {

    return;
  }

  int speed = cmd.substring(space1 + 1, space2).toInt();
  int time = cmd.substring(space2 + 1).toInt() * 1000;

  if (speed < 0 || speed > 255) {
    return;
  }

  analogWrite(SHAKERR_PIN2, speed);
  delay(time);
  analogWrite(SHAKERR_PIN2, 0);
}
void giveCup() {
  stepper6.moveTo(0);
  while (stepper6.distanceToGo() != 0) {
    stepper6.run();
  }
  myservo.write(180);
  delay(500);
  myservo.write(0);
  delay(500);
}
void homeSteppers() {
  stepper6.moveTo(0);
  while (stepper6.distanceToGo() != 0) {
    stepper6.run();
  }
  stepper7.moveTo(0);
  while (stepper7.distanceToGo() != 0) {
    stepper7.run();
  }
}
void shake(int seconds, int power, int stepper7StartPos, int stepper7EndPos) {
  stepper7.moveTo(stepper7StartPos);
  while (stepper7.distanceToGo() != 0) {
    stepper7.run();
  }
  if (power < 250) {
    analogWrite(SHAKERR_PIN2, power);
    delay(seconds * 1000);
    analogWrite(SHAKERR_PIN2, 0);
  } else {
    digitalWrite(SHAKERR_PIN2, HIGH);
    delay(seconds * 1000);
    digitalWrite(SHAKERR_PIN2, LOW);
  }

  stepper7.moveTo(stepper7EndPos);
  while (stepper7.distanceToGo() != 0) {
    stepper7.run();
  }
}
void waitUntilCupTaken() {
  Serial.println("Очікую, поки заберуть коктейль...");
  delay(2000);  // стабілізація

  int count = 0;

  while (true) {
    long weight = scale.get_units(5);
    Serial.print("Вага: ");
    Serial.println(weight);

    if (weight < -3) { // якщо вага стала від’ємною
      count++;
    } else {
      count = 0;
    }

    if (count >= 4) { // 4 поспіль рази від’ємна вага
      Serial.println("Коктейль забрано. Закриваю двері.");
      break;
    }

    delay(500);
  }
}
void cook() {
  goToPositionWithEndstop(stepper6, -19000);
  pourWater(130, 27000);
  shake(20, 90, 27500, 28500);
  shake(20, 90, 28500, 30500);
  shake(20, 90, 30500, 25000);
  pourWater(120, 25000);
  shake(5, 68, 25000, 0);
  findZeroPositionStepper7();
  stepper6.moveTo(-33000);

  while (stepper6.distanceToGo() != 0) {
    stepper6.run();
  }
  openDoor();
  waitUntilCupTaken();  
  closeDoor();
  findZeroPositionStepper7();
  findZeroPosition();
  stepper6.moveTo(-10000);

  while (stepper6.distanceToGo() != 0) {
    stepper6.run();
  }
  wdt_enable(WDTO_15MS); // Вмикаємо watchdog на 15 мс
  while (1);
}

void doseMilk(int weigth){
  stepper6.moveTo(-8000);
  while (stepper6.distanceToGo() != 0) {
    stepper6.run();
  }
  scale.tare();
  while (scale.get_units(5) < weigth) {


    stepper2.move(60);
    stepper2.runToPosition();

    delayMicroseconds(stepDelay);

    stepper2.move(-1);
    stepper2.runToPosition();

    delayMicroseconds(stepDelay);
  }
  stepper2.move(-120);
  stepper2.runToPosition();
}

void coctailCode101() {
  homeSteppers();
  giveCup();

  stepper6.moveTo(-11000);
  while (stepper6.distanceToGo() != 0) {
    stepper6.run();
  }
  //насипаєм протеїн
  scale.tare();
  while (scale.get_units(5) < 30) {

    for (int i = 0; i < 500; i++) {
      stepper1.move(-3);
      stepper1.runToPosition();

      delayMicroseconds(stepDelay);

      stepper1.move(1);
      stepper1.runToPosition();

      delayMicroseconds(stepDelay);
    }
  }

  cook();
}
void coctailCode102() {
  homeSteppers();
  giveCup();
  stepper6.moveTo(-17000);
  while (stepper6.distanceToGo() != 0) {
    stepper6.run();
  }
  //насипаєм протеїн
  scale.tare();
  while (scale.get_units(5) < 30) {

    for (int i = 0; i < 500; i++) {
      stepper3.move(-3);
      stepper3.runToPosition();

      delayMicroseconds(stepDelay);

      stepper3.move(1);
      stepper3.runToPosition();

      delayMicroseconds(stepDelay);
    }
  }

  cook();
}
void coctailCode103() {
  homeSteppers();
  giveCup();
  stepper6.moveTo(-14500);
  while (stepper6.distanceToGo() != 0) {
    stepper6.run();
  }
  //насипаєм протеїн
  scale.tare();
  while (scale.get_units(5) < 30) {

    for (int i = 0; i < 500; i++) {
      stepper4.move(3);
      stepper4.runToPosition();

      delayMicroseconds(stepDelay);

      stepper4.move(-1);
      stepper4.runToPosition();

      delayMicroseconds(stepDelay);
    }
  }
  cook();
}
void coctailCode104() {
  homeSteppers();
  giveCup();
  stepper6.moveTo(-5200);
  while (stepper6.distanceToGo() != 0) {
    stepper6.run();
  }
  //насипаєм протеїн
  scale.tare();
  while (scale.get_units(3) < 30) {

    for (int i = 0; i < 500; i++) {
      stepper5.move(3);
      stepper5.runToPosition();

      delayMicroseconds(stepDelay);

      stepper5.move(-1);
      stepper5.runToPosition();

      delayMicroseconds(stepDelay);
    }
  }

  cook();
}
void coctailCode201() {

  homeSteppers();
  giveCup();
  stepper6.moveTo(-11000);
  while (stepper6.distanceToGo() != 0) {
    stepper6.run();
  }
  //насипаєм протеїн
  scale.tare();
  while (scale.get_units(5) < 30) {

    for (int i = 0; i < 500; i++) {
      stepper1.move(-3);
      stepper1.runToPosition();

      delayMicroseconds(stepDelay);

      stepper1.move(1);
      stepper1.runToPosition();

      delayMicroseconds(stepDelay);
    }
  }
  doseMilk(MILK_WEIGHT);
  cook();
}
void coctailCode202() {
  homeSteppers();
  giveCup();
  stepper6.moveTo(-17000);
  while (stepper6.distanceToGo() != 0) {
    stepper6.run();
  }
  //насипаєм протеїн
  scale.tare();
  while (scale.get_units(5) < 30) {

    for (int i = 0; i < 500; i++) {
      stepper3.move(-5);
      stepper3.runToPosition();

      delayMicroseconds(stepDelay);

      stepper3.move(1);
      stepper3.runToPosition();

      delayMicroseconds(stepDelay);
    }
  }
  doseMilk(MILK_WEIGHT);
  cook();
}
void coctailCode203() {

  homeSteppers();
  giveCup();
  stepper6.moveTo(-14500);
  while (stepper6.distanceToGo() != 0) {
    stepper6.run();
  }
  //насипаєм протеїн
  scale.tare();
  while (scale.get_units(10) < 30) {

    for (int i = 0; i < 500; i++) {
      stepper4.move(3);
      stepper4.runToPosition();

      delayMicroseconds(stepDelay);

      stepper4.move(-1);
      stepper4.runToPosition();

      delayMicroseconds(stepDelay);
    }
  }
  doseMilk(MILK_WEIGHT);

  cook();
}
void coctailCode204() {

  homeSteppers();
  giveCup();
  stepper6.moveTo(-5400);
  while (stepper6.distanceToGo() != 0) {
    stepper6.run();
  }
  //насипаєм протеїн
  scale.tare();
  while (scale.get_units(5) < 30) {

    for (int i = 0; i < 500; i++) {
      stepper5.move(7);
      stepper5.runToPosition();

      delayMicroseconds(stepDelay);

      stepper5.move(-1);
      stepper5.runToPosition();

      delayMicroseconds(stepDelay);
    }
  }
  doseMilk(MILK_WEIGHT);
  cook();
}
void openDoor() {
  door_servo_1.write(0);
  door_servo_2.write(180);
}
void closeDoor() {
  door_servo_2.write(40);
  door_servo_1.write(125);
  delay(1000);
  door_servo_2.write(60);
}

void loop() {
  static String inputString = "";

  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n' || inChar == '\r') {
      inputString.trim();

      if (inputString.length() > 0) {
        if (inputString.startsWith("m ")) {
          manualMoveCommand(inputString);
        } else if (inputString.startsWith("s ")) {
          manualMoveSpeed(inputString);
        } else if (inputString == "zero7") {
          findZeroPositionStepper7();
        } else if (inputString == "zero6") {
          findZeroPosition();
        } else {
          int coctailCode = inputString.toInt();
          Serial.print("Received coctailCode: ");
          Serial.println(coctailCode);

          switch (coctailCode) {
            case 101:
              coctailCode101();
              Serial.println("Done");
              break;
            case 102:
              coctailCode102();
              Serial.println("Done");
              break;
            case 103:
              coctailCode103();
              Serial.println("Done");
              break;
            case 104:
              coctailCode104();
              Serial.println("Done");
              break;
            case 201:
              coctailCode201();
              Serial.println("Done");
              break;
            case 202:
              coctailCode202();
              Serial.println("Done");
              break;
            case 203:
              coctailCode203();
              Serial.println("Done");
              break;
            case 204:
              coctailCode204();
              Serial.println("Done");
              break;
            case 123: openDoor(); break;

            case 321: closeDoor(); break;
            case 444: giveCup(); break;
            case 666: cook(); break;
            default: Serial.println("Unknown coctailCode"); break;
          }
        }

        inputString = "";
      }
    } else {
      inputString += inChar;
    }
  }
}