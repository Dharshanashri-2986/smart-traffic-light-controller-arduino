/*
 * Project: Smart Traffic Light Controller with Pedestrian Crossing
 *
 * Description:
 * An intelligent traffic signal system controlling vehicle and pedestrian lights.
 * Pedestrian crossing priority is dynamically adjusted using an ultrasonic sensor.
 *
 * Components Required:
 * Hardware: Arduino Uno, LEDs, resistors, pushbutton, piezo buzzer, 7-segment display, HC-SR04 ultrasonic sensor.
 * Software: Arduino IDE, Wokwi simulator, standard Arduino libraries (no extra libraries required).
 *
 * Expected Outcome:
 * Vehicle lights follow a proper Green → Yellow → Red sequence.
 * Pedestrian requests trigger green light and buzzer countdown dynamically.
 *
 * Individual Contributions:
 * 1) Tejasvi Senka – Vehicle Traffic Light Control
 *    • Manage vehicle LEDs and state sequence.
 *    • Implement timing and state transitions (VEHICLE_GO, VEHICLE_SLOW, VEHICLE_STOP).
 *
 * 2) Priyadharshika – Pedestrian Signals & Button
 *    • Control pedestrian LEDs and button behavior.
 *    • Queue requests and calculate pedestrian demand using ultrasonic sensor.
 *
 * 3) Dharshanashri – Buzzer & 7-Segment Display
 *    • Implement buzzer alerts for pedestrian crossing.
 *    • Control 7-segment countdown display synced with buzzer and pedestrian green.
 *
 * 4) Geethika – Ultrasonic Sensor & System Integration
 *    • Handle ultrasonic distance measurement for pedestrian priority.
 *    • Integrate vehicle lights, pedestrian lights, buzzer, and 7-segment display.
 */

#include <Arduino.h>

#define VEHICLE_RED_PIN    10
#define VEHICLE_YELLOW_PIN 9
#define VEHICLE_GREEN_PIN  8
#define PED_RED_PIN        7
#define PED_GREEN_PIN      6
#define BUTTON_PIN         2
#define BUZZER_PIN         4
#define SEG_A_PIN          A1
#define SEG_B_PIN          A2
#define SEG_C_PIN          A3
#define SEG_D_PIN          A4
#define SEG_E_PIN          A5
#define SEG_F_PIN          12
#define SEG_G_PIN          13
#define VEHICLE_RED_PIN    10
#define VEHICLE_YELLOW_PIN 9
#define SEG_G_PIN          13
#define TRIG_PIN 5
#define ECHO_PIN 3


#define MIN_VEHICLE_GREEN_TIME 5000
#define VEHICLE_GREEN_TIME     10000
#define VEHICLE_RED_TIME       5000
#define YELLOW_DELAY           3000
#define PEDESTRIAN_CROSS_TIME  9000
#define ALL_RED_DELAY          1000

typedef enum {
  VEHICLE_GO,
  VEHICLE_SLOW,
  VEHICLE_STOP,
  PEDESTRIAN_GO
} SystemState;

typedef struct {
  SystemState currentState;
  SystemState nextStateAfterYellow;
  int buttonPressedFlag;
  unsigned long stateChangeTime;
} TrafficController;

TrafficController controller;
unsigned long lastBeepTime = 0;
int beepState = LOW;

unsigned long lastFlashTime = 0;
int flashState = HIGH;

const byte sevenSegDigits[10][7] = {
  { 1,1,1,1,1,1,0 }, { 0,1,1,0,0,0,0 }, { 1,1,0,1,1,0,1 }, { 1,1,1,1,0,0,1 }, { 0,1,1,0,0,1,1 },
  { 1,0,1,1,0,1,1 }, { 1,0,1,1,1,1,1 }, { 1,1,1,0,0,0,0 }, { 1,1,1,1,1,1,1 }, { 1,1,1,1,0,1,1 }
};

const int segmentPins[] = {SEG_A_PIN, SEG_B_PIN, SEG_C_PIN, SEG_D_PIN, SEG_E_PIN, SEG_F_PIN, SEG_G_PIN};


int pedestrianDemand = 0; 
long getPedestrianDistance() {
  
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);

  return duration * 0.034 / 2;
}

void displayDigit(int digit) {
  if (digit < 0 || digit > 9) return;
  for (int i = 0; i < 7; i++) {
    digitalWrite(segmentPins[i], sevenSegDigits[digit][i] ? LOW : HIGH);
  }
}

void displayOff() {
  for (int i = 0; i < 7; i++) digitalWrite(segmentPins[i], HIGH); 
}


void trafficLightsOff() {
  digitalWrite(VEHICLE_RED_PIN, LOW);
  digitalWrite(VEHICLE_YELLOW_PIN, LOW);
  digitalWrite(VEHICLE_GREEN_PIN, LOW);
  digitalWrite(PED_RED_PIN, LOW);
  digitalWrite(PED_GREEN_PIN, LOW);
  noTone(BUZZER_PIN);
}

void checkButton(TrafficController *ctrl) {
  if (digitalRead(BUTTON_PIN) == LOW) {
    if ((ctrl->currentState == VEHICLE_GO || ctrl->currentState == VEHICLE_STOP || ctrl->currentState == VEHICLE_SLOW) && !ctrl->buttonPressedFlag) {
      ctrl->buttonPressedFlag = 1;
      tone(BUZZER_PIN, 1500, 100);
      Serial.println("Pedestrian request queued!");
      delay(100);

      long distance = getPedestrianDistance();
      Serial.print("Distance measured: ");
      Serial.print(distance);
      Serial.println(" cm");

  
      if (distance < 50) { 
        pedestrianDemand = 3;
      } else if (distance < 150) { 
        pedestrianDemand = 2;
      } else { 
        pedestrianDemand = 1;
      }
      Serial.print("Pedestrian demand set to: ");
      Serial.println(pedestrianDemand);
    }
  }
}

void runPedestrianCountdown(TrafficController *ctrl) {
  
  unsigned long dynamicCrossTime = 5000 + (pedestrianDemand * 3000);
  unsigned long elapsedTime = millis() - ctrl->stateChangeTime;

  if (elapsedTime < dynamicCrossTime) {
    int secondsLeft = (dynamicCrossTime - elapsedTime) / 1000;
    displayDigit(secondsLeft);

    if (secondsLeft < 4) {
      if (millis() - lastBeepTime > 200) {
        lastBeepTime = millis();
        beepState = !beepState;

        if (beepState) {
          tone(BUZZER_PIN, 2000);
          digitalWrite(PED_GREEN_PIN, HIGH);
        } else {
          noTone(BUZZER_PIN);
          digitalWrite(PED_GREEN_PIN, LOW);
        }
      }
    } else {
      
      digitalWrite(PED_GREEN_PIN, HIGH);
      if (millis() - lastBeepTime > 500) {
        lastBeepTime = millis();
        beepState = !beepState;

        if (beepState) {
          tone(BUZZER_PIN, 1000);
        } else {
          noTone(BUZZER_PIN);
        }
      }
    }
  } else {
    
    noTone(BUZZER_PIN);
    displayOff();
    trafficLightsOff();
    digitalWrite(VEHICLE_RED_PIN, HIGH);
    digitalWrite(PED_RED_PIN, HIGH);
    delay(ALL_RED_DELAY);

    trafficLightsOff();
    digitalWrite(VEHICLE_GREEN_PIN, HIGH);
    digitalWrite(PED_RED_PIN, HIGH);

    controller.currentState = VEHICLE_GO;
    controller.stateChangeTime = millis();
    controller.buttonPressedFlag = 0;
    pedestrianDemand = 0; 
    Serial.println("Crossing finished. Returning to normal traffic flow.");
  }
}


void setup() {
  Serial.begin(9600);
  Serial.println("Smart Traffic Controller Initializing...");

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(VEHICLE_RED_PIN, OUTPUT);
  pinMode(VEHICLE_YELLOW_PIN, OUTPUT);
  pinMode(VEHICLE_GREEN_PIN, OUTPUT);
  pinMode(PED_RED_PIN, OUTPUT);
  pinMode(PED_GREEN_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  for (int i=0; i<7; i++) pinMode(segmentPins[i], OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  trafficLightsOff();
  displayOff();
  digitalWrite(VEHICLE_GREEN_PIN, HIGH);
  digitalWrite(PED_RED_PIN, HIGH);
  controller.currentState = VEHICLE_GO;
  controller.buttonPressedFlag = 0;
  controller.stateChangeTime = millis();
  Serial.println("Initialization Complete. System running.");
}


void loop() {
  checkButton(&controller);

  switch (controller.currentState) {
    case VEHICLE_GO:
      if (controller.buttonPressedFlag && (millis() - controller.stateChangeTime >= MIN_VEHICLE_GREEN_TIME)) {
        controller.nextStateAfterYellow = PEDESTRIAN_GO;
        controller.currentState = VEHICLE_SLOW;
        controller.stateChangeTime = millis();
        trafficLightsOff();
        digitalWrite(VEHICLE_YELLOW_PIN, HIGH);
        digitalWrite(PED_RED_PIN, HIGH);
      } else if (millis() - controller.stateChangeTime >= VEHICLE_GREEN_TIME) {
        controller.nextStateAfterYellow = VEHICLE_STOP;
        controller.currentState = VEHICLE_SLOW;
        controller.stateChangeTime = millis();
        trafficLightsOff();
        digitalWrite(VEHICLE_YELLOW_PIN, HIGH);
        digitalWrite(PED_RED_PIN, HIGH);
      }
      break;

    case VEHICLE_SLOW:
      if (millis() - controller.stateChangeTime >= YELLOW_DELAY) {
        if (controller.buttonPressedFlag) {
          controller.currentState = PEDESTRIAN_GO;
          trafficLightsOff();
          digitalWrite(VEHICLE_RED_PIN, HIGH);
          digitalWrite(PED_GREEN_PIN, HIGH);
        } 
        else {
          controller.currentState = VEHICLE_STOP;
          trafficLightsOff();
          digitalWrite(VEHICLE_RED_PIN, HIGH);
          digitalWrite(PED_RED_PIN, HIGH);
        }
        controller.stateChangeTime = millis();
      }
      break;

    case VEHICLE_STOP:
      if (millis() - controller.stateChangeTime >= VEHICLE_RED_TIME) {
        if (controller.buttonPressedFlag) {
          controller.currentState = PEDESTRIAN_GO;
          trafficLightsOff();
          digitalWrite(VEHICLE_RED_PIN, HIGH);
          digitalWrite(PED_GREEN_PIN, HIGH);
        } 
        else {
          controller.currentState = VEHICLE_GO;
          trafficLightsOff();
          digitalWrite(VEHICLE_GREEN_PIN, HIGH);
          digitalWrite(PED_RED_PIN, HIGH);
        }
        controller.stateChangeTime = millis(); 
      }
      break;

    case PEDESTRIAN_GO:
      runPedestrianCountdown(&controller);
      break;
  }
}