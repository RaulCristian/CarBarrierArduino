
#include <Servo.h>
#include <math.h>       /* fmod */

// Pin definitions
const int redPin = 5;
const int greenPin = 3;
const int bluePin = 4;
const int buttonPin = 2;
const int servoPin = 9;
const int trigPin = 7;
const int echoPin = 6;

// Variable to store the current LED color state
int colorState = 0; // 0 = Red, 1 = Yellow, 2 = Green

// variables will change:
int angle = 100;          // the current angle of servo motor
volatile bool buttonState = LOW;

void myISR() {
  buttonState = !buttonState;
}

// Create a Servo object
Servo servoMotor;

// Servo motor state variable
int servoMotorState = 0; // 0 - down, 1 - moving up/down, 2 - up

// Variables for ultrasonic sensor
long duration;
int distance;

unsigned long objectDetectedTime = 0;

void setup()
{
  // Set the LED pins as outputs
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // Set the button pin as input with internal pull-up resistor
  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(buttonPin), myISR, FALLING);  // trigger when button pressed, but not when released.

  // Attach the servo to the servo pin
  servoMotor.attach(servoPin);
  servoMotor.write(angle);

  // Initialize ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Initialize Serial communication
  Serial.begin(9600);
}

void turn_on_red() {
  // Turn on the red LED
  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, HIGH);
  digitalWrite(bluePin, HIGH);
  // Serial.println("RED!");
}

void turn_on_yellow() {
  // Turn on the yellow LED
  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, LOW);
  digitalWrite(bluePin, HIGH);
  // Serial.println("YELLOW!");
}

void turn_on_green() {
  // Turn on the green LED
  digitalWrite(redPin, HIGH);
  digitalWrite(greenPin, LOW);
  digitalWrite(bluePin, HIGH);
  // Serial.println("GREEN!");
}

void blink_yellow(int duration) {
  unsigned long startTime = millis();
  unsigned long endTime = startTime + duration;

   while (millis() < endTime) {
    turn_on_yellow();
    delay(500); // LED on for 500ms
    digitalWrite(redPin, HIGH);
    digitalWrite(greenPin, HIGH);
    digitalWrite(bluePin, HIGH);
    delay(500); // LED off for 500ms
  }
}

void turn_off_led() {
  // Turn off the LED
  digitalWrite(redPin, HIGH);
  digitalWrite(greenPin, HIGH);
  digitalWrite(bluePin, HIGH);
}

bool checkObjectDetected() {
  // Trigger the ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the duration of the echo pulse
  duration = pulseIn(echoPin, HIGH);

  // Calculate the distance based on the duration
  distance = duration * 0.034 / 2;

  // Check if an object is within a certain distance threshold
  if (distance < 20) { // Adjust this threshold as needed
    Serial.println("Object detected!");
    objectDetectedTime = millis();
    return true; // Object detected
  } else {
    return false; // No object detected
  }
}

void down_servo() {
  float pre_pos = 0.0;
  bool objectDetected = false;

  for (float pos = 0.0; pos <= 100.0; pos += 0.01) {

    if (fmod(pos, 15) >= 0.0 && fmod(pos, 15) <= 1.0) {
      turn_on_yellow();
    } else {
      turn_off_led();
    }
  
    if (!objectDetected) {
      servoMotor.write(pos);
      pre_pos = pos;
    } else {
      pos -= 0.01;
    }
  }
}

void lift_servo() {
  for (float pos = 100.0; pos >= 0.0; pos -= 0.01) {
    if (fmod(pos, 15) >= 0.0 && fmod(pos, 15) <= 1.0) {
      turn_on_yellow();
    } else {
      turn_off_led();
    }
    servoMotor.write(pos);

  }
}


void loop()
{

  // Check if the button is pressed
  if (buttonState)
  {
    // Button is pressed, change the LED color state
    colorState = (colorState + 1) % 3;

    Serial.println("Button pressed!");

    // Check if an object is detected by the ultrasonic sensor
    bool objectDetected = checkObjectDetected();

    // change angle of servo motor
    if(angle == 0 && !objectDetected) {
      down_servo();
      angle = 100;
    } else if(angle == 100) {
      lift_servo();
      angle = 0;
    }

    delay(200); // Debounce delay to avoid multiple rapid button presses
    buttonState = LOW;
  }
  
  // If servo is up and the sensor see an object wait until the car is gone then count down to 2 seconds then close the barrier
  bool objectDetected = checkObjectDetected();
  if (angle == 0 && objectDetected) {
    Serial.println("I see car");
    Serial.println((millis() - objectDetectedTime));

    while(objectDetected) {
      objectDetected = checkObjectDetected();
    }

    while (millis() - objectDetectedTime <= 3000) {
    }

    objectDetected = checkObjectDetected();
    if (millis() - objectDetectedTime >= 3000 && !objectDetected) {
      down_servo();
      angle = 100;
    }
  }

  // Get the current servo motor position
  int servoPosition = servoMotor.read();

  if (servoPosition >= 98 && servoPosition <= 100) {
      turn_on_red();
  } else if (servoPosition >= 0 && servoPosition <= 2) {
      turn_on_green();
  } 

}



