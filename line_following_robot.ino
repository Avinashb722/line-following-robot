#include <NewPing.h>

// Motor pins
#define LEFT_MOTOR_A 5
#define LEFT_MOTOR_B 6
#define RIGHT_MOTOR_A 9
#define RIGHT_MOTOR_B 10

// IR sensor pins
#define LEFT_IR A0
#define CENTER_IR A1
#define RIGHT_IR A2

// Ultrasonic sensor pins
#define TRIGGER_PIN 7
#define ECHO_PIN 8
#define MAX_DISTANCE 200

// Constants
#define MOTOR_SPEED 150
#define TURN_SPEED 120
#define OBSTACLE_DISTANCE 15
#define LINE_THRESHOLD 500

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

enum RobotState {
  LINE_FOLLOWING,
  OBSTACLE_AVOIDING,
  SEARCHING_LINE
};

RobotState currentState = LINE_FOLLOWING;
unsigned long lastLineTime = 0;
unsigned long searchStartTime = 0;

void setup() {
  Serial.begin(9600);
  
  // Initialize motor pins
  pinMode(LEFT_MOTOR_A, OUTPUT);
  pinMode(LEFT_MOTOR_B, OUTPUT);
  pinMode(RIGHT_MOTOR_A, OUTPUT);
  pinMode(RIGHT_MOTOR_B, OUTPUT);
  
  // Initialize IR sensor pins
  pinMode(LEFT_IR, INPUT);
  pinMode(CENTER_IR, INPUT);
  pinMode(RIGHT_IR, INPUT);
  
  delay(2000); // Startup delay
}

void loop() {
  int distance = sonar.ping_cm();
  
  if (distance > 0 && distance < OBSTACLE_DISTANCE) {
    currentState = OBSTACLE_AVOIDING;
    avoidObstacle();
  } else {
    switch (currentState) {
      case LINE_FOLLOWING:
        if (!followLine()) {
          currentState = SEARCHING_LINE;
          searchStartTime = millis();
        }
        break;
        
      case SEARCHING_LINE:
        if (searchForLine()) {
          currentState = LINE_FOLLOWING;
        } else if (millis() - searchStartTime > 3000) {
          // If no line found after 3 seconds, continue searching
          searchStartTime = millis();
        }
        break;
        
      case OBSTACLE_AVOIDING:
        currentState = SEARCHING_LINE;
        searchStartTime = millis();
        break;
    }
  }
  
  delay(50);
}

bool followLine() {
  int leftSensor = analogRead(LEFT_IR);
  int centerSensor = analogRead(CENTER_IR);
  int rightSensor = analogRead(RIGHT_IR);
  
  bool leftOnLine = leftSensor > LINE_THRESHOLD;
  bool centerOnLine = centerSensor > LINE_THRESHOLD;
  bool rightOnLine = rightSensor > LINE_THRESHOLD;
  
  if (centerOnLine) {
    moveForward();
    lastLineTime = millis();
    return true;
  } else if (leftOnLine && !rightOnLine) {
    turnLeft();
    lastLineTime = millis();
    return true;
  } else if (rightOnLine && !leftOnLine) {
    turnRight();
    lastLineTime = millis();
    return true;
  } else if (leftOnLine && rightOnLine) {
    moveForward();
    lastLineTime = millis();
    return true;
  }
  
  return false; // No line detected
}

bool searchForLine() {
  static int searchDirection = 1; // 1 for right, -1 for left
  static unsigned long lastTurn = 0;
  
  if (millis() - lastTurn > 200) {
    if (searchDirection == 1) {
      turnRight();
    } else {
      turnLeft();
    }
    lastTurn = millis();
    
    // Change direction every 1 second
    if (millis() - searchStartTime > 1000) {
      searchDirection *= -1;
      searchStartTime = millis();
    }
  }
  
  // Check if line is found
  int centerSensor = analogRead(CENTER_IR);
  if (centerSensor > LINE_THRESHOLD) {
    return true;
  }
  
  return false;
}

void avoidObstacle() {
  stopMotors();
  delay(200);
  
  // Turn right to avoid obstacle
  turnRight();
  delay(800);
  
  // Move forward
  moveForward();
  delay(1000);
  
  // Turn left to get back on track
  turnLeft();
  delay(800);
  
  // Move forward
  moveForward();
  delay(1000);
  
  // Turn left again
  turnLeft();
  delay(800);
}

void moveForward() {
  analogWrite(LEFT_MOTOR_A, MOTOR_SPEED);
  analogWrite(LEFT_MOTOR_B, 0);
  analogWrite(RIGHT_MOTOR_A, MOTOR_SPEED);
  analogWrite(RIGHT_MOTOR_B, 0);
}

void turnLeft() {
  analogWrite(LEFT_MOTOR_A, 0);
  analogWrite(LEFT_MOTOR_B, TURN_SPEED);
  analogWrite(RIGHT_MOTOR_A, TURN_SPEED);
  analogWrite(RIGHT_MOTOR_B, 0);
}

void turnRight() {
  analogWrite(LEFT_MOTOR_A, TURN_SPEED);
  analogWrite(LEFT_MOTOR_B, 0);
  analogWrite(RIGHT_MOTOR_A, 0);
  analogWrite(RIGHT_MOTOR_B, TURN_SPEED);
}

void stopMotors() {
  analogWrite(LEFT_MOTOR_A, 0);
  analogWrite(LEFT_MOTOR_B, 0);
  analogWrite(RIGHT_MOTOR_A, 0);
  analogWrite(RIGHT_MOTOR_B, 0);
}