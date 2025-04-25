#include <CytronMotorDriver.h>

const int numMotors = 4;
String array_string;
int parsed_values[numMotors];
const int mechanisms = 2;
int parsed_values2[mechanisms];
bool rackMotorRun = false;

CytronMD motor1(PWM_DIR, 5, 18);  // RIGHT motor
CytronMD motor2(PWM_DIR, 4, 0);   // LEFT motor
CytronMD motor3(PWM_DIR, 2, 15);  // LEFT motor
CytronMD motor4(PWM_DIR, 16, 17); // RIGHT motor

CytronMD rack_motor(PWM_DIR, 21, 22); // Rack motor
CytronMD seed_motor(PWM_DIR, 23, 24); // Seed motor

// Timing variables for non-blocking delays
unsigned long previousMillis = 0;
const long rackDelay = 2000; // Delay for rack motor

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);  
}

void loop() {
  if (Serial.available() > 0) {
    if (!rackMotorRun) {
    // Run the rack motor once
    runRackMotor();
    rackMotorRun = true; // Set flag to true after running the motor
    }
    array_string = Serial.readStringUntil('\n');
    array_string.trim(); // Remove any whitespace or newline characters

    // Check if input is valid and print for debugging
    Serial.print("Received: ");
    Serial.println(array_string);

    // First, check if we are receiving a command for the mechanisms (length 3)
    if (array_string.length() == 3 && array_string.indexOf(',') != -1) {
      parseMechanismData();
    } else if (array_string.length() > 0) {
      parseMotorData();
    } else {
      Serial.println("Invalid data received.");
    }
  }

  // Continuously update motor speeds (non-blocking)
  updateMotors();
  seed_motor.setSpeed(100);
}

// Parse mechanism control data for rack and seed motor
void parseMechanismData() {
  for (int j = 0; j < mechanisms; j++) {
    int comma_index = array_string.indexOf(',');
    if (comma_index == -1) {
      parsed_values2[j] = array_string.toInt();  // If no comma, parse last value
    } else {
      parsed_values2[j] = array_string.substring(0, comma_index).toInt();  // Parse value before comma
      array_string = array_string.substring(comma_index + 1);  // Remove parsed part
    }

    // Debug parsed values
    Serial.print("Parsed mechanism value ");
    Serial.print(j);
    Serial.print(": ");
    Serial.println(parsed_values2[j]);
  }

  // Check parsed_values2[0] to control rack_motor
  if (parsed_values2[0] == 1) {
    runRackMotor();
  } 

  // Check parsed_values2[1] to control seed_motor
  if (parsed_values2[1] == 1) {
    seed_motor.setSpeed(100);
    Serial.println("Seed Dispenser running.");
  } else {
    seed_motor.setSpeed(0);
    Serial.println("Seed Dispenser stopped.");
  }
}

// Parse motor speed data for motor1, motor2, motor3, motor4
void parseMotorData() {
  for (int i = 0; i < numMotors; i++) {
    int comma_index = array_string.indexOf(',');
    if (comma_index == -1) {
      parsed_values[i] = array_string.toInt();  // Parse the last value
    } else {
      parsed_values[i] = array_string.substring(0, comma_index).toInt();  // Parse value before the comma
      array_string = array_string.substring(comma_index + 1);  // Remove parsed part
    }

    // Debug parsed values
    Serial.print("Parsed motor value ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(parsed_values[i]);
  }
}

void updateMotors() {
  motor2.setSpeed(parsed_values[0]);  // Left motor 1
  motor1.setSpeed(parsed_values[1]);  // Right motor 1
  motor3.setSpeed(parsed_values[2]);  // Left motor 2
  motor4.setSpeed(parsed_values[3]);  // Right motor 2
}

// Non-blocking control for the rack motor
void runRackMotor() {
  unsigned long currentMillis = millis();

  // Run rack_motor with non-blocking delay
  if (currentMillis - previousMillis >= rackDelay) {
    rack_motor.setSpeed(50);  // Run motor counterclockwise
    delay(270);
    rack_motor.setSpeed(0);
    delay(rackDelay);
    rack_motor.setSpeed(-50);   // Run motor clockwise for 8ms
    previousMillis = currentMillis;  // Update last delay time
    delay(270);
    rack_motor.setSpeed(0);
    delay(3000);
    
  }
  Serial.println("Rack motor running");
}

