//gkapsid
//26/9/2021 worked OK
//add servo motor 90 position  
//clean Serial.print statements OK

//28/9/2021 to be tested again with different maximum speed 
// otherwise it runs so fast that it can't stop on time!!

#include <SharpDistSensor.h>
//#include <Servo.h>

// Analog pin to which the sensor is connected
const byte sensorPin = A0;

// Window size of the median filter (odd number, 1 = no filtering)
const byte medianFilterWindowSize = 5;

// Create an object instance of the SharpDistSensor class
SharpDistSensor sensor(sensorPin, medianFilterWindowSize);

//Servo myservo; //for servo motor bearing the ultrasonic distance sensor 

// Motor 1
int motor1_S = 9; //PWM speed control
int motor1_b = 8;
int motor1_f = 7;

// Motor 2 
int motor2_S = 10; // PWM speed control
int motor2_f = 11;
int motor2_b = 12;

//initial motor speed
int speedR = 128; //PWM speed for Right wheel
int speedL = 128; // PWM speed for Left wheel


int motormaxSpeed = 128; // maximum speed (PWM value) 
// maximum speed could be with 255 PWM value but the vehicle becomes unstable 
//it is running fast and cannot react quickly to reduce speed and stop 
///the maximum PWM speed  should be set after some tests.
int motorminSpeed = 60; // minimum speed (PWM value) practical minimum PWM value that can actually move the robot
// Less than minimum speed value cannot move the robot. Motors are stalled.

/*
  Distance Sensor
*/

//float Distance = 0; //in cm if ultrasonic sensor is used
unsigned int distance = 0;//in mm 


// set true to view messages for debugging
bool debug = false;

void setup() {
  // all those pins are output
  pinMode(motor1_S, OUTPUT);
  pinMode(motor2_S, OUTPUT);
  pinMode(motor1_b, OUTPUT);
  pinMode(motor1_f, OUTPUT);
  pinMode(motor2_f, OUTPUT);
  pinMode(motor2_b, OUTPUT);

  
  // Output pin initialization
  digitalWrite(motor1_b, LOW);
  digitalWrite(motor1_f, LOW);
  digitalWrite(motor2_f, LOW);
  digitalWrite(motor2_b, LOW);
  
  if (debug) Serial.begin(9600); //used for debugging

  // Set sensor model
  sensor.setModel(SharpDistSensor::GP2Y0A41SK0F_5V_DS);

//  myservo.attach(13);

 // myservo.write(90); //set servo motor to 90 degs
}

  

void loop() {
 // Get distance from sensor
 distance = sensor.getDist();
startMotor();
speedCalc();
setMotorSpeed();

} 
  

// stop the robot, stop both motors 
void stop_all() {
  speedL = 0;
  speedR = 0;
  digitalWrite(motor1_b, LOW);
  digitalWrite(motor1_f, LOW);
  digitalWrite(motor2_f, LOW);
  digitalWrite(motor2_b, LOW);
  setMotorSpeed();
   
}

// calculate the wheel speed against the distance from an obstacle
void speedCalc() {
    distance = sensor.getDist(); //in mm
  if (distance>350) {
    speedR = motormaxSpeed;
    speedL = motormaxSpeed;
    if (debug) {
      Serial.print("Max Speed L ");
      Serial.println(speedL);
      delay(10);
    }
}
  else if (distance >100 && distance <350) {
    speedL = map(distance, 100, 350, motorminSpeed, motormaxSpeed);
    speedR = map(distance,100, 350, motorminSpeed, motormaxSpeed);
     if (debug) {
      Serial.print("SpeedL ");
      Serial.println(speedL);
      delay(10);
     }

  }
  
  else if (distance <100){
    stop_all();
  }
  if (debug) {
    Serial.println(distance);
    Serial.println(digitalRead(motor1_f));
  }

}

//Set the calculated speed to each motor
void setMotorSpeed() {
  analogWrite(motor1_S, speedR);
  analogWrite(motor2_S, speedL);
}

//start the movement of the motors
void startMotor() {
  
  digitalWrite(motor1_f, HIGH); // Χ Ξεκίνα τον αριστερό μπροστά
  digitalWrite(motor1_b, LOW); //Χ
  digitalWrite(motor2_f, HIGH);// Χ Ξεκίνα τον δεξιό μπροστά
  digitalWrite(motor2_b, LOW); //Χ
   
}

//needed if ultrasonic sensor is used instead
/*
long readUltrasonicDistance(int triggerPin, int echoPin)
{
  pinMode(triggerPin, OUTPUT);  // Clear the trigger
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  // Sets the trigger pin to HIGH state for 10 microseconds
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  pinMode(echoPin, INPUT);
  // Reads the echo pin, and returns the sound wave travel time in microseconds
  return pulseIn(echoPin, HIGH);
}
*/
