#include <Arduino.h>
#include <Encoder.h>
#include <Servo.h>
#include "BasicStepperDriver.h"


Servo myServo;
#define servoPin 9
#define feedbackPin A0

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 120

// Since microstepping is set externally, make sure this matches the selected mode
// If it doesn't, the motor will move at a different RPM than chosen
// 1=full step, 2=half step etc.
#define MICROSTEPS 8 //was 1

// All the wires needed for full functionality
#define DIR 4 //was 8
#define STEP 5 //was 9
//Uncomment line to use enable/disable functionality
//#define SLEEP 13

// 2-wire basic config, microstepping is hardwired on the driver
BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP);

//Uncomment line to use enable/disable functionality
//BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP, SLEEP);

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(2 /*5*/, 3 /*6*/);
//   avoid using pins with LEDs attached
// Optical Rotary Encoder PIN layout from colored end 1,  2,   3,   4,  5
//                                                    0V, N.C, ChB, 5V, ChA

// 14 * 9 = 126
#define FIRST (14 * 9)
// 32 * 9 = 288
#define SECOND (32 * 9)
// 26 * 9 = 234
#define THIRD (26 * 9)

// This variable will help determine whether a second number can be found with the given first and third number. 
bool congruency = false;

// This will store the possibilities for the second number.
int secondNumberPossibilities[10];

void setup() {
  Serial.begin(9600);
  myServo.attach(servoPin);
  myServo.write(65);
  stepper.begin(RPM, MICROSTEPS);
  // if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next line
  stepper.setEnableActiveState(LOW);
  stepper.enable();

  if ((FIRST % 4) == (THIRD % 4)) {
    congruency = true;
    findSecondNumber();
  } else {
    congruency = false;
    Serial.println("First and third number are not congruent by mod 4.");
  }
}

void loop() {
  if (congruency) {
    openLock();
  }

  // pause and allow the motor to be moved by hand
  stepper.disable();
}

void findSecondNumber() {
  //this variable is used to check congruency between the second and third numbers
  int testVariable = ((THIRD % 4) - 2);
  int count = 0;

  for (int i = 0; i < 40; i++) {
    if ((i % 4) == testVariable) {
      secondNumberPossibilities[count] = (i * 9);
      //Serial.println(secondNumberPossibilities[count]);
      count++;
    }
  }
  return;
}

void openLock() {
  /*
   * Try to open the lock for as many possibilities exist for the second number. 
   */
  for (int i = 0; i < 10; i++) {
    Serial.println(secondNumberPossibilities[i]/9);
    /*
     * Reset the lock by rotating the stepper counterclockwise three times.  
     */
    stepper.rotate(-360*3);
    Serial.println(myEnc.read()/9);
    /*
     * Rotate to first number.  
     */
    stepper.rotate(-360+FIRST);
    Serial.println(myEnc.read()/9);
    delay(100);
    /*
     * Rotate to second number.  
     */
    //stepper.rotate(360+SECOND-FIRST);
    if (secondNumberPossibilities[i] > FIRST) {
      stepper.rotate(360+secondNumberPossibilities[i]-FIRST); // works for any second number bigger than the first number.
      Serial.println(myEnc.read()/9);
    } else {
      stepper.rotate(360+(360-FIRST)+secondNumberPossibilities[i]); //works for any second number smaller than the first number.
      Serial.println(myEnc.read()/9);
    }
    delay(100);
    /*
     * Rotate to third number.  
     */
    //stepper.rotate(-(SECOND-THIRD));
    if (secondNumberPossibilities[i] > THIRD) {
      stepper.rotate(-(abs(secondNumberPossibilities[i]-THIRD)));
      Serial.println(myEnc.read()/9);
    } else {
      stepper.rotate(-(abs(secondNumberPossibilities[i]+(40*9)-THIRD)));
      Serial.println(myEnc.read()/9);
    }
    delay(250);

    openShackle();
    /*
     * Rotate back to zero. 
     */
    stepper.rotate(-THIRD-(1*9));
    Serial.println(myEnc.read()/9);
    delay(100);

//    stepper.rotate((secondNumberPossibilities[i]*9));
//    delay(1000);
//    stepper.rotate(-(secondNumberPossibilities[i]*9));
//    delay(1000);
  }
  return;
}

void openShackle(){ // Slowly try to open shackle. If wrong combo, 
  
  int a = 0;
  int b = 1;
  int k=65;
  myServo.write(k);
  delay(500);
  for(k;k<120;k+=5)
  {
    myServo.write(k);
    delay(500);
    a = analogRead(feedbackPin);//mid-80 open-100
    Serial.println(a);
    k+=5;
    myServo.write(k);
    delay(500);
    b = analogRead(feedbackPin);
    Serial.println(b);
    Serial.println(b-a);
    if((b - a < 7)||(b - a > 25 ))//8 sensitivity allows enough force against the shackle to lift it open but small enough to stop trying 
     {                           // 25 because sometimes when its too much force, the servo is around 34. not sure why but its pretty consitant
      myServo.write(k-30);       //and 34 is also a number that should open the shackle but doesnt
      delay(2500);
      a = analogRead(feedbackPin);//mid-80 open-100
      Serial.println("Did not open shackle");
      Serial.println(k);
      break;
      }
      myServo.write(65);
    }
    
}
