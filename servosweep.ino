#include <Servo.h>

Servo pan;  // create servo object to control a servo
Servo tilt;
// twelve servo objects can be created on most boards

int panpos = 0;    // variable to store the servo position
int tiltpos = 0;

void setup() {
  pan.attach(9);  // attaches the servo on pin 9 to the servo object(pan)
  tilt.attach(10); // (tilt)
  Serial.begin(9600); 
}

void loop() {
  
  //tilt.write(180);    //This is the upright position of the tilt servo
  
  /*for (tiltpos = 100; tiltpos <= 150; tiltpos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    tilt.write(tiltpos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  
  for (panpos = 0; panpos <= 180; panpos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    pan.write(panpos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
    
  for (tiltpos = 150; tiltpos >= 100; tiltpos -= 1) { // goes from 180 degrees to 0 degrees
    tilt.write(tiltpos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }

  for (panpos = 180; panpos >= 0; panpos -= 1) { // goes from 180 degrees to 0 degrees
    pan.write(panpos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }*/

  int val = analogRead(2);
  Serial.print(val);
  
  
}
