#include <Servo.h>

//attach tilt servo to shield servo pins 1 and pan servo to shield pins 2. Black wire facing hole/usb input.

Servo pan;  
Servo tilt;

int panpos = 0;        // variable to store the servo position
int tiltpos = 0;
int delayspeed = 1;   //the delay is required for the servos to move properly. I would recommend not
                      // going below 4-5 ms 

void setup() {
  tilt.attach(9); // attaches the servo on pin 9 to the servo object(pan)
  pan.attach(10);  // tilt
  tilt.write(150);  //initialize servo position in case it has been tampered with
  pan.write(120);
  Serial.begin(115200);   //dont change, used for communication with windows machine
}

void move(int pos){

    //positions describe view of room and order of movement:
    //    0    |     1
    //    3    |     2
  
    switch(pos){

        // these values have been configured for our construction of the pantilt bracket. 
        // they will be changed based on size of room/scope of project
      
        case 0:
          for (tiltpos; tiltpos <= 180; tiltpos += 1) {
            tilt.write(tiltpos);              
            delay(delayspeed);                                     
          }  
          for (panpos; panpos <= 120; panpos += 1) { 
            pan.write(panpos);              
            delay(delayspeed);                       
          } 
          break;
          
        case 1:
          for (tiltpos; tiltpos <= 180; tiltpos += 1) {
            tilt.write(tiltpos);              
            delay(delayspeed);  
          }
          for (panpos; panpos >= 80; panpos -= 1) { 
            pan.write(panpos);              
            delay(delayspeed);                       
          }    
          break;
        
        case 2:
          for (tiltpos; tiltpos >= 140; tiltpos -= 1) {
            tilt.write(tiltpos);              
            delay(delayspeed);  
          }
          for (panpos; panpos >= 80; panpos -= 1) { 
            pan.write(panpos);              
            delay(delayspeed);                       
          }    
          break;

        case 3:
          for (tiltpos; tiltpos >= 140; tiltpos -= 1) {
              tilt.write(tiltpos);              
              delay(delayspeed);  
            }
            for (panpos; panpos <= 120; panpos += 1) { 
              pan.write(panpos);              
              delay(delayspeed);                      
            }    
            break;     
      }
  }

void loop() {

  //TODO: Send ack to windows machine when arduino has reached position. Then windows
  //machine can take photo and give next location instruction.
  if(Serial.available() > 0){ //read input position from windows machine and move
      int pos = Serial.read()- '0';   //convert byte to integer
      move(pos);
      delay(1000);
      Serial.print(pos); //printing a line acts as pseudo ack
    }
    
}
