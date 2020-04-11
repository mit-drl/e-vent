//On Roboclaw set switch 1 and 6 on. // <-- what does this refer to?
//mode 2 option 4 // <-- my note based on user manual pg 26

#include <Servo.h> 

Servo myservo1;  // create servo object to control a Roboclaw channel
Servo myservo2;  // create servo object to control a Roboclaw channel

//int pos = 0;    // variable to store the servo position  //<-- left-over from arduino ide servo sweep example?

void setup() 
{ 
  myservo1.attach(5);  // attaches the RC signal on pin 5 to the servo object 
  myservo2.attach(6);  // attaches the RC signal on pin 6 to the servo object 
} 
 

void loop() 
{ 
  //forward
  myservo1.writeMicroseconds(1600);
  myservo2.writeMicroseconds(1500);
  delay(2000);

  //backward
  myservo1.writeMicroseconds(1400);
  myservo2.writeMicroseconds(1500);
  delay(2000);

  //left
  myservo1.writeMicroseconds(1500);
  myservo2.writeMicroseconds(1600);
  delay(2000);
  
  //right
  myservo1.writeMicroseconds(1500);
  myservo2.writeMicroseconds(1400);
  delay(2000);
  
  //mixed forward/left
  myservo1.writeMicroseconds(1600);
  myservo2.writeMicroseconds(1600);
  delay(2000);

  //mixed forward/right
  myservo1.writeMicroseconds(1600);
  myservo2.writeMicroseconds(1400);
  delay(2000);
  
  //mixed backward/left
  myservo1.writeMicroseconds(1400);
  myservo2.writeMicroseconds(1600);
  delay(2000);

  //mixed backward/right
  myservo1.writeMicroseconds(1400);
  myservo2.writeMicroseconds(1400);
  delay(2000);
  
}

