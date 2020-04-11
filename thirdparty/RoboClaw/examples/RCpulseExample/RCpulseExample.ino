//On Roboclaw set switch 1 and 6 on.

#include <Servo.h> 
 
Servo myservo1;  // create servo object to control a Roboclaw channel
Servo myservo2;  // create servo object to control a Roboclaw channel
 
int pos = 0;    // variable to store the servo position 
 
void setup() 
{ 
  myservo1.attach(5);  // attaches the RC signal on pin 5 to the servo object 
  myservo2.attach(6);  // attaches the RC signal on pin 6 to the servo object 
} 
 
void loop() 
{ 
  myservo1.writeMicroseconds(1250);  //full forward
  myservo2.writeMicroseconds(1750);  //full reverse
  delay(2000);
  myservo1.writeMicroseconds(1750);  //full reverse
  myservo2.writeMicroseconds(1250);  //full forward
  delay(2000);
} 
