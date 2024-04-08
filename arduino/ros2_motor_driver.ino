#include <Servo.h>

Servo myservo;  // create servo object to control a servo

void setup() {
  Serial.begin(9600);
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo.write(90);
}

void loop() {
  // if we get a valid byte, read analog ins:
  if (Serial.available() > 0) {

    // get incoming byte:
    String motor = Serial.readStringUntil('\n');
    if(motor.substring(0, motor.indexOf(':')) != "MOTORS") {
      String m = "MOTORS:";
      String c = ",";
      String r = "\n";
      String result = m + myservo.read() + c + myservo.read() + c + myservo.read() + r;
      Serial.write(result.c_str());
    }
    else if (motor.substring(0, motor.indexOf(':')) == "MOTORS") {
      String result = "OK:";
      String tmpString = motor.substring(motor.indexOf(':')+1);
      String angle1string = tmpString.substring(0,tmpString.indexOf(','));
      int angle1 = angle1string.toInt();
      if(angle1 >= 0 && angle1 <= 180) {
        myservo.write(angle1);  
      }
      else {
        result += ":";
        result += angle1;
      }
      
      tmpString = tmpString.substring(tmpString.indexOf(',')+1);
      String angle2string = tmpString.substring(0,tmpString.indexOf(','));
      int angle2 = angle2string.toInt();
      if(angle2 >= 0 && angle2 <= 180) {
        myservo.write(angle2);  
      }
      else {
        result += ":";
        result += angle2;
      }

      tmpString = tmpString.substring(tmpString.indexOf(',')+1);
      String angle3string = tmpString;
      int angle3 = angle3string.toInt();
      if(angle3 >= 0 && angle3 <= 180) {
        myservo.write(angle3);  
      }
      else {
        result += ":";
        result += angle3;
      }

      result += "\n";
      Serial.write(result.c_str());
    }
  }
}