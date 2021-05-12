#ifndef _MOTOR_H
#define _MOTOR_H

// A class to neatly contain commands for the 
// motors, to take care of +/- values, a min/max
// power value, & pin setup.

class motor_c {
  //float pwm_pin;
  //int dir_pin;
  public:
    int dir_pin;
    float pwm_pin;


    // This is a function prototype.
    // The actual function is written outside
    // of the class (see below).
    motor_c(float _pwm_pin, int _dir_pin );

    void setMotor(float power){
      boolean dir;
      if(power<0){
        dir=LOW;
      } else{
        dir=HIGH;
      }
      if(power<0) power = power*-1;
      if(power>=255){
        power=255;
      }
      digitalWrite(dir_pin, dir);
      analogWrite(pwm_pin, power);
//      return dir;
      } //end of set motor
}; //end of class

// Constructor: when you create an instance
// of this class, your constructor will be
// called automatically.  You can treat it 
// a bit like the main setup().  What needs
// to be setup initially within this class?
motor_c::motor_c( float _pwm_pin, int _dir_pin ) {
  // ...
  pwm_pin = _pwm_pin;
  dir_pin = _dir_pin;

  pinMode( pwm_pin, OUTPUT);

  pinMode( dir_pin, OUTPUT);

// set direction of motors
  digitalWrite( dir_pin, LOW);
  // set initial pwm to motors to 0
  analogWrite( pwm_pin, 0);
}

// You can add your own functions to the class.
// Don't forget to also prototype in the class
// definition above.
//void motor_c::myOwnFunction( float an_argument ) {
//  
//}

#endif
