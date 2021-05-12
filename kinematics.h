#ifndef _KINEMATICS_H
#define _KINEMATICS_H
#include<stdint.h>

const float WHEEL_DIAMETER = 70;
const float WHEEL_RADIUS = 35;
const float WHEEL_CIRCUMFERENCE = M_PI * WHEEL_DIAMETER;
const float WHEEL_SEPARATION = 143.5;
const float GEAR_RATIO=120;
const float COUNTS_PER_WHEEL_REVOLUTION = 1440;
const float MM_PER_COUNT = WHEEL_CIRCUMFERENCE/COUNTS_PER_WHEEL_REVOLUTION;

const float COUNTS_PER_MM = 1/MM_PER_COUNT;

class Kinematics {
  public:
     
    // What variables do you need?
    // What is the appropriate type?
    // ...

    // Function Prototypes
    Kinematics();   // constructor 
    void update();    // update kinematics
    float getHeadingTarget();
    float getDistanceTarget();
    float getX();
    float getY();
    float getTheta();
    float getDistanceInCounts(float distance);
    void Coordinates();
  private:
    float x;
    float y;
    float d;
    float theta;
    float count_left_e;
    float count_right_e;
    float last_count_left_e;
    float last_count_right_e;
    
    
}; // End of class definition.


Kinematics::Kinematics() {
  x = 0.0;
  y = 0.0;
  theta = 0.0;
} // end of constructor.

// Routine to execute the update to
void Kinematics :: update() {
  float current_count_left = count_left_e;
  float current_count_right = count_right_e;
  float count_left_diff = current_count_left - last_count_left_e;
  float count_right_diff = current_count_right - last_count_right_e;
  float d = (count_left_diff*MM_PER_COUNT +current_count_right*MM_PER_COUNT)/2;
//  x += d * cos(theta);
//  y += d * sin(theta);
  float left_distance = count_left_diff/COUNTS_PER_MM;
  float right_distance = count_right_diff/COUNTS_PER_MM;

  float mean_diff = (left_distance + right_distance)/2;
  
  // Update x, y and theta
  theta = theta + (left_distance - right_distance)/WHEEL_SEPARATION;
  if (theta >= 180)
  {
    theta = theta - 360;
  }
  if (theta <= -180)
  {
    theta += 360;
  }
  x += (d * cos(theta));
  y += (d * sin(theta));

  last_count_left_e = current_count_left;
  last_count_right_e = current_count_right;

  return;
}

float Kinematics :: getHeadingTarget() {
  return atan2(-y, -x);
}

float Kinematics :: getDistanceTarget() {
  return (float)sqrt(pow(x, 2) + pow(y, 2));
}

float Kinematics :: getDistanceInCounts(float distance) {
  return (distance * COUNTS_PER_MM);
}

float Kinematics :: getX() {
  return x;
}

float Kinematics :: getY() {
  return y;
}

float Kinematics :: getTheta() {
  return theta;
}
void Kinematics::Coordinates()
{
    Serial.print(x);
    Serial.print(",");
    Serial.print(y); 
    Serial.print(",");
    Serial.println(theta);
}

#endif
