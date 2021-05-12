

#include "lineSensor.h"
#include "encoders.h"
#include "kinematics.h"
#include "motor.h"
#include "pid.h"


#define LINE_LEFT_PIN A4 //Pin for the left line sensor
#define LINE_CENTRE_PIN A3 //Pin for the centre line sensor
#define LINE_RIGHT_PIN A2 //Pin for the right line sensor
#define LED_BUILT_IN 13

#define L_MOTOR_PWM 10
#define L_MOTOR_DIR 16
#define R_MOTOR_PWM 9
#define R_MOTOR_DIR 15

#define BUZZER_PIN 6

#define STATE_INITIAL        0
#define STATE_DRIVE_FORWARDS  1
#define STATE_FOLLOW_LINE    2
#define STATE_LOST_LINE      3
#define STATE_REJOIN_LINE    4
#define STATE_TURN_LEFT_TO_HOME   5
#define STATE_DRIVE_HOME     6
#define STATE_STOP           10
float update_ts;

motor_c LeftMotor( L_MOTOR_PWM, L_MOTOR_DIR);
motor_c RightMotor(R_MOTOR_PWM, R_MOTOR_DIR);


lineSensor_c line_left(LINE_LEFT_PIN); //Create a line sensor object for the left sensor
lineSensor_c line_centre(LINE_CENTRE_PIN); //Create a line sensor object for the centre sensor
lineSensor_c line_right(LINE_RIGHT_PIN); //Create a line sensor object for the right sensor
//boolean dir;
float LeftSensorRead;
float RightSensorRead;
float CentreSensorRead;
float M;
int pin;
int I_total;
int power_max;
float pwm_left;
float pwm_right;
float distance;
int STATE;
bool first_line_found;
float confidence_threshold;
float left_line_confidence;
float centre_line_confidence;
float right_line_confidence;
float Angle=0;
float threshold;
float distance_to_home;
bool on_line;
bool check_left;
bool check_right;
unsigned long timestamp;
int recored_count;
float initial_speed = 30;
int dc=0;
float line_demand;
float heading_demand;
float measurement;
//PID drive home
#define kp_drive 50
#define kd_drive 0.01
#define ki_drive 0.0
PID_c driveHomeSpeed(kp_drive,kd_drive,ki_drive );

//PID turntoHOME rotation controller
#define kp_angle 0.1
#define kd_angle -0.05
#define ki_angle 0.00015
PID_c leftAngle(kp_angle,kd_angle,ki_angle);
PID_c rightAngle(kp_angle,kd_angle,ki_angle);




//PID drive forward
#define line_kp 0.1
#define line_ki 0.0
#define line_kd -0.05
PID_c leftCount(line_kp,line_kd,line_ki);
PID_c rightCount(line_kp,line_kd,line_ki);



//PID line folling wheel speed controller
#define wheel_kp 0.1
#define wheel_ki 0.0
#define wheel_kd 0.0
PID_c leftWheel(wheel_kp,wheel_kd,wheel_ki);
PID_c rightWheel(wheel_kp,wheel_kd,wheel_ki);

Kinematics pose;

void setupMotor(){
  pinMode( L_MOTOR_PWM, OUTPUT);
  pinMode( L_MOTOR_DIR, OUTPUT);
  pinMode( R_MOTOR_PWM, OUTPUT);
  pinMode( R_MOTOR_DIR, OUTPUT);
  pinMode( BUZZER_PIN, OUTPUT );
  digitalWrite( L_MOTOR_DIR, LOW);
  digitalWrite( R_MOTOR_DIR, LOW);
}
void setup() {
  //setupLineSensor0();
  setupEncoder0();
  setupEncoder1();
  setupMotor();
  // Start up the serial port.
  
  threshold=200.0;
  confidence_threshold= 60.0;
  left_line_confidence = 0.0;
  centre_line_confidence = 0.0;
  right_line_confidence = 0.0;
  distance_to_home=0.0;
  distance = 0.0;
  Serial.begin(9600);

  // Delay to connect properly.
  delay(1000);
  Serial.println("***RESET***");

    
//  test_pwm=0;
//  test_dir=-1;
  // Calibrate the three line sensors.
  line_left.calibrate();
  line_centre.calibrate();
  line_right.calibrate();
  on_line = false;
  first_line_found = false;
  
  //dir = LOW;
  LeftSensorRead=0;
  RightSensorRead=0;
  CentreSensorRead=0;
  update_ts = 0;
 
  check_left=false;
  check_right=false;
  initial_speed=18.0;

  power_max = 100;
  
  timestamp= millis();
  leftAngle.setMax(30);
  rightAngle.setMax(30);
  driveHomeSpeed.setMax(100);
  measurement=0.0;
  STATE = 0;
} // end of setup()



void loop() {

  
  LineConfidence();
  on_line=onLine();
  //Bang();
  //M_scale_Bang();
  unsigned long elapsed_time = millis() - timestamp;
  update_ts = timestamp;
  
  switch( STATE ) {
    case STATE_INITIAL:
      // heading_demand = M_PI / 2;
      // updateFrontPID(0);
      initialisingBeeps(); 
      break;
    case STATE_DRIVE_FORWARDS:
      driveForwards();     
      break;
    case STATE_FOLLOW_LINE:
      if (elapsed_time > 10) {
        update_ts = millis();
        followLine();
      } 
      break;
    case STATE_LOST_LINE:
      lostLine();
      break;
    case STATE_REJOIN_LINE:
      if (elapsed_time > 10) {
        update_ts = millis();
        rejoinLine();
      }
      break;
    case STATE_TURN_LEFT_TO_HOME:
      facingHome();
      break;
    case STATE_DRIVE_HOME:
      if (elapsed_time > 5) {
        update_ts = millis();
        driveHome();
      }
      break;
    case STATE_STOP:
      updateSpeed(0,0);
      break;
    default:
      Serial.println("System Error, Unknown state!");
      break;
  }

        
  
  delay(200);
  
}
void leftMotor(float a){
  LeftMotor.pwm_pin = a;
  //LeftMotor.dir_pin = LeftMotor.setMotor(a);
  LeftMotor.setMotor(a);
  
}
void rightMotor(float a){
  RightMotor.pwm_pin=a;
  //RightMotor.dir_pin = RightMotor.setMotor(a);

  RightMotor.setMotor(a);
}


void Bang(){
  LeftSensorRead = line_left.readCalibrate();
  RightSensorRead = line_right.readCalibrate();
  CentreSensorRead = line_centre.readCalibrate();

  bool left_online =false;
  bool right_online =false;
  bool centre_online =false;
  
//
  if (LeftSensorRead > 90) left_online = true;
  if (CentreSensorRead > 90) right_online = true;
  if (RightSensorRead > 110) centre_online = true;

  if (centre_online) {
    leftMotor(37.0f);
    rightMotor(36.0f);
  }
  else if (left_online) {
    rightMotor(36.0f);
    leftMotor(-37.0f);
  }
  else if (right_online) {
    leftMotor(37.0f);
    rightMotor(-36.0f);
  }
  else {
    leftMotor(0.0f);
    rightMotor(0.0f);

}
}
float M_scale_Bang(){
  LeftSensorRead = line_left.readCalibrate();
  RightSensorRead = line_right.readCalibrate();
  CentreSensorRead = line_centre.readCalibrate();


//  if (LeftSensorRead || RightSensorRead || CentreSensorRead != 0){
  
  I_total = abs(LeftSensorRead) + abs(RightSensorRead) + abs(CentreSensorRead);

  float P_left =  LeftSensorRead / I_total;
  float P_right = RightSensorRead / I_total;
  float P_center = CentreSensorRead / I_total;
  M = P_left - P_right;
  
  pwm_left = M * power_max * 1;
  pwm_right = M * power_max * (-1);

  float line_centre = (P_left * 1000) + (P_right * 2000) + (P_center * 3000) - 2000;
  return constrain(line_centre, -2000, 2000);

//    if(M<0){ //move to the left
//      leftMotor(pwm_left);
//      rightMotor(pwm_right);
//    }
//    else if(M>0){//moves to the right
//      leftMotor(pwm_left);
//      rightMotor(pwm_right); 
//    }
//    else if(((LeftSensorRead && RightSensorRead)==0)&& CentreSensorRead != 0){//towards 0 move to the centre
//      leftMotor(power_max);
//      rightMotor(power_max);
//      
//    }
//    else if((LeftSensorRead && RightSensorRead && CentreSensorRead) == 0){
//      leftMotor(0.0f);
//      rightMotor(0.0f);
//      
//    }

  
}


//void move_Straight(){
//  LeftSensorRead = line_left.readCalibrate();
//  RightSensorRead = line_right.readCalibrate();
//  CentreSensorRead = line_centre.readCalibrate();
//  if((LeftSensorRead==0) && (RightSensorRead==0) && (CentreSensorRead==0)){
//    leftMotor(power_max);
//    rightMotor(power_max);  
//      
//  }
void LineConfidence(){
  LeftSensorRead = line_left.readCalibrate();
  RightSensorRead = line_right.readCalibrate();
  CentreSensorRead = line_centre.readCalibrate();
  if (LeftSensorRead>threshold){
    left_line_confidence += 0.5;
  }
  else {
    left_line_confidence -= 0.5;
  }
  
  if (RightSensorRead>threshold){
    right_line_confidence += 0.5;
  }
  else {
    right_line_confidence -= 0.5;
  }
  if (CentreSensorRead>threshold){
    centre_line_confidence += 0.5;
  }
  else {
    centre_line_confidence -= 0.5;
  }
  left_line_confidence = constrain(left_line_confidence, 0.0, 100.0);
  centre_line_confidence = constrain(centre_line_confidence, 0.0, 100.0);
  right_line_confidence = constrain(right_line_confidence, 0.0, 100.0);
  }

bool onLine(){
  bool left_online =false;
  bool right_online =false;
  bool centre_online =false;
  if(left_online>confidence_threshold){
    left_online = true;
  }
  if(right_online>confidence_threshold){
    right_online = true;
  }
  if(centre_online>confidence_threshold){
    centre_online = true;
  }  

  if(left_online||right_online||centre_online){
    if(!first_line_found){
      first_line_found = true;
      left_line_confidence = 100.0;
      right_line_confidence = 100.0;
      centre_line_confidence = 100.0;
      
      
    }
    return true;
  }
  else{
    return false;
  }
}
void rejoinLine(){
  unsigned long elapsed_time = millis() - timestamp;
  if(check_left && check_right){
    heading_demand = measurement;
    updateSpeed(-15.0f, 15.0f);
    if(pose.getTheta() <= heading_demand) {
      updateSpeed(0, 0);
      delay(3000);
      STATE = STATE_TURN_LEFT_TO_HOME;
      return;
    }
  }
  if (!check_left && !check_right) {
    heading_demand = measurement - M_PI/8;
    updateSpeed(-15.0f, 15.0f);
    
    if (pose.getTheta() <= heading_demand) {
      check_left = true;
    }
  }

  if (check_left && !check_right) {
    heading_demand = measurement + M_PI/8;
    updateSpeed(15.0f, -15.0f);

    if (pose.getTheta() >= heading_demand) {
      check_right = true;
    }
  }  
  if(onLine){
    STATE = STATE_FOLLOW_LINE;
  }
  
}
void initialisingBeeps(){
  for(int i = 0;i<2;i++){
    delayResponse(0, 50);
    delay(2000);
  }
  STATE = STATE_DRIVE_FORWARDS;
}
void driveForwards(){
  if(on_line){
    first_line_found =true;
    STATE = STATE_FOLLOW_LINE;
  }
  else{
     dc = (e0_speed + e1_speed) / 2;
     float countSpeedLeft = leftCount.update(dc, e0_speed);
     float countSpeedRight = rightCount.update(dc, e1_speed);
     //pose.Update(count_e0, count_e1);
     analogWrite( L_MOTOR_PWM, initial_speed + countSpeedLeft);
     analogWrite( R_MOTOR_PWM, initial_speed + countSpeedRight);    
  }
  }
//}
//void driveForwards() {
//  if (!on_line) {
//    updateFrontPID(30.0);
//  } else {
//    first_line_found = true;
//    STATE = STATE_FOLLOW_LINE;
//  }
//}
void delayResponse(int a, int b){
  analogWrite(BUZZER_PIN, a);
  delay(b);
  analogWrite(BUZZER_PIN, 0);
}

void lostLine(){
  if (first_line_found) {
    updateSpeed(0.0, 0.0);
    measurement = pose.getTheta();
    STATE = STATE_REJOIN_LINE;
  }
}
void followLine(){
  if(on_line){
    PID_of_Line();
  }
  else{
    STATE = STATE_LOST_LINE;
  }
}

void driveHome(){
  heading_demand = pose.getHeadingTarget();
  updateFrontPID(18.0);
  if (abs(pose.getDistanceTarget()) <= 45.0) {
    if (distance_to_home >= distance){
      STATE = STATE_STOP;
    } 
  }

  distance = distance_to_home;
}

void PID_of_Line(){
  LeftSensorRead = line_left.readCalibrate();
  RightSensorRead = line_right.readCalibrate();
  CentreSensorRead = line_centre.readCalibrate();
  float centre_line= M_scale_Bang();
  
  float d = (LeftSensorRead+RightSensorRead)/2;
  float output_PID_signal = leftCount.update(line_demand, centre_line);
  float left_speed = initial_speed - output_PID_signal;
  float right_speed= initial_speed + output_PID_signal;
  if(left_speed<0){
    left_speed = left_speed * -1;
    digitalWrite(L_MOTOR_DIR, HIGH);
  }else{
    digitalWrite(L_MOTOR_DIR, LOW);
  }
  if(right_speed<0){
    right_speed = right_speed * -1;
    digitalWrite(R_MOTOR_DIR, HIGH);
  }else{
    digitalWrite(R_MOTOR_DIR, LOW);
  }
  analogWrite(L_MOTOR_PWM,abs(left_speed));
  analogWrite(R_MOTOR_PWM,abs(right_speed));

 //pose.update(count_e0,count_e1);
  

}
void updateSpeed(float left_speed, float right_speed){
  if(left_speed<0){
    left_speed = left_speed * -1;
    digitalWrite(L_MOTOR_DIR, HIGH);
  }else{
    digitalWrite(L_MOTOR_DIR, LOW);
  }
  if(right_speed<0){
    right_speed = right_speed * -1;
    digitalWrite(R_MOTOR_DIR, HIGH);
  }else{
    digitalWrite(R_MOTOR_DIR, LOW);
  }
  analogWrite(L_MOTOR_PWM,left_speed);
  analogWrite(R_MOTOR_PWM,right_speed);
  
}
void updateFrontPID(float forward_bias) {
  float heading_output = driveHomeSpeed.update(heading_demand, pose.getTheta());

  float left_speed = forward_bias + heading_output;
  float right_speed = forward_bias - heading_output;

  left_speed = constrain(left_speed, -254, 254);
  right_speed = constrain(right_speed, -254, 254);

  updateSpeed(left_speed, right_speed);
}
void facingHome(){
  if(pose.getHeadingTarget()>= pose.getTheta()){
    updateSpeed(30, -30);
  }
  else{
    updateSpeed(-30,30);
  }
  if(abs(pose.getHeadingTarget()-pose.getTheta())<=0.01){
    updateSpeed(0,0);
    distance = pose.getDistanceTarget();
    STATE = STATE_DRIVE_HOME;
    return;    
  }
}
