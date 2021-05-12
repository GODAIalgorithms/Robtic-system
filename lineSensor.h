#ifndef _LINESENSOR_H
#define _LINESENSOR_H
#define BUZZER 5

//#define LINE_LEFT_PIN A4 //Pin for the left line sensor
//#define LINE_CENTRE_PIN A3 //Pin for the centre line sensor
//#define LINE_RIGHT_PIN A2 //Pin for the right line sensor

const int calibrate_count = 100;
class lineSensor_c {
  private:
    
    int pin;
    int threshold;
    float rawValue = 0;
    float bias_compensation;
  public:
    // Constructor, accepts a pin number as
    // argument and sets this as input.
    lineSensor_c( int pin );




    // Write your calibration routine here
    // to remove bias offset
    void calibrate();
    int readRaw();
    float readCalibrate();
    bool not_onLine();
    


};


lineSensor_c::lineSensor_c(int linePin) {
  threshold = 100;
  pin = linePin;
  pinMode(pin, INPUT);
}
int lineSensor_c::readRaw() {
  return analogRead(pin);
}

void lineSensor_c::calibrate() {
  int add = 0;
  for (float j = 0; j < calibrate_count; j++) {
    add = add + analogRead(pin);
  }
  bias_compensation = (float)add / (float)calibrate_count;


  analogWrite(BUZZER, 10);
  delay(100);
  analogWrite(BUZZER, 0);

}
float lineSensor_c::readCalibrate() {
  int reading = (float)analogRead(pin) - bias_compensation;
  reading = constrain(reading, 0, 1023);
  reading = bitRead(reading, 1023);
  return reading;
}

bool lineSensor_c::not_onLine(){
  if (readCalibrate()> threshold){
    return true;
  }
  else{
    return false;
    }
}

#endif
