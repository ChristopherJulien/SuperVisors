#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
//#include <SD.h>
#include <Adafruit_ADS1015.h>
#include <Arduino_LSM9DS1.h>
#include <Arduino_HTS221.h>

//----------------------------------------SETUP PINS--------------------------------------------------//

//Analog pins
#define phototransistorA_pin    A0
#define phototransistorB_pin    A1
#define phototransistorC_pin    A2
#define phototransistorD_pin    A3
#define SDA_pin                 A4
#define SCL_pin                 A5
#define rain_pin                A6
#define luminosityPot_pin        0  
#define pressurePot_pin          1  

//Digital pins
#define brakingLed_pin           3
#define button_pin               4
#define servoRight_pin           5
#define servoLeft_pin            6

//end of SETUP PINS

//----------------------------------------GENERAL VARIABLES--------------------------------------------------//
boolean flag_vel = false;                        //flag_vel = 1 if above velocity threshold
boolean flag_rain = false;                       //flag_rain = 1 if rain detected
boolean flag_sun = false;                        //flag_sun = 1 if luminosity above threshold
boolean flag_braking = false;                    //flag_braking = 1 if braking is detected
boolean stateFlag = 1;                           //stateFlag = 1 when it's time to check state and move motor

enum State {OPEN, TRANSPARENT, OPAQUE};          //three possible states of the system
State state = OPEN;                              //default state to begin

//String console_input;

//set up intervals for the different timers
const int luminosityInterval = 25;               //measure luminosity every 25 ms
const int pressureInterval = 200;
const int rainInterval = 500;
const int PotentiometerInterval = 1000;
const int brakingInterval = 100;
const int temperatureInterval = 200;

unsigned long currentTime;
long previousTimeLuminosity = 0;
long previousTimePressure = -20;                 //all pressure measures will be delayed by 20 ms to prevent overlapping
long previousTimeRain = -40;
long previousTimePotentiometer = -360;
long previousTimeBraking = -60;
long previousTimeTemperature = -20;

////set up SD card
//File myFile;

//end of GENERAL VARIABLES

//-------------------------------SENSORS AND ACTUATORS VARIABLES--------------------------------------------//
//servomotors variables
Servo servoRight;
Servo servoLeft;
static int angleOpen = 1;
static int angleTransparent = 50;
static int angleOpaque = 75;
static float opaque_min = 50;             //angle with the minimum tint
static float opaque_max = 75;              //angle with the maximum tint
int gradient;                           //gradient will vary between opaque_min and opaque_max
static float angle;

//rain variables
int rain_threshold = 500;

//luminosity variables
int luminosity_counter = 0;               //counter going from 0 to 4
static int phototransistorA;
int previousPhototransistorA;
int moyenne_phototransistors = 0;
int previousMoyenne_phototransistors = 0;
static float luminosity_threshold;
int lum[4][4];                            //[lignes]x[colonnes] : lignes aux 4 timings différents, colonnes les valeurs des 4 phototransistors
//calibration of the values of phototransistorA
static int phototransistor_value_max = 1023;
static int phototransistor_value_min = 0;

//pressure variables
const byte pressureAddress = 0x25;        //address of the pressure sensor
const byte readPressureCmd1 = 0x36;       //continuous reading (part 1 msb) with average
const byte readPressureCmd2 = 0x15;       //continuous reading (part 2 lsb) with average
//static float velocity_threshold;
float pressure=0;
static float previousPressure = 0;

//braking variables
const int RLed_Pin            = 2;
float braking_threshold       = 0.4;
static boolean alreadyBlinked = 0;
static int downcounter        = 0;
static int downcounter_max    = 8;           //number of times the LED will blink once the user stops braking

//Multiplexer
//Adafruit_ADS1015 ads1015(0x4A);           // construct an ads1015 at address 0x4A (ADDR pin must be connected to SDA pin)

Adafruit_ADS1015 ads1015; //the pin ADDR is now connected to the GND
//manual mode variables
volatile boolean flagManual = 0;

//end of SENSORS AND ACTUATORS VARIABLES


//-------------------------------------INITIALIZE FUNCTIONS---------------------------------------------//
//State checkState(boolean _velocity, boolean _rain, boolean _sun);
//float measurePressure();
//void measureLuminosity(int counter);
//float computeMoyenneDesPhototransistors();
//boolean checkVelocity(float _pressure, float _pressure_threshold);
//boolean checkRain();
//boolean checkSun(float _luminosity, float threshold);
//boolean checkBraking();
//float computeVelocityThreshold();
//float computeLuminosityThreshold();
//int modulateGradient(float phototransistorA);
//
//void move2Servos(float desired_angle);
//boolean isVisorOpen();
//boolean isVisorTransparent();
//boolean isVisorOpaque();
//void moveOpen();
//void moveTransparent();
//void moveOpaque(int grad);
//void doSomethingWithTheBrakingLED(boolean _braking);
//
//void startContinuousReadingForPressureSensor();
float getPressure();
//float getTemperature();
//float smoothingExponential(float previousMeasure, float currentMeasure, float smoothingFactor);
//
//void buttonPressed();
//
////end of INITIALIZE FUNCTIONS
//
//boolean LEDcommand = 0;
//int potValue;


//************* START SETUP *******************//

void setup() {
//  //initialize pins
//  pinMode(phototransistorA_pin, INPUT);
//  pinMode(phototransistorB_pin, INPUT);
//  pinMode(phototransistorC_pin, INPUT);
//  pinMode(phototransistorD_pin, INPUT);
//  pinMode(rain_pin, INPUT);
//  pinMode(servoRight_pin, OUTPUT);
//  pinMode(servoLeft_pin, OUTPUT);
//  pinMode(brakingLed_pin, OUTPUT);
  //interrupt pin
//  pinMode(button_pin,INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(button_pin),buttonPressed,RISING);  //configure interrput later

  //initialize Serial monitor
  Serial.begin(9600);
  delay(100);



   //initialize IMU
   if (!IMU.begin()) {
     Serial.println("Failed to initialize IMU!");
   }

   //initialize Temperature and Humidity
   if (!HTS.begin()) {
    Serial.println("Failed to initialize humidity temperature sensor!");
  }

  // //initialize SD card
  // if (!SD.begin(10)) {
  //   Serial.println("initialization failed!");
  //   return;
  // }
  // Serial.println("Initialization of SD card done.");
  // myFile = SD.open("JamiePaik.txt",FILE_WRITE);
  // if (myFile) {
  //   myFile.println("Time  Pressure  Smoothed pressure");
  //   myFile.close();
  // } else {
  //   // if the file didn't open, print an error:
  //   Serial.println("error opening JamiePaik.txt");
  // }

  //initialize pressure sensor
 // startContinuousReadingForPressureSensor();

  //initialize multiplexer
  ads1015.begin();

  //initialize i2c bus
  Wire.begin();

  //initialize Servo motor
//  servoRight.attach(servoRight_pin);
//  servoLeft.attach(servoLeft_pin);
  //move2Servos(30);

  //compute thresholds for the first time
//  velocity_threshold = computeVelocityThreshold();
//  luminosity_threshold = computeLuminosityThreshold();
}

//end of SETUP


//************* START LOOP *******************//

void loop() {
  //Multiplexer values
  int16_t pot1_val, pot2_val, adc2, adc3;
  // Braking value variable
  float acc_x, acc_y, acc_z;
  currentTime= millis();

  //Measure LUMINOSITY every 100ms
//  if(currentTime- previousTimeLuminosity >= luminosityInterval) {
//    measureLuminosity(luminosity_counter);  //luminosity_counter va de 0 à 3
//    luminosity_counter++;

////    if(luminosity_counter == 4) { //every 100 ms we check the values of the luminosity
////      moyenne_phototransistors = computeMoyenneDesPhototransistors();
////      float smoothingFactor = 0.3;
////      moyenne_phototransistors = smoothingExponential(previousMoyenne_phototransistors,moyenne_phototransistors,smoothingFactor);
//
//      phototransistorA = (lum[0][0]+lum[1][0]+lum[2][0]+lum[3][0]) / 4; //compute the moyenne of the front phototransistor
//      // Serial.print(", phototransistor A : ");
//      // Serial.println(phototransistorA);
//      phototransistorA = smoothingExponential(previousPhototransistorA,phototransistorA,smoothingFactor);
//
//      sun = checkSun(moyenne_phototransistors, luminosity_threshold);  //test if the luminosity is higher than the threshold
//      gradient = modulateGradient(phototransistorA);
//
//      // Serial.print("moyenne : ");
//      // Serial.println(moyenne_phototransistors);
//      // Serial.print(", phototransistor A : ");
//      // Serial.println(phototransistorA);
//
//      stateFlag = 1;
//      luminosity_counter = 0;
//      previousMoyenne_phototransistors = moyenne_phototransistors;
//      previousPhototransistorA = phototransistorA;
//    }
//    previousTimeLuminosity =currentTime;
//  }

  //Measure Temperature and humidity every 200ms
  if(currentTime- previousTimeTemperature >= temperatureInterval) {
    float temperature = HTS.readTemperature();
    float humidity    = HTS.readHumidity();
  
    // print each of the sensor values
    Serial.print("Temperature = ");
    Serial.print(temperature);
    Serial.println(" °C");
  
    Serial.print("Humidity    = ");
    Serial.print(humidity);
    Serial.println(" %");
    // print an empty line
    Serial.println();
    previousTimeTemperature = temperatureInterval;
  }

  //Measure PRESSURE every 200ms
//  if(currentTime- previousTimePressure >= pressureInterval) {
////    pressure = getPressure(); 
////    velocity = checkVelocity(pressure, velocity_threshold);                      //test if the velocity is higher than the threshold
//    // Serial.println(velocity);
//
//    // Serial.print("pressure : ");
//    // Serial.print(pressure);
//    // Serial.print(", threshold : ");
//    // Serial.println(velocity_threshold);
//
//    previousTimePressure =currentTime;
//  }
  
//  //Measure RAIN every 500ms
//  if(currentTime- previousTimeRain >= rainInterval) {
//    rain = checkRain();                              //test if there is rain
//    previousTimeRain =currentTime;
//  }

  //Measure 2 POTENTIOMETERS every 1000ms ------------------------------------------------------------------------------------------------
  if(currentTime- previousTimePotentiometer >= PotentiometerInterval) {
//    pot2_val = ads1015.readADC_SingleEnded(0);
////    pot1_val = ads1015.readADC_SingleEnded(1);
////    luminosity_threshold = computeLuminosityThreshold();
//    Serial.print("Pot1: ");
//    Serial.print(pot1_val);
//    
//    Serial.print(" Pot2: ");
//    Serial.print(pot2_val);
//
//    Serial.print(" Pressure: ");
//    Serial.println(pressure);

      //Communication with multiplexer
      int16_t pot0, pot1, adc2, adc3;
      pot0 = ads1015.readADC_SingleEnded(0);
      pot1 = ads1015.readADC_SingleEnded(1);
      delay(10);
      
      //Communication with pressure sensor
      Wire.beginTransmission(pressureAddress);
      Wire.write(readPressureCmd1);
      Wire.write(readPressureCmd2);
      Wire.requestFrom(pressureAddress,2,false); //asks for 2 bytes, then stops
      byte pressure1 = Wire.read();
      byte pressure2 = Wire.read();
      Wire.endTransmission();
      int16_t pressureOutput = pressure1 << 8;
      pressureOutput |= (int16_t)pressure2;
      float pressure = (float)pressureOutput/240; //max : 136.53
      delay(10);

      //Check the measured pressure (without smoothing) + potentiometer values (without mapping)
      Serial.print("Pot0: "); Serial.print(pot0);
      Serial.print(" Pot1: "); Serial.print(pot1);
      Serial.print(" Pressure: ");
      Serial.println(pressure);
    //  Serial.print("AIN2: "); Serial.print(adc2);
    //  Serial.print("AIN3: "); Serial.println(adc3);
      Serial.println(" ");


    
//    Serial.print(", luminosity value : ");
//    Serial.println(moyenne_phototransistors);

    previousTimePotentiometer =currentTime;
  }

  //Measure BRAKING every 100ms
  if(currentTime- previousTimeBraking >= brakingInterval) {
        if ( IMU.accelerationAvailable()) {
          IMU.readAcceleration(acc_x, acc_y, acc_z); // Retrieve Acceleration Values
        }

          if( abs(acc_x) > braking_threshold){
             delay(100);
            digitalWrite(RLed_Pin,LOW);
            delay(100);
            digitalWrite(RLed_Pin,HIGH);
            delay(100);
            digitalWrite(RLed_Pin,LOW);
            delay(100);
            digitalWrite(RLed_Pin,HIGH);
            delay(100);
            digitalWrite(RLed_Pin,LOW);
        }
  }
  

  //recompute state every 100 ms and move motor accordingly
//  if(stateFlag == 1) {
//    state = checkState(velocity,rain,sun);
//
//    // //Verify the state with the serial print
//    // if (state == OPEN){
//    //   Serial.println("State = OPEN");
//    // } else if (state == TRANSPARENT){
//    //   Serial.println("State = TRANSPARENT");
//    // } else {
//    //   Serial.println("State = OPAQUE");
//    // }
//
//    switch (state){
//      case OPEN:
//        if (!isVisorOpen()){                         //test if the visor is open and open it if necessary
//          moveOpen();
//        }
//        break;
//      case TRANSPARENT:
//        if (!isVisorTransparent()){                  //test if the visor is in transparent state and move it if necessary
//          moveTransparent();
//        }
//        break;
//      case OPAQUE:
//        if (!isVisorOpaque()){                       //test if the visor is in opaque state and move it if necessary
//          moveOpaque(gradient);
//        }
//        break;    
//    }
//    stateFlag = 0;
//  }

}

//end of LOOP

//~~~~~~~~~~~~~~ FUNCTIONS ~~~~~~~~~~~~~~~~~~//

State checkState(boolean _velocity, boolean _rain, boolean _sun) {
  boolean open_state = !_velocity && !_rain && !_sun;
  boolean transparent_state = (!_velocity && _rain || _velocity) && !_sun;
  boolean opaque_state = _sun;

  if (open_state){
    return OPEN;
  }else if (transparent_state){
    return TRANSPARENT;
  }else if (opaque_state){
    return OPAQUE;
  }
}

void move2Servos(float desired_angle) {
  //moves the 2 servo motors in opposite directions
  servoRight.write(0 + desired_angle); // par ex 90 + desired_angle
  servoLeft.write(180 - desired_angle);
  angle = desired_angle;
}

float measurePressure() {
  //reads the value of the pressure sensor and filters it
  float _pressure = getPressure();
  float smoothingFactor = 0.2;
  if(_pressure < 0) _pressure = 0;
//  smoothedPressure = smoothingExponential(previousPressure, _pressure, smoothingFactor);
//  previousPressure = smoothedPressure;
  return _pressure;  //normalement we would code smoothedPressure
}

void measureLuminosity(int counter) {
  //reads 4 luminosity
  lum[counter][0] = analogRead(phototransistorA_pin);
  lum[counter][1] = analogRead(phototransistorB_pin);
  lum[counter][2] = analogRead(phototransistorC_pin);
  lum[counter][3] = analogRead(phototransistorD_pin);
}

float computeMoyenneDesPhototransistors() {
  //we might give more weight to the one of phototransistorA !
  int moyenne = 0;
  for(int i=0;i<4;i++) {
    for(int j=0;j<4;j++) {
      moyenne += lum[i][j] / 16;  //moyenne de 16 éléments
      // Serial.print(i);
      // Serial.print(", ");
      // Serial.print(j);
      // Serial.print(": ");
      // Serial.println(lum[i][j]);
    }
  }
  return moyenne;
}

boolean checkVelocity(float _pressure, float _velocity_threshold){
  //determines if the velocity is too high depending on the threshold, pressure sensor, temperature sensor and current state
  if(_pressure > _velocity_threshold) {
    return 1;
  } else {
    return 0;
  }
}

boolean checkRain(){
  //determines if there is rain depending on rain sensor and threshold (+ current state?)

  int rain_value = analogRead(rain_pin);

  Serial.print("rain value: ");
  Serial.println(rain_value);

  if(rain_value < 900) {
    return 1;
  }
  else {
    return 0;
  }


  //CHANGE

  // int rain_sensor_value = analogRead(rain_pin);
  // if (state == OPEN){
  //   if (rain_sensor_value >= rain_threshold){
  //     return true;
  //   } else {
  //     return false;
  //   }
  // } else { 
  //   if (rain_sensor_value <= rain_threshold){                   //if the visor is already closed and there is less rain, wait to be sure
  //     //to be completed
  //   } else {
  //     return true;
  //   }  
  // }
  // //add other states
}

boolean checkSun(float _luminosity_moyenne, float threshold){
  //determines if the luminosity is too high depending on phototransistors and current state
  if(_luminosity_moyenne > threshold) {
    return 1;
  }
  else{
    return 0;
  }
}


  
  // float x,y,z;
  // if (IMU.accelerationAvailable()) {
  //   IMU.readAcceleration(x, y, z);
  // }
  // if(x > braking_threshold) {
  //   return 1;
  // }
  // else {
  //   return 0;
  // }
//}

//float computeVelocityThreshold() {
  //computes the threshold at which the visor should become transparent, based on the value of the pressure potentiometer
  //int pressure_pot_value = ads1015.readADC_SingleEnded(pressurePot_pin);
//  float temperature = getTemperature();
  //if(pressure_pot_value > 4000) pressure_pot_value = 0;
  // Serial.print(pressure_pot_value);
  // Serial.print(" ");
//  int velocity_thresh = map(pressure_pot_value,0,1090,5,25);

//  return pressure_pot_value;
//}

//float computeLuminosityThreshold() {
  //computes the threshold at which the visor should become opaque, based on the value of the luminosity potentiometer
  //int luminosity_pot_value = ads1015.readADC_SingleEnded(luminosityPot_pin);
  //if(luminosity_pot_value > 4000) luminosity_pot_value = 0;
  // Serial.println(luminosity_pot_value);
  //int lum_threshold = map(luminosity_pot_value, 0, 1090, 10, 900); //maps the value of the potentiometer to a value between 300 and 900

  //return lum_threshold;
//}

int modulateGradient(float _phototransistorA) {
  int g = map(_phototransistorA, phototransistor_value_min, phototransistor_value_max, opaque_min, opaque_max);
  return g;
}

boolean isVisorOpen(){
  //returns true if the visor is open, false otherwise
  if (angle == angleOpen){
    return true;
  } else {
    return false;
  }
}

boolean isVisorTransparent(){
  //returns true if the visor is in the transparent state, false otherwise
  if (angle == angleTransparent){
    return true;
  } else {
    return false;
  }
}

boolean isVisorOpaque(){
  //returns true if the visor is in the opaque state, false otherwise
  if (angle != angleOpen and angle != angleTransparent){
    return true;
  } else {
    return false;
  }
}

void moveOpen(){
  //open the visor
  move2Servos(angleOpen);
}

void moveTransparent(){
  //move the visor to the transparent state
  move2Servos(angleTransparent);
}

void moveOpaque(int grad){
  //move the visor to the opaque state
  move2Servos(angleOpaque);
  //move2Servos(grad);
}

void doSomethingWithTheBrakingLED(boolean _braking) {
  if(_braking && !alreadyBlinked) {  //if alreadyBlinked = 1, on n'entre pas ici
      digitalWrite(brakingLed_pin, HIGH);
      alreadyBlinked = 1;
      downcounter = downcounter_max;
    }
    else if (!alreadyBlinked && downcounter) { //if it's not braking anymore but it should still blink a few times
      digitalWrite(brakingLed_pin, HIGH);
      alreadyBlinked = 1;
      downcounter--;
    }
    else {
      digitalWrite(brakingLed_pin, LOW);
      alreadyBlinked = 0;
    }
}

void startContinuousReadingForPressureSensor() {
  //sends a message to pressure sensor to start continuous reading
  Wire.beginTransmission(pressureAddress);
  Serial.println("Begin transmission with pressure sensor");
  Wire.write(readPressureCmd1);
  Wire.write(readPressureCmd2);
  int a = Wire.endTransmission();
  
}

float getPressure() {
  //reads pressure from pressure sensor
  Wire.requestFrom(pressureAddress,2,false); //asks for 2 bytes, then stops
  byte pressure1 = Wire.read();
  byte pressure2 = Wire.read();

  int16_t pressureOutput = pressure1 << 8;
  pressureOutput |= (int16_t)pressure2;
  float pressureMeasured = (float)pressureOutput/240; //max : 136.53
  return pressureMeasured;
}

float getTemperature() {
  return 20;
}

float smoothingExponential(float previousMeasure, float currentMeasure, float smoothingFactor) {
  float alpha = smoothingFactor;
  float smooth = alpha * currentMeasure + (1-alpha) * previousMeasure;
  return smooth;
}

void buttonPressed(void) {
  flagManual = 1;
  Serial.println("Button pressed!");
  //delay(600);
}
