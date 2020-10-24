#include <Arduino.h>


const int ph_FRONT_TOP    = A1;
const int ph_FRONT_BOTTOM = A0;
const int ph_TOP_RIGHT    = A5;
const int ph_TOP_LEFT     = A2;
const int ph_TOP_FORWARD  = A4;
const int ph_TOP_BACKWARD = A3;

int val_ph_FRONT_TOP    = 0;
int val_ph_FRONT_BOTTOM = 0;
int val_ph_TOP_RIGHT    = 0;
int val_ph_TOP_LEFT     = 0;
int val_ph_TOP_FORWARD  = 0;
int val_ph_TOP_BACKWARD =0;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(ph_FRONT_TOP, INPUT);
  pinMode(ph_FRONT_BOTTOM, INPUT);

  pinMode(ph_TOP_RIGHT, INPUT);
  pinMode(ph_TOP_LEFT, INPUT);
  pinMode(ph_TOP_FORWARD, INPUT);
  pinMode(ph_TOP_BACKWARD, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  val_ph_FRONT_TOP = analogRead(ph_FRONT_TOP);
  val_ph_FRONT_BOTTOM = analogRead(ph_FRONT_BOTTOM);
  val_ph_TOP_RIGHT = analogRead(ph_TOP_RIGHT);
  val_ph_TOP_LEFT = analogRead(ph_TOP_LEFT);
  val_ph_TOP_FORWARD = analogRead(ph_TOP_FORWARD);
  val_ph_TOP_BACKWARD = analogRead(ph_TOP_BACKWARD);


String format = "%-20s %5d\n";
Serial.println("");
System.out.format(format, "FT", val_ph_FRONT_TOP);
System.out.format(format, "\t FB", val_ph_FRONT_BOTTOM);
System.out.format(format, "\t TF", val_ph_TOP_FORWARD);
System.out.format(format, "\t TB", val_ph_TOP_BACKWARD);
System.out.format(format, "\t TR", val_ph_TOP_RIGHT);
System.out.format(format, "\t TL", val_ph_TOP_LEFT);
Serial.println("");

  // Serial.println("");
  // Serial.print("FT: ");
  // Serial.print(val_ph_FRONT_TOP);
  // Serial.print("   FB: ");
  // Serial.print(val_ph_FRONT_BOTTOM);
  // Serial.print("   TF: ");
  // Serial.print(val_ph_TOP_FORWARD);
  // Serial.print("   TB: ");
  // Serial.print(val_ph_TOP_BACKWARD);
  // Serial.print("   TR: ");
  // Serial.print(val_ph_TOP_RIGHT);
  // Serial.print("   TL: ");
  // Serial.print(val_ph_TOP_LEFT);
  // Serial.println("");


  delay(500);
  

}