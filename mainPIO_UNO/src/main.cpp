#include <Arduino.h>

const int ph_A    = A3;
const int ph_B    = A2;
const int ph_C    = A1;
const int ph_D    = A0;

int val_ph_A    = 0;
int val_ph_B    = 0;
int val_ph_C    = 0;
int val_ph_D    = 0;


int iterations = 100;
int i = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(ph_A, INPUT);
  pinMode(ph_B, INPUT);
  pinMode(ph_C, INPUT);
  pinMode(ph_D, INPUT);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  val_ph_A = analogRead(ph_A);
  val_ph_B = analogRead(ph_B);
  val_ph_C = analogRead(ph_C);
  val_ph_D = analogRead(ph_D);


if(i< iterations){
  Serial.println("");
  Serial.print("A: ");
  Serial.print(val_ph_A);
  Serial.print("   B: ");
  Serial.print(val_ph_B);
  Serial.print("   C: ");
  Serial.print(val_ph_C);
  Serial.print("   D: ");
  Serial.print(val_ph_D);
  Serial.println("");
  delay(500);
  // i++;
  }

}