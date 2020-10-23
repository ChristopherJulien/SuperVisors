# SuperVisors
#test geraud

//Structure du code informatique


bool speed  //0 if below speed limit, otherwise 1
bool rain  //0 if below no rain, otherwise 1
bool sun  //0 if below no sun, otherwise 1
bool measureValues = 0
bool manualMode = 0
volatile char State = ‘open’  //use enum

void loop {

if(measureValues == 1 && manualMode == 0) {
	readSensorValues()
	checkIfValuesAboveThresholds()
	changeSpeedRainSunValues()
	getState(speed, rain, sun)
measureValues = 0
}

switch State {
case ‘open’
		control motor
case ‘transparent’
		control motor
case ‘opaque’
		control motor
}
}



char getState(speed, rain, sun) {
	tableau de Géraud
	return char State
}

void callback() {
	measureValues = 1
}


void manualModeInterrupt() {
	//manual mode on/off
	if(button held for 1 second) {
		manualMode = !manualMode
	}
	else {
//in manual mode, change State
		if(manualMode == 1 && State = ‘open’) {
			State = ‘transparent’
		}
if(manualMode == 1 && State = ‘transparent’) {
			State = ‘opaque
		}
		if(manualMode == 1 && State = ‘opaque’) {
			State = ‘open’
		}
	}

