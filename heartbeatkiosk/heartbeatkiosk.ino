//Pin 5 is motor speed.
//Reversing the outputs of 10 and 11 control direction.

int currentBPM = 20;
int basePWM = 55;
long checktime = 35;
long previousMillis = 0;
double factor = 0.5;

int beatThreshold = 350;
int beatValue = 0;
long beatTimeStamp = 0;
long maxBeatTime = 8000;
long downTime = 0;
long maxDownTime = 5000;
bool beatState = false;
int BPMinc = 1;

bool isInSession = false;

//Smoothing Variables:
const int numReadings = 20;

int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;  
int delta = 0;
int sense_threshold = 75;

int inputPin = A0;

void setup() {
Serial.begin(9600);

for (int thisReading = 0; thisReading < numReadings; thisReading++) {
  readings[thisReading] = 0;
}
  
pinMode(5, OUTPUT);
pinMode(10, OUTPUT);
pinMode(11, OUTPUT);
pinMode(13, OUTPUT);
digitalWrite(10, LOW); //forward
digitalWrite(11, HIGH); //forward
analogWrite(5, 75); //pin 3 is PWM, 178/255 = (about) 70% speed. Max is 255.

}

void loop() {
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis > checktime) {
    //Check for heartbeat state and possible change flag.
    previousMillis = currentMillis; 
    int sensorValue = analogRead(inputPin);
    updateRunningAverage(sensorValue, readings, numReadings, average, readIndex, total);
    //Average is now set.
    delta = sensorValue - average;
    Serial.print(sensorValue, DEC);
    Serial.print("::");
    Serial.println(average);
    //updateBeatState(sensorValue, currentMillis, beatThreshold);
    updateBeatState(delta, currentMillis, sense_threshold);
  }
}

void updateBeatState(int _input, long _timestamp, int _threshold){
  //Delta between most recent and second most recent analog value comes in. 350+ is mid-beat, 350- is nonbeat.
  beatValue = _input;
  bool temp = false;
  //Upper bound to reduce false positives from changing lighting conditions
  if(_input > _threshold){
    temp = true;
  }
  if(_input <= _threshold){
    temp = false;
    downTime = _timestamp;
    if(isInSession){
       if(downTime - beatTimeStamp >= maxDownTime){
       //We've been in a low state for more than 15 seconds. End visitor session and go back to default behavior.
       cycleSession(false);
      }
    }
  }
  if(temp != beatState){
    //Process change in state. 
    //Start by measuring change in time:
    if(temp == true){
       digitalWrite(13, HIGH);
       downTime = 0; //Reset downtime
       long lastBeatDuration = _timestamp - beatTimeStamp;
       //The time between the most recent beat and the current one is more than 8 seconds.
       //Must be a new visitor or a new session.
       //if(lastBeatDuration > maxBeatTime || beatTimeStamp == 0){
       //  cycleSession(true);
       //}
       //If there's a beat, there's a visitor.
       //TODO: debounce this by waiting for 2 or 3 beats to get recorded before activating session.
       if(!isInSession){
        cycleSession(true);
       }
       beatTimeStamp = _timestamp;
       integrate(lastBeatDuration);
    }
    if(temp == false){
      digitalWrite(13, LOW);
      currentBPM -= BPMinc;
    }
  }
  beatState = temp;
  updateRPM();
}

//Take in a beat duration and turn it into a BPM value, then integrate with a cached BPM value.
void integrate(long timing){
  int myBPM = 60000 / timing;
  if(myBPM > 190){
    myBPM = 90;
  }
  Serial.println(myBPM);
  if(myBPM > currentBPM){
    if(myBPM < 120){
      currentBPM = myBPM;
    } else{
      currentBPM = 120;
    }
  } else {
    if(currentBPM > 20){
      currentBPM -= BPMinc;
    }
  }
}

void updateRPM(){
  //Take in BPM value, map to PWM range
  if(isInSession){
      int pwmval = map(currentBPM, 35, 150, basePWM, 255);
      analogWrite(5, pwmval);
  } else {
      analogWrite(5, 0);
  }

}

//Starting or stopping a visitor session
void cycleSession(bool endstate){
  isInSession = endstate;
  if(endstate){
    Serial.println("New Session");
  } else {
    Serial.println("Ending Session");
  }
}

void updateRunningAverage(int _input, int _avgarr[], int _samples, int& _targetavg, int& _targetind, int& _targettotal){
  _targettotal = _targettotal - _avgarr[_targetind];
  _avgarr[_targetind] = _input;
  // add the reading to the total:
  _targettotal = _targettotal + _avgarr[_targetind];
  // advance to the next position in the array:
  _targetind = _targetind + 1;

  // if we're at the end of the array...
  if (_targetind >= _samples) {
    // ...wrap around to the beginning:
    _targetind = 0;
  }

  // calculate the average:
  _targetavg = _targettotal / _samples;
}

void clearAvg(int _avgarr[], int _arrsize){
  for (int thisReading = 0; thisReading < _arrsize; thisReading++) {
    _avgarr[thisReading] = 0;
  }
}

//TODO: set up a running average of BPM values so we can listen for a ramp-up.
