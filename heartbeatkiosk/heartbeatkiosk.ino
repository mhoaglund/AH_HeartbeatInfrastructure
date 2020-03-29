//Pin 5 is motor speed.
//Reversing the outputs of 10 and 11 control direction.

int currentBPM = 20;
int basePWM = 70;
int maxPWM = 200;
int basalPWM = 50;
long checktime = 35;
long previousMillis = 0;
double factor = 0.5;

long beatTolerance = 250;
int beatThreshold = 350;
int beatValue = 0;
long beatTimeStamp = 0;
long maxBeatTime = 8000;
long downTime = 0;
long maxDownTime = 2000;
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
  digitalWrite(10, LOW);
  digitalWrite(11, HIGH);
  analogWrite(5, basalPWM);
  clearTrio();
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
    // Serial.print(sensorValue, DEC);
    // Serial.print("::");
    // Serial.println(average);
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
       //We've been in a low state for more than maxdowntime/1000 seconds. End visitor session and go back to default behavior.
       cycleSession(false);
       clearTrio();
      }
    }
  }
  if(temp != beatState){
    //Process change in state. 
    //Start by measuring change in time:
    if(temp == true){
       digitalWrite(13, HIGH);
       downTime = 0;
       long lastBeatDuration = _timestamp - beatTimeStamp;
       if(!isInSession){
         if(awaitTrio(lastBeatDuration) == true){
           cycleSession(true);
         }
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
      int pwmval = map(currentBPM, 45, 150, basePWM, maxPWM);
      analogWrite(5, pwmval);
  } else {
      analogWrite(5, basalPWM);
  }

}

//Starting or stopping a visitor session
void cycleSession(bool endstate){
  isInSession = endstate;
  if(endstate){
    Serial.println("New Session");
  } else {
    clearTrio();
    Serial.println("Ending Session");
  }
}

long trio[3];  

//We need to filter out env. disturbances.
//We know it's a heartbeat if we see three of them in repeating time.
bool awaitTrio(long timing){
  marchArray(timing, 3, trio);
  byte emptycount = 0;
  //Make sure we don't get a false positive from 0s
  for (int x = 0; x < 3; x++) {
    if(trio[x] == 0){
      emptycount++;
    }
    //We have two beats but they're more than 3 seconds apart. Unrelated.
    if(trio[x] > 3000){
      return false;
    }
  }
  if(emptycount == 0){
    long firstdiff = abs(trio[0]-trio[1]);
    long seconddiff = abs(trio[1]-trio[2]);
    long diffdiff = abs(firstdiff - seconddiff);
    if(firstdiff < beatTolerance && seconddiff < beatTolerance && diffdiff < beatTolerance){
      Serial.println("Got three repeating beats.");
      Serial.print(trio[0]);
      Serial.print(" : ");
      Serial.print(trio[1]);
      Serial.print(" : ");
      Serial.print(trio[2]);
      Serial.println("Diffs: ");
      Serial.print(firstdiff);
      Serial.print(" :: ");
      Serial.print(seconddiff);
      return true;
    } else {
      return false;
    }
  }
}

void clearTrio(){
  for (int x = 0; x < 3; x++) {
    trio[x] = 0;
  }
}

void marchArray(long _value, int _arrsize, long _arr[]){
  for (int x = 0; x < _arrsize-1; x++) {
    _arr[x] = _arr[x+1];
  }
  _arr[_arrsize-1] = _value;
}

void updateRunningAverage(int _input, int _avgarr[], int _samples, int& _targetavg, int& _targetind, int& _targettotal){
  _targettotal = _targettotal - _avgarr[_targetind];
  _avgarr[_targetind] = _input;
  _targettotal = _targettotal + _avgarr[_targetind];
  _targetind = _targetind + 1;

  if (_targetind >= _samples) {
    _targetind = 0;
  }

  _targetavg = _targettotal / _samples;
}

void clearAvg(int _avgarr[], int _arrsize){
  for (int thisReading = 0; thisReading < _arrsize; thisReading++) {
    _avgarr[thisReading] = 0;
  }
}
