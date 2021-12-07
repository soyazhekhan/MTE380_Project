#include <FastPID.h>
#include <Servo.h>

#define Ain1 5
#define Ain2 6
#define Bin1 10
#define Bin2 9

int reds[5] = {500, 500, 500, 500, 500};

int woods[5] = {0, 0, 0, 0, 0};
/*
 * A0 -> left
 * A5 -> right
 */

Servo intakeServo;

bool testEnd = false;

const int OPEN_POS = 165;
const int CLOSE_POS = 65;

int baseSpeed = 39;
int offset = 0;
//int baseSpeedRight = baseSpeed; //50
//int baseSpeedLeft = baseSpeed + offset; //50

int middle = 1500;
float KP = 0.065; //0.065
float slow_KP = 0.065;
float KI = 0; //0
float KD = 0.007; //0.0055
float slow_KD = 0.0055;
float Hz = 200; //10
int output_bits = 8;
bool output_signed = true;

FastPID slow_linePID(slow_KP, KI, slow_KD, Hz, output_bits, output_signed);
FastPID linePID(KP, KI, KD, Hz, output_bits, output_signed);
unsigned long loopTime;
unsigned long blueTime;

unsigned long speedTime;

long bullseyeTime = -1;
int state = 0;
int internState = 0;
int oldState = 0;

void setup() {
  Serial.begin(9600);
  //motor driver
  pinMode(Ain1, OUTPUT);  //Ain1
  pinMode(Ain2, OUTPUT);  //Ain2
  pinMode(Bin1, OUTPUT);  //Bin1
  pinMode(Bin2, OUTPUT);  //Bin2
  //straight(255, 20000);
// setLeft(120);
// setRight(40);
//  delay(200000);
  //servo initialization
  intakeServo.attach(8);
  intakeServo.write(CLOSE_POS);
  calibrateSensors();
  for(int i=0; i<5; i++){
    Serial.print(reds[i]);
    Serial.print('\t');
    Serial.println(woods[i]);
  }
  loopTime = millis();
  blueTime = millis();
  delay(3000);

  pinMode(LED_BUILTIN, OUTPUT);
  startLED(5);
  bullseyeTime = millis()+1900;
  if(testEnd){
    state = 3;
    oldState = 3;
    bullseyeTime = 0;
    baseSpeed = 45;
  }
  speedTime = millis()+1500;
}

int sensorVals[5] = {0, 0, 0, 0, 0};
long averageS = 0;
long sumS = 0;
int diffOutput = 0;
int colourCounter = 0;

int16_t curLinePos = 0;
int16_t oldLinePos = 0;
int maxDiff = 4000;

bool pickedUp = false;
bool droppedOff = false;
bool lookForRed = false;

int zoneThresh[2] = {500, 1500};
int redThresh[2] = {600, 1000};
int lineThresh = 300;
long zoneTime = 0;

bool ledOn = true;
 
void loop() {
  if(baseSpeed == 38 && millis() > speedTime){
    baseSpeed = 40;
  }
  if(millis()-loopTime >= 5){
    stateCheck();
    averageS = 0;
    sumS = 0;
    if(state == 1 && millis() <= zoneTime){
      for(int i=0; i<3; i++){
        if(sensorVals[i] >= lineThresh){
          averageS += (long)(sensorVals[i])*(i*1000);
          sumS += sensorVals[i];
        }
      }
    }else if(state <= 1){
      if(ledOn){
        digitalWrite(LED_BUILTIN, LOW);
        ledOn = false;
      }
      for(int i=0; i<5; i++){
        if(sensorVals[i] >= lineThresh){
          averageS += (long)(sensorVals[i])*(i*1000);
          sumS += sensorVals[i];
        }
      }
    }
    else if(state == 3 && millis() <= zoneTime){
      for(int i=2; i<5; i++){
        if(sensorVals[i] >= lineThresh){
          averageS += (long)(sensorVals[i])*(i*1000);
          sumS += sensorVals[i];
        }
      }
    }else if(state == 3 && millis() > zoneTime){
      if(middle != 1600){
        middle = 1600;
        curLinePos = 2000;
        oldLinePos = 2000;
        linePID.clear();
        slow_linePID.clear();
      }
      for(int i=0; i<5; i++){
        if(sensorVals[i] >= lineThresh){
          averageS += (long)(sensorVals[i])*(i*1000);
          sumS += sensorVals[i];
        }
      }
    }else if(state == 4 && millis() <= zoneTime){
      for(int i=0; i<3; i++){
        if(sensorVals[i] >= lineThresh){
          averageS += (long)(sensorVals[i])*(i*1000);
          sumS += sensorVals[i];
        }
      }
    }else if(state == 4 && millis() > zoneTime){
      for(int i=0; i<5; i++){
        if(sensorVals[i] >= lineThresh){
          averageS += (long)(sensorVals[i])*(i*1000);
          sumS += sensorVals[i];
        }
      }
      
      if(middle != 2000){
        intakeServo.write(OPEN_POS);
        middle = 2000;
        linePID.clear();
        slow_linePID.clear();
      }
    }else if(state == 6 && millis() <= zoneTime){
      for(int i=1; i<5; i++){
        if(sensorVals[i] >= lineThresh){
          averageS += (long)(sensorVals[i])*(i*1000);
          sumS += sensorVals[i];
        }
      }
    }else if(state == 7){
      for(int i=0; i<4; i++){
        if(sensorVals[i] >= lineThresh){
          averageS += (long)(sensorVals[i])*(i*1000);
          sumS += sensorVals[i];
        }
      }
    }else if(state == 9 && millis() <= zoneTime){
      for(int i=1; i<5; i++){
        if(sensorVals[i] >= lineThresh){
          averageS += (long)(sensorVals[i])*(i*1000);
          sumS += sensorVals[i];
        }
      }
    }else{
      if(middle != 2000){
        middle = 2000;
        linePID.clear();
        slow_linePID.clear();
        curLinePos = 2000;
        oldLinePos = 2000;
      }
      digitalWrite(LED_BUILTIN, LOW);
      for(int i=0; i<5; i++){
        if(sensorVals[i] >= lineThresh){
          averageS += (long)(sensorVals[i])*(i*1000);
          sumS += sensorVals[i];
        }
      }
    }
    
    if(sumS != 0){
      curLinePos = averageS/sumS;
      if(state != oldState && oldLinePos != 0 && oldLinePos != -1 && abs(curLinePos-oldLinePos) > maxDiff){
        curLinePos = oldLinePos;
      }
    }
    int output = 0;
    if(state<2 || state >= 4){
      output = slow_linePID.step(middle, curLinePos);
    }else{
      output = linePID.step(middle, curLinePos);
    }
    setRight(constrain(baseSpeed+output, -baseSpeed, baseSpeed)); //-40 70
    setLeft(constrain(baseSpeed-output, -baseSpeed, baseSpeed)); //-40 70
    loopTime = millis();
    oldLinePos = curLinePos;
    oldState = state;
  }else if(millis()-loopTime <= 4){
    stateCheck();
  }
}

void readSensors(){
  sensorVals[0] = normalizeVal(analogRead(A0), reds[0], woods[0]);
  sensorVals[1] = normalizeVal(analogRead(A1), reds[1], woods[1]);
  sensorVals[2] = normalizeVal(analogRead(A2), reds[2], woods[2]);
  sensorVals[3] = normalizeVal(analogRead(A3), reds[3], woods[3]);
  sensorVals[4] = normalizeVal(analogRead(A4), reds[4], woods[4]); 
}

void calibrateSensors(){
  for(int i=0; i<1000; i++){
    sensorVals[0] = analogRead(A0);
    sensorVals[1] = analogRead(A1);
    sensorVals[2] = analogRead(A2);
    sensorVals[3] = analogRead(A3);
    sensorVals[4] = analogRead(A4);
    for(int j=0; j<5; j++){
      reds[j] = min(reds[j], sensorVals[j]-20);
      woods[j] = max(woods[j], sensorVals[j]+20);
    }
    delay(10);
  }
}

int normalizeVal(int rawVal, int minVal, int maxVal){
  return 1000 - (rawVal-minVal)*(1000.0)/(maxVal-minVal);
}

void stateCheck(){
  readSensors();
  //overlaping zone
  if(state < 2 && millis() >= bullseyeTime+1250 && (sensorVals[0] >= zoneThresh[0] && sensorVals[4] >= zoneThresh[0])){
    if(internState == 0){
      straight(50, 100);
      internState = 1;
    }else{
      state = 2;
      straight(45, 250);
      baseSpeed = 45;
      internState = 0;
      maxDiff = 700;
      slow_linePID.clear();
      linePID.clear();
      curLinePos = 2000;
      oldLinePos = -1;
      bullseyeTime = millis()+2500;
    }
  }
  //first corner zone
  else if(state == 0 && millis() >= bullseyeTime && millis <= bullseyeTime+500 && sensorVals[4] >= zoneThresh[0] && (sensorVals[0] >= zoneThresh[0] || sensorVals[1] >= zoneThresh[0] || sensorVals[2] >= zoneThresh[0])){
    state = 1;
    digitalWrite(LED_BUILTIN, HIGH);
    ledOn = true;
    zoneTime = millis()+600;
    slow_linePID.clear();
    linePID.clear();
    //straight(-15, 1000);
    //change sensors used for line following to exclude right sensor until state is 2
  }
  //second corner zone
  else if(state == 2 && (sensorVals[0] >= zoneThresh[0] && (sensorVals[2] >= zoneThresh[0] || sensorVals[3] >= zoneThresh[0]) && sensorVals[4] <= zoneThresh[0])){
    state = 3;
    //straight(-15, 1000);
    zoneTime = millis()+1500;
    middle = 2400;
    curLinePos = 2400;
    oldLinePos = 2400;
    slow_linePID.clear();
    linePID.clear();
    //change sensors used for line following to exclude left sensor for fixed time
  }
  //skip dropoff zone
  else if(state >= 2 && state < 4 && millis() >= bullseyeTime && ((sensorVals[4] >= zoneThresh[0] && (sensorVals[1] >= zoneThresh[0] || sensorVals[2] >= zoneThresh[0])) || (sensorVals[3] >= zoneThresh[0] && (sensorVals[0] >= zoneThresh[0] || sensorVals[1] >= zoneThresh[0])) )){
    state = 4;
    zoneTime = millis()+1000;
    bullseyeTime = millis()+1000;
    baseSpeed = 40;
    maxDiff = 1500;
    middle = 1600;
    curLinePos = 1600;
    oldLinePos = 1600;
    slow_linePID.clear();
    linePID.clear();
    //change sensors used for line following to exclude right sensor for fixed time
  }
  //bullseye
  else if(millis() > bullseyeTime && state == 4 && (sensorVals[0] >= zoneThresh[0] && sensorVals[4] >= zoneThresh[0])){
    state = 5;
    intake();
    bullseyeTime = millis()+500;
    baseSpeed = 40;
    maxDiff = 900;
    linePID.clear();
    slow_linePID.clear();
    //    straight(-10, 1000);
  }
  //dropoff
  else if(state == 5 && millis() >= bullseyeTime && (sensorVals[0] >= zoneThresh[0] && (sensorVals[2] >= zoneThresh[0] || sensorVals[3] >= zoneThresh[0]))){
    state = 6;
    dropOff();
    slow_linePID.clear();
    linePID.clear();
    maxDiff = 700;
    zoneTime = millis() + 1250;
    bullseyeTime = millis()+ 2000;

  }
  //overlapping zone back
  else if(state < 8 && state >=6 && millis() >= bullseyeTime && (sensorVals[0] >= zoneThresh[0] && sensorVals[4] >= zoneThresh[0])){
    if(internState == 0){
      straight(50, 100);
      internState = 1;
    }else{
      state = 8;
      straight(50, 100);
      baseSpeed = 40;
      slow_linePID.clear();
      linePID.clear();
      internState = 0;
      bullseyeTime = millis()+1500;
      maxDiff = 1500;
    }
  }
  //second corner zone back
  else if(state == 6 && (sensorVals[4] >= zoneThresh[0] && (sensorVals[1] >= redThresh[0] || sensorVals[2] >= redThresh[0]) && sensorVals[0] <= zoneThresh[0])){
    state = 7;
    //straight(-15, 1000);
    //change sensors used for line following to exclude right sensor until state is 2
  }
  //stop at end
  else if(millis() > bullseyeTime && state >= 8 && (sensorVals[0] >= 450 && sensorVals[1] >= 450 && sensorVals[2] >= 450 && sensorVals[3] >= 450 && sensorVals[4] >= 450)){
    state = 10;
    straight(-15, 2000);
    setRight(-80);
    setLeft(50);
    delay(10000);
    setRight(10);
    setLeft(-10);
    delay(500);
    straight(0, 10000);
  }
  //first corner zone back
  else if(state == 8 && (sensorVals[0] >= zoneThresh[0] && (sensorVals[2] >= redThresh[0] || sensorVals[3] >= redThresh[0]) && sensorVals[4] <= zoneThresh[0])){
    state = 9;
    //straight(-15, 1000);
    zoneTime = millis()+500;
    //change sensors used for line following to exclude left sensor for fixed time
  }
}

void throughBullsEye(){
  setLeft(55);
  setRight(60);
  delay(200);
  while(sensorVals[2] <= 600){
    readSensors();
  }
  blueTime = millis();
  linePID.clear();
  lookForRed = true;
}
void driveThroughZone(){
    setRight(baseSpeed-5);
    setLeft(baseSpeed-5);
    delay(100);
    while(sensorVals[0] <= zoneThresh[0]-50 && sensorVals[4] <= zoneThresh[0]-50);
    delay(200);
//    straight(-10, 1000);
    linePID.clear();
}

void intake(){
  //straight(40, 50)
  intakeServo.write(CLOSE_POS);
  straight(0, 200);
  //straight(-15, 200);
  setLeft(0);
  setRight(-30);
  delay(200);
  straight(0, 1000);
  //straight(-40, 700);
  setRight(-45);
  delay(450); //450
  setRight(25);
  delay(200);
  turnUntilLine();
  slow_linePID.clear();
  linePID.clear();
  curLinePos = 2000;
  oldLinePos = 2000;
}

void turnUntilLine(){
  setRight(-35);
  setLeft(35);
  delay(100);
  int sensorCounter = 4;
  while(sensorCounter > 1){
    readSensors();
    if(sensorVals[sensorCounter] >= zoneThresh[0]-50){
      sensorCounter--;
    }
  }
  setRight(25);
  setLeft(0);
  delay(1000);
}

void dropOff(){
  straight(-25, 100);
  setRight(50);
  delay(150);
  setRight(-20);
  delay(200);
  intakeServo.write(OPEN_POS);
  delay(200);
  straight(-40, 200);
  straight(20, 50);
  setRight(-50);
  delay(150);
  setRight(20);
  intakeServo.write(CLOSE_POS);
}

void setRight(int speed){
  if(speed > 0){
    analogWrite(Ain1, speed);
    analogWrite(Ain2, LOW);
  }else{
    speed = -speed;
    analogWrite(Ain1, LOW);
    analogWrite(Ain2, speed);
  }
}

void setLeft(int speed){
  if(speed > 0){
    analogWrite(Bin1, speed*3);
    analogWrite(Bin2, LOW);
  }else{
    speed = -speed;
    analogWrite(Bin1, LOW);
    analogWrite(Bin2, speed*3);
  }
}

void straight(int speed, int timeDelay){
  setRight(speed);
  setLeft(speed);
  delay(timeDelay);
}

void startLED(int startDelay){
  for(int i=0; i<startDelay; i++){
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }
}
