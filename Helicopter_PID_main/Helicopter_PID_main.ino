#include <LiquidCrystal.h>
#include <NewPing.h>
#include <Bounce2.h>

//
// The struct for a MA profile.
//
#define MAX_NUMBER_MA_PROFILES 5

typedef struct
{
  int* val;
  int window;
  int sum;
  int index;
  int* valueArrayPointer;
} movingAverageData;


//
// LCD
//
const int rs = 12, en = 11, d4 = 2, d5 = 3, d6 = 4, d7 = 5;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

unsigned long lastLCD_Update = 0;
unsigned long LCD_Update_Time = 100;

//
// PID
//
typedef struct
{
  float P;
  float I;
  float D;
} PID;

int setpoint = 0;
int lastSetPoint = 0;
PID myPID = {2, 3, 0.2};

double pid_error = 0;
double pid_error_sum = 0;
double pid_last_error = 0;
double pid_output = 0;
double pid_Iterm = 0;
double pid_outMax = 255;
double pid_outMin = 0;
double pid_lastInput = 0;
unsigned long lastTime = 0;

const int B0_select = 6;
const int B1_P      = 7;
const int B2_I      = 8;
const int B3_D      = 13;

Bounce B0_select_debounce = Bounce();
Bounce B1_P_debounce      = Bounce();
Bounce B2_I_debounce      = Bounce();
Bounce B3_D_debounce      = Bounce();

bool B1_P_lastState = false;
bool B2_I_lastState = false;
bool B3_D_lastState = false;

float PID_AdjustStep = 0.05;

const int setpointSelectPin = A0;

unsigned sampleTime = 1000; // 1 second.

//
// Distance sensor
//

//uint8_t heightOffset = 4; //  The inital value is 6 centimeters above the ground
int measuredHeight = 0;
int lastMeasuredHeight = 0;
int measurementErrorMargin = 2;

const int echo        = A1;
const int trig        = A2;
const int maxDistance = 100; // In centimeters
NewPing sonar(trig, echo, maxDistance);


//
// Helicopter
//
const int helicopterPin = 10;
const int PWM_Reference_Pin = 9;
uint8_t PWM = 0;

void setup()
{
  Serial.begin(115200);

  lcd.begin(16, 2);
  ShowValues();

  pinMode(setpointSelectPin, INPUT);

  pinMode(B0_select , INPUT);
  pinMode(B1_P      , INPUT);
  pinMode(B2_I      , INPUT);
  pinMode(B3_D      , INPUT);

  B0_select_debounce.attach(B0_select);
  B0_select_debounce.interval(1); // interval in ms

  B1_P_debounce.attach(B1_P);
  B1_P_debounce.interval(1); // interval in ms

  B2_I_debounce.attach(B2_I);
  B2_I_debounce.interval(1); // interval in ms

  B3_D_debounce.attach(B3_D);
  B3_D_debounce.interval(1); // interval in ms

  pinMode(helicopterPin, OUTPUT);
  pinMode(PWM_Reference_Pin, OUTPUT);


  /*float sampleTimeInSeconds = sampleTime / 1000;

    myPID.P = myPID.P;
    myPID.I = myPID.I * sampleTimeInSeconds;
    myPID.D = myPID.D / sampleTimeInSeconds;*/
}

void loop()
{
  lastMeasuredHeight = measuredHeight;
  measuredHeight = sonar.ping_cm();
  measuredHeight = movingAverage(&measuredHeight, 20); // Update measuredHeight.
  
  if ((lastMeasuredHeight + measurementErrorMargin) < measuredHeight)
  {
    measuredHeight = lastMeasuredHeight + measurementErrorMargin;
  }
  else if ((lastMeasuredHeight - measurementErrorMargin) > measuredHeight)
  {
    measuredHeight = lastMeasuredHeight - measurementErrorMargin;
  }

  //measuredHeight = measuredHeight - heightOffset;

  HandleButtons();
  HandlePotentioMeter();

  Handle_PID();
  Apply_PID();

  ShowValues();

  //Serial.print("setpoint: ");
  Serial.print(setpoint);
  Serial.print("\t");

  //Serial.print("measuredHeight: ");
  Serial.println(measuredHeight);
  //Serial.print("\t");

  //Serial.print("PWM: ");
  //Serial.println(PWM);

  //Serial.println();

  //delayMicroseconds(10);
  //Serial.println();
}

void Handle_PID()
{
  // http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

  /*How long since we last calculated*/
  unsigned long now = millis();
  double timeChange = (double)(now - lastTime) / 1000;

  /*Compute all the working error variables*/
  pid_error = setpoint - measuredHeight;
  pid_error /= 28.57; // cm -> volt

  //Serial.print("pid_error: ");
  //Serial.println(pid_error);

  //pid_error_sum += (pid_error * timeChange);

  pid_Iterm += myPID.I * pid_error * timeChange;
  //Serial.print("pid_Iterm: ");
  //Serial.println(pid_Iterm);
  if (pid_Iterm > pid_outMax)
  {
    pid_Iterm = pid_outMax;
  }
  else if (pid_Iterm < pid_outMin)
  {
    pid_Iterm = pid_outMin;
  }

  //double dErr = ((pid_error - pid_last_error) / timeChange);
  double dInput = ((measuredHeight - pid_lastInput) / timeChange);
  //Serial.print("dInput: ");
  //Serial.println(dInput);

  /*Compute PID Output*/
  //pid_output = (myPID.P * pid_error) + (myPID.I * pid_error_sum) + (myPID.D * dErr);
  //pid_output = (myPID.P * pid_error) + (myPID.I * pid_error_sum) - (myPID.D * dInput);
  //pid_output = (myPID.P * pid_error) + pid_Iterm + (myPID.D * dErr);
  pid_output = (myPID.P * pid_error) + pid_Iterm - (myPID.D * dInput);
  //Serial.print("pid_output: ");
  //Serial.println(pid_output);
  /*if (pid_output > pid_outMax)
    {
    pid_output = pid_outMax;
    }
    else if (pid_output < pid_outMin)
    {
    pid_output = pid_outMin;
    }*/

  //Serial.println(pid_output);

  /*Remember some variables for next time*/
  pid_lastInput = measuredHeight;
  pid_last_error = pid_error;
  lastTime = now;
}

void Apply_PID()
{
  if (PWM + pid_output > 255)
  {
    PWM = 255;
  }
  else if (PWM + pid_output < 0)
  {
    PWM = 0;
  }
  else
  {
    PWM += pid_output;
  }
  //Serial.println(PWM);
  analogWrite(helicopterPin, PWM);
  analogWrite(PWM_Reference_Pin, PWM);
}

void HandleButtons()
{
  B0_select_debounce.update();
  B1_P_debounce.update();
  B2_I_debounce.update();
  B3_D_debounce.update();

  bool B0_select_val = B0_select_debounce.read();
  bool B1_P_val = B1_P_debounce.read();
  bool B2_I_val = B2_I_debounce.read();
  bool B3_D_val = B3_D_debounce.read();

  bool B1_P_state = B1_P_val && !B1_P_lastState;
  bool B2_I_state = B2_I_val && !B2_I_lastState;
  bool B3_D_state = B3_D_val && !B3_D_lastState;

  B1_P_lastState = B1_P_val;
  B2_I_lastState = B2_I_val;
  B3_D_lastState = B3_D_val;

  float used_PID_AdjustStep = B0_select_val ? -PID_AdjustStep : PID_AdjustStep;

  float sampleTimeInSeconds = sampleTime / 1000;

  myPID.P += used_PID_AdjustStep * B1_P_state;
  //myPID.I += (used_PID_AdjustStep * B2_I_state) * sampleTimeInSeconds;
  //myPID.D += (used_PID_AdjustStep * B3_D_state) / sampleTimeInSeconds;
  myPID.I += used_PID_AdjustStep * B2_I_state;
  myPID.D += used_PID_AdjustStep * B3_D_state;
}

void HandlePotentioMeter()
{
  //analogRead(setpointSelectPin);
  setpoint = map(analogRead(setpointSelectPin), 0, 1023, 0, maxDistance);

  lastSetPoint = setpoint;
  setpoint = movingAverage(&setpoint, 20); // Update measuredHeight.

  if ((lastSetPoint + measurementErrorMargin) < setpoint)
  {
    setpoint = lastSetPoint + measurementErrorMargin;
  }
  else if ((lastSetPoint - measurementErrorMargin) > setpoint)
  {
    setpoint = lastSetPoint - measurementErrorMargin;
  }

}



void ShowValues()
{

  if ((millis() - lastLCD_Update) > LCD_Update_Time)
  {
    lcd.clear();

    lcd.setCursor(0, 0);
    lcd.print(myPID.P, 4);

    lcd.setCursor(7, 0);
    lcd.print(myPID.I, 4);

    lcd.setCursor(0, 1);
    lcd.print(myPID.D, 4);

    lcd.setCursor(7, 1);
    lcd.print(setpoint);

    lcd.setCursor(10, 1);
    lcd.print(measuredHeight);

    lcd.setCursor(13, 1);
    lcd.print(PWM);

    lastLCD_Update = millis();
  }
}

