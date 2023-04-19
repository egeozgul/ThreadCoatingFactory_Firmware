#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
 
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

#define thermistorPin A6
#define fanPin 6
#define conductivitySensorPin A2 
#define tensionSensorPin A3
#define heaterPin 9

float Speed = 10000.0f/(float)300;





float convertToTemp(float Vo)
{
  int ThermistorPin = 0;
  float R1 = 10000;
  float logR2, R2, T;
  float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  T = T - 273.15;
 // T = (T * 9.0)/ 5.0 + 32.0; 
  return T;
}


float tempBuffer[30];
float movingAverage(float *tempBuffer)
{
  float sum = 0;
  for (int i = 0;i<30;i++)
    sum += tempBuffer[i];
  return sum/30.0f;
}
int index = 0;
void insertData(float newData)
{
  tempBuffer[index] = newData;
  index = (index+1)%30;
}





int heaterValue=0;

#define dirPin 2
#define  stepPin 3

#define  dirPinB 7
#define  stepPinB 8

#define  pumpDirPin 10
#define  pumpStepPin 11

#define stepsPerRevolution 200


int stepperA_state = LOW;
int stepperB_state = LOW;
int stepperC_state = LOW;

void stepperStepA()
{
    if(stepperA_state == LOW) stepperA_state = HIGH;
    else if(stepperA_state == HIGH) stepperA_state = LOW;
    
    digitalWrite(stepPin , stepperA_state);
}
void stepperStepB()
{    
    if(stepperB_state == LOW) stepperB_state = HIGH;
    else if(stepperB_state == HIGH) stepperB_state = LOW;
    
    digitalWrite(stepPinB , stepperB_state);
}
void stepperStepC()
{    
    if(stepperC_state == LOW) stepperC_state = HIGH;
    else if(stepperC_state == HIGH) stepperC_state = LOW;
    
    digitalWrite(pumpStepPin, stepperC_state);
}
bool LED_STATE = true;

void interruptSetup()
{
  cli();                      //stop interrupts for till we make the settings
  /*1. First we reset the control register to amke sure we start with everything disabled.*/
  TCCR1A = 0;                 // Reset entire TCCR1A to 0 
  TCCR1B = 0;                 // Reset entire TCCR1B to 0
 
  /*2. We set the prescalar to the desired value by changing the CS10 CS12 and CS12 bits. */  
  TCCR1B |= B00000100;        //Set CS12 to 1 so we get prescalar 256  
  /*3. We enable compare match mode on register A*/
  TIMSK1 |= B00000010;        //Set OCIE1A to 1 so we enable compare match A 
  /*4. Set the value of register A to 31250*/
  OCR1A = 5;             //Finally we set compare register A to this value  ---  80 microseconds 
  sei();                     //Enable back the interrupts
}
long int timerPeriod = 80;

/*
16000000/256/5 = 12500 times per second
12500
(16*200)

(16*200)/12500/30 rev per second

round/second
*/

//With the settings above, this IRS will trigger each 500ms === 31250 2500/31250 
int counterA = 0;
int counterB = 0;
int counterC = 0;


long int posA = 0;
long int posB = 0;
long int posC = 0;
int motorDirB = 1;

int enableMotorA = 0;
int enableMotorB = 0;
int enableMotorC = 0;
volatile int counterALimit = 30;
volatile int counterBLimit = 30;
volatile int counterCLimit = 2;

ISR(TIMER1_COMPA_vect)
{
  TCNT1  = 0;                  //First, set the timer back to 0 so it resets for next interrupt

  counterA++;
  counterB++;
  counterC++;

  if(counterA >= counterALimit and enableMotorA==1)
  {
    counterA =0;
    stepperStepA();  
    posA++;
  }
  if(counterB >= counterBLimit and enableMotorB==1)
  {
    counterB =0;
    stepperStepB();
    

    if(motorDirB == 1)
      digitalWrite(dirPinB, LOW);
    else if(motorDirB == -1)
      digitalWrite(dirPinB, HIGH);

    posB+=motorDirB;
    
  }
  if(counterC >= counterCLimit and enableMotorC==1)
  {
    counterC =0;
    stepperStepC();
    posC++;
  }

      
  static int cc = 0;
  int dutyCycle = heaterValue;
  if(cc==0)
    digitalWrite(heaterPin,HIGH);
  if(cc==dutyCycle)
    digitalWrite(heaterPin,LOW); 

  cc ++;
  if(cc >= 10)
      cc=0;
}


void setup() 
{
  Serial.begin(115200);
  
  // Declare pins as output:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(stepPinB, OUTPUT);
  pinMode(dirPinB, OUTPUT);
  pinMode(pumpStepPin, OUTPUT);
  pinMode(pumpDirPin, OUTPUT);
  
  digitalWrite(dirPinB, LOW);
  digitalWrite(dirPin, HIGH);
  digitalWrite(pumpDirPin, LOW);
  interruptSetup();

  lcd.init();
  // Print a message to the LCD.
  lcd.backlight();
  
  pinMode(13,OUTPUT);
  pinMode(fanPin,OUTPUT);
  pinMode(heaterPin,OUTPUT);
  analogWrite(fanPin,255);
  //Serial.begin(115200);
  pinMode(1,INPUT_PULLUP);
  pinMode(A0,INPUT_PULLUP);
   pinMode(0,INPUT_PULLUP);
  pinMode(A1,INPUT_PULLUP);
  
  counterBLimit = 10000.0f/(float)Speed;

  int stepperEnablePin = 4;
  pinMode(stepperEnablePin, OUTPUT);
  digitalWrite(stepperEnablePin,HIGH);
  delay(1000);
  digitalWrite(stepperEnablePin,LOW);

}

  static float P = 0;
  static float I = 0;
  static float D = 0;
  static float kP = 150.0;
  static float kI = 0.0001f;
  static float kD = 0;
  static float totalError = 0;

int ProcessStarted = false;
//int ThreadTestStarted = false;
int ThreadTestStarted = true;


float inkFeedrate = 10;
float currentTemp = 0;
int desiredTendionRate = 700;
float desiredTemp  = 40;
static int selection = 0;
void buttonRight()
{
  if(selection == 0)
    desiredTemp++;

  if(selection == 2)
    desiredTendionRate+=5;

  if(selection == 1)
    {
      Speed+=10;
      counterBLimit = 10000.0f/(float)Speed;
    }
    
  if(selection == 4)
    {
      inkFeedrate+=10;
      counterCLimit = 1000000.0f/(float)inkFeedrate;
    }

    if(selection == 5)
    {
      ProcessStarted = not ProcessStarted;
      if(ThreadTestStarted == true)
        ThreadTestStarted = false;
    }
    
    if(selection == 6)
    {
      ThreadTestStarted = not ThreadTestStarted;
      if(ProcessStarted == true)
        ProcessStarted = false;
    } 
}

int nSelections = 7;
void buttonLeft()
{
    if(selection == 0)
    desiredTemp--;
      
    if(selection == 2)
    desiredTendionRate-=5;

  if(selection == 1)
  {
    Speed-=10;
    if(Speed <5)Speed = 10;
  }
  counterBLimit = 10000.0f/(float)Speed;

  if(selection == 4)
    {
      inkFeedrate-=10;
      counterCLimit = 1000000.0f/(float)inkFeedrate;
    }

    //if(selection == 5)
      //ProcessStarted = not ProcessStarted;

      //  if(selection == 6)
      //ThreadTestStarted = not ThreadTestStarted;

 }

void buttonDown()
{
   selection--;
       if(selection<0)
        selection=nSelections-1;
}

void buttonUp()
{
  selection=(selection+1)%nSelections;
}

float conductivity;
float tensionRate;

#define SC() break; case (__LINE__ - startLineAddress ):
//#define SC() 
int lineIndex = 0;

void updateLCD()
{   
    lineIndex = (lineIndex+1)%26;
    int caseCoutner = 0;
    switch(lineIndex)
    {
      static const int startLineAddress = __LINE__;
      SC() lcd.setCursor(0,0);
      lcd.print("tmp:");
      lcd.print((int)currentTemp);
      lcd.print("/");
      lcd.print((int)desiredTemp);
      
      SC() lcd.setCursor(10,0);
      lcd.print("Ink:");
      lcd.print(inkFeedrate);
  
      SC() lcd.setCursor(0,1);
      lcd.print("Con:");
      lcd.print(conductivity);

  
      SC() lcd.setCursor(0,2);
      lcd.print("Tension:");
      lcd.print((int)tensionRate);
      lcd.print("/");
      lcd.print((int)desiredTendionRate);
    
      lcd.setCursor(10,1);
      lcd.print("Spd:");
      lcd.print(Speed);


      lcd.setCursor(8,1);
      lcd.print("  ");
      
      
      lcd.setCursor(0,3);
      if(selection == 0)
        lcd.print("Adjust temperature");  
      else if(selection == 1)
        lcd.print("Adjust spool speed");
      else if(selection == 2)
        lcd.print("Adjust tension    ");
      else if(selection == 3)
        lcd.print("Manual pump contrl");
      else if(selection == 4)
        lcd.print("Adjust pump rate  ");
      else if(selection == 5 and ProcessStarted == true)
        lcd.print("STOP COATING      ");
      else if(selection == 5 and ProcessStarted == false)
        lcd.print("START COATING     ");
      else if(selection == 6 and ThreadTestStarted == true)
        lcd.print("STOP THREAD TEST  ");
      else if(selection == 6 and ThreadTestStarted == false)
        lcd.print("START THREAD TEST ");
    }
}

//int lineIndex = 0;
void updateLCD2()
{   
      lcd.setCursor(0,0);
      lcd.print("t:");
      lcd.print(currentTemp);
      lcd.print("/");
      lcd.print(desiredTemp);
      lcd.setCursor(0,1);
      lcd.print("C:");
      lcd.print(conductivity);
  
      lcd.setCursor(0,2);
      lcd.print("T:");
      lcd.print(tensionRate);
      lcd.print("/");
      lcd.print(desiredTendionRate);
    
      lcd.setCursor(0,3);
      lcd.print("S:");
      lcd.print(Speed);


      lcd.setCursor(19,0);
      lcd.print(" ");
      lcd.setCursor(19,1);
      lcd.print(" ");
      lcd.setCursor(19,2);
      lcd.print(" ");
      lcd.setCursor(19,3);
      lcd.print(" ");
      lcd.setCursor(19,selection);
      lcd.print("<");
}

void pid(int tensionRate)
{
  //PID
  float error = tensionRate - desiredTendionRate;
  //totalError += error;

  
  float speedB = Speed;//10000.0f/(float)counterBLimit;
  float speedA = speedB - kP*error;



  float result = 10000.0f/speedA;
  
  if(speedA >= 0)
    digitalWrite(dirPin, HIGH);
  else if(speedA < 0)
    digitalWrite(dirPin, LOW);
  
  counterALimit = abs(result);

  if(totalError > 0.30)
  totalError  = 0.3;
  if(totalError < -0.30)
  totalError  = -0.3;
}

void (*buttonActions[4])();

void loop() 
{
    if(ProcessStarted  == true)
    {
      enableMotorA = 1;
      enableMotorB = 1;
      digitalWrite(pumpDirPin,HIGH);
      
      if(inkFeedrate<=5)
        {
          inkFeedrate=5;
          counterCLimit = 1000000.0f/(float)inkFeedrate;
          enableMotorC = 0;
        }
        else
        {
          counterCLimit = 1000000.0f/(float)inkFeedrate;
          enableMotorC = 1;
        }
    }
    else if(ThreadTestStarted == true)
    {
      enableMotorA = 1;
      enableMotorB = 1;
      Speed=5000;
      counterBLimit = 10000.0f/(float)Speed; 
    }
    else if(ProcessStarted  == false)
    {
    enableMotorA = 0;
    enableMotorB = 0;
    }

    if(digitalRead(1) ==false and selection==3)
    {
      digitalWrite(pumpDirPin,LOW );
      counterCLimit = 5;
      enableMotorC = 1;
    }
    else if(digitalRead(A0) == false and selection==3)
    {
      digitalWrite(pumpDirPin,HIGH);
      counterCLimit = 5;
      enableMotorC = 1;
    }
    else if(ProcessStarted == true)
    {
      enableMotorC = 1;
    }
    else if(ProcessStarted == false)
    {
      enableMotorC = 0;
    }

 
    //button state detection
    buttonActions[0] = buttonUp;
    buttonActions[1] = buttonDown;
    buttonActions[2] = buttonRight;
    buttonActions[3] = buttonLeft;
    
    int debouncingDuration = 250;
    int pins[4] = {0,A1,A0,1};
    static long debouncingTimer[4] = {0,0,0,0};
    static bool enable[4] = {1,1,1,1};
    for(int i =0;i<4;i++)
    {
      if(digitalRead(pins[i]) == false and enable[i]==1)
      {
         debouncingTimer[i] = millis();
         enable[i]=0;
         buttonActions[i]();
      }
      if(enable[i]==0 and millis() > debouncingTimer[i] + debouncingDuration)
        enable[i] = 1;
    }


    //tension sensor
    tensionRate = analogRead(tensionSensorPin);
  
    //conductivity sensor
    conductivity = analogRead(conductivitySensorPin);

    //pid
    pid(tensionRate);

    //heater code
    int rawTemp = analogRead(thermistorPin);
    insertData(rawTemp);
    rawTemp = movingAverage(tempBuffer);
    currentTemp = convertToTemp(rawTemp);

    if(currentTemp < desiredTemp)
      heaterValue = 10; //9 out of 10 so 90 percent software pwm
    if(currentTemp > desiredTemp)
      heaterValue = 0;  
    
  //LCD show code
  int LCDupdate_period_millis = 1000;
  static unsigned long int LCD_update_counterStartTime = millis();
  if(millis() - LCD_update_counterStartTime  > LCDupdate_period_millis)
  {
    //updateLCD();
    //LCD_update_counterStartTime = millis();
  }

//USB serial log code
  int Log_period_millis = 200;
  static unsigned long int Log_counterStartTime = millis();
  if(millis() - Log_counterStartTime  > Log_period_millis and  ThreadTestStarted == true)
  {
    Serial.print((int)tensionRate);
    Serial.print(" ");
    //Serial.print((long int)posB);
    //Serial.print(" ");
    Serial.println((int)conductivity);
    Log_counterStartTime = millis();
  }

static int counter_ = 0;
enableMotorA = 0;
static bool countingUp = true;
static int counter_period_millis = 200;
static long int counter_counterStartTime = millis();
if(millis() - counter_counterStartTime  > counter_period_millis and ThreadTestStarted)
{
  if(countingUp)
  {
    if(tensionRate < 800)
      desiredTendionRate++;
    if(tensionRate > 800)
      countingUp = false;
  }
  else
  {
    if(tensionRate > 450)
      desiredTendionRate--;
    if(tensionRate < 450)
      countingUp = true;
  }
  counter_counterStartTime = millis();
}

if(counter_ >= 100)
  ThreadTestStarted = false; 
}
