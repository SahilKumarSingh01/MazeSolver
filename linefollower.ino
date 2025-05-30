#define Inverse 0                       //This is for adjusting with course 1 if running on Black Tack 0 if running on White Track
#define Active !Inverse                 //Accoring to Track sense of active stutus also vary means for black track 0 means Active (or track detacted) and for White track 1 means active 
#define FarLeftSensor A0                //Pin for left most Sensor 
#define LeftSensor A1                   //Pin for left sensor
#define FrontSensor A2                  //Pin for front sensor
#define FarFrontSensor A5               //pin no for one sensor which is 3cm away from front sensor   ///leave sensor
#define RightSensor A3                  //pin for right sensor
#define FarRightSensor A4               //pin for right most sensor
#define Speed 30                      // Max Speed of motor in terms of percentage // don't increase it over(50) otherwise change move function else call sahil
#define MaxDutyCycle (Speed*  255)/100    // Corressponding DutyCycle according to Max Speed
#define LeftMotorPin1 11                //PWM pin for Left motor pin
#define LeftMotorPin2 10                //pin second for Left motor
#define RightMotorPin1 6               //PWM pin for Right motor pin
#define RightMotorPin2 9                //second Pin for Rigth motor
#define Threshold 300                   //treshold value for sensor
#define LearningRate 0.01               // It is for backpropogration
#define LedFL  2
#define LedL  3
#define LedF  4
#define LedR  5
#define LedFR 7
#define DelayPerDegree 3.25f            //we need to find how much delay is required to turn 1 degree
char DirectionArray[100];               //an array to store turns for each junction 
int ArraySize=0;                        //It store the number of direction stored
////////////////////////////////////////  /////////////
//variables to store to sensor array input value;
///////////////////////////////////////////////////
int FL;
int L;
int F;
int FF;
int R;
int FR;
//variables for PID algorithm
//it will be modify using backpropagation
float Kp=2;                               //constant for proportional (current error)
float Ki=Kp/60;                              //constant for integral     (Sum of error over time)
float Kd=1.8*Kp;                               //constant for differential (current error -previous error)
float PreviousError=0;                     //a variable to store previous error set to zero
float Integral=0;                          //a variable to store the sum of all error
//float streeing=0;                          //a variable to store streeing value corresponding to error

//////////////////////////////////////////////////////////////
//Checking if we are on function or terminal
//return 1 if so 
//else return 0
////////////////////////////////////////////////////////////
//char LastDirection;
bool IsNode()
{
    int TotalActiveSensor=FL+L+F+R+FR;
    if(TotalActiveSensor>=3 or TotalActiveSensor==0)
      return 1;
    else return 0;
}



////////////////////////////////////////////////////////////
//Taking input from sensor now using Threshold we assign 
//1 if a track is below sensor
//0 if a track is not below sensor
//Active is 0 1 depending on track type as defined in pre processor directive
/////////////////////////////////////////////////////////////
void StoreInput() 
{
  FL=L=F=FF=R=FR=0;
  //taking average over n input
  const int n=50;
  for(int i=0;i<n;i++)
  {
      FL  +=(analogRead(FarLeftSensor)>Threshold)?Active:!Active;
      L   +=(analogRead(LeftSensor)>Threshold)?Active:!Active;
      F   +=(analogRead(FrontSensor)>Threshold)?Active:!Active;
      FF  +=(analogRead(FarFrontSensor)>Threshold)?Active:!Active;
      R   +=(analogRead(RightSensor)>Threshold)?Active:!Active;
      FR  +=(analogRead(FarRightSensor)>Threshold)?Active:!Active;
      delay(1);//it is required
  }
  //converting it to 0 to 1
  if(FL>n/2)FL=1;
  else FL=0;
  if(L>n/2)L=1;
  else L=0;
  if(F>n/2)F=1;
  else F=0;
  if(R>n/2)R=1;
  else R=0;
  if(FR>n/2)FR=1;
  else FR=0;
}
void UpdateLed()
{
  digitalWrite(LedFL,FL);
  digitalWrite(LedL,L);
  digitalWrite(LedF,F);
  digitalWrite(LedR,R);
  digitalWrite(LedFR,FR);
}

float CalculateError()
{
  if(FL+L+F+R+FR)
      return (float)(6.f*FL+3.f*L+0.f*F-3.f*R-6.f*FR);
  else return PreviousError;
}



/////////////////////////////////////////////////////////////////////////////
//Calculating Streeing value using Pid
////////////////////////////////////////////////////////////////////////////
float CalculateStreeing()
{
  float error=CalculateError();
  Integral+=error;
  float streeing=-(Kp*error+Ki*Integral+Kd*(error-PreviousError));
  PreviousError=error;
  //make sure streeing is in range -100 to 100
  if(streeing<-100)streeing=-100;
  else if(streeing>100)streeing=100;
  return streeing;
}
//////////////////////////////////////////////////////////////////
//Moving Bot according to Streeing value(-100,100)
//////////////////////////////////////////////////////////////////
void Move()
{
  float streeing=CalculateStreeing();
  //PMW is kind of maximum of Duty cycle ;
  int DutyCycle=(streeing*MaxDutyCycle)/100;
  if(DutyCycle>0)
  {
    analogWrite(RightMotorPin1,MaxDutyCycle-DutyCycle);             //slowing down right motor to turn right
    analogWrite(LeftMotorPin1,MaxDutyCycle+DutyCycle);                        //reseting left motor to max speed
  }
  else
  {
    //Duty Cycle is negative so we are subtracting
    analogWrite (LeftMotorPin1,MaxDutyCycle+DutyCycle);             //slowing down left motor to turn left
    analogWrite(RightMotorPin1,MaxDutyCycle-DutyCycle);                       //reseting right motor to max speed
  }
  //by default it is zero in case any other function change its value so to ensure that it remain zero
  analogWrite(RightMotorPin2,0);                                  //defining ground pin for right motor 
  analogWrite(LeftMotorPin2,0);                                   //defining ground pin for left motor
  UpdateLed();
}

void DisplayConstants()
{
//  Serial.print(FL);
//  Serial.print(',');
//  Serial.print(L);
//  Serial.print(',');
//  Serial.print(F);
//  Serial.print(',');
//  Serial.print(R);
//  Serial.print(',');
//  Serial.print(FR);
//  Serial.print('\n');
  Serial.print(analogRead(FarLeftSensor));
  Serial.print(',');
  Serial.print(analogRead(LeftSensor));
  Serial.print(',');
  Serial.print(analogRead(FrontSensor));
  Serial.print(',');
  Serial.print(analogRead(RightSensor));
  Serial.print(',');
  Serial.print(analogRead(FarRightSensor));
  Serial.print('\n');
}
void MoveTillJunctionEnd()
{
    while(!(FR==0 && FL==0))
    {
      StoreInput();
      Move();
    }
}

int DryRun=1;

#define TurningTime 800
void JunctionDealerInDryRun  ()
{
    analogWrite(RightMotorPin1,0);                                  //defining ground pin1 for right motor 
    analogWrite(LeftMotorPin1,0);                                   //defining ground pin1 for left motor
    analogWrite(RightMotorPin2,0);                                  //defining ground pin2 for right motor 
    analogWrite(LeftMotorPin2,0); 
//    delay(5000);
    float turningSpeed    =20;
    float TurningDutyCycle=turningSpeed*255/100;
    float reducingFactor  =23;
    if(FL==1)
    {
        MoveTillJunctionEnd();
        analogWrite (LeftMotorPin2,TurningDutyCycle);                   //reversing sense of rotation in left motor to turn left
        analogWrite (RightMotorPin1,TurningDutyCycle);    
        delay(TurningTime);
        float reduct=0;
        do
        {   
            StoreInput(); 
            analogWrite(LeftMotorPin2,TurningDutyCycle-reduct);                                   //defining ground pin1 for left motor
            analogWrite(RightMotorPin1,TurningDutyCycle-reduct);
            reduct+=TurningDutyCycle/reducingFactor;
        }while(!(F));
        DirectionArray[ArraySize++]='L';
    }
    else if(F==1) 
    {
       MoveTillJunctionEnd();
       DirectionArray[ArraySize++]='F';
      
    }
    else if(FR==1) 
    {
        MoveTillJunctionEnd();
        analogWrite (LeftMotorPin1,TurningDutyCycle);                   //reversing sense of rotation in left motor to turn left
        analogWrite (RightMotorPin2,TurningDutyCycle);    
        delay(TurningTime);
        float reduct=0;
        do
        {   
            StoreInput(); 
            analogWrite(LeftMotorPin1,TurningDutyCycle-reduct);                                   //defining ground pin1 for left motor
            analogWrite(RightMotorPin2,TurningDutyCycle-reduct);
            reduct+=TurningDutyCycle/reducingFactor;
        }while(!(F));
  
       DirectionArray[ArraySize++]='R';
    }
    else if(!(FL+L+F+R+FR))
    {   
//        turningSpeed =0.75;
        reducingFactor=30;
        if(PreviousError<=0)
        {//defining ground pin1 for right motor 
            analogWrite (LeftMotorPin1,TurningDutyCycle);                   //reversing sense of rotation in left motor to turn left
            analogWrite (RightMotorPin2,TurningDutyCycle);   
            float reduct=0;
            do
            {   
                StoreInput(); 
                analogWrite(LeftMotorPin1,TurningDutyCycle-reduct);                                   //defining ground pin1 for left motor
                analogWrite(RightMotorPin2,TurningDutyCycle-reduct);
                reduct+=TurningDutyCycle/reducingFactor;
            }while(!(F));
        }
        else 
        {
            analogWrite (LeftMotorPin2,TurningDutyCycle);                   //reversing sense of rotation in left motor to turn left
            analogWrite (RightMotorPin1,TurningDutyCycle);   
            float reduct=0;
            do
            {   
                StoreInput(); 
                analogWrite(LeftMotorPin2,TurningDutyCycle-reduct);                                   //defining ground pin1 for left motor
                analogWrite(RightMotorPin1,TurningDutyCycle-reduct);
                reduct+=TurningDutyCycle/reducingFactor;
            }while(!(F));
        }
  
    }
    analogWrite(RightMotorPin1,0);                                  //defining ground pin1 for right motor 
    analogWrite(LeftMotorPin1,0);                                   //defining ground pin1 for left motor
    analogWrite(RightMotorPin2,0);                                  //defining ground pin2 for right motor 
    analogWrite(LeftMotorPin2,0); 
    UpdateLed();
//    delay( 5000);
}


int JunctionCrossed=0;//Number of junction crossed in active run or in other word store the pos of array in which we have to move in next junction        
void JunctionDealerInActiveRun ()
{
   char ch=DirectionArray[JunctionCrossed++];
   if(ch=='L')
   {
     for(int i=0;i<TurningTime;i++)
     {
//          StoreInput();
          //considering only left part of junction ,for that turning off sensor value of FR and FF 
          FF=FR=R=0;
          L=FL=1;
          CalculateStreeing();
          Move();
     }
   }
   if(ch=='R')
   {
      for(int i=0;i<TurningTime;i++)
     {
          StoreInput();
          //considering only right part of junction ,for that turning off sensor value of FL and FF
          FF=FL=L=0;                          
          CalculateStreeing();
          Move();
     }
   }
   if(ch=='F')
   {
      for(int i=0;i<TurningTime;i++)
     {
          StoreInput();
          //considering only Foeward part of junction ,for that turning off sensor value of FL and FR
          FL=FR=0;
          CalculateStreeing();
          Move();
     }
   }
}
void RemoveUTurns()
{
  String TmpStr=DirectionArray;
  //til any 'U' is left in string
  while(TmpStr.indexOf('U')!=-1)
  {
    TmpStr.replace("LULULUL","U");
    TmpStr.replace("FULUL","U");
    TmpStr.replace("LUFUL","U");
    TmpStr.replace("LUFUF","U");
    TmpStr.replace("LUR","U");
    TmpStr.replace("RUL","U");
    TmpStr.replace("LUL","F");
    TmpStr.replace("LULUL","R");
    TmpStr.replace("FUL","R");
    TmpStr.replace("LUF","R");
  }
  ArraySize=TmpStr.length();
  //storing the data from temp string to DirecctionArray;
  for(int i=0;i<ArraySize;i++)
  {
    DirectionArray[i]=TmpStr[i];
  }
}
void setup() {
    Serial.begin(9600);
    pinMode(FarLeftSensor,INPUT);
    pinMode(LeftSensor,INPUT);
    pinMode(FrontSensor,INPUT);
    pinMode(FarFrontSensor,INPUT);
    pinMode(RightSensor,INPUT);
    pinMode(FarRightSensor,INPUT);
    pinMode(LeftMotorPin1,OUTPUT);
    pinMode(LeftMotorPin2,OUTPUT);
    pinMode(RightMotorPin1,OUTPUT);
    pinMode(RightMotorPin2,OUTPUT);
    pinMode(LedFL,OUTPUT);
    pinMode(LedL,OUTPUT);
    pinMode(LedF,OUTPUT);
    pinMode(LedR,OUTPUT);
    pinMode(LedFR,OUTPUT);

}
void loop() {
  StoreInput();
  UpdateLed();
  if(!IsNode())
  {
    Move();
  }
  else
  {
    if(DryRun)JunctionDealerInDryRun();
    else JunctionDealerInActiveRun();
    DisplayConstants();
  }
//    DisplayConstants();
}
