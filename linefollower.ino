#define Inverse 1                       //This is for adjusting with course 1 if running on Black Tack 0 if running on White Track
#define Active !Inverse                 //Accoring to Track sense of active stutus also vary means for black track 0 means Active (or track detacted) and for White track 1 means active 
#define FarLeftSensor A0                //Pin for left most Sensor 
#define LeftSensor A1                   //Pin for left sensor
#define FrontSensor A2                  //Pin for front sensor
#define FarFrontSensor A6               //pin no for one sensor which is 3cm away from front sensor   ///leave sensor
#define RightSensor A3                  //pin for right sensor
#define FarRightSensor A4               //pin for right most sensor
#define Speed 100                       // Max Speed of motor in terms of percentage
#define MaxDutyCycle (Speed*255)/100    // Corressponding DutyCycle according to Max Speed
#define LeftMotorPin1 11                //PWM pin for Left motor pin
#define LeftMotorPin2 12                //pin second for Left motor
#define RightMotorPin1 10               //PWM pin for Right motor pin
#define RightMotorPin2 8                //second Pin for Rigth motor
#define Threshold 300                   //treshold value for sensor
#define LearningRate 0.01               // It is for backpropogration 
#define DelayPerDegree 11.f             //we need to find how much delay is required to turn 1 degree
char DirectionArray[100];               //an array to store turns for each junction 
int ArraySize=0;                        //It store the number of direction stored
/////////////////////////////////////////////////////
//variables to store to sensor array input value;
///////////////////////////////////////////////////
bool FL;
bool L;
bool F;
bool FF;
bool R;
bool FR;
//variables for PID algorithm
//it will be modify using backpropagation
float Kp=40;                               //constant for proportional (current error)
float Ki=Kp/4;                             //constant for integral     (Sum of error over time)
float Kd=Kp;                               //constant for differential (current error -previous error)
float PreviousError=0;                     //a variable to store previous error set to zero
float Integral=0;                          //a variable to store the sum of all error
float streeing=0;                          //a variable to store streeing value corresponding to error
////////////////////////////////////////////////////////////
//Taking input from sensor now using Threshold we assign 
//1 if a track is below sensor
//0 if a track is not below sensor
//Active is 0 1 depending on track type as defined in pre processor directive
/////////////////////////////////////////////////////////////
void StoreInput() 
{
  FL=(analogRead(FarLeftSensor)>Threshold)?Active:!Active;
  L=(analogRead(LeftSensor)>Threshold)?Active:!Active;
  F=(analogRead(FrontSensor)>Threshold)?Active:!Active;
  FF=(analogRead(FarFrontSensor)>Threshold)?Active:!Active;
  R=(analogRead(RightSensor)>Threshold)?Active:!Active;
  FR=(analogRead(FarRightSensor)>Threshold)?Active:!Active;
}
//////////////////////////////////////////////////////////////
//Checking if we are on function or terminal
//return 1 if so 
//else return 0
////////////////////////////////////////////////////////////
bool IsNode()
{
  if((FL==1 and L==1 and F==1) or (R==1 and FR==1 and F==1) )//or !(FL+L+FF+R+FR))         //remove comment if final value it is required for u turn
    return 1;
  else return 0;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//It generate error value depending on different input sensor 
//like for 0,0,0,0,1 will return -6 as we are 6 cm away from track in left side
//like for 0,0,0,1,1 will return -4.5 as we are somewhere in between -3 to -6  cm away from track in left side
//like for 0,0,0,1,0 will return -3 as we are 3 cm away from track in left side
//like for 0,0,1,1,0 will return -1.5 as we are somewhere in between 0 to -3  cm away from track in left side
//like for 0,0,1,0,0 will return 0 as we are on track
//similarly for right side just we positive values
///////////////////////////////////////////////////////////////////////////////////////////////////////////////


float CalculateError()
{
  if(FL+L+F+R+FR)
    return (float)(6.f*FL+3.f*L+0.f*F-3.f*R-6.f*FR)/(float)(FL+L+F+R+FR);
  else return 0;
}
/////////////////////////////////////////////////////////////////////////////
//Backpropogation Algorithm to modify Kp,Kd,Ki to minimize error
////////////////////////////////////////////////////////////////////////////
void backpropogate()
{
  
  float error=CalculateError();
  Kp+=error*LearningRate;
  Ki+=Integral*LearningRate;
  Kd+=(error-PreviousError)*LearningRate;
}
/////////////////////////////////////////////////////////////////////////////
//Calculating Streeing value using Pid
////////////////////////////////////////////////////////////////////////////
void CalculateStreeing()
{
  float error=CalculateError();
  Integral+=error;
  streeing=-(Kp*error+Ki*Integral+Kd*(error-PreviousError));
  backpropogate();
  PreviousError=error;
  //make sure streeing is in range -100 to 100
  if(streeing<-100)streeing=-100;
  else if(streeing>100)streeing=100;
}
//////////////////////////////////////////////////////////////////
//Moving Bot according to Streeing value(-100,100)
//////////////////////////////////////////////////////////////////
void Move()
{
  //PMW is kind of maximum of Duty cycle ;
  int DutyCycle=(streeing*MaxDutyCycle)/100;
  if(DutyCycle>0)
  {
    analogWrite(RightMotorPin1,MaxDutyCycle-DutyCycle);             //slowing down right motor to turn right
    analogWrite(LeftMotorPin1,MaxDutyCycle);                        //reseting left motor to max speed
  }
  else
  {
    //Duty Cycle is negative so we are subtracting
    analogWrite (LeftMotorPin1,MaxDutyCycle+DutyCycle);             //slowing down left motor to turn left
    analogWrite(RightMotorPin1,MaxDutyCycle);                       //reseting right motor to max speed
  }
  //by default it is zero in case any other function change its value so to ensure that it remain zero
  analogWrite(RightMotorPin2,0);                                  //defining ground pin for right motor 
  analogWrite(LeftMotorPin2,0);                                   //defining ground pin for left motor
}
void DisplayConstants()
{
  Serial.print(Kp);
  Serial.print(",");
  Serial.print(Ki);
  Serial.print(",");
  Serial.print(Kd);
  Serial.print(",");
  Serial.print("\n");
}
////////////////////////////////////////////////////////////////
//This function help to turn the bot without any lateral motion
//about the center of axis of wheels
///////////////////////////////////////////////////////////////
void Turn(int degree)
{
  //Resetting all pin to zero it try to stop the bot for a moment
  analogWrite(RightMotorPin1,0);                                  //defining ground pin1 for right motor 
  analogWrite(LeftMotorPin1,0);                                   //defining ground pin1 for left motor
  analogWrite(RightMotorPin2,0);                                  //defining ground pin2 for right motor 
  analogWrite(LeftMotorPin2,0);                                   //defining ground pin2 for left motor
  if(degree<0)
  {
       //////////////////////////////////////////////////////////////////////
       //considering pin1 as negative for foward moment in both the motors///
       //////////////////////////////////////////////////////////////////////
      analogWrite (LeftMotorPin2,MaxDutyCycle);                   //reversing sense of rotation in left motor to turn left
      analogWrite (RightMotorPin1,MaxDutyCycle);                  //keeping sense of rotation in right motor same
  }
  else 
  {
     /////////////////////////////////////////////////////////////////////
     //considering pin1 as negative for foward moment in both the motors//
     /////////////////////////////////////////////////////////////////////
      analogWrite (LeftMotorPin1,MaxDutyCycle);                  //keeping sense of rotation in left motor same
      analogWrite (RightMotorPin2,MaxDutyCycle);                 //reversing sense of rotation in right motor to turn right
       
  }
  delay((float)abs(degree)*DelayPerDegree);
}

int DryRun=1;
#define TurningTime 200
void JunctionDealerInDryRun  ()
{
  //for left turn 
  if(FL==1 and L==1 and F==1) 
  {       
     for(int i=0;i<TurningTime;i++)
     {
          StoreInput();
          //considering only left part of junction ,for that turning off sensor value of FR and FF 
          FF=FR=0;
          CalulateSteering();
          Move();
     }
     DirectionArray[ArraySize++]='L';
  }
  else if(FF==1 and F==1) 
  {
     for(int i=0;i<TurningTime;i++)
     {
          StoreInput();
          //considering only Foeward part of junction ,for that turning off sensor value of FL and FR
          FL=FR=0;
          CalulateSteering();
          Move();
     }
     DirectionArray[ArraySize++]='F';
    
  }
  else if(FR==1 and R==1 and F==1) 
  {
     for(int i=0;i<TurningTime;i++)
     {
          StoreInput();
          //considering only right part of junction ,for that turning off sensor value of FL and FF
          FF=FL=0;                          
          CalulateSteering();
          Move();
     }
     DirectionArray[ArraySize++]='R';
  }
  else if(!(FL+L+FF+R+FR))
  {
    Turn(180);
    DirectionArray[ArraySize++]='U';
  }
}
int JunctionCrossed=0;//Number of junction crossed in active run or in other word store the pos of array in which we have to move in next junction        
void JunctionDealerInActiveRun ()
{
   char ch=DirectionArray[JunctionCrossed++];
   if(ch=='L')
   {
     for(int i=0;i<TurningTime;i++)
     {
          StoreInput();
          //considering only left part of junction ,for that turning off sensor value of FR and FF 
          FF=FR=0;
          CalulateSteering();
          Move();
     }
   }
   if(ch=='R')
   {
      for(int i=0;i<TurningTime;i++)
     {
          StoreInput();
          //considering only right part of junction ,for that turning off sensor value of FL and FF
          FF=FL=0;                          
          CalulateSteering();
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
          CalulateSteering();
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
}
void loop() {
  StoreInput();
  if(!IsNode())
  {
    CalculateError();
    CalculateStreeing();
    Move();
    DisplayConstants();
  }
  else
  {
    if(DryRun)JunctionDealerInDryRun();
    else JunctionDealerInActiveRun();
  }

}
