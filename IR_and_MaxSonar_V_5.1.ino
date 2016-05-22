//#include <mySoftwareSerial.h>

/*
 * v 5.1
 * 기체 내부 확인완료
 * 블루투스 확인완료
 * 따라서 남은건 소스를 합치는 것 뿐!
 * 하 SoftwareSerial 과 PinChangeInt 라이브러리는 함께 사용하지 못한다 -> https://code.google.com/p/arduino-pinchangeint/issues/detail?id=7https://code.google.com/p/arduino-pinchangeint/issues/detail?id=7
 * 
 * 
 */

#include <PinChangeInt.h>
#include <PID_v1.h>
#include <SharpIR.h>
#include <NewPing.h>
#include <Servo.h>
#include <Wire.h>


//Arduino Uno or Pro mini, MaxSonar, IR, Bluetoothe connection diagram
#define aux3Out 4
#define aOut 5 //TX to flight controller ch1:aileron
#define eOut 6 //TX to flight controller ch2:elevator
#define tOut 7 //TX to flight controller ch3:throttle 
#define aIn 8 //RX from receiver ch1:aileron
#define eIn 9 //RX from receiver ch2:elevator
#define tIn 10 //RX from receiver ch3:throttle
#define aux1 11 //RX from receiver ch5:Obstacle Avoidance Mode
#define aux2 12 //RX from receiver ch6:Althold Mode
#define aux3 13
#define sonarIn 14
#define yOut 15
#define yIn 16

unsigned long aux1_read;
unsigned long aux2_read;
unsigned long aux3_read;
unsigned long escape_auto_takeoff;
unsigned long previousMillis = 0;

const long interval = 1000;

//IR sesonr connection diagram
#define frontIr A4 //RX from front IR seonsor 
#define rightIr A5 //RX from right IR sensor
#define backIr A6 //RX from back IR sensor
#define leftIr A7 //RX from left IR sensor
#define model 20150 //IR sensor model number

//IR sensor and Maxsonar sensor variable declaration
double Setpoint, altSetpoint;
double frontIrInput, frontIrOutput;
double rightIrInput, rightIrOutput;
double backIrInput, backIrOutput;
double leftIrInput, leftIrOutput;
double altSonarInput, altSonarOutput,lastSonarInput;
double Kp = 2.9; //Kp variable must declare before IR sesor object declaration
double Ki = 0; //Ki variable must declare before IR sesor object declaration
double Kd = 0; //Kd variable must declare before IR sesor object declaration

//aux1 전용 pid
double Kp2 = 1; //Kp variable must declare before IR sesor object declaration
double Ki2 = 0; //Ki variable must declare before IR sesor object declaration
double Kd2 = 0; //Kd variable must declare before IR sesor object declaration

long pulse;
long inches;

//IR sensor and MaxSonar object declaration
SharpIR fIr(frontIr, 25, 70, model); //acquire 25 samples only 70% corretiveness
SharpIR rIr(rightIr, 25, 70, model); //acquire 25 samples only 70% corretiveness
SharpIR bIr(backIr, 25, 70, model); //acquire 25 samples only 70% corretiveness
SharpIR lIr(leftIr, 25, 70, model); //acquire 25 samples only 70% corretiveness

//IR sensor and MaxSonar PID object declaration
PID frontPID(&frontIrInput, &frontIrOutput, &Setpoint, Kp, Ki, Kd, DIRECT);
PID rightPID(&rightIrInput, &rightIrOutput, &Setpoint, Kp, Ki, Kd, DIRECT);
PID leftPID(&leftIrInput, &leftIrOutput, &Setpoint, Kp, Ki, Kd, DIRECT);
PID backPID(&backIrInput, &backIrOutput, &Setpoint, Kp, Ki, Kd, DIRECT);

//aux1이 1700이상일때 pid를 바꿔주기위해
PID frontPID2(&frontIrInput, &frontIrOutput, &Setpoint, Kp2, Ki2, Kd2, DIRECT);
PID rightPID2(&rightIrInput, &rightIrOutput, &Setpoint, Kp2, Ki2, Kd2, DIRECT);
PID leftPID2(&leftIrInput, &leftIrOutput, &Setpoint, Kp2, Ki2, Kd2, DIRECT);
PID backPID2(&backIrInput, &backIrOutput, &Setpoint, Kp2, Ki2, Kd2, DIRECT);

//Servo object declaration
Servo aleo; //Servo aleo input:aIn and output:xOut
Servo eleo; //Servo eleo input:yIn and output:yOut
Servo thro; //Servo thro input:zIn and output:zOut
Servo yawo;
Servo aux;

//Interrupt variable declaration
#define aleoFlag 1
#define eleoFlag 2
#define throFlag 4
#define yawoFlag 1
  
volatile uint8_t bUpdateFlagsShared;
volatile uint8_t cUpdateFlagsShared;

volatile uint16_t unAleoInShared;
volatile uint16_t unEleoInShared;
volatile uint16_t unThroInShared;
volatile uint16_t unYawoInShared;

uint32_t ulAleoStart;
uint32_t ulEleoStart;
uint32_t ulThroStart;
uint32_t ulYawoStart;

//mySoftwareSerial BTSerial(2, 3); // SoftwareSerial(TX, RX)
int roll=75, pitch=25, th=100, yaw=175; // bluetooth로 전해지는 값을 받음, 0~99의 값만 가진다.
int mroll=1500, mpitch=1500, mth=1000, myaw=1500; 

void setup()
{

//  if(BTSerial.available() > 0)
  {
//    BTSerial.begin(38400); 
    Serial.begin(38400); 
    Serial.flush();
  }
//  else
  {
    Serial.begin(115200);
  }
  
  escape_auto_takeoff = 0;

  //Setpoint setting
  Setpoint = 50;
  
  //SetOutputLimits setting
  frontPID.SetOutputLimits(30,100);
  rightPID.SetOutputLimits(30,100);
  backPID.SetOutputLimits(30,100);
  leftPID.SetOutputLimits(30,100);

  frontPID2.SetOutputLimits(20,200);
  rightPID2.SetOutputLimits(20,200);
  backPID2.SetOutputLimits(20,200);
  leftPID2.SetOutputLimits(20,200);

  //SetMode setting
  frontPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);
  backPID.SetMode(AUTOMATIC);
  leftPID.SetMode(AUTOMATIC);

  frontPID2.SetMode(AUTOMATIC);
  rightPID2.SetMode(AUTOMATIC);
  backPID2.SetMode(AUTOMATIC);
  leftPID2.SetMode(AUTOMATIC);
  
  //aux1 and aux2 pinMode setting
  pinMode(aux1, INPUT); //aux1 pin mode is input
  pinMode(aux2, INPUT); //aux2 pin mode is input
  pinMode(aux3, INPUT);
  pinMode(aux3Out, OUTPUT);
  
  //Attach servo setting
  aleo.attach(aOut); //aleo servo(flight controller ch1) was attached to xOut pin
  eleo.attach(eOut); //eleo servo(flight controller ch2) was attached to yOut pin
  thro.attach(tOut); //thro servo(flight controller ch3) was attached to zOut pin
  yawo.attach(yOut);
  
  aux.attach(aux3Out);
  //Attach Interrupt setting
  PCintPort::attachInterrupt(aIn, calcAleo,CHANGE); //xIn interrupt
  PCintPort::attachInterrupt(eIn, calcEleo,CHANGE); //yIn interrupt
  PCintPort::attachInterrupt(tIn, calcThro,CHANGE); //zIn interrupt
  PCintPort::attachInterrupt(yIn, calcYawo,CHANGE);

}

void loop()
{
  //aleo, eleo, thro variable declaration
  static uint16_t unAleoIn;
  static uint16_t unEleoIn;
  static uint16_t unThroIn;
  static uint16_t unYawoIn;
  
  static uint16_t autoThro;
  static uint16_t lastThro;

  static uint8_t bUpdateFlags;
  static uint8_t cUpdateFlags;

  //aux1, aux2 reads signals(microseconds)
  aux1_read = pulseIn(aux1, HIGH);
  aux2_read = pulseIn(aux2, HIGH);
  aux3_read = pulseIn(aux3, HIGH);

  if(bUpdateFlagsShared)
  {
    noInterrupts(); 
    
    bUpdateFlags = bUpdateFlagsShared;

    if(bUpdateFlags & aleoFlag)
    {
      unAleoIn = unAleoInShared;
    }
    
    if(bUpdateFlags & eleoFlag)
    {
      unEleoIn = unEleoInShared;
    }
    
    if(bUpdateFlags & throFlag)
    {
      unThroIn = unThroInShared;
    }
     
    bUpdateFlagsShared = 0;
    
    interrupts(); 
  }
  
  if(cUpdateFlagsShared)
  {
    noInterrupts(); 
    
    cUpdateFlags = cUpdateFlagsShared;

    if(cUpdateFlags & yawoFlag)
    {
      unYawoIn = unYawoInShared;
    }
         
    cUpdateFlagsShared = 0;
    
    interrupts(); 
  }

  if((aux2_read>1300)&&(aux2_read<1700))
  {

    Serial.println("auto landing mode");

    unsigned long currentMillis = millis();

    escape_auto_takeoff = 0;

    aux3_read = 1600;
    aux.writeMicroseconds(aux3_read);

    if(bUpdateFlags & aleoFlag)
    {
      if(aleo.readMicroseconds() != unAleoIn)
      {
        aleo.writeMicroseconds(unAleoIn);
      }
    }
      
    if(bUpdateFlags & eleoFlag)
    {
      if(eleo.readMicroseconds() != unEleoIn)
      {
        eleo.writeMicroseconds(unEleoIn);
      }
    }

    if(cUpdateFlags & yawoFlag)
    {
      if(yawo.readMicroseconds() != unYawoIn)
      {
       yawo.writeMicroseconds(unYawoIn);
      }
    }
 
    pulse = pulseIn(sonarIn, HIGH);
    inches = pulse/147;
    altSonarInput = inches * 2.54;
      
    if(altSonarInput>500)
    { 
      altSonarInput = lastSonarInput;
    }
     
    lastSonarInput = altSonarInput;
      
    Serial.print("altSonarInput : ");
    Serial.println(altSonarInput);

    if(currentMillis - previousMillis >= interval)
    {
      autoThro += 8;
      thro.writeMicroseconds(lastThro-autoThro);
      previousMillis = currentMillis;      
    }

    if(lastThro-autoThro < 1640)
    {
      thro.writeMicroseconds(1040);
    }

  }

  else if((aux1_read<1300)&&(aux2_read<1300) || escape_auto_takeoff == 1)
  {
      Serial.println("Basic mode");

      autoThro = 0;
      
      if(aux3_read < 1300)
      {
        aux3_read = 1000;
      }
      else if(aux3_read > 1300 && aux3_read < 1700)
      {
        aux3_read = 1600;
      }
      else
      {
        aux3_read = 2000;
      }
      
      aux.writeMicroseconds(aux3_read);

             
      if(bUpdateFlags & aleoFlag)
      {
        if(aleo.readMicroseconds() != unAleoIn)
        {
          aleo.writeMicroseconds(unAleoIn);
        }
      }
      
      if(bUpdateFlags & eleoFlag)
      {
        if(eleo.readMicroseconds() != unEleoIn)
        {
          eleo.writeMicroseconds(unEleoIn);
        }
      }
      
      if(bUpdateFlags & throFlag)
      {
        if(thro.readMicroseconds() != unThroIn)
        {
          thro.writeMicroseconds(unThroIn);
        }
      }
        
      if(cUpdateFlags & yawoFlag)
      {
        if(yawo.readMicroseconds() != unYawoIn)
        {
          yawo.writeMicroseconds(unYawoIn);
        }
      }
      
      lastThro = unThroIn;
  }
  
  else if((aux1_read>1300)&&(aux1_read<1800)&&(aux2_read<1300))
  {
      Serial.println("PingPong mode");
      
      Setpoint = 50;
      frontIrInput = fIr.cm();
      rightIrInput = rIr.cm();
      backIrInput = bIr.cm();
      leftIrInput = lIr.cm();  
        
      frontPID.Compute();
      rightPID.Compute();
      backPID.Compute();
      leftPID.Compute();

      Serial.print("FRONTInput : ");
      Serial.println(frontIrInput);
      Serial.print("    rightInput : ");
      Serial.println(rightIrInput);
      Serial.print("    backInput : ");
      Serial.println(backIrInput);
      Serial.print("    leftInput : ");
      Serial.println(leftIrInput);
      Serial.print("rightir output : ");
      Serial.println(rightIrOutput);
      
      
      if(bUpdateFlags & aleoFlag)
      {
       if(aleo.readMicroseconds() != unAleoIn)
       {
          unAleoIn = 1505-rightIrOutput+leftIrOutput;
          aleo.writeMicroseconds(unAleoIn);
        }
      }
      
      if(bUpdateFlags & eleoFlag)
      {
          unEleoIn = 1510-frontIrOutput+backIrOutput;
          eleo.writeMicroseconds(unEleoIn);
      }
      
      if(bUpdateFlags & throFlag)
      {
        if(thro.readMicroseconds() != unThroIn)
        {
          thro.writeMicroseconds(unThroIn);
        }
      }  
      
      if(cUpdateFlags & yawoFlag)
      {
        if(yawo.readMicroseconds() != unYawoIn)
        {
          yawo.writeMicroseconds(unYawoIn);
        }
      }                  
  }

  else if((aux1_read>1800)&&(aux2_read<1500))
  {
      Serial.println("IR mode");
      Setpoint = 50;
  
      frontIrInput = fIr.cm();
      rightIrInput = rIr.cm();
      backIrInput = bIr.cm();
      leftIrInput = lIr.cm();  
        
      frontPID2.Compute2();
      rightPID2.Compute2();
      backPID2.Compute2();
      leftPID2.Compute2();

      Serial.print(backIrInput);
      Serial.print(backIrInput);


      /*
      Serial.print("leftoutput : ");
      Serial.println(leftIrOutput);

      Serial.print("rightir output : ");
      Serial.println(rightIrOutput);
      */
      
      if(bUpdateFlags & aleoFlag)
      {
       if(aleo.readMicroseconds() != unAleoIn)
       {
          unAleoIn = 1503+rightIrOutput-leftIrOutput;
          aleo.writeMicroseconds(unAleoIn);
        }
      }
      
      if(bUpdateFlags & eleoFlag)
      {
       if(eleo.readMicroseconds() != unAleoIn)
       {
          unEleoIn = 1510+frontIrOutput-backIrOutput;
          eleo.writeMicroseconds(unEleoIn);
       }
      }
      
      if(bUpdateFlags & throFlag)
      {
        if(thro.readMicroseconds() != unThroIn)
        {
          thro.writeMicroseconds(unThroIn);
        }
      }
      
      if(cUpdateFlags & yawoFlag)
      {
        if(yawo.readMicroseconds() != unYawoIn)
        {
          yawo.writeMicroseconds(unYawoIn);
        }
      }              
  }

  else if((aux1_read<1500)&&(aux2_read>1700))
  { 
      Serial.println("auto take-off mode");

      if(bUpdateFlags & aleoFlag)
      {
        if(aleo.readMicroseconds() != unAleoIn)
        {
          aleo.writeMicroseconds(unAleoIn);
        }
      }
      
      if(bUpdateFlags & eleoFlag)
      {
        if(eleo.readMicroseconds() != unEleoIn)
        {
          eleo.writeMicroseconds(unEleoIn);
        }
      }
      if(cUpdateFlags & yawoFlag)
      {
        if(yawo.readMicroseconds() != unYawoIn)
        {
          yawo.writeMicroseconds(unYawoIn);
        }
      }

      pulse = pulseIn(sonarIn, HIGH);
      inches = pulse/147;
      altSonarInput = inches * 2.54;
      //만약 잔디에서 처음에값을 600이상으로 읽었다 -> 그러면 altSonarInput이 0이될 것이다. 따라서 altSonarInput의 값은 계속 0이되므로 기체의 쓰로틀이 1790일때까지는 계속해서 올라간다는것이다. 
      if(altSonarInput>500)
      { 
        altSonarInput = lastSonarInput;
      }
     
      lastSonarInput = altSonarInput;
      
      //여기서 altSOnarInput이 600이 넘어가면 전값으로 넣어줘야한다 왜냐하면 디폴트값이 나면 계속해서 기체가 상승하는 효과가 일어날 수 있다.
      //Serial.print("altSonarInput : ");
      //Serial.println(altSonarInput);
  
      if(altSonarInput < 30)
      {

        Serial.println("auto take-off mode : altSonarInput < 30");
        
        aux3_read = 1600;
        aux.writeMicroseconds(aux3_read);
      
        autoThro += 1;//autoThro가 0에서 시작하는지 확인해야 한다.
      
        if(autoThro > 90)
        {
          autoThro = 90;
        }

        // for(int i=0; i<) 한번에 1700을 주면 ESC 또는 모터에 무리가 갈 것같아서 1000에서 시작해서 1700까지 값을 올려주고 그다음에 시도하면 될듯??
        lastThro = autoThro+1700;
        thro.writeMicroseconds(autoThro+1700);
      }
      
      else
      {
        if(autoThro+1700 > unThroIn)
        {

           Serial.println("auto take-off mode : altSonarInput > 30");
           
           aux3_read = 1600;
           aux.writeMicroseconds(aux3_read);//이거 지워도 될것같은데 확인해보자
           
           lastThro = autoThro+1700;
           thro.writeMicroseconds(autoThro+1700);
           /*
           Serial.print("autoThro : ");
           Serial.print(autoThro+1700);
           Serial.print("   unThroIn : ");
           Serial.println(unThroIn);
           */
        
        }
        else
        {  
           escape_auto_takeoff = 1;
        }
      }
  }
  
  /*
  Serial.print("     aux3read : ");
  Serial.print(aux3_read);
  Serial.print("altSonarInput : ");
  Serial.println(altSonarInput);
  Serial.print("     lastThro : ");
  Serial.print(lastThro);
  Serial.print("     autoThro : ");
  Serial.print(autoThro);
  Serial.print("     unThroIn : ");
  Serial.println(unThroIn);
  */

  bUpdateFlags = 0;
  cUpdateFlags = 0;
}

  
void calcAleo()
{
  if(digitalRead(aIn) == HIGH)
  { 
    ulAleoStart = micros();
  }
  else
  {
    unAleoInShared = (uint16_t)(micros() - ulAleoStart);
    bUpdateFlagsShared |= aleoFlag;
  }
}

void calcEleo()
{
  if(digitalRead(eIn) == HIGH)
  { 
    ulEleoStart = micros();
  }
  else
  {
    unEleoInShared = (uint16_t)(micros() - ulEleoStart);
    bUpdateFlagsShared |= eleoFlag;
  }
}

void calcThro()
{
  if(digitalRead(tIn) == HIGH)
  { 
    ulThroStart = micros();
  }
  else
  {
    unThroInShared = (uint16_t)(micros() - ulThroStart);
    bUpdateFlagsShared |= throFlag;
 
  }
}

void calcYawo()
{
  if(digitalRead(yIn) == HIGH)
  { 
    ulYawoStart = micros();
  }
  else
  {
    unYawoInShared = (uint16_t)(micros() - ulYawoStart);
    cUpdateFlagsShared |= yawoFlag;
 
  }
}
