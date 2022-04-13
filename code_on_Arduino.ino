#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;


//pin definition for motor
int ML_Ctrl = 4;    //define the direction control pin of B motor
int ML_PWM = 5;   //define the PWM control pin of B motor
int MR_Ctrl = 2;    //define the direction control pin of A motor
int MR_PWM = 9;   //define the PWM control pin of A motor

//pin definition for ultrasonic sensor
int trigPin = 12;
int echoPin = 13;

//variable definition
int x, y;  //coordinate diff in x and y axis, used for movement instruction
int testSpeed = 230;
int move_time = 1000;   //specify standard turing time
int turn_time = 500;

//var definition - ultrasonic sensor
double cm, duration, distance;    //measurement from ultrasonic sensor  
int safe_dist = 50;
bool flag = 0;

//var definition - Gyro
int testSpeed1 = 150;
int testSpeed2 = 160;
unsigned long timer = 0;
double timeStep = 0.01;
double turningAngle;//don't set it to zero.
double store_angle=0;//may not needed

//var definition for instruction transfer
const int NUMBER_OF_FIELDS = 2; // how many comma separated fields we expect
int fieldIndex = 0;            // the current field being received
int values[NUMBER_OF_FIELDS];   // array holding values for all the fields
int sign[NUMBER_OF_FIELDS];
String instCheck;
bool instDone = false;
String instruction;


void setup() {
  Serial.begin (9600);
  
  //motor pin setup
  pinMode(ML_Ctrl, OUTPUT);//Right side motor direction
  pinMode(ML_PWM, OUTPUT);//Right side motor direction
  pinMode(MR_Ctrl, OUTPUT);//Left side motor direction
  pinMode(MR_PWM, OUTPUT);//Left side motor speed
  
  //ultrasonic sensor pin setup
  pinMode (trigPin, OUTPUT);
  pinMode (echoPin, INPUT);

  //Gyro setup
  //Initialize MPU6050
  gyro_setup();
 
}


//next section defines the functions for move forward, backward, right turn and left turn
void forward(){ // move forward
      ultrasonic_avoid();
      digitalWrite(ML_Ctrl, HIGH); 
      analogWrite(ML_PWM, 255); 
      digitalWrite(MR_Ctrl, LOW); 
      analogWrite(MR_PWM, 175);
      delay(move_time);
}

void rightturn(){ // turn right
      turningAngle=0;
      while(1){ 
        timer = millis();
  
        // Read normalized values
        Vector norm = mpu.readNormalizeGyro();
        turningAngle = turningAngle + norm.XAxis * timeStep;
        delay((timeStep*1000) - (millis() - timer));
        digitalWrite(ML_Ctrl, LOW); 
        analogWrite(ML_PWM, testSpeed1); 
        digitalWrite(MR_Ctrl, LOW);
        analogWrite(MR_PWM, testSpeed1);
        //Serial.println(turningAngle);
       
        if(turningAngle>-88&&turningAngle<=0){
          digitalWrite(ML_Ctrl, LOW); 
          analogWrite(ML_PWM, testSpeed1); 
          digitalWrite(MR_Ctrl, LOW);
          analogWrite(MR_PWM, testSpeed1);
        }else if (turningAngle<-92){
          digitalWrite(ML_Ctrl, HIGH); 
          analogWrite(ML_PWM, testSpeed1); 
          digitalWrite(MR_Ctrl, HIGH);
          analogWrite(MR_PWM, testSpeed1);
        }else{
          digitalWrite(ML_Ctrl, HIGH); 
          analogWrite(ML_PWM, 0); 
          digitalWrite(MR_Ctrl, HIGH); 
          analogWrite(MR_PWM, 0);
          break; 
        }   
      } 
}

void leftturn(){ // turn left
      turningAngle=0;
      while(1){
        timer = millis();
  
        // Read normalized values
        Vector norm = mpu.readNormalizeGyro();
        turningAngle = turningAngle + norm.XAxis * timeStep;
        delay((timeStep*1000) - (millis() - timer));
        digitalWrite(ML_Ctrl, HIGH); 
        analogWrite(ML_PWM, testSpeed2); 
        digitalWrite(MR_Ctrl, HIGH);
        analogWrite(MR_PWM, testSpeed2); 
        //Serial.println(turningAngle);
        if(turningAngle<86&&turningAngle>=0){
          digitalWrite(ML_Ctrl, HIGH); 
          analogWrite(ML_PWM, testSpeed2); 
          digitalWrite(MR_Ctrl, HIGH);
          analogWrite(MR_PWM, testSpeed2);
        }else if (turningAngle>92){
          digitalWrite(ML_Ctrl, LOW); 
          analogWrite(ML_PWM, testSpeed1); 
          digitalWrite(MR_Ctrl, LOW);
          analogWrite(MR_PWM, testSpeed1);
        }else{
          digitalWrite(ML_Ctrl, HIGH); 
          analogWrite(ML_PWM, 0); 
          digitalWrite(MR_Ctrl, HIGH); 
          analogWrite(MR_PWM, 0);
          break; 
        }
      }
}

void backward(){ // turn around
      leftturn();
      delay(500);
      leftturn();
}

void stop(){  //stop 
      digitalWrite(ML_Ctrl, HIGH); 
      analogWrite(ML_PWM, 0); 
      digitalWrite(MR_Ctrl, HIGH); 
      analogWrite(MR_PWM, 0);
      delay(200);
}


//next section defines how arduino execute instruction sent from pi
void execute_move(int x1, int y1){  // execute the move instruction from raspberry pi
  if (x1 == -1){
    if (y1 == -1){
      stop(); delay(500); leftturn(); delay(500); stop(); delay(500); forward();
      stop(); delay(500); leftturn(); delay(500); stop(); delay(500); forward();
      stop(); delay(500); backward(); stop();
    }else if (y1 == 0){
      stop(); delay(500); leftturn(); delay(500); stop(); delay(500); forward();
      stop(); delay(500); rightturn(); delay(500); stop();
    }else if(y1 == 1){
      stop(); delay(500); leftturn(); delay(500); stop(); delay(500); forward();
      stop(); delay(500); rightturn(); delay(500); stop(); delay(500); forward();
      stop();
    }  
  }else if (x1 == 0){
    if(y1 == -1){
      stop(); delay(500); backward(); delay(500); stop(); delay(500); forward();
      stop(); delay(500); backward(); delay(500); stop();
    }else if (y1 == 0){
      stop(); delay(500);
    }else if (y1 == 1){
      stop(); delay(500); forward(); stop();
    }
  }else if (x1 == 1){
    if(y1 == -1){
      stop(); delay(500); rightturn(); delay(500); stop(); delay(500); forward();
      stop(); delay(500); rightturn(); delay(500); stop(); delay(500); forward();
      stop(); delay(500); backward(); delay(500);stop();
    }else if (y1 == 0){
      stop; delay(500); rightturn(); delay(500); stop(); delay(500); forward();
      stop(); delay(500); leftturn(); delay(500); stop();
      //instDone = true;
    }else if(y1 == 1){
      stop(); delay(500); rightturn(); delay(500); stop(); delay(500); forward();
      stop(); delay(500); leftturn(); delay(500); stop(); delay(500); forward();
      stop();
    }
  }
}



// functions for ultrasonic sensor  
double read_ultrasonic(){
  double time;
  // initialize: turn off the signal for 5 microseconds
  digitalWrite (trigPin, LOW);
  delayMicroseconds (5);

  // turn on the sensor and let it emit the signal for 10 microseconds (8 pulses)
  digitalWrite (trigPin, HIGH);
  delayMicroseconds (10);
  // turn off 
  digitalWrite (trigPin, LOW);

  // set echopin active to receive the signal sent reflected back
  // pulseIn calculate the time used between sending out and receiving the ultrasonic signal
  duration = pulseIn (echoPin, HIGH);
  // calculate the distance in cm
  cm = (duration/2)/29.1;
  
  return cm;
}

void ultrasonic_avoid(){
  Serial.println("Checking obstacles!");
  //Serial.println();
  delay(2000);
  //Serial.println();
  //forward();

  distance = read_ultrasonic();
  Serial.println(distance);
  //Serial.println();
  delay(2000);
  if (distance < safe_dist){
    Serial.println("Distance less than 50 cm, obstacle ahead detected!");
    //Serial.println();
    delay(2000);
    while (distance < safe_dist){
      distance = read_ultrasonic();
      Serial.println("Wait until obstacle disappear!");
      Serial.println(distance);
      delay(1000);
    }
    Serial.println("Interrupt finish! Go!");
  }
}



// functions for Gyro
void gyro_setup()
{
    // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    //Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  
  // Calibrate gyroscope. The calibration must be at rest.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  mpu.setThreshold(3);
}



//next section defines the functions for data transfer between pi and arduino
void instruction_update(int new_x, int new_y){
   x = new_x;
   y = new_y;
}

//fill sign value with 1s
void reset_data(){
    for (int j = 0; j <= fieldIndex; j++){
      values[j] = 0;
      sign[j] = 1;
    }
    fieldIndex = 0;  // ready to start over
}


void getInstruction()
{
    //while(Serial.available())
    instruction = Serial.readStringUntil('\n');
    //Serial.print("instruction: ");
    for (int i = 0; i < instruction.length(); i++)
    {
        char ch = instruction[i];
        if(ch >= '0' && ch <= '9')
        {
            values[fieldIndex] = (values[fieldIndex] * 10) + (ch - '0'); 
            //Serial.write(ch);   
        }
        else if(ch == '-')
        {
            sign[fieldIndex] = -1;
            //Serial.write(ch);
        }
        else if (ch == ',')
        {  
            if(fieldIndex < NUMBER_OF_FIELDS-1)
            {
                fieldIndex++; 
            }
            //Serial.write(ch);
        }
    }
    for(int i = 0; i <= fieldIndex; i++)
    {
       values[i] = values[i]*sign[i];   
    }
    instruction_update(values[0],values[1]);
    sendFeedback(x,y);
    //reset_data(); 
}
  
void sendFeedback(int sendValue1, int sendValue2){
    Serial.print(String(sendValue1)+","+String(sendValue2));
    Serial.write('\n');
}


//main function
void loop()
{ 
   while(Serial.available())
   {
      getInstruction();
      instCheck = Serial.readStringUntil('\n');
      if(instCheck == "ck"){
          Serial.print("T");
          Serial.write('\n'); 


          execute_move(x,y);

          instDone = true;
          while(instDone == false){
              delay(500);
          }
          Serial.print("instDone");
          Serial.write('\n'); 
               
      }else{
          Serial.print("F");
          Serial.write('\n'); 
      }
   }
   reset_data();
}
