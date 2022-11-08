
#include <esp_now.h>
#include <WiFi.h>

#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4
#define FORWARD_LEFT 5
#define FORWARD_RIGHT 6
#define BACKWARD_LEFT 7
#define BACKWARD_RIGHT 8
#define CLOCKWISE 9
#define ANTICLOCKWISE 10
#define STOP 0

#define BACK_RIGHT_MOTOR 1
#define BACK_LEFT_MOTOR 2
#define FRONT_RIGHT_MOTOR 3
#define FRONT_LEFT_MOTOR 4

#define MAX_MOTOR_SPEED 120 // about 50% duty cycle

unsigned long lastTimeStamp = 0;

//FRONT RIGHT MOTOR
int enableFrontRightMotor = 14; 
int FrontRightMotorPin1 = 26;
int FrontRightMotorPin2 = 27;
//BACK RIGHT MOTOR
int enableBackRightMotor=22; 
int BackRightMotorPin1=16;
int BackRightMotorPin2=21;
//FRONT LEFT MOTOR
int enableFrontLeftMotor = 32;
int FrontLeftMotorPin1 = 33;
int FrontLeftMotorPin2 = 25;
//BACK LEFT MOTOR
int enableBackLeftMotor = 23;
int BackLeftMotorPin1 = 18;
int BackLeftMotorPin2 = 19;

const int PWMFreq = 1000; /* 1 KHz */
const int PWMResolution = 8; // 8-bit
const int PWMSpeedChannel_1 = 1; // 0-15
const int PWMSpeedChannel_2 = 2; // 0-15
const int PWMSpeedChannel_3 = 3; // 0-15
const int PWMSpeedChannel_4 = 4; // 0-15

void moveCar(int inputValue)
{
  switch(inputValue)
  {
    case FORWARD:
      rotateMotor(FRONT_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, MAX_MOTOR_SPEED);                  
      break;
  
    case BACKWARD:
      rotateMotor(FRONT_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, -MAX_MOTOR_SPEED);   
      break;
  
    case LEFT:
      rotateMotor(FRONT_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, MAX_MOTOR_SPEED);   
      break;
  
    case RIGHT:
      rotateMotor(FRONT_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, -MAX_MOTOR_SPEED);  
      break;
     // case RIGHT:
      //rotateMotor(FRONT_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      //rotateMotor(BACK_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      //rotateMotor(FRONT_LEFT_MOTOR, MAX_MOTOR_SPEED);
      //rotateMotor(BACK_LEFT_MOTOR, -MAX_MOTOR_SPEED);  
      //break;
  
    case FORWARD_LEFT:
      rotateMotor(FRONT_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, STOP);
      rotateMotor(FRONT_LEFT_MOTOR, STOP);
      rotateMotor(BACK_LEFT_MOTOR, MAX_MOTOR_SPEED);  
      break;
  
    case FORWARD_RIGHT:
      rotateMotor(FRONT_RIGHT_MOTOR, STOP);
      rotateMotor(BACK_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, STOP);  
      break;
  
    case BACKWARD_LEFT:
      rotateMotor(FRONT_RIGHT_MOTOR, STOP);
      rotateMotor(BACK_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, STOP);   
      break;
  
    case BACKWARD_RIGHT:
      rotateMotor(FRONT_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, STOP);
      rotateMotor(FRONT_LEFT_MOTOR, STOP);
      rotateMotor(BACK_LEFT_MOTOR, -MAX_MOTOR_SPEED);   
      break;
  
    case CLOCKWISE:
      rotateMotor(FRONT_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, -MAX_MOTOR_SPEED);  
      break;
  
    case ANTICLOCKWISE:
      rotateMotor(FRONT_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, MAX_MOTOR_SPEED);   
      break;
  
    case STOP:
      rotateMotor(FRONT_RIGHT_MOTOR, STOP);
      rotateMotor(BACK_RIGHT_MOTOR, STOP);
      rotateMotor(FRONT_LEFT_MOTOR, STOP);
      rotateMotor(BACK_LEFT_MOTOR, STOP);    
      break;
  
    default:
      rotateMotor(FRONT_RIGHT_MOTOR, STOP);
      rotateMotor(BACK_RIGHT_MOTOR, STOP);
      rotateMotor(FRONT_LEFT_MOTOR, STOP);
      rotateMotor(BACK_LEFT_MOTOR, STOP);    
      break;
  }
}

void rotateMotor(int motorNumber, int motorSpeed)
{
    if (motorSpeed < 0 && motorNumber == 1)
    {
    digitalWrite(BackRightMotorPin1,LOW);
    digitalWrite(BackRightMotorPin2,HIGH);  
    }
    else if (motorSpeed < 0 && motorNumber == 2)  
    { 
    digitalWrite(BackLeftMotorPin1,LOW);
    digitalWrite(BackLeftMotorPin2,HIGH);
    }
    else if (motorSpeed < 0 && motorNumber == 3)
    {
    digitalWrite(FrontRightMotorPin1,LOW);
    digitalWrite(FrontRightMotorPin2,HIGH);  
    }
    else if (motorSpeed < 0 && motorNumber == 4)  
    { 
    digitalWrite(FrontLeftMotorPin1,LOW);
    digitalWrite(FrontLeftMotorPin2,HIGH);
    } 

    else if (motorSpeed > 0 && motorNumber == 1)
    {
    digitalWrite(BackRightMotorPin1,HIGH);
    digitalWrite(BackRightMotorPin2,LOW);
    }
    else if (motorSpeed > 0 && motorNumber == 2)
    {
    digitalWrite(BackLeftMotorPin1,HIGH);
    digitalWrite(BackLeftMotorPin2,LOW);
    }
    else if (motorSpeed > 0 && motorNumber == 3)
    {
    digitalWrite(FrontRightMotorPin1,HIGH);
    digitalWrite(FrontRightMotorPin2,LOW);  
    }
    else if (motorSpeed > 0 && motorNumber == 4)  
    { 
    digitalWrite(FrontLeftMotorPin1,HIGH);
    digitalWrite(FrontLeftMotorPin2,LOW);
    }

    else if (motorSpeed == 0 && motorNumber == 1)
    {
    digitalWrite(FrontRightMotorPin1,LOW);
    digitalWrite(FrontRightMotorPin2,LOW);     
    }
    else if (motorSpeed == 0 && motorNumber == 2)
    {
    digitalWrite(FrontLeftMotorPin1,LOW);
    digitalWrite(FrontLeftMotorPin2,LOW);
    }
    else if (motorSpeed == 0 && motorNumber == 3)
    {
    digitalWrite(BackRightMotorPin1,LOW);
    digitalWrite(BackRightMotorPin2,LOW);  
    }
    else if (motorSpeed == 0 && motorNumber == 4)
    {   
    digitalWrite(BackLeftMotorPin1,LOW);
    digitalWrite(BackLeftMotorPin2,LOW);   
    }
    ledcWrite(PWMSpeedChannel_1, abs(motorSpeed));
    ledcWrite(PWMSpeedChannel_2, abs(motorSpeed));
    ledcWrite(PWMSpeedChannel_3, abs(motorSpeed));
    ledcWrite(PWMSpeedChannel_4, abs(motorSpeed));
}

void setUpPinModes()
{
  pinMode(enableFrontRightMotor,OUTPUT);
  pinMode(FrontRightMotorPin1,OUTPUT);
  pinMode(FrontRightMotorPin2,OUTPUT);
  
  pinMode(enableFrontLeftMotor,OUTPUT);
  pinMode(FrontLeftMotorPin1,OUTPUT);
  pinMode(FrontLeftMotorPin2,OUTPUT);

  pinMode(enableBackRightMotor,OUTPUT);
  pinMode(BackRightMotorPin1,OUTPUT);
  pinMode(BackRightMotorPin2,OUTPUT);
  
  pinMode(enableBackLeftMotor,OUTPUT);
  pinMode(BackLeftMotorPin1,OUTPUT);
  pinMode(BackLeftMotorPin2,OUTPUT);

  //Set up PWM for motor speed
  ledcSetup(PWMSpeedChannel_1, PWMFreq, PWMResolution);
  ledcSetup(PWMSpeedChannel_2, PWMFreq, PWMResolution);
  ledcSetup(PWMSpeedChannel_3, PWMFreq, PWMResolution);
  ledcSetup(PWMSpeedChannel_4, PWMFreq, PWMResolution);
  ledcAttachPin(enableBackRightMotor, PWMSpeedChannel_1);
  ledcAttachPin(enableBackLeftMotor, PWMSpeedChannel_2);
  ledcAttachPin(enableFrontRightMotor, PWMSpeedChannel_3);
  ledcAttachPin(enableFrontLeftMotor, PWMSpeedChannel_4); 
    
  rotateMotor(BACK_RIGHT_MOTOR, 0);
  rotateMotor(BACK_LEFT_MOTOR, 0);
  rotateMotor(FRONT_RIGHT_MOTOR, 0);
  rotateMotor(FRONT_LEFT_MOTOR, 0);
}

// structure example to receive data
// must be matched to the sender structure
typedef struct struct_message {
    int forward;
    int backward;
    int left;
    int right;
    int clockwise;
    int anticlockwise;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("forward: ");
  Serial.println(myData.forward);
  Serial.print("backward: ");
  Serial.println(myData.backward);
  Serial.print("left: ");
  Serial.println(myData.left);
  Serial.print("right: ");
  Serial.println(myData.right);
  Serial.println();
  Serial.println(myData.clockwise);
  Serial.println();
  Serial.println(myData.anticlockwise);
  Serial.println();
  
}

void setup() {
  // put your setup code here, to run once:
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_MODE_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

  setUpPinModes();
}
 
void loop() {
  // put your main code here, to run repeatedly:


  if (myData.forward == 1) {moveCar(FORWARD);}
  else if (myData.backward == 1) {moveCar(BACKWARD);}
  else if (myData.left == 1) {moveCar(LEFT);}
  else if (myData.right == 1) {moveCar(RIGHT);}
  else if (myData.clockwise == 1) {moveCar(CLOCKWISE);}
  else if (myData.anticlockwise == 1) {moveCar(ANTICLOCKWISE);}
//  else if (LStickY > 50 && LStickX < -50) {moveCar(FORWARD_LEFT);}
//  else if (LStickY > 50 && LStickX > 50) {moveCar(FORWARD_RIGHT);}
//  else if (LStickY < -50 && LStickX < -50) {moveCar(BACKWARD_LEFT);}
//  else if (LStickY < -50 && LStickX > 50) {moveCar(BACKWARD_RIGHT);}
  else{moveCar(STOP);}
  
  
}
