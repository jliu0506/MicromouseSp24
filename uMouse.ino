#include <Adafruit_Sensor.h>

#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_SPIDevice.h>

#include <Adafruit_LSM6DS3TRC.h>

#include <VL53L0X.h>

#include <Arduino.h>

/*Constants-------------------------------------------------------------------------*/

//Maximum distance possible for TOF sensor to see one cell
#define CELL_THRESHOLD        1

//Cell length
#define CELL_LENGTH           1

//Current mouse direction
#define FACING_UP             1
#define FACING_LEFT           2
#define FACING_DOWN           3
#define FACING_RIGHT          4

//Input pins controlling H bridge
#define LEFT_MOTOR1           A0
#define LEFT_MOTOR2           A1
#define RIGHT_MOTOR1          A2
#define RIGHT_MOTOR2          A3

//Enable pins controlling H bridge
#define LEFT_MOTOR_EN         A6
#define RIGHT_MOTOR_EN        A7

//Chip select pin on LSM
#define LSM_CS                D13

//Tof XSHUT pins
#define TOF_XSHUT_LEFT        D2
#define TOF_XSHUT_FRONT       D3
#define TOF_XSHUT_RIGHT       D4 

#define PI                    3.14159265358979324

/*Enumerations----------------------------------------------------------------------*/

//Traverse the maze decision
enum TRAVERSE
{
  TURN_LEFT,
  TURN_RIGHT,
  GO_FORWARD,
  GO_BACK
};

//If left/right is too close, correct respective side
//Flip if traversing backwards
enum CORRECT
{
  CORRECT_LEFT,
  CORRECT_RIGHT
};

/*Function declaration--------------------------------------------------------------*/

//ToF specific functions
void changeToFAddress();
void readDistance();

//LSM specific functions
void readLSM();

//Maze specific functions
void updateMaze();
void wallLogic();
void cellMove(TRAVERSE decision);
void calculatePos();

Adafruit_LSM6DS3TRC lsm6ds3trc;
VL53L0X sensorL;
VL53L0X sensorF;
VL53L0X sensorR;

//Sensor event variables
sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;

//ToF distance variables
int left;
int front;
int right;

//ToF old distance variables
int old_left;
int old_right;
int old_front;

//ToF mark distance variables
int mark_left;
int mark_front;
int mark_right;

//LSM old angle variables
double old_angle_speed{0};

//Old time variable
double old_time;

//Gyro variables
double gyroZ;

//Acceleration variables
double accelX;
double accelY;

//Absolute position variable
double posX{0};
double posY{0};

//Absolute angle variable
double angle{0};

//Mark absolute angle
double mark_angle{0};

//Current cell
int cell_x;
int cell_y;

//Facing direction, can be 1,2,3, or 4
int facing;

/*Maze matrix-----------------------------------------------------------------------*/

//Each cell will sense how many walls are 
//3rd dimension: 0-> distance from center, 1-4 -> NWSE
uint8_t maze[16][16][5] = {0};


/*Structures------------------------------------------------------------------------*/

//Mouse facing direction linked list
struct node
{
  //Data
  int data;

  //Pointer to counter-clockwise node
  node *cc;

  //Pointer to clockwise node
  node *c;
};

/*Linked List Declaration-----------------------------------------------------------*/

node* north = new node();
node* west = new node();
node* south = new node();
node* east = new node();

node* current = north;
/*Setup-----------------------------------------------------------------------------*/

void setup(void)
{
  //Direction steup
  north->data = FACING_UP;
  west->data = FACING_LEFT;
  south->data = FACING_DOWN;
  east->data = FACING_RIGHT;
  
  north->cc = west;
  north->c = east;

  west->cc = south;
  west->c = north;

  south->cc = east;
  south->c = west;

  east->cc = north;
  east->c = south;

  //Pin mode configurations
  //Motor input pins
  pinMode(LEFT_MOTOR1,OUTPUT);
  pinMode(LEFT_MOTOR2,OUTPUT);
  pinMode(RIGHT_MOTOR1,OUTPUT);
  pinMode(RIGHT_MOTOR2,OUTPUT);

  //Motor enable pins
  pinMode(LEFT_MOTOR_EN,OUTPUT);
  pinMode(RIGHT_MOTOR_EN,OUTPUT);

  //LSM chip select pin
  pinMode(LSM_CS,OUTPUT);

  //ToF XSHUT pins
  pinMode(TOF_XSHUT_LEFT,OUTPUT);
  pinMode(TOF_XSHUT_FRONT,OUTPUT);
  pinMode(TOF_XSHUT_RIGHT,OUTPUT);
  
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  if (!lsm6ds3trc.begin_I2C()) {
    Serial.println("Failed to find LSM6DS3TR-C chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("LSM6DS3TR-C Found!");

  //Initialize ToF sensors
  changeToFAddress();
}

/*Loop-----------------------------------------------------------------------------*/

void loop() {

  /*
  //Read gyro on z, accel on x and y
  readLSM();

  //Read all ToF sensors
  readDistance();

  //Calculate position

  delay(200);
  */

  //Movement
  cellMove(GO_FORWARD);

  //
}

/*Function definitions-------------------------------------------------------------*/

//ToF change address
void changeToFAddress()
{
  //Pull low only one XSHUT pin since it is active LOW
  digitalWrite(TOF_XSHUT_LEFT,LOW);
  digitalWrite(TOF_XSHUT_FRONT,HIGH);
  digitalWrite(TOF_XSHUT_RIGHT,HIGH);

  //Left sensor
  sensorL.setTimeout(500);
  if (!sensorL.init())
  {
    Serial.println("Failed to detect and initialize left sensor!");
    while (1) {}
  }
  sensorL.setAddress((uint8_t)76);

  digitalWrite(TOF_XSHUT_LEFT,HIGH);
  digitalWrite(TOF_XSHUT_FRONT,LOW);
  digitalWrite(TOF_XSHUT_RIGHT,HIGH);

  //Front sensor
  sensorF.setTimeout(500);
  if (!sensorF.init())
  {
    Serial.println("Failed to detect and initialize front sensor!");
    while (1) {}
  }
  sensorF.setAddress((uint8_t)70);

  digitalWrite(TOF_XSHUT_LEFT,HIGH);
  digitalWrite(TOF_XSHUT_FRONT,HIGH);
  digitalWrite(TOF_XSHUT_RIGHT,LOW);

  //Right sensor
  sensorR.setTimeout(500);
  if (!sensorR.init())
  {
    Serial.println("Failed to detect and initialize right sensor!");
    while (1) {}
  }
  sensorR.setAddress((uint8_t)82);

  //High speed
  sensorL.setMeasurementTimingBudget(20000);
  sensorF.setMeasurementTimingBudget(20000);
  sensorR.setMeasurementTimingBudget(20000);
}

//Read distance and put into variables
void readDistance()
{
  //Update old values
  old_left = left;
  old_right = right;
  old_front = front;

  left = sensorL.readRangeSingleMillimeters();
  if (sensorL.timeoutOccurred()) { Serial.print(" TIMEOUT on Left sensor"); }

  front = sensorF.readRangeSingleMillimeters();
  if (sensorF.timeoutOccurred()) { Serial.print(" TIMEOUT on Front sensor"); }

  right = sensorR.readRangeSingleMillimeters();
  if (sensorR.timeoutOccurred()) { Serial.print(" TIMEOUT on Right sensor"); }
}

//Read gyro and put into variables (in rad/s)
void readLSM()
{

  //New normalized event
  lsm6ds3trc.getEvent(&accel, &gyro, &temp);

  gyroZ = gyro.gyro.z;
  accelX = accel.acceleration.x;
  accelY = accel.acceleration.y;
}

void updateMaze()
{
  //First read the distance from wall
  readDistance();

  //Then do logic to update maze
  wallLogic();
}

//Configured so that the mouse always thinks it's facing NORTH at the beginning
void wallLogic()
{
  if(left <= CELL_THRESHOLD)
  {
    maze[cell_x][cell_y][current->cc->data] = true;
  }
  if(right <= CELL_THRESHOLD)
  {
    maze[cell_x][cell_y][current->c->data] = true;
  }
  if(front <= CELL_THRESHOLD)
  {
    maze[cell_x][cell_y][current->data] = true;
  }
}

void calculatePos()
{
  //Have to take direction into account

  //If currently facing north or south
  if(current->data%2)
  {
    //First sees if we should add or subtract to absolute pos coord
    //Then calculates the distance elapsed relative to old position
    posY += pow(-1,(current->data - 1)/2)*(old_front - front);
  }
  else
  {
    posX += pow(-1,(current->data/2))*(old_front - front);
  }
}

void cellMove(TRAVERSE decision)
{
  //Take absolute coords and move ourselves up one cell
  
  //First disable enable pin to make sure it's all synchronized
  digitalWrite(LEFT_MOTOR_EN, LOW);
  digitalWrite(RIGHT_MOTOR_EN,LOW);

  switch(decision)
  {
    //Read the distance first
    readDistance();
    mark_left = old_left;
    mark_front = old_front;
    mark_right = old_right;

    case GO_FORWARD:
      //Left motor
      digitalWrite(LEFT_MOTOR1,HIGH);
      digitalWrite(LEFT_MOTOR2,LOW);

      //Right motor
      digitalWrite(RIGHT_MOTOR1,HIGH);
      digitalWrite(RIGHT_MOTOR2,LOW);
      break;
    case TURN_LEFT:
      //Get time mark at the beginning of call
      old_time = millis();

      //Left motor
      digitalWrite(LEFT_MOTOR1,LOW);
      digitalWrite(LEFT_MOTOR2,HIGH);

      //Right motor
      digitalWrite(RIGHT_MOTOR1,HIGH);
      digitalWrite(RIGHT_MOTOR2,LOW);
      break;
    case TURN_RIGHT:
      //Get time mark at the beginning of call
      old_time = millis();

      //Left motor
      digitalWrite(LEFT_MOTOR1,HIGH);
      digitalWrite(LEFT_MOTOR2,LOW);

      //Right motor
      digitalWrite(RIGHT_MOTOR1,LOW);
      digitalWrite(RIGHT_MOTOR2,HIGH);
      break;
    case GO_BACK:
      //Left motor
      digitalWrite(LEFT_MOTOR1,LOW);
      digitalWrite(LEFT_MOTOR2,HIGH);

      //Right motor
      digitalWrite(RIGHT_MOTOR1,LOW);
      digitalWrite(RIGHT_MOTOR2,HIGH);
      break;
  } 

  //For now, I'm just going to put this at the highest speed setting
  digitalWrite(LEFT_MOTOR_EN,HIGH);
  digitalWrite(RIGHT_MOTOR_EN,HIGH);

  switch(decision)
  {
    case GO_FORWARD:
      //Move forward until the mouse knows it has travelled one cell threshold
      while(mark_front - front <= CELL_THRESHOLD)
      {
        //Keep updating until it moves one threshold
        readDistance();
      }
      digitalWrite(LEFT_MOTOR_EN,LOW);
      digitalWrite(RIGHT_MOTOR_EN,LOW);
      break;
    case GO_BACK:
      //Same concept
      //Move forward until the mouse knows it has travelled one cell threshold
      while(front - mark_front <= CELL_THRESHOLD)
      {
        //Keep updating until it moves one threshold
        readDistance();
      }
      digitalWrite(LEFT_MOTOR_EN,LOW);
      digitalWrite(RIGHT_MOTOR_EN,LOW);
      break;
    case TURN_LEFT:
    case TURN_RIGHT:
      //We are allowed to do this because gyroZ will automatically be either positive or negative depending on turn direction

      //Mark down angle for comparison
      mark_angle = angle;

      while((fmod((angle - mark_angle),2*PI)) < PI/2)
      {
        //Read LSM to get rad/s values
        readLSM();
        
        //Delta angle/sec * delta t = total rads
        angle += gyroZ*(millis() - old_time);
      }
      break;
  }
}
