// Include the necessary libraries
#include "math.h" 
#include <AS5047P.h> //Library for the rotary encoder
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <infrastructure_msgs/Door.h>

//define pins for motor controller
#define enable_motor_channel 44 //pwm pin
#define motor_channel3  42
#define motor_channel4  43

// define pins for electromagnet driver
#define ELECTROMAGNET_PIN 2 //pwm pin

// Initialize FSRs in door frame
#define fsr_13 A12
#define fsr_14 A13
#define fsr_1 A0
#define fsr_2 A1
#define fsr_3 A2
#define fsr_4 A3
#define fsr_5 A4
#define fsr_6 A5
#define fsr_7 A6
#define fsr_8 A7
#define fsr_9 A8
#define fsr_10 A9
#define fsr_11 A10
#define fsr_12 A11

#define NUM_OF_FSRS 14


#define ENCODER_CHIP_SELECT_PORT 47
#define ENCODER_SPI_BUS_SPEED 100000

// Initialize the encoder
AS5047P encoder(ENCODER_CHIP_SELECT_PORT, ENCODER_SPI_BUS_SPEED);

// Initialize user input variables
unsigned long time;
unsigned long stoptime;

double init_angle;

//declare motor variable for time
const int time_unwind = 10000; // in ms

//ros variables
ros::NodeHandle n;
infrastructure_msgs::Door data;
std_msgs::Bool door_status;
ros::Publisher datapub("Door/Data", &data);
ros::Publisher statuspub("Door/MovementStatus", &door_status, true);

//ros callback functions for reset_door and start_door services
void reset_door_callback(const std_msgs::Empty& msg)
{
  door_status.data = true;
  statuspub.publish(&door_status);
  Reset_Door();
  door_status.data = false;
  statuspub.publish(&door_status);
}

void start_door_callback(const std_msgs::UInt8& msg)
{
  Set_Electromagnets(msg.data); //changes electromagnets based on the input of the message
}


ros::Subscriber<std_msgs::UInt8> start_sub("Door/Start_Door", &start_door_callback);
ros::Subscriber<std_msgs::Empty> reset_sub("Door/Reset_Door", &reset_door_callback);


void setup()
{
  //Start up all of the ROS stuff
  n.initNode();
  n.subscribe(start_sub);
  n.subscribe(reset_sub);
  n.advertise(datapub);
  n.advertise(statuspub);
  
  //Initialize the encoder.  Send an error message to ROS if the encoder is unable to be initialized
  uint8_t connection_counter = 0;
  while (!encoder.initSPI())
  {
    if(connection_counter >=3)
      n.logerror("Unable to connect to the encoder");

    connection_counter++;
    delay(5000);
  }

  init_angle = encoder.readAngleDegree();

  //Initialize the door status pub
  door_status.data = false;
  statuspub.publish(&door_status);

  //Initialize memory space in door msg
  data.fsr_readings = (short unsigned int *)malloc(sizeof(uint8_t) * NUM_OF_FSRS);
  data.fsr_readings_length = NUM_OF_FSRS;

  //Initialize the electromagnets as being off
  Set_Electromagnets(0);
  
  // initialize motor pins as outputs
  pinMode(motor_channel3, OUTPUT);
  pinMode(motor_channel4, OUTPUT);
  pinMode(enable_motor_channel, OUTPUT);
  // Initialize electromagnet pin as output
  pinMode(ELECTROMAGNET_PIN, OUTPUT);
  //initialize fsr's
  pinMode(fsr_1, INPUT);
  pinMode(fsr_2, INPUT);
  pinMode(fsr_3, INPUT);
  pinMode(fsr_4, INPUT);
  pinMode(fsr_5, INPUT);
  pinMode(fsr_6, INPUT);
  pinMode(fsr_7, INPUT);
  pinMode(fsr_8, INPUT);
  pinMode(fsr_9, INPUT);
  pinMode(fsr_10, INPUT);
  pinMode(fsr_11, INPUT);
  pinMode(fsr_12, INPUT);
  pinMode(fsr_13, INPUT);
  pinMode(fsr_14, INPUT);
}

void loop()
{
  collect_data();

  //Turn off the electromagnets if the door has exceeded a certain angle since the electromagnets have no effect once the door has swung out enough
  if(data.door_angle > 20)
  {
    Set_Electromagnets(0);
  }
  
  n.spinOnce();

  delay(100); //Runs loop at reasonable rate
}

void Reset_Door()
{
  bool did_move = false;
  int motor_speed = 100; //max speed for the motor

  Set_Electromagnets(0); // turns off magnets. makes motor faster
  
  analogWrite(enable_motor_channel, motor_speed); //turns motor on
  digitalWrite(motor_channel3, LOW);// turns motor
  digitalWrite(motor_channel4, HIGH); // counter clockwise
  
  while (true)
  { // door is open more than 1 degree
    n.spinOnce();
    if (encoder.readAngleDegree() - init_angle < 2  && encoder.readAngleDegree() - init_angle > -2) {
      break;
    }
    did_move = true;
  }
  if (did_move)
  {
    time = millis();
    unsigned long time_stop = time + time_unwind;
    digitalWrite(motor_channel3, HIGH); // turns motor
    digitalWrite(motor_channel4, LOW);// clockwise
    while(time < (time_stop)){
      n.spinOnce();
      time=millis();
      //testing purposes only
      //data_point.data = -5;
      //datapub.publish(&data_point);
    }
  }
  analogWrite(enable_motor_channel, 0); // turns motor off
}

void Set_Electromagnets(uint8_t electromagnet_power)
{
  analogWrite(ELECTROMAGNET_PIN, electromagnet_power);
}

void read_handle_val()
{
  data.fsr_readings[2] = analogRead(fsr_1);
  data.fsr_readings[3] = analogRead(fsr_2);
  data.fsr_readings[4] = analogRead(fsr_3);
  data.fsr_readings[5] = analogRead(fsr_4);
  data.fsr_readings[6] = analogRead(fsr_5);
  data.fsr_readings[7] = analogRead(fsr_6);
  data.fsr_readings[8] = analogRead(fsr_7);
  data.fsr_readings[9] = analogRead(fsr_8);
  data.fsr_readings[10] = analogRead(fsr_9);
  data.fsr_readings[11] = analogRead(fsr_10);
  data.fsr_readings[12] = analogRead(fsr_11);
  data.fsr_readings[13] = analogRead(fsr_12);
}

void read_encoder_val()
{
  data.door_angle = encoder.readAngleDegree() - init_angle;
}

void read_pull_force()
{
  data.fsr_readings[0] = analogRead(fsr_13);
  data.fsr_readings[1] = analogRead(fsr_14);
}

void collect_data()
{
    read_encoder_val();
    read_handle_val();
    read_pull_force();
    data.header.stamp = n.now(); //gets the time of the device roscore is running on
    datapub.publish(&data);
}
