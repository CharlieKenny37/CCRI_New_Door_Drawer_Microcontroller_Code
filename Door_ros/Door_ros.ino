// Include the necessary libraries
#include "math.h" 
#include <AS5047P.h> //Library for the rotary encoder
#include <ros.h>
#include <std_srvs/Empty.h>
#include <infrastructure_msgs/Door.h>
#include <infrastructure_msgs/UInt8.h>

//define pins for motor controller
#define enable_motor_channel 6 //pwm pin
#define motor_channel3  48
#define motor_channel4  42

// define pins for electromagnet driver
#define ELECTROMAGNET_PIN 20 //pwm pin

// Initialize FSRs in door frame
#define fsr_13 A0
#define fsr_14 A3
#define fsr_1 A7
#define fsr_2 A15
#define fsr_3 A10
#define fsr_4 A4
#define fsr_5 A12
#define fsr_6 A6
#define fsr_7 A5
#define fsr_8 A13
#define fsr_9 A8
#define fsr_10 A14
#define fsr_11 A9
#define fsr_12 A11

#define NUM_OF_FSRS 14


#define ENCODER_CHIP_SELECT_PORT 5
#define ENCODER_SPI_BUS_SPEED 100000

// Initialize the encoder
AS5047P encoder(ENCODER_CHIP_SELECT_PORT, ENCODER_SPI_BUS_SPEED);

// Initialize user input variables
unsigned long time;
unsigned long stoptime;
unsigned long start_time;

//declare motor variable for time
const int time_unwind = 2000; // in ms

//ros variables
ros::NodeHandle n;
infrastructure_msgs::Door data;
ros::Publisher datapub("door_data", &data);

//ros callback functions for reset_door and start_door services
void reset_door_callback(const std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
  Reset_Door();
}

void start_door_callback(const infrastructure_msgs::UInt8::Request& req, infrastructure_msgs::UInt8::Response& res){
  Set_Electromagnets(req.resistance); // changes electromagnets based on input
}

void setup() {
  Serial.begin(57600);
  n.initNode();
  ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> reset_door_server("reset_door",&reset_door_callback);
  ros::ServiceServer<infrastructure_msgs::UInt8::Request, infrastructure_msgs::UInt8::Response> start_door_server("start_door",&start_door_callback);
  n.advertiseService(reset_door_server);
  n.advertiseService(start_door_server);
  n.advertise(datapub);
  //don't run door if encoder is not working.
  //TODO: send error message to ROS here
  uint8_t connection_counter = 0;
  while (!encoder.initSPI()) {
    Serial.println(F("Can't connect to the AS5047P sensor! Please check the connection..."));
    if(connection_counter >=3)
      n.logerror("Unable to connect to the encoder");

    connection_counter++;
    delay(5000);
  }

  //Initialize space in door msg
  data.fsr_readings = (short unsigned int *)malloc(sizeof(uint8_t) * NUM_OF_FSRS);
  data.fsr_readings_length = NUM_OF_FSRS;
  
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

void loop() {
  start_time = millis();
  collect_data();
  n.spinOnce();

  delay(20); //Runs loop at reasonable rate
}

void Reset_Door() {
  bool did_move = false;
  int motor_speed = 100; //max speed for the motor

  Set_Electromagnets(0); // turns off magnets. makes motor faster
  
  analogWrite(enable_motor_channel, motor_speed); //turns motor on
  digitalWrite(motor_channel3, LOW);// turns motor
  digitalWrite(motor_channel4, HIGH); // counter clockwise
  
  while (true) { // door is open more than 1 degree
    n.spinOnce();
    if (encoder.readAngleDegree() < 2) {
      break;
    }
    did_move = true;
  }
  if (did_move) {
    time = millis();
    unsigned long time_stop = time + time_unwind;
    digitalWrite(motor_channel3, HIGH); // turns motor
    digitalWrite(motor_channel4, LOW);// clockwise
    while(time < (time_stop)){
      n.spinOnce();
      time=millis();
      //testing purposes only
      /*data_point.data = -5;
      datapub.publish(&data_point);*/
    }
  }
  analogWrite(enable_motor_channel, 0); // turns motor off
}

void Set_Electromagnets(uint8_t electromagnet_power)
{
  analogWrite(ELECTROMAGNET_PIN, electromagnet_power);
}

void read_handle_val() {
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

void read_encoder_val() {
  data.door_angle = encoder.readAngleDegree();
}

void read_pull_force() {
  data.fsr_readings[0] = analogRead(fsr_13);
  data.fsr_readings[1] = analogRead(fsr_14);
}

void collect_data(){
    read_encoder_val();
    read_handle_val();
    read_pull_force();
    data.header.stamp = n.now(); //gets the time of the device roscore is running on
    datapub.publish(&data);
}
