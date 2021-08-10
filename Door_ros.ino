// Include the necessary libraries
#include "math.h" //Includes math library
#include <AS5047P.h> //Includes libray for the rotary encoder
#include <ros.h>
#include <std_srvs/Int32.h>
#include <infrastructure_srvs/UInt8.h>

//define pins for motor controller
#define enable_motor_channel 6 //pwm pin
#define motor_channel3  48
#define motor_channel4  42

// define pins for electromagnet driver
#define ELECTROMAGNET_PIN 20 //pwm pin

// Initialize FSR and Potentiometer in door frame
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

#define ENCODER_CHIP_SELECT_PORT 14
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

//ros callback functions for data_start and reset_start topics 
void reset_door_callback(const std_srvs::Empty::Request& req, const std_srvs::Empty::Response& res){
  Reset_Door();
  res.response = true;
}

void start_door_callback(const infrastructure_srvs::UInt8::Request& req, const infrastructure_srvs::UInt8::Response& res){
    Enable_Relays(req.data); // changes electromagnets based on input
    res.response = true;
}

void setup() {
  Serial.begin(57600);
  n.initNode();
  n.advertiseService("reset_door", reset_door_callback);
  n.advertiseService("start_door", start_door_callback);
  n.advertise(datapub);
  //don't run door if encoder is not working.
  //TODO: send error message to ROS here
  uint8_t connection_counter = 0;
  while (!encoder.initSPI()) {
    Serial.println(F("Can't connect to the AS5047P sensor! Please check the connection..."));
    if(connection_counter >=3)
      ROS_ERROR("Unable to connect to the encoder");

    connection_counter++;
    delay(5000);
  }
  
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
    //testing
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
  /*data_point.data = analogRead(fsr_1);
  datapub.publish(&data_point);
  data_point.data = analogRead(fsr_2);
  datapub.publish(&data_point);
  data_point.data = analogRead(fsr_3);
  datapub.publish(&data_point);
  data_point.data = analogRead(fsr_4);
  datapub.publish(&data_point);
  data_point.data = analogRead(fsr_5);
  datapub.publish(&data_point);
  data_point.data = analogRead(fsr_6);
  datapub.publish(&data_point);
  data_point.data = analogRead(fsr_7);
  datapub.publish(&data_point);
  data_point.data = analogRead(fsr_8);
  datapub.publish(&data_point);
  data_point.data = analogRead(fsr_9);
  datapub.publish(&data_point);
  data_point.data = analogRead(fsr_10);
  datapub.publish(&data_point);
  data_point.data = analogRead(fsr_11);
  datapub.publish(&data_point);
  data_point.data = analogRead(fsr_12);
  datapub.publish(&data_point);*/
  data.fsr1 = analogRead(fsr_1);
  data.fsr2 = analogRead(fsr_2);
  data.fsr3 = analogRead(fsr_3);
  data.fsr4 = analogRead(fsr_4);
  data.fsr5 = analogRead(fsr_5);
  data.fsr6 = analogRead(fsr_6);
  data.fsr7 = analogRead(fsr_7);
  data.fsr8 = analogRead(fsr_8);
  data.fsr9 = analogRead(fsr_9);
  data.fsr10 = analogRead(fsr_10);
  data.fsr11 = analogRead(fsr_11);
  data.fsr12 = analogRead(fsr_12);
}

void read_encoder_val() {
  data.door_angle = encoder.readAngleDegree();
  //data_point.data = door_angle;
  //datapub.publish(&data_point);
}

void read_pull_force() {
  data.fsr_contact_1 = analogRead(fsr_13);
  data.fsr_contact_2 = analogRead(fsr_14);
  /*data_point.data = analogRead(fsr_13);
  datapub.publish(&data_point);
  data_point.data = analogRead(fsr_14);
  datapub.publish(&data_point);*/
}

void collect_data(){
    read_encoder_val();
    read_handle_val();
    read_pull_force();
    data.current_time = n.now(); //gets the time of the device roscore is running on
    pub.publish(&data);
    //time = millis() - start_time;
    //data_point.data = time;
    //datapub.publish(&data_point);
}
