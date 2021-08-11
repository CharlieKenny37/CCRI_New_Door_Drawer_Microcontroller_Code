//necessary libraries
#include <AccelStepper.h>
#include "Adafruit_VL53L0X.h" //TOF sensor library
#include <ros.h>
#include <std_srvs/Empty.h>
#include <infrastructure_msgs/Drawer.h>
#include <infrastructure_msgs/UInt8.h>

//reset motor pins
#define pulse_reset 3
#define direction_reset 2

//friction motor pins
#define pulse_friction 9
#define direction_friction 8

//FSR402 data pins (from handle)
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

#define NUM_OF_FSRS 12

//Init the stepper motors
AccelStepper reset_motor(1, pulse_reset, direction_reset);
AccelStepper friction_motor(1, pulse_friction, direction_friction);

//Init tof sensor
Adafruit_VL53L0X tof = Adafruit_VL53L0X();

int start_pos; //distance of drawer from tof in starting pos, the '0' position.
const int buffer_val = 5; //buffer value for tof sensor for calculating distance
const float fric_steps = .00032; //constant that represents relation between friction setting to motor steps
const float base_friction = .3; //min resistance that drawer has
const int min_steps = 2500; //min steps it takes to get brake to touch drawer fin

const int time_unwind = 4000; //in ms
unsigned long time;
unsigned long time_stop;

//ros variables
ros::NodeHandle n;
infrastructure_msgs::Drawer data;
ros::Publisher datapub("drawer_data", &data);

//ros callback functions for start_drawer and reset_drawer services
void reset_drawer_callback(const std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
  reset_drawer();
}

void start_drawer_callback(const infrastructure_msgs::UInt8::Request& req, infrastructure_msgs::UInt8::Response& res){
  set_friction(req.resistance);
}

void setup() {
  Serial.begin(57600);
  //Setup ros pubs and services
  n.initNode();
  
  ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> reset_drawer_server("reset_drawer",&reset_drawer_callback);
  ros::ServiceServer<infrastructure_msgs::UInt8::Request, infrastructure_msgs::UInt8::Response> start_drawer_server("start_drawer",&start_drawer_callback);
  n.advertiseService(reset_drawer_server);
  n.advertiseService(start_drawer_server);
  
  n.advertise(datapub);
  
  //don't run drawer if TOF is not working.
  uint8_t connection_counter = 0;
  if (!tof.begin()) {
    while (1){
      if(connection_counter >= 3)
        n.logerror("Unable to initialize ToF sensor");
      
      connection_counter++;
      delay(5000);
    }
  }

  //Initialize space in drawer msg
  data.fsr_readings = (short unsigned int *)malloc(sizeof(uint8_t) * NUM_OF_FSRS);
  data.fsr_readings_length = NUM_OF_FSRS;
  
  //configure the max speed for the motors
  friction_motor.setMaxSpeed(20000);
  reset_motor.setMaxSpeed(20000);

  //setup the used arduino pins
  pinMode(pulse_reset, OUTPUT);
  pinMode(direction_reset, OUTPUT);
  pinMode(pulse_friction, OUTPUT);
  pinMode(direction_friction, OUTPUT);

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

  // initialize starting pos of drawer. Assumes drawer is closed.
  VL53L0X_RangingMeasurementData_t measure; //value from tof sensor
  tof.rangingTest(&measure, false);
  start_pos = measure.RangeMilliMeter; 
}

void loop() {
  collect_data();
  n.spinOnce();
  
  delay(20);
}

void reset_drawer() {
  reset_friction(); //turn off friction
  reset_motor.setSpeed(-5000); //set speed to negative value to change direction
  bool did_move = false;
  VL53L0X_RangingMeasurementData_t measure; //value from tof sensor
  while (true) {
    tof.rangingTest(&measure, false);
    if (measure.RangeMilliMeter < (start_pos + buffer_val)) {
      break;
    }
    did_move = true;
    time = millis();
    time_stop = time + 100; //runs motor for 100ms before checking if drawer is closed
    while (time < time_stop) {
      n.spinOnce();
      reset_motor.runSpeed();
      time = millis();
      delay(10);
    }
  }
  if (did_move) { //if drawer did not move at all (or did not need to be reeled in), don't unwind
    time = millis();
    time_stop = time + time_unwind;
    reset_motor.setSpeed(5000);
    while (time < time_stop) { //unwinds motor so string has slack
      n.spinOnce();
      reset_motor.runSpeed();
      delay(10);
    }
  }
}

void set_friction(float resistance) {
  float steps = ((resistance - base_friction) / fric_steps) + min_steps;
  friction_motor.setSpeed(5000);
  friction_motor.moveTo(steps);
  do {
    friction_motor.runSpeed();
    n.spinOnce();
    delay(10);
  } while (friction_motor.currentPosition() < friction_motor.targetPosition());
}

void reset_friction() {
  friction_motor.setSpeed(-5000);
  friction_motor.moveTo(0);
  do {
    friction_motor.runSpeed();
    n.spinOnce();
    delay(10);
  } while (friction_motor.currentPosition() > friction_motor.targetPosition());
}

void read_handle_val() {
  data.fsr_readings[0] = analogRead(fsr_1);
  data.fsr_readings[1] = analogRead(fsr_2);
  data.fsr_readings[2]= analogRead(fsr_3);
  data.fsr_readings[3] = analogRead(fsr_4);
  data.fsr_readings[4] = analogRead(fsr_5);
  data.fsr_readings[5] = analogRead(fsr_6);
  data.fsr_readings[6] = analogRead(fsr_7);
  data.fsr_readings[7] = analogRead(fsr_8);
  data.fsr_readings[8] = analogRead(fsr_9);
  data.fsr_readings[9] = analogRead(fsr_10);
  data.fsr_readings[10] = analogRead(fsr_11);
  data.fsr_readings[11] = analogRead(fsr_12);
}

void read_TOF_val() {
  VL53L0X_RangingMeasurementData_t measure;
  tof.rangingTest(&measure, false);
  int drawer_distance;
  if (measure.RangeMilliMeter <= start_pos) {
    drawer_distance = 0;
   }
  else {
   drawer_distance = measure.RangeMilliMeter - start_pos;
  }
  data.drawer_distance = drawer_distance;
  //data_point.data = drawer_distance;
  //datapub.publish(&data_point);
}

void collect_data() {
  read_TOF_val();
  read_handle_val();
  data.header.stamp = n.now(); //gets the time of the device roscore is running on
  datapub.publish(&data);
}
