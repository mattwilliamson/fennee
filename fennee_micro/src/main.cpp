// Create a wifi.h file with the following contents:
// #define WIFI_SSID "ssid"
// #define WIFI_PASS "passwod"
#include "wifi.h"

#include <WiFi.h>
// Set the rosserial socket server IP address
IPAddress server(192,168,50,42);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

#define SERIAL_BAUD 500000
#define I2C_SPEED 400000
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define PCA9685_SPEED 26000000 // 25MHz calibration
#define SERVO_DUTY_TOPIC "servo_duty"
#define JOINT_STATES_TOPIC "joint_states"


// This is a hack to prevent trying to connect to a WiFi network by default
// https://github.com/frankjoshua/rosserial_arduino_lib/blob/master/src/ros.h#L40
// https://github.com/espressif/arduino-esp32/issues/4807
// #undef ESP32
// #include <ros.h>
// #define ESP32

#include <ros.h>

// TODO: Publish to joint_states per https://github.com/chvmp/champ/wiki/Hardware-Integration

#include <std_msgs/UInt16MultiArray.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver();


ros::NodeHandle nh;

#include <std_msgs/String.h>
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);


int servoChannels[12] = {
    2,  // front_left_shoulder
    1,  // front_left_leg
    0,  // front_left_foot
    5,  // front_right_shoulder
    4,  // front_right_leg
    3,  // front_right_foot
    8,  // rear_left_shoulder
    7,  // rear_left_leg
    6,  // rear_left_foot
    11, // rear_right_shoulder
    10, // rear_right_leg
    9   // rear_right_foot
};


void servo_cb(const std_msgs::UInt16MultiArray &input)
{
  // We get a list of PWM signals for each joint
  // It might be cleaner to do the conversion here instead of server-side
  // but this adds a lot more flexibility without having to flash the micro
  // TODO: Publish the new position to joint_states
  for (size_t i = 0; i < input.data_length; i++)
  {
    pca9685.setPWM(servoChannels[i], 0, input.data[i]);
  }
}

ros::Subscriber<std_msgs::UInt16MultiArray> servoDutyCycles(SERVO_DUTY_TOPIC, servo_cb);

void setup()
{
  Serial.begin(SERIAL_BAUD);

  Serial.println("\nConnecting to WiFi SSID " WIFI_SSID "");

  // WiFi.mode(WIFI_STA); //Optional
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.println("\nwaiting for wifi...");

  while(WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(100);
  }

  Serial.println("\nConnected.");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());

  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);

  Serial.println("\nConnecting to rosserial TCP socket at " + server.toString() + ":" + serverPort);

  nh.initNode();

  //nh.advertise(jointStates);
  nh.advertise(chatter);
  nh.subscribe(servoDutyCycles);
  // nh.subscribe(sub);

  // TODO: Keep getting error: [Wire.cpp:381] setClock(): could not acquire lock
  Wire.setClock(I2C_SPEED);
  pca9685.begin();
  pca9685.setOscillatorFrequency(PCA9685_SPEED);
  pca9685.setPWMFreq(SERVO_FREQ);

  nh.loginfo("Fennee robot started");
}

void loop()
{
  // Wait for a connection
  while (!nh.connected())
  {
    Serial.println("Waiting for rosserial TCP socket connection...");
    nh.spinOnce();
    sleep(100);
  }

  nh.spinOnce();
  // str_msg.data = "after spin";
  // chatter.publish(&str_msg);

  // delay(1000);
  // nh.loginfo("looping");
}