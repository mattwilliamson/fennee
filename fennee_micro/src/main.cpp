#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// This is a hack to prevent trying to connect to a WiFi network by default
// https://github.com/frankjoshua/rosserial_arduino_lib/blob/master/src/ros.h#L40
// https://github.com/espressif/arduino-esp32/issues/4807
#undef ESP32
#include <ros.h>
#define ESP32

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
  // We get a list of positions to achieve
  // Set the servo position for each joint in each position
  // TODO: Publish the new position to joint_states
  for (size_t i = 0; i < input.data_length; i++)
  {
    pca9685.setPWM(servoChannels[i], 0, input.data[i]);
  }

}

ros::Subscriber<std_msgs::UInt16MultiArray> servoDutyCycles("servo_duty_cycles", servo_cb);

void setup()
{
  nh.initNode();
  Serial.begin(115200);

  Wire.setClock(400000);
  pca9685.begin();
    /*
    * In theory the internal oscillator (clock) is 25MHz but it really isn't
    * that precise. You can 'calibrate' this by tweaking this number until
    * you get the PWM update frequency you're expecting!
    * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
    * is used for calculating things like writeMicroseconds()
    * Analog servos run at ~50 Hz updates, It is importaint to use an
    * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
    * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
    *    the I2C PCA9685 chip you are setting the value for.
    * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
    *    expected value (50Hz for most ESCs)
    * Setting the value here is specific to each individual I2C PCA9685 chip and
    * affects the calculations for the PWM update frequency.
    * Failure to correctly set the int.osc value will cause unexpected PWM results
    */
  // pca9685.setOscillatorFrequency(27000000);
  pca9685.setOscillatorFrequency(26000000);
  pca9685.setPWMFreq(SERVO_FREQ); // Analog servos run at ~50 Hz updates

  //nh.advertise(jointStates);
  nh.advertise(chatter);
  nh.subscribe(servoDutyCycles);
  // nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  str_msg.data = "after spin";
  chatter.publish(&str_msg);
}