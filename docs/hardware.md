# Hardware

## Parts List


- [SpotMicro](https://spotmicroai.readthedocs.io/en/latest/) robot frame
- [NVIDIA Jetson Nano](https://www.amazon.com/NVIDIA-Jetson-Nano-Developer-945-13450-0000-100/dp/B084DSDDLT/ref=sr_1_3?crid=K3EEP2QYGRKR&keywords=NVIDIA+Jetson+Nano&qid=1681300762&s=electronics&sprefix=nvidia+jetson+nano%252Celectronics%252C161&sr=1-3&ufe=app_do%253Aamzn1.fos.f5122f16-c3e8-4386-bf32-63e904010ad0&_encoding=UTF8&tag=aimatt02-20&linkCode=ur2&linkId=7d3968aff925cd432b2f563af53e1284&camp=1789&creative=9325) for high-level control
- [SD Card](https://www.amazon.com/dp/B07FCMBLV6?psc=1&ref=ppx_yo2ov_dt_b_product_details&_encoding=UTF8&tag=aimatt02-20&linkCode=ur2&linkId=d9ee3513e15091796e6be4dad591e27c&camp=1789&creative=9325)
- LDS-01 LIDAR
- USB TTL converter for LIDAR
- PCA9685 PWM controller for controlling servos: https://www.adafruit.com/product/815
- 12x [DS3218 Digital Servo Motor](https://www.amazon.com/gp/product/B07WYQ9P3F/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1&_encoding=UTF8&tag=aimatt02-20&linkCode=ur2&linkId=aac388a45ead7e4239ba14d217c2a3e6&camp=1789&creative=9325)
- [LiPO Battery](https://www.amazon.com/dp/B086D71TZC?psc=1&ref=ppx_yo2ov_dt_b_product_details&_encoding=UTF8&tag=aimatt02-20&linkCode=ur2&linkId=2e59462443f181c82c3a7a22fd7b1b8c&camp=1789&creative=9325) this one is 5200mAh 7.4v, but you have flexibility here
- [Buck Converter](https://www.amazon.com/dp/B07Y7YB14L?psc=1&ref=ppx_yo2ov_dt_b_product_details&_encoding=UTF8&tag=aimatt02-20&linkCode=ur2&linkId=9f2c21c8d2dcc244b0d85624aa2fc704&camp=1789&creative=9325)
- 2x [Bearings](https://www.amazon.com/dp/B07JHKKGKT?psc=1&ref=ppx_yo2ov_dt_b_product_details&_encoding=UTF8&tag=aimatt02-20&linkCode=ur2&linkId=92a49fcb0e2082979097c1ca9db1d71c&camp=1789&creative=9325)
- [Lots](https://www.amazon.com/dp/B015A30R1I?psc=1&ref=ppx_yo2ov_dt_b_product_details&_encoding=UTF8&tag=aimatt02-20&linkCode=ur2&linkId=4594994ca7d27816592dd19072e0148a&camp=1789&creative=9325) of [m3 screws and nuts (some m2 as well)](https://www.amazon.com/dp/B08JCKH31Q?psc=1&ref=ppx_yo2ov_dt_b_product_details&_encoding=UTF8&tag=aimatt02-20&linkCode=ur2&linkId=85b1b003df00aa2316b49650ea373ecb&camp=1789&creative=9325)
- [PETG Filament](https://www.amazon.com/dp/B08JCKH31Q?psc=1&ref=ppx_yo2ov_dt_b_product_details&_encoding=UTF8&tag=aimatt02-20&linkCode=ur2&linkId=85b1b003df00aa2316b49650ea373ecb&camp=1789&creative=9325)

*Disclaimer: I did add Amazon affiliate links to these*

### Optional

- [Waveshare AC8265](https://www.amazon.com/dp/B07SGDRG34?psc=1&ref=ppx_yo2ov_dt_b_product_details&_encoding=UTF8&tag=aimatt02-20&linkCode=ur2&linkId=5afefd950c9ea730d13b46f8e915f168&camp=1789&creative=9325) WiFi and Bluetooth for Jetson Nano
- [IPEX Wifi Antennas](https://www.amazon.com/Antenna-2-4GHz-Internal-Laptop-Wireless/dp/B08XN6WMXJ/ref=sr_1_3?crid=141NMPSVVCSRX&keywords=IPEX%252Bantenna%252B5ghz&qid=1681301616&s=electronics&sprefix=ipex%252Bantenna%252B5ghz%252Celectronics%252C103&sr=1-3&th=1&_encoding=UTF8&tag=aimatt02-20&linkCode=ur2&linkId=b760f9104679d63096c3b43a57f38a0c&camp=1789&creative=9325)
- [Volt Display](https://www.amazon.com/dp/B0761MG9NS?psc=1&ref=ppx_yo2ov_dt_b_product_details&_encoding=UTF8&tag=aimatt02-20&linkCode=ur2&linkId=fa09f95b7c494d9456b38b2fabb7f38a&camp=1789&creative=9325)
- [Creality Ender 3 S1 Pro 3d printer](https://www.amazon.com/3D-High-Temperature-Removable-Touchscreen-Languages/dp/B09TKCY9HY/ref=sr_1_3?camp=1789&creative=9325&keywords=Creality+Ender+3+S1+Pro&linkCode=ur2&linkId=6745c52615b49225b7af7e0865687db1&qid=1681302453&sr=8-3&ufe=app_do%253Aamzn1.fos.c3015c4a-46bb-44b9-81a4-dc28e6d374b3&_encoding=UTF8&tag=aimatt02-20&linkCode=ur2&linkId=8694ab1cb75ba1d75b8df1cbfc0ca5be&camp=1789&creative=9325)

## 3d Printed

- [SpotMicro prints without supports](https://www.thingiverse.com/thing:4559827)
- [Reinforced shoulders](https://www.thingiverse.com/thing:4937631)
- [Assembly Guide](https://github.com/mattwilliamson/SpotMicroESP32/tree/master/assembly) *Needs some updates*


---

## TODO Documentation

- [ ] Lidar specs - LDS-01 - https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_01/ - https://medium.com/@Matthew_Hogan/interesting-electronic-components-1-hls-lfcd2-640d897f9fc9
- [ ] Oak D Lite - needs right angle cable



---

PCA9685 Connection
 - Connect VCC Pin of PCA9685 to 3.3 Volt Pin of Jetson Nano (Pin 1 upper outside)
 - Connect GND Pin of PCA9685 to Jetson Nano GND (Row 3, inside pin)
 - Connect SDA Pin of PCA9685 to Jetson Nano Pin 3 (Row 2, outside pin)
 - Connect SCL Pin of PCA9685 to Jetson Nano Pin 5 (Row 3, outside pin)

```
jetson@fennee:~/ros_ws$ sudo i2cdetect -y -r 1
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: 40 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: 70 -- -- -- -- -- -- --             
```


---

BNO085 IMU Connection

If you solder a header on the opposite side of the PCA9685, you can just daisy-chain onto that same I2C bus.

```
jetson@fennee:~/ros_ws/src/fennee$ i2cdetect -r -y 1
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: 40 -- -- -- -- -- -- -- -- -- -- 4b -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: 70 -- -- -- -- -- -- --       
```

For mine, the address was 0x4b.

Testing:

```
$ sudo pip3 install adafruit-circuitpython-bno08x

$ python3
import board
import busio
import time
import adafruit_bno08x
from adafruit_bno08x.i2c import BNO08X_I2C
i2c = busio.I2C(board.SCL, board.SDA)
i2c.scan()
#bno = BNO08X_I2C(i2c)
bno = BNO08X_I2C(i2c, address=0x4b)
bno.enable_feature(BNO_REPORT_ACCELEROMETER)
bno.enable_feature(BNO_REPORT_GYROSCOPE)
bno.enable_feature(BNO_REPORT_MAGNETOMETER)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
while True:
     time.sleep(0.5)
     print("Acceleration:")
     accel_x, accel_y, accel_z = bno.acceleration  # pylint:disable=no-member
     print("X: %0.6f  Y: %0.6f Z: %0.6f  m/s^2" % (accel_x, accel_y, accel_z))
     print("")
     #
     print("Gyro:")
     gyro_x, gyro_y, gyro_z = bno.gyro  # pylint:disable=no-member
     print("X: %0.6f  Y: %0.6f Z: %0.6f rads/s" % (gyro_x, gyro_y, gyro_z))
     print("")
     #
     print("Magnetometer:")
     mag_x, mag_y, mag_z = bno.magnetic  # pylint:disable=no-member
     print("X: %0.6f  Y: %0.6f Z: %0.6f uT" % (mag_x, mag_y, mag_z))
     print("")
     #
     print("Rotation Vector Quaternion:")
     quat_i, quat_j, quat_k, quat_real = bno.quaternion  # pylint:disable=no-member
     print(
      "I: %0.6f  J: %0.6f K: %0.6f  Real: %0.6f" % (quat_i, quat_j, quat_k, quat_real)
     )
     print("")
```