
#include <Wire.h>
#include <SPI.h>
#include <Pixy2.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>


//=================================================================
// ===                    Pixy related                          ===
//=================================================================

//pixy related
Pixy2 pixy;
int count = 0;
int signature, x, y, width, height;
float cx, cy, area;

#define originalArea
#define originalR 50

//=============================================================================================================================================================




//==============================================================
// ===                     IMU and PID                       ===
//==============================================================


// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
   ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
   ========================================================================= */



// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT



#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

//=================================================================
// ===                         TIME                             ===
//=================================================================

unsigned long Time = 0;
unsigned long oldTime = 0;
unsigned long elapsedTime;

unsigned long previousMillis = 0;

//=================================================================
// ===                    PID constants                         ===
//=================================================================

const float rollkp = 2.5;
const float rollki = 0;
const float rollkd = 0.003;


const float pitchkp = 2.5;
const float pitchki = 0;
const float pitchkd = 0.003;



// ============================================================
// ===         CONSTANTS, STRUCT POINT, QUATERNION          ===
// ============================================================

#define SERVOMIN 1000
#define SERVOMAX 2000
#define FREQUENCY 50

#define L 148.7
#define W 78.4
#define H 90
#define Lc 10
#define Lf 50
#define Lt 50
#define s 30
#define R 100
#define sc 20

#define pi 3.1415926535


struct orientation {
  float roll;
  float pitch;
  float yaw;
};

struct quaternion {
  float r;
  float x;
  float y;
  float z;
};

struct point {
  float x;
  float y;
  float z;
};

struct point lfs0 = {L / 2, 0, -W / 2};
struct point lbs0 = { -L / 2, 0, -W / 2};
struct point rfs0 = {L / 2, 0, W / 2};
struct point rbs0 = { -L / 2, 0, W / 2};

struct point lff0 = {L / 2, -H, -W / 2 - Lc};
struct point lbf0 = { -L / 2, -H, -W / 2 - Lc};
struct point rff0 = {L / 2, -H, W / 2 + Lc};
struct point rbf0 = { -L / 2, -H, W / 2 + Lc};

struct point lff;
struct point lbf;
struct point rff;
struct point rbf;

struct point lfd;
struct point lbd;
struct point rfd;
struct point rbd;

struct point x_axis = {1, 0, 0};
struct point y_axis = {0, 1, 0};
struct point z_axis = {0, 0, 1};

struct point lfdsp;
struct point rfdsp;
struct point lbdsp;
struct point rbdsp;

//=============================================================================================================================================================

// ============================================================
// ===                     ORIENTATION                      ===
// ============================================================

float roll;
float pitch;
float yaw;

float oldRoll = 0;
float oldPitch = 0;
float oldYaw = 0;

float rollSlope = 0;
float pitchSlope = 0;
float yawSlope = 0;

float rollArea = 0;
float pitchArea = 0;
float yawArea = 0;

// =========================================================================================================================

// ============================================================
// ===                 VECTOR CALCULATIONS                  ===
// ============================================================


struct quaternion quat_multiply(struct quaternion q1, struct quaternion q2)
{
  float qr = q1.r * q2.r - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
  float qx = q1.r * q2.x + q2.r * q1.x + q1.y * q2.z - q2.y * q1.z;
  float qy = q1.r * q2.y + q2.r * q1.y + q1.z * q2.x - q2.z * q1.x;
  float qz = q1.r * q2.z + q2.r * q1.z + q1.x * q2.y - q2.x * q1.y;
  struct quaternion q = {qr, qx, qy, qz};
  return q;
}

struct point get_coordinates(struct quaternion q)
{
  struct point p = {q.x, q.y, q.z};
  return p;
}

struct point quat_rot(struct point a, struct point p, float theta)
{
  struct quaternion q = {cos(theta / 2), a.x * sin(theta / 2), a.y * sin(theta / 2), a.z * sin(theta / 2)};
  struct quaternion p1 = {0, p.x, p.y, p.z};
  struct quaternion qi = {cos(theta / 2), -a.x * sin(theta / 2), -a.y * sin(theta / 2), -a.z * sin(theta / 2)};
  struct quaternion qp = quat_multiply(q, p1);
  struct quaternion qpqi = quat_multiply(qp, qi);
  return get_coordinates(qpqi);
}

struct point vec_sum(struct point p1, struct point p2)
{
  float px = p1.x + p2.x;
  float py = p1.y + p2.y;
  float pz = p1.z + p2.z;
  return {px, py, pz};
}

struct point vec_sub(struct point p1, struct point p2)
{
  float px = p1.x - p2.x;
  float py = p1.y - p2.y;
  float pz = p1.z - p2.z;
  return {px, py, pz};
}

void stat_rot(struct point a, float theta)
{
  struct point lff = quat_rot(a, lff0, theta);
  struct point lbf = quat_rot(a, lbf0, theta);
  struct point rff = quat_rot(a, rff0, theta);
  struct point rbf = quat_rot(a, rbf0, theta);
  struct point lfd = vec_sub(lff, lfs0);
  struct point lbd = vec_sub(lbf, lbs0);
  struct point rfd = vec_sub(rff, rfs0);
  struct point rbd = vec_sub(rbf, rbs0);
  actuate(0, lfd.x, lfd.y, lfd.z);
  actuate(0, lbd.x, lbd.y, lbd.z);
  actuate(0, rfd.x, rfd.y, rfd.z);
  actuate(0, rbd.x, rbd.y, rbd.z);
}

//=============================================================================================================================================================


// ============================================================
// ===                    SETUP AND LOOP                    ===
// ============================================================

Adafruit_PWMServoDriver driver1 = Adafruit_PWMServoDriver(0x40);


void setup() {

  Serial.begin(115200);
  // ==================================================
  // ===               PCA9685 setup                ===
  // ==================================================

  driver1.begin();
  driver1.setPWMFreq(FREQUENCY);


  // ===========================================================
  // ===                Initializing actuators               ===
  // ===========================================================

  init_pos();
  //delay(500);

  // ==================================================
  // ===               Pixy setup                ===
  // ==================================================

  pixy.init();


  // ==================================================
  // ===           MPU 6050 INITIAL SETUP           ===
  // ==================================================

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
  // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));



  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

}

void loop() {

  //stepInPlace(200);
  //continuous_creep();
  balance();

  //trot(10);
  //trotWithMillis(10);

  //rightcurve(60);
  //leftcurve(60);
  //translateleft(10);
  //translateright(10);
  //spin(60);
  //roll(20);
  //pitch(30);
  //yaw(30);


  //final_pos();
  /*
    driver1.setPWM(0,0,307);
    driver1.setPWM(1,0,307);
    driver1.setPWM(2,0,307);
    driver1.setPWM(4,0,307);
    driver1.setPWM(5,0,307);
    driver1.setPWM(6,0,307);

    driver1.setPWM(8,0,307);
    driver1.setPWM(9,0,337);
    driver1.setPWM(10,0,307);
    driver1.setPWM(12,0,307);
    driver1.setPWM(13,0,307);
    driver1.setPWM(14,0,307);
  */
  /*

    for(int t=0;t<=50;t+=1)
    {
    actuate(0,0,-H+t,-Lc);
    actuate(1,0,-H+t,Lc);
    actuate(2,0,-H+t,-Lc);
    actuate(3,0,-H+t,Lc);
    delay(5);
    }
  */
  /*
    for(int t=0;t<=50;t++)
    {
    driver1.setPWM(0,0,307-t);
    driver1.setPWM(1,0,307+t);
    driver1.setPWM(2,0,307-t);

    driver1.setPWM(4,0,307+t);
    driver1.setPWM(5,0,307-t);
    driver1.setPWM(6,0,307+t);

    driver1.setPWM(8,0,307+t);
    driver1.setPWM(9,0,307+t);
    driver1.setPWM(10,0,307-t);

    driver1.setPWM(12,0,307-t);
    driver1.setPWM(13,0,307-t);
    driver1.setPWM(14,0,307+t);
    delay(50);
    }
  */
  /*
    for (int i = 1; i <= 4; i++)
    {
    for (int t = 0; t <= 40; t += 5)
    {
      actuate(0, 0, -H+t, 0);
      actuate(1, 0, -H+t, 0);
      actuate(2, 0, -H+t, 0);
      actuate(3, 0, -H+t, 0);
      delay(20);
    }

    for (int t = 40; t >= 0; t -= 5)
    {
      actuate(0, 0, -H+t, 0);
      actuate(1, 0, -H+t, 0);
      actuate(2, 0, -H+t, 0);
      actuate(3, 0, -H+t, 0);
      delay(20);
    }
    }

    for (int i = 1; i <= 4; i++)
    {
    for (int t = 0; t <= 40; t += 5)
    {
      actuate(0, 0, -H, t);
      actuate(1, 0, -H, t);
      actuate(2, 0, -H, t);
      actuate(3, 0, -H, t);
      delay(20);
    }

    for (int t = 40; t >= 0; t -= 5)
    {
      actuate(0, 0, -H, t);
      actuate(1, 0, -H, t);
      actuate(2, 0, -H, t);
      actuate(3, 0, -H, t);
      delay(20);
    }

    for (int i = 1; i <= 4; i++)
    {
    for (int t = 0; t <= 40; t += 5)
    {
      actuate(0, t, -H, 0);
      actuate(1, t, -H, 0);
      actuate(2, t, -H, 0);
      actuate(3, t, -H, 0);
      delay(20);
    }

    for (int t = 40; t >= 0; t -= 5)
    {
      actuate(0, t, -H, 0);
      actuate(1, t, -H, 0);
      actuate(2, t, -H, 0);
      actuate(3, t, -H, 0);
      delay(20);
    }
    }
    }*/









}



// ==========================================================
// ===              Inverse Kinematics                    ===
// ==========================================================

void actuate(int a, float x, float y, float z)
{
  if (a == 1)
  {
    rfd.x = x;
    rfd.y = y;
    rfd.z = z;
    rff = vec_sum(rfd, rfs0);

    z = -z;
    driver1.setPWM(4, 0, int(mapping(-atan(z / y), -pi / 2, pi / 2, SERVOMIN, SERVOMAX) / 1000000 * FREQUENCY * 4096));
    driver1.setPWM(6, 0, int(mapping(2 * asin(sqrt(x * x + (sqrt(x * x + y * y + z * z) - Lc) * (sqrt(x * x + y * y + z * z) - Lc)) / (2 * Lt)) - pi / 2, -pi / 2, pi / 2, SERVOMIN, SERVOMAX) / 1000000 * FREQUENCY * 4096));
    driver1.setPWM(5, 0, int(mapping(-atan(x / (sqrt(x * x + y * y + z * z) - Lc)) + acos(sqrt(x * x + (sqrt(x * x + y * y + z * z) - Lc) * (sqrt(x * x + y * y + z * z) - Lc)) / (2 * Lt)), -pi / 2, pi / 2, SERVOMAX, SERVOMIN) / 1000000 * FREQUENCY * 4096) - 50);
  }
  if (a == 0)
  {
    lfd.x = x;
    lfd.y = y;
    lfd.z = z;
    lff = vec_sum(lfd, lfs0);

    driver1.setPWM(0, 0, int(mapping(-atan(z / y), -pi / 2, pi / 2, SERVOMAX, SERVOMIN) / 1000000 * FREQUENCY * 4096));
    driver1.setPWM(2, 0, int(mapping(2 * asin(sqrt(x * x + (sqrt(x * x + y * y + z * z) - Lc) * (sqrt(x * x + y * y + z * z) - Lc)) / (2 * Lt)) - pi / 2, -pi / 2, pi / 2, SERVOMAX, SERVOMIN) / 1000000 * FREQUENCY * 4096));
    driver1.setPWM(1, 0, int(mapping(-atan(x / (sqrt(x * x + y * y + z * z) - Lc)) + acos(sqrt(x * x + (sqrt(x * x + y * y + z * z) - Lc) * (sqrt(x * x + y * y + z * z) - Lc)) / (2 * Lt)), -pi / 2, pi / 2, SERVOMIN, SERVOMAX) / 1000000 * FREQUENCY * 4096) + 50);

  }
  if (a == 3)
  {
    rbd.x = x;
    rbd.y = y;
    rbd.z = z;
    rbf = vec_sum(rbd, rbs0);

    z = -z;
    driver1.setPWM(12, 0, int(mapping(-atan(z / y), -pi / 2, pi / 2, SERVOMAX, SERVOMIN) / 1000000 * FREQUENCY * 4096) - 20);
    driver1.setPWM(14, 0, int(mapping(2 * asin(sqrt(x * x + (sqrt(x * x + y * y + z * z) - Lc) * (sqrt(x * x + y * y + z * z) - Lc)) / (2 * Lt)) - pi / 2, -pi / 2, pi / 2, SERVOMIN, SERVOMAX) / 1000000 * FREQUENCY * 4096));
    driver1.setPWM(13, 0, int(mapping(-atan(x / (sqrt(x * x + y * y + z * z) - Lc)) + acos(sqrt(x * x + (sqrt(x * x + y * y + z * z) - Lc) * (sqrt(x * x + y * y + z * z) - Lc)) / (2 * Lt)), -pi / 2, pi / 2, SERVOMAX, SERVOMIN) / 1000000 * FREQUENCY * 4096) - 70);
  }

  if (a == 2)
  {
    lbd.x = x;
    lbd.y = y;
    lbd.z = z;
    lbf = vec_sum(lbd, lbs0);

    driver1.setPWM(8, 0, int(mapping(-atan(z / y), -pi / 2, pi / 2, SERVOMIN, SERVOMAX) / 1000000 * FREQUENCY * 4096));
    driver1.setPWM(10, 0, int(mapping(2 * asin(sqrt(x * x + (sqrt(x * x + y * y + z * z) - Lc) * (sqrt(x * x + y * y + z * z) - Lc)) / (2 * Lt)) - pi / 2, -pi / 2, pi / 2, SERVOMAX, SERVOMIN) / 1000000 * FREQUENCY * 4096) - 20);
    driver1.setPWM(9, 0, int(mapping(-atan(x / (sqrt(x * x + y * y + z * z) - Lc)) + acos(sqrt(x * x + (sqrt(x * x + y * y + z * z) - Lc) * (sqrt(x * x + y * y + z * z) - Lc)) / (2 * Lt)), -pi / 2, pi / 2, SERVOMIN, SERVOMAX) / 1000000 * FREQUENCY * 4096) + 100);
  }
}



void actuatePID(int a, float x, float y, float z)
{
  getValues();
  roll = roll;
  pitch = pitch + 36;

  oldTime = Time;
  Time = millis();
  elapsedTime = Time - oldTime;

  oldRoll = roll;
  rollSlope = (roll - oldRoll) / elapsedTime;
  //rollArea=rollArea+elapsedTime*roll;

  oldPitch = pitch;
  pitchSlope = (pitch - oldPitch) * 100;
  //pitchArea=pitchArea+0.01*pitch;

  float z_act = rollkp * roll + rollkd * rollSlope;
  float x_act = pitchkp * pitch + pitchkd * pitchSlope;


  actuate(a, x - x_act, y, z + z_act);
}


//=================================================================
//===                   get Roll, Pitch, Yaw                    ===
//=================================================================

void getValues()
{
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    // other program behavior stuff here
    // .
    // .
    // .
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    // .
    // .
    // .
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;


#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    yaw = ypr[0] * 180 / M_PI;
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    pitch = ypr[1] * 180 / M_PI;
    Serial.print("\t");
    Serial.println(ypr[2] * 180 / M_PI);
    roll = ypr[2] * 180 / M_PI;
#endif

#ifdef OUTPUT_READABLE_REALACCEL
    // display real acceleration, adjusted to remove gravity
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    Serial.print("areal\t");
    Serial.print(aaReal.x);
    Serial.print("\t");
    Serial.print(aaReal.y);
    Serial.print("\t");
    Serial.println(aaReal.z);
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
    // display initial world-frame acceleration, adjusted to remove gravity
    // and rotated based on known orientation from quaternion
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    Serial.print("aworld\t");
    Serial.print(aaWorld.x);
    Serial.print("\t");
    Serial.print(aaWorld.y);
    Serial.print("\t");
    Serial.println(aaWorld.z);
#endif

#ifdef OUTPUT_TEAPOT
    // display quaternion values in InvenSense Teapot demo format:
    teapotPacket[2] = fifoBuffer[0];
    teapotPacket[3] = fifoBuffer[1];
    teapotPacket[4] = fifoBuffer[4];
    teapotPacket[5] = fifoBuffer[5];
    teapotPacket[6] = fifoBuffer[8];
    teapotPacket[7] = fifoBuffer[9];
    teapotPacket[8] = fifoBuffer[12];
    teapotPacket[9] = fifoBuffer[13];
    Serial.write(teapotPacket, 14);
    teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
#endif

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}


// ================================================================
// ===         Other Calculations & Curve Generator             ===
// ================================================================



float mapping(float val, float fromLow, float fromHigh, float toLow, float toHigh)
{
  float scale = (toHigh - toLow) / (fromHigh - fromLow);
  return toLow + (val - fromLow) * scale;
}


struct point generateBentLine(float x0, float y0, float z0, float x2, float y2, float z2, float height, float t)
{
  //0<=t<=1
  struct point p0 = {x0, y0, z0};
  struct point p2 = {x2, y2, z2};
  struct point p1;
  struct point curve;

  curve.x = x0 + (x2 - x0) * t;
  curve.y = -H + height - 2 * height * abs(t - 0.5);
  curve.z = z0 + (z2 - z0) * t;

  return curve;

}

struct point intersection(struct point p0, struct point p3, struct point p1, struct point p2)
{
  float x = p3.x * (p3.z - p0.z) / (p2.x - p1.x) / ((p3.z - p0.z) / (p3.x - p0.x) - (p2.z - p1.z) / (p2.x - p1.x));
  float z = (p3.z - p0.z) / (p3.x - p0.x) * x - p3.x * (p3.z - p0.z / p3.x - p0.x) + p3.z;
  float y = -H;
  struct point intersection;
  intersection.x = x;
  intersection.y = y;
  intersection.z = z;
  return intersection;
}

/*Bezier Curve Generator*/

struct point bezier(float x0, float y0, float z0, float x2, float y2, float z2, float height, float t)
{
  //0<=t<=1
  struct point p0 = {x0, y0, z0};
  struct point p3 = {x2, y2, z2};
  struct point p1 = {x0, y0 + height, z0};
  struct point p2 = {x2, y2 + height, z2};
  struct point q0;
  struct point q1;
  struct point q2;
  struct point r0;
  struct point r1;
  struct point curve;


  q0.x = p0.x * t + p1.x * (1 - t);
  q0.y = p0.y * t + p1.y * (1 - t);
  q0.z = p0.z * t + p1.z * (1 - t);

  q1.x = p1.x * t + p2.x * (1 - t);
  q1.y = p1.y * t + p2.y * (1 - t);
  q1.z = p1.z * t + p2.z * (1 - t);

  q2.x = p2.x * t + p3.x * (1 - t);
  q2.y = p2.y * t + p3.y * (1 - t);
  q2.z = p2.z * t + p3.z * (1 - t);

  r0.x = q0.x * t + q1.x * (1 - t);
  r0.y = q0.y * t + q1.y * (1 - t);
  r0.z = q0.z * t + q1.z * (1 - t);

  r1.x = q1.x * t + q2.x * (1 - t);
  r1.y = q1.y * t + q2.y * (1 - t);
  r1.z = q1.z * t + q2.z * (1 - t);

  curve.x = r0.x * t + r1.x * (1 - t);
  curve.y = r0.y * t + r1.y * (1 - t);
  curve.z = r0.z * t + r1.z * (1 - t);

  return curve;

}



// ===========================================================
// ===              Pixy object detection                  ===
// ===========================================================

float pixyCheck() {
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32];
  // grab blocks!
  blocks = pixy.ccc.getBlocks();

  // If there are detect blocks, print them!
  if (blocks)
  {
    signature = pixy.ccc.blocks[0].m_signature;
    height = pixy.ccc.blocks[0].m_height;
    width = pixy.ccc.blocks[0].m_width;
    x = pixy.ccc.blocks[0].m_x;
    y = pixy.ccc.blocks[0].m_y;
    cx = (x + (width / 2));
    cy = (y + (height / 2));
    cx = mapping(cx, 0, 320, -1, 1);
    cy = mapping(cy, 0, 200, 1, -1);
    area = width * height;

    //        Serial.print("sig: ");
    //        Serial.print(signature);
    //        Serial.print(" x:");
    //        Serial.print(x);
    //        Serial.print(" y:");
    //        Serial.print(y);
    //        Serial.print(" width: ");
    //        Serial.print(width);
    //        Serial.print(" height: ");
    //        Serial.print(height);
    //        Serial.print(" cx: ");
    //        Serial.print(cx);
    //        Serial.print(" cy: ");
    //        Serial.println(cy);

  }
  else {
    count += 1;
    if (count == 100) {
      count = 0;
      cx = 0;
    }
  }
  return cx;
}




// ===========================================================
// ===          Continuous movement functions              ===
// ===========================================================



void move03(float r, float thetax)
{
  lff = vec_sum(lfs0, lfd);
  rff = vec_sum(rfs0, rfd);
  lbf = vec_sum(lbs0, lbd);
  rbf = vec_sum(rbs0, rbd);
  //pixyCheck();
  //float r=sqrt(originalR*originalR*originalArea/area)
  //float thetax=find_direction();
  float curveRad = r / 2 / sin(pi / 4 - thetax / 2);
  float curveTheta = 2 * s / curveRad;
  if (thetax > 0)
  {
    lff0.x = lff0.x - curveRad;
    rbf0.x = rbf0.x - curveRad;
    struct point goallff = quat_rot(lff0, y_axis, -curveTheta);
    struct point goalrbf = quat_rot(rbf0, y_axis, -curveTheta);
    goallff.x = goallff.x + curveRad;
    goalrbf.x = goalrbf.x + curveRad;
    struct point goallfd = vec_sub(goallff, lfs0);
    struct point goalrbd = vec_sub(goalrbf, rbs0);
    for (int t = 0; t <= 30; t += 5)
    {
      actuateCurve(0, generateBentLine(lfd.x, lfd.y, lfd.z, goallfd.x, goallfd.y, goallfd.z, 50, t / 30));
      actuateCurve(3, generateBentLine(rbd.x, rbd.y, rbd.z, goalrbd.x, goalrbd.y, goalrbd.z, 50, t / 30));
      struct point center = {0, 0, -curveRad};
      calcRotatedFootVector(1, curveTheta * t / 30, center);
      calcRotatedFootVector(2, curveTheta * t / 30, center);
      actuateCurve(1, rfd);
      actuateCurve(2, lbd);
      delay(10);
    }
    lfd = goallfd;
    rbd = goalrbd;
  }
  if (thetax < 0)
  {
    lff0.x = lff0.x + curveRad;
    rbf0.x = rbf0.x + curveRad;
    struct point goallff = quat_rot(lff0, y_axis, curveTheta);
    struct point goalrbf = quat_rot(rbf0, y_axis, curveTheta);
    goallff.x = goallff.x - curveRad;
    goalrbf.x = goalrbf.x - curveRad;
    struct point goallfd = vec_sub(goallff, lfs0);
    struct point goalrbd = vec_sub(goalrbf, rbs0);
    for (int t = 0; t <= 30; t += 5)
    {
      actuateCurve(0, generateBentLine(lfd.x, lfd.y, lfd.z, goallfd.x, goallfd.y, goallfd.z, 50, t / 30));
      actuateCurve(3, generateBentLine(rbd.x, rbd.y, rbd.z, goalrbd.x, goalrbd.y, goalrbd.z, 50, t / 30));
      struct point center = {0, 0, curveRad};
      calcRotatedFootVector(1, -curveTheta * t / 30, center);
      calcRotatedFootVector(2, -curveTheta * t / 30, center);
      actuateCurve(1, rfd);
      actuateCurve(2, lbd);
      delay(10);
    }
    lfd = goallfd;
    rbd = goalrbd;
  }
  if (thetax == 0)
  {
    for (int t = 0; t <= 30; t += 5)
    {
      actuateCurve(0, generateBentLine(lfd.x, lfd.y, lfd.z, s, -H, 0, 50, t / 30));
      actuateCurve(3, generateBentLine(rbd.x, rbd.y, rbd.z, s, -H, 0, 50, t / 30));
      rfd.x = rfd.x - s * t / 30;
      lbd.x = lbd.x - s * t / 30;
      actuateCurve(1, rfd);
      actuateCurve(2, lbd);
      delay(10);
    }
    lfd = {s, -H, 0};
    rbd = {s, -H, 0};
  }
}

void move12(float r, float thetax)
{
  lff = vec_sum(lfs0, lfd);
  rff = vec_sum(rfs0, rfd);
  lbf = vec_sum(lbs0, lbd);
  rbf = vec_sum(rbs0, rbd);
  //pixyCheck();
  //float r=sqrt(originalR*originalR*originalArea/area)
  //float thetax=find_direction();
  float curveRad = r / 2 / sin(pi / 4 - thetax / 2);
  float curveTheta = 2 * s / curveRad;
  if (thetax > 0)
  {
    lbf0.x = lbf0.x - curveRad;
    rff0.x = rff0.x - curveRad;
    struct point goallbf = quat_rot(lbf0, y_axis, -curveTheta);
    struct point goalrff = quat_rot(rff0, y_axis, -curveTheta);
    goallbf.x = goallbf.x + curveRad;
    goalrff.x = goalrff.x + curveRad;
    struct point goallbd = vec_sub(goallbf, lbs0);
    struct point goalrfd = vec_sub(goalrff, rfs0);
    for (int t = 0; t <= 30; t += 5)
    {
      actuateCurve(1, generateBentLine(rfd.x, rfd.y, rfd.z, goalrfd.x, goalrfd.y, goalrfd.z, 50, t / 30));
      actuateCurve(2, generateBentLine(lbd.x, lbd.y, lbd.z, goallbd.x, goallbd.y, goallbd.z, 50, t / 30));
      struct point center = {0, 0, -curveRad};
      calcRotatedFootVector(0, curveTheta * t / 30, center);
      calcRotatedFootVector(3, curveTheta * t / 30, center);
      actuateCurve(0, lfd);
      actuateCurve(3, rbd);
      delay(10);
    }
    lbd = goallbd;
    rfd = goalrfd;
  }
  if (thetax < 0)
  {
    lbf0.x = lbf0.x + curveRad;
    rff0.x = rff0.x + curveRad;
    struct point goallbf = quat_rot(lbf0, y_axis, curveTheta);
    struct point goalrff = quat_rot(rff0, y_axis, curveTheta);
    goallbf.x = goallbf.x - curveRad;
    goalrff.x = goalrff.x - curveRad;
    struct point goallbd = vec_sub(goallbf, lbs0);
    struct point goalrfd = vec_sub(goalrff, rfs0);
    for (int t = 0; t <= 30; t += 5)
    {
      actuateCurve(1, generateBentLine(rfd.x, rfd.y, rfd.z, goalrfd.x, goalrfd.y, goalrfd.z, 50, t / 30));
      actuateCurve(2, generateBentLine(lbd.x, lbd.y, lbd.z, goallbd.x, goallbd.y, goallbd.z, 50, t / 30));
      struct point center = {0, 0, curveRad};
      calcRotatedFootVector(0, -curveTheta * t / 30, center);
      calcRotatedFootVector(3, -curveTheta * t / 30, center);
      actuateCurve(0, lfd);
      actuateCurve(3, rbd);
      delay(10);
    }
    lbd = goallbd;
    rfd = goalrfd;
  }
  if (thetax == 0)
  {
    for (int t = 0; t <= 30; t += 5)
    {
      actuateCurve(1, generateBentLine(rfd.x, rfd.y, rfd.z, s, -H, 0, 50, t / 30));
      actuateCurve(2, generateBentLine(lbd.x, lbd.y, lbd.z, s, -H, 0, 50, t / 30));
      lfd.x = lfd.x - s * t / 30;
      rbd.x = rbd.x - s * t / 30;
      actuateCurve(0, lfd);
      actuateCurve(3, rbd);
      delay(10);
    }
    lbd = {s, -H, 0};
    rfd = {s, -H, 0};
  }
}


void actuateCurve(int a, struct point p)
{
  actuate(a, p.x, p.y, p.z);
}




void continuous_trot()
{
  for (int temp = 60; temp >= 30; temp -= 5)
  {
    int t = 60 - temp;
    float theta = temp * pi / 180;
    lfd = { - s * t / 30, -H, -Lc};
    rfd = {s / 2 + s * cos(2 * theta), -H + 30 - 2 * fabs(t - 15), Lc};
    lbd = {s / 2 + s * cos(2 * theta), -H + 30 - 2 * fabs(t - 15), -Lc};
    rbd = { - s * t / 30, -H, Lc};
    actuate(1, s / 2 + s * cos(2 * theta), -H + 30 - 2 * fabs(t - 15), Lc);
    actuate(2, s / 2 + s * cos(2 * theta), -H + 30 - 2 * fabs(t - 15), -Lc);
    actuate(0,  - s * t / 30, -H, -Lc);
    actuate(3,  - s * t / 30, -H, Lc);
    delay(10);
  }
  move03(200, pi / 6);
  delay(10);
  move12(200, -pi / 6);
  delay(1000);
}

float find_direction()
{
  return atan(cx * tan(pi / 6));
}

struct point calcRotatedFootVector(int a, float theta, struct point center)
{
  struct point lfs1 = vec_sum(center, lfs0);
  struct point rfs1 = vec_sum(center, rfs0);
  struct point lbs1 = vec_sum(center, lbs0);
  struct point rbs1 = vec_sum(center, rbs0);
  struct point lff1 = vec_sum(center, lff0);
  struct point rff1 = vec_sum(center, rff0);
  struct point lbf1 = vec_sum(center, lbf0);
  struct point rbf1 = vec_sum(center, rbf0);
  struct point lffsp = quat_rot(y_axis, lff1, theta);
  struct point rffsp = quat_rot(y_axis, rff1, theta);
  struct point lbfsp = quat_rot(y_axis, lbf1, theta);
  struct point rbfsp = quat_rot(y_axis, rbf1, theta);
  switch (a) {
    case 0:
      lfdsp = vec_sub(lffsp, lfs1);
      return lfdsp;
      break;

    case 1:
      rfdsp = vec_sub(rffsp, rfs1);
      return rfdsp;
      break;

    case 2:
      lbdsp = vec_sub(lbfsp, lbs1);
      return lbdsp;
      break;

    case 3:
      rbdsp = vec_sub(rbfsp, rbs1);
      return rbdsp;
      break;
  }
}


struct point calcRotatedRandomVector(int a, struct point vector, float theta, struct point center)
{
  struct point lfs1 = vec_sum(center, lfs0);
  struct point lff1 = vec_sum(center, vec_sum(lfs0, vector));
  struct point lffsp = quat_rot(y_axis, lff1, theta);

  struct point rfs1 = vec_sum(center, rfs0);
  struct point rff1 = vec_sum(center, vec_sum(rfs0, vector));
  struct point rffsp = quat_rot(y_axis, rff1, theta);

  struct point lbs1 = vec_sum(center, lbs0);
  struct point lbf1 = vec_sum(center, vec_sum(lbs0, vector));
  struct point lbfsp = quat_rot(y_axis, lbf1, theta);

  struct point rbs1 = vec_sum(center, rbs0);
  struct point rbf1 = vec_sum(center, vec_sum(rbs0, vector));
  struct point rbfsp = quat_rot(y_axis, rbf1, theta);

  switch (a) {
    case 0:
      return vec_sub(lffsp, lfs1);
      break;

    case 1:
      return vec_sub(rffsp, rfs1);
      break;

    case 2:
      return vec_sub(lbfsp, lbs1);
      break;

    case 3:
      return vec_sub(rbfsp, rbs1);
      break;
  }
}



void continuous_creep()
{
  int t;

  t = 0;
  previousMillis = millis();
  while (t <= 10) {
    if (millis() - previousMillis >= 20) {
      previousMillis = millis();
      t += 5;
      actuate(0, 0, -H, -Lc + t);
      actuate(1, 0, -H, Lc + t);
      actuate(2, 0, -H, -Lc + t);
      actuate(3, 0, -H, Lc + t);
    }
  }

  t = 0;
  previousMillis = millis();
  while (t <= 40) {
    if (millis() - previousMillis >= 20) {
      previousMillis = millis();
      t += 5;
      actuate(1, -s * t / 40, -H + 20 - abs(t - 20), 10);
    }
  }


  t = 0;
  previousMillis = millis();
  while (t <= 40) {
    if (millis() - previousMillis >= 20) {
      previousMillis = millis();
      t += 5;
      actuate(3, s * t / 40, -H + 20 - abs(t - 20), 10);
    }
  }

  //pixyCheck();
  //float r=sqrt(originalR*originalR*originalArea/area);
  //float thetax = find_direction();
  float thetax = pi / 6;

  while (true)
  {
    //pixyCheck();
    //float r=sqrt(originalR*originalR*originalArea/area);
    //float thetax=find_direction();

    struct point center = {0, 0, 0};

    if (thetax > 0) //leftcurve
    {
      center.z = s / thetax;

      struct point lfdp = calcRotatedFootVector(0, thetax, center);
      struct point rfdp = calcRotatedFootVector(1, thetax, center);
      struct point lbdp = calcRotatedFootVector(2, thetax, center);
      struct point rbdp = calcRotatedFootVector(3, thetax, center);

      struct point oldlfd;
      struct point oldrfd;
      struct point oldlbd;
      struct point oldrbd;

      int t;

      oldrfd = rfd;
      t = 0;
      previousMillis = millis();
      while (t <= 40) {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t += 5;
          actuate(1, oldrfd.x + (rfdp.x - oldrfd.x)*t / 40, -H + 20 - abs(t - 20), oldrfd.z + (rfdp.z + 10 - oldrfd.z)*t / 40);
        }
      }

      t = 1;
      previousMillis = millis();
      while (t <= 8) {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t++;
          lfd = calcRotatedRandomVector(0, lfd, -thetax / 16, center);
          rfd = calcRotatedRandomVector(1, rfd, -thetax / 16, center);
          lbd = calcRotatedRandomVector(2, lbd, -thetax / 16, center);
          rbd = calcRotatedRandomVector(3, rbd, -thetax / 16, center);
          actuate(0, lfd.x, lfd.y, lfd.z);
          actuate(1, rfd.x, rfd.y, rfd.z);
          actuate(2, lbd.x, lbd.y, lbd.z);
          actuate(3, rbd.x, rbd.y, rbd.z);
        }
      }

      t = 1;
      previousMillis = millis();
      while (t <= 4)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t++;
          lfd.z = lfd.z - 5;
          rfd.z = rfd.z - 5;
          lbd.z = lbd.z - 5;
          rbd.z = rbd.z - 5;
          actuate(0, lfd.x, lfd.y, lfd.z);
          actuate(1, rfd.x, rfd.y, rfd.z);
          actuate(2, lbd.x, lbd.y, lbd.z);
          actuate(3, rbd.x, rbd.y, rbd.z);
        }
      }

      oldlbd = lbd;
      t = 0;
      previousMillis = millis();
      while (t <= 40)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t += 5;
          actuate(2, oldlbd.x + (lbdp.x - oldlbd.x)*t / 40, -H + 20 - abs(t - 20), oldlbd.z + (lbdp.z - 10 - oldlbd.z)*t / 40);
        }
      }

      oldlfd = lfd;
      t = 0;
      previousMillis = millis();
      while (t <= 40)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t += 5;
          actuate(0, oldlfd.x + (lfdp.x - oldlfd.x)*t / 40, -H + 20 - abs(t - 20), oldlfd.z + (lfdp.z - 10 - oldlfd.z)*t / 40);
        }
      }


      t = 1;
      previousMillis = millis();
      while (t <= 8)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t += 1;
          lfd = calcRotatedRandomVector(0, lfd, -thetax / 16, center);
          rfd = calcRotatedRandomVector(1, rfd, -thetax / 16, center);
          lbd = calcRotatedRandomVector(2, lbd, -thetax / 16, center);
          rbd = calcRotatedRandomVector(3, rbd, -thetax / 16, center);
          actuate(0, lfd.x, lfd.y, lfd.z);
          actuate(1, rfd.x, rfd.y, rfd.z);
          actuate(2, lbd.x, lbd.y, lbd.z);
          actuate(3, rbd.x, rbd.y, rbd.z);
        }
      }

      t = 1;
      previousMillis = millis();
      while (t <= 4)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t++;
          lfd.z = lfd.z + 5;
          rfd.z = rfd.z + 5;
          lbd.z = lbd.z + 5;
          rbd.z = rbd.z + 5;
          actuate(0, lfd.x, lfd.y, lfd.z);
          actuate(1, rfd.x, rfd.y, rfd.z);
          actuate(2, lbd.x, lbd.y, lbd.z);
          actuate(3, rbd.x, rbd.y, rbd.z);
        }
      }

      oldrbd = rbd;
      t = 0;
      previousMillis = millis();
      while (t <= 40)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t += 5;
          actuate(3, oldrbd.x + (rbdp.x - oldrbd.x)*t / 40, -H + 20 -  abs(t - 20), oldrbd.z + (rbdp.z + 10 - oldrbd.z)*t / 40);
        }
      }

    }

    if (thetax < 0) //rightcurve
    {
      center.z = s / thetax;

      struct point lfdp = calcRotatedFootVector(0, thetax, center);
      struct point rfdp = calcRotatedFootVector(1, thetax, center);
      struct point lbdp = calcRotatedFootVector(2, thetax, center);
      struct point rbdp = calcRotatedFootVector(3, thetax, center);

      struct point oldlfd;
      struct point oldrfd;
      struct point oldlbd;
      struct point oldrbd;


      oldrfd = rfd;
      t = 0;
      previousMillis = millis();
      while (t <= 40)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t += 5;
          actuate(1, oldrfd.x + (rfdp.x - oldrfd.x)*t / 40, -H + 20 -  abs(t - 20), oldrfd.z + (rfdp.z + 10 - oldrfd.z)*t / 40);
        }
      }

      t = 1;
      previousMillis = millis();
      while (t <= 8)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t++;
          lfd = calcRotatedRandomVector(0, lfd, -thetax / 16, center);
          rfd = calcRotatedRandomVector(1, rfd, -thetax / 16, center);
          lbd = calcRotatedRandomVector(2, lbd, -thetax / 16, center);
          rbd = calcRotatedRandomVector(3, rbd, -thetax / 16, center);
          actuate(0, lfd.x, lfd.y, lfd.z);
          actuate(1, rfd.x, rfd.y, rfd.z);
          actuate(2, lbd.x, lbd.y, lbd.z);
          actuate(3, rbd.x, rbd.y, rbd.z);
        }
      }

      t = 1;
      previousMillis = millis();
      while (t <= 4)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t++;
          lfd.z = lfd.z - 5;
          rfd.z = rfd.z - 5;
          lbd.z = lbd.z - 5;
          rbd.z = rbd.z - 5;
          actuate(0, lfd.x, lfd.y, lfd.z);
          actuate(1, rfd.x, rfd.y, rfd.z);
          actuate(2, lbd.x, lbd.y, lbd.z);
          actuate(3, rbd.x, rbd.y, rbd.z);
        }
      }

      oldlbd = lbd;
      t = 0;
      previousMillis = millis();
      while (t <= 40)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t += 5;
          actuate(2, oldlbd.x + (lbdp.x - oldlbd.x)*t / 40, -H + 20 - abs(t - 20), oldlbd.z + (lbdp.z - 10 - oldlbd.z)*t / 40);
        }
      }

      oldlfd = lfd;
      t = 0;
      previousMillis = millis();
      while (t <= 40)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t += 5;
          actuate(0, oldlfd.x + (lfdp.x - oldlfd.x)*t / 40, -H + 20 - abs(t - 20), oldlfd.z + (lfdp.z - 10 - oldlfd.z)*t / 40);
        }
      }

      t = 1;
      previousMillis = millis();
      while (t <= 8)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t++;
          lfd = calcRotatedRandomVector(0, lfd, -thetax / 16, center);
          rfd = calcRotatedRandomVector(1, rfd, -thetax / 16, center);
          lbd = calcRotatedRandomVector(2, lbd, -thetax / 16, center);
          rbd = calcRotatedRandomVector(3, rbd, -thetax / 16, center);
          actuate(0, lfd.x, lfd.y, lfd.z);
          actuate(1, rfd.x, rfd.y, rfd.z);
          actuate(2, lbd.x, lbd.y, lbd.z);
          actuate(3, rbd.x, rbd.y, rbd.z);
        }
      }

      t = 1;
      previousMillis = millis();
      while (t <= 4)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t++;
          lfd.z = lfd.z + 5;
          rfd.z = rfd.z + 5;
          lbd.z = lbd.z + 5;
          rbd.z = rbd.z + 5;
          actuate(0, lfd.x, lfd.y, lfd.z);
          actuate(1, rfd.x, rfd.y, rfd.z);
          actuate(2, lbd.x, lbd.y, lbd.z);
          actuate(3, rbd.x, rbd.y, rbd.z);
        }
      }

      oldrbd = rbd;
      t = 0;
      previousMillis = millis();
      while (t <= 40)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t += 5;
          actuate(3, oldrbd.x + (rbdp.x - oldrbd.x)*t / 40, -H + 20 - abs(t - 20), oldrbd.z + (rbdp.z + 10 - oldrbd.z)*t / 40);
        }
      }
    }



    if (thetax == 0)
    {
      t = 0;
      previousMillis = millis();
      while (t <= 40)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t += 5;
          actuate(1, -s + 2 * s * t / 40, -H + 20 -  abs(t - 20), 10);
        }
      }


      t = 0;
      previousMillis = millis();
      while (t <= 40)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t += 5;
          actuate(0, -s * t / 40, -H, 10);
          actuate(1, s - s * t / 40, -H, 10);
          actuate(2, -s * t / 40, -H, 10);
          actuate(3, s - s * t / 40, -H, 10);
        }
      }

      t = 0;
      previousMillis = millis();
      while (t <= 10)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t += 5;
          actuate(0, -s, -H, 10 - 2 * t);
          actuate(1, 0, -H, 10 - 2 * t);
          actuate(2, -s, -H, 10 - 2 * t);
          actuate(3, 0, -H, 10 - 2 * t);
        }
      }


      t = 0;
      previousMillis = millis();
      while (t <= 40)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t += 5;
          actuate(2, -s + 2 * s * t / 40, -H + 20 - abs(t - 20), - 10);
        }
      }

      t = 0;
      previousMillis = millis();
      while (t <= 40)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t += 5;
          actuate(0, -s + 2 * s * t / 40, -H + 20 - abs(t - 20), - 10);
        }
      }

      t = 0;
      previousMillis = millis();
      while (t <= 40)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t += 5;
          actuate(0, s - s * t / 40, -H, - 10);
          actuate(1, -s * t / 40, -H, - 10);
          actuate(2, s - s * t / 40, -H, - 10);
          actuate(3, -s * t / 40, -H, - 10);
        }
      }


      t = 0;
      previousMillis = millis();
      while (t <= 10)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t += 5;
          actuate(0, 0, -H,  - 10 + 2 * t);
          actuate(1, -s, -H, - 10 + 2 * t);
          actuate(2, 0, -H,  - 10 + 2 * t);
          actuate(3, -s, -H, - 10 + 2 * t);
        }
      }


      t = 0;
      previousMillis = millis();
      while (t <= 40)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t += 5;
          actuate(3, -s + 2 * s * t / 40, -H + 20 - abs(t - 20), 10);
        }
      }


    }
  }

}

void continuous_creep2()
{
  int t;

  t = 0;
  previousMillis = millis();
  while (t <= 10) {
    if (millis() - previousMillis >= 20) {
      previousMillis = millis();
      t += 5;
      actuate(0, 0, -H, t);
      actuate(1, 0, -H, t);
      actuate(2, 0, -H, t);
      actuate(3, 0, -H, t);
    }
  }

  t = 0;
  previousMillis = millis();
  while (t <= 40) {
    if (millis() - previousMillis >= 20) {
      previousMillis = millis();
      t += 5;
      actuateCurve(1, bezier(0, -H, 10, -s, -H, 10, 20, t / 40));
    }
  }


  t = 0;
  previousMillis = millis();
  while (t <= 40) {
    if (millis() - previousMillis >= 20) {
      previousMillis = millis();
      t += 5;
      actuateCurve(3, bezier(0, -H, 10, s, -H, 10, 20, t / 40));
    }
  }

  //pixyCheck();
  //float r=sqrt(originalR*originalR*originalArea/area);
  //float thetax = find_direction();
  float thetax = pi / 6;

  while (true)
  {
    //pixyCheck();
    //float r=sqrt(originalR*originalR*originalArea/area);
    //float thetax=find_direction();

    struct point center = {0, 0, 0};

    if (thetax > 0) //leftcurve
    {
      center.z = s / thetax;

      struct point lfdp = calcRotatedFootVector(0, thetax, center);
      struct point rfdp = calcRotatedFootVector(1, thetax, center);
      struct point lbdp = calcRotatedFootVector(2, thetax, center);
      struct point rbdp = calcRotatedFootVector(3, thetax, center);

      struct point oldlfd;
      struct point oldrfd;
      struct point oldlbd;
      struct point oldrbd;

      int t;

      oldrfd = rfd;
      t = 0;
      previousMillis = millis();
      while (t <= 40) {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t += 5;
          actuateCurve(1, bezier(oldrfd.x, -H, oldrfd.z, rfdp.x, -H, rfdp.z + 10, 20, t / 40));
        }
      }

      t = 1;
      previousMillis = millis();
      while (t <= 8) {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t++;
          lfd = calcRotatedRandomVector(0, lfd, -thetax / 16, center);
          rfd = calcRotatedRandomVector(1, rfd, -thetax / 16, center);
          lbd = calcRotatedRandomVector(2, lbd, -thetax / 16, center);
          rbd = calcRotatedRandomVector(3, rbd, -thetax / 16, center);
          actuate(0, lfd.x, lfd.y, lfd.z);
          actuate(1, rfd.x, rfd.y, rfd.z);
          actuate(2, lbd.x, lbd.y, lbd.z);
          actuate(3, rbd.x, rbd.y, rbd.z);
        }
      }

      t = 1;
      previousMillis = millis();
      while (t <= 4)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t++;
          lfd.z = lfd.z - 5;
          rfd.z = rfd.z - 5;
          lbd.z = lbd.z - 5;
          rbd.z = rbd.z - 5;
          actuate(0, lfd.x, lfd.y, lfd.z);
          actuate(1, rfd.x, rfd.y, rfd.z);
          actuate(2, lbd.x, lbd.y, lbd.z);
          actuate(3, rbd.x, rbd.y, rbd.z);
        }
      }

      oldlbd = lbd;
      t = 0;
      previousMillis = millis();
      while (t <= 40)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t += 5;
          actuateCurve(2, bezier(oldlbd.x, -H, oldlbd.z, lbdp.x, -H, lbdp.z - 10, 20, t / 40));
        }
      }

      oldlfd = lfd;
      t = 0;
      previousMillis = millis();
      while (t <= 40)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t += 5;
          actuateCurve(0, bezier(oldlfd.x, -H, oldlfd.z, lfdp.x, -H, lfdp.z - 10, 20, t / 40));
        }
      }


      t = 1;
      previousMillis = millis();
      while (t <= 8)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t += 1;
          lfd = calcRotatedRandomVector(0, lfd, -thetax / 16, center);
          rfd = calcRotatedRandomVector(1, rfd, -thetax / 16, center);
          lbd = calcRotatedRandomVector(2, lbd, -thetax / 16, center);
          rbd = calcRotatedRandomVector(3, rbd, -thetax / 16, center);
          actuate(0, lfd.x, lfd.y, lfd.z);
          actuate(1, rfd.x, rfd.y, rfd.z);
          actuate(2, lbd.x, lbd.y, lbd.z);
          actuate(3, rbd.x, rbd.y, rbd.z);
        }
      }

      t = 1;
      previousMillis = millis();
      while (t <= 4)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t++;
          lfd.z = lfd.z + 5;
          rfd.z = rfd.z + 5;
          lbd.z = lbd.z + 5;
          rbd.z = rbd.z + 5;
          actuate(0, lfd.x, lfd.y, lfd.z);
          actuate(1, rfd.x, rfd.y, rfd.z);
          actuate(2, lbd.x, lbd.y, lbd.z);
          actuate(3, rbd.x, rbd.y, rbd.z);
        }
      }

      oldrbd = rbd;
      t = 0;
      previousMillis = millis();
      while (t <= 40)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t += 5;
          actuateCurve(3, bezier(oldrbd.x, -H, oldrbd.z, rbdp.x, -H, rbdp.z + 10, 20, t / 40));
        }
      }

    }

    if (thetax < 0) //rightcurve
    {
      center.z = s / thetax;

      struct point lfdp = calcRotatedFootVector(0, thetax, center);
      struct point rfdp = calcRotatedFootVector(1, thetax, center);
      struct point lbdp = calcRotatedFootVector(2, thetax, center);
      struct point rbdp = calcRotatedFootVector(3, thetax, center);

      struct point oldlfd;
      struct point oldrfd;
      struct point oldlbd;
      struct point oldrbd;


      oldrfd = rfd;
      t = 0;
      previousMillis = millis();
      while (t <= 40)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t += 5;
          actuateCurve(1, bezier(oldrfd.x, -H, oldrfd.z, rfdp.x, -H, rfdp.z + 10, 20, t / 40));
        }
      }

      t = 1;
      previousMillis = millis();
      while (t <= 8)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t++;
          lfd = calcRotatedRandomVector(0, lfd, -thetax / 16, center);
          rfd = calcRotatedRandomVector(1, rfd, -thetax / 16, center);
          lbd = calcRotatedRandomVector(2, lbd, -thetax / 16, center);
          rbd = calcRotatedRandomVector(3, rbd, -thetax / 16, center);
          actuate(0, lfd.x, lfd.y, lfd.z);
          actuate(1, rfd.x, rfd.y, rfd.z);
          actuate(2, lbd.x, lbd.y, lbd.z);
          actuate(3, rbd.x, rbd.y, rbd.z);
        }
      }

      t = 1;
      previousMillis = millis();
      while (t <= 4)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t++;
          lfd.z = lfd.z - 5;
          rfd.z = rfd.z - 5;
          lbd.z = lbd.z - 5;
          rbd.z = rbd.z - 5;
          actuate(0, lfd.x, lfd.y, lfd.z);
          actuate(1, rfd.x, rfd.y, rfd.z);
          actuate(2, lbd.x, lbd.y, lbd.z);
          actuate(3, rbd.x, rbd.y, rbd.z);
        }
      }

      oldlbd = lbd;
      t = 0;
      previousMillis = millis();
      while (t <= 40)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t += 5;
          actuateCurve(2, bezier(oldlbd.x, -H, oldlbd.z, lbdp.x, -H, lbdp.z - 10, 20, t / 40));
        }
      }

      oldlfd = lfd;
      t = 0;
      previousMillis = millis();
      while (t <= 40)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t += 5;
          actuateCurve(0, bezier(oldlfd.x, -H, oldlfd.z, lfdp.x, -H, lfdp.z - 10, 20, t / 40));
        }
      }

      t = 1;
      previousMillis = millis();
      while (t <= 8)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t++;
          lfd = calcRotatedRandomVector(0, lfd, -thetax / 16, center);
          rfd = calcRotatedRandomVector(1, rfd, -thetax / 16, center);
          lbd = calcRotatedRandomVector(2, lbd, -thetax / 16, center);
          rbd = calcRotatedRandomVector(3, rbd, -thetax / 16, center);
          actuate(0, lfd.x, lfd.y, lfd.z);
          actuate(1, rfd.x, rfd.y, rfd.z);
          actuate(2, lbd.x, lbd.y, lbd.z);
          actuate(3, rbd.x, rbd.y, rbd.z);
        }
      }

      t = 1;
      previousMillis = millis();
      while (t <= 4)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t++;
          lfd.z = lfd.z + 5;
          rfd.z = rfd.z + 5;
          lbd.z = lbd.z + 5;
          rbd.z = rbd.z + 5;
          actuate(0, lfd.x, lfd.y, lfd.z);
          actuate(1, rfd.x, rfd.y, rfd.z);
          actuate(2, lbd.x, lbd.y, lbd.z);
          actuate(3, rbd.x, rbd.y, rbd.z);
        }
      }

      oldrbd = rbd;
      t = 0;
      previousMillis = millis();
      while (t <= 40)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t += 5;
          actuateCurve(3, bezier(oldrbd.x, -H, oldrbd.z, rbdp.x, -H, rbdp.z + 10, 20, t / 40));
        }
      }
    }



    if (thetax == 0)
    {
      t = 0;
      previousMillis = millis();
      while (t <= 40)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t += 5;
          actuateCurve(1, bezier(-s, -H, 10, s, -H, 10, 20, t / 40));
        }
      }


      t = 0;
      previousMillis = millis();
      while (t <= 40)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t += 5;
          actuate(0, -s * t / 40, -H, 10);
          actuate(1, s - s * t / 40, -H, 10);
          actuate(2, -s * t / 40, -H, 10);
          actuate(3, s - s * t / 40, -H, 10);
        }
      }

      t = 0;
      previousMillis = millis();
      while (t <= 10)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t += 5;
          actuate(0, -s, -H, 10 - 2 * t);
          actuate(1, 0, -H, 10 - 2 * t);
          actuate(2, -s, -H, 10 - 2 * t);
          actuate(3, 0, -H, 10 - 2 * t);
        }
      }


      t = 0;
      previousMillis = millis();
      while (t <= 40)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t += 5;
          actuateCurve(2, bezier(-s, -H, -10, s, -H, -10, 20, t / 40));
        }
      }

      t = 0;
      previousMillis = millis();
      while (t <= 40)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t += 5;
          actuateCurve(0, bezier(-s, -H, -10, s, -H, -10, 20, t / 40));
        }
      }

      t = 0;
      previousMillis = millis();
      while (t <= 40)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t += 5;
          actuate(0, s - s * t / 40, -H, - 10);
          actuate(1, -s * t / 40, -H, - 10);
          actuate(2, s - s * t / 40, -H, - 10);
          actuate(3, -s * t / 40, -H, - 10);
        }
      }


      t = 0;
      previousMillis = millis();
      while (t <= 10)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t += 5;
          actuate(0, 0, -H,  - 10 + 2 * t);
          actuate(1, -s, -H, - 10 + 2 * t);
          actuate(2, 0, -H,  - 10 + 2 * t);
          actuate(3, -s, -H, - 10 + 2 * t);
        }
      }


      t = 0;
      previousMillis = millis();
      while (t <= 40)
      {
        if (millis() - previousMillis >= 20) {
          previousMillis = millis();
          t += 5;
          actuateCurve(3, bezier(-s, -H, 10, s, -H, 10, 20, t / 40));
        }
      }


    }
  }

}



// ==========================================================
// ===                  Various Gaits                     ===
// ==========================================================

void init_pos()
{
  actuate(0, 0, -H, 0);
  actuate(1, 0, -H, 0);
  actuate(2, 0, -H, 0);
  actuate(3, 0, -H, 0);
}

void final_pos()
{
  for (int t = 0; t <= 80; t += 5)
  {
    actuate(0, 0, -H + t, -Lc);
    actuate(1, 0, -H + t,  Lc);
    actuate(2, 0, -H + t, - Lc);
    actuate(3, 0, -H + t,  Lc);
    delay(10);
  }
  delay(1000);
}

void stepInPlace(int steps)
{
  float k = steps / 2;
  for (int i = 1; i <= k; i++)
  {
    int t;

    t = 0;
    previousMillis = millis();
    while (t <= 60)
    {
      if (millis() - previousMillis >= 10) {
        previousMillis = millis();
        t += 5;
        actuate(0, 0, -H + 60 - 2 * abs(t - 30), -Lc);
        actuatePID(1, 0, -H, Lc);
        actuatePID(2, 0, -H, -Lc);
        actuate(3, 0, -H + 60 - 2 * abs(t - 30), Lc);
      }
    }

    while(millis()-previousMillis>=10){
      //delaying 10ms
    }

    t = 0;
    previousMillis = millis();
    while (t <= 60)
    {
      if (millis() - previousMillis >= 10) {
        previousMillis = millis();
        t += 5;
        actuate(2, 0, -H + 60 - 2 * abs(t - 30), -Lc);
        actuatePID(3, 0, -H, Lc);
        actuatePID(0, 0, -H, -Lc);
        actuate(1, 0, -H + 60 - 2 * abs(t - 30), Lc);
      }
    }
    
    while(millis()-previousMillis>=10){
      //delaying 10ms
    }
    
  }
}


void balance()
{
  while (true)
  {
    actuatePID(0, 0, -H, 0);
    actuatePID(1, 0, -H, 0);
    actuatePID(2, 0, -H, 0);
    actuatePID(3, 0, -H, 0);
  }
}



//creep

void start()
{
  for (int cnt = 0; cnt <= 30; cnt += 5)
  {
    actuate(0, 0, -H, -Lc);
    actuate(1, 0, -H, Lc);
    actuate(2, 0, -H, -Lc);
    actuate(3, 10 + sc * cnt / 30, -H + 30 - 2 * fabs(cnt - 15), Lc);
    delay(20);
  }
  for (int cnt = 0; cnt <= 30; cnt += 5)
  {
    actuate(1,  - sc * cnt / 30, -H + 30 - 2 * fabs(cnt - 15), Lc);
    delay(20);
  }
}

void cycle()
{
  for (int cnt = 0; cnt <= 60; cnt += 5)
  {
    actuate(1,  2 * sc * cnt / 60, -H + 60 - 2 * abs(cnt - 30), Lc);
    delay(20);
  }
  for (int cnt = 0; cnt <= 30; cnt += 5)
  {
    actuate(0,  - sc * cnt / 30, -H, -Lc);
    actuate(1, + sc  - sc * cnt / 30, -H, Lc);
    actuate(2,  - sc * cnt / 30, -H, -Lc);
    actuate(3, + sc  - sc * cnt / 30, -H, Lc);
    delay(20);
  }
  for (int cnt = 0; cnt <= 60; cnt += 5)
  {
    actuate(2, - sc + 2 * sc * cnt / 60, -H  + 60 - 2 * abs(cnt - 30), -Lc);
    delay(20);
  }
  for (int cnt = 0; cnt <= 60;  cnt += 5)
  {
    actuate(0, - sc + 2 * sc * cnt / 60, -H + 60 - 2 * abs(cnt - 30), -Lc);
    delay(20);
  }
  for (int cnt = 0; cnt <= 30; cnt += 5)
  {
    actuate(0, + sc  - sc * cnt / 30, -H, -Lc);
    actuate(1, - sc * cnt / 30, -H, Lc);
    actuate(2, + sc - sc * cnt / 30, -H, -Lc);
    actuate(3, - sc * cnt / 30, -H, Lc);
    delay(20);
  }
}

void cont()
{
  for (int cnt = 0; cnt <= 60; cnt += 5)
  {
    actuate(3, - sc + 2 * sc * cnt / 60, -H  + 60 - 2 * abs(cnt - 30), Lc);
    delay(20);
  }
}

void ending()
{
  for (int cnt = 0; cnt <= 30; cnt += 5)
  {
    actuate(3,  - sc + sc * cnt / 30, -H + 30 - 2 * abs(cnt - 15), Lc);
    delay(20);
  }
  for (int cnt = 0; cnt <= 30; cnt += 5)
  {
    actuate(1,  - sc + sc * cnt / 30, -H + 30 - 2 * abs(cnt - 15), Lc);
    delay(20);
  }
}

void creep(int steps)
{
  int k = (steps - 2) / 2;
  start();
  for (int i = 1; i <= k; i++)
  {
    cycle();
    cont();
  }
  cycle();
  ending();
}


void trotWithMillis(int steps)
{
  int k = (steps - 2) / 2;

  int temp = 60;
  previousMillis = millis();
  while (temp >= 30) {
    if (millis() - previousMillis >= 20) { //interval=10ms
      previousMillis = millis();
      temp -= 5;

      int t = 60 - temp;
      float theta = temp * pi / 180;
      actuate(1, s * t / 30, -H + 7.5 -  abs(t - 15) / 2, 0);
      actuate(2, s * t / 30, -H + 7.5 - abs(t - 15) / 2, 0);
      actuate(0, -s * t / 30, -H, 0);
      actuate(3, -s * t / 30, -H, 0);
    }
  }


  for (int i = 1; i <= 6; i++)
  {
    int temp = 120;
    previousMillis = millis();
    while (temp >= 60) {
      if (millis() - previousMillis >= 20) {
        previousMillis = millis();
        temp -= 5;

        int t = 120 - temp;
        float theta = temp * pi / 180;
        actuate(1, s - s * t / 30, -H, 0);
        actuate(2, s - s * t / 30, -H, 0);
        actuate(0, -s + s * t / 30, - H + 15 - abs(t - 30) / 2, 0);
        actuate(3, -s + s * t / 30, - H + 15 - abs(t - 30) / 2, 0);
      }
    }

    temp = 120;
    previousMillis = millis();
    while (temp >= 60) {
      if (millis() - previousMillis >= 20) {
        previousMillis = millis();
        temp -= 5;

        int t = 120 - temp;
        float theta = temp * pi / 180;
        actuate(0, s - s * t / 30, -H, 0);
        actuate(3, s - s * t / 30, -H, 0);
        actuate(1, -s + s * t / 30, - H + 15 - abs(t - 30) / 2, 0);
        actuate(2, -s + s * t / 30, - H + 15 - abs(t - 30) / 2, 0);
      }
    }
  }

  temp = 60;
  previousMillis = millis();
  while (temp >= 30) {
    if (millis() - previousMillis >= 20) {
      previousMillis = millis();
      temp -= 5;

      int t = 120 - temp;
      float theta = temp * pi / 180;
      actuate(1, s - s * t / 30, -H, 0);
      actuate(2, s - s * t / 30, -H, 0);
      actuate(0, -s + s * t / 30, -H + 7.5 - abs(t - 15) / 2, 0);
      actuate(3, -s + s * t / 30, -H + 7.5 - abs(t - 15) / 2, 0);
    }
  }
}


void trot(int steps)
{
  int k = (steps - 2) / 2;
  for (int temp = 60; temp >= 30; temp -= 5)
  {
    int t = 60 - temp;
    float theta = temp * pi / 180;
    actuate(1, s * t / 30, -H + 7.5 - abs(t - 15) / 2 , 0);
    actuate(2, s * t / 30, -H + 7.5 - abs(t - 15) / 2 , 0);
    actuate(0, -s * t / 30, -H, 0);
    actuate(3, -s * t / 30, -H, 0);
    delay(10);
  }
  for (int i = 1; i <= k; i++)
  {
    for (int temp = 120; temp >= 60; temp -= 5)
    {
      int t = 120 - temp;
      float theta = temp * pi / 180;
      actuate(1, s - s * t / 30, -H, 0);
      actuate(2, s - s * t / 30, -H, 0);
      actuate(0, -s + s * t / 30, - H + 15 - abs(t - 30) / 2 , 0);
      actuate(3, -s + s * t / 30, - H + 15 - abs(t - 30) / 2 , 0);
      delay(10);
    }

    for (int temp = 120; temp >= 60; temp -= 5)
    {
      int t = 120 - temp;
      float theta = temp * pi / 180;
      actuate(0, s - s * t / 30, -H, 0);
      actuate(3, s - s * t / 30, -H, 0);
      actuate(1, -s + s * t / 30, - H + 15 - abs(t - 30) / 2 , 0);
      actuate(2, -s + s * t / 30, - H + 15 - abs(t - 30) / 2 , 0);
      delay(10);
    }
  }


  for (int temp = 60; temp >= 30; temp -= 5)
  {
    int t = 60 - temp;
    float theta = temp * pi / 180;
    actuate(1, s - s * t / 30, -H, 0);
    actuate(2, s - s * t / 30, -H, 0);
    actuate(0, -s + s * t / 30, -H + 7.5 - abs(t - 15) / 2 , 0);
    actuate(3, -s + s * t / 30, -H + 7.5 - abs(t - 15) / 2 , 0);
    delay(10);
  }
}



void calcFootVector(int a, float theta, struct point center)
{
  struct point lfs1 = vec_sum(center, lfs0);
  struct point rfs1 = vec_sum(center, rfs0);
  struct point lbs1 = vec_sum(center, lbs0);
  struct point rbs1 = vec_sum(center, rbs0);
  struct point lff1 = vec_sum(center, lff0);
  struct point rff1 = vec_sum(center, rff0);
  struct point lbf1 = vec_sum(center, lbf0);
  struct point rbf1 = vec_sum(center, rbf0);
  struct point lffsp = quat_rot(y_axis, lff1, theta);
  struct point rffsp = quat_rot(y_axis, rff1, theta);
  struct point lbfsp = quat_rot(y_axis, lbf1, theta);
  struct point rbfsp = quat_rot(y_axis, rbf1, theta);
  switch (a) {
    case 0:
      lfdsp = vec_sub(lffsp, lfs1);
      break;

    case 1:
      rfdsp = vec_sub(rffsp, rfs1);
      break;

    case 2:
      lbdsp = vec_sub(lbfsp, lbs1);
      break;

    case 3:
      rbdsp = vec_sub(rbfsp, rbs1);
      break;
  }
}

void leftcurve(float angle)
{
  struct point center = {0, 0, 100};
  float theta = 0.2;
  int k = angle / theta / 2 - 1;


  for (int t = 0; t <= 60; t += 20)
  {
    float currAngle = theta * t / 60;
    calcFootVector(0, currAngle, center);
    calcFootVector(3, currAngle, center);
    calcFootVector(1, -currAngle, center);
    calcFootVector(2, -currAngle, center);
    actuate(0, lfdsp.x, lfdsp.y + 30 - fabs(t - 30), lfdsp.z);
    actuate(3, rbdsp.x, rbdsp.y + 30 - fabs(t - 30), rbdsp.z);
    actuate(1, rfdsp.x, rfdsp.y, rfdsp.z);
    actuate(2, lbdsp.x, lbdsp.y, lbdsp.z);
    delay(20);
  }
  for (int i = 0; i <= k; i++)
  {
    for (int t = 0; t <= 120; t += 20)
    {
      float currAngle = theta - theta * t / 60;
      calcFootVector(0, currAngle, center);
      calcFootVector(3, currAngle, center);
      calcFootVector(1, -currAngle, center);
      calcFootVector(2, -currAngle, center);
      actuate(0, lfdsp.x, lfdsp.y , lfdsp.z);
      actuate(3, rbdsp.x, rbdsp.y , rbdsp.z);
      actuate(1, rfdsp.x, rfdsp.y + 60 - fabs(t - 60), rfdsp.z);
      actuate(2, lbdsp.x, lbdsp.y + 60 - fabs(t - 60), lbdsp.z);
      delay(20);
    }
    for (int t = 0; t <= 120; t += 20)
    {
      float currAngle = -theta + theta * t / 60;
      calcFootVector(0, currAngle, center);
      calcFootVector(3, currAngle, center);
      calcFootVector(1, -currAngle, center);
      calcFootVector(2, -currAngle, center);
      actuate(0, lfdsp.x, lfdsp.y + 60 - fabs(t - 60) , lfdsp.z);
      actuate(3, rbdsp.x, rbdsp.y + 60 - fabs(t - 60), rbdsp.z);
      actuate(1, rfdsp.x, rfdsp.y , rfdsp.z);
      actuate(2, lbdsp.x, lbdsp.y , lbdsp.z);
      delay(20);
    }
  }
  for (int t = 60; t >= 0; t -= 20)
  {
    float currAngle = theta - theta * t / 60;
    calcFootVector(0, currAngle, center);
    calcFootVector(3, currAngle, center);
    calcFootVector(1, -currAngle, center);
    calcFootVector(2, -currAngle, center);
    actuate(0, lfdsp.x, lfdsp.y, lfdsp.z);
    actuate(3, rbdsp.x, rbdsp.y , rbdsp.z);
    actuate(1, rfdsp.x, rfdsp.y + 30 - abs(t  - 30), rfdsp.z);
    actuate(2, lbdsp.x, lbdsp.y + 30 - abs(t  - 30), lbdsp.z);
    delay(20);
  }
}

void rightcurve(float angle)
{
  struct point center = {0, 0, -100};
  float theta = 0.2;
  int k = angle / theta / 2 - 1;


  for (int t = 0; t <= 60; t += 20)
  {
    float currAngle = theta * t / 60;
    calcFootVector(0, -currAngle, center);
    calcFootVector(3, -currAngle, center);
    calcFootVector(1, currAngle, center);
    calcFootVector(2, currAngle, center);
    actuate(0, lfdsp.x, lfdsp.y + 30 - abs(t - 30), lfdsp.z);
    actuate(3, rbdsp.x, rbdsp.y + 30 - abs(t  - 30), rbdsp.z);
    actuate(1, rfdsp.x, rfdsp.y, rfdsp.z);
    actuate(2, lbdsp.x, lbdsp.y, lbdsp.z);
    delay(20);
  }
  for (int i = 0; i <= k; i++)
  {
    for (int t = 0; t <= 120; t += 20)
    {
      float currAngle = theta - theta * t / 60;
      calcFootVector(0, -currAngle, center);
      calcFootVector(3, -currAngle, center);
      calcFootVector(1, currAngle, center);
      calcFootVector(2, currAngle, center);
      actuate(0, lfdsp.x, lfdsp.y , lfdsp.z);
      actuate(3, rbdsp.x, rbdsp.y , rbdsp.z);
      actuate(1, rfdsp.x, rfdsp.y + 60 - abs(t - 60), rfdsp.z);
      actuate(2, lbdsp.x, lbdsp.y + 60 - abs(t - 60), lbdsp.z);
      delay(20);
    }
    for (int t = 0; t <= 120; t += 20)
    {
      float currAngle = -theta + theta * t / 60;
      calcFootVector(0, -currAngle, center);
      calcFootVector(3, -currAngle, center);
      calcFootVector(1, currAngle, center);
      calcFootVector(2, currAngle, center);
      actuate(0, lfdsp.x, lfdsp.y + 60 - abs(t - 60) , lfdsp.z);
      actuate(3, rbdsp.x, rbdsp.y + 60 - abs(t - 60), rbdsp.z);
      actuate(1, rfdsp.x, rfdsp.y , rfdsp.z);
      actuate(2, lbdsp.x, lbdsp.y , lbdsp.z);
      delay(20);
    }
  }
  for (int t = 60; t >= 0; t -= 20)
  {
    float currAngle = theta - theta * t / 60;
    calcFootVector(0, -currAngle, center);
    calcFootVector(3, -currAngle, center);
    calcFootVector(1, currAngle, center);
    calcFootVector(2, currAngle, center);
    actuate(0, lfdsp.x, lfdsp.y, lfdsp.z);
    actuate(3, rbdsp.x, rbdsp.y , rbdsp.z);
    actuate(1, rfdsp.x, rfdsp.y + 30 - abs(t - 30), rfdsp.z);
    actuate(2, lbdsp.x, lbdsp.y + 30 - abs(t - 30), lbdsp.z);
    delay(20);
  }
}


// ==========================================================
// ===               Change Orientation                   ===
// ==========================================================

void changeRoll(float angle)
{
  angle = angle * pi / 180;
  for (int t = 0; t <= 60; t++)
  {
    stat_rot(x_axis, t * angle / 60);
    delay(10);
  }
  for (int t = 60; t >= 0; t--)
  {
    stat_rot(x_axis, t * angle / 60);
    delay(10);
  }
}


void changePitch(float angle)
{
  angle = angle * pi / 180;
  for (int t = 0; t <= 60; t++)
  {
    stat_rot(z_axis, t * angle / 60);
    delay(10);
  }
  for (int t = 60; t >= 0; t--)
  {
    stat_rot(z_axis, t * angle / 60);
    delay(10);
  }
}

void changeYaw(float angle)
{
  angle = angle * pi / 180;
  for (int t = 0; t <= 60; t++)
  {
    stat_rot(y_axis, t * angle / 60);
    delay(10);
  }
  for (int t = 60; t >= 0; t--)
  {
    stat_rot(y_axis, t * angle / 60);
    delay(10);
  }
}



void bounce(int h)
{
  for (int t = 0; t <= h; t++)
  {
    actuate(0, 0, -H + t, -Lc);
    actuate(1, 0, -H + t, Lc);
    actuate(2, 0, -H + t, -Lc);
    actuate(3, 0, -H + t, Lc);
    delay(1);
  }
  for (int t = h - 1; t >= 0; t--)
  {
    actuate(0, 0, -H + t, -Lc);
    actuate(1, 0, -H + t, Lc);
    actuate(2, 0, -H + t, -Lc);
    actuate(3, 0, -H + t, Lc);
    delay(1);
  }
}
