

#include <math.h>
#include <Wire.h>
#include <HCPCA9685.h>

// ===========================================================
// ===                      CONSTANTS                      ===
// ===========================================================

#define servomin 0
#define servomax 480

#define L 86
#define s 10
#define W 40
#define H 65
#define Lc 20
#define Lf 35.9
#define Lt 42.6
#define R 70
#define sc 15

#define pi 3.1415926535

// ===========================================================
// ==                   MPU 6050 related                    ==
// ===========================================================


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
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */



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
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

//=================================================================
// ===                    PID constants                         ===
//=================================================================

const float rollkp = 0.1;
const float rollki = 0.1;
const float rollkd = 0.1;


const float pitchkp = 0.1;
const float pitchki = 0.1;
const float pitchkd = 0.1;


// ============================================================
// ===                 STRUCT POINT, QUATERNION             ===
// ============================================================


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

// ============================================================
// ===                     ORIENTATION                      ===
// ============================================================

float roll;
float pitch;
float yaw;

float oldRoll=0;
float oldPitch=0;
float oldYaw=0;

float rollSlope=0;
float pitchSlope=0;
float yawSlope=0;

float rollArea=0;
float pitchArea=0;
float yawArea=0;

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

// ============================================================
// ===                    SETUP AND LOOP                    ===
// ============================================================


HCPCA9685 HCPCA9685(0x40);


void setup() {
  Serial.begin(115200);
  // ==================================================
  // ===               PCA9685 setup                ===
  // ==================================================
  HCPCA9685.Init(SERVO_MODE);
  HCPCA9685.Sleep(false);

  // ==================================================
  // ===           MPU 6050 INITIAL SETUP           ===
  // ==================================================
/*
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
  */
  // ================================================================
  // ===                    Initialize position                   ===
  // ================================================================
  
  init_pos();
  delay(1000);
}

void loop() {
  continuous_creep();
}


void init_pos()
{
  actuate(0, 0, -H, - Lc);
  actuate(1, 0, -H,  Lc);
  actuate(2, 0, -H, - Lc);
  actuate(3, 0, -H,  Lc);
  delay(500);
}

void stepInPlace(int steps)
{
  float k=steps/2;
  for(int i=1;i<=k;i++)
  {
    for(int t=0;t<=60;t+=5)
    {
      actuate(0,0,-H+10-abs(t/3-10),-Lc);
      actuatePID(1,0,-H,Lc);
      actuatePID(2,0,-H,-Lc);
      actuate(3,0,-H+10-abs(t/3-10),Lc);
      delay(10);
    }
    delay(10);
    for(int t=0;t<=60;t+=5)
    {
      actuate(2,0,-H+10-abs(t/3-10),-Lc);
      actuatePID(3,0,-H,Lc);
      actuatePID(0,0,-H,-Lc);
      actuate(1,0,-H+10-abs(t/3-10),Lc);
      delay(10);
    }
    delay(10);
  }
}

void balance()
{
  while (true)
  {
    actuatePID(0, 0, -H,- Lc);
    actuatePID(1, 0, -H, Lc);
    actuatePID(2, 0, -H,- Lc);
    actuatePID(3, 0, -H, Lc);
    delay(5);
  }

}

// ========================================================================
// ===                       Continuous creep                           ===
// ========================================================================

struct point calcRotatedFeetVector(int a, float theta, struct point center)
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


struct point calcRotatedRandomVector(int a, struct point vector, float theta, struct point center)
{
  switch (a) {
    case 0:
      struct point lfs1 = vec_sum(center, lfs0);
      struct point lff1 = vec_sum(center, vec_sum(lfs0,vector));
      struct point lffsp = quat_rot(y_axis, lff1, theta);
      return vec_sub(lffsp, lfs1);
      break;

    case 1:
      struct point rfs1 = vec_sum(center, rfs0);
      struct point rff1 = vec_sum(center, vec_sum(rfs0,vector));
      struct point rffsp = quat_rot(y_axis, rff1, theta);
      return vec_sub(rffsp, rfs1);
      break;

    case 2:
      struct point lbs1 = vec_sum(center, lbs0);
      struct point lbf1 = vec_sum(center, vec_sum(lbs0,vector));
      struct point lbfsp = quat_rot(y_axis, lbf1, theta);
      return vec_sub(lbfsp, lbs1);
      break;

    case 3:
      struct point rbs1 = vec_sum(center, rbs0);
      struct point rbf1 = vec_sum(center, vec_sum(rbs0,vector));
      struct point rbfsp = quat_rot(y_axis, rbf1, theta);
      return vec_sub(rbfsp, rbs1);
      break;
  }
}



void continuous_creep()
{
  float thetax=pi/6;
  
  for(int t=0;t<=40;t+=5)
  {
    actuate(0,0,-H,-Lc+t/2);
    actuate(1,0,-H,Lc+t/2);
    actuate(2,0,-H,-Lc+t/2);
    actuate(3,0,-H,Lc+t/2);
    delay(500);
  }
  for(int t=0;t<=40;t+=5)
  {
    actuate(1,-s*t/40,-H+20-abs(t-20),Lc+20);
    delay(500);
  }
  for(int t=0;t<=40;t+=5)
  {
    actuate(3,s*t/40,-H+20-abs(t-20),Lc+20);
    delay(500);
  }

  
  while(true)
  {
    struct point center={0,0,0};
    if(thetax<0)   //leftcurve
    {
      center.z=s/thetax+W/2;
      
      struct point lfdp=calcRotatedFeetVector(0,thetax,center);
      struct point rfdp=calcRotatedFeetVector(1,thetax,center);
      struct point lbdp=calcRotatedFeetVector(2,thetax,center);
      struct point rbdp=calcRotatedFeetVector(3,thetax,center);

      struct point oldlfd;
      struct point oldrfd;
      struct point oldlbd;
      struct point oldrbd;


      Serial.println("rightcurve");
      oldrfd=rfd;
      for(int t=0;t<=40;t+=5)
      {
        actuate(1,oldrfd.x+(rfdp.x-oldrfd.x)*t/40,-H+20-abs(t-20),oldrfd.z+(rfdp.z+20-oldrfd.z)*t/40);
        delay(500);
      }

      for(int t=1;t<=8;t++)
      {
        lfd=calcRotatedRandomVector(0,lfd,-thetax/8,center);
        rfd=calcRotatedRandomVector(1,rfd,-thetax/8,center);
        lbd=calcRotatedRandomVector(2,lbd,-thetax/8,center);
        rbd=calcRotatedRandomVector(3,rbd,-thetax/8,center);
        actuate(0,lfd.x,lfd.y,lfd.z);
        actuate(1,rfd.x,rfd.y,rfd.z);
        actuate(2,lbd.x,lbd.y,lbd.z);
        actuate(3,rbd.x,rbd.y,rbd.z);
        delay(500);
      }

      for(int t=1;t<=8;t++)
      {
        lfd.z=lfd.z-5;
        rfd.z=rfd.z-5;
        lbd.z=lbd.z-5;
        rbd.z=rbd.z-5;
        actuate(0,lfd.x,lfd.y,lfd.z);
        actuate(1,rfd.x,rfd.y,rfd.z);
        actuate(2,lbd.x,lbd.y,lbd.z);
        actuate(3,rbd.x,rbd.y,rbd.z);
        delay(500);
      }

      oldlbd=lbd;
      for(int t=0;t<=40;t+=5)
      {
        actuate(2,oldlbd.x+(lbdp.x-oldlbd.x)*t/40,-H+20-abs(t-20),oldlbd.z+(lbdp.z-20-oldlbd.z)*t/40);
        delay(500);
      }

      oldlfd=lfd;
      for(int t=0;t<=40;t+=5)
      {
        actuate(0,oldlfd.x+(lfdp.x-oldlfd.x)*t/40,-H+20-abs(t-20),oldlfd.z+(lfdp.z-20-oldlfd.z)*t/40);
        delay(500);
      }

      for(int t=1;t<=8;t++)
      {
        lfd=calcRotatedRandomVector(0,lfd,-thetax/8,center);
        rfd=calcRotatedRandomVector(1,rfd,-thetax/8,center);
        lbd=calcRotatedRandomVector(2,lbd,-thetax/8,center);
        rbd=calcRotatedRandomVector(3,rbd,-thetax/8,center);
        actuate(0,lfd.x,lfd.y,lfd.z);
        actuate(1,rfd.x,rfd.y,rfd.z);
        actuate(2,lbd.x,lbd.y,lbd.z);
        actuate(3,rbd.x,rbd.y,rbd.z);
        delay(500);
      }
      
      for(int t=1;t<=8;t++)
      {
        lfd.z=lfd.z+5;
        rfd.z=rfd.z+5;
        lbd.z=lbd.z+5;
        rbd.z=rbd.z+5;
        actuate(0,lfd.x,lfd.y,lfd.z);
        actuate(1,rfd.x,rfd.y,rfd.z);
        actuate(2,lbd.x,lbd.y,lbd.z);
        actuate(3,rbd.x,rbd.y,rbd.z);
        delay(500);
      }

      oldrbd=rbd;
      for(int t=0;t<=40;t+=5)
      {
        actuate(3,oldrbd.x+(rbdp.x-oldrbd.x)*t/40,-H+20-abs(t-20),oldrbd.z+(rbdp.z+20-oldrbd.z)*t/40);
        delay(500);
      }

    }

    if(thetax>0)   //rightcurve
    {
      center.z=s/thetax-W/2;
      
      struct point lfdp=calcRotatedFeetVector(0,-thetax,center);
      struct point rfdp=calcRotatedFeetVector(1,-thetax,center);
      struct point lbdp=calcRotatedFeetVector(2,-thetax,center);
      struct point rbdp=calcRotatedFeetVector(3,-thetax,center);

      struct point oldlfd;
      struct point oldrfd;
      struct point oldlbd;
      struct point oldrbd;


      Serial.println("rightcurve");
      oldrfd=rfd;
      for(int t=0;t<=40;t+=5)
      {
        actuate(1,oldrfd.x+(rfdp.x-oldrfd.x)*t/40,-H+20-abs(t-20),oldrfd.z+(rfdp.z+20-oldrfd.z)*t/40);
        delay(500);
      }

      for(int t=1;t<=8;t++)
      {
        lfd=calcRotatedRandomVector(0,lfd,thetax/8,center);
        rfd=calcRotatedRandomVector(1,rfd,thetax/8,center);
        lbd=calcRotatedRandomVector(2,lbd,thetax/8,center);
        rbd=calcRotatedRandomVector(3,rbd,thetax/8,center);
        actuate(0,lfd.x,lfd.y,lfd.z);
        actuate(1,rfd.x,rfd.y,rfd.z);
        actuate(2,lbd.x,lbd.y,lbd.z);
        actuate(3,rbd.x,rbd.y,rbd.z);
        delay(500);
      }

      for(int t=1;t<=8;t++)
      {
        lfd.z=lfd.z-5;
        rfd.z=rfd.z-5;
        lbd.z=lbd.z-5;
        rbd.z=rbd.z-5;
        actuate(0,lfd.x,lfd.y,lfd.z);
        actuate(1,rfd.x,rfd.y,rfd.z);
        actuate(2,lbd.x,lbd.y,lbd.z);
        actuate(3,rbd.x,rbd.y,rbd.z);
        delay(500);
      }

      oldlbd=lbd;
      for(int t=0;t<=40;t+=5)
      {
        actuate(2,oldlbd.x+(lbdp.x-oldlbd.x)*t/40,-H+20-abs(t-20),oldlbd.z+(lbdp.z-20-oldlbd.z)*t/40);
        delay(500);
      }

      oldlfd=lfd;
      for(int t=0;t<=40;t+=5)
      {
        actuate(0,oldlfd.x+(lfdp.x-oldlfd.x)*t/40,-H+20-abs(t-20),oldlfd.z+(lfdp.z-20-oldlfd.z)*t/40);
        delay(500);
      }

      for(int t=1;t<=8;t++)
      {
        lfd=calcRotatedRandomVector(0,lfd,thetax/8,center);
        rfd=calcRotatedRandomVector(1,rfd,thetax/8,center);
        lbd=calcRotatedRandomVector(2,lbd,thetax/8,center);
        rbd=calcRotatedRandomVector(3,rbd,thetax/8,center);
        actuate(0,lfd.x,lfd.y,lfd.z);
        actuate(1,rfd.x,rfd.y,rfd.z);
        actuate(2,lbd.x,lbd.y,lbd.z);
        actuate(3,rbd.x,rbd.y,rbd.z);
        delay(500);
      }
      
      for(int t=1;t<=8;t++)
      {
        lfd.z=lfd.z+5;
        rfd.z=rfd.z+5;
        lbd.z=lbd.z+5;
        rbd.z=rbd.z+5;
        actuate(0,lfd.x,lfd.y,lfd.z);
        actuate(1,rfd.x,rfd.y,rfd.z);
        actuate(2,lbd.x,lbd.y,lbd.z);
        actuate(3,rbd.x,rbd.y,rbd.z);
        delay(500);
      }

      oldrbd=rbd;
      for(int t=0;t<=40;t+=5)
      {
        actuate(3,oldrbd.x+(rbdp.x-oldrbd.x)*t/40,-H+20-abs(t-20),oldrbd.z+(rbdp.z+20-oldrbd.z)*t/40);
        delay(500);
      }
    
    }

    if(thetax==0)
    {
      for(int t=0;t<=40;t+=5)
      {
        actuate(1,-s+2*s*t/40,-H+20-abs(t-20),Lc+20);
        delay(500);
      }
      for(int t=0;t<=40;t+=5)
      {
        actuate(0,-s*t/40,-H,-Lc+20);
        actuate(1,s-s*t/40,-H,Lc+20);
        actuate(2,-s*t/40,-H,-Lc+20);
        actuate(3,s-s*t/40,-H,Lc+20);
        delay(500);
      }
      for(int t=0;t<=40;t+=5)
      {
        actuate(0,-s,-H,-Lc+20-t);
        actuate(1,0,-H,Lc+20-t);
        actuate(2,-s,-H,-Lc+20-t);
        actuate(3,0,-H,Lc+20-t);
        delay(500);
      }

      for(int t=0;t<=40;t+=5)
      {
        actuate(2,-s+2*s*t/40,-H+20-abs(t-20),-Lc-20);
        delay(500);
      }
      for(int t=0;t<=40;t+=5)
      {
        actuate(0,-s+2*s*t/40,-H+20-abs(t-20),-Lc-20);
        delay(500);
      }
      
      for(int t=0;t<=40;t+=5)
      {
        actuate(0,s-s*t/40,-H,-Lc-20);
        actuate(1,-s*t/40,-H,Lc-20);
        actuate(2,s-s*t/40,-H,-Lc-20);
        actuate(3,-s*t/40,-H,Lc-20);
        delay(500);
      }

      for(int t=0;t<=40;t+=5)
      {
        actuate(0,0,-H,-Lc-20+t);
        actuate(1,-s,-H,Lc-20+t);
        actuate(2,0,-H,-Lc-20+t);
        actuate(3,-s,-H,Lc-20+t);
        delay(500);
      }

      for(int t=0;t<=40;t+=5)
      {
        actuate(3,-s+2*s*t/40,-H+20-abs(t-20),Lc+20);
        delay(500);
      }
    }
  }
  
}


// ==========================================================
// ===                  Various Gaits                     ===
// ==========================================================

//creep

void start()
{
  for (int cnt = 0; cnt <= 30; cnt++)
  {
    actuate(0, 0, -H, -Lc);
    actuate(1, 0, -H, Lc);
    actuate(2, 0, -H, -Lc);
    actuate(3, 10 + sc * cnt / 30, -H + 30 - 2 * abs(cnt - 15), Lc);
    delay(10);
  }
  for (int cnt = 0; cnt <= 30; cnt++)
  {
    actuate(1,  - sc * cnt / 30, -H + 30 - 2 * abs(cnt - 15), Lc);
    delay(10);
  }
}

void cycle()
{
  for (int cnt = 0; cnt <= 60; cnt+=5)
  {
    actuate(1,  2 * sc * cnt / 60, -H + 60 - 2 * abs(cnt - 30), Lc);
    delay(10);
  }
  for (int cnt = 0; cnt <= 30; cnt+=5)
  {
    actuate(0,  - sc * cnt / 30, -H, -Lc);
    actuate(1, + sc  - sc * cnt / 30, -H, Lc);
    actuate(2,  - sc * cnt / 30, -H, -Lc);
    actuate(3, + sc  - sc * cnt / 30, -H, Lc);
    delay(10);
  }
  for (int cnt = 0; cnt <= 60; cnt+=5)
  {
    actuate(2, - sc + 2 * sc * cnt / 60, -H  + 60 - 2 * abs(cnt - 30), -Lc);
    delay(10);
  }
  for (int cnt = 0; cnt <= 60;  cnt+=5)
  {
    actuate(0, - sc + 2 * sc * cnt / 60, -H + 60 - 2 * abs(cnt - 30), -Lc);
    delay(10);
  }
  for (int cnt = 0; cnt <= 30; cnt+=5)
  {
    actuate(0, + sc  - sc * cnt / 30, -H, -Lc);
    actuate(1, - sc * cnt / 30, -H, Lc);
    actuate(2, + sc - sc * cnt / 30, -H, -Lc);
    actuate(3, - sc * cnt / 30, -H, Lc);
    delay(10);
  }
}

void cont()
{
  for (int cnt = 0; cnt <= 60; cnt+=5)
  {
    actuate(3, - sc + 2 * sc * cnt / 60, -H  + 60 - 2 * abs(cnt - 30), Lc);
    delay(10);
  }
}

void ending()
{
  for (int cnt = 0; cnt <= 30; cnt+=5)
  {
    actuate(3,  - sc + sc * cnt / 30, -H + 30 - 2 * abs(cnt - 15), Lc);
    delay(10);
  }
  for (int cnt = 0; cnt <= 30; cnt+=5)
  {
    actuate(1,  - sc + sc * cnt / 30, -H + 30 - 2 * abs(cnt - 15), Lc);
    delay(10);
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


// trot

void trot(int steps)
{
  int k=steps/2-1;
  for (int temp = 60; temp >= 30; temp-=10)
  {
    int t = 60 - temp;
    float theta = temp * pi / 180;
    actuate(1, s / 2 + s * cos(2 * theta), -H +10 -2*fabs(t/3-5), Lc);
    actuate(2, s / 2 + s * cos(2 * theta), -H +10 -2*fabs(t/3-5), -Lc);
    actuate(0,  - s * t / 30, -H, -Lc);
    actuate(3,  - s * t / 30, -H, Lc);
    delay(10);
  }
  delay(100);
  for(int i=1;i<=k;i++)
  {
    for (int temp = 120; temp >= 60; temp-=10)
    {
      int t = 120 - temp;
      float theta = temp * pi / 180;
      actuate(1, s - s * t / 30, -H, Lc);
      actuate(2, s - s * t / 30, -H, -Lc);
      actuate(0, 2 * s * cos(theta), -H +20 -2*fabs(t/3-10), -Lc);
      actuate(3, 2 * s * cos(theta), -H +20 -2*fabs(t/3-10), Lc);
      delay(10);
    }
    delay(100);
    for (int temp = 120; temp >= 60; temp-=10)
    {
      int t = 120 - temp;
      float theta = temp * pi / 180;
      actuate(0, s - s * t / 30, -H, -Lc);
      actuate(3, s - s * t / 30, -H, Lc);
      actuate(1, 2 * s * cos(theta), -H +20 -2*fabs(t/3-10), Lc);
      actuate(2, 2 * s * cos(theta), -H +20 -2*fabs(t/3-10), -Lc);
      delay(10);
    }
    delay(100);
  }
  for (int temp = 60; temp >= 30; temp-=10)
  {
    int t = 60 - temp;
    float theta = temp * pi / 180;
    actuate(1, s - s * t / 30, -H, Lc);
    actuate(2, s - s * t / 30, -H, -Lc);
    actuate(0, -s / 2 + s * cos(2 * theta), -H +10 -2*fabs(t/3-5), -Lc);
    actuate(3, -s / 2 + s * cos(2 * theta), -H +10 -2*fabs(t/3-5), Lc);
    delay(10);
  }
}



void backtrot(int steps)
{
  int k=steps/2-1;
  for (int temp = 30; temp <= 60; temp+=5)
  {
    float theta = temp * pi / 180;
    int t = 60 - temp;
    actuate(1, - s / 2 + s * cos(2 * theta), -H+10 -2*fabs(t/3-5), Lc);
    actuate(2, - s / 2 + s * cos(2 * theta), -H+10 -2*fabs(t/3-5), -Lc);
    actuate(0, s - s * t / 30, -H, -Lc);
    actuate(3, s - s * t / 30, -H, Lc);
    delay(10);
  }
  delay(10);
  for(int i=1;i<=k;i++)
  {
    for (int temp = 60; temp <= 120; temp+=5)
    {
      float theta = temp * pi / 180;
      int t = 120 - temp;
      actuate(1, s - s * t / 30, -H, Lc);
      actuate(2, s - s * t / 30, -H, -Lc);
      actuate(0, 2 * s * cos(theta),-H +20 -2*fabs(t/3-10), -Lc);
      actuate(3, 2 * s * cos(theta),-H +20 -2*fabs(t/3-10), Lc);
      delay(10);
    }
    delay(10);
    for (int temp = 60; temp <= 120; temp+=5)
    {
      float theta = temp * pi / 180;
      int t = 120 - temp;
      actuate(0, s - s * t / 30, -H, -Lc);
      actuate(3, s - s * t / 30, -H, Lc);
      actuate(1, 2 * s * cos(theta), -H +20 -2*fabs(t/3-10), Lc);
      actuate(2, 2 * s * cos(theta), -H +20 -2*fabs(t/3-10), -Lc);
      delay(10);
    }
    delay(10);
  }
  for (int temp = 30; temp <= 60; temp+=5)
  {
    float theta = temp * pi / 180;
    int t = 60 - temp;
    actuate(1,  -s * t / 30, -H, Lc);
    actuate(2,  -s * t / 30, -H, -Lc);
    actuate(0, s / 2 + s * cos(2 * theta), -H +10 -2*fabs(t/3-5), -Lc);
    actuate(3, s / 2 + s * cos(2 * theta), -H +10 -2*fabs(t/3-5), Lc);
    delay(10);
  }
}




void spin(int steps)
{
  int k=steps/2;
  float theta =  pi / 6;
  for(int i=1;i<=k;i++)
  {
    //code1
    //step1
    for (int t = 0; t <= 60; t+=5)
    {
      lff = quat_rot(y_axis, lff0, theta / 2 * t / 60);
      lbf = quat_rot(y_axis, lbf0, -theta / 2 * t / 60);
      rff = quat_rot(y_axis, rff0, -theta / 2 * t / 60);
      rbf = quat_rot(y_axis, rbf0, theta / 2 * t / 60);
      lfd = vec_sub(lff, lfs0);
      lbd = vec_sub(lbf, lbs0);
      rfd = vec_sub(rff, rfs0);
      rbd = vec_sub(rbf, rbs0);
      actuate(0, lfd.x, lfd.y + 10 - abs(t / 3 - 10), lfd.z);
      actuate(1, rfd.x, rfd.y, rfd.z);
      actuate(2, lbd.x, lbd.y, lbd.z);
      actuate(3, rbd.x, rbd.y + 10 - abs(t / 3 - 10), rbd.z);
      delay(10);
    }
    //step 2
    for (int t = -60; t <= 0; t+=5)
    {
      lff = quat_rot(y_axis, lff0, -theta / 2 * t / 60);
      lbf = quat_rot(y_axis, lbf0, theta / 2 * t / 60);
      rff = quat_rot(y_axis, rff0, theta / 2 * t / 60);
      rbf = quat_rot(y_axis, rbf0, -theta / 2 * t / 60);
      lfd = vec_sub(lff, lfs0);
      lbd = vec_sub(lbf, lbs0);
      rfd = vec_sub(rff, rfs0);
      rbd = vec_sub(rbf, rbs0);
      actuate(0, lfd.x, lfd.y, lfd.z);
      actuate(1, rfd.x, rfd.y + 10 - abs(-t / 15 - 10), rfd.z);
      actuate(2, lbd.x, lbd.y + 10 - abs(-t / 15 - 10), lbd.z);
      actuate(3, rbd.x, rbd.y, rbd.z);
      delay(10);
    }
  }
}

void backspin(int steps)
{
  int k=steps/2;
  float theta = -pi / 6;
  for(int i=1;i<=k;i++)
  {
    //step1
    for (int t = 0; t <= 60; t+=5)
    {
      lff = quat_rot(y_axis, lff0, -theta / 2 * t / 60);
      lbf = quat_rot(y_axis, lbf0, theta / 2 * t / 60);
      rff = quat_rot(y_axis, rff0, theta / 2 * t / 60);
      rbf = quat_rot(y_axis, rbf0, -theta / 2 * t / 60);
      actuate(0, lfd.x, lfd.y + 10 - abs(t / 3 - 10), lfd.z);
      actuate(1, rfd.x, rfd.y, rfd.z);
      actuate(2, lbd.x, lbd.y, lbd.z);
      actuate(3, rbd.x, rbd.y + 10 - abs(t / 3 - 10), rbd.z);
      delay(10);
    }
    //step2
    for (int t = -60; t <= 0; t+=5)
    {
      lff = quat_rot(y_axis, lff0, theta / 2 * t / 60);
      lbf = quat_rot(y_axis, lbf0, -theta / 2 * t / 60);
      rff = quat_rot(y_axis, rff0, -theta / 2 * t / 60);
      rbf = quat_rot(y_axis, rbf0, theta / 2 * t / 60);
      actuate(0, lfd.x, lfd.y, lfd.z);
      actuate(1, rfd.x, rfd.y + 10 - abs(-t / 3 - 10), rfd.z);
      actuate(2, lbd.x, lbd.y + 10 - abs(-t / 3 - 10), lbd.z);
      actuate(3, rbd.x, rbd.y, rbd.z);
      delay(10);
    }
  }
}

void translateright(int steps)
{
  int k=steps/2-1;
  /*replace z with x in 'trot'*/
  for (int temp = 60; temp >= 30; temp-=5)
  {
    float theta = temp * pi / 180;
    int t = 60 - temp;
    actuate(1, 0, -H +10 -2*fabs(t/3-5), s / 2 + s * cos(2 * theta) + Lc);
    actuate(2, 0, -H +10 -2*fabs(t/3-5), s / 2 + s * cos(2 * theta) - Lc);
    actuate(0, 0, -H, -s * t / 30 - Lc);
    actuate(3, 0, -H, -s * t / 30 + Lc);
    delay(10);
  }
  delay(100);
  for(int i=1;i<=k;i++)
  {
    for (int temp = 120; temp >= 60; temp-=5)
    {
      float theta = temp * pi / 180;
      int t = 120 - temp;
      actuate(1, 0, -H, s - s * t / 30 + Lc);
      actuate(2, 0, -H, s - s * t / 30 - Lc);
      actuate(0, 0, -H +20 -2*fabs(t/3-10), 2 * s * cos(theta) - Lc);
      actuate(3, 0, -H +20 -2*fabs(t/3-10), 2 * s * cos(theta) + Lc);
      delay(10);
    }
    delay(100);
    for (int temp = 120; temp >= 60; temp-=5)
    {
      float theta = temp * pi / 180;
      int t = 120 - temp;
      actuate(0, 0 , -H, s - s * t / 30 - Lc);
      actuate(3, 0, -H, s - s * t / 30 + Lc);
      actuate(1, 0, -H +20 -2*fabs(t/3-10), 2 * s * cos(theta) + Lc);
      actuate(2, 0, -H +20 -2*fabs(t/3-10), 2 * s * cos(theta) - Lc);
      delay(10);
    }
    delay(100);
  }
  for (int temp = 60; temp >= 30; temp-=5)
  {
    float theta = temp * pi / 180;
    int t = 60 - temp;
    actuate(1, 0, -H, s - s * t / 30 + Lc);
    actuate(2, 0, -H, s - s * t / 30 - Lc);
    actuate(0, 0, -H +10 -2*fabs(t/3-5), -s / 2 + s * cos(2 * theta) - Lc);
    actuate(3, 0, -H +10 -2*fabs(t/3-5), -s / 2 + s * cos(2 * theta) + Lc);
    delay(10);
  }
}



void translateleft(int steps)
{
  int k=steps/2-1;
  /*replace z with x in 'trot'*/
  for (int temp = 30; temp <= 60; temp+=5)
  {
    float theta = temp * pi / 180;
    int t = 60 - temp;
    actuate(1, 0, -H +10 -2*fabs(t/3-5), -s / 2 + s * cos(2 * theta) + Lc);
    actuate(2, 0, -H +10 -2*fabs(t/3-5), -s / 2 + s * cos(2 * theta) - Lc);
    actuate(0, 0, -H, s - s * t / 30 - Lc);
    actuate(3, 0, -H, s - s * t / 30 + Lc);
    delay(10);
  }
  delay(100);
  for (int i=1;i<=k;i++)
  {
    for (int temp = 60; temp <= 120; temp+=5)
    {
      float theta = temp * pi / 180;
      int t = 120 - temp;
      actuate(1, 0, -H, s - s * t / 30 + Lc);
      actuate(2, 0, -H, s - s * t / 30 - Lc);
      actuate(0, 0, -H +20 -2*fabs(t/3-10), 2 * s * cos(theta) - Lc);
      actuate(3, 0, -H +20 -2*fabs(t/3-10), 2 * s * cos(theta) + Lc);
      delay(10);
    }
    delay(100);
    for (int temp = 60; temp <= 120; temp+=5)
    {
      float theta = temp * pi / 180;
      int t = 120 - temp;
      actuate(0, 0 , -H, s - s * t / 30 - Lc);
      actuate(3, 0, -H, s - s * t / 30 + Lc);
      actuate(1, 0, -H +20 -2*fabs(t/3-10), 2 * s * cos(theta) + Lc);
      actuate(2, 0, -H +20 -2*fabs(t/3-10), 2 * s * cos(theta) - Lc);
      delay(10);
    }
    delay(100);
  }
  for (int temp = 30; temp <= 60; temp+=5)
  {
    float theta = temp * pi / 180;
    int t = 60 - temp;
    actuate(1, 0, -H, - s * t / 30 + Lc);
    actuate(2, 0, -H, - s * t / 30 - Lc);
    actuate(0, 0, -H +10 -2*fabs(t/3-5), s / 2 + s * cos(2 * theta) - Lc);
    actuate(3, 0, -H +10 -2*fabs(t/3-5), s / 2 + s * cos(2 * theta) + Lc);
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

void leftcurve(int steps)
{
  struct point center = {0, 0, 100};
  float theta = 0.2;
  int k=steps/2-1;

  for (int t = 0; t <= 60; t+=15)
  {
    float currAngle = theta * t / 60;
    calcFootVector(0, currAngle, center);
    calcFootVector(3, currAngle, center);
    calcFootVector(1, -currAngle, center);
    calcFootVector(2, -currAngle, center);
    actuate(0, lfdsp.x, lfdsp.y + 10 - abs(t / 3 - 10), lfdsp.z);
    actuate(3, rbdsp.x, rbdsp.y + 10 - abs(t / 3 - 10), rbdsp.z);
    actuate(1, rfdsp.x, rfdsp.y, rfdsp.z);
    actuate(2, lbdsp.x, lbdsp.y, lbdsp.z);
    delay(10);
  }
  delay(100);
  for (int i=1;i<=k;i++)
  {
    for (int t = 0; t <= 120; t+=15)
    {
      float currAngle = theta - theta * t / 60;
      calcFootVector(0, currAngle, center);
      calcFootVector(3, currAngle, center);
      calcFootVector(1, -currAngle, center);
      calcFootVector(2, -currAngle, center);
      actuate(0, lfdsp.x, lfdsp.y , lfdsp.z);
      actuate(3, rbdsp.x, rbdsp.y , rbdsp.z);
      actuate(1, rfdsp.x, rfdsp.y + 20 - abs(t / 3 - 20), rfdsp.z);
      actuate(2, lbdsp.x, lbdsp.y + 20 - abs(t / 3 - 20), lbdsp.z);
      delay(10);
    }
    delay(100);
    for (int t = 0; t <= 120; t+=15)
    {
      float currAngle = -theta + theta * t / 60;
      calcFootVector(0, currAngle, center);
      calcFootVector(3, currAngle, center);
      calcFootVector(1, -currAngle, center);
      calcFootVector(2, -currAngle, center);
      actuate(0, lfdsp.x, lfdsp.y + 20 - abs(t / 3 - 20) , lfdsp.z);
      actuate(3, rbdsp.x, rbdsp.y + 20 - abs(t / 3 - 20), rbdsp.z);
      actuate(1, rfdsp.x, rfdsp.y , rfdsp.z);
      actuate(2, lbdsp.x, lbdsp.y , lbdsp.z);
      delay(10);
    }
    delay(100);
  }
  for (int t = 60; t >= 0; t-=15)
  {
    float currAngle = theta - theta * t / 60;
    calcFootVector(0, currAngle, center);
    calcFootVector(3, currAngle, center);
    calcFootVector(1, -currAngle, center);
    calcFootVector(2, -currAngle, center);
    actuate(0, lfdsp.x, lfdsp.y, lfdsp.z);
    actuate(3, rbdsp.x, rbdsp.y , rbdsp.z);
    actuate(1, rfdsp.x, rfdsp.y + 10 - abs(t / 3 - 10), rfdsp.z);
    actuate(2, lbdsp.x, lbdsp.y + 10 - abs(t / 3 - 10), lbdsp.z);
    delay(10);
  }
}

void rightcurve(int steps)
{
  struct point center = {0, 0, -100};
  float theta = 0.2;
  int k=steps/2-1;

  for (int t = 0; t <= 60; t+=15)
  {
    float currAngle = theta * t / 60;
    calcFootVector(0, -currAngle, center);
    calcFootVector(3, -currAngle, center);
    calcFootVector(1, currAngle, center);
    calcFootVector(2, currAngle, center);
    actuate(0, lfdsp.x, lfdsp.y + 10 - abs(t / 3 - 10), lfdsp.z);
    actuate(3, rbdsp.x, rbdsp.y + 10 - abs(t / 3 - 10), rbdsp.z);
    actuate(1, rfdsp.x, rfdsp.y, rfdsp.z);
    actuate(2, lbdsp.x, lbdsp.y, lbdsp.z);
    delay(10);
  }
  delay(100);
  for(int i=1;i<=k;i++)
  {
    for (int t = 0; t <= 120; t+=15)
    {
      float currAngle = theta - theta * t / 60;
      calcFootVector(0, -currAngle, center);
      calcFootVector(3, -currAngle, center);
      calcFootVector(1, currAngle, center);
      calcFootVector(2, currAngle, center);
      actuate(0, lfdsp.x, lfdsp.y , lfdsp.z);
      actuate(3, rbdsp.x, rbdsp.y , rbdsp.z);
      actuate(1, rfdsp.x, rfdsp.y + 20 - abs(t / 3 - 20), rfdsp.z);
      actuate(2, lbdsp.x, lbdsp.y + 20 - abs(t / 3 - 20), lbdsp.z);
      delay(10);
    }
    delay(100);
    for (int t = 0; t <= 120; t+=15)
    {
      float currAngle = -theta + theta * t / 60;
      calcFootVector(0, -currAngle, center);
      calcFootVector(3, -currAngle, center);
      calcFootVector(1, currAngle, center);
      calcFootVector(2, currAngle, center);
      actuate(0, lfdsp.x, lfdsp.y + 20 - abs(t / 3 - 20) , lfdsp.z);
      actuate(3, rbdsp.x, rbdsp.y + 20 - abs(t / 3 - 20), rbdsp.z);
      actuate(1, rfdsp.x, rfdsp.y , rfdsp.z);
      actuate(2, lbdsp.x, lbdsp.y , lbdsp.z);
      delay(10);
    }
    delay(100);
  }
  for (int t = 60; t >= 0; t-=15)
  {
    float currAngle = theta - theta * t / 60;
    calcFootVector(0, -currAngle, center);
    calcFootVector(3, -currAngle, center);
    calcFootVector(1, currAngle, center);
    calcFootVector(2, currAngle, center);
    actuate(0, lfdsp.x, lfdsp.y, lfdsp.z);
    actuate(3, rbdsp.x, rbdsp.y , rbdsp.z);
    actuate(1, rfdsp.x, rfdsp.y + 10 - abs(t / 3 - 10), rfdsp.z);
    actuate(2, lbdsp.x, lbdsp.y + 10 - abs(t / 3 - 10), lbdsp.z);
    delay(10);
  }
}

void wave()
{
  for (int t = 0; t <= 20; t++)
  {
    actuate(0, t, -H - 10, -Lc);
    actuate(1, t, -H - 10, Lc);
    actuate(2, t, -H - 10, -Lc);
    actuate(3, t, -H - 10, Lc);
    delay(2);
  }
  HCPCA9685.Servo(1, 120);
  for (int t = 240; t >= 120; t--)
  {
    HCPCA9685.Servo(2, t);
    delay(3);
  }
  for (int t = 1; t <= 3; t++)
  {
    for (int t = 120; t <= 360; t++)
    {
      HCPCA9685.Servo(2, t);
      delay(3);
    }
    for (int t = 360; t >= 120; t--)
    {
      HCPCA9685.Servo(2, t);
      delay(3);
    }
  }
  for (int t = 120; t <= 240; t++)
  {
    HCPCA9685.Servo(2, t);
    delay(3);
  }
  actuate(0, 0, -H, -Lc);
  for (int t = 20; t >= 0; t--)
  {
    actuate(0, t, -H - 10, -Lc);
    actuate(1, t, -H - 10, Lc);
    actuate(2, t, -H - 10, -Lc);
    actuate(3, t, -H - 10, Lc);
    delay(2);
  }
}

// ==========================================================
// ===               Change Orientation                   ===
// ==========================================================

void changeRoll(float angle)
{
  angle = angle * pi / 180;
  for (int t = 0; t <= 60; t+=5)
  {
    stat_rot(x_axis, t * angle / 60);
    delay(10);
  }
  for (int t = 60; t >= 0; t-=5)
  {
    stat_rot(x_axis, t * angle / 60);
    delay(10);
  }
}


void changePitch(float angle)
{
  angle = angle * pi / 180;
  for (int t = 0; t <= 60; t+=5)
  {
    stat_rot(z_axis, t * angle / 60);
    delay(10);
  }
  for (int t = 60; t >= 0; t-=5)
  {
    stat_rot(z_axis, t * angle / 60);
    delay(10);
  }
}

void changeYaw(float angle)
{
  angle = angle * pi / 180;
  for (int t = 0; t <= 60; t+=5)
  {
    stat_rot(y_axis, t * angle / 60);
    delay(10);
  }
  for (int t = 60; t >= 0; t-=5)
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



// ==========================================================
// ===              Inverse Kinematics                    ===
// ==========================================================


void actuate(int a, float x, float y, float z)
{
  if (a == 1)
  {
    rfd.x=x;
    rfd.y=y;
    rfd.z=z;
    float x1 = -x;
    float y1 = -sqrt(y * y + z * z - Lc * Lc);
    HCPCA9685.Servo(4 * a + 0, mapping(asin(Lc / sqrt(y * y + z * z)) + atan(z / y), -pi / 2, pi / 2, -10, 470));
    HCPCA9685.Servo(4 * a + 1, mapping(acos((Lf * Lf + x1 * x1 + y1 * y1 - Lt * Lt) / (2 * Lf * sqrt(x1 * x1 + y1 * y1))) - atan(x1 / y1), -pi / 2, pi / 2, 460, -20));
    HCPCA9685.Servo(4 * a + 2, mapping(asin((x * x + y * y + z * z - Lc * Lc - Lt * Lt - Lf * Lf) / (2 * Lf * Lt)), -pi / 2, pi / 2, -10, 470));

  }
  if (a == 0)
  {
    lfd.x=x;
    lfd.y=y;
    lfd.z=z;
    Serial.print("x: ");
    Serial.print(x);
    Serial.print("z: ");
    Serial.println(z);
    float x1 = -x;
    float y1 = -sqrt(y * y + z * z - Lc * Lc);
    HCPCA9685.Servo(4 * a + 0, mapping(asin(Lc / sqrt(y * y + z * z)) - atan(z / y), -pi / 2, pi / 2, 470, -10));
    HCPCA9685.Servo(4 * a + 1, mapping(acos((Lf * Lf + x1 * x1 + y1 * y1 - Lt * Lt) / (2 * Lf * sqrt(x1 * x1 + y1 * y1))) - atan(x1 / y1), -pi / 2, pi / 2, -30, 450));
    HCPCA9685.Servo(4 * a + 2, mapping(asin((x * x + y * y + z * z - Lc * Lc - Lt * Lt - Lf * Lf) / (2 * Lf * Lt)), -pi / 2, pi / 2, 450, -30));
  }
  if (a == 3)
  {
    rbd.x=x;
    rbd.y=y;
    rbd.z=z;
    float x1 = x;
    float y1 = -sqrt(y * y + z * z - Lc * Lc);
    HCPCA9685.Servo(4 * a + 0, mapping(asin(Lc / sqrt(y * y + z * z)) + atan(z / y), -pi / 2, pi / 2, 450, -30));
    HCPCA9685.Servo(4 * a + 1, mapping(acos((Lf * Lf + x1 * x1 + y1 * y1 - Lt * Lt) / (2 * Lf * sqrt(x1 * x1 + y1 * y1))) - atan(x1 / y1), -pi / 2, pi / 2, 0, 480));
    HCPCA9685.Servo(4 * a + 2, mapping(asin((x * x + y * y + z * z - Lc * Lc - Lt * Lt - Lf * Lf) / (2 * Lf * Lt)), -pi / 2, pi / 2, 450, -30));
  }
  if (a == 2)
  {
    lbd.x=x;
    lbd.y=y;
    lbd.z=z;
    float x1 = x;
    float y1 = -sqrt(y * y + z * z - Lc * Lc);
    HCPCA9685.Servo(4 * a + 0, mapping(asin(Lc / sqrt(y * y + z * z)) - atan(z / y), -pi / 2, pi / 2, -10, 470));
    HCPCA9685.Servo(4 * a + 1, mapping(acos((Lf * Lf + x1 * x1 + y1 * y1 - Lt * Lt) / (2 * Lf * sqrt(x1 * x1 + y1 * y1))) - atan(x1 / y1), -pi / 2, pi / 2, 480, 0));
    HCPCA9685.Servo(4 * a + 2, mapping(asin((x * x + y * y + z * z - Lc * Lc - Lt * Lt - Lf * Lf) / (2 * Lf * Lt)), -pi / 2, pi / 2, -30, 450));
  }
}

float mapping(float val, float fromLow, float fromHigh, float toLow, float toHigh)
{
  float scale = (toHigh - toLow) / (fromHigh - fromLow);
  return toLow + (val - fromLow) * scale;
}


void actuatePID(int a, float x, float y, float z)
{
  getValues();
  oldRoll=roll;
  rollSlope=(roll-oldRoll)*100;
  rollArea=rollArea+0.01*roll;

  oldPitch=pitch;
  pitchSlope=(pitch-oldPitch)*100;
  pitchArea=pitchArea+0.01*pitch;
  
  float z_act= rollkp*roll+rollki*rollArea+rollkd*rollSlope;
  float x_act= pitchkp*pitch+pitchki*pitchArea+pitchkd*pitchSlope;


  actuate(a,x-x_act,y,z+z_act);
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

        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            yaw=ypr[0] * 180/M_PI;
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            pitch=ypr[1] * 180/M_PI;
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
            roll=ypr[2] * 180/M_PI;
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
            // and rotated based on known orientation from quaterniong
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
