#include <HCPCA9685.h>

#define Lc 59.8
#define Lf 76.8
#define Lt 104.0
#define L 43.7
#define W 47.7
#define Loff 80
#define Woff 80
#define Hoff 50
#define s 25
#define sc 50

#define pi 3.1415926535

int order = 0;

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

struct point lff0 = {L / 2 + Loff, -Hoff, -W / 2 - Woff};
struct point lbf0 = { -L / 2 - Loff, -Hoff, -W / 2 - Woff};
struct point rff0 = {L / 2 + Loff, -Hoff, W / 2 + Woff};
struct point rbf0 = { -L / 2 - Loff, -Hoff, W / 2 + Woff};

struct point lff;
struct point lbf;
struct point rff;
struct point rbf;

struct point x_axis = {1, 0, 0};
struct point y_axis = {0, 1, 0};
struct point z_axis = {0, 0, 1};

/*vector calculations*/

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
  actuate(2, lbd.x, lbd.y, lbd.z);
  actuate(1, rfd.x, rfd.y, rfd.z);
  actuate(3, rbd.x, rbd.y, rbd.z);
}



HCPCA9685 HCPCA9685(0x40);


void setup() {
  // put your setup code here, to run once:
  HCPCA9685.Init(SERVO_MODE);
  HCPCA9685.Sleep(false);
  Serial.begin(38400);
  delay(1000);
  init_pos(1);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (getOrder() == '0')
  {
    creep();
  }
  if (getOrder() == '1')
  {
    creepback();
  }
  if (getOrder() == '2')
  {
    creepright();
  }
  if (getOrder() == '3')
  {
    creepleft();
  }
  if (getOrder() == '4' || getOrder == '5')
  {
    spin();
  }
  if(getOrder()=='6')
  {
    init_pos(1);
  }
}


int getOrder()
{
  if (Serial.available() > 0)
  {
    order = Serial.read();
  }
  return order;
}


void init_pos(int t)
{
  for (int cnt = 1; cnt <= t; cnt++)
  {
    actuate(0, Loff, -Hoff, -Woff);
    actuate(1, Loff, -Hoff, Woff);
    actuate(2, -Loff, -Hoff, -Woff);
    actuate(3, -Loff, -Hoff, Woff);
    delay(500);
  }
}

void bounce(int n)
{
  for (int cnt = 0; cnt < n; cnt++)
  {
    for (int i = 0; i <= 20; i++)
    {
      int t = 20 - abs(i - 10);
      actuate(0, Loff, -Hoff + t, -Woff);
      actuate(1, Loff, -Hoff + t, Woff);
      actuate(2, -Loff, -Hoff + t, -Woff);
      actuate(3, -Loff, -Hoff + t, Woff);
    }
  }
}


/////////////////////////////////////////



//gaits//

//creep gait//

void start()
{
  for (int cnt = 0; cnt <= 30; cnt++)
  {
    actuate(0, Loff, -Hoff, -Woff);
    actuate(1, Loff, -Hoff, Woff);
    actuate(2, -Loff, -Hoff, -Woff);
    actuate(3, -Loff + sc * cnt / 30, -Hoff + 30 - 2 * abs(cnt - 15), Woff);
    delay(10);
  }
  for (int cnt = 0; cnt <= 30; cnt++)
  {
    actuate(1, Loff - sc * cnt / 30, -Hoff + 30 - 2 * abs(cnt - 15), Woff);
    delay(10);
  }
}

void cycle()
{
  for (int cnt = 0; cnt <= 60; cnt++)
  {
    actuate(1, Loff + 2 * sc * cnt / 60, -Hoff + 60 - 2 * abs(cnt - 30), Woff);
    delay(10);
  }
  for (int cnt = 0; cnt <= 30; cnt++)
  {
    actuate(0, Loff - sc * cnt / 30, -Hoff, -Woff);
    actuate(1, Loff + sc  - sc * cnt / 30, -Hoff, Woff);
    actuate(2, -Loff - sc * cnt / 30, -Hoff, -Woff);
    actuate(3, -Loff + sc - sc * cnt / 30, -Hoff, Woff);
    delay(5);
  }
  for (int cnt = 0; cnt <= 60; cnt++)
  {
    actuate(2, -Loff - sc + 2 * sc * cnt / 60, -Hoff + 60 - 2 * abs(cnt - 30), -Woff);
    delay(10);
  }
  for (int cnt = 0; cnt <= 60;  cnt++)
  {
    actuate(0, Loff - sc + 2 * sc * cnt / 60, -Hoff + 60 - 2 * abs(cnt - 30), -Woff);
    delay(10);
  }
  for (int cnt = 0; cnt <= 30; cnt++)
  {
    actuate(0, Loff + sc  - sc * cnt / 30, -Hoff, -Woff);
    actuate(1, Loff  - sc * cnt / 30, -Hoff, Woff);
    actuate(2, -Loff + sc - sc * cnt / 30, -Hoff, -Woff);
    actuate(3, -Loff - sc * cnt / 30, -Hoff, Woff);
    delay(5);
  }
}

void cont()
{
  for (int cnt = 0; cnt <= 60; cnt++)
  {
    actuate(3, -Loff - sc + 2 * sc * cnt / 60, -Hoff + 60 - 2 * abs(cnt - 30), Woff);
    delay(10);
  }
}

void ending()
{
  for (int cnt = 0; cnt <= 30; cnt++)
  {
    actuate(3, -Loff - sc + sc * cnt / 30, -Hoff + 30 - 2 * abs(cnt - 15), Woff);
    delay(10);
  }
  for (int cnt = 0; cnt <= 30; cnt++)
  {
    actuate(1, Loff - sc + sc * cnt / 30, -Hoff + 30 - 2 * abs(cnt - 15), Woff);
    delay(10);
  }
}



void creep()
{
  start();
  while (getOrder() == '0')
  {
    cycle();
    cont();
  }
  cycle();
  ending();
  return;
}


void creepback()
{
  for (int cnt = 0; cnt <= 30; cnt++)
  {
    actuate(3, -Loff, -Hoff, Woff);
    actuate(2, -Loff, -Hoff, -Woff);
    actuate(1, Loff, -Hoff, Woff);
    actuate(0, Loff - sc * cnt / 30, -Hoff + 30 - 2 * abs(cnt - 15), -Woff);
    delay(10);
  }
  for (int cnt = 0; cnt <= 30; cnt++)
  {
    actuate(2, -Loff + sc * cnt / 30, -Hoff + 30 - 2 * abs(cnt - 15), -Woff);
    delay(10);
  }
  while (getOrder() == '1')
  {
    for (int cnt = 0; cnt <= 60; cnt++)
    {
      actuate(2, -Loff - 2 * sc * cnt / 60, -Hoff + 60 - 2 * abs(cnt - 30), -Woff);
      delay(10);
    }
    for (int cnt = 0; cnt <= 30; cnt++)
    {
      actuate(3, -Loff + sc * cnt / 30, -Hoff, Woff);
      actuate(2, -Loff - sc  + sc * cnt / 30, -Hoff, -Woff);
      actuate(1, Loff + sc * cnt / 30, -Hoff, Woff);
      actuate(0, Loff - sc + sc * cnt / 30, -Hoff, -Woff);
      delay(5);
    }
    for (int cnt = 0; cnt <= 60; cnt++)
    {
      actuate(1, Loff + sc - 2 * sc * cnt / 60, -Hoff + 60 - 2 * abs(cnt - 30), Woff);
      delay(10);
    }
    for (int cnt = 0; cnt <= 60;  cnt++)
    {
      actuate(3, -Loff + sc - 2 * sc * cnt / 60, -Hoff + 60 - 2 * abs(cnt - 30), Woff);
      delay(10);
    }
    for (int cnt = 0; cnt <= 30; cnt++)
    {
      actuate(3, -Loff - sc  + sc * cnt / 30, -Hoff, Woff);
      actuate(2, -Loff  + sc * cnt / 30, -Hoff, -Woff);
      actuate(1, Loff - sc + sc * cnt / 30, -Hoff, Woff);
      actuate(0, Loff + sc * cnt / 30, -Hoff, -Woff);
      delay(5);
    }
    for (int cnt = 0; cnt <= 60; cnt++)
    {
      actuate(0, Loff + sc - 2 * sc * cnt / 60, -Hoff + 60 - 2 * abs(cnt - 30), -Woff);
      delay(10);
    }
  }
  for (int cnt = 0; cnt <= 60; cnt++)
  {
    actuate(2, -Loff - 2 * sc * cnt / 60, -Hoff + 60 - 2 * abs(cnt - 30), -Woff);
    delay(10);
  }
  for (int cnt = 0; cnt <= 30; cnt++)
  {
    actuate(3, -Loff + sc * cnt / 30, -Hoff, Woff);
    actuate(2, -Loff - sc  + sc * cnt / 30, -Hoff, -Woff);
    actuate(1, Loff + sc * cnt / 30, -Hoff, Woff);
    actuate(0, Loff - sc + sc * cnt / 30, -Hoff, -Woff);
    delay(5);
  }
  for (int cnt = 0; cnt <= 60; cnt++)
  {
    actuate(1, Loff + sc - 2 * sc * cnt / 60, -Hoff + 60 - 2 * abs(cnt - 30), Woff);
    delay(10);
  }
  for (int cnt = 0; cnt <= 60;  cnt++)
  {
    actuate(3, -Loff + sc - 2 * sc * cnt / 60, -Hoff + 60 - 2 * abs(cnt - 30), Woff);
    delay(10);
  }
  for (int cnt = 0; cnt <= 30; cnt++)
  {
    actuate(3, -Loff - sc  + sc * cnt / 30, -Hoff, Woff);
    actuate(2, -Loff  + sc * cnt / 30, -Hoff, -Woff);
    actuate(1, Loff - sc + sc * cnt / 30, -Hoff, Woff);
    actuate(0, Loff + sc * cnt / 30, -Hoff, -Woff);
    delay(5);
  }
  for (int cnt = 0; cnt <= 30; cnt++)
  {
    actuate(0, Loff + sc - sc * cnt / 30, -Hoff + 30 - 2 * abs(cnt - 15), -Woff);
    delay(10);
  }
  for (int cnt = 0; cnt <= 30; cnt++)
  {
    actuate(2, -Loff + sc - sc * cnt / 30, -Hoff + 30 - 2 * abs(cnt - 15), -Woff);
    delay(10);
  }
  return;
}


void creepleft()
{
  for (int cnt = 0; cnt <= 30; cnt++)
  {
    actuate(2, -Loff, -Hoff, -Woff);
    actuate(0, Loff, -Hoff, -Woff);
    actuate(3, -Loff, -Hoff, Woff);
    actuate(1, Loff , -Hoff + 30 - 2 * abs(cnt - 15), Woff - sc * cnt / 30);
    delay(10);
  }
  for (int cnt = 0; cnt <= 30; cnt++)
  {
    actuate(0, Loff , -Hoff + 30 - 2 * abs(cnt - 15), -Woff + sc * cnt / 30);
    delay(10);
  }
  while (getOrder() == '3')
  {
    for (int cnt = 0; cnt <= 60; cnt++)
    {
      actuate(0, Loff , -Hoff + 60 - 2 * abs(cnt - 30), -Woff - 2 * sc * cnt / 60);
      delay(10);
    }
    for (int cnt = 0; cnt <= 30; cnt++)
    {
      actuate(2, -Loff , -Hoff, -Woff + sc * cnt / 30);
      actuate(0, Loff , -Hoff, -Woff - sc  + sc * cnt / 30);
      actuate(3, -Loff , -Hoff, Woff + sc * cnt / 30);
      actuate(1, Loff , -Hoff, Woff - sc + sc * cnt / 30);
      delay(5);
    }
    for (int cnt = 0; cnt <= 60; cnt++)
    {
      actuate(3, -Loff , -Hoff + 60 - 2 * abs(cnt - 30), Woff + sc - 2 * sc * cnt / 60);
      delay(10);
    }
    for (int cnt = 0; cnt <= 60;  cnt++)
    {
      actuate(2, -Loff , -Hoff + 60 - 2 * abs(cnt - 30), -Woff + sc - 2 * sc * cnt / 60);
      delay(10);
    }
    for (int cnt = 0; cnt <= 30; cnt++)
    {
      actuate(2, -Loff, -Hoff, -Woff - sc  + sc * cnt / 30);
      actuate(0, Loff , -Hoff, -Woff + sc * cnt / 30);
      actuate(3, -Loff , -Hoff, Woff - sc + sc * cnt / 30);
      actuate(1, Loff, -Hoff, Woff + sc * cnt / 30);
      delay(5);
    }
    for (int cnt = 0; cnt <= 60; cnt++)
    {
      actuate(1, Loff , -Hoff + 60 - 2 * abs(cnt - 30), Woff + sc - 2 * sc * cnt / 60);
      delay(10);
    }
  }
  for (int cnt = 0; cnt <= 60; cnt++)
  {
    actuate(0, Loff , -Hoff + 60 - 2 * abs(cnt - 30), -Woff - 2 * sc * cnt / 60);
    delay(10);
  }
  for (int cnt = 0; cnt <= 30; cnt++)
  {
    actuate(2, -Loff, -Hoff, -Woff + sc * cnt / 30);
    actuate(0, Loff, -Hoff, -Woff - sc  + sc * cnt / 30);
    actuate(3, -Loff , -Hoff, Woff + sc * cnt / 30);
    actuate(1, Loff , -Hoff, Woff - sc + sc * cnt / 30);
    delay(5);
  }
  for (int cnt = 0; cnt <= 60; cnt++)
  {
    actuate(3, -Loff , -Hoff + 60 - 2 * abs(cnt - 30), Woff + sc - 2 * sc * cnt / 60);
    delay(10);
  }
  for (int cnt = 0; cnt <= 60;  cnt++)
  {
    actuate(2, -Loff , -Hoff + 60 - 2 * abs(cnt - 30), -Woff + sc - 2 * sc * cnt / 60);
    delay(10);
  }
  for (int cnt = 0; cnt <= 30; cnt++)
  {
    actuate(2, -Loff , -Hoff, -Woff - sc  + sc * cnt / 30);
    actuate(0, Loff  , -Hoff, -Woff + sc * cnt / 30);
    actuate(3, -Loff , -Hoff, Woff - sc + sc * cnt / 30);
    actuate(1, Loff , -Hoff, Woff + sc * cnt / 30);
    delay(5);
  }
  for (int cnt = 0; cnt <= 30; cnt++)
  {
    actuate(1, Loff, -Hoff + 30 - 2 * abs(cnt - 15), Woff + sc - sc * cnt / 30);
    delay(10);
  }
  for (int cnt = 0; cnt <= 30; cnt++)
  {
    actuate(0, Loff , -Hoff + 30 - 2 * abs(cnt - 15), -Woff + sc - sc * cnt / 30);
    delay(10);
  }
  return;
}



void creepright()
{
  for (int cnt = 0; cnt <= 30; cnt++)
  {
    actuate(1, Loff, -Hoff, Woff);
    actuate(3, -Loff, -Hoff, Woff);
    actuate(0, Loff, -Hoff, -Woff);
    actuate(2, -Loff , -Hoff + 30 - 2 * abs(cnt - 15), -Woff + sc * cnt / 30);
    delay(10);
  }
  for (int cnt = 0; cnt <= 30; cnt++)
  {
    actuate(0, -Loff , -Hoff + 30 - 2 * abs(cnt - 15), Woff - sc * cnt / 30);
    delay(10);
  }
  while (getOrder() == '2')
  {
    for (int cnt = 0; cnt <= 60; cnt++)
    {
      actuate(3, -Loff , -Hoff + 60 - 2 * abs(cnt - 30), Woff + 2 * sc * cnt / 60);
      delay(10);
    }
    for (int cnt = 0; cnt <= 30; cnt++)
    {
      actuate(1, Loff , -Hoff, Woff - sc * cnt / 30);
      actuate(3, -Loff , -Hoff, Woff + sc  - sc * cnt / 30);
      actuate(0, Loff , -Hoff, -Woff - sc * cnt / 30);
      actuate(2, -Loff , -Hoff, -Woff + sc - sc * cnt / 30);
      delay(5);
    }
    for (int cnt = 0; cnt <= 60; cnt++)
    {
      actuate(0, Loff , -Hoff + 60 - 2 * abs(cnt - 30), -Woff - sc + 2 * sc * cnt / 60);
      delay(10);
    }
    for (int cnt = 0; cnt <= 60;  cnt++)
    {
      actuate(1, Loff , -Hoff + 60 - 2 * abs(cnt - 30), Woff - sc + 2 * sc * cnt / 60);
      delay(10);
    }
    for (int cnt = 0; cnt <= 30; cnt++)
    {
      actuate(1, Loff, -Hoff, Woff + sc  - sc * cnt / 30);
      actuate(3, -Loff , -Hoff, Woff - sc * cnt / 30);
      actuate(0, Loff , -Hoff, -Woff + sc - sc * cnt / 30);
      actuate(2, -Loff, -Hoff, -Woff - sc * cnt / 30);
      delay(5);
    }
    for (int cnt = 0; cnt <= 60; cnt++)
    {
      actuate(2, -Loff , -Hoff + 60 - 2 * abs(cnt - 30), -Woff - sc + 2 * sc * cnt / 60);
      delay(10);
    }
  }
  for (int cnt = 0; cnt <= 60; cnt++)
  {
    actuate(3, -Loff , -Hoff + 60 - 2 * abs(cnt - 30), Woff + 2 * sc * cnt / 60);
    delay(10);
  }
  for (int cnt = 0; cnt <= 30; cnt++)
  {
    actuate(1, Loff, -Hoff, Woff - sc * cnt / 30);
    actuate(3, -Loff, -Hoff, Woff + sc  - sc * cnt / 30);
    actuate(0, Loff , -Hoff, -Woff - sc * cnt / 30);
    actuate(2, Loff , -Hoff, -Woff + sc - sc * cnt / 30);
    delay(5);
  }
  for (int cnt = 0; cnt <= 60; cnt++)
  {
    actuate(0, Loff , -Hoff + 60 - 2 * abs(cnt - 30), -Woff - sc + 2 * sc * cnt / 60);
    delay(10);
  }
  for (int cnt = 0; cnt <= 60;  cnt++)
  {
    actuate(1, Loff , -Hoff + 60 - 2 * abs(cnt - 30), Woff - sc + 2 * sc * cnt / 60);
    delay(10);
  }
  for (int cnt = 0; cnt <= 30; cnt++)
  {
    actuate(1, Loff , -Hoff, Woff + sc  - sc * cnt / 30);
    actuate(3, -Loff  , -Hoff, Woff - sc * cnt / 30);
    actuate(0, Loff , -Hoff, -Woff + sc - sc * cnt / 30);
    actuate(2, -Loff , -Hoff, -Woff - sc * cnt / 30);
    delay(5);
  }
  for (int cnt = 0; cnt <= 30; cnt++)
  {
    actuate(2, -Loff, -Hoff + 30 - 2 * abs(cnt - 15), -Woff - sc + sc * cnt / 30);
    delay(10);
  }
  for (int cnt = 0; cnt <= 30; cnt++)
  {
    actuate(3, -Loff , -Hoff + 30 - 2 * abs(cnt - 15), Woff - sc + sc * cnt / 30);
    delay(10);
  }
}





///////////////////////////////////////

///Trot///
/*
  void trot(int steps)
  {
  int k = (steps - 2) / 2;
  for (int t = 0; t <= 30; t++)
  {
    actuate(1, Loff + s * t / 30, -Hoff + 30 - 2 * abs(t - 15), Woff);
    actuate(2, -Loff + s * t / 30, -Hoff + 30 - 2 * abs(t - 15), -Woff);
    actuate(0, Loff - s * t / 30, -Hoff, -Woff);
    actuate(3,  -Loff - s * t / 30, -Hoff, Woff);
    delay(30);
  }
  for (int i = 1; i <= k; i++)
  {
    for (int t = 0; t <= 60; t++)
    {
      actuate(1, Loff + s - s * t / 30, -Hoff, Woff);
      actuate(2, -Loff + s - s * t / 30, -Hoff, -Woff);
      actuate(0, Loff - s + s * t / 30, - Hoff + 60 - 2 * abs(t - 30), -Woff);
      actuate(3, -Loff - s + s * t / 3, - Hoff + 60 - 2 * abs(t - 30), Woff);
      delay(30);
    }
    for (int t = 0; t <= 60; t++)
    {
      actuate(0, Loff + s - s * t / 30, -Hoff, -Woff);
      actuate(3, -Loff + s - s * t / 30, -Hoff, Woff);
      actuate(1, Loff  - s + s * t / 30,  - Hoff + 60 - 2 * abs(t - 30), Woff);
      actuate(2, -Loff  - s + s * t / 30, - Hoff + 60 - 2 * abs(t - 30), -Woff);
      delay(30);
    }
  }
  for (int t = 0; t <= 30; t++)
  {
    actuate(1, Loff + s - s * t / 30, -Hoff, Woff);
    actuate(2, -Loff + s - s * t / 30, -Hoff, -Woff);
    actuate(0, Loff - s + s * t / 30, -Hoff + 30 - 2 * abs(t - 15), -Woff);
    actuate(3, -Loff - s + s * t / 30, -Hoff + 30 - 2 * abs(t - 15), Woff);
    delay(30);
  }
  }
*/

//////spin/////////////


void spin()
{
  float theta;

  while (getOrder() == '4' || getOrder() == '5')
  {
    if (getOrder == '4')
    {
      theta = -pi / 6;
    }

    else if (getOrder == '5')
    {
      theta = pi / 6;
    }
    //step0
    for (int t = 0; t <= 60; t++)
    {
      lff = quat_rot(y_axis, lff0, theta * 3 / 4 * t / 60);
      lbf = quat_rot(y_axis, lbf0, -theta / 4 * t / 60);
      rff = quat_rot(y_axis, rff0, -theta / 4 * t / 60);
      rbf = quat_rot(y_axis, rbf0, -theta / 4 * t / 60);
      struct point lfd = vec_sub(lff, lfs0);
      struct point lbd = vec_sub(lbf, lbs0);
      struct point rfd = vec_sub(rff, rfs0);
      struct point rbd = vec_sub(rbf, rbs0);
      actuate(0, lfd.x, lfd.y + 30 - 3 * abs(t / 3 - 10), lfd.z);
      actuate(1, rfd.x + 20 - 2 * abs(t / 3 - 10), rfd.y, rfd.z - 20 + 2 * abs(t / 3 - 10));
      actuate(2, lbd.x + 20 - 2 * abs(t / 3 - 10), lbd.y, lbd.z - 20 + 2 * abs(t / 3 - 10));
      actuate(3, rbd.x + 20 - 2 * abs(t / 3 - 10), rbd.y , rbd.z - 20 + 2 * abs(t / 3 - 10));
      delay(5);
    }
    //step3
    for (int t = 0; t <= 60; t++)
    {
      lff = quat_rot(y_axis, lff0, theta * 3 / 4 - theta / 4 * t / 60);
      lbf = quat_rot(y_axis, lbf0, -theta / 4 - theta / 4 * t / 60);
      rff = quat_rot(y_axis, rff0, -theta / 4 - theta / 4 * t / 60);
      rbf = quat_rot(y_axis, rbf0, -theta / 4 + theta * 3 / 4 * t / 60);
      struct point lfd = vec_sub(lff, lfs0);
      struct point lbd = vec_sub(lbf, lbs0);
      struct point rfd = vec_sub(rff, rfs0);
      struct point rbd = vec_sub(rbf, rbs0);
      actuate(0, lfd.x - 20 + 2 * abs(t / 3 - 10), lfd.y, lfd.z + 20 - 2 * abs(t / 3 - 10));
      actuate(1, rfd.x - 20 + 2 * abs(t / 3 - 10), rfd.y, rfd.z + 20 - 2 * abs(t / 3 - 10));
      actuate(2, lbd.x - 20 + 2 * abs(t / 3 - 10), lbd.y, lbd.z + 20 - 2 * abs(t / 3 - 10));
      actuate(3, rbd.x, rbd.y + 30 - 3 * abs(t / 3 - 10) , rbd.z);
      delay(5);
    }
    //step 1
    for (int t = 0; t <= 60; t++)
    {
      lff = quat_rot(y_axis, lff0, theta / 2 - theta / 4 * t / 60);
      lbf = quat_rot(y_axis, lbf0, -theta / 2 - theta / 4 * t / 60);
      rff = quat_rot(y_axis, rff0, -theta / 2 + theta * 3 / 4 * t / 60);
      rbf = quat_rot(y_axis, rbf0, theta / 2 - theta / 4 * t / 60);
      struct point lfd = vec_sub(lff, lfs0);
      struct point lbd = vec_sub(lbf, lbs0);
      struct point rfd = vec_sub(rff, rfs0);
      struct point rbd = vec_sub(rbf, rbs0);
      actuate(0, lfd.x + 20 - 2 * abs(t / 3 - 10), lfd.y, lfd.z + 20 - 2 * abs(t / 3 - 10));
      actuate(1, rfd.x, rfd.y + 30 - 3 * abs(t / 3 - 10), rfd.z);
      actuate(2, lbd.x + 20 - 2 * abs(t / 3 - 10), lbd.y , lbd.z + 20 - 2 * abs(t / 3 - 10));
      actuate(3, rbd.x + 20 - 2 * abs(t / 3 - 10), rbd.y, rbd.z + 20 - 2 * abs(t / 3 - 10));
      delay(5);
    }
    //step 2
    for (int t = 0; t <= 60; t++)
    {
      lff = quat_rot(y_axis, lff0, theta / 4 - theta / 4 * t / 60);
      lbf = quat_rot(y_axis, lbf0, -theta * 3 / 4 + theta * 3 / 4 * t / 60);
      rff = quat_rot(y_axis, rff0, theta / 4 - theta / 4 * t / 60);
      rbf = quat_rot(y_axis, rbf0, theta / 4 - theta / 4 * t / 60);
      struct point lfd = vec_sub(lff, lfs0);
      struct point lbd = vec_sub(lbf, lbs0);
      struct point rfd = vec_sub(rff, rfs0);
      struct point rbd = vec_sub(rbf, rbs0);
      actuate(0, lfd.x - 20 + 2 * abs(t / 3 - 10), lfd.y, lfd.z - 20 + 2 * abs(t / 3 - 10));
      actuate(1, rfd.x - 20 + 2 * abs(t / 3 - 10) , rfd.y , rfd.z - 20 + 2 * abs(t / 3 - 10));
      actuate(2, lbd.x , lbd.y + 30 - 3 * abs(t / 3 - 10) , lbd.z);
      actuate(3, rbd.x - 20 + 2 * abs(t / 3 - 10), rbd.y, rbd.z - 20 + 2 * abs(t / 3 - 10));
      delay(5);
    }
  }
}



/*
  void hexspin(float theta)
  {
  theta = theta * pi / 180;
  if (theta >= -pi / 2 && theta <= pi / 2)
  {
    //code1
    //step1
    for (int t = 0; t <= 60; t++)
    {
      struct point lff = quat_rot(y_axis, lff0, theta / 2 * t / 60);
      struct point lbf = quat_rot(y_axis, lbf0, -theta / 2 * t / 60);
      struct point rff = quat_rot(y_axis, rff0, -theta / 2 * t / 60);
      struct point rbf = quat_rot(y_axis, rbf0, theta / 2 * t / 60);
      struct point lfd = vec_sub(lff, lfs0);
      struct point lbd = vec_sub(lbf, lbs0);
      struct point rfd = vec_sub(rff, rfs0);
      struct point rbd = vec_sub(rbf, rbs0);
      actuate(0, lfd.x, lfd.y + 20 - 2 * abs(t / 3 - 10), lfd.z);
      actuate(1, rfd.x, rfd.y, rfd.z);
      actuate(2, lbd.x, lbd.y, lbd.z);
      actuate(3, rbd.x, rbd.y + 20 - 2 * abs(t / 3 - 10), rbd.z);
      delay(3);
    }
    //step 2
    for (int t = -60; t <= 0; t++)
    {
      struct point lff = quat_rot(y_axis, lff0, -theta / 2 * t / 60);
      struct point lbf = quat_rot(y_axis, lbf0, theta / 2 * t / 60);
      struct point rff = quat_rot(y_axis, rff0, theta / 2 * t / 60);
      struct point rbf = quat_rot(y_axis, rbf0, -theta / 2 * t / 60);
      struct point lfd = vec_sub(lff, lfs0);
      struct point lbd = vec_sub(lbf, lbs0);
      struct point rfd = vec_sub(rff, rfs0);
      struct point rbd = vec_sub(rbf, rbs0);
      actuate(0, lfd.x, lfd.y, lfd.z);
      actuate(1, rfd.x, rfd.y + 20 - 2 * abs(-t / 3 - 10), rfd.z);
      actuate(2, lbd.x, lbd.y + 20 - 2 * abs(-t / 3 - 10), lbd.z);
      actuate(3, rbd.x, rbd.y, rbd.z);
      delay(3);
    }
  }
  if (theta >= pi / 2 || theta <= -pi / 2)
  {
    theta = theta / 2;
    for (int i = 1; i <= 2; i++)
    {
      //step1
      for (int t = 0; t <= 60; t++)
      {
        struct point lff = quat_rot(y_axis, lff0, theta / 2 * t / 60);
        struct point lbf = quat_rot(y_axis, lbf0, -theta / 2 * t / 60);
        struct point rff = quat_rot(y_axis, rff0, -theta / 2 * t / 60);
        struct point rbf = quat_rot(y_axis, rbf0, theta / 2 * t / 60);
        struct point lfd = vec_sub(lff, lfs0);
        struct point lbd = vec_sub(lbf, lbs0);
        struct point rfd = vec_sub(rff, rfs0);
        struct point rbd = vec_sub(rbf, rbs0);
        actuate(0, lfd.x, lfd.y + 20 - 2 * abs(t / 3 - 10), lfd.z);
        actuate(1, rfd.x, rfd.y, rfd.z);
        actuate(2, lbd.x, lbd.y, lbd.z);
        actuate(3, rbd.x, rbd.y + 20 - 2 * abs(t / 3 - 10), rbd.z);
        delay(3);
      }
      //step2
      for (int t = -60; t <= 0; t++)
      {
        struct point lff = quat_rot(y_axis, lff0, -theta / 2 * t / 60);
        struct point lbf = quat_rot(y_axis, lbf0, theta / 2 * t / 60);
        struct point rff = quat_rot(y_axis, rff0, theta / 2 * t / 60);
        struct point rbf = quat_rot(y_axis, rbf0, -theta / 2 * t / 60);
        struct point lfd = vec_sub(lff, lfs0);
        struct point lbd = vec_sub(lbf, lbs0);
        struct point rfd = vec_sub(rff, rfs0);
        struct point rbd = vec_sub(rbf, rbs0);
        actuate(0, lfd.x, lfd.y, lfd.z);
        actuate(1, rfd.x, rfd.y + 20 - 2 * abs(-t / 3 - 10), rfd.z);
        actuate(2, lbd.x, lbd.y + 20 - 2 * abs(-t / 3 - 10), lbd.z);
        actuate(3, rbd.x, rbd.y, rbd.z);
        delay(3);
      }
    }
  }
  }
*/

///////////////////////////////////



void roll(float angle)
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


void pitch(float angle)
{
  angle = angle * pi / 180;
  for (int t = 0; t <= 60; t++)
  {
    stat_rot(z_axis, t * angle / 60);
    delay(5);
  }
  for (int t = 60; t >= 0; t--)
  {
    stat_rot(z_axis, t * angle / 60);
    delay(5);
  }
}

void yaw(float angle)
{
  angle = angle * pi / 180;
  for (int t = 0; t <= 60; t++)
  {
    stat_rot(y_axis, t * angle / 60);
    delay(5);
  }
  for (int t = 60; t >= 0; t--)
  {
    stat_rot(y_axis, t * angle / 60);
    delay(5);
  }
}


void actuate(int a, float x, float y, float z)
{
  int temp = z;
  z = x;
  x = temp;
  if (a == 0)
  {
    z = -z;
    float r = sqrt(x * x + z * z) - Lc;
    float thetac = atan(x / z);
    float thetaf = acos((r * r + y * y + Lf * Lf - Lt * Lt) / (2 * Lf * sqrt(r * r + y * y))) + atan(y / r);
    float thetat = acos((Lf * Lf + Lt * Lt - r * r - y * y) / (2 * Lf * Lt)) - pi / 2;
    thetac = mapping(thetac, -pi / 2, pi / 2, 0, 480);
    thetaf = mapping(thetaf, -pi / 2, pi / 2, 0, 480);
    thetat = mapping(thetat, -pi / 2, pi / 2, 0, 480);
    HCPCA9685.Servo(4 * a + 0, thetac);
    HCPCA9685.Servo(4 * a + 1, thetaf);
    HCPCA9685.Servo(4 * a + 2, thetat);
  }
  if (a == 1)
  {
    float r = sqrt(x * x + z * z) - Lc;
    float thetac = atan(x / z);
    float thetaf = acos((r * r + y * y + Lf * Lf - Lt * Lt) / (2 * Lf * sqrt(r * r + y * y))) + atan(y / r);
    float thetat = acos((Lf * Lf + Lt * Lt - r * r - y * y) / (2 * Lf * Lt)) - pi / 2;
    thetac = mapping(thetac, -pi / 2, pi / 2, 480, 0);
    thetaf = mapping(thetaf, -pi / 2, pi / 2, 480, 0);
    thetat = mapping(thetat, -pi / 2, pi / 2, 480, 0);
    HCPCA9685.Servo(4 * a + 0, thetac);
    HCPCA9685.Servo(4 * a + 1, thetaf);
    HCPCA9685.Servo(4 * a + 2, thetat);
  }
  if (a == 2)
  {
    x = -x;
    z = -z;
    float r = sqrt(x * x + z * z) - Lc;
    float thetac = atan(x / z);
    float thetaf = acos((r * r + y * y + Lf * Lf - Lt * Lt) / (2 * Lf * sqrt(r * r + y * y))) + atan(y / r);
    float thetat = acos((Lf * Lf + Lt * Lt - r * r - y * y) / (2 * Lf * Lt)) - pi / 2;
    thetac = mapping(thetac, -pi / 2, pi / 2, 480, 0);
    thetaf = mapping(thetaf, -pi / 2, pi / 2, 480, 0);
    thetat = mapping(thetat, -pi / 2, pi / 2, 480, 0);
    HCPCA9685.Servo(4 * a + 0, thetac);
    HCPCA9685.Servo(4 * a + 1, thetaf);
    HCPCA9685.Servo(4 * a + 2, thetat);
  }
  if (a == 3)
  {
    x = -x;
    float r = sqrt(x * x + z * z) - Lc;
    float thetac = atan(x / z);
    float thetaf = acos((r * r + y * y + Lf * Lf - Lt * Lt) / (2 * Lf * sqrt(r * r + y * y))) + atan(y / r);
    float thetat = acos((Lf * Lf + Lt * Lt - r * r - y * y) / (2 * Lf * Lt)) - pi / 2;
    thetac = mapping(thetac, -pi / 2, pi / 2, 0, 480);
    thetaf = mapping(thetaf, -pi / 2, pi / 2, 0, 480);
    thetat = mapping(thetat, -pi / 2, pi / 2, 0, 480);
    HCPCA9685.Servo(4 * a + 0, thetac);
    HCPCA9685.Servo(4 * a + 1, thetaf);
    HCPCA9685.Servo(4 * a + 2, thetat);
  }
}

float mapping(float val, float fromLow, float fromHigh, float toLow, float toHigh)
{
  float scale = (toHigh - toLow) / (fromHigh - fromLow);
  return toLow + (val - fromLow) * scale;
}
