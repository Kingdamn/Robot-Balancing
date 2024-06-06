#include <LMotorController.h>
#include <PID_v1.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define MIN_ABS_SPEED 20

MPU6050 mpu;

bool dmpReady = false;
uint8_t mpuIntStatus; 
uint8_t devStatus; 
uint16_t packetSize;
uint16_t fifoCount; 
uint8_t fifoBuffer[64]; 

// orientation/motion vars
Quaternion q; // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

//PID
double originalSetpoint = 181; 
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;


//Trial
double Kp = 35    ;   //60 Present 28    Perfect values as on now: 28, 2.2, 160 //27.5 (P=41 )
double Kd = 2;  //2.2 or 2.4future  1.4 to 1.8  also try upto 2.2 //2.2//2.5//1.8//1.6 (P=1.8)
double Ki = 200;   // 270 Past    80  to 140  ; too good 160 //160 //170-180   (P=250)


PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 0.55;  //0.65
double motorSpeedFactorRight = 0.5; //0.5
//MOTOR CONTROLLER
int ENA = 5;
int IN1 = 6;
int IN2 = 7;
int IN3 = 8;
int IN4 = 9;
int ENB = 10;



LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
mpuInterrupt = true;
}


void setup()
{
Serial.begin(9600); 
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
Wire.begin();
TWBR = 24; 
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
Fastwire::setup(400, true);
#endif

mpu.initialize();

devStatus = mpu.dmpInitialize();

// supply your own gyro offsets here, scaled for min sensitivity
mpu.setXGyroOffset(220);
mpu.setYGyroOffset(76);
mpu.setZGyroOffset(-85);
mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

// make sure it worked (returns 0 if so)
if (devStatus == 0)
{
// turn on the DMP, now that it's ready
mpu.setDMPEnabled(true);

// enable Arduino interrupt detection
attachInterrupt(0, dmpDataReady, RISING);
mpuIntStatus = mpu.getIntStatus();

// set our DMP Ready flag so the main loop() function knows it's okay to use it
dmpReady = true;

// get expected DMP packet size for later comparison
packetSize = mpu.dmpGetFIFOPacketSize();

//setup PID
pid.SetMode(AUTOMATIC);
pid.SetSampleTime(10);
pid.SetOutputLimits(-255, 255); 

}
else
{
Serial.print(F("DMP Initialization failed (code "));
Serial.print(devStatus);
Serial.println(F(")"));
}
}


void loop()
{
if (!dmpReady) return;

while (!mpuInterrupt && fifoCount < packetSize)
{
pid.Compute();
motorController.move(output, MIN_ABS_SPEED);

}

mpuInterrupt = false;
mpuIntStatus = mpu.getIntStatus();

fifoCount = mpu.getFIFOCount();

if ((mpuIntStatus & 0x10) || fifoCount == 1024)
{
mpu.resetFIFO();
Serial.println(F("FIFO overflow!"));

}
else if (mpuIntStatus & 0x02)
{

while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

mpu.getFIFOBytes(fifoBuffer, packetSize);

fifoCount -= packetSize;

mpu.dmpGetQuaternion(&q, fifoBuffer);
mpu.dmpGetGravity(&gravity, &q);
mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
input = ypr[1] * 180/M_PI + 180;
}
}