/***********PID************/
#include <PID_v2.h>


double kp = 6; //To be Tuned
double ki = 0; //To be Tuned
double kd = 0; //To be Tuned
double filtered_output = 0.0;
double setpoint, input, output; //PID Variables
double output_mod;
double output_prev = 0.0;
double error;
PID_v2 pid(kp, ki, kd, DIRECT); //PID Setup

/**************************/

//We Do not have Radio:

///********** radio **********/
//#include <nRF24L01.h>
//#include <SPI.h>
//#include <RF24.h>
//#include <nRF24L01.h>
//
//RF24 radio(10, 4);    // CE, CSN
//const byte address[6] = "00001";
//float angleV = 0, turnV = 0; // values from remote
//float controlValues[2]; // array to receive both data values
///***************************/


/********** L298N **********/
#include <L298N.h>

//Pin Definition:
const unsigned int EN_A = 3;
const unsigned int IN1_A = 5;
const unsigned int IN2_A = 6;

const unsigned int IN1_B = 7;
const unsigned int IN2_B = 8;
const unsigned int EN_B = 9;

// Create motor instances
L298N rightMotor(EN_A, IN1_A, IN2_A);
L298N leftMotor(EN_B, IN1_B, IN2_B);

// motor speeds
int speedLeft = 0;
int speedRight = 0;

/****************************/
/************MPU************/

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>


MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
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

float pitch;
long velocity;

float trimPot;
float trimAngle;

int IMUdataReady = 0;
volatile bool mpuInterrupt = false;

/****Interrupt Handling****/
void dmpDataReady() {
    mpuInterrupt = true;
}
/**********Setup***********/

/*********Interrupt*******/

//// IMU interrupt service routine
//
//void dmpDataReady() {
//     IMUdataReady = 1;
//}        
//
//
//// function that actually read the angle when the flag is set by the ISR
//
//void readAngles()  {
//
//    mpuIntStatus = mpu.getIntStatus();
//    fifoCount = mpu.getFIFOCount();
//    
//    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
//        // reset so we can continue cleanly
//        mpu.resetFIFO();
//        Serial.println(F("FIFO overflow!"));
//     } 
//     
//     else if (mpuIntStatus & 0x02) {
//        // wait for correct available data length, should be a VERY short wait
//        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
//
//        // read a packet from FIFO
//        mpu.getFIFOBytes(fifoBuffer, packetSize);
//        
//        // track FIFO count here in case there is > 1 packet available
//        // (this lets us immediately read more without waiting for an interrupt)
//        fifoCount -= packetSize;
//
//        mpu.dmpGetQuaternion(&q, fifoBuffer);
//        mpu.dmpGetGravity(&gravity, &q);
//        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//
//        IMUdataReady = 0;
//        //count = count + 1;
//    }
//}

/*************************/

/*************Filtered_output***********/
double lowPassFilter(double input, double outputPrev) {
    // Low-pass filter equation
    double output = 0.95 * input + (1 - 0.95) * outputPrev;

    return output;
}


void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(9600);

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
//    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
//    while (Serial.available() && Serial.read()); // empty buffer
//    while (!Serial.available());                 // wait for data
//    while (Serial.available() && Serial.read()); // empty buffer again


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
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
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
    
    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(-255,255);
    pid.SetSampleTime(10);
    
    pinMode(LED_PIN, OUTPUT);

    pid.Start(input,0,0);

//    radio.begin();
//    radio.openReadingPipe(0, address);
//    radio.setPALevel(RF24_PA_MIN);
//    radio.startListening();

}

void loop() {
  // only read remote data if remote is available, otherwise balance normally
//  if (radio.available()) {
//    radio.read(&controlValues, sizeof(controlValues)); // read values of array
//    
//    angleV = controlValues[0]; // assign array values to control variables
//    turnV = controlValues[1];
//  } 
  
//  trimPot = analogRead(A3)*-1; // read dial on robot and *-1 for direction
//
//  trimAngle = (trimPot/100) + 5 + angleV; // adjust to degrees

//  if (IMUdataReady == 1) {
//    readAngles();
//  }

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
//        #ifdef OUTPUT_READABLE_QUATERNION
//            // display quaternion values in easy matrix form: w x y z
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            Serial.print("quat\t");
//            Serial.print(q.w);
//            Serial.print("\t");
//            Serial.print(q.x);
//            Serial.print("\t");
//            Serial.print(q.y);
//            Serial.print("\t");
//            Serial.println(q.z);
//        #endif

//        #ifdef OUTPUT_READABLE_EULER
//            // display Euler angles in degrees
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            mpu.dmpGetEuler(euler, &q);
//            Serial.print("euler\t");
//            Serial.print(euler[0] * 180/M_PI);
//            Serial.print("\t");
//            Serial.print(euler[1] * 180/M_PI);
//            Serial.print("\t");
//            Serial.println(euler[2] * 180/M_PI);
//        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//            Serial.print("ypr\t");
//            Serial.print(ypr[0] * 180/M_PI);
//            Serial.print("\t");
//            Serial.print(ypr[1] * 180/M_PI);
//            Serial.print("\t");
//            Serial.println(ypr[2] * 180/M_PI);
        #endif

//        #ifdef OUTPUT_READABLE_REALACCEL
//            // display real acceleration, adjusted to remove gravity
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            mpu.dmpGetAccel(&aa, fifoBuffer);
//            mpu.dmpGetGravity(&gravity, &q);
//            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//            Serial.print("areal\t");
//            Serial.print(aaReal.x);
//            Serial.print("\t");
//            Serial.print(aaReal.y);
//            Serial.print("\t");
//            Serial.println(aaReal.z);
//        #endif

//        #ifdef OUTPUT_READABLE_WORLDACCEL
//            // display initial world-frame acceleration, adjusted to remove gravity
//            // and rotated based on known orientation from quaternion
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            mpu.dmpGetAccel(&aa, fifoBuffer);
//            mpu.dmpGetGravity(&gravity, &q);
//            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
//            Serial.print("aworld\t");
//            Serial.print(aaWorld.x);
//            Serial.print("\t");
//            Serial.print(aaWorld.y);
//            Serial.print("\t");
//            Serial.println(aaWorld.z);
//        #endif
    
//        #ifdef OUTPUT_TEAPOT
//            // display quaternion values in InvenSense Teapot demo format:
//            teapotPacket[2] = fifoBuffer[0];
//            teapotPacket[3] = fifoBuffer[1];
//            teapotPacket[4] = fifoBuffer[4];
//            teapotPacket[5] = fifoBuffer[5];
//            teapotPacket[6] = fifoBuffer[8];
//            teapotPacket[7] = fifoBuffer[9];
//            teapotPacket[8] = fifoBuffer[12];
//            teapotPacket[9] = fifoBuffer[13];
//            Serial.write(teapotPacket, 14);
//            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
//        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }

  pitch = (ypr[1] * 180/M_PI); // adjust to degrees

//  if (abs(turnV) < 15) { // turnV threshold
//    turnV = 0;
//  }

//  if (abs(angleV) < .17) { // angleV threshold
//    angleV = 0;
//  }

  // PID vars
  setpoint = 0.0;
//  setpoint = trimAngle + angleV; 
  input = pitch;

//  output = lowPassFilter(input,output_prev);
  output = input;
  const double output = pid.Run(input);
//  pid.Compute();

//  error = setpoint - output;
//  error_d = error - error_old;
//  error
//  output = kp*error;


  // set motor speed with adjusted turn values
//  speedLeft = output -  turnV;
    if (output > 0 && output <35){
      output_mod = 35;
    }
//  speedLeft = ;

    else if (output > -35 && output < 0){
      output_mod = 35;
    }

    else 
      output_mod = output;
    
  speedLeft = output_mod;
//  speedLeft = output;
//  speedRight = output + turnV;
//  speedRight = output;
  speedRight = output_mod;
  
  
  if (pitch > 25 || pitch < -25) { // angle threshold
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0); 
  } else {
    leftMotor.setSpeed(abs(speedLeft));
    rightMotor.setSpeed(abs(speedRight));
  }

  // move motors
  if (speedLeft > 0) { 
    leftMotor.forward();
  } else {
    leftMotor.backward();
  }
  
  if (speedRight > 0) { 
    rightMotor.forward();
  } else {
    rightMotor.backward();
  }
  
  // print some control info
  Serial.print("pitch: ");
  Serial.print(ypr[1] * 180/M_PI);
  Serial.print(" output: ");
  Serial.print(output);
  Serial.print(" output_mod: ");
  Serial.print(output_mod);
  Serial.print(" filtered output: ");
  Serial.print(filtered_output);
  Serial.println();
  
//  Serial.print(" , trimPot: ");
//  Serial.print(trimPot);
//  Serial.print(" , angleV: ");
//  Serial.print(angleV);
//  Serial.print(" , turnV: ");
//  Serial.println(turnV);
output_prev = output;

}
