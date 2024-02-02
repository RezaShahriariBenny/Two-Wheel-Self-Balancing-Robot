/***********PID************/
#include <PID_v2.h>

double kp = 8; // To be Tuned
double ki = 0;  // To be Tuned
double kd = 0; // To be Tuned
double filtered_value = 0.0;
double setpoint, input, output; // PID Variables
double output_mod;
double output_prev = 0.0;
double error;
PID_v2 pid(kp, ki, kd, DIRECT); // PID Setup

/********** L298N **********/
#include <L298N.h>

// Pin Definition:
const unsigned int EN_A = 3;
const unsigned int IN1_A = 5;
const unsigned int IN2_A = 6;

const unsigned int IN1_B = 7;
const unsigned int IN2_B = 8;
const unsigned int EN_B = 9;

// Create motor instances
L298N rightMotor(EN_A, IN1_A, IN2_A);
L298N leftMotor(EN_B, IN1_B, IN2_B);

// Motor speeds
int speedLeft = 0;
int speedRight = 0;

/************MPU************/
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2 // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13       // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus;  // holds actual interrupt status byte from MPU
uint8_t devStatus;     // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;   // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;    // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Orientation/motion vars
Quaternion q;           // [w, x, y, z] quaternion container
VectorInt16 aa;         // [x, y, z] accel sensor measurements
VectorInt16 aaReal;     // [x, y, z] gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z] world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z] gravity vector
float euler[3];         // [psi, theta, phi] Euler angle container
float ypr[3];           // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

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
/*************************/

void initializeMPU() {
    // Join I2C bus
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(9600);

    // Initialize device
//    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // Verify connection
//    Serial.println(F("Testing device connections..."));
//    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // Load and configure the DMP
//    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // Supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // Make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // Turn on the DMP, now that it's ready
//        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // Enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
//        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // Set our DMP Ready flag so the main loop() function knows it's okay to use it
//        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // Get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void initializePID() {
    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(-255, 255);
    pid.SetSampleTime(10);
    pid.Start(input, 0, 0);
}

//*************Kalman Filter************
/***********Kalman Filter************/
//#include <Kalman.h>
//
//Kalman kalmanPitch(100, 50, 0.01, 0.0);  // Initialize Kalman filter
//
double lowPassFilter(double input, double outputPrev) {
    // Low-pass filter equation
    double output = 0.95 * input + (1 - 0.95) * outputPrev;

    return output;
}




void setup() {
    initializeMPU();
    initializePID();
}

void readIMUData() {
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // Display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            pitch = (ypr[1] * 180/M_PI); // Adjust to degrees
            setpoint = 0.0;
            input = pitch;

            //Filter pitch using lowpass filter
            filtered_value = lowPassFilter(input,output_prev);
        #endif
        // Blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}

void processPitch() {
    pitch = (ypr[1] * 180/M_PI); // Adjust to degrees
    setpoint = 0.0;
    input = pitch;
}

void runPIDController() {
    const double output = pid.Run(input);

    if (output > -10 && output < 10) {
        output_mod = 0;
    } else {
        output_mod = output;
    }

    speedLeft = output;
    speedRight = output;
}

void moveMotors() {
    if (pitch > 25 || pitch < -25) { // Angle threshold
        leftMotor.setSpeed(0);
        rightMotor.setSpeed(0);
    } else {
        leftMotor.setSpeed(abs(speedLeft));
        rightMotor.setSpeed(abs(speedRight));
    }

    // Move motors
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

    // Print some control info
    Serial.print("pitch: ");
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print(" modified_pitch: ");
    Serial.print(filtered_value);
    Serial.print(" output: ");
    Serial.print(output);
    Serial.print(" output_mod: ");
    Serial.print(output_mod);
    Serial.println();
}

void loop() {
    readIMUData();
//    processPitch();

    input = pitch;
    runPIDController();
    moveMotors();
    output_prev = output;
}
