// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
 // for both classes must be in the include path of your project
#include "I2Cdev.h"
#include <avr/pgmspace.h>
#if __CLION_IDE__
#include "MPU6050/MPU6050_6Axis_MotionApps20.h"
#include "MPU6050/MPU6050.h" // not necessary if using MotionApps include file
#else
#include "MPU6050_6Axis_MotionApps20.h"
#endif
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif


#include <Servo.h>
#define MAX_DELTATIME 20000 //used in main program loop 
#define DIR_PIN 4 //motor drive pin 
#define SERVO_PIN 10 //servo pin

#define ENCODER_PIN 3 //motor encoder pin
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
//#define TEST_COMMUNICATION_LATENCY
#define SERVO_FEEDBACK_MOTOR_PIN 0 //servo feedback pin 

#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>


const char STEERING_TOPIC[]  PROGMEM  = { "control/steering" };
const char ACCEL_TOPIC[]  PROGMEM  = { "control/accel" };

//const char FLAG_TOPIC[]  PROGMEM  = { "controller_flag" };

//IMU info
const char TWIST_TOPIC[]  PROGMEM  = { "twist" };
const char POSE_TOPIC[] PROGMEM = { "pose" };
const char STEERING_ANGLE_TOPIC[]  PROGMEM  = { "steering_feedback" };
const char MOTOR_RPM_TOPIC[]  PROGMEM  = { "motor_rpm_feedback" };
const char WHEEL_RPM_TOPIC[]  PROGMEM  = { "wheel_rpm_feedback" };



ros::NodeHandle nh;


std_msgs::Float32 motor_rpm;
std_msgs::Float32 wheel_rpm;
std_msgs::Float32 steering_msg;
std_msgs::Float32 accel_msg;
geometry_msgs::Twist twist_msg;
geometry_msgs::Pose pose_msg;


// Acceleration & Steering control parameters and variables:

float wheel_radius  = 0.0305;
float vel           = 0;
float vel_lin_vel           = 0;
float Wheel_RPM = 0;

int counter = 0;
ros::Publisher pubTwist(FCAST(TWIST_TOPIC), &twist_msg);
ros::Publisher pubSteeringAngle(FCAST(STEERING_ANGLE_TOPIC), &steering_msg);
ros::Publisher pubRPM(FCAST(MOTOR_RPM_TOPIC), &motor_rpm);
ros::Publisher pubWHEELRPM(FCAST(WHEEL_RPM_TOPIC), &wheel_rpm);

ros::Publisher pubPose(FCAST(POSE_TOPIC), &pose_msg);

void onSteeringCommand(const std_msgs::Float32 &cmd_msg);
void onAccelCommand(const std_msgs::Float32 &cmd_msg);
//void onFlagCommand(const std_msgs::Bool &cmd_msg);

ros::Subscriber<std_msgs::Float32> steeringCommand(FCAST(STEERING_TOPIC), onSteeringCommand);
ros::Subscriber<std_msgs::Float32> accelCommand(FCAST(ACCEL_TOPIC), onAccelCommand);
//ros::Subscriber<std_msgs::Bool> flagCommand("controller_flag", onFlagCommand);


#ifdef TEST_COMMUNICATION_LATENCY
#include <std_msgs/Bool.h>
std_msgs::Bool resp_msg;

ros::Publisher pubResponse("resp", &resp_msg);
void onLatency(const std_msgs::Bool &cmd_msg) {
    resp_msg.data = true;
    pubResponse.publish(&resp_msg);
}
ros::Subscriber<std_msgs::Bool> requestCommand("req", onLatency);
#endif



// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high


// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

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

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
Servo myservo; // create servo object to control a servo
int servo_pw = 1400;    // variable to set the angle of servo motor
int servo_home = 0;
int last_pw = 0;
float Motor_RPM = 0;
bool servo_initialized = false;
volatile unsigned long T1Ovs2;
volatile int16_t encoder_counter;              //CAPTURE FLAG
volatile uint16_t ticks_counter;
volatile int16_t last_encoder_counter;
volatile unsigned long deltatime = 0;
volatile boolean first_rising = true;


int8_t direction_motor = 1;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint8_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 gyro;        // [x, y, z]            angular velocity vector
VectorFloat gravity;    // [x, y, z]            gravity vector
//float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

int first_iteration = 1;
float yaw_offset;
int warm_up_counter = 0;





// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void StartTimer2(void) {
    pinMode(11, OUTPUT); //976.5625Hz
    TCNT2 = 0;
    TIFR2 = 0x00;
    TIMSK2 = TIMSK2 | 0x01;
    TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
    TCCR2B = _BV(CS22);
    TCCR2B = (TCCR2B & 0b11111000) | 0x02;//62500HZ= 16MHz/1024/2=7812
    sei();
}

ISR(TIMER2_OVF_vect) {
    TIFR2 = 0x00;
    T1Ovs2++;//INCREMENTING OVERFLOW COUNTER
}

void encoder() {
    cli();
    if (!first_rising) {
        deltatime = T1Ovs2 * 25 + T1Ovs2 * 5 / 10 + TCNT2 / 10;// prevent overflow of integer number!
    }

    T1Ovs2 = 0;         //SAVING FIRST OVERFLOW COUNTER
    TCNT2 = 0;
    first_rising = false;
    encoder_counter++;
    ticks_counter++;
    sei();
}




// ================================================================
// ===               SUBSCRIBERS                                ===
// ================================================================
void onSteeringCommand(const std_msgs::Float32 &cmd_msg) {
    if ((cmd_msg.data <= 0.35) && (cmd_msg.data >= -0.35 )) {
        // scale it to use it with the servo mapping

        // transform angle command from rad to deg 

        float angle_command = cmd_msg.data * (180/PI); //degree conversion

        servo_pw = (int)(-19.3*angle_command + 1402.6064359154816); //
//        0.1 1408.97/ 0.2 1407.04 // 0.3 1405.11
        if (!servo_initialized) {
            // attaches the servo on pin 9 to the servo object
            myservo.attach(SERVO_PIN);
            servo_initialized = true;
            myservo.writeMicroseconds(servo_home);  // https://arduinogetstarted.com/reference/library/servo-writemicroseconds
        }

        if (last_pw!=servo_pw) {
            myservo.writeMicroseconds(servo_pw);
        }

        last_pw = servo_pw;
    }
}



void onAccelCommand(const std_msgs::Float32 &cmd_msg) {

    float vel = cmd_msg.data;

 //   if(accel != 0.0){
//    int16_t motor_/val = vel;// = (int)(rpm*2 - 22.5);
      

    // Deadzone defined in the region -0.05 to 0.05 DS: -40 to 40
    uint8_t servo_val = (uint8_t) 226.31578947368422*abs(vel) + 28.684210526315788;

    // if speed is set to 0 we keep the old direction
    // and just do nothing but set the val
    // else the speed direction might get inversed
    if (vel < 0) {
        digitalWrite(DIR_PIN, HIGH);
        direction_motor = -1;
    } else if (vel> 0) {
        digitalWrite(DIR_PIN, LOW);
        direction_motor = 1;
    }

    if (servo_val < 40) {
        servo_val = 10;
    }

    OCR2A = servo_val;

}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
    nh.getHardware()->setBaud(500000); // ### 500000 throws Lost sync with device, restarting...
    nh.initNode();
    nh.advertise(pubTwist);
    nh.advertise(pubRPM);
    nh.advertise(pubWHEELRPM);

    nh.advertise(pubSteeringAngle);
    nh.advertise(pubPose);
    //nh.advertise(pubAccel);

   // nh.subscribe(ledCommand);
    nh.subscribe(steeringCommand);
    nh.subscribe(accelCommand);
    //nh.subscribe(flagCommand);

    
#ifdef TEST_COMMUNICATION_LATENCY
    nh.subscribe(requestCommand);
    nh.advertise(pubResponse);
#endif

    // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    // initialize device
    nh.logerror(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    nh.loginfo(F("Testing device connections..."));
    nh.loginfo(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    /*mpu.setXAccelOffset(-3180);
    mpu.setYAccelOffset(-2813);
    mpu.setZAccelOffset(1103); // 1688 factory default for my test chip
    mpu.setXGyroOffset(197);
    mpu.setYGyroOffset(-41);
    mpu.setZGyroOffset(1);
    */

//    //EUGE calibration:
//    mpu.setXAccelOffset(-1001);
//    mpu.setYAccelOffset(-875);
//    mpu.setZAccelOffset(1847); // 1688 factory default for my test chip
//    mpu.setXGyroOffset(60);
//    mpu.setYGyroOffset(-66);
//    mpu.setZGyroOffset(58); 

//    //CARLOS calibration 31/07/2019:
    mpu.setXAccelOffset(-963);
    mpu.setYAccelOffset(871);
    mpu.setZAccelOffset(5742);
    mpu.setXGyroOffset(69);
    mpu.setYGyroOffset(-62);
    mpu.setZGyroOffset(62); 

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        nh.loginfo(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        nh.loginfo(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        nh.logerror(F("DMP Initialization failed (code"));
        nh.logerror("" + devStatus);
        nh.logerror(F(")"));
    }

    pinMode(ENCODER_PIN, INPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(ENCODER_PIN, INPUT_PULLUP);
    digitalWrite(ENCODER_PIN, HIGH);             //pull up
    StartTimer2();
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoder, RISING);

}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    //while (!mpuInterrupt && fifoCount < packetSize) {}

    if (mpuInterrupt || fifoCount >= packetSize) {

        // reset interrupt flag and get INT_STATUS byte
        mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();

        // get current FIFO count
        fifoCount = mpu.getFIFOCount();

        // check for overflow (this should never happen unless our code is too inefficient)
        if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
            // reset so we can continue cleanly
            mpu.resetFIFO();
            nh.logerror(F("FIFO overflow!"));

            // otherwise, check for DMP data ready interrupt (this should happen frequently)
        } else if (mpuIntStatus & 0x02) {
            // wait for correct available data length, should be a VERY short wait
            uint16_t fifoCount = mpu.getFIFOCount();
            if (fifoCount >= packetSize) {

                // read a packet from FIFO
                mpu.getFIFOBytes(fifoBuffer, packetSize);

                // track FIFO count here in case there is > 1 packet available
                // (this lets us immediately read more without waiting for an interrupt)
                fifoCount -= packetSize;

                #ifdef OUTPUT_READABLE_QUATERNION
                  //display quaternion values in easy matrix form: w x y z
                  mpu.dmpGetQuaternion(&q, fifoBuffer);
                #endif

                #ifdef OUTPUT_READABLE_EULER
                  //display Euler angles in degrees
                  //mpu.dmpGetQuaternion(&q, fifoBuffer);
                  //mpu.dmpGetEuler(euler, &q);
                #endif

                // display Euler angles in degrees
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                //mpu.dmpGetAccel(&aa, fifoBuffer);
                mpu.dmpGetGyro(&gyro, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);


                // Orientation info
                if ((warm_up_counter > 1000) && (first_iteration == 1)){
                  yaw_offset = ypr[0];
                  first_iteration = 0;
                }
                pose_msg.orientation.x = ypr[2];
                pose_msg.orientation.y = ypr[1];
                pose_msg.orientation.z = ypr[0];

//// Quaternion q;           // [w, x, y, z] 
                pubPose.publish(&pose_msg);

                // Angular velocity on the 3 axis in rad/sec
                twist_msg.angular.x=gyro.x*(M_PI/180); 
                twist_msg.angular.y=gyro.y*(M_PI/180);
                twist_msg.angular.z=gyro.z*(M_PI/180);

                //Serial.print("ang. velocity: \t");//yaw
                //Serial.println(gyro.z*(M_PI/180));
                twist_msg.linear.x = aaReal.x;
                twist_msg.linear.y = aaReal.y;
                twist_msg.linear.z = aaReal.z;
               
                pubTwist.publish(&twist_msg);


                Motor_RPM = 0;
                Wheel_RPM = 0;
                // if the motor has stopped we publish the speed when IMU data is ready because this runs at 100hz
                if (last_encoder_counter == encoder_counter) {

                    //Motor_RPM = 0;
                    // we received nothing so check if the motor has stopped
                    if (T1Ovs2 * 25 + T1Ovs2 * 5 / 10 + TCNT2 / 10 > MAX_DELTATIME) {
                        vel_lin_vel = 0.0;
                        first_rising = true;
                        deltatime = 0;
                    }

                }
                // we received data from the DC motor
                else {
                    if (deltatime != 0) {
                        //rad/second -> each tick is 0.005 ms: Arduino timer is 2Mhz , but counter divided by 10 in arduino! 6 lines per revolution!
                        
                        Motor_RPM = (M_PI / 3.0) / (deltatime * 0.005 * 0.001);
                        
                        Wheel_RPM  = 1.66219 * Motor_RPM + 0.69465; //OBTAINED FROM FITTING MODEL
//                        vel_lin_vel = (wheel_radius*2*M_PI*Wheel_RPM)/60;
                        //motor_rpm.data = vel_current;
                        //pubRPM.publish(&motor_rpm);                       
                    } 
                }


                motor_rpm.data = Motor_RPM* direction_motor;
                pubRPM.publish(&motor_rpm); 
                
                wheel_rpm.data = Wheel_RPM * direction_motor;
                pubWHEELRPM.publish(&wheel_rpm);   
                last_encoder_counter = encoder_counter;

                steering_msg.data = map(analogRead(SERVO_FEEDBACK_MOTOR_PIN), 0, 360, 0, 180);

                pubSteeringAngle.publish(&steering_msg);

                ticks_counter = 0;
            }
        }
    }
    
    warm_up_counter++; // Variable to control the calibration of yaw

    nh.spinOnce();
}
