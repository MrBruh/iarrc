

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include <Servo.h>

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
Servo escXL5;
Servo steering;
int escPin = 9;
int servoPin = 8;
float prev_y = 182;
float temp_y = 183;
float y_offset = 0;
float y_value = 181;
int same_count = 0;
int throwaway = 0;
float s_goal = 0;
float s_gain = -1.5;
float v_sensor = 0;
float v_goal = 0;
float v_gain = 0.5;
float starting_y = 0.0;
bool read_from_vision = false; 
bool gyro_stable = false;
bool debug_done = false;
bool is_positive;
bool is_y_positive;
int current_speed = 0;
int servo_pos = 85;
int p_value = 85;
byte input;
byte incomingByte;
byte prevByte;
float integerValue;

int random_offset = 0;
int serial_count = 0;
/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
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

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    Serial1.begin(9600);
    escXL5.attach(escPin);
    steering.attach(servoPin);
    
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer

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

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // set neutral speed for drive motor
    escXL5.writeMicroseconds(1480);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    //Serial.print(" a p b");
    // if programming failed, don't try to do anything
    //Serial.println("Begin");
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    //Serial.print(" interrupt");
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        }  
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    //Serial.print(" FIFO");
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
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
          /*  mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI); */
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

        //Serial.print("p b: ");
        //getting the yaw value (rotation)
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        temp_y = ypr[0]*180/M_PI;
        if(temp_y > 0){
            is_y_positive = true;
        } else {
            is_y_positive = false;
        }
        Serial.print(" ty:");
        Serial.print(temp_y);
        Serial.print(" ");

        //adjust for sensor dumbness once calibrated
        if(gyro_stable){
            //check if y is positive or negative globally
            if(y_offset == 0 && is_positive == true && temp_y < 0){
                is_positive = false;
            } else if(y_offset == 0 && is_positive == false && temp_y > 0){
                is_positive = true;
            }

            //check if dumb
            if(temp_y > 180){
                temp_y = 180;
            } else if(temp_y < -180){
                temp_y = -180;
            }

            //check if over range
            /*Serial.print(is_positive); 
            Serial.print(" ");
            Serial.print(is_y_positive);
            Serial.print(" ");
            */
            Serial.print(y_offset);
            Serial.print(" ");

        
            if(prev_y < 0 && temp_y > 0 && temp_y - prev_y > 120){
                Serial.print("negative to positive");
                Serial.print(" ");
                y_offset -= 360;
            } else if(prev_y > 0 && temp_y < 0 && prev_y - temp_y > 120){
                Serial.print("postive to negative");
                Serial.print(" ");
                y_offset += 360;
            }
            y_value = temp_y + y_offset;
        }

        //adjustments and stuff
        /*if(is_positive != is_y_positive){
            if(y_offset > 0){
                temp_y = float_map(y_value, -180.0, 0.0, 0.0, 180.0) + y_offset;
                y_value = temp_y;

            } else if(y_offset < 0){
                temp_y = float_map(y_value, 180.0, 0.0, 0.0, -180.0) + y_offset;
                y_value = temp_y;
            } 
        } else {
            y_value = y_value + y_offset;
        }
        */

        Serial.print(prev_y);
        Serial.print(" y_value:");
        Serial.println(y_value);

        //wait until gyro is stable
        if(gyro_stable == false){
          if(temp_y == prev_y){
            same_count++;
          }
          if(temp_y != prev_y){
            same_count = 0;
          }
          if(same_count >= 5){
            digitalWrite(LED_PIN, HIGH);
            Serial.println("done");
            gyro_stable = true;
          }
        }
        
        //run once after gyro stable
        if(gyro_stable == true && !debug_done){
            if(temp_y > 0){
                is_positive = true;
            } else {
                is_positive = false;
            }
          s_goal = temp_y;
          steering.write(80);
          //current_speed = run(escXL5, 2150, current_speed); //min 1750 

          debug_done = true;
        }

        //run forever after gyro stable 
        if(gyro_stable == true){
            if (Serial1.available()) {     // If anything comes in Serial1 (pins 0 & 1)
          
                input = Serial1.read();
                Serial.print( input);
                switch(input){
                case 's':
                {
                    //Serial.println( input);
                    char buffer[] = {' ',' ',' ',' ',' ',' ',' '}; // Receive up to 7 bytes
                    integerValue = 0;         // throw away previous integerValue
                    while(1) {            // force into a loop until 'n' is received
                    incomingByte = Serial1.read();
                    if (incomingByte == '\n') break;   // exit the while(1), we're done receiving
                    if (incomingByte == 255) continue;  // if no characters are in the buffer read() returns -1
                    if (incomingByte == 45) {
                        integerValue *= -1; 
                        continue;
                    }
                    //Serial.println(incomingByte);
                    integerValue *= 10;  // shift left 1 decimal place
                    // convert ASCII to integer, add, and shift left 1 decimal place
                    integerValue = ((incomingByte - 48) + integerValue);
                    }
                    s_goal += integerValue;
                    read_from_vision = false;
                    break;
                }
                case 'a':
                {
                    //Serial.println( input);
                    char buffer[] = {' ',' ',' ',' ',' ',' ',' '}; // Receive up to 7 bytes
                    integerValue = 0;         // throw away previous integerValue
                    while(1) {            // force into a loop until 'n' is received
                    incomingByte = Serial1.read();
                    if (incomingByte == '\n') break;   // exit the while(1), we're done receiving
                    if (incomingByte == 255) continue;  // if no characters are in the buffer read() returns -1
                    if (incomingByte == 45) {
                        integerValue *= -1; 
                        continue;
                    }
                    //Serial.println(incomingByte);
                    integerValue *= 10;  // shift left 1 decimal place
                    // convert ASCII to integer, add, and shift left 1 decimal place
                    integerValue = ((incomingByte - 48) + integerValue);
                    }
                    s_goal = integerValue;
                    read_from_vision = false;
                    break;
                }
                case 'f':
                {
                    //Serial.println( input);
                    char buffer[] = {' ',' ',' ',' ',' ',' ',' '}; // Receive up to 7 bytes
                    integerValue = 0;         // throw away previous integerValue
                    while(1) {            // force into a loop until 'n' is received
                    incomingByte = Serial1.read();
                    if (incomingByte == '\n') break;   // exit the while(1), we're done receiving
                    if (incomingByte == 255) continue;  // if no characters are in the buffer read() returns -1
                    if (incomingByte == 45) {
                        integerValue *= -1; 
                        continue;
                    }
                    //Serial.println(incomingByte);
                    integerValue *= 10;  // shift left 1 decimal place
                    // convert ASCII to integer, add, and shift left 1 decimal place
                    integerValue = ((incomingByte - 48) + integerValue);
                    }
                    current_speed = run(escXL5, map(integerValue, 0, 100, 1700, 2150), current_speed);
                    break;
                }   
                case 'b':
                    Serial.println("s");
                    brake(escXL5);
                    break;
                case 'r':
                    Serial.println("r");
                    read_from_vision = false;
                    break;
                case 'c':
                    Serial.println("c");
                    starting_y = y_value;
                    //s_goal = y_value;
                    break;
                case 'g':
                {
                    //Serial.println( input);
                    char buffer[] = {' ',' ',' ',' ',' ',' ',' '}; // Receive up to 7 bytes
                    integerValue = 0;         // throw away previous integerValue
                    while(1) {            // force into a loop until 'n' is received
                    incomingByte = Serial1.read();
                    if (incomingByte == '\n') break;   // exit the while(1), we're done receiving
                    if (incomingByte == 255) continue;  // if no characters are in the buffer read() returns -1
                    if (incomingByte == 45) {
                        integerValue *= -1; 
                        continue;
                    }
                    //Serial.println(incomingByte);
                    integerValue *= 10;  // shift left 1 decimal place
                    // convert ASCII to integer, add, and shift left 1 decimal place
                    integerValue = ((incomingByte - 48) + integerValue);
                    }
                    Serial.print( "v g" );
                    read_from_vision = true;
                    v_sensor = integerValue/10;
                    break;
                }
                case 'v':
                {
                    //Serial.println( input);
                    char buffer[] = {' ',' ',' ',' ',' ',' ',' '}; // Receive up to 7 bytes
                    integerValue = 0;         // throw away previous integerValue
                    while(1) {            // force into a loop until 'n' is received
                    incomingByte = Serial1.read();
                    if (incomingByte == '\n') break;   // exit the while(1), we're done receiving
                    if (incomingByte == 255) continue;  // if no characters are in the buffer read() returns -1
                    if (incomingByte == 45) {
                        integerValue *= -1; 
                        continue;
                    }
                    Serial.print(incomingByte);
                    Serial.print(" ");
                    Serial.print(integerValue);
                    integerValue *= 10;  // shift left 1 decimal place
                    // convert ASCII to integer, add, and shift left 1 decimal place
                    integerValue = ((incomingByte - 48) + integerValue);
                    prevByte = incomingByte;
                    }
                    Serial.println(" v g ");
                    read_from_vision = false;
                    random_offset += integerValue;
                    break;
                }
                default:
                    Serial.println("error");
                    break;
                }
                //Serial.println(input);
                //Serial.write(Serial1.read());   // read it and send it out Serial (USB)
            
            }

          //vision pcontroller
          if(read_from_vision){
            s_goal = (v_sensor - v_goal) * v_gain + y_value + random_offset;
            Serial.print(" s g: ");
            Serial.print(s_goal);
            Serial.print(" ");
            Serial.print(v_sensor);
            Serial.print(" ");
          } 
          //servo pcontroller
          p_value = int((y_value - s_goal) * s_gain) + 85;
          if(p_value > 120){
            p_value = 120;
          } else if(p_value < 45){
            p_value = 45;
          }
          steering.write(p_value);
        }


        // blink LED to indicate activity
        //blinkState = !blinkState;
        //digitalWrite(LED_PIN, blinkState);
        if(debug_done){
          
        }
        prev_y = temp_y;
        //Serial.print("p e" );
    }
}

static int run(Servo esc, int speed, int c_speed) {
  Serial.println("running...");
  if (c_speed == 0){
    c_speed = 1480;
  }
  for (int pos = c_speed; pos <= speed; pos += 1) {
    // in steps of 1 degree
    //Serial.println(pos);
    esc.writeMicroseconds(pos);             
    delay(1);
    c_speed = pos;                       
  }
  return c_speed;
}

void brake(Servo esc) {
  Serial.println("braking...");
  for (int pos = 900; pos >= 800; pos -= 1) {
    esc.writeMicroseconds(pos);              
    delay(1);                       
  }
}

float float_map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}