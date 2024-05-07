// Include Libraries
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

// FUNCTION FORWARD DECLARATION
void IMUDataReady();
void convertUnits();
void generateDataString();


// DEFINITIONS
#define G               9.8         // Gravitation constant
#define FLEX_THUMB      A4
#define FLEX_INDEX      A3
#define FLEX_MIDDLE     A2
#define FLEX_RING       33
#define FLEX_LITTLE     37                                                            
#define INTERRUPT_PIN   14
#define DIV             ","

// FLEX SENSOR RELATED GLOBAL VARIABLES
int FLEX_PINS[] = {FLEX_THUMB, FLEX_INDEX, FLEX_MIDDLE, FLEX_RING, FLEX_LITTLE};
int flex_data[] = {0, 0, 0, 0, 0};
// int calibrated_values[] = {1492, 1374, 1430, 1427, 1357};
int calibrated_values[] = {1371, 1328, 1297, 1344, 1413};

// OUTPUT BUFFER AND VARIABLE
String data;
char buffer[2056];

// MPU6050 RELATED GLOBAL VARIABLES
MPU6050 mpu;
bool imuReady = false;
uint8_t imuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
volatile bool imuInterrupt = false;

// MPU6050 DATA CLASSES AND VARIABLES
Quaternion q;
VectorInt16 aa;
VectorInt16 gyr;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float Gyr[3];
float Acc[3];
float AccReal[3];
float AccWorld[3];


//-------------------------- MAIN ROUTINE --------------------------//

void setup()
{
  // SET PINMODE OF ON_BOARD_LED, MPU6050_INTERRUPT_PIN
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // ON_BOARD_LED is LOW till initialization
  pinMode(INTERRUPT_PIN, INPUT);
  Serial.begin(115200); // INITIALIZE SERIAL  
  while (!Serial);      // wait until serial is available

  // INITIALIZE I2C
    Wire.begin();
    Wire.setClock(400000);

  // INITIALIZE MPU6050
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  
  // TEST CONNECTION
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read());

  // INITIALIZE DIGITAL MOTION PROCESSOR (DMP) OF MPU6050
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // SET ACC, GYR OFFSETS << THESE VALUES SHOULD BE SET AFTER CALIBRATION FOR EACH SENSOR MODULES >>
  mpu.setXAccelOffset(-3015);
  mpu.setYAccelOffset(-759);
  mpu.setZAccelOffset(5165);
  mpu.setXGyroOffset(12);
  mpu.setYGyroOffset(133);
  mpu.setZGyroOffset(-19);

  // CHECK DMP
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    // DEFINE INTERRUPT PIN, FUNCTION AND MODE
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), IMUDataReady, RISING);
    imuIntStatus = mpu.getIntStatus();
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    imuReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } 
  else 
  {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // SUCCESSFULL INITIALIZATION INDICATOR ~ BLINK ONCE
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
}

void loop() {

  if (!imuReady) return;
  // WAIT TILL (MPU INTERRUPT IS NOT TRIGGERED) AND (DMP FIFO IS NOT FILLED)
  while (!imuInterrupt && fifoCount < packetSize) {
    // IF (MPU INTERRUPT IS TRIGGERED) AND (DMP FIFO IS NOT FILLED) UPDATE FIFO COUNTER
    if (imuInterrupt && fifoCount < packetSize) {
      fifoCount = mpu.getFIFOCount();
    }
  }
  // SET MPU INTERRUPT VARIABLE TO LOW VALUE
  imuInterrupt = false;
  imuIntStatus = mpu.getIntStatus();
  // UPDATE FIFO COUNTER
  fifoCount = mpu.getFIFOCount();
  if ((imuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) 
  // CHECK IF FIFO IS OVERFLOWED OR MPU INTERNAL INTERRUPT IS TRIGGERED
  {
    // RESET FIFO AND UPDATE FIFO COUNTER
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();
  }
  // CHECK IF FIFO IS OVERFLOWED
  else if (imuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) 
  {
    // WAIT UNTILL IMU FIFO IS FILLED
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    
    // READ DMP FIFO DATA AND WRITE FIFO BUFFER
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    // READ MPU6050 DMP DATA FROM FIFO BUFFER
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGyro(&gyr, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

    // READ FLEX DATA
    for (int r = 0; r < 5; r++)
    {
      flex_data[r] = analogRead(FLEX_PINS[r]) - calibrated_values[r];
      // Serial.print("Flex data: ");
      // Serial.println(flex_data[r]);
    }

    // ACC, GYR UNIT CONVERTION
    convertUnits();

    // SEND SENSOR READINGS VIA SERIAL
    generateDataString();
    Serial.println(data);
  }

  // CLEAR MPU FIFO
  mpu.resetFIFO();
}


//-------------------------- FUNCTION DEFINITIONS --------------------------//


// IMU INTERRUPT SERVICE ROUTINE
void IMUDataReady()
{
  imuInterrupt = true;
}


// ACC, GYR UNIT CONVERSION ROUTINE
void convertUnits()
{
  /*
The accelerometer is set at the sensitivity of +/-2g while the value is ranged within +/-16384.
The gyroscope is set at the sensitivity of 250 deg which can be translated as 1 degree = 131 measurement units.

ACC (in m.s^-2) = ACC (DMP measurement) / 8192 * G 
GYR (in deg.s^-1) = GYR (DMP measurement) / 131
*/

  Acc[0] = (float)aa.x / 8192 * G;
  Acc[1] = (float)aa.y / 8192 * G;
  Acc[2] = (float)aa.z / 8192 * G;
  AccReal[0] = (float)aaReal.x / 8192 * G;
  AccReal[1] = (float)aaReal.y / 8192 * G;
  AccReal[2] = (float)aaReal.z / 8192 * G;
  AccWorld[0] = (float)aaWorld.x / 8192 * G;
  AccWorld[1] = (float)aaWorld.y / 8192 * G;
  AccWorld[2] = (float)aaWorld.z / 8192 * G;
  Gyr[0] = (float)gyr.x / 131;
  Gyr[1] = (float)gyr.y / 131;
  Gyr[2] = (float)gyr.z / 131;
}

// GENERATE DATA STRING TO PRINT ON SERIAL TERMINAL
void generateDataString()
{
  sprintf(buffer, "%d,%d,%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", flex_data[0], flex_data[1], flex_data[2], flex_data[3], flex_data[4], q.w, q.x, q.y, q.z, Gyr[0], Gyr[1], Gyr[2], Acc[0], Acc[1], Acc[2], AccReal[0], AccReal[1], AccReal[2], AccWorld[0], AccWorld[1], AccWorld[2]);
  data = buffer;
}
