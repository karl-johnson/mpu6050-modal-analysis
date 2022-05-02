// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 accelgyro;

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO

bool blinkState = false;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(115200);

    // initialize device
    //Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    //Serial.println("Testing device connections...");
    accelgyro.testConnection();
    //Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    // use the code below to change accel/gyro offset values
    // ensure low pass filter is set to 183 Hz
    accelgyro.setDLPFMode(1);
    //Serial.print("Set DLPF to mode: ");
    //Serial.println(accelgyro.getDLPFMode());

    // set sample rate to 500 Hz
    accelgyro.setRate(1); // 1 kHz / (1 + 1)
    //Serial.print("Set sample rate to mode: ");
    //Serial.println(accelgyro.getRate());

    accelgyro.setFullScaleAccelRange(1);

    // enable only accelerometers in FIFO
    accelgyro.setAccelFIFOEnabled(true);

    pinMode(LED_BUILTIN, OUTPUT);    
    Serial.println("Arduino Ready!");
}

const int packetSize = 6;
uint8_t tempData[packetSize];
int16_t xAccelVal, yAccelVal, zAccelVal;
uint32_t lastReadMicros, microsReadTime;
uint8_t readReturnVal;

void loop() {
    // while in this state, allow for commands over serial to change the settings:
    // commands like "f500" change the frequency (valid values: 100, 200, 250, 333, 500)
    // commands like "s4" change the accelerometer scale in g (1,2,4,8,16)
    // commands like "g2048" (go!) will start the accelerometer, and read that # of samples before
        // returning to this setup state
    if(Serial.available() > 0) {
      String stringIn = Serial.readStringUntil('\n');
      char firstChar = stringIn.charAt(0);
      int commandValue = 0;
      int returnVal = 0;
      switch(firstChar) {
        case 'f':
          // update frequency to provided value, if it's a reasonable value
          if(commandValue = getValueFromCommand(stringIn)) {
            if(returnVal = setSampleFreq(commandValue)) {
              Serial.print("Set frequency to ");
              Serial.print(returnVal);
              Serial.println(" Hz.");
            }
            else {
              Serial.print("Invalid frequency value: ");
              Serial.print(commandValue);
              Serial.println(". Only values of 100, 200, 250, 333, and 500 Hz are valid.");
            }
          }
          break;
        case 's':
          // update 
          if(commandValue = getValueFromCommand(stringIn)) {
            if(returnVal = setScale(commandValue)) {
              Serial.print("Set scale to ");
              Serial.print(returnVal);
              Serial.println(" g.");
            }
            else {
              Serial.print("Invalid scale value: ");
              Serial.print(commandValue);
              Serial.println(". Only values of 2, 4, 8, and 16 g are valid.");
            }
          }
          break;
        case 'g':
          if(commandValue = getValueFromCommand(stringIn)) {
            digitalWrite(LED_BUILTIN, HIGH);
            accelgyro.resetFIFO();
            accelgyro.setFIFOEnabled(true);
            digitalWrite(LED_BUILTIN, LOW);
            // time to do an acquisition
            uint16_t readDataCount = 0;
            while(readDataCount < commandValue) {
              // read raw accel/gyro measurements from device
              readReturnVal = accelgyro.GetCurrentFIFOPacket(tempData, packetSize);
              //Serial.println(readReturnVal);
              if(readReturnVal == 1) {
                for(int i = 0; i < packetSize; i++) {
                  Serial.write(tempData[i]);
                }
                Serial.write('\n');
                readDataCount++;
              }
              else if(readReturnVal == 2) {
                Serial.println("Buffer overflow!");
              }
            }
            Serial.print("Finished acquisition of ");
            Serial.print(readDataCount);
            Serial.println(" values.");
          }
          break;
        default:
          Serial.print("Bad command: ");
          Serial.println(stringIn);
      }
    }  
}

int getValueFromCommand(String stringIn) {
  stringIn.remove(0,1);
  int value = stringIn.toInt();
  if(value) return value;
  Serial.print("Received bad command value: ");
  Serial.println(stringIn);
  return 0;
}

int setScale(int scaleIn) {
  uint8_t modeVal;
  switch(scaleIn) {
    // bitshifting would be faster
    // however this is easier to validate inputs
    case 2:
      modeVal = 0;
      break;
    case 4:
      modeVal = 1;
      break;
    case 8:
      modeVal = 2;
      break;
    case 16:
      modeVal = 3;
      break;
    default:
      return 0;
  }
  accelgyro.setFullScaleAccelRange(modeVal);
  if(accelgyro.getFullScaleAccelRange() == modeVal) {
    return scaleIn;
  }
  else {
    Serial.println("Scale setting verification failed!");
    return 0;
  }
}

int setSampleFreq(int freqIn) {
  uint8_t modeVal;
  switch(freqIn) {
    // doing math would be faster
    // however this is easier to validate inputs
    case 100:
      modeVal = 9;
      break;
    case 200:
      modeVal = 4;
      break;
    case 250:
      modeVal = 3;
      break;
    case 333:
      modeVal = 2;
      break;
    case 500:
      modeVal = 1;
      break;
    default:
      return 0;
  }
  accelgyro.setRate(modeVal);
  if(accelgyro.getRate() == modeVal) {
    return freqIn;
  }
  else {
    Serial.println("Frequency setting verification failed!");
    return 0;
  }
}
