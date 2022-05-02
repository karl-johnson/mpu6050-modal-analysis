// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high
//MPU6050 accelgyro(0x68, &Wire1); // <-- use for AD0 low, but 2nd Wire (TWI/I2C) object

#define NUM_ACCELS 3
#define PACKET_SIZE 6
#define LINE_SIZE (NUM_ACCELS*PACKET_SIZE)
MPU6050* accelArray[NUM_ACCELS];
const int accelEnablePins[NUM_ACCELS] = {2,4,5};

#define LED_PIN 13
bool blinkState = false;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);

    // initialize all devices
    for(int accelIndex = 0; accelIndex < NUM_ACCELS; accelIndex++) {
      digitalWrite(accelEnablePins[accelIndex],HIGH); // disable all accelerometers
      accelArray[accelIndex] = new MPU6050(0x68);
    }
    // loop through accelerometers and set each one up
    for(int accelIndex = 0; accelIndex < NUM_ACCELS; accelIndex++) {
      digitalWrite(accelEnablePins[accelIndex],LOW); // enable this accelerometer
      accelgyro = *(accelArray[accelIndex]); 
      accelgyro.initialize();
      // verify connection
      accelgyro.testConnection();
      // set default values for some things
      // set digital low pass filter (DLPF) to 183 Hz
      accelgyro.setDLPFMode(1);
      // set sample rate to 500 Hz
      accelgyro.setRate(1); // 1 kHz / (1 + 1)
      // set accelerometer scale to 2 g
      accelgyro.setFullScaleAccelRange(1);
      // enable only accelerometers in FIFO
      accelgyro.setAccelFIFOEnabled(true);
      digitalWrite(accelEnablePins[accelIndex],HIGH); // disable this accelerometer
    }
    // reset FIFO's and enable them all at once for some semblance being synchronous
    resetAllFIFOs(accelArray,accelEnablePins);
    // configure Arduino LED pin for output
    pinMode(LED_PIN, OUTPUT);
}

uint8_t tempData[PACKET_SIZE];
//uint32_t lastReadMicros, microsReadTime;
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
        /*
        case 'f':
          // update frequency to provided value, if it's a reasonable value
          if(commandValue = getValueFromCommand(stringIn)) {
            if(returnVal = setSampleFreq(commandValue)) {
              Serial.print("Set all frequencies to ");
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
              Serial.print("Set all scales to ");
              Serial.print(returnVal);
              Serial.println(" g.");
            }
            else {
              Serial.print("Invalid scale value: ");
              Serial.print(commandValue);
              Serial.println(". Only values of 2, 4, 8, and 16 g are valid.");
            }
          }
          break;*/
        case 'g':
          if(commandValue = getValueFromCommand(stringIn)) {
            // time to do an acquisition
            uint16_t readDataCount = 0;
            uint8_t tempData[LINE_SIZE];
            resetAllFIFOs(accelArray,accelEnablePins);
            while(readDataCount < commandValue) {
              // read raw accel/gyro measurements from device
              readReturnVal = readAllAccelerometers(accelArray, accelEnablePins, tempData);
              //readReturnVal = accelgyro.GetCurrentFIFOPacket(tempData, packetSize);
              //Serial.println(readReturnVal);
              if(readReturnVal == 1) {
                /*for(int i = 0; i < LINE_SIZE; i++) {
                  Serial.write(tempData[i]);
                }*/
                for(int i = 0; i < LINE_SIZE; i+=2) {
                  Serial.print((((int16_t)tempData[i]) << 8) | tempData[i+1]);
                  Serial.print(" ");
                }
                Serial.write('\n');
                readDataCount++;
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

void resetAllFIFOs(MPU6050* accelerometers[NUM_ACCELS], int accelPins[NUM_ACCELS]) {
  for(int accelIndex = 0; accelIndex < NUM_ACCELS; accelIndex++) {
    digitalWrite(accelPins[accelIndex],LOW); // enable this accelerometer
    accelgyro = *(accelArray[accelIndex]); 
    accelgyro.resetFIFO();
    accelgyro.setFIFOEnabled(true);
    digitalWrite(accelPins[accelIndex],HIGH); // disable this accelerometer
  }
}




int readAllAccelerometers(MPU6050* accelerometers[NUM_ACCELS], int accelPins[NUM_ACCELS], uint8_t *outputData) {
  // accelerometers: 
  // accelPins:
  // outputData: int16_t[LINE_LENGTH] for saving output line
  // loop through all accelerometers and wait on FIFO for each one
  // since each has a FIFO, they will still be synced as long as nothing overflows
  for(int accelIndex = 0; accelIndex < NUM_ACCELS; accelIndex++) {
    int readReturnVal = 0;
    uint8_t tempData[PACKET_SIZE];
    digitalWrite(accelPins[accelIndex],LOW); // enable this accelerometer
    accelgyro = *(accelArray[accelIndex]); 
    while(readReturnVal != 1) {
      readReturnVal = accelgyro.GetCurrentFIFOPacket(tempData, PACKET_SIZE);
    }
    for(int i = 0; i < PACKET_SIZE; i++) {
       outputData[i + PACKET_SIZE*accelIndex] = tempData[i];
    }
    digitalWrite(accelPins[accelIndex],HIGH); // disable this accelerometer
  }
  return 1;
}
