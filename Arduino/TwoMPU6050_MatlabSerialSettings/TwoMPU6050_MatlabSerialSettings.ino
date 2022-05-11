// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

//MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high
//MPU6050 accelgyro(0x68, &Wire1); // <-- use for AD0 low, but 2nd Wire (TWI/I2C) object

typedef int (*accelAction)(MPU6050* accel, int argument);

#define NUM_ACCELS 2
#define PACKET_SIZE 6
#define LINE_SIZE (NUM_ACCELS*PACKET_SIZE)
MPU6050* accelArray[NUM_ACCELS];
const int accelEnablePins[NUM_ACCELS] = {2,4};

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
    // HACK for two devices:
    /*
    for(int accelIndex = 0; accelIndex < NUM_ACCELS; accelIndex++) {
      digitalWrite(accelEnablePins[accelIndex],HIGH); // disable all accelerometers
      accelArray[accelIndex] = new MPU6050(0x68);
    }
    */
    accelArray[0] = new MPU6050(0x68); // AD0 low
    accelArray[1] = new MPU6050(0x69); // AD0 high
    // loop through accelerometers and set each one up
    for(int accelIndex = 0; accelIndex < NUM_ACCELS; accelIndex++) {
      digitalWrite(accelEnablePins[accelIndex],LOW); // enable this accelerometer
      accelArray[accelIndex]->initialize();
      // verify connection
      accelArray[accelIndex]->testConnection();
      // set default values for some things
      // set digital low pass filter (DLPF) to 183 Hz
      accelArray[accelIndex]->setDLPFMode(1);
      // set sample rate to 500 Hz
      accelArray[accelIndex]->setRate(1); // 1 kHz / (1 + 1)
      // set accelerometer scale to 2 g
      accelArray[accelIndex]->setFullScaleAccelRange(1);
      // enable only accelerometers in FIFO
      accelArray[accelIndex]->setAccelFIFOEnabled(true);
      digitalWrite(accelEnablePins[accelIndex],HIGH); // disable this accelerometer
    }
    // reset FIFO's and enable them all at once for some semblance being synchronous
    resetAllFIFOs(accelArray,accelEnablePins);
    // configure Arduino LED pin for output
    pinMode(LED_PIN, OUTPUT);
    Serial.print(NUM_ACCELS);
    Serial.println(" accels ready!");
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
        case 'f':
          // update frequency to provided value, if it's a reasonable value
          if(commandValue = getValueFromCommand(stringIn)) {
            if(returnVal = setSampleFreqs(accelArray, accelEnablePins, commandValue)) {
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
            if(returnVal = setScales(accelArray, accelEnablePins, commandValue)) {
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
          break;
        case 'g':
          // GO! take X number of samples
          if(commandValue = getValueFromCommand(stringIn)) {
            // time to do an acquisition
            uint16_t readDataCount = 0;
            uint8_t tempData[LINE_SIZE];
            resetAllFIFOs(accelArray,accelEnablePins);
            while(readDataCount < commandValue) {
              // read raw accel/gyro measurements from device
              readReturnVal = readAllAccelerometers(accelArray, accelEnablePins, tempData);
              //Serial.println(readReturnVal);
              if(readReturnVal == 1) {
                /*
                for(int i = 0; i < LINE_SIZE; i++) {
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
        case 'c':
          // CONTINUOUS: print out values forever (ignore rest of command)
          uint8_t tempData[LINE_SIZE];
          resetAllFIFOs(accelArray,accelEnablePins);
          while(1) {
            // read raw accel/gyro measurements from device
            readReturnVal = readAllAccelerometers(accelArray, accelEnablePins, tempData);
            if(readReturnVal == 1) {
              /*for(int i = 0; i < LINE_SIZE; i++) {
                Serial.write(tempData[i]);
              }*/
              for(int i = 0; i < LINE_SIZE; i+=2) {
                Serial.print((((int16_t)tempData[i]) << 8) | tempData[i+1]);
                Serial.print(" ");
              }
              Serial.write('\n');
            }
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



int doForAllAccels(MPU6050* accels[NUM_ACCELS], const int accelPins[NUM_ACCELS], 
                  accelAction func, int argument) {
  // taking array of accel objects, their pins, a function, and that function's argument as an input,
  // do this on all the accelerometers
  int returnVal;
  for(int accelIndex = 0; accelIndex < NUM_ACCELS; accelIndex++) {
    digitalWrite(accelPins[accelIndex],LOW); // enable this accelerometer
    returnVal = func(accels[accelIndex], argument);
    if(returnVal == 0) return 0;
    digitalWrite(accelPins[accelIndex],HIGH); // disable this accelerometer
  }
  return returnVal; // ehhhhh kinda bad
}

int resetAllFIFOs(MPU6050* accels[NUM_ACCELS], const int accelPins[NUM_ACCELS]) {
  return doForAllAccels(accels, accelPins, &resetOneFIFO, 0);
}

int resetOneFIFO(MPU6050* accel, int dummyArgument) {
  // need int argument to be compatible with doForAllAccels
  accel->resetFIFO();
  accel->setFIFOEnabled(true);
  return 1;
}

int setSampleFreqs(MPU6050* accels[NUM_ACCELS], const int accelPins[NUM_ACCELS], int freqIn) {
  return doForAllAccels(accels, accelPins, &setSampleFreq, freqIn);
}

int setSampleFreq(MPU6050* accel, int freqIn) {
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
  accel->setRate(modeVal);
  if(accel->getRate() == modeVal) {
    Serial.print("Set a freq to ");
    Serial.println(freqIn);
    return freqIn;
  }
  else {
    Serial.println("Frequency setting verification failed!");
    return 0;
  }
}

int setScales(MPU6050* accels[NUM_ACCELS], const int accelPins[NUM_ACCELS], int scaleIn) {
  return doForAllAccels(accels, accelPins, &setScale, scaleIn);
}

int setScale(MPU6050* accel, int scaleIn) {
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
  accel->setFullScaleAccelRange(modeVal);
  if(accel->getFullScaleAccelRange() == modeVal) {
    return scaleIn;
  }
  else {
    Serial.println("Scale setting verification failed!");
    return 0;
  }
}

int readAllAccelerometers(MPU6050* accels[NUM_ACCELS], const int accelPins[NUM_ACCELS], uint8_t *outputData) {
  for(int accelIndex = 0; accelIndex < NUM_ACCELS; accelIndex++) {
    int readReturnVal = 0;
    uint8_t tempData[PACKET_SIZE];
    digitalWrite(accelPins[accelIndex],LOW); // enable this accelerometer
    while(readReturnVal != 1) {
      Serial.println(accelIndex);
      readReturnVal = accelArray[accelIndex]->GetCurrentFIFOPacket(tempData, PACKET_SIZE);
    }
    for(int i = 0; i < PACKET_SIZE; i++) {
       outputData[i + PACKET_SIZE*accelIndex] = tempData[i];
    }
    digitalWrite(accelPins[accelIndex],HIGH); // disable this accelerometer
  }
  return 1;
}
