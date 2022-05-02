// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
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

int16_t ax, ay, az;
int16_t gx, gy, gz;



// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO


#define LED_PIN 13
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
    /*
    Serial.println("Updating internal sensor offsets...");
    // -76	-2359	1688	0	0	0
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    accelgyro.setXGyroOffset(220);
    accelgyro.setYGyroOffset(76);
    accelgyro.setZGyroOffset(-85);
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    */
    // ensure low pass filter is set to 260 Hz
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

    // ONCE MULTIPLE: look at getMultiMaser

    // TODO buffer overflow
    accelgyro.resetFIFO();
    accelgyro.setFIFOEnabled(true);
    
    // configure Arduino LED pin for output
    pinMode(LED_PIN, OUTPUT);

    
}

const int packetSize = 6;
uint8_t tempData[packetSize];
int16_t xAccelVal, yAccelVal, zAccelVal;
uint32_t lastReadMicros, microsReadTime;
uint8_t readReturnVal;

void loop() {
    // read raw accel/gyro measurements from device
    readReturnVal = accelgyro.GetCurrentFIFOPacket(tempData, packetSize);
    //Serial.println(readReturnVal);
    if(readReturnVal == 1) {
      for(int i = 0; i < packetSize; i++) {
         Serial.write(tempData[i]);
      }
      Serial.write('\n');
    }
    else if(readReturnVal == 2) {
      Serial.println("Buffer overflow!");
    }
}
