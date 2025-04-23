


#define useEncoder
#define useMLX90393
//#define usePickup

#include "limits.h"


bool triggerReading = false;
unsigned long triggerTime = ULONG_MAX; // pgm time at which reading is triggered
// unsigned long readDelay = 200 + 10;
int16_t dataPt = 0; // incremented on each reading

//****************************************************************************//


#ifdef useEncoder

long triggerInt = 0; // integer counter of trigger (goes up and down)
float mmPerTrigger = 0.4242650; // each edge counts as a trigger in Encoder

const int encoderDebugLevel = 0; // how verbose the output is

#include "src/Encoder-master/Encoder.h"
// Note 1: The version 1.4.4 Paul Stoffregen Encoder library provided by the IDE
// does not include support for Uno R4 boards. Download the github version
// instead (commit c7627dd, reported version 1.4.4), and keep it local to the
// project directory so it doesn't conflict with IDE provided library
// Note 2: One Uno R4 boards it doesn't seem to work if the second, dtPin, is one 
// of the physical interrupt pins, even though both being interrupt pins is 
// supposedly optimal.

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
//   avoid using pins with LEDs attached
#ifdef ARDUINO_UNOR4_MINIMA
// for some strange reason pins 2/3 don't seem to work for UNO R4
// but 3/4 do.
const int clkPin = 3;
const int dtPin = 4;
#else
const int clkPin = 2;
const int dtPin = 3;
#endif

Encoder myEnc(clkPin,dtPin);
long oldPosition  = -999;

long encoderStep = 4; // take a measurement every n steps

#endif

//****************************************************************************//

#ifdef useMLX90393

const int bFieldDebugLevel = 0; // how verbose the output is

#include "src/Adafruit_MLX90393/Adafruit_MLX90393.h"
// Same as the stock library, but readRegister
// has been moved from private to public

Adafruit_MLX90393 bFieldSensor = Adafruit_MLX90393();

bool doCalPrompt = true;      // only ask to run calibration loop once
int startupDelay = 5;
int16_t data_pt;
int nCalPts = 1000;


mlx90393_gain_t analogGain = MLX90393_GAIN_1X; // _1X = 0x07 = max analog gain
mlx90393_resolution_t adcRes = MLX90393_RES_16; // _16 = 0x00 = lowest 16 bits from 19 bit adc

// OSR_3 = 0x03 = highest OSR
// FILTER_7 = 0x07 = most filtering
// select OSR = 0, filter = 6
// this drops the T_conv down to 50+10 ms
// and noise will be ~5to8 mGauss (0.5 to 0.8 uT)
mlx90393_oversampling_t osr = MLX90393_OSR_0; // _0 = 0x00 = lowest OSR
mlx90393_filter_t digFilt = MLX90393_FILTER_6; // _6 = 0x06 = second most filtering
unsigned long readDelay = mlx90393_tconv[digFilt][osr] + 5; // [6][0] : tconv = 13.36+5
//

#endif

//****************************************************************************//

#ifdef usePickup

#endif

void setup() {
    // put your setup code here, to run once:
    // put your setup code here, to run once:
    Serial.begin(115200);
    Serial.setTimeout(10);

    while ((!Serial) && (millis() < 2000)) {
        // wait for serial port to connect. Needed for native USB
        // timeout after 2 seconds
        ;
    }

    if (Serial) {
        Serial.println("Starting Program");
    }

#ifdef useMLX90393
    uint16_t registerData;
    //readDelay = mlx90393_tconv[digFilt][osr] + 5;

    if (! bFieldSensor.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
        Serial.println("No sensor found ... check your wiring?");
        while (1) { delay(10); }
    }
    Serial.println("Found a MLX90393 sensor");

    // Set analog gain
    bFieldSensor.setGain(analogGain);

    // Set resolution (which ADC bits), per axis
    bFieldSensor.setResolution(MLX90393_X, adcRes);
    bFieldSensor.setResolution(MLX90393_Y, adcRes);
    bFieldSensor.setResolution(MLX90393_Z, adcRes);

    // Set oversampling
    bFieldSensor.setOversampling(osr);

    //Serial.print("Max Field (uT, XY / Z):");
    //Serial.print(2^15*

    // Set digital filtering
    bFieldSensor.setFilter(digFilt);
    if (bFieldDebugLevel > 0) {
        Serial.print("Gain set to: ");
        switch (bFieldSensor.getGain()) {
        case MLX90393_GAIN_1X: Serial.println("1 x"); break;
        case MLX90393_GAIN_1_33X: Serial.println("1.33 x"); break;
        case MLX90393_GAIN_1_67X: Serial.println("1.67 x"); break;
        case MLX90393_GAIN_2X: Serial.println("2 x"); break;
        case MLX90393_GAIN_2_5X: Serial.println("2.5 x"); break;
        case MLX90393_GAIN_3X: Serial.println("3 x"); break;
        case MLX90393_GAIN_4X: Serial.println("4 x"); break;
        case MLX90393_GAIN_5X: Serial.println("5 x"); break;
        }

        bFieldSensor.readRegister(0x00,&registerData);
        Serial.print("Register 0x00:"); Serial.println(registerData);
        bFieldSensor.readRegister(0x01,&registerData);
        Serial.print("Register 0x01:"); Serial.println(registerData);
        bFieldSensor.readRegister(0x02,&registerData);
        Serial.print("Register 0x02:"); Serial.println(registerData);
        bFieldSensor.readRegister(0x03,&registerData);
        Serial.print("Register 0x03:"); Serial.println(registerData);
        bFieldSensor.readRegister(0x04,&registerData);
        Serial.print("Register 0x04:"); Serial.println(registerData);
        bFieldSensor.readRegister(0x05,&registerData);
        Serial.print("Register 0x05:"); Serial.println(registerData);
        bFieldSensor.readRegister(0x06,&registerData);
        Serial.print("Register 0x06:"); Serial.println(registerData);
    }
#endif

} // end setup




void loop() {
    // put your main code here, to run repeatedly:

    // millis() wraps in around 49 days. Should be enough for our purposes
    unsigned long pgmTime = millis();
    char outStr[80];

#ifdef useMLX90393
    float x, y, z;
    char inChar;

    // Run the calibration loop if requested
    if ( doCalPrompt ) {

        Serial.println("Run CalLoop? (y/n)");

        while ( doCalPrompt && (startupDelay > 0) ) {
        //for (int i = startupDelay; i>0; i--) {
            Serial.print(startupDelay,DEC);
            Serial.print("..");
            Serial.flush();
            delay(1000);
            while (Serial.available()>0){
                inChar = Serial.read();
                if (inChar=='y') {
                    Serial.println("While loop is running (1000 pts), rotate sensor slowly about each axis");
                    CalLoop();
                    doCalPrompt=false;
                } else if (inChar=='n') {
                    doCalPrompt=false;
                }
            }
            startupDelay--;
        }
        doCalPrompt=false;
        Serial.println("");
        Serial.println("Press R [ENTER] to take a reading, N [ENTER] for breakpoint");
        Serial.println("MeasI,posI,pos(mm),b(uT)X,Y,Z");
    }

    //if (Serial.available() > 0) {
    //    String inStr = Serial.readString();
    //    inStr.trim();
    //    if ( (inStr == "r") && (triggerTime==ULONG_MAX)) {
    //        triggerReading = true;
    //    } else if (inStr=="n") {
    //        Serial.println("NEWLINE");
    //    } else {
    //        Serial.println(inStr);
    //    }
    //}

    while (Serial.available()>0){
        inChar = Serial.read();
        //Serial.println((int)inChar);
        if ( (inChar=='R') && (triggerTime==ULONG_MAX) ) {
            triggerReading = true;
        } else if (inChar=='N') {
            Serial.print("NEWLINE");
        } else if ( (inChar=='\n'||inChar=='\r') && triggerReading ) {
            //Serial.println("");
            ;
        } else if ( (inChar=='\n'||inChar=='\r') && !triggerReading ) {
            Serial.println("");
        } else {
            Serial.print(inChar);
        }
    }

    if ( triggerReading && (triggerTime==ULONG_MAX) ) {

        if (!bFieldSensor.startSingleMeasurement()) {
            Serial.println("Sensor Trig Failed");
            return;
        }
        triggerTime = pgmTime;
        if (encoderDebugLevel > 1) {
            Serial.print("reading Triggered:");
            Serial.println(triggerTime,DEC);
        }
    }

    if ( triggerReading && ((pgmTime-triggerTime)>=readDelay) ) {
        // reset the trigger
        triggerTime = ULONG_MAX;
        triggerReading = false;
        //Serial.print("take Reading:");
        //Serial.print(pgmTime,DEC);
        //Serial.print(", ");
        //Serial.print(triggerTime,DEC);
        //Serial.print(", ");
        //Serial.println(pgmTime-triggerTime,DEC);
        if (bFieldSensor.readMeasurement(&x,&y,&z)){
            dataPt ++;
            // NOTE: on arduino uno, etc %f does not work for sprintf!
            // I'm pretty sure it works for R4 chips
            //sprintf(outStr,"%5.0f, %7.2f, %7.2f, %7.2f",dataPt,x,y,z);
            //Serial.println(outStr);
            //Serial.print(dataPt); Serial.print(",");
            Serial.print(pgmTime); Serial.print(",");
            Serial.print(triggerInt); Serial.print(",");
            Serial.print(float(triggerInt)*mmPerTrigger,1); Serial.print(",");
            Serial.print(x,2);    Serial.print(",");
            Serial.print(y,2);    Serial.print(",");
            Serial.println(z,2);
        } else {
            Serial.print("readingAttempted:");
            Serial.println(pgmTime,DEC);
            Serial.println("Unable to read XYZ data from the sensor.");
        }
    }

#endif

#ifdef useEncoder

    long newPosition = myEnc.read();
    //Serial.print(clkPin);
    //Serial.print("newPosition ");
    //Serial.println(newPosition);
    if (newPosition != oldPosition) {

        oldPosition = newPosition;
        if (encoderDebugLevel > 1) {
            Serial.print("EncoderCount: ");
            Serial.println(newPosition);
            //Serial.print(',');
            //Serial.println(newPosition%measStep);
        }

        if (newPosition % encoderStep == 0) {
            if (triggerReading) {
                Serial.println("Warning: Second trigger detected before 1st complete");
            } else {
                triggerReading = true;
            }
            triggerInt = newPosition;

            if (encoderDebugLevel > 0) {
                Serial.print(newPosition);
                Serial.print(',');
                Serial.println(newPosition % encoderStep);
            }
        }

    }
#endif

}

#ifdef useMLX90393

uint8_t CalLoop() {

    float x, y, z;
    uint16_t offset, xi,yi,zi,ti;
    uint8_t txbuf[1];
    uint8_t rxbuf[9] = {0};
    uint8_t interdelay = 0;
    uint8_t status, status_sm;

    /* Set analog gain and precision
     * GAIN_1X = 0x07
     * RES_16 = 0x00 (lowest 16 bits of 19bit ADC)
     * so Gain = 0x07, Res=0x00 is 0.150/0.242 uT/LBS
     * on XY/Z
     * */
    bFieldSensor.setGain(MLX90393_GAIN_1X);
    bFieldSensor.setResolution(MLX90393_X, MLX90393_RES_16);
    bFieldSensor.setResolution(MLX90393_Y, MLX90393_RES_16);
    bFieldSensor.setResolution(MLX90393_Z, MLX90393_RES_16);

    /* Set oversampling and digital filtering
     * OSR_2 = 0x02
     * FILTER_6 = 0x06
     * above result in a Tconv = 51.38 ms, but also still relatively low noise ~< 5 mGauss (0.5 uT)
     * */
    bFieldSensor.setOversampling(MLX90393_OSR_2);
    bFieldSensor.setFilter(MLX90393_FILTER_6);

    delay(10);


    Serial.println("Offset Values X, Y, Z");
    bFieldSensor.readRegister(0x04,&offset); // OFFSET_X
    Serial.print(offset); Serial.print(", ");
    bFieldSensor.readRegister(0x05,&offset); // OFFSET_Y
    Serial.print(offset); Serial.print(", ");
    bFieldSensor.readRegister(0x06,&offset); // OFFSET_Z
    Serial.print(offset); Serial.println();

    delay(1000);

    Serial.println("Raw Values: SM status, RM status, T, X, Y, Z");

    for (int i=0; i<nCalPts; i++){

      delay(5);

        /* Start a single measurement. */
        txbuf[0] = MLX90393_REG_SM | MLX90393_AXIS_ALL | 0x01;
        //Serial.print("tx command: ");
        //Serial.println(txbuf[0]);
        if (bFieldSensor.i2c_dev) {
            if (!bFieldSensor.i2c_dev->write(txbuf, 1))
                return false;
            delay(interdelay);

            if (!bFieldSensor.i2c_dev->read(&status_sm, 1))
                return false;

            //Serial.print("Start Measurement Status: ");
            //Serial.print(status_sm); Serial.println();
        }

        delay(65);


        /* Read a single data sample. */
        txbuf[0] = MLX90393_REG_RM | MLX90393_AXIS_ALL | 0x01;
        //Serial.print("tx command:");
        //Serial.println(txbuf[0]);

        /*
        status = sensor.transceive(txbuf,1,rxbuf,6,0);
        Serial.println(status);
        */

        if (bFieldSensor.i2c_dev) {
            if (!bFieldSensor.i2c_dev->write(txbuf, 1))
                return false;
            delay(interdelay);

            if (!bFieldSensor.i2c_dev->read(rxbuf, 9))
                return false;
            //Serial.println(rxbuf[0]);
            //Serial.println(rxbuf[1]);
            //Serial.println(rxbuf[2]);
            //Serial.println(rxbuf[3]);
            //Serial.println(rxbuf[4]);
        }
        status = rxbuf[0];
        ti = (rxbuf[1] << 8) | rxbuf[2];
        xi = (rxbuf[3] << 8) | rxbuf[4];
        yi = (rxbuf[5] << 8) | rxbuf[6];
        zi = (rxbuf[7] << 8) | rxbuf[8];

        //Serial.println("Raw Values: i, SM status, RM status, T, X, Y, Z");
        Serial.print(status_sm); Serial.print(", ");
        Serial.print(i); Serial.print(", ");
        Serial.print(status); Serial.print(", ");
        Serial.print(ti); Serial.print(", ");
        Serial.print(xi); Serial.print(", ");
        Serial.print(yi); Serial.print(", ");
        Serial.print(zi); Serial.println();

    }

    // reset sampling

    // Set analog gain
    bFieldSensor.setGain(analogGain);

    // Set resolution (which ADC bits), per axis
    bFieldSensor.setResolution(MLX90393_X, adcRes);
    bFieldSensor.setResolution(MLX90393_Y, adcRes);
    bFieldSensor.setResolution(MLX90393_Z, adcRes);

    // Set oversampling
    bFieldSensor.setOversampling(osr);

    // Set digital filtering
    bFieldSensor.setFilter(digFilt);

    return true;


    /* Start a single measurement. */
    txbuf[1] = {MLX90393_REG_SM | MLX90393_AXIS_ALL};
    Serial.println(txbuf[1]);
    if (bFieldSensor.i2c_dev) {
        if (!bFieldSensor.i2c_dev->write(txbuf, 1))
            return false;
        delay(interdelay);

        if (!bFieldSensor.i2c_dev->read(&status, 1))
            return false;

        //status = rxbuf[0];
        Serial.print("Start Measurement Status: ");
        Serial.print(status,BIN); Serial.println();
    }

    delay(200);

    /* Read a single data sample. */
    if (bFieldSensor.i2c_dev) {
        if (!bFieldSensor.i2c_dev->write(txbuf, 1))
            return false;
        delay(interdelay);

        if (!bFieldSensor.i2c_dev->read(rxbuf, 7))
            return false;
    }

    status = rxbuf[0];
    ti = (rxbuf[1] << 8) | rxbuf[2];
    xi = (rxbuf[3] << 8) | rxbuf[4];
    yi = (rxbuf[5] << 8) | rxbuf[6];
    zi = (rxbuf[7] << 8) | rxbuf[8];

    Serial.println("Raw Values: Status, T, X, Y, Z");
    Serial.print(status,BIN); Serial.print(", ");
    Serial.print(ti); Serial.print(", ");
    Serial.print(xi); Serial.print(", ");
    Serial.print(yi); Serial.print(", ");
    Serial.print(zi); Serial.println();

}

#endif
