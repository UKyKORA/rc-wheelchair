// Accelerometer based remote control
// KORA 2023
// Written by Matt Ruffner 1/31/2023
//
// This software reads Euler angles (yaw/pitch/roll) from a BNO055 
// attched via I2C and send them over a LoRa packet radio
// to node address 1 on network 1 ( wheelchair #1)
// https://reyax.com/products/rylr896/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define DEBUG 1
#define DEBUG_RAD 1

// hardware pin definitions
#define RADIO_RESET_PIN 11
#define BUTTON 41
#define LED 13

#define RX_TIMEOUT_PERIOD 4000

// Serial port assignments
#define SERIAL Serial
#define RADIO_SERIAL Serial1

// radio variables
#define RBUF_SIZE 300 // rx buffer size
#define SBUF_SIZE 240 // tc buffer size
bool newCmdToSend = false; // whether the send buffer needs to be sent
static uint8_t rbuf[RBUF_SIZE]; // rx buffer
static char sbuf[SBUF_SIZE]; // tx buffer
unsigned long last_send = 0; // // last send in millis
unsigned long send_timeout = 100; // milliseconds

// structure to hold variables for sending over radio
struct ypr_t {
  float yaw;
  float pitch;
  float roll;
};
ypr_t dataBuf;

// create adafruit BNO055 object on the Wire2 port
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire2);

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  SERIAL.begin(115200);
  RADIO_SERIAL.begin(115200);

  //while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  delay(1000);

  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(LED, OUTPUT);

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(100);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");


  memset(rbuf, 0, RBUF_SIZE);
  memset(sbuf, 0, SBUF_SIZE);

  pinMode(RADIO_RESET_PIN, OUTPUT);
  digitalWrite(RADIO_RESET_PIN, LOW);
  delay(1000);
  digitalWrite(RADIO_RESET_PIN, HIGH);
  digitalWrite(RADIO_RESET_PIN, LOW);
  delay(1000);
  digitalWrite(RADIO_RESET_PIN, HIGH);
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{

  if ( millis() - last_send > send_timeout ) {

    last_send = millis();
    newCmdToSend = true;

    // Possible vector values can be:
    // - VECTOR_ACCELEROMETER - m/s^2
    // - VECTOR_MAGNETOMETER  - uT
    // - VECTOR_GYROSCOPE     - rad/s
    // - VECTOR_EULER         - degrees
    // - VECTOR_LINEARACCEL   - m/s^2
    // - VECTOR_GRAVITY       - m/s^2
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);



    // send real data when the button is pressed, 
    // otherwise send stop data (all zeros YPR)
    if( digitalRead(BUTTON)==LOW ){
      digitalWrite(LED, LOW);
      dataBuf.yaw = 0;
      dataBuf.pitch = 0;
      dataBuf.roll = 0;
    } else { 
      digitalWrite(LED, HIGH);
      dataBuf.yaw = euler.x();
      dataBuf.pitch = euler.y();
      dataBuf.roll = euler.z();
    }
  } 
  
  /******************************************/
  // now do a loop through the radio state machine

  radio_tick();
  
}

void radio_tick() {

  static int state = 0;
  
  if ( state == 1 ) {
    Serial.println("in state 1");
    int len = sprintf(sbuf, "AT+ADDRESS=2\r\n");
    RADIO_SERIAL.write(sbuf, len);
    state = 2;
  }
  else if ( state == 3 ) {
    int len = sprintf(sbuf, "AT+NETWORKID=1\r\n");
    RADIO_SERIAL.write(sbuf, len);
    state = 4;
  }
  else if ( state == 5 ) {
    int len = sprintf(sbuf, "AT+PARAMETER=8,8,1,4\r\n"); // recommended for less than 3km
    //int len = sprintf(sbuf, "AT+PARAMETER=12,4,1,7\r\n"); // recommended for more than 3km
    RADIO_SERIAL.write(sbuf, len);
    state = 6;
  }

  if ( newCmdToSend && state == 7) {
    // START SENDING
    // first get size of payload data in bytes
    const int dataSize = sizeof(ypr_t);
    // now prefill the send buffer with the AT command
    sprintf(sbuf, "AT+SEND=1,%d,", dataSize);
    int pre = strlen(sbuf); // number of preamble (AT command size) bytes

    // embed payload in send buffer
    memcpy(&sbuf[pre], &dataBuf, dataSize);
    // fill in terminator characters
    sbuf[pre + dataSize] = '\r';
    sbuf[pre + dataSize + 1] = '\n';
    sbuf[pre + dataSize + 2] = 0;


    #ifdef DEBUG_RAD
    SERIAL.print("sending radio binary packet of size ");
    SERIAL.println(pre + dataSize + 2);
    SERIAL.print("actual data was (bytes): ");
    SERIAL.println(dataSize);
    #endif

   // send all bytes in send buffer to lora module
   RADIO_SERIAL.write(sbuf, pre + dataSize + 2);

   // go to state 8 so that we wait for a response
   state = 8;
   newCmdToSend = false;
  }

  // handle incoming message from LORA radio
  // AT message from module
  bool eol = false;
  int pos = 0;
  bool timeout = false;
  int sawComma = 0;
  unsigned long timeoutStart = 0;
  int expectedDataLen = 0;

  char rbts[4]; // receive buffer text length i.e "127"
  int rbtsLen = 0; // number of chars in rbts
  int payloadSize = 0;

  char rxbyte;
  if ( RADIO_SERIAL.available() ) rxbyte = RADIO_SERIAL.read();

  // if we see the start of a message in the available bytes, 
  // start reading into memory until EOL or timeout
  if ( rxbyte == '+' ) {
    unsigned long timeoutStart = millis();
    rbuf[pos++] = rxbyte;
    while (!eol && !timeout && pos < RBUF_SIZE - 1) {
      if ( RADIO_SERIAL.available() ) {
        rbuf[pos] = RADIO_SERIAL.read();
        if ( pos > 1 ) {
          if ( rbuf[pos] == '\n' && rbuf[pos - 1] == '\r' ) {
            memset(&rbuf[pos + 1], 0, RBUF_SIZE - (pos + 1));
            eol = true;
          }
        }

        if ( pos++ >= RBUF_SIZE ) {
          break;
        }
      }
      if ( millis() - timeoutStart > RX_TIMEOUT_PERIOD ) {
        memset(rbuf, 0, RBUF_SIZE);
        timeout = true;
      }
    }


    if ( timeout ) {
      #ifdef DEBUG
      SERIAL.println("Radio rx timed out");
      #endif
    } else if (!timeout && eol) {
      #ifdef DEBUG
      SERIAL.println("Radio got packet!");
      #endif
    } else if ( !timeout && !eol) {
      #ifdef DEBUG
      SERIAL.println("Radio receive buffer overrun!");
      #endif
    }

    // if first byte is non-zero then we received data ( also check timeout and eol vars to be safe)
    // now process the data line received
    if ( (rbuf[0] == '+') && !timeout && eol) {
      int eqpos = 1;
      while ( rbuf[eqpos] != '=' &&  eqpos < pos) {
        eqpos++;
      }
      if ( eqpos == pos ) {
        #ifdef DEBUG
        SERIAL.print("We think we got a +READY or +OK message, we actually got: ");
        SERIAL.write(rbuf, pos);
        #endif
        
        if ( state < 7 ) {
          state ++;
          if ( state == 7 ) {
            #ifdef DEBUG_RAD
            SERIAL.println("STATE = 7, successfully configured radio!");
            #endif
          }
        } else if ( state > 7 ) {
          state = 7;
          #ifdef DEBUG_RAD
          SERIAL.println("STATE was 8, received +OK from a data send operation!");
          #endif
        }

      } else {
        // found an '=', parse rest of message
        #ifdef DEBUG
        SERIAL.print("We think we got a message with an '=' in it, we actually got: ");
        SERIAL.write(rbuf, pos);
        #endif

        // check if its a receive message
        if ( rbuf[0] == '+' &&
             rbuf[1] == 'R' &&
             rbuf[2] == 'C' &&
             rbuf[3] == 'V') {


          #ifdef PRINT_RX_STATS
          SERIAL.print("data: "); SERIAL.write(rbuf, pos);
          SERIAL.println();
          #endif

          // parse data
          // example rx string: +RCV=50,5,HELLO,-99,40
          const char *comma = ",";
          char *token;
          char *data;
          int rssi;
          int snr;
          int addr = -1;
          int datalen = -1;
          int cc = 0;

          // find start of data chunk
          int dataPos = 0;
          while ( dataPos < pos) {
            if ( rbuf[dataPos] == ',' ) {
              cc++;
              if ( cc == 2 ) {
                dataPos++;
                break;
              }
            }
            dataPos++;
          }

          //SERIAL.print("dataPos is ");
          //SERIAL.println(dataPos);

          // assume that data coming from the capsule
          // is a specific data structure for now
          //memcpy((void*)&rxtlm, (void *)&rbuf[dataPos], sizeof(tlm_t));

          // parse target address
          token = strtok((char *) &rbuf[5], comma);
          addr = atoi(token);

          // extract data length
          token = strtok(NULL, comma);
          datalen = atoi(token);

          // get pointer to start of data
          //data = strtok(NULL, comma);

          // get the rssi
          token = strtok((char *) &rbuf[8 + datalen], comma);
          token = strtok(NULL, comma);
          rssi = atoi(token);

          // get the SNR
          token = strtok(NULL, comma);
          snr = atoi(token);


          // end of parse received data

        }
      }
    }
  }
}
