// Accelerometer based remote control
// KORA Robotics
// Written by Matt Ruffner 1/31/2023
//
// This software listens for euler angles (yaw/pitch/roll)
// being transmitted via LoRa packet radio
// and converts them into two wheel drive commands
// sent to the RoboClaw motor controller connected 
// to the wheelchair motors
// https://reyax.com/products/rylr896/


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <RoboClaw.h>

// serial port definitions
#define SERIAL Serial
#define RADIO_SERIAL Serial2


//#define DEBUG 1
//#define DEBUG_RAD 1


#define RBUF_SIZE 300
#define SBUF_SIZE 240
#define RX_TIMEOUT_PERIOD 4000
volatile bool newCmdToSend = false;
volatile unsigned char cmdToSend;
static uint8_t rbuf[RBUF_SIZE];
static char sbuf[SBUF_SIZE];
static char printbuf[1000];
unsigned long lastSendTime = 0;
unsigned long lastRxTime = 0;


// hardware pin definitions
#define RADIO_RESET_PIN 8

struct ypr_t {
  float yaw;
  float pitch;
  float roll;
};

RoboClaw roboclaw = RoboClaw(&Serial1, 10000);

ypr_t rxBuf;

void setup(void)
{
  Serial.begin(115200); // USB serial
  Serial1.begin(38400); // roboclaw serial
  Serial2.begin(115200); // Radio serial

  //while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Wheelchair basestation test"); Serial.println("");

  pinMode(RADIO_RESET_PIN, OUTPUT);
  digitalWrite(RADIO_RESET_PIN, LOW);
  delay(1000);
  digitalWrite(RADIO_RESET_PIN, HIGH);
  digitalWrite(RADIO_RESET_PIN, LOW);
  delay(1000);
  digitalWrite(RADIO_RESET_PIN, HIGH);

  Serial.println("Starting roboclaw...");

  roboclaw.begin(38400);

}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{

  if ( millis() - lastRxTime > 1000) {
    roboclaw.ForwardM1(0x80, 0);
    roboclaw.ForwardM2(0x80, 0);
  }

  /******************************************/
  // now do a loop through the radio state machine

  radio_tick();
}

void radio_tick() {
  static int state = 0;
  
  if ( state == 1 ) {
    Serial.println("in state 1");
    int len = sprintf(sbuf, "AT+ADDRESS=1\r\n");
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
    const int dataSize = sizeof(uint8_t);
    sprintf(sbuf, "AT+SEND=1,%d,", dataSize); // where 2 is the address
    int pre = strlen(sbuf);

    // embed command
    sbuf[pre] = cmdToSend;

    sbuf[pre + dataSize] = '\r';
    sbuf[pre + dataSize + 1] = '\n';
    sbuf[pre + dataSize + 2] = 0;


#ifdef DEBUG_RAD
    //if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    SERIAL.print("sending radio binary packet of size ");
    SERIAL.println(pre + dataSize + 2);
    SERIAL.print("actual data was (bytes): ");
    SERIAL.println(dataSize);
    //SERIAL.println("DONE");
    //xSemaphoreGive( dbSem );
    //}
#endif

    // send to lora module
    RADIO_SERIAL.write(sbuf, pre + dataSize + 2);
    //SERIAL_LOR.write(sbuf, pre);

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
      // RX TIMEOUT
    } else if (!timeout && eol) {
      // PACKET RECIEVED
    } else if ( !timeout && !eol) {
      // BUFFER OVERRUN
    }

    // if first byte is non-zero then we received data ( also check timeout and eol vars to be safe)
    // now process the data line received
    if ( (rbuf[0] == '+') && !timeout && eol) {
      int eqpos = 1;
      while ( rbuf[eqpos] != '=' &&  eqpos < pos) {
        eqpos++;
      }
      if ( eqpos == pos ) {
        if ( state < 7 ) {
          state ++;
        } else if ( state > 7 ) {
          state = 7;
        }

      } else {
        // check if its a receive message
        if ( rbuf[0] == '+' &&
             rbuf[1] == 'R' &&
             rbuf[2] == 'C' &&
             rbuf[3] == 'V') {

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



          ///////////////////////////
          // PARSE RECEIVED DATA!!!!
          ////////////////////////////
          data_callback(&rbuf[dataPos]);


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
          memset(rbuf, 0, RBUF_SIZE);
        }
      }
    }
  }
}

void data_callback(uint8_t *data) {
  memcpy((void*)&rxBuf, (void *)data, sizeof(ypr_t));

  Serial.print("Yaw: ");
  Serial.print(rxBuf.yaw);
  Serial.print(", Pitch: ");
  Serial.print(rxBuf.pitch);
  Serial.print(", Roll: ");
  Serial.println(rxBuf.roll);

  lastRxTime = millis();


  float pitch = min(max(-50, rxBuf.pitch), 50);
  float roll = min(max(-50, rxBuf.roll), 50);
  pitch = map(pitch, -50.0, 50.0, -1.0, 1.0);
  roll  = map(roll, -50.0, 50.0, -1.0, 1.0);
  float theta = atan2(pitch, roll);
  float r = sqrt(pitch * pitch + roll * roll);
  float m1, m2;
  
  // rotate by 45 degrees
  theta += PI / 4;

  //# back to cartesian
  float left = r * cos(theta);
  float right = r * sin(theta);

  //# rescale the new coords
  left = left * sqrt(2);
  right = right * sqrt(2);

  //# clamp to -1/+1
  left = max(-1, min(left, 1));
  right = max(-1, min(right, 1));

  m1 = map(left, -1.0, 1.0, -127, 127);
  m2 = map(right, -1.0, 1.0, -127, 127);
  

  if(m1 < 0) {
    roboclaw.ForwardM1(0x80, abs(m1));
  } else {
    roboclaw.BackwardM1(0x80, abs(m1));
  }

  if (m2 < 0) {
    roboclaw.ForwardM2(0x80, abs(m2));
  } else {
    roboclaw.BackwardM2(0x80, abs(m2));
  }

  Serial.print("M1: "); Serial.print(m1);
  Serial.print(", M2: "); Serial.print(m2);
}
