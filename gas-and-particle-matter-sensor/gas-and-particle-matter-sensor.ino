#include <Sodaq_RN2483.h>
#include "Arduino.h"


#define debugSerial SerialUSB
#define loraSerial Serial1

int sensorGasPin = A0;
int sensorDustPin = A1;

void RED() {
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
}

void GREEN() {
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, HIGH);
}

void BLUE() {
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, LOW);
}

void pwm(byte rval,byte gval,byte bval)
{
    //unsigned long v= (rval << 24) | (gval << 16) | (bval << 8);
   
    analogWrite(LED_RED, constrain(rval,0,0xff));
    analogWrite(LED_GREEN, constrain(gval,0,0xff));
    analogWrite(LED_BLUE, constrain(bval,0,0xff));
}

void pwm_int(unsigned long value)
{
  byte rval=(byte)((value & 0xff000000UL) >> 24);
  byte gval=(byte)((value & 0x00ff0000UL) >> 16);
  byte bval=(byte)((value & 0x0000ff00UL) >>  8);
  
  pwm(rval,gval,bval);
}

// Some complete random hex
uint8_t gas_dust_payload[] =
{
  0x53, 0x4F, 0x44, 0x41
};

const uint8_t devAddr[4] =
{
  0x8D, 0xFC, 0x5A, 0x88
};

// TODO: USE YOUR OWN KEYS!
const uint8_t appSKey[16] =
{
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
};

// TODO: USE YOUR OWN KEYS!
const uint8_t nwkSKey[16] =
{
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
};

void setup()
{
  digitalWrite(ENABLE_PIN_IO, HIGH);

  pinMode(sensorGasPin, INPUT);
  pinMode(sensorDustPin, INPUT);

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);

  delay(3000);
  while ((!debugSerial) && (millis() < 10000)){
    // Wait 10 seconds for the debugSerial
  }

   while ((!loraSerial) && (millis() < 10000)){
    // Wait 10 seconds for the debugSerial
  }
  
  debugSerial.begin(57600);
  loraSerial.begin(LoRaBee.getDefaultBaudRate());

  LoRaBee.setDiag(debugSerial); // optional
  if (LoRaBee.initABP(loraSerial, devAddr, appSKey, nwkSKey, false))
  {
    debugSerial.println("Connection to the network was successful.");
  }
  else
  {
    debugSerial.println("Connection to the network failed!");
  }
}

void loop()
{
  debugSerial.println("Sleeping for 5 seconds before starting sending out test packets.");
  for (uint8_t i = 5; i > 0; i--)
  {
    //debugSerial.println(i);
    delay(1000);
  }

  // send 10 packets, with at least a 5 seconds delay after each transmission (more seconds if the device is busy)
  while (true)
  {
    //switch (LoRaBee.sendReqAck(1, testPayload, 5, 3))

    int gas = analogRead(sensorGasPin);
    int dust = analogRead(sensorDustPin);

    gas_dust_payload[0] = (uint8_t) (gas & 0xff);
    gas_dust_payload[1] = (uint8_t) ((gas >> 8) & 0xff);
    gas_dust_payload[2] = (uint8_t) (dust & 0xff);
    gas_dust_payload[3] = (uint8_t) ((dust >> 8) & 0xff);
    
    switch (LoRaBee.send(1, gas_dust_payload, 4))
    {
    case NoError:
      debugSerial.println("Successful transmission.");
      //i--;
      uint8_t buffer[256];
       
      LoRaBee.receive(buffer, sizeof(buffer), 0);
      debugSerial.println((char*)buffer);

      if (strcmp((char*)buffer, "red")==0) {
            RED();

      } else  if (strcmp((char*)buffer, "green")==0) {
            GREEN();

      } else  if (strcmp((char*)buffer, "blue")==0) {
            BLUE();

      } else {
        debugSerial.print(buffer[0]);
        debugSerial.print(buffer[1]);
        debugSerial.print(buffer[2]);
        pwm(buffer[0],buffer[1],buffer[2]);
      }
  
      break;
    case NoResponse:
      debugSerial.println("There was no response from the device.");
      break;
    case Timeout:
      debugSerial.println("Connection timed-out. Check your serial connection to the device! Sleeping for 20sec.");
      delay(20000);
      break;
    case PayloadSizeError:
      debugSerial.println("The size of the payload is greater than allowed. Transmission failed!");
      break;
    case InternalError:
      debugSerial.println("Oh No! This shouldn't happen. Something is really wrong! Try restarting the device!\r\nThe program will now halt.");
      while (1) {};
      break;
    case Busy:
      debugSerial.println("The device is busy. Sleeping for 10 extra seconds.");
      delay(10000);
      break;
    case NetworkFatalError:
      debugSerial.println("There is a non-recoverable error with the network connection. You should re-connect.\r\nThe program will now halt.");
      while (1) {};
      break;
    case NotConnected:
      debugSerial.println("The device is not connected to the network. Please connect to the network before attempting to send data.\r\nThe program will now halt.");
      while (1) {};
      break;
    case NoAcknowledgment:
      debugSerial.println("There was no acknowledgment sent back!");
      break;
    default:
      break;
    }
    //delay(5000);
  }
  while (1) { } // block forever
}
