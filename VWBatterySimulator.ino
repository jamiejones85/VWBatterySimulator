#include <Ticker.h>
#ifndef ARDUINO_ARCH_ESP32
  #error "Select an ESP32 board" 
#endif

#include <ACAN_ESP32.h>
#define LED_BUILTIN 2

volatile bool isSending = false; 
unsigned long lastRecievedMillis = 0;

CANMessage inFrame;
CANMessage outFrame;

Ticker timeoutTicker; 

uint8_t moduleCount = 8; //default to one pack
uint8_t currentModule = 1;

uint16_t cellVolatge = 3800;

uint16_t moduleAddresses[] = {0x1B0, 0x1B4, 0x1B8, 0x1BC, 0x1C0, 0x1C4, 0x1C8, 0x1CC};

void printHelp() {
  Serial.println(F("Commands"));
  Serial.println(F("h - prints this message"));
  Serial.println(F("V - Sets the mV of all the cells"));
  Serial.println(F("T - Sets the tempearture of modules"));

  Serial.println(F("P - Sets the Pack Count, e.g if paralleling packs and using a MITM"));
  Serial.println(F("Example: V=3800 - sets all cells to 3800mV"));
}


void timeout() {
    if (millis() - lastRecievedMillis > 1000) {
      isSending = false;
    }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin (115200) ;
  while (!Serial) {
    delay (50) ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
  }
  digitalWrite (LED_BUILTIN, LOW);

  //config on board can
  Serial.println("Initializing CAN...");
  ACAN_ESP32_Settings settings2(500 * 1000);
  settings2.mRxPin = GPIO_NUM_4;
  settings2.mTxPin = GPIO_NUM_5;
  const uint32_t errorCode2 = ACAN_ESP32::can.begin (settings2) ;
  if (errorCode2 == 0) {
    Serial.println ("Configuration ESP32 OK!");
  }else{
    Serial.print ("Configuration error 0x") ;
    Serial.println (errorCode2, HEX) ;
  }
  timeoutTicker.attach(1, timeout);

  
  Serial.println ("VW PHEV Battery Simulator running");
  printHelp();
}


void canCheck() {
  if (ACAN_ESP32::can.receive (inFrame)) {
    if (inFrame.id == 0x0BA) {
        lastRecievedMillis = millis();
        isSending = true;
    }
  }
}

void sendModuleVoltage(uint16_t frameId) {
  outFrame.id = frameId;
  outFrame.ext = 0;

  outFrame.data[1] = ((cellVolatge - 1000 )  & 0xF) << 4;
  outFrame.data[2] = (cellVolatge - 1000 ) >> 4 & 0xFF;
  
  //cell2
  outFrame.data[3] = (cellVolatge - 1000 )  & 0xFF;
  outFrame.data[4] = (cellVolatge - 1000 ) >> 8 & 0xF;

  //cell 3
  outFrame.data[4] =  (((cellVolatge - 1000 )  & 0xF) << 4) + outFrame.data[4];
  outFrame.data[5] = (cellVolatge - 1000 >> 4 ) & 0xFF;

  //cell 4
  outFrame.data[6] = (cellVolatge - 1000 )  & 0xFF;
  outFrame.data[7] = (cellVolatge - 1000 ) >> 8 & 0xF;

  ACAN_ESP32::can.tryToSend(outFrame);

}

//send a modules messages
void sendModuleVoltages(uint8_t moduleNumber) {
  sendModuleVoltage(moduleAddresses[moduleNumber]);
  sendModuleVoltage(moduleAddresses[moduleNumber] + 1);
  sendModuleVoltage(moduleAddresses[moduleNumber] + 2);
}

void sendCan() {
  if (isSending) {
    sendModuleVoltages(currentModule);

    if (currentModule > moduleCount) {
      currentModule = 1;
    }
  }
}


void loop() {
  // put your main code here, to run repeatedly:
  canCheck();
  sendCan();
}
