#include <Ticker.h>
#ifndef ARDUINO_ARCH_ESP32
  #error "Select an ESP32 board" 
#endif

#include <ACAN_ESP32.h>
#define LED_BUILTIN 2

const int BUFF_SIZE = 32; // make it big enough to hold your longest command

volatile bool isSending = false; 
unsigned long lastRecievedMillis = 0;

CANMessage inFrame;
CANMessage outFrame;

Ticker timeoutTicker; 

uint8_t moduleCount = 8; //default to one pack
uint8_t currentModule = 1;
uint8_t currentModuleMessage = 0;//4 messages per module, send 1 at a time

uint16_t cellVolatge = 3800;
uint16_t moduleTemperature = 20;

uint16_t moduleAddresses[] = { 0x1B0, 0x1B4, 0x1B8, 0x1BC, 0x1C0, 0x1C4, 0x1C8, 0x1CC };
uint32_t feedbackAddresses[] = { 0x1A555401, 0x1A555402, 0x1A555403, 0x1A555404, 0x1A555405, 0x1A555406, 0x1A555407, 0x1A555408 };

void printHelp() {
  Serial.println(F("Commands"));
  Serial.println(F("h - prints this message"));
  Serial.println(F("V - Sets the mV of all the cells"));
  Serial.println(F("T - Sets the tempearture of modules"));

  Serial.println(F("P - Sets the Pack Count, e.g if paralleling packs and using a MITM"));
  Serial.println(F("Example: V=3800 - sets all cells to 3800mV"));
}

void printSettings() {
    Serial.println(F("Current Settings"));
    Serial.print(F("V: "));
    Serial.print(cellVolatge);
    Serial.println(F("mV"));

    Serial.print(F("T: "));
    Serial.print(moduleTemperature);
    Serial.println(F("C"));
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
  timeoutTicker.attach(0.1, sendCan);

  
  Serial.println ("VW PHEV Battery Simulator running");
  printHelp();
  printSettings();
}


void canCheck() {
  if (ACAN_ESP32::can.receive (inFrame)) {
    if (inFrame.id == 0x0BA) {
        lastRecievedMillis = millis();
        isSending = true;
    }
  }
}

void sendModuleTemperature(uint32_t frameId) {
  outFrame.id = frameId;
  outFrame.ext = 1;
  outFrame.len = 8;
  outFrame.data[0] = (moduleTemperature + 40) * 2;
  //report other 2 readings one degree above and below
  outFrame.data[1] = (moduleTemperature + 41) * 2;
  outFrame.data[2] = (moduleTemperature + 39) * 2;
  outFrame.data[3] = 0;
  outFrame.data[4] = 0;
  outFrame.data[5] = 0;
  outFrame.data[6] = 0;
  outFrame.data[7] = 0;

  ACAN_ESP32::can.tryToSend(outFrame);

}

void sendModuleVoltage(uint16_t frameId) {
  outFrame.id = frameId;
  outFrame.ext = 0;
  outFrame.len = 8;
  
  outFrame.data[0] = 0;
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


void sendCan() {
  if (isSending) {
    if (currentModuleMessage < 3) {
        uint16_t messageId = moduleAddresses[currentModule - 1] + currentModuleMessage;
        sendModuleVoltage(messageId);
    } else if (currentModuleMessage == 3) {
        sendModuleTemperature(feedbackAddresses[currentModule - 1]);
    }

    currentModuleMessage++;
    if (currentModuleMessage > 3) {
      currentModuleMessage = 0;
      currentModule++;
    }
    
    if (currentModule > moduleCount) {
      currentModule = 1;
    }
  }
}

void handleReceivedMessage(char *msg)
{
   String cmdStr;
   String valueStr;
   bool equalsFound = false;

   for (uint8_t i = 0; i < BUFF_SIZE + 1;  i++) {

      if (msg[i] == '=') {
        equalsFound = true;
      }

      if (msg[i] == '\0') {
        break;
      }

      if (!equalsFound) {
        cmdStr += msg[i];
      } else if(msg[i] != '=') {
        valueStr += msg[i];
      }
   }

   if (cmdStr == "V") {
      cellVolatge = valueStr.toInt();
      printSettings();
   } else if (cmdStr == "T") {
      moduleTemperature = valueStr.toInt();
      printSettings();
   }
}

void handleSerial()
{
    static char buffer[BUFF_SIZE+1]; // +1 allows space for the null terminator
    static int length = 0; // number of characters currently in the buffer

    if(Serial.available())
    {
        char c = Serial.read();
        if((c == '\r') || (c == '\n'))
        {
            // end-of-line received
            if(length > 0)
            {
                handleReceivedMessage(buffer);
            }
            length = 0;
        }
        else
        {
            if(length < BUFF_SIZE)
            {
                buffer[length++] = c; // append the received character to the array
                buffer[length] = 0; // append the null terminator
            }
            else
            {
                // buffer full - discard the received character
            }
        }
    }
}


void loop() {
  // put your main code here, to run repeatedly:
  canCheck();
  handleSerial();
}
