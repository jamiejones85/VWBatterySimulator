#include <Ticker.h>
#ifndef ARDUINO_ARCH_ESP32
  #error "Select an ESP32 board" 
#endif

#include <ACAN_ESP32.h>
#define LED_BUILTIN 2
#define MAX_CELL_COUNT 96

const int BUFF_SIZE = 32; // make it big enough to hold your longest command

volatile bool isSending = false; 
bool isForced = false;
unsigned long lastRecievedMillis = 0;

CANMessage inFrame;
CANMessage outFrame;

Ticker timeoutTicker; 

uint8_t moduleCount = 8; //default to one pack
uint8_t currentModule = 1;
uint8_t currentModuleMessage = 0;//4 messages per module, send 1 at a time

uint16_t cellVolatge = 3800;
uint16_t moduleTemperature = 20;
uint16_t cellVoltages[MAX_CELL_COUNT];

uint16_t moduleAddresses[] = { 0x1B0, 0x1B4, 0x1B8, 0x1BC, 0x1C0, 0x1C4, 0x1C8, 0x1CC };
uint32_t feedbackAddresses[] = { 0x1A555401, 0x1A555402, 0x1A555403, 0x1A555404, 0x1A555405, 0x1A555406, 0x1A555407, 0x1A555408 };
volatile uint16_t balancingStatus[] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};//each module of 12 has a isBalancing flag, 0x01 is cell 1 0x800 is cell 12

void printHelp() {
  Serial.println(F("Commands"));
  Serial.println(F("h - prints this message"));
  Serial.println(F("S - prints the pack status"));
  Serial.println(F("F - Force Sending even if 0x0BA not recieved"));
  Serial.println(F("V - Sets the mV of all the cells"));
  Serial.println(F("Vx - Sets the mV of a cell in position given by x"));
  Serial.println(F("Bx - Sets the balance resistor status 1 or 0 of cell x"));
  Serial.println(F("M - Mock balance message"));

  Serial.println(F("T - Sets the tempearture of modules"));

  Serial.println(F("P - Sets the Pack Count, e.g if paralleling packs and using a MITM"));
  Serial.println(F("Example: V=3800 - sets all cells to 3800mV"));
  Serial.println(F("Example: V0=3900 - set cell 0 to 3900mV"));

}

void printSettings() {
    Serial.println(F("Current Settings"));
    Serial.print(F("V: "));
    Serial.print(cellVolatge);
    Serial.println(F("mV"));

    Serial.print(F("T: "));
    Serial.print(moduleTemperature);
    Serial.println(F("C"));

    Serial.print(F("Cell Count: "));
    Serial.println(moduleCount * 12);
}

bool isCellBalancing(uint8_t cellIndex, uint8_t moduleIndex) {

  //uint16_t enableFlag = 0x1 << (cellIndex);
  uint16_t enabledFlag = (balancingStatus[moduleIndex] >> (cellIndex)) & 0x1;
  return enabledFlag == 0x1;
}

void printStatus() {
  //print the status of each module
  for( uint8_t i = 1; i < moduleCount + 1; i++) {
    Serial.print(F("Module: "));
    Serial.print(i);
    Serial.print(F(" Temperature: "));
    Serial.println(moduleTemperature);
    for(uint8_t cellIndex = 0; cellIndex < 12; cellIndex++) {
      uint8_t voltageIndex = ((i - 1) * 12) + cellIndex;
      Serial.print(cellIndex);
      Serial.print(F(": "));
      Serial.print(cellVoltages[voltageIndex]);
      Serial.print(F("mV "));
      if (isCellBalancing(cellIndex, i-1)) {
        Serial.print(F("B "));
      }
      if (cellIndex == 6) {
        Serial.println();
      }
    }
    Serial.println();
    Serial.println();

  }
}

void setAllCellVoltages() {
  for (uint8_t i; i< MAX_CELL_COUNT; i++) {
    cellVoltages[i] = cellVolatge;
  }
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

  setAllCellVoltages();
  
  Serial.println ("VW PHEV Battery Simulator running");
  printHelp();
  printSettings();
}

uint8_t getModuleStartIndexBalanceFrame(CANMessage message) {
  switch(message.id) {
    case (0x1A55540A) : return 0;
    case (0x1A55540B) : return 0;
    case (0x1A55540C) : return 1;
    case (0x1A55540D) : return 1;
    case (0x1A55540E) : return 2;
    case (0x1A55540F) : return 2;
    case (0x1A555410) : return 3;
    case (0x1A555411) : return 2;
    case (0x1A555412) : return 4;
    case (0x1A555413) : return 4;
    case (0x1A555414) : return 5;
    case (0x1A555415) : return 5;
    case (0x1A555416) : return 6;
    case (0x1A555417) : return 6;
    case (0x1A555418) : return 7;
    case (0x1A555419) : return 7;
  }
}

void setCellBalanceStatusFromCanMessage(uint8_t moduleIndex, uint8_t cellIndexInModule, uint8_t dataByte) {
  if (dataByte > 0) {
    //set is balancing
     uint16_t enableFlag = 0x1 << (cellIndexInModule);
     balancingStatus[moduleIndex] = balancingStatus[moduleIndex] | enableFlag;
  } else {
      balancingStatus[moduleIndex] = balancingStatus[moduleIndex] & ~(1 << cellIndexInModule);
  }
}

void handleBalanceFrame(CANMessage message) {
 uint8_t moduleIndex = getModuleStartIndexBalanceFrame(message);
 uint8_t cellCount = 4;
  if ((message.id % 2) == 0) {
    cellCount = 8;
  }
  for (uint8_t i = 0; i < cellCount; i++) {
     uint8_t cellIndexInModule = i;
     if (cellCount == 4) {
      cellIndexInModule = 8 + i;
     }
     //uint8_t moduleIndex, uint8_t cellIndexInModule, uint8_t dataByte
    setCellBalanceStatusFromCanMessage(moduleIndex, cellIndexInModule, message.data[i]);
  }
}

void canCheck() {
  if (ACAN_ESP32::can.receive (inFrame)) {
    if (inFrame.id == 0x0BA) {
        lastRecievedMillis = millis();
        isSending = true;
    }
    //handle balance requests, just one pack for now
    if (inFrame.id >= 0x1A55540A && inFrame.id <= 0x1A555419){
        void handleBalanceFrame(CANMessage inFrame);
    }
  }
}

//add balance feedback
void sendModuleTemperature(uint32_t frameId, uint8_t moduleIndex) {
  outFrame.id = frameId;
  outFrame.ext = 1;
  outFrame.len = 8;
  outFrame.data[0] = (moduleTemperature + 40) * 2;
 
  outFrame.data[1] = 0;
  outFrame.data[2] = balancingStatus[moduleIndex]; 
  outFrame.data[3] = balancingStatus[moduleIndex] >> 8;
  outFrame.data[4] = 0;
  outFrame.data[5] = 0;
  outFrame.data[6] = 0;
  outFrame.data[7] = 0;

  ACAN_ESP32::can.tryToSend(outFrame);

}

void sendModuleVoltage(uint16_t frameId, uint8_t startIndex) {
  outFrame.id = frameId;
  outFrame.ext = 0;
  outFrame.len = 8;
  
  outFrame.data[1] = ((cellVoltages[startIndex] - 1000 )  & 0xF) << 4;
  outFrame.data[2] = (cellVoltages[startIndex] - 1000 ) >> 4 & 0xFF;
  
  //cell2
  outFrame.data[3] = (cellVoltages[startIndex + 1] - 1000 )  & 0xFF;
  outFrame.data[4] = (cellVoltages[startIndex + 1] - 1000 ) >> 8 & 0xF;

  //cell 3
  outFrame.data[4] =  (((cellVoltages[startIndex + 2] - 1000 )  & 0xF) << 4) + outFrame.data[4];
  outFrame.data[5] = (cellVoltages[startIndex + 2 ] - 1000 >> 4 ) & 0xFF;

  //cell 4
  outFrame.data[6] = (cellVoltages[startIndex + 3] - 1000 )  & 0xFF;
  outFrame.data[7] = (cellVoltages[startIndex + 3] - 1000 ) >> 8 & 0xF;

  ACAN_ESP32::can.tryToSend(outFrame);

}


void sendCan() {
  if (isSending || isForced) {
    if (currentModuleMessage < 3) {
        uint16_t messageId = moduleAddresses[currentModule - 1] + currentModuleMessage;
        //get address of cell voltages, there's 3 messages per module, so 4 cells per message
        //current module = 4 current message = 2 so 4 * 12
        uint8_t index = ((currentModule - 1) * 12) + (currentModuleMessage * 4);
        sendModuleVoltage(messageId, index);
    } else if (currentModuleMessage == 3) {
        sendModuleTemperature(feedbackAddresses[currentModule - 1], currentModule - 1);
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

uint8_t getModuleIndex(uint8_t cellNumber) {
   return (cellNumber) / 12;
}

uint8_t getCellInModuleIndex(uint8_t moduleIndex, uint8_t cellNumber) {
   uint8_t moduleStartIndex = moduleIndex * 12;
   return cellNumber - moduleStartIndex;

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
      setAllCellVoltages();
      printSettings();
   } else if (cmdStr == "T") {
      moduleTemperature = valueStr.toInt();
      printSettings();
   } else if (cmdStr == "h") {
    printHelp();
   } else if (cmdStr == "S") {
    printStatus();
   } else if (cmdStr == "F") {
    uint8_t force = valueStr.toInt();
    isForced = force == 1;
    Serial.print("Forced Sending: ");
    Serial.println(isForced);
   } else if (cmdStr.charAt(0) == 'V') {
    cmdStr.remove(0,1);
    uint8_t cellNumber = cmdStr.toInt();
    cellVoltages[cellNumber] = valueStr.toInt();
    Serial.print(F("Setting Cell: "));
    Serial.print(cellNumber);
    Serial.print(F(" to: "));
    Serial.println(valueStr.toInt());
   } else if (cmdStr.charAt(0) == 'B') {
    cmdStr.remove(0,1);
    uint8_t cellNumber = cmdStr.toInt();
    uint8_t moduleIndex = getModuleIndex(cellNumber);
    uint8_t cellIndex = getCellInModuleIndex(moduleIndex, cellNumber);
    Serial.print(F("Module: "));
    Serial.print(moduleIndex);
    Serial.print(F(" Cell: ")); 
    Serial.print(cellIndex);
    if (valueStr.toInt() == 1) {
      uint16_t enableFlag = 0x1 << (cellIndex);
      balancingStatus[moduleIndex] = balancingStatus[moduleIndex] | enableFlag;
      Serial.println(F(" Balance On"));
    } else {
      uint16_t enableFlag = 0x1 << (cellIndex);
      balancingStatus[moduleIndex] = balancingStatus[moduleIndex] ^ enableFlag;
      Serial.println(F(" Balance Off"));
    }
   } else if (cmdStr == "M") {
    CANMessage msg;
    msg.id = 0x1A55540A;
    msg.ext = 1;
    outFrame.len = 8;

    msg.data[0] = 0xC;
    msg.data[1] = 0xC;
    msg.data[2] = 0xC;
    msg.data[3] = 0x0;
    msg.data[4] = 0x0;
    msg.data[5] = 0x0;
    msg.data[6] = 0x0;
    msg.data[7] = 0x0;
    handleBalanceFrame(msg);
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
