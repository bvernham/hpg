#ifndef __CANBUS_H__
#define __CANBUS_H__
//#define CAN_Setup_Debug//print debug statement for CAN startup
//#define CAN_Recv_Debug//debug for CAN received messages
#define CAN_Active_Report//report when CAN becomes active or time out
#define GPS_Config_Debug true//allow verbose config message output
//#define Esf_Meas_Speed_Debug//output EFS_Meas_Speed to CDC
#define mph2mmpsec 447.04 //convert mph to mm/set
/*
   Copyright 2022 by Michael Ammann (@mazgch)

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
#include "GNSS.h"

// Arduino CAN by sandeepmistry, version 0.3.1
// Library Manager:   http://librarymanager/All#arduino-CAN
// Github Repository: https://github.com/sandeepmistry/arduino-CAN
//#include <CAN.h>
//elapsedMillis by pfeerick
//Github Repository: https://github.com/pfeerick/elapsedMillis
#include <elapsedMillis.h>

//const int CAN_TEST_ID  = 0x55;
//const int CAN_SPEED_ID = 0x55;
#define rxPin GPIO_NUM_4
#define txPin GPIO_NUM_5
#define dataTimeOutlimit 2000//500//zero out CAN variables
#define CAN0Baud 500000
#define SpeedID 0x3E9
#define Speed_Scale 0.01
#define DirID 0x135
//#define mph2mmpsec 447.04 //convert mph to mm/set
//Call back function prototypes
void Calc_Speed(CAN_FRAME *frame);//Callback function prototype to calculate vehicle speed
void Calc_Direct(CAN_FRAME *frame);//Callback function prototype to calculate transmission gear position

class CANBUS {
  public:
    elapsedMillis dataTimeOut = 0;//resets when CAN messages is recieved
    bool dataActive = false;//indicates data interval < timeout
    float Veh_Speed = 0;//MPH
    uint8_t Veh_Dir = 0;//
    uint32_t Speed_Time = 0;//mills of when the speed CAN message is recieved.
    void init() {
      /*// start the CAN bus at 500kbps
        const int freq = 500E3; // use 250kbps in the Logic analyzer? unclear why
        CAN.setPins(CAN_RX, CAN_TX);
        if (!CAN.begin(freq)) {
        Log.warning("CAN init failed");
        } else {
        Log.info("CAN init %d successful", freq);
        }
        CAN.observe();
        // ATTENTION the callback is executed from an ISR only do essential things
        // CAN.onReceive(onPushESFMeas); */
#ifdef  CAN_Setup_Debug
      Log.info("CAN Start Up!");
#endif
      // void setCANPins(gpio_num_t rxPin, gpio_num_t txPin);
      CAN0.setCANPins(rxPin, txPin);
      CAN0.begin(CAN0Baud);
      CAN0.watchFor(SpeedID, 0xFFF); //setup a special filter
      CAN0.watchFor(DirID, 0xFFF); //setup a special filter
      CAN0.setCallback(0, Calc_Speed); //callback on that Vehicle Speed filter*/
      CAN0.setCallback(1, Calc_Direct); //callback on that Vehicle Direction filter*/
    }
    void poll()//check for data time out
    {
      if (dataTimeOut > dataTimeOutlimit)
      {
        Veh_Speed = 0;//MPH
        Veh_Dir = 0;//
        Speed_Time = millis();//use time when the speed message was recieved.
        Gnss.sendEsfMeasSpeed(Veh_Speed, Veh_Dir);
#ifdef CAN_Active_Report
        Log.info("CAN Timeout!");
#endif
        dataActive = false;

      }
    }
    void printFrame(CAN_FRAME *message)
    {
      char txt[50] = "";//0x##_ 5 * 8
      if (!message->rtr) {
        char space[] = " ";
        for (int i = 0; i < message->length; i ++) {
          char buff[6] = "";//temp buff for hex char
          sprintf(buff,"0x%02X",message->data.byte[i]);
          strncat (txt, buff, 45);
          strncat (txt, space, 45);
        }
      }
      else {
        //packetSize = message->length;
        strcpy(txt, "RTR");
      }
      Log.info("CAN read 0x%0*X %d %s", message->extended ? 8 : 3, message->id, message->length, txt);
      //Log.info("CAN read 0x%0*X %d \"%s\"", CAN.packetExtended() ? 8 : 3, CAN.packetId(), packetSize, txt);

    }

    /*void poll() {
      int packetSize;
      while (0 != (packetSize = CAN.parsePacket())) {
        onPushESFMeas(packetSize);
      }
      //writeTestPacket(); // need to remove the CAN.observe mode above
      }*/

    //protected:
    /*void onPushESFMeas(int packetSize) {
      if (!CAN.packetRtr() && (CAN.packetId() == CAN_SPEED_ID)) {
        uint32_t ttag = millis();
        uint8_t packet[packetSize];
        for (int i = 0; i < packetSize; i ++) {
          packet[i] = CAN.read();
        }
        uint32_t speed = ((0xF & packet[1]) << 8) + packet[0];  // speed  12 bits from 0
        bool reverse = (0x10 & packet[1]) >> 4;        // bit 12 == moving reverse
        Gnss.sendEsfMeas(ttag, speed, reverse);
      }
      }

      #if 0
      void onPacketDump(int packetSize) {
      char txt[packetSize+1] = "";
      if (!CAN.packetRtr()) {
        for (int i = 0; i < packetSize; i ++) {
          char ch = CAN.read();
          txt[i] = isprint(ch) ? ch : '?';
        }
      }
      else {
        packetSize = CAN.packetDlc();
        strcpy(txt, "RTR");
      }
      Log.info("CAN read 0x%0*X %d \"%s\"", CAN.packetExtended() ? 8 : 3, CAN.packetId(), packetSize, txt);
      }

      void writeTestPacket(void) {
      int id = CAN_TEST_ID;
      uint8_t test[9];
      static uint8_t cnt = 0;
      int len = sprintf((char*)test, "mazgch%02X", cnt++);
      CAN.beginPacket(id);
      CAN.write(test, len);
      CAN.endPacket();
      Log.info("CAN write 0x%03X %d \"%s\"", id, len, test);
      }
      #endif */
    //private:
};
CANBUS Canbus;

//Must add declarations to the GNSS class
void GNSS::setEsfMeasSpeed() {
  bool ok = rx.setVal8(UBLOX_CFG_SFODO_USE_SPEED, 1, VAL_LAYER_RAM); // Set F9 to use UBX Speed message input
  if (ok && GPS_Config_Debug) Log.info("setVal8 UBLOX_CFG_SFODO_USE_SPEED == 1");
}
void GNSS::sendEsfMeasSpeed(float speed, uint8_t dir) {
  //uint8_t tempDir = (0x03 & signals[sig1].results.uint);
  //VAL_TABLE_ PRNDL 3 "Reverse" 2 "Drive" 1 "Neutral" 0 "Park" ;

  /*int32_t speedout = 0;
    if (dir == 0)
    {
    speedout = 0;
    }
    else
    {
    speedout = lroundf(speed);//speed is now mph2mmpsec
    if (dir == 3)
    {
      speedout = -1 * speedout;
    }
    }*/
  UBX_ESF_MEAS_data_t message;
  memset(&message, 0, sizeof(message));
  message.timeTag = Canbus.Speed_Time;//millis();
  message.flags.bits.numMeas = 1;
  message.id = 4;//(1<<3)
  if (dir == 0)
  {
    message.data[0].data.bits.dataField = 0;
  }
  else
  {
    message.data[0].data.bits.dataField = (lroundf(speed) & 0xFFFFF);//speed is now mph2mmpsec
    if (dir == 3)
    {
      message.data[0].data.bits.dataField = ~message.data[0].data.bits.dataField + 1;
    }
  }
  //message.data[0].data.bits.dataField = (speedout & 0x7FFFFF);
  message.data[0].data.bits.dataType = 11; // 11 = Speed
  ubxPacket packetEsfMeas = {UBX_CLASS_ESF, UBX_ESF_MEAS, 12, 0, 0, (uint8_t*)&message,
                             0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED
                            };
  rx.sendCommand(&packetEsfMeas, 0); // don't expect ACK
#ifdef Esf_Meas_Speed_Debug
  Log.info("Send Esf_Meas_Speed: Time: %u ID: %u Data Type: %u Data Field: 0x%06X" , message.timeTag, message.id, message.data[0].data.bits.dataType,
           message.data[0].data.bits.dataField);
#endif
}
void Calc_Speed(CAN_FRAME *frame)//Calculate vehicle speed as mm/sec.
{
  //  message->id
  //  message->extended
  //  message->length
  //  message->data
  /* BO_ 1001 ECMVehicleSpeed: 8 K20_ECM
    SG_ VehicleSpeed : 7|16@0+ (0.01,0) [0|0] "mph" NEO
  */
  Canbus.Speed_Time = millis();//use time when the speed message was recieved.
  if (frame->id == SpeedID)
  {
#ifdef CAN_Recv_Debug
    Canbus.printFrame(frame);
#endif
    Canbus.Veh_Speed = (mph2mmpsec * Speed_Scale) * ((frame->data.byte[0] << 8) + frame->data.byte[1]);
    Gnss.sendEsfMeasSpeed(Canbus.Veh_Speed, Canbus.Veh_Dir);
    if (!Canbus.dataActive)
    {
#ifdef CAN_Active_Report
      Log.info("CAN Active!");
#endif
      Canbus.dataActive = true;
    }
    Canbus.dataTimeOut -= Canbus.dataTimeOut;//reset watchdog timer
  }
#ifdef CAN_Recv_Debug
  else
  {
    Log.info("Recieved wrong frame: 0x%0*X %d" , frame->extended ? 8 : 3, frame->id);
    //Log.info("Got wrong frame: ");
    //Log.info(frame->id, HEX);
  }
#endif
}
void Calc_Direct(CAN_FRAME *frame)
{
  /*  BO_ 309 ECMPRDNL: 8 K20_ECM
     SG_ PRNDL : 2|3@0+ (1,0) [0|0] "" NEO
     SG_ ESPButton : 4|1@0+ (1,0) [0|1] "" Vector__XXX
  */
  //VAL_TABLE_ PRNDL 3 "Reverse" 2 "Drive" 1 "Neutral" 0 "Park" ;
  if (frame->id == DirID)
  {
#ifdef CAN_Recv_Debug
    Canbus.printFrame(frame);
#endif
    Canbus.Veh_Dir = 0x07 & (frame->data.byte[0]);//first 3 bits
    if (!Canbus.dataActive)
    {
#ifdef CAN_Active_Report
      Log.info("CAN Active!");
#endif
      Canbus.dataActive = true;
    }
    Canbus.dataTimeOut -= Canbus.dataTimeOut;//reset watchdog timer
  }
#ifdef CAN_Recv_Debug
  else
  {
    Log.info("Recieved wrong frame: 0x%0*X %d" , frame->extended ? 8 : 3, frame->id);
    //Log.info("Got wrong frame: ");
    //Log.info(frame->id, HEX);
  }
#endif
}

#endif // __CANBUS_H__
