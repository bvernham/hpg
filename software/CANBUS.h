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

#ifndef __CANBUS_H__
#define __CANBUS_H__
//#define CAN_Setup_Debug//print debug statement for CAN startup
//#define CAN_Recv_Debug//debug for CAN received messages
#define CAN_Active_Report//report when CAN becomes active or time out
#define GPS_Config_Debug true//allow verbose config message output
//#define Esf_Meas_Speed_Debug//output EFS_Meas_Speed to CDC
#define mph2mmpsec 447.04 //convert mph to mm/set																
// Arduino CAN by sandeepmistry, version 0.3.1
// Library Manager:   http://librarymanager/All#arduino-CAN
// Github Repository: https://github.com/sandeepmistry/arduino-CAN
//#include <CAN.h>
//elapsedMillis by pfeerick
//Github Repository: https://github.com/pfeerick/elapsedMillis
#include <elapsedMillis.h>
/*  https://github.com/commaai/opendbc/
    has lots of information on different car models. Extracting CAN data is highly
    vehicle specific and not standardized. You may need to change the code if ticks
    should be used or data spreads accross two messages.

    e.g. BMW https://github.com/commaai/opendbc/blob/master/bmw_e9x_e8x.dbc
*/
//const long CAN_FREQ     = 500000;
//const int CAN_MESSAGE_ID = 416 /* BMW*/;
//#define CAN_SPEED(p)      (1e6/3600 /* kmh => mm/s */ *  0.103/* unit => km/h */ * (((p[1] & 0xF) << 8) | p[0]))
//#define CAN_REVERSE(p)    (0x10 == (p[1] & 0x10))
#define CAN_ESF_MEAS_TXO LTE_DTR  // -> make a connection from this pin to ZED-RXI 

extern class CANBUS Canbus;

//#define Use_CBuf //define if circular buffer is to be used//if not defined will send ESF_MEAS from speed ISR
#define SPD_BUFF_SIZE 10//!< ESF-MEAS Circular Buffer Size
#define rxPin GPIO_NUM_4
#define txPin GPIO_NUM_5
#define dataTimeOutlimit 2000//500//zero out CAN variables
#define CAN0Baud 500000
#define SpeedID 0x3E9
#define Speed_Scale 0.01
#define DirID 0x135
#define mph2mmpsec 447.04 //convert mph to mm/set
//Call back function prototypes
//void Calc_Speed(CAN_FRAME *frame);//Callback function prototype to calculate vehicle speed
//void Calc_Direct(CAN_FRAME *frame);//Callback function prototype to calculate transmission gear position

class CANBUS {
  public:
    elapsedMillis dataTimeOut = 0;//resets when CAN messages is recieved
    bool dataActive = false;//indicates data interval < timeout
    void init() {
      Serial2.begin(38400, SERIAL_8N1, -1/*no input*/, CAN_ESF_MEAS_TXO);
      CAN0.setCANPins(rxPin, txPin);
      CAN0.begin(CAN0Baud);
      CAN0.watchFor(SpeedID, 0xFFF); //setup a special filter
      CAN0.watchFor(DirID, 0xFFF); //setup a special filter
      CAN0.setCallback(0, Calc_Speed); //callback on that Vehicle Speed filter*/
      CAN0.setCallback(1, Calc_Direct); //callback on that Vehicle Direction filter*/
#ifdef Use_CBuf
      memset(&Esf_CBuf, 0, sizeof(Esf_CBuf)); //Clear circular buffer if used
#endif
    }
    void poll()//check for data time out
    {
#ifdef Use_CBuf
      sendEsfMeas();
#endif
      if (Canbus.dataTimeOut > dataTimeOutlimit)
      {
        Canbus.PRNDL = 0;
#ifdef CAN_Active_Report
        Log.info("CAN Timeout!");
#endif
        dataActive = false;

      }
    }
  protected:
#ifdef Use_CBuf
    typedef struct {
      uint32_t ttag[SPD_BUFF_SIZE];
      uint32_t data[SPD_BUFF_SIZE];//data is already encoded and ready to be sent
      bool ready;
      uint8_t head;//new buffer entries are placed
      uint8_t tail;//old buffer entries are removed
    } ESF_CBUF_STRUCT;
    ESF_CBUF_STRUCT Esf_CBuf;
    bool pushEsfMeas(uint32_t ttag, uint32_t data) {
      bool over_run = false;
      Esf_CBuf.ttag[Esf_CBuf.head] = ttag;
      Esf_CBuf.data[Esf_CBuf.head] = data;
      Esf_CBuf.head++;//Increment
      if (Esf_CBuf.head >= SPD_BUFF_SIZE) Esf_CBuf.head = 0; //Reset to beginning of array
      if ((Esf_CBuf.tail == Esf_CBuf.head) && Esf_CBuf.ready) over_run = true;
      //if head == tail and still have data then we have we are overwriting unsent speed data
      Esf_CBuf.ready = true;//We have new data
      return over_run;
    }
    void sendEsfMeas(void) {
      if (Esf_CBuf.ready) {
        if (!esfMeas(Esf_CBuf.ttag[Esf_CBuf.tail], &Esf_CBuf.data[Esf_CBuf.tail], 1)) {
          Log.warning("Esf_CBuf failed to send!");
        }
        Esf_CBuf.tail++;//Increment
        if (Esf_CBuf.tail >= SPD_BUFF_SIZE)Esf_CBuf.tail = 0; //Reset to beginning
        if (Esf_CBuf.tail == Esf_CBuf.head)Esf_CBuf.ready = false;
        //if  they are the same we caught up so no new data
        //Esf_CBuf.ready = false;
      }
    }
#endif
    uint8_t PRNDL = 0; //VAL_TABLE_ PRNDL 3 "Reverse" 2 "Drive" 1 "Neutral" 0 "Park" ;
    void printFrame(CAN_FRAME *message)
    {
      char txt[50] = "";//0x##_ 5 * 8
      if (!message->rtr) {
        char space[] = " ";
        for (int i = 0; i < message->length; i ++) {
          char buff[6] = "";//temp buff for hex char
          sprintf(buff, "0x%02X", message->data.byte[i]);
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
    // ATTENTION the callback is executed from an ISR only do essential things
    /*  static void onPushESFMeasFromISR(int packetSize) {
        if (!CAN.packetRtr() && (CAN.packetId() == CAN_MESSAGE_ID) && (packetSize <= 8)) {
          uint32_t ms = millis();
          uint8_t packet[packetSize];
          for (int i = 0; i < packetSize; i ++) {
            packet[i] = CAN.read();
          }
          uint32_t speed = CAN_SPEED(packet);
          bool reverse = CAN_REVERSE(packet);
          speed = reverse ? -speed : speed;*/
    //      ESF_QUEUE_STRUCT meas = { ms, (11/*SPEED*/ << 24)  | (speed & 0xFFFFFF) };
    //      BaseType_t xHigherPriorityTaskWoken;
    //      if (xQueueSendToBackFromISR(Canbus.queue,&meas,&xHigherPriorityTaskWoken) == pdPASS ) {
    //        if(xHigherPriorityTaskWoken){
    //          portYIELD_FROM_ISR();
    //        }
    //      }
    //    }
    //  }
    static void Calc_Speed(CAN_FRAME *frame)//Calculate vehicle speed as mm/sec.
    {
      //  message->id
      //  message->extended
      //  message->length
      //  message->data
      /* BO_ 1001 ECMVehicleSpeed: 8 K20_ECM
        SG_ VehicleSpeed : 7|16@0+ (0.01,0) [0|0] "mph" NEO
      */
      if (frame->id == SpeedID)
      {
        uint32_t ms = millis();
        //#ifdef CAN_Recv_Debug
        //        Canbus.printFrame(frame);
        //#endif
        //VAL_TABLE_ PRNDL 3 "Reverse" 2 "Drive" 1 "Neutral" 0 "Park" ;
        uint32_t speed = 0;
        bool reverse = false;
        if (Canbus.PRNDL == 0)
        {
          speed = 0;
        }
        else
        {
          speed = lroundf(((mph2mmpsec * Speed_Scale) * ((frame->data.byte[0] << 8) + frame->data.byte[1])));
          if (Canbus.PRNDL == 3) reverse = true;
          speed = reverse ? -speed : speed;
        }
        speed = (((11/*SPEED*/ << 24)  | (speed & 0xFFFFFF)));//finish ESF_Meas encoding in speed
#ifdef Use_CBuf //push to cbuf
        if (Canbus.pushEsfMeas(ms, speed)) Log.warning("esfData Buffer Overrun!");
#else//send esfMeas directly in ISR         
        if (!Canbus.esfMeas(ms, &speed, 1)) {
          Log.warning("Esf_CBuf failed to send!");
        }
#endif
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
    static void Calc_Direct(CAN_FRAME *frame)
    {
      /*  BO_ 309 ECMPRDNL: 8 K20_ECM
         SG_ PRNDL : 2|3@0+ (1,0) [0|0] "" NEO
         SG_ ESPButton : 4|1@0+ (1,0) [0|1] "" Vector__XXX
      */
      //VAL_TABLE_ PRNDL 3 "Reverse" 2 "Drive" 1 "Neutral" 0 "Park" ;
      if (frame->id == DirID)
      {
        //#ifdef CAN_Recv_Debug
        //        Canbus.printFrame(frame);
        //#endif
        Canbus.PRNDL = 0x07 & (frame->data.byte[0]);//first 3 bits
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
    size_t esfMeas(uint32_t ttag, uint32_t* p, size_t num) {
      size_t i = 0;
      uint8_t m[6 + 8 + (4 * num) + 2];
      m[i++] = 0xB5; // µ
      m[i++] = 0x62; // b
      m[i++] = 0x10; // ESF
      m[i++] = 0x02; // MEAS
      uint16_t esfSize = (8 + 4 * num);
      m[i++] = esfSize >> 0;
      m[i++] = esfSize >> 8;
      m[i++] = ttag >> 0;
      m[i++] = ttag >> 8;
      m[i++] = ttag >> 16;
      m[i++] = ttag >> 24;
      uint16_t esfFlags = num << 11;
      m[i++] = esfFlags >> 0;
      m[i++] = esfFlags >> 8;
      uint16_t esfProvider = 0;
      m[i++] = esfProvider >> 0;
      m[i++] = esfProvider >> 8;
      for (int s = 0; s < num; s ++) {
        m[i++] = p[s] >> 0;
        m[i++] = p[s] >> 8;
        m[i++] = p[s] >> 16;
        m[i++] = p[s] >> 24;

      }
      uint8_t cka = 0;
      uint8_t ckb = 0;
      for (int c = 2; c < i; c++) {
        cka += m[c];
        ckb += cka;
      }
      m[i++] = cka;
      m[i++] = ckb;
      return Serial2.write(m, i);
    }

};

CANBUS Canbus;

#endif // __CANBUS_H__
