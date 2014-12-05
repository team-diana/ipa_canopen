#ifndef CAN_ENUM_H
#define CAN_ENUM_H

//   setPort.mode     = 0;       //  0 : 11-bit ;  1 : 29-bit CAN network
//   setPort.accCode  = 0;     //  Only ID = accCode can pass the filter
//   setPort.accMask  = 0x7ff;   //  Don't care bit
//   setPort.baudrate = 0;       //  0: 125kBps; 1:250kBps;  2:500kBps; 3:1MBps

enum CANMODE {
  BIT11 = 0,
  BIT29 = 29
};

enum CANBAUDRATE {
  CAN_BAUD_1M = 3,
  CAN_BAUD_500K = 2,
  CAN_BAUD_250K = 1,
  CAN_BAUD_125K = 0,
};


enum NMTMESSAGES {
  NMT_START_REMOTE_NODE = 0x01,
  NMT_STOP_REMOTE_NODE = 0x02,
  NMT_ENTER_PRE_OPERATIONAL = 0x80,
  NMT_RESET_NODE = 0x81,
  NMT_RESET_COMMUNICATION = 0x82
};



#endif // CAN_ENUM_H



