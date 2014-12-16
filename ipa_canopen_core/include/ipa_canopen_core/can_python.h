#ifndef CAN_PYTHON_H
#define CAN_PYTHON_H


#include <boost/python.hpp>
#include <ipa_canopen_core/can_enum.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <memory>

extern "C" {
  #include "pci_7841.h"
}

enum TPDO {
  TPDO_INDEX = 0x1800,
  TPDO1_MSG = 0x180,
  TPDO2_MSG = 0x280,
  TPDO3_MSG = 0x380,
  TPDO4_MSG = 0x480
};

enum RDPO {
  RDPO_INDEX = 0x1400,
  RPDO1_MSG = 0x200,
  RPDO2_MSG = 0x300,
  RPDO3_MSG = 0x400,
  RPDO4_MSG = 0x500
};

enum DATA_BLOCK_NUM {
  DATA_BLOCK_1 = 0x60,
  DATA_BLOCK_2 = 0x70
};

using namespace std;

struct CanException : public std::exception {
  CanException(const std::string& msg);

  const char* what() const throw() override;

private:
  std::string msg;
};

struct CanMsg {
  CanMsg();

  int size();
  int canId;
  int rtr;
  boost::python::list data;
};

struct SDOkey {
        uint16_t index;
        uint8_t subindex;

        inline SDOkey(CAN_PACKET m):
            index((m.data[2] << 8) + m.data[1]),
            subindex(m.data[3]) {};

        inline SDOkey(uint16_t i, uint8_t s):
            index(i),
            subindex(s) {};

        bool operator==(const SDOkey& o) {
          if (index == o.index && subindex == o.subindex) return true;
          else return false;
        }

        bool operator!=(const SDOkey& o) {return !(*this == o);}
};

//TPDO MAPPING
const SDOkey TPDO_map(0x1A00, 0x0);
//RPDO MAPPING
const SDOkey RPDO_map(0x1600, 0x0);


class CanPort {
public:
  CanPort(int cardNumber, int portNumber);

  void open();
  void setDebugEnabled(bool enabled);
  void sendMsg(int canId, const boost::python::list& data, int rtr = 0);
  CanMsg rcvMsg();
  CAN_PACKET rcvMsgImpl();
  void requestDataBlock(uint8_t canId, DATA_BLOCK_NUM dataBlockNum);
  void configPort(CANMODE mode, DWORD accCode,
                  DWORD accMask, CANBAUDRATE baudrate);
  void cleanBuffers();
  void cleanTxBuffer();
  void cleanRxBuffer();
  void close();
  void resetCommunication(u_int8_t nodeId = 0);
  void resetNode(u_int8_t nodeId = 0);
  void startRemoteNode(u_int8_t nodeId = 0);
  void sendNMT(uint8_t command, uint8_t CANid = 0);
  void controlPDO(uint8_t CANid, u_int16_t control1, u_int16_t control2);
  void uploadSDO(uint8_t CANid, SDOkey sdo);
  void initSendSegmentedSdo(uint8_t CANid, SDOkey sdo);
  void sendSDO16(uint8_t CANid, SDOkey sdo,
                 uint16_t value, bool sizeIndicated = true);
  void sendSDO(uint8_t CANid, SDOkey sdo,
               uint32_t value, bool sizeIndicated = true);
  void clearTPDOMapping(uint8_t id, int object);
  void clearRPDOMapping(uint8_t id, int object);

private:
  void sendCanPacket(CAN_PACKET& msg);
  void printCanPacketData(const CAN_PACKET& msg);
  void isPortOpenedAssert(bool opened);
  void makeTPDOMapping(uint8_t id, int object,
                       std::vector<std::string> registers,
                       std::vector<int> sizes, u_int8_t sync_type);
  void makeRPDOMapping(uint8_t id, int object,
                       std::vector<std::string> registers,
                       std::vector<int> sizes , u_int8_t sync_type);
  void enableTPDO(uint8_t id, int object);
  void enableRPDO(uint8_t id, int object);
  unsigned int getCOBType(unsigned long cobId);
  unsigned int getCanId(long cobId);

private:
  int cardNumber;
  int portNumber;
  bool opened;
  int handle;
  bool debugEnabled;
};

#endif // CAN_PYTHON_H
