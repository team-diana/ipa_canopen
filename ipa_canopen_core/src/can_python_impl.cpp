
#include <boost/python.hpp>
#include <ipa_canopen_core/can_enum.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <memory>
#include "ipa_canopen_core/can_python.h"

extern "C" {
  #include "pci_7841.h"
}

using namespace std;

CanException::CanException(const string& msg)
{
    cerr << "new exception: " << msg << endl;
}

const char* CanException::what()  const throw()
{
    return msg.c_str();
}

CanMsg::CanMsg(): canId(0), rtr(0) {

}

int CanMsg::size()
{
  return len(data);
}

CanPort::CanPort(int cardNumber, int portNumber) :
  cardNumber(cardNumber),
  portNumber(portNumber),
  opened(false),
  handle(0),
  debugEnabled(true) {

  }

void CanPort::open()
{
    if(debugEnabled) {
      cout << "trying to open..." << endl;
    }
    isPortOpenedAssert(false);
    if((handle = CanOpenDriver(cardNumber, portNumber)) == -1) {
      cerr << "unable to open" << endl;
      throw new CanException("Unable to open can port");
    } else {
      opened = true;
      if(debugEnabled) {
        cout << "port opened." << endl;
      }
    }
}


void CanPort::setDebugEnabled(bool enabled)
{
    debugEnabled = enabled;
}

void CanPort::sendMsg(int canId, const boost::python::list& data, int rtr)
{
    CAN_PACKET canPacket;
    memset(&canPacket, 0, sizeof(canPacket));
    canPacket.CAN_ID = canId;
    canPacket.rtr = rtr;
    canPacket.len = len(data); /// WARNING: we assume that each element in `data' is a byte

    for (int i = 0; i < len(data); ++i)
    {
      canPacket.data[i] = boost::python::extract<BYTE>(data[i]) ;
    }

    sendCanPacket(canPacket);
  }

CAN_PACKET CanPort::rcvMsgImpl() {
    CAN_PACKET canPacket = {0};
    if (CanRcvMsg(handle, &canPacket) == 0) {
      if(debugEnabled) {
        printf("receiving data: ");
        std::cout << " -- COB-ID:  " << canPacket.CAN_ID <<
                    " COB-type:  " << getCOBType(canPacket.CAN_ID) <<
                    " can-id:  " << getCanId(canPacket.CAN_ID) << " -- ";
        printCanPacketData(canPacket);
      }
    } else {
      if(debugEnabled) {
        std::cout << "no data received" << std::endl;
      }
      CAN_PACKET empty = {0};
      canPacket = empty;
    }

   return canPacket;
}

CanMsg CanPort::rcvMsg()
{
  CAN_PACKET canPacket = rcvMsgImpl();
  CanMsg canMsg;
  canMsg.canId = canPacket.CAN_ID;
  canMsg.rtr = canPacket.rtr;
  for (int i = 0; i < canPacket.len; ++i)
  {
    canMsg.data.append<BYTE>(canPacket.data[i]);
  }
  return canMsg;
}

void CanPort::requestDataBlock(uint8_t canId, DATA_BLOCK_NUM dataBlockNum)
{
    CAN_PACKET msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.CAN_ID = canId + 0x600;
    msg.rtr = 0x00;
    msg.len = 8;
    msg.data[0] = dataBlockNum;
    sendCanPacket(msg);
}

void CanPort::configPort(CANMODE mode, DWORD accCode, DWORD accMask, CANBAUDRATE baudrate)
{
    PORT_STRUCT setPort;
    isPortOpenedAssert(true);

    setPort.mode = mode;
    setPort.accCode = accCode;
    setPort.accMask = accMask;
    setPort.baudrate = baudrate;

    CanConfigPort(handle, &setPort);
}

void CanPort::cleanBuffers()
{
    cleanRxBuffer();
    cleanTxBuffer();
}

void CanPort::cleanTxBuffer()
{
    isPortOpenedAssert(true);
    CanClearTxBuffer(handle);
}

void CanPort::cleanRxBuffer()
{
    isPortOpenedAssert(true);
    CanClearRxBuffer(handle);
}

void CanPort::close()
{
    isPortOpenedAssert(true);
    CanCloseDriver(handle);
}

void CanPort::resetCommunication(u_int8_t nodeId)
{
    sendNMT(NMT_RESET_COMMUNICATION, nodeId);
}

void CanPort::resetNode(u_int8_t nodeId)
{
    sendNMT(NMT_RESET_NODE, nodeId);
}

void CanPort::startRemoteNode(u_int8_t nodeId)
{
    sendNMT(NMT_START_REMOTE_NODE, nodeId);
}

void CanPort::sendNMT(uint8_t command, uint8_t CANid)
{
    CAN_PACKET NMTmsg;
    std::memset(&NMTmsg, 0, sizeof(NMTmsg));

    NMTmsg.CAN_ID = 0;
    NMTmsg.rtr = 0;
    NMTmsg.len = 2;

    NMTmsg.data[0] = command;
    NMTmsg.data[1] = CANid;

    sendCanPacket(NMTmsg);
}

void CanPort::controlPDO(uint8_t CANid, u_int16_t control1, u_int16_t control2)
{
    CAN_PACKET msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.CAN_ID = CANid + 0x200;
    msg.rtr = 0x00;
    msg.len = 2;
    msg.data[0] = control1;
    msg.data[1] = control2;

    sendCanPacket(msg);
}

void CanPort::uploadSDO(uint8_t CANid, SDOkey sdo)
{
    CAN_PACKET msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.CAN_ID = CANid + 0x600;
    msg.rtr = 0x00; // Corresponding to Standard frame
    msg.len = 8;
    msg.data[0] = 0x40;
    msg.data[1] = sdo.index & 0xFF;
    msg.data[2] = (sdo.index >> 8) & 0xFF;
    msg.data[3] = sdo.subindex;
    msg.data[4] = 0x00;
    msg.data[5] = 0x00;
    msg.data[6] = 0x00;
    msg.data[7] = 0x00;

    sendCanPacket(msg);
}

void CanPort::initSendSegmentedSdo(uint8_t CANid, SDOkey sdo)
{
    CAN_PACKET msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.CAN_ID = CANid + 0x600;
    msg.len = 8;
    msg.data[0] = 0x21;
    msg.data[1] = sdo.index & 0xFF;
    msg.data[2] = (sdo.index >> 8) & 0xFF;
    msg.data[3] = sdo.subindex;
    sendCanPacket(msg);
}

void CanPort::sendSDO16(uint8_t CANid, SDOkey sdo, uint16_t value, bool sizeIndicated)
{
    CAN_PACKET msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.CAN_ID = CANid + 0x600;
    // msg.rtr already set to zero
    msg.len = 8;
    msg.data[0] = 0x2B;
    msg.data[1] = sdo.index & 0xFF;
    msg.data[2] = (sdo.index >> 8) & 0xFF;
    msg.data[3] = sdo.subindex;
    msg.data[4] = value & 0xFF;
    msg.data[5] = (value >> 8) & 0xFF;
    msg.data[6] = 0x00;
    msg.data[7] = 0x00;
    sendCanPacket(msg);
}

void CanPort::sendSDO(uint8_t CANid, SDOkey sdo, uint32_t value, bool sizeIndicated)
{
    CAN_PACKET msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.CAN_ID = CANid + 0x600;
    // msg.rtr already set to zero
    msg.len = 8;
    if(sizeIndicated) {
      msg.data[0] = 0x23;
    } else {
      msg.data[0] = 0x22;
    }
    msg.data[1] = sdo.index & 0xFF;
    msg.data[2] = (sdo.index >> 8) & 0xFF;
    msg.data[3] = sdo.subindex;
    msg.data[4] = value & 0xFF;
    msg.data[5] = (value >> 8) & 0xFF;
    msg.data[6] = (value >> 16) & 0xFF;
    msg.data[7] = (value >> 24) & 0xFF;

    sendCanPacket(msg);
}

void CanPort::clearTPDOMapping(uint8_t id, int object)
{
    sendSDO(id, SDOkey(TPDO_map.index + object, 0x00), u_int8_t(0x00));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}


void CanPort::clearRPDOMapping(uint8_t id, int object)
{
    int32_t data = (0x00 << 16) + (0x80 << 24);
    sendSDO(id, SDOkey(RPDO_map.index + object, 0x00), data, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

void CanPort::sendCanPacket(CAN_PACKET& msg)
{
    isPortOpenedAssert(true);

    if(debugEnabled) {
      printf("sending to canId: %X data: ", (unsigned int)msg.CAN_ID);
      printCanPacketData(msg);
    }

    CanSendMsg(handle, &msg);
}

void CanPort::printCanPacketData(const CAN_PACKET& msg)
{
    printf("<");
    std::cout << " len:  " << msg.len << " -- ";
    for(BYTE i =0; i < msg.len; i++) {
      printf("%X", (unsigned int)msg.data[i]);
      if(i != (msg.len - 1)) {
        printf(":");
      }
    }
    printf(">\n");
}

void CanPort::isPortOpenedAssert(bool opened)
{
    if(this->opened != opened) {
      std::string msg = "Port is ";
      if(this->opened) {
        msg += "open";
      } else {
        msg += "closed";
      }
      throw new CanException(msg);
    }
}

void CanPort::makeTPDOMapping(uint8_t id, int object, vector< string > registers, vector< int > sizes, u_int8_t sync_type)
{
    int ext_counter=0;
    for(int counter=0; counter < registers.size();counter++)
    {
        int index_data;

        std::stringstream str_stream;
        str_stream << registers[counter];
        str_stream >> std::hex >> index_data;

        str_stream.str( std::string() );
        str_stream.clear();

        int32_t data = (sizes[counter]) + (index_data << 8);

        sendSDO(id, SDOkey(TPDO_map.index+object,counter+1), data);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        ext_counter++;
      }

    sendSDO(id, SDOkey(TPDO_INDEX+object,0x02), u_int8_t(sync_type));

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    sendSDO(id, SDOkey(TPDO_map.index+object,0x00), u_int8_t(ext_counter));
}

void CanPort::makeRPDOMapping(uint8_t id, int object, vector< string > registers, vector< int > sizes, u_int8_t sync_type)
{
    int ext_counter=0;
    for(int counter=0; counter < registers.size();counter++)
    {
      int index_data;

      std::stringstream str_stream;
      str_stream << registers[counter];
      str_stream >> std::hex >> index_data;

      str_stream.str( std::string() );
      str_stream.clear();

      int32_t data = (sizes[counter]) + (index_data << 8);

      sendSDO(id, SDOkey(RPDO_map.index+object,counter+1), data);

      std::this_thread::sleep_for(std::chrono::milliseconds(10));

      ext_counter++;
    }

    sendSDO(id, SDOkey(RDPO_INDEX+object,0x02), u_int8_t(sync_type));

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    sendSDO(id, SDOkey(RPDO_map.index+object,0x00), u_int8_t(ext_counter));
}

void CanPort::enableTPDO(uint8_t id, int object)
{
      static int msgs[] = { TPDO1_MSG, TPDO2_MSG, TPDO3_MSG, TPDO4_MSG};
      int32_t data = msgs[id] + (0x00 << 16) + (0x00 << 24);
      sendSDO(id, SDOkey(TPDO_INDEX+object,0x01), data);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

void CanPort::enableRPDO(uint8_t id, int object)
{
      static int msgs[] = { RPDO1_MSG, RPDO2_MSG, RPDO3_MSG, RPDO4_MSG};
      int32_t data = (msgs[id]) + (0x00 << 16) + (0x00 << 24);
      sendSDO(id, SDOkey(RDPO_INDEX+object,0x01), data);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

unsigned int CanPort::getCOBType(long unsigned int cobId)
{
    return (cobId >> 7) & 0b1111;
}

unsigned int CanPort::getCanId(long int cobId)
{
    return cobId & 0x7F;
}
