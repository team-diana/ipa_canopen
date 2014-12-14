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
  CanException(const std::string& msg) : msg(msg) {
    cerr << "new exception: " << msg << endl;
  }

  const char* what() const throw() override {
    return msg.c_str();
  }

private:
  std::string msg;
};

static void translateCanException(const CanException & e) {
  PyErr_SetString(PyExc_UserWarning, e.what() );
}

struct CanMsg {
  CanMsg() : canId(0), rtr(0) {

  }

  int size() {
    return len(data);
  }

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
  CanPort(int cardNumber, int portNumber) :
  cardNumber(cardNumber),
  portNumber(portNumber),
  opened(false),
  handle(0),
  debugEnabled(true) {

  }

  void open() {
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

  void setDebugEnabled(bool enabled) {
    debugEnabled = enabled;
  }

  void sendMsg(int canId, const boost::python::list& data, int rtr = 0) {
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

  CanMsg rcvMsg() {
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

    CanMsg canMsg;
    canMsg.canId = canPacket.CAN_ID;
    canMsg.rtr = canPacket.rtr;
    for (int i = 0; i < canPacket.len; ++i)
    {
      canMsg.data.append<BYTE>(canPacket.data[i]);
    }


    return canMsg;
  }

  void requestDataBlock(uint8_t canId, DATA_BLOCK_NUM dataBlockNum) {
    CAN_PACKET msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.CAN_ID = canId + 0x600;
    msg.rtr = 0x00;
    msg.len = 8;
    msg.data[0] = dataBlockNum;
    sendCanPacket(msg);
  }


  void configPort(CANMODE mode, DWORD accCode, DWORD accMask, CANBAUDRATE baudrate) {
    PORT_STRUCT setPort;
    isPortOpenedAssert(true);

    setPort.mode = mode;
    setPort.accCode = accCode;
    setPort.accMask = accMask;
    setPort.baudrate = baudrate;

    CanConfigPort(handle, &setPort);
  }

  void cleanBuffers() {
    cleanRxBuffer();
    cleanTxBuffer();
  }

  void cleanTxBuffer() {
    isPortOpenedAssert(true);
    CanClearTxBuffer(handle);
  }

  void cleanRxBuffer() {
    isPortOpenedAssert(true);
    CanClearRxBuffer(handle);
  }

  void close() {
    isPortOpenedAssert(true);
    CanCloseDriver(handle);
  }

  void resetCommunication(u_int8_t nodeId = 0) {
    sendNMT(NMT_RESET_COMMUNICATION, nodeId);
  }

  void resetNode(u_int8_t nodeId = 0) {
    sendNMT(NMT_RESET_NODE, nodeId);
  }

  void startRemoteNode(u_int8_t nodeId = 0) {
    sendNMT(NMT_START_REMOTE_NODE, nodeId);
  }

  void sendNMT(uint8_t command, uint8_t CANid = 0) {
    CAN_PACKET NMTmsg;
    std::memset(&NMTmsg, 0, sizeof(NMTmsg));

    NMTmsg.CAN_ID = 0;
    NMTmsg.rtr = 0;
    NMTmsg.len = 2;

    NMTmsg.data[0] = command;
    NMTmsg.data[1] = CANid;

    sendCanPacket(NMTmsg);
  }

  void controlPDO(uint8_t CANid, u_int16_t control1, u_int16_t control2) {
    CAN_PACKET msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.CAN_ID = CANid + 0x200;
    msg.rtr = 0x00;
    msg.len = 2;
    msg.data[0] = control1;
    msg.data[1] = control2;

    sendCanPacket(msg);
  }

  void uploadSDO(uint8_t CANid, SDOkey sdo) {
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

  void initSendSegmentedSdo(uint8_t CANid, SDOkey sdo) {
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

  // Expedited transfer
  void sendSDO16(uint8_t CANid, SDOkey sdo, uint16_t value, bool sizeIndicated = true) {
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

  // Expedited transfer
  void sendSDO(uint8_t CANid, SDOkey sdo, uint32_t value, bool sizeIndicated = true) {
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

  void clearTPDOMapping(uint8_t id, int object) {
    sendSDO(id, SDOkey(TPDO_map.index + object, 0x00), u_int8_t(0x00));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  void clearRPDOMapping(uint8_t id, int object) {
    int32_t data = (0x00 << 16) + (0x80 << 24);
    sendSDO(id, SDOkey(RPDO_map.index + object, 0x00), data, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }


private:
  void sendCanPacket(CAN_PACKET& msg) {
    isPortOpenedAssert(true);

    if(debugEnabled) {
      printf("sending to canId: %X data: ", (unsigned int)msg.CAN_ID);
      printCanPacketData(msg);
    }

    CanSendMsg(handle, &msg);
  }

  void printCanPacketData(const CAN_PACKET& msg) {
    printf("<");
    for(unsigned char i =0; i < msg.len; i++) {
      printf("%X", (unsigned int)msg.data[i]);
      if(i != (msg.len - 1)) {
        printf(":");
      }
    }
    printf(">\n");
  }

  void isPortOpenedAssert(bool opened) {
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


  void makeTPDOMapping(uint8_t id, int object, std::vector<std::string> registers, std::vector<int> sizes, u_int8_t sync_type) {
    //////////////////// sub ind1=63
    ///
    ///
    ///
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

  void makeRPDOMapping(uint8_t id, int object, std::vector<std::string> registers, std::vector<int> sizes , u_int8_t sync_type) {
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

    void enableTPDO(uint8_t id, int object) {
      static int msgs[] = { TPDO1_MSG, TPDO2_MSG, TPDO3_MSG, TPDO4_MSG};
      int32_t data = msgs[id] + (0x00 << 16) + (0x00 << 24);
      sendSDO(id, SDOkey(TPDO_INDEX+object,0x01), data);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

  void enableRPDO(uint8_t id, int object) {
      static int msgs[] = { RPDO1_MSG, RPDO2_MSG, RPDO3_MSG, RPDO4_MSG};
      int32_t data = (msgs[id]) + (0x00 << 16) + (0x00 << 24);
      sendSDO(id, SDOkey(RDPO_INDEX+object,0x01), data);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

  unsigned int getCOBType(unsigned long cobId) {
    return (cobId >> 7) & 0b1111;
  }

  unsigned int getCanId(long cobId) {
    return cobId & 0x7F;
  }

  private:
    int cardNumber;
    int portNumber;
    bool opened;
    int handle;
    bool debugEnabled;
};


std::string initModule() {
}

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(canPortResetCommunicationOverloads, resetCommunication, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(canPortResetNodeOverloads, resetNode, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(canPortStartRemoteNodeOverloads, startRemoteNode, 0, 1)


BOOST_PYTHON_MODULE(can)
{
  using namespace boost::python;
  register_exception_translator<CanException>(&translateCanException);

  def("init", initModule);

  class_<SDOkey>("SDOKey", init<int, int>())
    .def_readonly("index", &SDOkey::index)
    .def_readonly("subindex", &SDOkey::subindex)
    .def(self == self)
    .def(self != self);

  class_<CanPort>("CanPort", init<int, int>())
    .def("clean_buffers",   &CanPort::cleanBuffers)
    .def("clean_rx_buffer", &CanPort::cleanRxBuffer)
    .def("clean_tx_buffer", &CanPort::cleanTxBuffer)
    .def("close",           &CanPort::close)
    .def("config_port",     &CanPort::configPort)
    .def("open",            &CanPort::open)
    .def("send_msg",        &CanPort::sendMsg)
    .def("recv_msg",        &CanPort::rcvMsg)
    .def("reset_communication",        &CanPort::resetCommunication, canPortResetCommunicationOverloads())
    .def("reset_node",        &CanPort::resetNode, canPortResetNodeOverloads())
    .def("start_remote_node",        &CanPort::startRemoteNode, canPortStartRemoteNodeOverloads())
    .def("upload_sdo", &CanPort::uploadSDO)
    .def("send_sdo", &CanPort::sendSDO)
    .def("send_sdo_16", &CanPort::sendSDO)
    .def("init_send_segmented_sdo", &CanPort::initSendSegmentedSdo)
    .def("request_data_block", &CanPort::requestDataBlock)
    .def("enable_debug",        &CanPort::setDebugEnabled);

  class_<CanMsg>("CanMsg")
    .def_readwrite("can_id", &CanMsg::canId)
    .def_readwrite("rtr",    &CanMsg::rtr)
    .def_readwrite("data",   &CanMsg::data);

  // CANMODE
  enum_<CANMODE>("mode")
    .value("BIT11", BIT11)
    .value("BIT29", BIT29);

  // CANBAUDRATE
  enum_<CANBAUDRATE>("baud_rate")
    .value("CAN_BAUD_1M", CAN_BAUD_1M)
    .value("CAN_BAUD_500K", CAN_BAUD_500K)
    .value("CAN_BAUD_250K", CAN_BAUD_250K)
    .value("CAN_BAUD_125K", CAN_BAUD_125K);

  // NMTMESSAGES
  enum_<NMTMESSAGES>("nmt_message")
    .value("NMT_START_REMOTE_NODE", NMT_START_REMOTE_NODE)
    .value("NMT_STOP_REMOTE_NODE", NMT_STOP_REMOTE_NODE)
    .value("NMT_ENTER_PRE_OPERATIONAL", NMT_ENTER_PRE_OPERATIONAL)
    .value("NMT_RESET_NODE", NMT_RESET_NODE)
    .value("NMT_RESET_COMMUNICATION", NMT_RESET_COMMUNICATION);

  enum_<DATA_BLOCK_NUM>("data_block_num")
    .value("DATA_BLOCK_1", DATA_BLOCK_1)
    .value("DATA_BLOCK_2", DATA_BLOCK_2);

}
