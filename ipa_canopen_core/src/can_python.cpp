#include <boost/python.hpp>
#include <ipa_canopen_core/can_enum.h>
#include <iostream>

extern "C" {
  #include "pci_7841.h"
}


using namespace std;

// struct World
// {
// void set(std::string msg) { mMsg = msg; }
// std::string greet() { return mMsg; }
// std::string mMsg;
// };
// #include <boost/python.hpp>
// using namespace boost::python;
// BOOST_PYTHON_MODULE(classes)
// {
// class_<World>("World")
// .def("greet", &World::greet)
// .def("set", &World::set)
// ;
// };

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
};

//TPDO MAPPING
const SDOkey TPDO_map(0x1A00, 0x0);

//RPDO MAPPING
const SDOkey RPDO_map(0x1600, 0x0);
    
enum NMTMESSAGES {
  NMT_START_REMOTE_NODE = 0x01,
  NMT_RESET_NODE = 0x81,
  NMT_RESET_COMMUNICATION = 0x82
};


class CanPort {
public:
  CanPort(int cardNumber, int portNumber) :
  cardNumber(cardNumber),
  portNumber(portNumber),
  opened(false),
  handle(0),
  debugEnabled(false) {

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
    CAN_PACKET canPacket;
    memset(&canPacket, 0, sizeof(canPacket));
    CanRcvMsg(handle, &canPacket);

    if(debugEnabled) {
      printf("receiving data: ");
      printCanPacketData(canPacket);
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
  
  void resetCommunication() {
    sendNMT(0x00, NMT_RESET_COMMUNICATION);
  }
  
  void resetNode() {
    sendNMT(0x00, NMT_RESET_NODE);
  }
  
  void startRemoteNode(u_int8_t id) {
    sendNMT(id, NMT_START_REMOTE_NODE);
  }

private:
  void sendCanPacket(CAN_PACKET& msg) {
    isPortOpenedAssert(true);

    if(debugEnabled) {
      printf("sending data: ");
      printCanPacketData(msg);
    }

    CanSendMsg(handle, &msg);
  }

  void printCanPacketData(const CAN_PACKET& msg) {
    printf("<");
    for(int i =0; i < msg.len; i++) {
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
  
  void sendNMT(uint8_t CANid, uint8_t command) {
    CAN_PACKET NMTmsg;
    std::memset(&NMTmsg, 0, sizeof(CAN_PACKET));
    NMTmsg.CAN_ID = 0;
    NMTmsg.rtr = 0;
    NMTmsg.len = 2;
    
    NMTmsg.data[0] = command;
    NMTmsg.data[1] = CANid;
    
    sendCanPacket(&NMTmsg);
  }
  
    void controlPDO(uint8_t CANid, u_int16_t control1, u_int16_t control2) {
    CAN_PACKET msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.CAN_ID = CANid + 0x200;
    msg.rtr = 0x00;
    msg.len = 2;
    msg.data[0] = control1;
    msg.data[1] = control2;
    
    sendCanPacket(&msg);
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
    
    sendCanPacket(&msg);
  }
  
  void sendSDO(uint8_t CANid, SDOkey sdo, uint32_t value) {
    CAN_PACKET msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.CAN_ID = CANid + 0x600;
    // msg.rtr already set to zero
    msg.len = 8;
    msg.data[0] = 0x23;
    msg.data[1] = sdo.index & 0xFF;
    msg.data[2] = (sdo.index >> 8) & 0xFF;
    msg.data[3] = sdo.subindex;
    msg.data[4] = value & 0xFF;
    msg.data[5] = (value >> 8) & 0xFF;
    msg.data[6] = (value >> 16) & 0xFF;
    msg.data[7] = (value >> 24) & 0xFF;
    
    sendCanPacket(&msg);
  }
  
  void clearTPDOMapping(uint8_t id, int object) {
    sendSDO(id, SDOkey(TPDO_map.index + object, 0x00), u_int8_t(0x00));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  
  void clearRPDOMapping(uint8_t id, int object) {
    int32_t data = (0x00 << 16) + (0x80 << 24);
    sendSDO_unknown(id, SDOkey(RPDO_map.index + object, 0x00), data);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  
  void makeTPDOMapping(uint8_t id, int object, std::vector<std::string> registers, std::vector<int> sizes, u_int8_t sync_type) {
    //////////////////// sub ind1=63
    ///
    ///
    ///
    int ext_counter=0;
    for(int counter=0; counter < registers.size();counter++)
    {
	/////////////////////////
	int index_data;

	std::stringstream str_stream;
	str_stream << registers[counter];
	str_stream >> std::hex >> index_data;

	str_stream.str( std::string() );
	str_stream.clear();

	/////////////////////////
	/// \brief data
	///
	int32_t data = (sizes[counter]) + (index_data << 8);

	sendSDO(id, SDOkey(TPDO_map.index+object,counter+1), data);

	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	ext_counter++;
    }
    /////////////////////////
    //////////////////// ASync

    sendSDO(id, SDOkey(TPDO.index+object,0x02), u_int8_t(sync_type));

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    //////////////////////
    ///
    ///
    /////////////////////// Mapping x objects
    sendSDO(id, SDOkey(TPDO_map.index+object,0x00), u_int8_t(ext_counter));
  }
  
  void makeRPDOMapping(uint8_t id, int object, std::vector<std::string> registers, std::vector<int> sizes , u_int8_t sync_type) {
    int ext_counter=0;
    for(int counter=0; counter < registers.size();counter++)
    {
	/////////////////////////
	int index_data;

	std::stringstream str_stream;
	str_stream << registers[counter];
	str_stream >> std::hex >> index_data;

	str_stream.str( std::string() );
	str_stream.clear();

	/////////////////////////
	/// \brief data
	///
	int32_t data = (sizes[counter]) + (index_data << 8);

	sendSDO(id, SDOkey(RPDO_map.index+object,counter+1), data);

	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	ext_counter++;
    }
    /////////////////////////
    //////////////////// ASync

    sendSDO(id, SDOkey(RPDO.index+object,0x02), u_int8_t(sync_type));

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    //////////////////////
    ///
    ///
    /////////////////////// Mapping x objects
    sendSDO(id, SDOkey(RPDO_map.index+object,0x00), u_int8_t(ext_counter));
  }
  
void enableTPDO(uint8_t id, int object) {
    //////////////////// Enable tpdo4
    ///
    ///
    if(object ==0)
    {
	int32_t data = (canopen::TPDO1_msg + id) + (0x00 << 16) + (0x00 << 24);

	sendSDO(id, SDOkey(TPDO.index+object,0x01), data);
    }
    else if(object == 1)
    {
	int32_t data = (canopen::TPDO2_msg + id) + (0x00 << 16) + (0x00 << 24);

	sendSDO(id, SDOkey(TPDO.index+object,0x01), data);
    }
    else if(object == 2)
    {
	int32_t data = (canopen::TPDO3_msg + id) + (0x00 << 16) + (0x00 << 24);

	sendSDO(id, SDOkey(TPDO.index+object,0x01), data);
    }
    else if(object == 3)
    {
	int32_t data = (canopen::TPDO4_msg + id) + (0x00 << 16) + (0x00 << 24);

	sendSDO(id, SDOkey(TPDO.index+object,0x01), data);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    /////////////////////////
  }

void enableRPDO(uint8_t id, int object) {
    if(object ==0)
    {
	int32_t data = (canopen::RPDO1_msg + id) + (0x00 << 16) + (0x00 << 24);

	sendSDO(id, SDOkey(RPDO.index+object,0x01), data);
    }
    else if(object == 1)
    {
	int32_t data = (canopen::RPDO2_msg + id) + (0x00 << 16) + (0x00 << 24);

	sendSDO(id, SDOkey(RPDO.index+object,0x01), data);
    }
    else if(object == 2)
    {
	int32_t data = (canopen::RPDO3_msg + id) + (0x00 << 16) + (0x00 << 24);

	sendSDO(id, SDOkey(RPDO.index+object,0x01), data);
    }
    else if(object == 3)
    {
	int32_t data = (canopen::RPDO4_msg + id) + (0x00 << 16) + (0x00 << 24);

	sendSDO(id, SDOkey(RPDO.index+object,0x01), data);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    /////////////////////////
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

BOOST_PYTHON_MODULE(can)
{
  using namespace boost::python;
  register_exception_translator<CanException>(&translateCanException);

  def("init", initModule);

  class_<CanPort>("CanPort", init<int, int>())
    .def("clean_buffers",   &CanPort::cleanBuffers)
    .def("clean_rx_buffer", &CanPort::cleanRxBuffer)
    .def("clean_tx_buffer", &CanPort::cleanTxBuffer)
    .def("close",           &CanPort::close)
    .def("config_port",     &CanPort::configPort)
    .def("open",            &CanPort::open)
    .def("send_msg",        &CanPort::sendMsg)
    .def("recv_msg",        &CanPort::rcvMsg)
    .def("enable_debug",        &CanPort::setDebugEnabled)
    .def("reset_communication", 	&CanPort::resetCommunication)
    .def("reset_node",		&CanPort::resetNode)
    .def("start_remote_node", 		&CanPort::startRemoteNode);

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
    .value("NMT_RESET_NODE", NMT_RESET_NODE)
    .value("NMT_RESET_COMMUNICATION", NMT_RESET_COMMUNICATION);
    
}
