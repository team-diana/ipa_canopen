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

}
