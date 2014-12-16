#include <boost/python.hpp>
#include <ipa_canopen_core/can_enum.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <memory>
#include "ipa_canopen_core/can_python.h"

std::string initModule() {
}

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(canPortResetCommunicationOverloads, resetCommunication, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(canPortResetNodeOverloads, resetNode, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(canPortStartRemoteNodeOverloads, startRemoteNode, 0, 1)


static void translateCanException(const CanException & e) {
  PyErr_SetString(PyExc_UserWarning, e.what() );
}

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
