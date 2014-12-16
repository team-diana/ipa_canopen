#include "ipa_canopen_core/can_port.h"

#include <iostream>
#include <time.h>
#include <unistd.h>
#include "assert.h"
#include <math.h>
#include <string>

using namespace std;

SDOkey OS_COMMAND_MODE(0x1024, 0);
SDOkey OS_COMMAND_PROMPT(0x1023, 0);

void mssleep(int ms) {
  usleep(ms * 10e3);
}

SDOkey getSdoKeyFromPacket(const CAN_PACKET& msg) {
  return SDOkey( msg.data[1] + (msg.data[2] << 8), msg.data[3]);
}

void sendSegmentedSdo(CanPort& p, int canId, std::vector<BYTE> data, bool toggle, bool moreSegments) {
  int objId = canId + 0x600;

  BYTE firstByte = 0x01;
  if(toggle) {
    firstByte += 0b10000;
  }
  if(moreSegments) {
    firstByte += 0b1;
  }
  firstByte += (7-data.size()) << 1;
  data.insert(data.begin(), firstByte);
  p.sendMsg(objId, data, 0);
}

void sendSegmentedData(CanPort& p, int canId, SDOkey sdoKey, std::vector<BYTE> data)
{
  p.initSendSegmentedSdo(canId, sdoKey);
  mssleep(1000);
  SDOkey receivedSdoKey = getSdoKeyFromPacket(p.rcvMsg());

  if(receivedSdoKey != sdoKey) {
    std::cerr << "sdo key was wrong during send segmented data" << std::endl;
    exit(-1);
  }

  bool toggle = false;
  int chunk_num = ceil(data.size()/7.0f);
  for(int cur_chunk = 0; cur_chunk < chunk_num; cur_chunk++) {
    bool moreSegments = cur_chunk+1 < chunk_num;
    std::vector<BYTE> chunk;
    for(int i = cur_chunk*7; i < min( (cur_chunk+1) * 7, (int) data.size()); i++) {
      chunk.push_back(data[i]);
    }
    sendSegmentedSdo(p, canId, chunk, toggle, moreSegments);
    CAN_PACKET res = p.rcvMsg();
    BYTE expectedFirstByte = 0x20;
    if(toggle) {
      expectedFirstByte += 0b10000;
    }
    if (res.data[0] != expectedFirstByte) {
      cerr << "ERROR: bad server response" << endl;
    }
    toggle = !toggle;
  }

}

std::vector<BYTE> dataToVector(const CAN_PACKET& msg) {
  std::vector<BYTE> v;
  for(int i = 0; i < msg.len; i++) {
    v.push_back(msg.data[i]);
  }
  return v;
}

bool areEquals(const std::vector<BYTE>& l, const std::vector<BYTE>& r) {
  return std::equal(l.begin(), l.end(), r.begin());
}

bool assertSameData(const CAN_PACKET& msg, const std::vector<BYTE>& data) {
  std::vector<BYTE> l = dataToVector(msg);
  if(!areEquals(l, data)) {
    throw new CanException("Different data");
  }
}


std::vector<BYTE> stringToVec(const std::string& s) {
  std::vector<BYTE> vec;

  for(int i = 0; i < s.size(); i++) {
    vec.push_back(s[i]);
  }

  return vec;
}

void startShell(int canId) {
  cout << " starting shell for canId: " << canId << endl;

  CanPort p = CanPort(0, 0);
  p.open();

  p.configPort(BIT11, 0, 0x7EE, CAN_BAUD_500K);
  p.cleanBuffers();

  mssleep(100);
  p.sendSDO(canId, OS_COMMAND_MODE, 0);

  mssleep(1000);
  CAN_PACKET res = p.rcvMsg();
  assertSameData(res, {0x60, 0x24, 0x10, 0,   0, 0, 0, 0});

  mssleep(500);
  p.cleanBuffers();


  while(true) {
    string s;
    cout << "-->";
    cin >> s;
    cout << endl;
    if(s == "exit" || s == "q") {
      break;
    }
    std::vector<BYTE> data = stringToVec(s);
    sendSegmentedData(p, canId, OS_COMMAND_PROMPT, data);
  }
}

int main(int argc, char** argv) {
  if (argc != 2) {
    cerr << "usage: " << argv[0]  << " canId " << endl;
  }

  int canId = atoi(argv[1]);
}
