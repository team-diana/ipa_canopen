#!/usr/bin/env python
from can import *
import time

def sleepms(ms):
    time.sleep(ms/1000)

def main():
    p = CanPort(0, 0)
    p.open()
    p.config_port(mode.BIT11, 0, 0x7EE, baud_rate.CAN_BAUD_500K) # TODO: check all parameters
    p.clean_buffers()
    p.enable_debug(True)
    canId = 12
    p.start_remote_node(canId)
    sleepms(5000)
    p.request_data_block(canId, data_block_num.DATA_BLOCK_1)
    sleepms(5000)
    p.recv_msg()

if __name__ == "__main__":
    main()
