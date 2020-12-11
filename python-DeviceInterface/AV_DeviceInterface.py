import argparse
import errno
import math
import msvcrt
import numpy as np
import os
import socket
import serial
import sys
import time
import traceback
import pickle

from pythonosc import dispatcher, osc_server
from scipy import fftpack, signal
from socket import SHUT_RDWR
from threading import Thread
from time import sleep
from EEGProfile import EEGProfile,DDSSetting,SamplingInfo,observeFreqRange
import observeConfig

import matplotlib.pyplot as plt

ROOTPATH_g = (os.path.dirname(os.path.abspath(__file__)).replace("\\", "/")).split("/Data")[0]

#sys.path.append(ROOTPATH_g + "/Data/Work/UtilSrcCode/python-Bluetooth")
sys.path.append( "E:/freelance/eeg-bluetooth/python-Bluetooth")
from AV_bluetooth import AV_ble_win10_getCOM

class DeviceInterface(object):

    def __init__(self, Fs_p, Name_p):
        """
        Params:
            Fs_p : The sampling rate of the device given in Hz.
        """

        super(DeviceInterface, self).__init__()

        self._Name = Name_p
        self._Fs = Fs_p


    def setSamplingRate(self, Fs_p):
        self._Fs = Fs_p
    def getSamplingRate(self):
        return self._Fs

    def getName(self):
        return self._Name

class EEGInterface(DeviceInterface):

    def __init__(self, Fs_p, Name_p, N_channels_p):

        super(EEGInterface, self).__init__(Fs_p, Name_p)

        self._N_channels = N_channels_p

    def getN_channels(self):
        return self._N_channels

class MindWaveMobile2Thread(EEGInterface, Thread):
    """
    MindWave Mobile 2 use TGAM1.
    """

    __STATE_1 = 1
    __STATE_2 = 2
    __STATE_3 = 3
    __STATE_4 = 4
    __STATE_6 = 6

    __CONNECT             = b'\xc0'
    __DISCONNECT          = b'\xc1'
    __AUTOCONNECT         = b'\xc2'
    __SYNC                = b'\xaa'
    __EXCODE              = b'\x55'
    __HEADSET_CONNECTED   = b'\xd0'
    __HEADSET_NOT_FOUND   = b'\xd1'
    __HEADSET_DISCONNECTED= b'\xd2'
    __REQUEST_DENIED      = b'\xd3'
    __STANDBY_SCAN        = b'\xd4'

    __POOR_SIG_QUALITY    = b'\x02'
    __ATTENTION           = b'\x04'
    __MEDITATION          = b'\x05'
    __BLINK               = b'\x16'
    __RAW_WAVE            = b'\x80'
    __ASIC_EEG_POWER      = b'\x83'

    __TGAM1_CMD_9600_NORMAL       = b'0x00' # 9600 baud, normal output mode
    __TGAM1_CMD_1200_NORMAL       = b'0x01' # 1200 baud, normal output mode
    __TGAM1_CMD_57600_NORMAL_RAW  = b'0x02' # 57.6k baud, normal+raw output mode 
    __TGAM1_CMD_GARBAGE           = b'0xAB' #     

    def __init__(self, fct_callback_p, Name_p, COM_p, baudrate_p=57600, fct_blink_callback_p=None): 

        super(MindWaveMobile2Thread, self).__init__(Fs_p=512, Name_p=Name_p, N_channels_p=1)

        self._srl       = None
        self._COM       = COM_p
        self._baudrate  = baudrate_p

        self._rawValue  = 0

        self._delta     = 0
        self._theta     = 0
        self._lowAlpha  = 0
        self._highAlpha = 0
        self._lowBeta   = 0
        self._highBeta  = 0
        self._lowGamma  = 0
        self._midGamma  = 0  

        self._poor_sig_quality  = 0
        self._attention         = 0
        self._meditation        = 0
        self._blink             = 0

        self._fct_callback          = fct_callback_p
        self._fct_blink_callback    = fct_blink_callback_p

    def terminate(self):
        self._running = False

        self._close_serial()

        print(self._Name + ": Close serial")
        
    def run(self):
        self._main()

    ##### Private
    def _main(self):

        self._running = True

        while self._running:

            try:

                try:

                    print("[MindWaveMobile2Thread]: openning serial port.")

                    if not self._open_serial():
                        raise Exception("[MindWaveMobile2Thread]: Fail to open serial")

                    print("[MindWaveMobile2Thread]: opened serial port.")

                    state_l = self.__STATE_1

                    k = 0
                    while self._running:
                
                        a_byte_l = self._srl.read(1)

                        if (state_l == self.__STATE_1):

                            if a_byte_l == self.__SYNC:
                                state_l = self.__STATE_2
                            else:
                                state_l = self.__STATE_1

                        elif (state_l == self.__STATE_2):

                            if a_byte_l == self.__SYNC:
                                state_l = self.__STATE_3
                            else:
                                state_l = self.__STATE_1

                        elif (state_l == self.__STATE_3):

                            if a_byte_l == self.__SYNC:
                                state_l = self.__STATE_3
                            elif int.from_bytes(a_byte_l, byteorder="big") > 170: # PLENGTH TOO LARGE
                                state_l = self.__STATE_1
                            else:
                                pLength_l = int.from_bytes(a_byte_l, byteorder="big") # Length of PAYLOAD. It is between 0 and 169.

                                if pLength_l > 0: # If we encounter non-empty PAYLOAD, we fetch PAYLOAD data.

                                    payload_l = []
                                    checksum_l = 0

                                    state_l = self.__STATE_4

                                else:
                                    print("Found empty PAYLOAD")

                                    state_l = self.__STATE_1


                        elif (state_l == self.__STATE_4):

                            tempPacket = a_byte_l.hex()

                            payload_l.append(tempPacket)

                            checksum_l += int(tempPacket, 16)

                            if len(payload_l) == pLength_l:
                                checksum_l = ~checksum_l & 0x000000ff

                                state_l = self.__STATE_6


                        elif (state_l == self.__STATE_6):

                            if checksum_l == int(a_byte_l.hex(), 16): # Checksum is calculated using PAYLOAD data.
                                
                                i_payload_l = 0

                                while i_payload_l < len(payload_l): # Parse PAYLOAD data.

                                    # Count the number of EXCODE, indicating the Extended Code Level.
                                    n_EXCODE_l  = 0
                                    while payload_l[i_payload_l] == self.__EXCODE.hex():
                                        i_payload_l += 1
                                        n_EXCODE_l  += 1

                                    if n_EXCODE_l == 0:
                                        # The Extended Code Level 0.

                                        if payload_l[i_payload_l] == self.__RAW_WAVE.hex(): # RAWWAVE
                                            i_payload_l += 1

                                            vLength_l = int(payload_l[i_payload_l], 16)
                                            i_payload_l += 1

                                            assert vLength_l == 2

                                            val0 = int(payload_l[i_payload_l], 16)
                                            i_payload_l += 1

                                            rawValue_l = (val0*256) + int(payload_l[i_payload_l], 16)
                                            i_payload_l += 1
                    
                                            if rawValue_l > 32768:
                                                rawValue_l = rawValue_l - 65536
                                    
                                            assert (-32768 <= rawValue_l) and (rawValue_l <= 32767)
                                            
                                            # print("RAW_WAVE: (", pLength_l, ") ", rawValue_l) 

                                            self._rawValue = rawValue_l
                                            k+=1
                                            if (k >= 512*observeConfig.observeTime) :
                                                self._running = False

                                            

                                            self._eeg_handler(CH0_p=self._rawValue)

                                        elif payload_l[i_payload_l] == self.__ASIC_EEG_POWER.hex(): # ASIC_EEG_POWER
                                            i_payload_l += 1

                                            vLength_l = int(payload_l[i_payload_l], 16)
                                            i_payload_l += 1

                                            assert vLength_l == 24

                                            def _ASIC_EEG_POWER(payload_p, i_payload_p):
                                            
                                                val0        = int(payload_p[i_payload_p], 16)
                                                i_payload_p += 1

                                                val1        = int(payload_p[i_payload_p], 16)
                                                i_payload_p += 1

                                                power_l     = (val0*65536) + (val1*256) + int(payload_p[i_payload_p], 16)
                                                i_payload_p += 1

                                                return i_payload_p, power_l

                                            i_payload_l, self._delta    = _ASIC_EEG_POWER(payload_l, i_payload_l)
                                            i_payload_l, self._theta    = _ASIC_EEG_POWER(payload_l, i_payload_l)
                                            i_payload_l, self._lowAlpha = _ASIC_EEG_POWER(payload_l, i_payload_l)
                                            i_payload_l, self._highAlpha= _ASIC_EEG_POWER(payload_l, i_payload_l)
                                            i_payload_l, self._lowBeta  = _ASIC_EEG_POWER(payload_l, i_payload_l)
                                            i_payload_l, self._highBeta = _ASIC_EEG_POWER(payload_l, i_payload_l)
                                            i_payload_l, self._lowGamma = _ASIC_EEG_POWER(payload_l, i_payload_l)
                                            i_payload_l, self._midGamma = _ASIC_EEG_POWER(payload_l, i_payload_l)

                                            # print("ASIC_EEG_POWER: (", pLength_l, ") ", self._delta, "/", self._theta, "/", self._lowAlpha, "/", self._highAlpha, "/", self._lowBeta, "/", self._highBeta, "/", self._lowGamma, "/", self._midGamma)

                                        elif payload_l[i_payload_l] == self.__POOR_SIG_QUALITY.hex():
                                            i_payload_l += 1

                                            self._poor_sig_quality = int(payload_l[i_payload_l], 16)
                                            i_payload_l += 1

                                            k += 1
                                            # if k > 60:
                                            #     self._running = False

                                            print("POOR_SIG_QUALITY: (", pLength_l, ") ", self._poor_sig_quality)

                                        elif payload_l[i_payload_l] == self.__ATTENTION.hex():
                                            i_payload_l += 1

                                            self._attention = int(payload_l[i_payload_l], 16)                                        
                                            i_payload_l += 1

                                            print("ATTENTION: (", pLength_l, ") ", self._attention)

                                        elif payload_l[i_payload_l] == self.__MEDITATION.hex():
                                            i_payload_l += 1

                                            self._meditation = int(payload_l[i_payload_l], 16)                                        
                                            i_payload_l += 1

                                            # print("MEDITATION: (", pLength_l, ") ", self._meditation)

                                        elif payload_l[i_payload_l] == self.__BLINK.hex(): # Blink Strength. (0-255). Sent only when Blink event occurs.
                                            i_payload_l += 1

                                            self._blink = int(payload_l[i_payload_l], 16)
                                            i_payload_l += 1

                                            print("BLINK: (", pLength_l, ") ", self._blink)

                                            if self._fct_blink_callback_p is not None:
                                                self._fct_blink_callback_p(self._blink)

                                        else:
                                            print("[MindWaveMobile2Thread]: Undefined Level Zero Code:", payload_l[i_payload_l])

                                            exit()
                                    else:
                                        print("[MindWaveMobile2Thread]: Unsupported Extended Code Level:", n_EXCODE_l)

                                        exit()

                            else:
                                print("[MindWaveMobile2Thread]: Incorrect checksum")

                            state_l = self.__STATE_1


                except Exception as err:

                    self._close_serial()

                    exception_type = type(err).__name__
                    print("[MindWaveMobile2Thread]: exception_type = ", err)

            except Exception as err:

                exception_type = type(err).__name__

                print(self._Name + ": Outermost Exception and resume in 5 sec. with casscaded exception_type = ", exception_type)
                time.sleep(5.0)

        self._close_serial()

    def _eeg_handler(self, CH0_p):

        sample_l = np.ndarray((self._N_channels, 1), dtype=np.float32)  

        sample_l[0, 0] = CH0_p

        self._fct_callback(sample_l)

    def _open_serial(self):
        
        self._close_serial()

        self._srl           = serial.Serial()
        self._srl.port      = self._COM        
        self._srl.baudrate  = self._baudrate
        self._srl.open()

        if self._srl.isOpen() == False:
            self._srl = None

            return False

        self._srl.flushInput()
        self._srl.flushOutput()

        return True

    def _close_serial(self):

        if (self._srl is not None) and self._srl.isOpen():
            self._srl.close()

        self._srl = None

    def _sendCmdToTGAM1(self, cmd_p, srl_p):
        
        if _isGetValidPacket(srl_p): # According to "ThinkGear Command Bytes" in http://developer.neurosky.com/docs/doku.php?id=thinkgear_communications_protocol. We check first if the baudrate is correct by checking if we have a valid packet.
            
            srl_p.write(cmd_p)

            return True

        else:
            return False

    def _isGetValidPacket(srl_p):

        state_l = self.__STATE_1

        while True:

            a_byte_l = self._srl.read(1)

            if (state_l == self.__STATE_1):

                if a_byte_l == self.__SYNC:
                    state_l = self.__STATE_2
                else:
                    state_l = self.__STATE_1

            elif (state_l == self.__STATE_2):

                if a_byte_l == self.__SYNC:
                    state_l = self.__STATE_3
                else:
                    state_l = self.__STATE_1

            elif (state_l == self.__STATE_3):

                if a_byte_l == self.__SYNC:
                    state_l = self.__STATE_3
                elif int.from_bytes(a_byte_l, byteorder="big") > 170: # PLENGTH TOO LARGE
                    state_l = self.__STATE_1
                else:
                    pLength_l = int.from_bytes(a_byte_l, byteorder="big")

                    if pLength_l > 0: # If we encounter non-empty PAYLOAD, we fetch PAYLOAD data.                    

                        payload_l = []
                        checksum_l = 0

                        state_l = self.__STATE_4

                    else:
                        print("Found empty PAYLOAD")

                        state_l = self.__STATE_1

            elif (state_l == self.__STATE_4):

                tempPacket = a_byte_l.hex()

                payload_l.append(tempPacket)

                checksum_l += int(tempPacket, 16)

                if len(payload_l) == pLength_l:
                    checksum_l = ~checksum_l & 0x000000ff                    

                    state_l = self.__STATE_6

            elif (state_l == self.__STATE_6):

                if checksum_l == int(a_byte_l.hex(), 16):
                    return True
                else:
                    return False


class iAwareBLEThread(EEGInterface, Thread):

    def __init__(self, Fs_p, Name_p, N_channels_p):

        super(iAwareBLEThread, self).__init__(Fs_p, Name_p, N_channels_p)

class iAwareTCPThread(EEGInterface, Thread):

    __SENDER_TYPE_IAWARE   = 0 # This is mapped to /Volumes/Data/Work/FIRST/EEG/Boards/esp32/mytest5/main/main.c (16-bit sample)
    __SENDER_TYPE_VAG      = 1 # This is mapped to /Volumes/Data/Work/FIRST/EEG/Boards/esp32/vag/main/main.c (24-bit sample)

    __PACKET_HEADER_GROUP1 = 1
    
    __PACKET_HEADER_GROUP2 = 2
    __GPIO_LED_SWITCH_INPUT_triggered = 0
    __GPIO_LED_BATTERY_READ_triggered = 1

    __COM_TCP_RECV_WAITFOR_DATALENGTH = 0
    __COM_TCP_RECV_FETCH_MSG          = 1
    __COM_TCP_RECV_PROCESS_MSG        = 2    

    def __init__(self, fct_callback_p, buff_socket_size_p, server_ip_p, server_port_p, Fs_senddata_p, Fs_p, Name_p, N_channels_p):
        """
        Params:
            buff_socket_p   : Buffer size in bytes.
        """

        super(iAwareTCPThread, self).__init__(Fs_p, Name_p, N_channels_p)
        
        self._server_ip = server_ip_p
        self._server_port = server_port_p

        self._buff_socket_size = buff_socket_size_p

        self._Fs_senddata = Fs_senddata_p
        self._fct_callback = fct_callback_p
        
    def setServerIP(server_ip_p):
        self._server_ip = server_ip_p
    def getServerIP():
        return self._server_ip

    def setServerPort(server_port_p):
        self._server_port = server_port_p
    def getServerPort():
        return self._server_port

    def terminate(self):
        self._running = False

    def run(self):        
        self._mainV2(sender_Type_p=self.__SENDER_TYPE_VAG)
        # self._mainV1()    # This is mapped to /Volumes/Data/Work/FIRST/EEG/Boards/iAwareV0/src/main.cpp        
    
    ##### Private
    def _chkIfSameStateTooLong(self, pre_state_p, recv_state_p, cnt_same_state_thr_p, N_same_state_thr_p):
        if (pre_state_p == recv_state_p):
            cnt_same_state_thr_p = cnt_same_state_thr_p + 1 

            if (cnt_same_state_thr_p > N_same_state_thr_p):
                print("Same state for too long: " + str(pre_state_p))
                raise Exception("Recreate a connection.")                                
        else:
            pre_state_p = recv_state_p
            cnt_same_state_thr_p = 0       

        return pre_state_p, cnt_same_state_thr_p

    def _mainV2(self, sender_Type_p):
        data_len_bytes_l    = bytearray(4)

        self._running = True

        while self._running:
            try:
                try:
                    ##### Open the TCP connection.
                    sock_l = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    # print("_mainV2: Init socket success")

                    sock_l.settimeout(10)
                    # print("_mainV2: Set timeout success")

                    # Connect the socket to the port where the server is listening
                    server_address_l = (self._server_ip, self._server_port)
                    # print(self._Name + ": Connecting to " + str(server_address_l))
                    sock_l.connect(server_address_l)
                    print(str(time.ctime()) + " " + self._Name + ": Connected to " + str(server_address_l))

                    i_data_len_bytes_l  = 0
                    cnt_same_state_thr_l= 0
                    pre_state_l         = -1
                    #TODO Make it adaptive.
                    N_same_state_thr_l  = 100 # If it is looping in the same for more than N_same_state_thr, we might reach an unexpected error and reboot the socket is required.                    
                    ##### Getting message loop.
                    while self._running:

                        recv_state_l = self.__COM_TCP_RECV_WAITFOR_DATALENGTH
                        pre_state_l, cnt_same_state_thr_l = self._chkIfSameStateTooLong(pre_state_p=pre_state_l, recv_state_p=recv_state_l, cnt_same_state_thr_p=cnt_same_state_thr_l, N_same_state_thr_p=N_same_state_thr_l)

                        # This loop is to fetch one message.
                        while self._running:

                            # print("_mainV2: sock_l.recv begin")

                            block_l = sock_l.recv(self._buff_socket_size)

                            # print("_mainV2: sock_l.recv end")

                            N_block_l           = len(block_l)
                            i_N_block_l         = 0
                            remain_N_block_l    = N_block_l

                            if (N_block_l == 0):
                                raise Exception("Recreate a connection.")


                            while (remain_N_block_l > 0):

                                # Get the length of the message.
                                if (recv_state_l == self.__COM_TCP_RECV_WAITFOR_DATALENGTH):
                                    pre_state_l, cnt_same_state_thr_l = self._chkIfSameStateTooLong(pre_state_p=pre_state_l, recv_state_p=recv_state_l, cnt_same_state_thr_p=cnt_same_state_thr_l, N_same_state_thr_p=N_same_state_thr_l)
                                    
                                    while (i_data_len_bytes_l < 4) and (i_N_block_l < N_block_l):

                                        data_len_bytes_l[i_data_len_bytes_l] = block_l[i_N_block_l]

                                        i_data_len_bytes_l  = i_data_len_bytes_l + 1
                                        i_N_block_l         = i_N_block_l + 1

                                        remain_N_block_l    = remain_N_block_l - 1

                                    if (i_data_len_bytes_l == 4):
                                        data_len_l      = self._bytes_to_uint32(data_len_bytes_l)

                                        i_data_len_l    = 0

                                        msg_l = bytearray(data_len_l)

                                        recv_state_l = self.__COM_TCP_RECV_FETCH_MSG

                                # Get the message.
                                if (recv_state_l == self.__COM_TCP_RECV_FETCH_MSG): 
                                    pre_state_l, cnt_same_state_thr_l = self._chkIfSameStateTooLong(pre_state_p=pre_state_l, recv_state_p=recv_state_l, cnt_same_state_thr_p=cnt_same_state_thr_l, N_same_state_thr_p=N_same_state_thr_l)

                                    while (i_data_len_l < data_len_l) and (i_N_block_l < N_block_l):
                                    
                                        msg_l[i_data_len_l] = block_l[i_N_block_l]

                                        i_data_len_l = i_data_len_l + 1
                                        i_N_block_l = i_N_block_l + 1

                                        remain_N_block_l = remain_N_block_l - 1
                                    

                                    # We have a complete message.
                                    if (i_data_len_l == data_len_l):

                                        recv_state_l = self.__COM_TCP_RECV_PROCESS_MSG
                                    
                                # Process the message.
                                if (recv_state_l == self.__COM_TCP_RECV_PROCESS_MSG):
                                    pre_state_l, cnt_same_state_thr_l = self._chkIfSameStateTooLong(pre_state_p=pre_state_l, recv_state_p=recv_state_l, cnt_same_state_thr_p=cnt_same_state_thr_l, N_same_state_thr_p=N_same_state_thr_l)

                                    PACKET_HEADER_GROUPx = msg_l[0]

                                    if (PACKET_HEADER_GROUPx == self.__PACKET_HEADER_GROUP1): # GROUP1 stands for the data obtained from ADS1256.

                                        if (sender_Type_p == self.__SENDER_TYPE_IAWARE):
                                            eff_sampling_freq_l = self._bytes_to_uint32(msg_l[1:5]) # [microsec]. It is the duration in microsec. for collecting the message.

                                            # print(msg_l[4])
                                            uint16arr_buff_l = self._combineHighByteLowByte(msg_l[5:], data_len_l - 9)

                                            self._fct_callback(uint16arr_buff_l, eff_sampling_freq_l)  

                                        elif (sender_Type_p == self.__SENDER_TYPE_VAG):
                                            uint32arr_buff_l = self._combine24Bits(msg_l[1:], data_len_l - 1)                                    

                                            # print(uint32arr_buff_l[0,0])

                                            self._fct_callback(uint32arr_buff_l)

                                    elif (PACKET_HEADER_GROUPx == self.__PACKET_HEADER_GROUP2): # GROUP2 stands for the data obtained from GPIO.

                                        if (msg_l[1] == self.__GPIO_LED_BATTERY_READ_triggered):
                                            hbyte = msg_l[2]
                                            lbyte = msg_l[3]

                                            uint16arr_buff_l = self._combineHighByteLowByte(msg_l[2:], 2)

                                            # print("Battery level:", uint16arr_buff_l)
                                            print(uint16arr_buff_l[0,0])
                                        else:
                                            print(msg_l[1])

                                    del msg_l

                                    recv_state_l        = self.__COM_TCP_RECV_WAITFOR_DATALENGTH
                                    i_data_len_bytes_l  = 0

                finally:
                    # traceback.print_exc()                    

                    sock_l.shutdown(SHUT_RDWR)
                    sock_l.close()

                    print(self._Name + ": Close socket and resume in 5 sec.")

                    time.sleep(5.0)

            except Exception as e:
                print(self._Name + ": Outermost Exception and resume in 5 sec.")
                # traceback.print_exc()

                time.sleep(5.0)

    def _bytes_to_uint64(self, bytes_array):
        return np.uint64((bytes_array[0] << 56) | (bytes_array[1] << 48) | (bytes_array[2] << 40) | (bytes_array[3] << 32) | (bytes_array[4] << 24) | (bytes_array[5] << 16) | (bytes_array[6] << 8) | bytes_array[7])

    def _bytes_to_uint32(self, bytes_array):
        return np.uint32((bytes_array[0] << 24) | (bytes_array[1] << 16) | (bytes_array[2] << 8) | bytes_array[3])

    def _mainV1(self):

        self._running = True

        N_bytesarr_buff_l = int((2*self._Fs)/self._Fs_senddata)
        bytesarr_buff_l = bytearray(N_bytesarr_buff_l)        

        while self._running:
            try:
                try:
                    ##### Open the TCP connection.
                    sock_l = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

                    # Connect the socket to the port where the server is listening
                    server_address_l = (self._server_ip, self._server_port)
                    print(self._Name + ": Connecting to " + str(server_address_l))
                    sock_l.connect(server_address_l)
                    
                    print(self._Name + ": Connected")
                    next_block_l = None    
                    while self._running:
                        tic_l = time.time()*1000

                        if next_block_l is None:
                            idx_bytesarr_buff_l = 0
                        else:
                            idx_bytesarr_buff_l = len(next_block_l)
                            bytesarr_buff_l[0:len(next_block_l)] = next_block_l

                        while self._running:
                            block_l = sock_l.recv(self._buff_socket_size)

                            idx_end_l, valid_block_l, next_block_l = \
                            self._chopChunk(chuck_p=block_l, idx_storage_p=idx_bytesarr_buff_l, N_storage_p=N_bytesarr_buff_l)                

                            bytesarr_buff_l[idx_bytesarr_buff_l:idx_end_l] = valid_block_l

                            idx_bytesarr_buff_l = idx_end_l

                            if idx_bytesarr_buff_l == N_bytesarr_buff_l:
                                idx_bytesarr_buff_l = 0
                                break

                        # Convert from an array of bytes to an array of 2-byte unsigned interger.
                        uint16arr_buff_l = self._combineHighByteLowByte(bytesarr_buff_l, N_bytesarr_buff_l)
                        N_uint16arr_buff_l = int(N_bytesarr_buff_l/2)

                        print(self._Name + ": Store " + str(len(bytesarr_buff_l)) + " bytes for " + str(time.time()*1000 - tic_l - 1000/self._Fs_senddata) + " ms.")

                        self._fct_callback(uint16arr_buff_l)
                finally:
                    print(self._Name + ": Closing socket")

                    sock_l.shutdown(SHUT_RDWR)
                    sock_l.close()

                    self._running = False       

            except Exception as e:
                print(self._Name + ": Fatal socket's problem")
                # traceback.print_exc()

                self._running = False        

    def _combine24Bits(self, bytesarr_buff_p, N_bytesarr_buff_p):
        N_bytes_per_sample = 3

        uint32arr_buff_l = np.zeros((1, int(N_bytesarr_buff_p/N_bytes_per_sample)), dtype=np.uint32)

        i_l = 0
        j_l = 0
        while i_l < N_bytesarr_buff_p:
            uint32arr_buff_l[0, j_l] = np.uint32( (bytesarr_buff_p[i_l] << 16) | (bytesarr_buff_p[i_l + 1] << 8) | bytesarr_buff_p[i_l + 2] )

            i_l = i_l + N_bytes_per_sample
            j_l = j_l + 1

        return uint32arr_buff_l

    def _combineHighByteLowByte(self, bytesarr_buff_p, N_bytesarr_buff_p):
        # Returns:
        #     uint16arr_buff_l    : It is a numpy array. np.shape(uint16arr_buff_l) == (1, N_bytesarr_buff_p/2).

        uint16arr_buff_l = np.zeros((1, int(N_bytesarr_buff_p/2)), dtype=np.uint16)

        i_l = 0
        j_l = 0
        while i_l < N_bytesarr_buff_p:
            
            uint16arr_buff_l[0, j_l] = np.uint16( (bytesarr_buff_p[i_l] << 8) | bytesarr_buff_p[i_l + 1] )

            i_l = i_l + 2            
            j_l = j_l + 1

        return uint16arr_buff_l

    def _chopChunk(self, chuck_p, idx_storage_p, N_storage_p):

        N_chuck_p = len(chuck_p)

        if idx_storage_p + N_chuck_p <= N_storage_p:
            idx_end_l = idx_storage_p + N_chuck_p

            valid_block_l   = chuck_p
            next_block_l    = None
        else:
            idx_end_l = N_storage_p

            valid_block_l   = chuck_p[0:(N_storage_p - idx_storage_p)]
            next_block_l    = chuck_p[(N_storage_p - idx_storage_p):]

        return idx_end_l, valid_block_l, next_block_l

class iAwareMuseMonitorThread(EEGInterface, Thread):
    
    def __init__(self, fct_callback_p, server_ip_p, server_port_p, Name_p):

        super(iAwareMuseMonitorThread, self).__init__(Fs_p=256, Name_p=Name_p, N_channels_p=4)

        self._running = True

        self._server_ip = server_ip_p
        self._server_port = server_port_p

        self._fct_callback = fct_callback_p

    def terminate(self):
        self._running = False

        print(self._Name + ": Start to shutdown")
        self._MUSEOSCserver.shutdown()
        print(self._Name + ": Start to close")
        self._MUSEOSCserver.server_close()
        print(self._Name + ": Done")

    def run(self):
        self._running = True

        while self._running:
            try:
                try:
                    parser_l = argparse.ArgumentParser()
                    parser_l.add_argument("--ip", default=self._server_ip, help="The ip to listen on")
                    parser_l.add_argument("--port", type=int, default=self._server_port, help="The port to listen on")
                    args_l = parser_l.parse_args()

                    dispatcher_l = dispatcher.Dispatcher()
                    dispatcher_l.map("/muse/eeg", self._eeg_handler, "EEG") # unused_addr = /muse/eeg, args = args[0] = EEG

                    self._MUSEOSCserver = osc_server.BlockingOSCUDPServer((args_l.ip, args_l.port), dispatcher_l)
                    print("Serving on {}".format(self._MUSEOSCserver.server_address))

                    self._MUSEOSCserver.serve_forever()

                except Exception as e:
                    print("Start to shutdown")
                    self._MUSEOSCserver.shutdown()
                    print("Done")
                    print("Start to close")
                    self._MUSEOSCserver.server_close()
                    print("Done")    

                    self._running = False

            except Exception as e:
                print(self._Name + ": Fatal socket's problem")
                traceback.print_exc()

                self._running = False

    def _eeg_handler(self, unused_addr, args, TP9, AF7, AF8, TP10, AUX):

        sample_l = np.ndarray((self._N_channels, 1), dtype=np.float32)  

        sample_l[0, 0] = TP9
        sample_l[1, 0] = AF7
        sample_l[2, 0] = AF8
        sample_l[3, 0] = TP10

        self._fct_callback(sample_l)

class EEGClientThread(Thread):

    def __init__(self, N_channels_p, samples_duration_p, img_Desired_freq_step_p=0.5, T_EEG_data_fft_p=0.2):
        """
        Params:
            N_channels_p        : The number of channel.
            samples_duration_p  : The duration of samples that we want to record.
            img_Desired_freq_step_p : [Hz.]
            T_EEG_data_fft_p    : [sec.], The duration of the signal that we use to calculate the FFT power. 
        """

        super(EEGClientThread, self).__init__()

        self._N_channels = N_channels_p

        self._idx_samples = 0
        self._samples_duration = samples_duration_p # [sec.]
        self._samples = None
        self._total_transmitted_samples = 0
        self._eff_sampling_freq = np.nan

        # FFT parameters
        self._T_EEG_data_fft = T_EEG_data_fft_p
        print("EEGClientThread: Meaningful at-least frequency in one duration for calculation FFT: " + str(3/T_EEG_data_fft_p) + " Hz")

        self._img_Desired_freq_step = img_Desired_freq_step_p

        # Various EEG Clients
        self._TCPClient = None
        self._MuseMonitorClient = None
        self._MindWaveMobile2 = None

    ##### Muse monitor
    def createMuseMonitorClient(self, server_ip_p, server_port_p, Name_p):

        self._init(Fs_p=256)

        self._MuseMonitorClient = iAwareMuseMonitorThread(fct_callback_p=self.callback, server_ip_p=server_ip_p, server_port_p=server_port_p, Name_p=Name_p)

    ##### iAware
    def createTCPClient(self, buff_socket_size_p, server_ip_p, server_port_p, Fs_senddata_p, Fs_p, Name_p):
        
        self._init(Fs_p=Fs_p)

        self._TCPClient = iAwareTCPThread(fct_callback_p=self.callback, buff_socket_size_p=buff_socket_size_p, \
            server_ip_p=server_ip_p, server_port_p=server_port_p, \
            Fs_senddata_p=Fs_senddata_p, Fs_p=Fs_p, \
            Name_p=Name_p, N_channels_p=self._N_channels)

    ##### MindWave Mobile 2 
    def createMindWaveMobile2Client(self, Name_p, COM_p, baudrate_p=57600):

        self._init(Fs_p=512)

        self._MindWaveMobile2 = MindWaveMobile2Thread(fct_callback_p=self.callback, Name_p=Name_p, COM_p=COM_p, baudrate_p=baudrate_p)

    ##### Public
    def callback(self, samples_p, eff_sampling_freq=-1):
        """
        Params:
            samples_p   : It is numpy.ndarrays where numpy.shape(samples_p)[0] = N_channels_p and numpy.shape(samples_p)[1] is the number of samples.
            eff_sampling_freq  : [Hz]. It is the effective sampling frequency that the source collects the samples.
        """

        samples_l = np.copy(samples_p)

        self._total_transmitted_samples = self._total_transmitted_samples + np.shape(samples_l)[1]

        if eff_sampling_freq > 0.0:
            self._eff_sampling_freq = eff_sampling_freq
            # print(eff_sampling_freq)
        else:
            self._eff_sampling_freq = np.nan

        self._appendSamples(chunk_p=samples_l)

    def terminate(self):

        if self._TCPClient is not None:
            self._TCPClient.terminate()

            del self._TCPClient; self._TCPClient = None

        if self._MuseMonitorClient is not None:
            self._MuseMonitorClient.terminate()

            del self._MuseMonitorClient; self._MuseMonitorClient = None

        if self._MindWaveMobile2 is not None:
            self._MindWaveMobile2.terminate()

            del self._MindWaveMobile2; self._MindWaveMobile2 = None

        del self._range_samples; self._range_samples = None
        del self._range_chunk; self._range_chunk = None
        del self._samples; self._samples = None

    def run(self):

        if self._TCPClient is not None:

            self._TCPClient.start()
            self._TCPClient.join()

        elif self._MuseMonitorClient is not None:

            self._MuseMonitorClient.start()
            self._MuseMonitorClient.join()

        elif self._MindWaveMobile2 is not None:

            self._MindWaveMobile2.start()
            self._MindWaveMobile2.join()

        print("EEGClientThread: Terminated.")
    
    def getTotalSamples(self, is_reset_p=True):

        tmp = self._total_transmitted_samples

        if is_reset_p:
            self._total_transmitted_samples = 0

        return tmp

    def getSamplingRate(self):
        return self._Fs

    def getSamplingFreq(self):
        return self._Fs

    def get_T_EEG_data_fft(self):
        return self._T_EEG_data_fft        

    def getEffSamplingFreq(self):
        return self._eff_sampling_freq

    def getASample(self):
        return self._samples[:, 0]

    #use this function to dump raw value
    def getRawData(self):
        return self._idx_samples, np.copy(self._samples)

    def getFFTPower(self):

        # try:

        #     for idx_Chns_l in range(self._N_channels):

        #         modulo_i_EEG_data_fft_l   = np.take(a=self._idx_EEG_data_fft, indices=(self._idx_samples + self._idx_EEG_data_fft), mode="wrap")

        #         if self._is_Padded:

        #             N_begin_padd_l  = int(np.floor((self._img_N_EEG_data_fft - self._N_EEG_data_fft)/2.0))
        #             N_end_padd_l    = int(np.ceil((self._img_N_EEG_data_fft - self._N_EEG_data_fft)/2.0))
                    
        #             # [3, 2, 1, 2, 3, 4, 5, 4, 3, 2]
        #             ch_l = np.pad(self._samples[idx_Chns_l, modulo_i_EEG_data_fft_l], (N_begin_padd_l, N_end_padd_l), "reflect")


        #             # ch1_l = np.pad(self._samples[idx_Chns_l, modulo_i_EEG_data_fft_l], (N_begin_padd_l, N_end_padd_l), 'constant', constant_values=(0, 0))
                    
        #         else:
        #             ch_l = self._samples[idx_Chns_l, modulo_i_EEG_data_fft_l]

        #         freq_positive_l, _, _, fft_pw_ch1_l, _, _, _    = self._AV_ShortFFT(fs_p=self._Fs, signal_p=ch_l - np.mean(ch_l), typeTaper_p="hann", taper_p=self._taper_EEG_data_fft)

        #         return freq_positive_l, fft_pw_ch1_l
        # except Exception as e:
        #     pass

        try:
            fft_pw_all_chs_l =[]

            for idx_Chns_l in range(self._N_channels):

                modulo_i_EEG_data_fft_l   = np.take(a=self._idx_EEG_data_fft, indices=(self._idx_samples + self._idx_EEG_data_fft), mode="wrap")
                N_begin_padd_l  = int(np.floor((self._img_N_EEG_data_fft - self._N_EEG_data_fft)/2.0))
                N_end_padd_l    = int(np.ceil((self._img_N_EEG_data_fft - self._N_EEG_data_fft)/2.0))
                ch_l = np.pad(self._samples[idx_Chns_l, modulo_i_EEG_data_fft_l], (N_begin_padd_l, N_end_padd_l), "reflect")
                
                if self._is_Padded:

                    N_begin_padd_l  = int(np.floor((self._img_N_EEG_data_fft - self._N_EEG_data_fft)/2.0))
                    N_end_padd_l    = int(np.ceil((self._img_N_EEG_data_fft - self._N_EEG_data_fft)/2.0))
                    
                    # [3, 2, 1, 2, 3, 4, 5, 4, 3, 2]
                    ch_l = np.pad(self._samples[idx_Chns_l, modulo_i_EEG_data_fft_l], (N_begin_padd_l, N_end_padd_l), "reflect")


                    # ch1_l = np.pad(self._samples[idx_Chns_l, modulo_i_EEG_data_fft_l], (N_begin_padd_l, N_end_padd_l), 'constant', constant_values=(0, 0))
                    
                else:
                    ch_l = self._samples[idx_Chns_l, modulo_i_EEG_data_fft_l]

                freq_positive_l, _, _, fft_pw_ch1_l, _, _, _    = self._AV_ShortFFT(fs_p=self._Fs, signal_p=ch_l - np.mean(ch_l), typeTaper_p="hann", taper_p=self._taper_EEG_data_fft)
                fft_pw_all_chs_l.append(fft_pw_ch1_l)
                
            return freq_positive_l, fft_pw_all_chs_l
        except Exception as e:
            pass
            
    ##### Private
    def _init(self, Fs_p):
        self._Fs = Fs_p
        self._idx_samples = 0
        self._total_transmitted_samples = 0
        self._samples = np.full((self._N_channels, int(Fs_p*self._samples_duration)), np.nan, dtype=np.float32)
        self._range_samples = np.arange(int(Fs_p*self._samples_duration))
        self._range_chunk = None

        self._N_EEG_data_fft = int(Fs_p*self._T_EEG_data_fft)
        self._idx_EEG_data_fft = np.arange(0, self._N_EEG_data_fft)

        if self._img_Desired_freq_step < (1/self._T_EEG_data_fft):
            self._is_Padded = True

            self._img_N_EEG_data_fft = self._AV_next_power_of_2(int(Fs_p*(1/self._img_Desired_freq_step)))

            self._img_Desired_freq_step = Fs_p/self._img_N_EEG_data_fft
            print("The frequency resolution in FFT image is changed from " + str(1/self._T_EEG_data_fft) + " Hz to " + str(self._img_Desired_freq_step) + " Hz (follow the power of 2 FFT speed improvement)")        
        else:
            self._is_Padded = False

            self._img_N_EEG_data_fft = int(Fs_p*self._T_EEG_data_fft)

            print("The frequency resolution in FFT image is " + str(1/self._T_EEG_data_fft) + " Hz")

        _, self._taper_EEG_data_fft = self._AV_taper_signal(signal_p=np.zeros(self._img_N_EEG_data_fft), typeTaper_p="hann")

    def _appendSamples(self, chunk_p):

        try:
            if self._range_chunk is None:
                self._range_chunk = np.arange((np.shape(chunk_p))[1])

            range_samples_chunk_l = np.take(a=self._range_samples, indices=(self._idx_samples + self._range_chunk), mode='wrap')

            for i_l in range(self._N_channels):

                self._samples[i_l, range_samples_chunk_l] = chunk_p[i_l]

            self._idx_samples = range_samples_chunk_l[-1] + 1
        except Exception as e:
            pass


    def _AV_next_power_of_2(self, x_p):
        return int(1 if (x_p == 0) else 2**math.ceil(math.log2(x_p)))

    def _AV_taper_signal(self, signal_p, typeTaper_p="hann", taper_p=None):
        # This function will taper the signal mainly because we want to reduce the edge artifacts before applying FFT.

        if taper_p is None:
            pnts_l = len(signal_p)
            if (typeTaper_p == "hann"):
                # With Hann taper
                hannw_l     = 0.5 - 0.5*np.cos(2*np.pi*np.linspace(0.0, 1.0, pnts_l))
                signal_l    = np.multiply(signal_p, hannw_l)
                taper_l     = hannw_l


            if (typeTaper_p == "hamming"):
                # With Hamming taper
                hammingw_l  = .54 - .46*np.cos(2*np.pi*np.linspace(0.0, 1.0, pnts_l))
                signal_l    = np.multiply(signal_p, hammingw_l)
                taper_l     = hammingw_l

            if (typeTaper_p == "gaussian"):
                # With Gaussian taper
                gaussw_l    = np.exp(-np.square(time_l))
                signal_l    = np.multiply(signal_p, gaussw_l)
                taper_l     = gaussw_l
        else:
            signal_l    = np.multiply(signal_p, taper_p)
            taper_l     = taper_p

        return signal_l, taper_l

    def _AV_ShortFFT(self, fs_p, signal_p, typeTaper_p="hann", taper_p=None):
        tapered_signal_l, taper_l = self._AV_taper_signal(signal_p=signal_p, typeTaper_p=typeTaper_p, taper_p=taper_p)
        freq_positive_p, fft_p, fft_amp_p, fft_pw_p, fft_gain_p = self._AV_fft(fs_p=fs_p, signal_p=tapered_signal_l)

        return freq_positive_p, fft_p, fft_amp_p, fft_pw_p, fft_gain_p, tapered_signal_l, taper_l

    def _AV_fft(self, fs_p, signal_p):
        """
        The function returns only the positive-frequency term.
        See https://docs.scipy.org/doc/scipy/reference/generated/scipy.fftpack.fft.html
        """

        try:
            print("AV_fft: Wrong signal_p dimension %d" % (int(np.shape(signal_p)[1])))
        except IndexError:    
            N_signal_l = len(signal_p)
            fft_l = fftpack.fft(signal_p)
            freq_l = np.fft.fftfreq(N_signal_l, d = (1/fs_p))

            if ((N_signal_l % 2) == 0):
                freq_positive_l = freq_l[0:((N_signal_l//2))]
                fft_l = fft_l[0:((N_signal_l//2))]
            else:
                freq_positive_l = freq_l[0:(((N_signal_l - 1)//2) + 1)]
                fft_l = fft_l[0:(((N_signal_l - 1)//2) + 1)]


            fft_amp_l = 2.0*np.abs(fft_l)/N_signal_l
            fft_pw_l = fft_amp_l**2
            fft_gain_l = np.abs(fft_l)

            # fft_amp_l has the same unit as signal_p. [fft_amp_l] = [signal_p]
            # [fft_pw_l] = [fft_amp_l]*[fft_amp_l], i.e. if the unit of the signal is microvolt, the unit of the power (fft_pw_l) is microvolt^2
            return freq_positive_l, fft_l, fft_amp_l, fft_pw_l, fft_gain_l

    def _AV_fft_getfreq(self, fs_p, signal_p):
        # return freq_positive_l

        try:
            print("AV_fft_getfreq: Wrong signal_p dimension %d" % (int(np.shape(signal_p)[1])))
        except IndexError:    
            N_signal_l = len(signal_p)
            freq_l = np.fft.fftfreq(N_signal_l, d = (1/fs_p))

            if ((N_signal_l % 2) == 0):
                freq_positive_l = freq_l[0:((N_signal_l//2))]
            else:
                freq_positive_l = freq_l[0:(((N_signal_l - 1)//2) + 1)]

            return freq_positive_l            

    def _AV_reflected_zerophaseshift_filter(self, signal_p, filtkern_b_p=None, filtkern_a_p=None, sos_p=None):
        # return fsignal_l

        if (sos_p is None):
            if (filtkern_a_p is None):
                filtkern_a_p = 1

                order_l = len(filtkern_b_p)

                relected_signal_l = np.append(np.append(np.flip(signal_p[0:order_l]), signal_p), (np.flip(signal_p))[0:order_l])

                # 1) Forward 2) Reverse 3) Flip forward
                relected_signal_l = np.flip(signal.lfilter(filtkern_b_p, filtkern_a_p, np.flip(signal.lfilter(filtkern_b_p, filtkern_a_p, relected_signal_l))))

                end_l = len(relected_signal_l)
                fsignal_l = relected_signal_l[order_l:(end_l - order_l)]
            else:
                fsignal_l = signal.filtfilt(filtkern_b_p, filtkern_a_p, signal_p)
        else:
            fsignal_l = signal.sosfiltfilt(sos_p, signal_p)

        return fsignal_l    

def _test2(self):
    # "packetParser runs continously in a separate thread to parse packets from mindwave and update the corresponding variables"
    while self.__threadRun:
        p1 = self.__srl.read(1).encode("hex")  # read first 2 packets
        p2 = self.__srl.read(1).encode("hex")
        while (p1 != 'aa' or p2 != 'aa') and self.__threadRun:
            p1 = p2
            p2 = self.__srl.read(1).encode("hex")
        else:
            if self.__threadRun == False:
                break
            # a valid packet is available
            self.__packetsReceived += 1
            payload = []
            checksum = 0
            payloadLength = int(self.__srl.read(1).encode("hex"), 16)
            for i in range(payloadLength):
                tempPacket = self.__srl.read(1).encode("hex")
                payload.append(tempPacket)
                checksum += int(tempPacket, 16)
            checksum = ~checksum & 0x000000ff
            if checksum == int(self.__srl.read(1).encode("hex"), 16):
                i = 0

                while i < payloadLength:
                    code = payload[i]
                    if (code == 'd0'):
                        print("Headset connected!")
                        self.__connected = True
                    elif (code == 'd1'):
                        print("Headset not found, reconnecting")
                        self.connect()
                    elif(code == 'd2'):
                        print("Disconnected!")
                        self.connect()
                    elif(code == 'd3'):
                        print("Headset denied operation!")
                    elif(code == 'd4'):
                        if payload[2] == 0 and not self.__connected:
                            print("Idle, trying to reconnect");
                            self.connect();
                    elif(code == '02'):  # poorSignal
                        i = i + 1
                        self.poorSignal = int(payload[i], 16)
                    elif(code == '04'):  # attention
                        i = i + 1
                        self.attention = int(payload[i], 16)
                    elif(code == '05'):  # meditation
                        i = i + 1
                        self.meditation = int(payload[i], 16)
                    elif(code == '16'):  # blink strength
                        i = i + 1
                        self.blinkStrength = int(payload[i], 16)
                    elif(code == '80'):  # raw value
                        i = i + 1  # for length/it is not used since length =1 byte long and always=2
                        i = i + 1
                        val0 = int(payload[i], 16)
                        i = i + 1
                        self.rawValue = val0 * 256 + int(payload[i], 16)
                        if self.rawValue > 32768:
                            self.rawValue = self.rawValue - 65536
                    elif(code == '83'):  # ASIC_EEG_POWER
                        i = i + 1  # for length/it is not used since length =1 byte long and always=2
                        # delta:
                        i = i + 1
                        val0 = int(payload[i], 16)
                        i = i + 1
                        val1 = int(payload[i], 16)
                        i = i + 1
                        self.delta = val0 * 65536 + \
                            val1 * 256 + int(payload[i], 16)
                        # theta:
                        i = i + 1
                        val0 = int(payload[i], 16)
                        i = i + 1
                        val1 = int(payload[i], 16)
                        i = i + 1
                        self.theta = val0 * 65536 + \
                            val1 * 256 + int(payload[i], 16)
                        # lowAlpha:
                        i = i + 1
                        val0 = int(payload[i], 16)
                        i = i + 1
                        val1 = int(payload[i], 16)
                        i = i + 1
                        self.lowAlpha = val0 * 65536 + \
                            val1 * 256 + int(payload[i], 16)
                        # highAlpha:
                        i = i + 1
                        val0 = int(payload[i], 16)
                        i = i + 1
                        val1 = int(payload[i], 16)
                        i = i + 1
                        self.highAlpha = val0 * 65536 + \
                            val1 * 256 + int(payload[i], 16)
                        # lowBeta:
                        i = i + 1
                        val0 = int(payload[i], 16)
                        i = i + 1
                        val1 = int(payload[i], 16)
                        i = i + 1
                        self.lowBeta = val0 * 65536 + \
                            val1 * 256 + int(payload[i], 16)
                        # highBeta:
                        i = i + 1
                        val0 = int(payload[i], 16)
                        i = i + 1
                        val1 = int(payload[i], 16)
                        i = i + 1
                        self.highBeta = val0 * 65536 + \
                            val1 * 256 + int(payload[i], 16)
                        # lowGamma:
                        i = i + 1
                        val0 = int(payload[i], 16)
                        i = i + 1
                        val1 = int(payload[i], 16)
                        i = i + 1
                        self.lowGamma = val0 * 65536 + \
                            val1 * 256 + int(payload[i], 16)
                        # midGamma:
                        i = i + 1
                        val0 = int(payload[i], 16)
                        i = i + 1
                        val1 = int(payload[i], 16)
                        i = i + 1
                        self.midGamma = val0 * 65536 + \
                            val1 * 256 + int(payload[i], 16)
                    else:
                        pass
                    i = i + 1

def _test():

    import serial


    mac_addr = "00:81:F9:2A:C4:32"

    com_port = "COM11"

    STATE_1 = 1
    STATE_2 = 2
    STATE_3 = 3
    STATE_4 = 4
    STATE_6 = 6

    CONNECT             = b'\xc0'
    DISCONNECT          = b'\xc1'
    AUTOCONNECT         = b'\xc2'
    SYNC                = b'\xaa'
    EXCODE              = b'\x55'
    HEADSET_CONNECTED   = b'\xd0'
    HEADSET_NOT_FOUND   = b'\xd1'
    HEADSET_DISCONNECTED= b'\xd2'
    REQUEST_DENIED      = b'\xd3'
    STANDBY_SCAN        = b'\xd4'

    POOR_SIG_QUALITY    = b'\x02'
    ATTENTION           = b'\x04'
    MEDITATION          = b'\x05'
    BLINK               = b'\x16'
    RAW_WAVE            = b'\x80'
    ASIC_EEG_POWER      = b'\x83'

    TGAM1_CMD_9600_NORMAL       = b'0x00' # 9600 baud, normal output mode
    TGAM1_CMD_1200_NORMAL       = b'0x01' # 1200 baud, normal output mode
    TGAM1_CMD_57600_NORMAL_RAW  = b'0x02' # 57.6k baud, normal+raw output mode 
    TGAM1_CMD_GARBAGE           = b'0xAB' # 

    try:
        print("Open/", com_port)

        srl_l = serial.Serial()
        # srl_l.baudrate = 115200
        srl_l.baudrate = 57600
        srl_l.port = com_port
        srl_l.open()

        # time.sleep(10)

        if srl_l.isOpen() == False:
            print("Cannot open serial.")

            exit()

        # # Re-apply settings to ensure packet stream
        # srl_l.write(DISCONNECT)
        # srl_dict_l = srl_l.getSettingsDict()
        # for i_l in range(2):
        #     srl_dict_l["rtscts"] = not srl_dict_l["rtscts"] # http://www.brainboxes.com/faq/items/what-is-rts--cts-hardware-flow-control-
        #     srl_l.applySettingsDict(srl_dict_l)

        raw_cnt_l = 0

        t_b_l = time.time()

        try:

            state_l = STATE_1

            while True:

                a_byte_l = srl_l.read(1)

                if (state_l == STATE_1):

                    if a_byte_l == SYNC:
                        state_l = STATE_2
                    else:
                        state_l = STATE_1

                elif (state_l == STATE_2):

                    if a_byte_l == SYNC:
                        state_l = STATE_3
                    else:
                        state_l = STATE_1

                elif (state_l == STATE_3):

                    if a_byte_l == SYNC:
                        state_l = STATE_3
                    elif int.from_bytes(a_byte_l, byteorder="big") > 170: # PLENGTH TOO LARGE
                        state_l = STATE_1
                    else:
                        pLength_l = int.from_bytes(a_byte_l, byteorder="big") # Length of PAYLOAD. It is between 0 and 169.

                        if pLength_l > 0: # If we encounter non-empty PAYLOAD, we fetch PAYLOAD data.

                            payload_l = []
                            checksum_l = 0

                            state_l = STATE_4

                        else:
                            print("Found empty PAYLOAD")

                            state_l = STATE_1


                elif (state_l == STATE_4):

                    tempPacket = a_byte_l.hex()

                    payload_l.append(tempPacket)

                    checksum_l += int(tempPacket, 16)

                    if len(payload_l) == pLength_l:
                        checksum_l = ~checksum_l & 0x000000ff

                        state_l = STATE_6


                elif (state_l == STATE_6):

                    if checksum_l == int(a_byte_l.hex(), 16): # Checksum is calculated using PAYLOAD data.
                        
                        i_payload_l = 0

                        while i_payload_l < len(payload_l): # Parse PAYLOAD data.

                            # Count the number of EXCODE, indicating the Extended Code Level.
                            n_EXCODE_l  = 0
                            while payload_l[i_payload_l] == EXCODE.hex():
                                i_payload_l += 1
                                n_EXCODE_l  += 1

                            if n_EXCODE_l == 0:
                                # The Extended Code Level 0.

                                if payload_l[i_payload_l] == RAW_WAVE.hex():
                                    i_payload_l += 1

                                    vLength_l = int(payload_l[i_payload_l], 16)
                                    i_payload_l += 1

                                    assert vLength_l == 2

                                    val0 = int(payload_l[i_payload_l], 16)
                                    i_payload_l += 1

                                    rawValue_l = (val0*256) + int(payload_l[i_payload_l], 16)
                                    i_payload_l += 1
            
                                    if rawValue_l > 32768:
                                        rawValue_l = rawValue_l - 65536
                            
                                    assert (-32768 <= rawValue_l) and (rawValue_l <= 32767)
                                    
                                    # print("RAW_WAVE: (", pLength_l, ") ", rawValue_l) 

                                    raw_cnt_l += 1

                                elif payload_l[i_payload_l] == ASIC_EEG_POWER.hex():
                                    i_payload_l += 1

                                    vLength_l = int(payload_l[i_payload_l], 16)
                                    i_payload_l += 1

                                    assert vLength_l == 24

                                    def _ASIC_EEG_POWER(payload_p, i_payload_p):
                                    
                                        val0        = int(payload_p[i_payload_p], 16)
                                        i_payload_p += 1

                                        val1        = int(payload_p[i_payload_p], 16)
                                        i_payload_p += 1

                                        power_l     = (val0*65536) + (val1*256) + int(payload_p[i_payload_p], 16)
                                        i_payload_p += 1

                                        return i_payload_p, power_l

                                    i_payload_l, delta_l    = _ASIC_EEG_POWER(payload_l, i_payload_l)
                                    i_payload_l, theta_l    = _ASIC_EEG_POWER(payload_l, i_payload_l)
                                    i_payload_l, lowAlpha_l = _ASIC_EEG_POWER(payload_l, i_payload_l)
                                    i_payload_l, highAlpha_l= _ASIC_EEG_POWER(payload_l, i_payload_l)
                                    i_payload_l, lowBeta_l  = _ASIC_EEG_POWER(payload_l, i_payload_l)
                                    i_payload_l, highBeta_l = _ASIC_EEG_POWER(payload_l, i_payload_l)
                                    i_payload_l, lowGamma_l = _ASIC_EEG_POWER(payload_l, i_payload_l)
                                    i_payload_l, midGamma_l = _ASIC_EEG_POWER(payload_l, i_payload_l)

                                    print("ASIC_EEG_POWER: (", pLength_l, ") ", delta_l, "/", theta_l, "/", lowAlpha_l, "/", highAlpha_l, "/", lowBeta_l, "/", highBeta_l, "/", lowGamma_l, "/", midGamma_l)

                                elif payload_l[i_payload_l] == POOR_SIG_QUALITY.hex():
                                    i_payload_l += 1

                                    value_l     = int(payload_l[i_payload_l], 16)
                                    i_payload_l += 1

                                    print("POOR_SIG_QUALITY: (", pLength_l, ") ", value_l)
                                elif payload_l[i_payload_l] == ATTENTION.hex():
                                    i_payload_l += 1

                                    value_l     = int(payload_l[i_payload_l], 16)                                        
                                    i_payload_l += 1

                                    print("ATTENTION: (", pLength_l, ") ", value_l)
                                elif payload_l[i_payload_l] == MEDITATION.hex():
                                    i_payload_l += 1

                                    value_l     = int(payload_l[i_payload_l], 16)                                        
                                    i_payload_l += 1

                                    print("MEDITATION: (", pLength_l, ") ", value_l)
                                elif payload_l[i_payload_l] == BLINK.hex():
                                    i_payload_l += 1

                                    value_l     = int(payload_l[i_payload_l], 16)
                                    i_payload_l += 1

                                    print("BLINK: (", pLength_l, ") ", value_l)                                        
                                else:
                                    print("Undefined Level Zero Code:", payload_l[i_payload_l])

                                    exit()
                            else:
                                print("Unsupported Extended Code Level:", n_EXCODE_l)

                                exit()

                    else:
                        print("Incorrect checksum")

                    state_l = STATE_1


        except KeyboardInterrupt:

            srl_l.close()
            print("Close")

            pass

        srl_l.close()
        print("Close")
    except serial.serialutil.SerialException as e:
        print(str(e))
        return

    # do stuff
    elapsed_l = time.time() - t_b_l # sec.

    print("effective sampling frequency:", raw_cnt_l/elapsed_l, " Hz")

def _test1():

    import os
    import time
    import select
    import serial
    import struct
    import datetime
    import threading

    from pprint import pprint


    # Byte codes
    CONNECT =               b'\xc0'
    DISCONNECT =            b'\xc1'
    AUTOCONNECT =           b'\xc2'
    SYNC =                  b'\xaa'
    EXCODE =                b'\x55'
    POOR_SIGNAL =           b'\x02'
    ATTENTION =             b'\x04'
    MEDITATION =            b'\x05'
    BLINK =                 b'\x16'
    HEADSET_CONNECTED =     b'\xd0'
    HEADSET_NOT_FOUND =     b'\xd1'
    HEADSET_DISCONNECTED =  b'\xd2'
    REQUEST_DENIED =        b'\xd3'
    STANDBY_SCAN =          b'\xd4'
    RAW_VALUE =             b'\x80'
    ASIC_EEG_POWER =        b'\x83'

    # Status codes
    STATUS_CONNECTED = 'connected'
    STATUS_SCANNING = 'scanning'
    STATUS_STANDBY = 'standby'


    class Headset(object):
        """
        A MindWave Headset
        """
        class DongleListener(threading.Thread):
            """
            Serial listener for dongle device.
            """
            def __init__(self, headset, *args, **kwargs):
                """Set up the listener device."""
                self.headset = headset
                self.counter = 0
                super(Headset.DongleListener, self).__init__(*args, **kwargs)

            def run(self):
                """Run the listener thread."""
                s = self.headset.dongle

                self.headset.running = True

                # Re-apply settings to ensure packet stream
                s.write(DISCONNECT)
                d = s.getSettingsDict()
                for i in range(2):
                    d['rtscts'] = not d['rtscts']
                    s.applySettingsDict(d)

                while self.headset.running:
                    # Begin listening for packets
                    try:
                        if s.read() == SYNC and s.read() == SYNC:
                            # Packet found, determine plength
                            while True:
                                plength = int.from_bytes(s.read(), byteorder='big')
                                if plength != 170:
                                    break
                            if plength > 170:
                                continue

                            # Read in the payload
                            payload = s.read(plength)

                            # # Verify its checksum
                            # val = sum(b for b in payload[:-1])
                            # val &= 0xff
                            # val = ~val & 0xff
                            # chksum = int.from_bytes(s.read(), byteorder='big')

                            # # if val == chksum:
                            # if True:  # ignore bad checksums
                            #     self.parse_payload(payload)

                            # Verify its checksum
                            val = 0
                            for b in payload:
                                val += b

                            val = ~val & 0x000000ff
                            chksum = int((s.read()).hex(), 16)
                            if val == chksum:
                                self.parse_payload(payload)                            


                    except serial.SerialException:
                        break
                    except (select.error, OSError):
                        break

                print('Closing connection...')
                if s and s.isOpen():
                    s.close()

            def parse_payload(self, payload):
                """Parse the payload to determine an action."""
                while payload:
                    # Parse data row
                    excode = 0
                    try:
                        code, payload = payload[0], payload[1:]
                        code_char = struct.pack('B',code)
                        self.headset.count = self.counter
                        self.counter = self.counter + 1
                        if (self.counter >= 100):
                            self.counter = 0
                    except IndexError:
                        pass
                    while code_char == EXCODE:
                        print('Excode bytes found')
                        # Count excode bytes
                        excode += 1
                        try:
                            code, payload = payload[0], payload[1:]
                        except IndexError:
                            pass

                    if code < 0x80:
                        # This is a single-byte code
                        try:
                            value, payload = payload[0], payload[1:]
                        except IndexError:
                            pass
                        if code_char == POOR_SIGNAL:
                            # Poor signal
                            old_poor_signal = self.headset.poor_signal
                            self.headset.poor_signal = value
                            if self.headset.poor_signal > 0:
                                if old_poor_signal == 0:
                                    for handler in self.headset.poor_signal_handlers:
                                        handler(self.headset,
                                                self.headset.poor_signal)
                            else:
                                if old_poor_signal > 0:
                                    for handler in self.headset.good_signal_handlers:
                                        handler(self.headset,
                                                self.headset.poor_signal)
                        elif code_char == ATTENTION:
                            # Attention level
                            self.headset.attention = value
                            for handler in self.headset.attention_handlers:
                                handler(self.headset, self.headset.attention)
                        elif code_char == MEDITATION:
                            # Meditation level
                            self.headset.meditation = value
                            for handler in self.headset.meditation_handlers:
                                handler(self.headset, self.headset.meditation)
                        elif code_char == BLINK:
                            # Blink strength
                            self.headset.blink = value
                            for handler in self.headset.blink_handlers:
                                handler(self.headset, self.headset.blink)
                    else:
                        # This is a multi-byte code
                        try:
                            vlength, payload = payload[0], payload[1:]
                        except IndexError:
                            continue
                        value, payload = payload[:vlength], payload[vlength:]

                        if code_char == RAW_VALUE and len(value) >= 2:
                            raw = value[0]*256+value[1]
                            if (raw >= 32768):
                                raw = raw-65536
                            self.headset.raw_value = raw
                            print("raw:", raw)
                            for handler in self.headset.raw_value_handlers:
                                handler(self.headset, self.headset.raw_value)
                        if code_char == HEADSET_CONNECTED:
                            # Headset connect success
                            run_handlers = self.headset.status != STATUS_CONNECTED
                            self.headset.status = STATUS_CONNECTED
                            self.headset.headset_id = value.encode('hex')
                            if run_handlers:
                                for handler in self.headset.headset_connected_handlers:
                                    handler(self.headset)
                        elif code_char == HEADSET_NOT_FOUND:
                            # Headset not found
                            if vlength > 0:
                                not_found_id = value.encode('hex')
                                for handler in self.headset.headset_notfound_handlers:
                                    handler(self.headset, not_found_id)
                            else:
                                for handler in self.headset.headset_notfound_handlers:
                                    handler(self.headset, None)
                        elif code_char == HEADSET_DISCONNECTED:
                            # Headset disconnected
                            headset_id = value.encode('hex')
                            for handler in self.headset.headset_disconnected_handlers:
                                handler(self.headset, headset_id)
                        elif code_char == REQUEST_DENIED:
                            # Request denied
                            for handler in self.headset.request_denied_handlers:
                                handler(self.headset)
                        elif code_char == STANDBY_SCAN:
                            # Standby/Scan mode
                            try:
                                byte = value[0]
                            except IndexError:
                                byte = None
                            if byte:
                                run_handlers = (self.headset.status != STATUS_SCANNING)
                                self.headset.status = STATUS_SCANNING
                                if run_handlers:
                                    for handler in self.headset.scanning_handlers:
                                        handler(self.headset)
                            else:
                                run_handlers = (self.headset.status != STATUS_STANDBY)
                                self.headset.status = STATUS_STANDBY
                                if run_handlers:
                                    for handler in self.headset.standby_handlers:
                                        handler(self.headset)
                        elif code_char == ASIC_EEG_POWER:
                            j = 0
                            for i in ['delta', 'theta', 'low-alpha', 'high-alpha', 'low-beta', 'high-beta', 'low-gamma', 'mid-gamma']:
                                self.headset.waves[i] = value[j]*255*255+value[j+1]*255+value[j+2]
                                j += 3
                            for handler in self.headset.waves_handlers:
                                handler(self.headset, self.headset.waves)

        def __init__(self, device, headset_id=None, open_serial=True):
            """Initialize the headset."""
            # Initialize headset values
            self.dongle = None
            self.listener = None
            self.device = device
            self.headset_id = headset_id
            self.poor_signal = 255
            self.attention = 0
            self.meditation = 0
            self.blink = 0
            self.raw_value = 0
            self.waves = {}
            self.status = None
            self.count = 0
            self.running = False

            # Create event handler lists
            self.poor_signal_handlers = []
            self.good_signal_handlers = []
            self.attention_handlers = []
            self.meditation_handlers = []
            self.blink_handlers = []
            self.raw_value_handlers = []
            self.waves_handlers = []
            self.headset_connected_handlers = []
            self.headset_notfound_handlers = []
            self.headset_disconnected_handlers = []
            self.request_denied_handlers = []
            self.scanning_handlers = []
            self.standby_handlers = []

            # Open the socket
            if open_serial:
                self.serial_open()

        def serial_open(self):
            """Open the serial connection and begin listening for data."""
            if not self.dongle or not self.dongle.isOpen():
                self.dongle = serial.Serial(self.device, 115200)

            if not self.listener or not self.listener.isAlive():
                self.listener = self.DongleListener(self)
                self.listener.daemon = True
                self.listener.start()

        def serial_close(self):
            """Close the serial connection."""
            self.dongle.close()

        def stop(self):
            self.running = False  
            
    import time
    import pandas as pd
    from datetime import datetime


    headset = Headset('COM11')
    time.sleep(10)

    # # print headers
    # wave = headset.waves
    # keys = ['raw', 'attention', 'meditation'] + list(wave.keys())
    # print(''.join(f'{k:<14}' for k in keys))

    # starttime = time.time()
    # values = []

    while True:

        time.sleep(1/256 - ((time.time() - starttime) % (1/256)))

    #     # print values
    #     wave = headset.waves
    #     values += [[datetime.now()] + [headset.raw_value, headset.attention, headset.meditation] + list(wave.values())]

    #     print(headset.raw_value)

    #     # print(''.join(f'{v:<14}' for v in values[-1]), end='\r')
        
    #     # # save data every 10 lines
    #     # if len(values) % 1024 == 0:
    #     #     df = pd.DataFrame(values)
    #     #     df.to_csv('raw.csv', mode='a', index=False, header=False)
    #     #     values = []  

def AV_fft(fs_p, signal_p, is_subtract_mean_p=False):
    """
    The function returns only the positive-frequency term.
    See https://docs.scipy.org/doc/scipy/reference/generated/scipy.fftpack.fft.html

    This function fixs the bug in AV_fft(). That is when len(signal_p) is even, in AV_fft() we forget to include the result at the nyquist frequency fs_p/2.
    But the result is correct when len(signal_p) is odd.
    """
    print(fs_p)
    try:
        print("AV_fft: Wrong signal_p dimension %d" % (int(np.shape(signal_p)[1])))
    except IndexError:    

        if is_subtract_mean_p:
            signal_p = copy.deepcopy(signal_p - np.mean(signal_p))

        N_signal_l = len(signal_p)
        fft_l = np.fft.fft(signal_p)
        freq_l = np.fft.fftfreq(N_signal_l, d = (1/fs_p))

        if ((N_signal_l % 2) == 0):
            freq_positive_l = np.abs(freq_l[0:( (N_signal_l//2) + 1 )])
            fft_l           = fft_l[0:( (N_signal_l//2) + 1)]
        else:
            freq_positive_l = freq_l[0:(((N_signal_l - 1)//2) + 1)]
            fft_l           = fft_l[0:(((N_signal_l - 1)//2) + 1)]


        fft_amp_l = 2.0*np.abs(fft_l)/N_signal_l
        fft_amp_l[0] /= 2.0 # We divide by 2.0 because there is no negative frequency of 0 Hz
        if ((N_signal_l % 2) == 0):
            fft_amp_l[-1] /= 2.0 # We divide by 2.0 because there is no negative frequency of the Nyquist frequency.

        fft_pw_l = fft_amp_l**2
        fft_gain_l = np.abs(fft_l)

        # fft_amp_l has the same unit as signal_p. [fft_amp_l] = [signal_p]
        # [fft_pw_l] = [fft_amp_l]*[fft_amp_l], i.e. if the unit of the signal is microvolt, the unit of the power (fft_pw_l) is microvolt^2
        # fft_gain_l is calculated based on the amplitude. Therefore, to convert to dB, we do 20*np.log10(fft_gain_l) because dB = 10*np.log10(power_output/power_input)
        # -3dB refers to the a frequency with which the power of output is reduced by half or the amplitude is reduced from 1 to sqrt(2).
        return freq_positive_l, fft_l, fft_amp_l, fft_pw_l, fft_gain_l

class EEGMonitorThread(Thread):

    def __init__(self, iaware_l, t_name):
        Thread.__init__(self)
        self.iaware_l = iaware_l
        self.t_name = t_name
        self.round = 0
        self.isRunning = True

    def run(self):
        # print("Hello " + self.firstName + " from " + self.name)
        # print("Heelo"+self.t_name)
        self._main()
        print("EEGMonitorThread: Terminated")
    
    def stop(self) :
        self.isRunning = False

    def getRawData(self) :
        return self.iaware_l.getRawData()[1][0]

    def _main(self) :
        print("hello",self.t_name)
        while True :
            # plt.plot([1, 2, 3, 4])
            # plt.ylabel('some numbers')
            # plt.show()
            rawEgg = self.iaware_l.getRawData()[1][0]
            if (not self.isRunning) : 
                break
            # print("iaware_log : ",rawEgg)
            # plt.plot(rawEgg)
            # plt.show()
            self.round += 1
            # if (self.round >= 20) :
            #     break
            time.sleep(1)
def setDDSFreq(ser="",freq=1) :
    setFreqCommand = 'WMF%s'%(str(freq))
    command = bytes(setFreqCommand,'ascii')+b'\x0a'
    print(command)
    return ser.read()

if __name__ == '__main__':

    # _test()
    # _test1()

    # iaware_l = EEGClientThread(N_channels_p=1, samples_duration_p=2)
    # iaware_l.createTCPClient(buff_socket_size_p=4096, server_ip_p="192.168.4.1", server_port_p=5000, Fs_senddata_p=10, Fs_p=500, Name_p="iAwareClient")

    # # iaware_l = EEGClientThread(N_channels_p=4, samples_duration_p=2)
    # # iaware_l.createMuseMonitorClient(server_ip_p="192.168.0.101", server_port_p=5000, Name_p="MuseMonitor")

    # comport_l = AV_ble_win10_getCOM("98:07:2D:7E:FF:3C") # MindWave
    #comport_l = AV_ble_win10_getCOM("00:81:F9:29:A3:7F") # MindWave
    #comport_l = AV_ble_win10_getCOM("00:81:f9:2a:c4:32") #MindWave MSI
    comport_l = "COM8"
    

    from numpy import load
    # if comport_l is not None:
    if True :
        # ser = serial.Serial(
        #     port = 'COM10',
        #     baudrate=115200,
        # )
        for freq in observeFreqRange(observeConfig.freqRange) :
            # ser.isOpen()
            # res = setDDSFreq(ser=ser,freq=freq)

            # iaware_l = EEGClientThread(N_channels_p=1, samples_duration_p=2)
            # iaware_l.createMindWaveMobile2Client(Name_p="iAware-MindwaveMobile2", COM_p=comport_l)

            # iaware_monitor = EEGMonitorThread(iaware_l=iaware_l,t_name="EEG_Monitor_test")
            # iaware_monitor.start()
            # iaware_l.start()
            # iaware_l.join()

            # lastResult = iaware_monitor.getRawData()

            # reArrangeResult = np.concatenate((lastResult[512:],lastResult[0:512]),axis=0 )

            # iaware_monitor.stop()

            # [freq_positive_l, fft_l, fft_amp_l, fft_pw_l, fft_gain_l] = AV_fft(fs_p=512,signal_p=reArrangeResult)
            # saveData = np.concatenate((reArrangeResult,freq_positive_l,fft_amp_l,fft_pw_l),axis=0)
            saveData = load('data.npy')

            samplingInfo = SamplingInfo(
                samplingRate=512,
                samplingTime=observeConfig.observeTime,
            )
  
            setting = DDSSetting(
                freq=freq,
                amp=1,
                offset=5,
                duty=0.1,
                phase=0,
            )

            profile = EEGProfile(
                rawValue=saveData,
                setting=setting,
                samplingInfo=samplingInfo,
            )

            profile.save()

    # mwmobile_l = MindWaveMobile2Thread(Name_p="Mobile")
    # print(mwmobile_l.getN_channels())
    # print(mwmobile_l.getSamplingRate())

    pass
