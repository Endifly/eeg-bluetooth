import observeConfig
from Lib.EEGProfile import observeFreqRange
import serial
from time import sleep

def setDDSFreq(ser="",freq=1) :
    setFreqCommand = 'WMF%s'%(str(freq*1000000))
    command = bytes(setFreqCommand,'ascii')+b'\x0a'
    print(command)
    ser.write(command)
    return ser.read()

def setDDSAmp(ser="",vol=0.001) :
    if (vol >= 0.01) :
      return
    setAmpCommand = 'WMA%s'%(str(vol))
    command = bytes(setAmpCommand,'ascii')+b'\x0a'
    print(command)
    ser.write(command)
    return ser.read()

ser = serial.Serial(
    port = observeConfig.signal_gen_port,
    baudrate=observeConfig.signal_gen_buardrate,
)
ser.isOpen()

input("Press Enter then inspect signal generator Ampliture\n It should be 0.001 V\n")
ampRes = setDDSAmp(ser=ser,vol=observeConfig.signal_gen_amp)

input("Press Enter then inspect signal generator Freq\n It should change from 0.001 KHz to 0.256 KHz\n")
for freq in observeFreqRange(observeConfig.freqRange) :
  freqRes = setDDSFreq(ser=ser,freq=freq)
  print('set freq to %s Hz'%(freq))
  sleep(0.4)

input("complete test comunication")