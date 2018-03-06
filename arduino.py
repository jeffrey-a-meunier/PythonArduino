import serial, time
import serial.tools.list_ports

__version__ = '2.5'
_versionDate = '2018.02.25'

'''
Use
  import arduino, time
  usbConnect()
  pinMode(13, 1)
  digitalWrite(13, 1)
  time.sleep(0.5)
  digitalWrite(13, 0)
  usbDisconnect()
'''

class _DisconnectedArduino:
  def __getattr__(self, _):
    raise Exception('Arduino not connected')

_the_arduino_ = _DisconnectedArduino()


# Version information ------------------------------------------------
def version():
  return (__version__, _versionDate)


# Connect and disconnect ---------------------------------------------
# These functions are wrappers around the Arduino class.

def usbConnect(*args, verbose=False, retryOnFail=True, **kwargs):
  global _the_arduino_
  if isinstance(_the_arduino_, _DisconnectedArduino):
    _the_arduino_ = Arduino()
  result = _the_arduino_.usbConnect(*args, verbose=verbose, **kwargs)
  if result or verbose or not retryOnFail:
    return result
  print('[INFO ] something went wrong, switching to verbose mode and retrying')
  return usbConnect(*args, verbose=True, **kwargs)

def usbDisconnect():
  global _the_arduino_
  _the_arduino_.usbDisconnect()

def guessUsbPort():
  # private internal functions
  def _devHwids(*ids):
    idStrs = ['={:04}:'.format(id) for id in ids]
    def _devHwids_(port):
      for idStr in idStrs:
        if idStr in port.hwid:
          return True
      return False
    return _devHwids_

  def _devNames(*names):
    def _devNames_(port):
      for name in names:
        if name in port.device:
          return True
      return False
    return _devNames_

  def _devDescrs(*descrs):
    def _devDescrs_(port):
      for descr in descrs:
        if descr in port.description:
          return True
      return False
    return _devDescrs_

  # UNO     : hardware ID 2341
  # Robo-1  : hardware ID 0403
  # RoboRED : hardware ID 2341

  # looks for Arduino on all platforms, this should always work
  devHwids = _devHwids(2341, 403)
  # backup for Mac in case the hwids check fails
  devNames = _devNames('cu.usbmodem', 'cu.usbserial', 'ttyACM', 'ttyUSB', 'arduino')
  # backup for Windows in case the devNames check fails
  devDescrs = _devDescrs('Arduino', 'USB Serial')
  ports = serial.tools.list_ports.comports()
  usbPorts = []
  for port in ports:
    if devHwids(port) or devNames(port) or devDescrs(port):
      usbPorts.append(port.device)
  return usbPorts

def portName():
  global _the_arduino_
  return _the_arduino_.portName()

# aliases
serialConnect = usbConnect
serialDisconnect = usbDisconnect
conn = usbConnect
disc = usbDisconnect
guess = guessUsbPort
  

# Analog I/O ---------------------------------------------------------

def analogRead(pin):
  global _the_arduino_
  return _the_arduino_.analogRead(pin)

def analogWrite(pin, value):
  global _the_arduino_
  _the_arduino_.analogWrite(pin, value)

def convert(value, fromLo, fromHi, toLo, toHi):
  '''Convert a value in the range fromLo - fromHi into a value
     in the range toLo - toHi.
     Returns an integer.

     Example:
       a0 = analogRead(0)
       d3 = convert(a0, 0, 1023, 0, 255)
       analogWrite(3, d3)
  '''
  value1 = (value - fromLo) / (fromHi - fromLo)
  value2 = value1 * (toHi - toLo) + toLo
  return int(value2)

# aliases
ar = analogRead
aw = analogWrite


# Digital I/O --------------------------------------------------------

def digitalRead(pin):
  global _the_arduino_
  return _the_arduino_.digitalRead(pin)

def digitalWrite(pin, value):
  global _the_arduino_
  _the_arduino_.digitalWrite(pin, value)

def pinMode(pin, mode):
  global _the_arduino_
  _the_arduino_.pinMode(pin, mode)

# aliases
dr = digitalRead
dw = digitalWrite
pm = pinMode


# LCD ----------------------------------------------------------------

def lcdConnect():
  global _the_arduino_
  return _the_arduino_.lcdConnect()

def lcdBacklight(status):
  global _the_arduino_
  return _the_arduino_.lcdBacklight(status)

def lcdClear():
  global _the_arduino_
  return _the_arduino_.lcdClear()

def lcdCursorBlink(status):
  global _the_arduino_
  return _the_arduino_.lcdCursorBlink(status)

def lcdCursorShow(status):
  global _the_arduino_
  return _the_arduino_.lcdCursorShow(status)

def lcdPrint(message):
  global _the_arduino_
  return _the_arduino_.lcdPrint(message)


#---------------------------------------------------------------------
# The class behind the functions
#---------------------------------------------------------------------
class Arduino:

  _EXPECTED_FIRMWARE_RESPONSE = b'11\r\n'

  def __init__(self):
    self.channel = None

  # Connect and disconnect -------------------------------------------

  def usbConnect(self, portName=None, failIfOpen=False, index=0, baudRate=115200, verbose=False):
    if (self.channel is not None):
      if verbose:
        print('[INFO ] already connected')
      if failIfOpen:
        return False
      if verbose:
        print('[DONE ]')
      return True
    if portName is None:
      if verbose:
        print('[INFO ] port name not specified, looking for ports')
      usbPorts = guessUsbPort()
      if verbose:
        print('[INFO ] found ports', usbPorts)
      if len(usbPorts) == 0:
        if verbose:
          print('[ERROR] no ports found')
          print('[FIX  ] please plug in the Arduino board')
          print('[ABORT]')
        return False
      portName = usbPorts[index]
    if verbose:
      print('[OK   ] using port', portName)

    port          = serial.Serial()
    port.port     = portName;
    port.baudrate = baudRate
    port.parity   = 'N'
    port.bytesize = 8
    port.stopbits = 1
    port.timeout  = 1
    port.open()

    self.portType = 'USB';
    self.channel  = port

    # determine the firmware type
    if verbose:
      print('[INFO ] attempting to establish communication with Arduino firmware')
      print('[INFO ] attempt', end='')
    nl = False
    for n in range(1, 11):
      if verbose:
        print('', n, end='', flush=True)
      time.sleep(0.2)
      port.write(b'99\n')
      time.sleep(0.2)
      if port.inWaiting() > 0:
        if verbose:
          print()
          nl = True
          print('[OK   ] received response from Arduino')
        resp = port.readline()
        if verbose:
          print('[INFO ] response =', resp,)
        if resp == Arduino._EXPECTED_FIRMWARE_RESPONSE:
          if verbose:
            print('[OK   ] response from Arduino is correct')
            print('[DONE ]')
          self._emptyInputBuffer()
          return True
        else:
          break
    if verbose:
      print('[ERROR] response from Arduino is incorrect')
    self.channel.close()
    self.channel = None
    if verbose:
      if not nl:
        print()
      print('[ERROR] found Arduino but unable to communicate with firmware')
      print('[FIX  ] please upload the correct firmware')
      print('[ABORT]')
    return False

  def usbDisconnect(self):
    if isinstance(self.channel, serial.Serial):
      time.sleep(0.5)
      self.channel.close()
    self.channel = None

  def portName(self):
    if self.channel is not None:
      return self.channel.port
    return None

  # aliases for legacy programs
  serialConnect = usbConnect
  serialDisconnect = usbDisconnect
  guessSerialPort = guessUsbPort

  # Analog I/O -------------------------------------------------------

  def analogRead(self, pinNumber):
    outStr = 'ar' + str(pinNumber) + '\n'
    self._writeStr(outStr)
    line = self.channel.readline()
    value = int(line)
    return value

  def analogWrite(self, pinNumber, value):
    outStr = 'aw' + str(pinNumber) + ',' + str(int(value)) + '\n'
    self._writeStr(outStr)

  # aliases
  ar = analogRead
  aw = analogWrite

  # Digital I/O ------------------------------------------------------

  def digitalRead(self, pinNumber):
    outStr = 'dr' + str(pinNumber) + '\n'
    self._writeStr(outStr)
    line = self.channel.readline()
    value = int(line)
    return value

  def digitalWrite(self, pinNumber, value):
    if value:
      value = 1
    else:
      value = 0
    outStr = 'dw' + str(pinNumber) + ',' + str(value) + '\n'
    self._writeStr(outStr)

  def pinMode(self, pinNumber, mode):
    mode = str(mode)
    firstChar = mode[0].lower()
    if firstChar == 'i' or firstChar == '0':
      mode = '0'
    elif firstChar == 'o' or firstChar == '1':
      mode = '1'
    elif firstChar == 'u' or firstChar == '2':
      mode = '2'
    outStr = 'dm' + str(pinNumber) + ',' + mode + '\n'
    self._writeStr(outStr)

  # aliases
  dr = digitalRead
  dw = digitalWrite
  pm = pinMode

  # LCD --------------------------------------------------------------

  def lcdConnect(self):
    outStr = 'la\n'
    self._writeStr(outStr)
    time.sleep(0.3)
  
  def lcdBacklight(self, value):
    if value:
      value = 1
    else:
      value = 0
    outStr = 'lb' + str(value) + '\n'
    self._writeStr(outStr)

  def lcdClear(self):
    outStr = 'lc\n'
    self._writeStr(outStr)

  def lcdCursorBlink(self, onOrOff):
    if onOrOff:
      value = '1'
    else:
      value = '0'
    outStr = 'lk{}\n'.format(value)
    self._writeStr(outStr)

  def lcdCursorShow(self, onOrOff):
    if onOrOff:
      value = 1
    else:
      value = 0
    outStr = 'ls{}\n'.format(value)
    self._writeStr(outStr)

  def lcdPrint(self, message):
    message = str(message)
    outStr = 'lp{}\n'.format(message)
    self._writeStr(outStr)

  # Private support methods ------------------------------------------

  def _emptyInputBuffer(self):
    time.sleep(0.25)
    numPendingChars = self.channel.inWaiting()
    if numPendingChars > 0:
      self.channel.read(numPendingChars)

  def _writeStr(self, outStr):
    if not self.channel:
      raise Exception('Arduino not connected')
    self.channel.write(str.encode(outStr))
