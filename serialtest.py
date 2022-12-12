import serial.tools.list_ports
import struct

def listAllPorts():
  ports = serial.tools.list_ports.comports()
  for port in ports:
    print(port)

if __name__ == "__main__":
  listAllPorts()
  ser = serial.Serial("/dev/ttyACM0", 115200)
  while (True):
    if (ser.read() == b'\xbc'):
      data = ser.read(4)
      print(struct.unpack('<HH', data))
      print("----")
  ser.close()


