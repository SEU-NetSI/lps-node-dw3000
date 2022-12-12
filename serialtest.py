import serial.tools.list_ports

def listAllPorts():
  ports = serial.tools.list_ports.comports()
  for port in ports:
    print(port)

if __name__ == "__main__":
  listAllPorts()
  ser = serial.Serial("/dev/ttyACM0", 115200)
  while (True):
    data = ser.readline()
    print(data)
  ser.close()


