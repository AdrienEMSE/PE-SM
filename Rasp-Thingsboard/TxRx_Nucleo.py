from time import sleep
from datetime import datetime

import serial

ser = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=0.25)

while True:
    ser.write(1)
    received_data= ser.read_until()
    ser.flush()
    data_to_write = received_data.decode("utf-8")
    data_to_write = data_to_write.replace('\r\n','')
    data_to_write = data_to_write + ',' + str(datetime.now())
    print(data_to_write)
    file = open('indoor_sensors.csv','a')
    file.write(data_to_write +'\n')
    file.close()
    sleep(30)

