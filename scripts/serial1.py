import serial
import time


arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=.01)


def write_read(x):
    arduino.write(bytes(x, 'utf-8'))
    time.sleep(0.05)
    data = arduino.readline()
    if data == b'':
        return None
    return data


while True:
    n = '0.00'
    num = str(f'13{n}')
    value = write_read(num)
    print(value) if value is not None else print()
    # arduino.close()
    # break
