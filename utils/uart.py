import serial

ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)

while True:
    if ser.in_waiting > 0:
        data = ser.readline().decode('utf-8', errors='ignore').strip()
        print("recieve:", data)

