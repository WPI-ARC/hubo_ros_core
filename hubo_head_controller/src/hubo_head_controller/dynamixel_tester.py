import serial

def compute_checksum(command):
    temp = 0
    for i in range(2,len(command) - 1):
        temp += i
    if (temp > 255):
        temp = temp % 255
    temp = ~temp
    return (temp % 255)

if __name__ == '__main__':
    dynamixel_port = serial.Serial('/dev/ttyUSB0', 1000000, 8, 'N', 1, timeout=0.1)
    ping_command = bytearray(6)
    ping_command[0] = 0xff
    ping_command[1] = 0xff
    ping_command[3] = 0x02
    ping_command[4] = 0x06
    for i in range(253):
        print "Checking ID: " + str(i)
        ping_command[2] = i
        ping_command[5] = compute_checksum(ping_command)
        dynamixel_port.write(ping_command)
        reply = dynamixel_port.read(100)
        print reply
    ping_command = bytearray(6)
    ping_command[0] = 0xff
    ping_command[1] = 0xff
    ping_command[3] = 0x02
    ping_command[4] = 0x01
    for i in range(253):
        print "Checking ID: " + str(i)
        ping_command[2] = i
        ping_command[5] = compute_checksum(ping_command)
        dynamixel_port.write(ping_command)
        reply = dynamixel_port.read(100)
        print reply
