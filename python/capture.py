import serial
import argparse

if __name__ == "__main__":  
    parser = argparse.ArgumentParser(description='Capture from serial')
    parser.add_argument('--port', default="/dev/ttyACM0", type=str, help='port')
    parser.add_argument('--baudrate', type=int,  default=115200, help='baudrate')
    parser.add_argument('--file', type=str,  default="capture.csv", help='output file')
    args = parser.parse_args()  
    ser = serial.Serial(args.port, args.baudrate)  # open serial port
    print("Capturing %s to file %s" % (ser.name, args.file) )         # check which port was really used
    try:
        fp = open(args.file,"w")
        while True:
            line = ser.readline()
            fp.write(line)
            print(line)        
    except (KeyboardInterrupt, SystemExit):
        print("Application interrupted by user")
        pass
    except:
        raise
    ser.close()             # close port
    fp.close()