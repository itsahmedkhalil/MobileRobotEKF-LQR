#! /usr/bin/python3
import sys
try:
    f = open("/dev/encoder-driver",'rb');
except Exception as e:
    print(e,file=sys.stderr)
    print("ENCODER DRIVER NOT FOUND... DID YOU FORGET TO sudo make load IT?",file=sys.stderr)
    exit()
    
def readEncoders():
    bytes = f.read(8);
    leftInt = int.from_bytes(bytes[0:3],"little",signed=True)
    rightInt = int.from_bytes(bytes[4:7],"little",signed=True)
    return [leftInt,rightInt]

[leftEnc, rightEnc] = readEncoders();

print("%d,%d" % (leftEnc, rightEnc));
