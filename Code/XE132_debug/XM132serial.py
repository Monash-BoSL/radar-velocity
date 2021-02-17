import serial
import time
from matplotlib import pyplot as plt
import numpy as np
from scipy import ndimage

ser = serial.Serial('COM41', 115200, timeout = 1)

bins = []

SWEEPS_PER_FRAME = 64
START_DISTANCE = 240
LENGTH = 560
SWEEP_RATE = 1000000
DISTANCES = [x for x in range(START_DISTANCE, START_DISTANCE+LENGTH, 60)]

def print_bts(bts):
    for n in bts:
        print("{0:0{1}X}".format(n,2), end = " ")
    print()


def read_reg(reg):
    cmdls = [0xCC, 0x01, 0x00, 0xF8, reg, 0xCD]
    cmd = bytearray(cmdls)
    send(cmd)

def write_reg(reg, val):
    hexes = []
    
    for i in range(4):
        hexes.append(val - ((val>>8)<<8))
        val = val>>8

    cmdls = [0xCC, 0x05, 0x00, 0xF9, reg] + hexes + [0xCD]
    cmd = bytearray(cmdls)
    send(cmd)

def read_buf():
    cmdls = [0xCC, 0x03, 0x00, 0xFA, 0xE8, 0x00, 0x00, 0xCD]
    cmd = bytearray(cmdls)
    rep = send(cmd)
    lenn = int.from_bytes(rep[1:3], "little")
    print("Length: ", lenn)
    frame = rep[5:lenn+4]

    fm = split_frame(frame)
     
    return fm

def split_frame(frame):
    len_frm = len(frame)
    
    frame_arr = np.empty((len(DISTANCES),SWEEPS_PER_FRAME))
    
    for i in range(0,len_frm,2):
        v = int.from_bytes(frame[i:i+2], "little")
        entry = i//2
        y = entry//len(DISTANCES)
        x = entry - y*len(DISTANCES)
        frame_arr[x,y] = v
    return frame_arr
    

def send(btyarr):
    print("Sending: ", end='')
    print_bts(btyarr)
    ser.write(btyarr)
    
    end = -1
    i = 0
    lenn = bytearray()
    rep = bytearray()
    
    
    while i != end:
        bt = ser.read()
        if i in range(1,3):
            lenn += bt
        if i == 2:
           end = int.from_bytes(lenn, "little") + 5
        ibt = int.from_bytes(bt, "little")

        rep.append(ibt)
       
        i += 1
        
    print("Reply:   ", end="")
    print_bts(rep)
    return rep

##close sensor
write_reg(0x03,0x00)
#clear status
write_reg(0x03,0x04)
read_reg(0x06)
##sparse mode
write_reg(0x02,0x04)
##close sensor
write_reg(0x03,0x00)
read_reg(0x06)
##sparse mode
write_reg(0x02,0x04)

##set update rate to 1 000 000 mHz
write_reg(0x23,0)
##set power mode to active
write_reg(0x25,0x03)
##set profile to 3
write_reg(0x28,0x03)
##set sparse sampling mode to A
write_reg(0x42,0x00)
##set repition mode to be limited to update rate
write_reg(0x22,0x02)
##set sweep rate to 1 000 000 mHz
write_reg(0x41, SWEEP_RATE)


##set start distace to 240 mm
write_reg(0x20,START_DISTANCE)
##set length to 560 mm
write_reg(0x21,LENGTH)

##set gain to 500
write_reg(0x24,500)
##set downsampling factor to 1
write_reg(0x29,1)
##set HW_ACC_AVE_SAMPLES to 32
write_reg(0x30,32)
##set sweeps per frame to 64
write_reg(0x40, SWEEPS_PER_FRAME)
##disable attenuation
write_reg(0x32, 0x00)
##enable TX
write_reg(0x26, 0x00)


##enable asynchonus measurement
write_reg(0x33, 0x01)
##set power mode to active
write_reg(0x25, 0x03)
##enable uart streaming
write_reg(0x05, 0x00)
##create service
write_reg(0x03, 0x01)


##check state
read_reg(0x06)

print("reading config")
#read start
read_reg(0x81)
#read len
read_reg(0x82)
#read data length
read_reg(0x83)
#read sweep rate
read_reg(0x84)
#read step length
read_reg(0x85)

##active service
write_reg(0x03,0x02)

time.sleep(1)

##check state
read_reg(0x06)

##read buffer
frame = read_buf()

for i,row in enumerate(frame):
    summ = 0
    for j,v in enumerate(row):
        summ += v
    ave = summ/len(row)
    for j,v in enumerate(row):
        frame[i,j] -= ave

fft_frame = np.fft.rfft(frame,axis=1)

for i,row in enumerate(fft_frame):
    for j,v in enumerate(row):
        fft_frame[i,j] = abs(v)

fft_frame = fft_frame.real.astype('float', copy = False)

maxx = np.amax(fft_frame)

for i,row in enumerate(fft_frame):
             for j,ent in enumerate(row):
                 if ent < 0.5*maxx:
                    fft_frame[i,j] = 0

#find average speed
half_wavelength = 2.445e-3

d,s = ndimage.measurements.center_of_mass(fft_frame)
d *= 60
d += START_DISTANCE
s *= SWEEP_RATE/(SWEEPS_PER_FRAME*1000) * half_wavelength
print("Speed: " + "{:10.4f}".format(s) + "m/s Depth: " + "{:10.4f}".format(d) + "mm Amp: " + "{:10.4f}".format(maxx))

plt.matshow(fft_frame)
plt.show()



# while 1:
    # end = False
    # rep = bytearray()
    # while not end:
        # bt = ser.read()
        # ibt = int.from_bytes(bt, "little")
        # #print(hex(ibt))
        # rep.append(ibt)
        # if ibt == 0xCD:
            # end = True
    # print_bts(rep)