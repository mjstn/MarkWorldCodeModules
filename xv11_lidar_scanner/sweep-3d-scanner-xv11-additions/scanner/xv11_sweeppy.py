"""XV11 Lidar implementation of sweeppy library methods/classes"""
"""This class supplied by mark@mark-world.com visit the site for details on this project"""
import collections
import serial
import math
import time
import sweep_helpers


class Scan(collections.namedtuple('Scan', 'samples')):
    """A collection of sensor readings"""
    pass


class Sample(collections.namedtuple('Sample', 'angle distance signal_strength')):
    """A single sensor reading, comprised of an angle, distance and signal strength"""
    pass

x11_samples = []
sampleCount = 0

# This checksum code from https://github.com/Xevel/NXV11/wiki and was stated as 'not quite right'
def xv11_checksum(data):
        # group the data by word, little endian
        data_list = []
        for t in range(10):
            data_list.append( data[2*t] + (data[2*t+1] << 8) )
        # compute the checksum.
        chk32 = 0
        for data in data_list:
            chk32 = (chk32 << 1) + data

        # return a value wrapped around on 15bits, and truncated to still fit into 15 bits
        checksum = (chk32 & 0x7FFF) + ( chk32 >> 15 ) # wrap around to fit into 15 bits
        checksum = checksum & 0x7FFF # truncate to 15 bits
        return int( checksum )

# Decode a single line of output from an XV11 Lidar
# There are 90 frames each with 4 samples for 360 data points total
# each line:  <start byte> <index> <speed> <Data 0> <Data 1> <Data 2> <Data 3> <checksum>
# returns  speed, angles, distances in mm, qualities
def x11_decode_string(string):
    speed     = 0
    pktcksum  = 0
    calccksum = 0
    angles    = [ 999, 999, 999, 999 ]
    distances = [ 0, 0, 0, 0 ]
    qualities = [ 0, 0, 0, 0 ]
    strengths = [ 0, 0, 0, 0 ]

    # If we don't have a full set of 4 data points ignore this line
    if len(string) < 66:
        angles    = [ 990, 990, 990, 990 ]
        return speed,angles,distances,qualities,strengths
        
    # show raw string of data
    data = []

    for byte in string.strip("\n").split(":")[:22]:
        data.append(int(byte,16))

    start = data[0]
    idx = data[1] - 0xa0
    speed = float(data[2] | (data[3] << 8)) / 64.0
    in_checksum = data[-2] + (data[-1] << 8)

    # bytes 20 and 21 are a checksum.
    # IMPROVEMENT: We should calculate to verify checksum and if bad return all 0 distances
    pktcksum = data[20] | (data[21] << 8)
    calccksum = xv11_checksum(data)
    if (pktcksum != calccksum):
        angles    = [ 991, 991, 991, 991 ]
        return speed,angles,distances,qualities,strengths

    # There are 4 bytes per sample with this meaning
    # byte 0:  distance
    # byte 1 : <"invalid data" flag> <"strength warning" flag> <distance 13:8>
    # byte 2 : <signal strength  7:0>
    # byte 3 : <signal strength 15:8>

    # first data point 
    angles[0]    = idx*4 + 0
    distances[0] = data[4] | ((data[5] & 0x1f) << 8)
    qualities[0] = data[5] & 0xc0
    strengths[0] = data[6] | (data[7] << 8)

    # second data point 
    angles[1]    = idx*4 + 1
    distances[1] = data[8] | ((data[9] & 0x1f) << 8)
    qualities[1] = data[9] & 0xc0
    strengths[1] = data[10] | (data[11] << 8)

    # third data point 
    angles[2]    = idx*4 + 2
    distances[2] = data[12] | ((data[13] & 0x1f) << 8)
    qualities[2] = data[13] & 0xc0
    strengths[2] = data[14] | (data[15] << 8)

    # forth data point 
    angles[3]    = idx*4 + 3
    distances[3] = data[16] | ((data[17] & 0x1f) << 8)
    qualities[3] = data[17] & 0xc0
    strengths[3] = data[18] | (data[19] << 8)


    return speed,angles,distances,qualities,strengths


class Sweep(object):
    """ XV11 Lidar implementation of Sweep """

    def __init__(self, port, bitrate=None):
        self.motor_speed = sweep_helpers.MOTOR_SPEED_5_HZ
        self.sample_rate = sweep_helpers.SAMPLE_RATE_500_HZ
        self.serial_dev  = serial.Serial(port,
                            baudrate=115200,
                            parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE,
                            bytesize=serial.EIGHTBITS,
                            timeout=0)

        # Initialize the XV11 Lidar assuming use of the getsurreal controller
        # Refer to https://github.com/getSurreal/XV_Lidar_Controller/blob/master/XV_Lidar_Controller.ino
        # We default speed and so on but to change that, this is the place to do changes
        self.serial_dev.write("MotorOn\n")

    def __enter__(self):
        return self

    def __exit__(self, *args):
        pass

    def start_scanning(self):
        """ Docstring """
        time.sleep(0.5)

    def stop_scanning(self):
        """ Docstring """
        self.serial_dev.write("MotorOff\n")
        time.sleep(0.5)

    def get_motor_ready(self):
        """ Docstring """
        time.sleep(0.1)
        return True

    def get_motor_speed(self):
        """ Docstring """
        time.sleep(0.1)
        return self.motor_speed

    def set_motor_speed(self, speed):
        """ Docstring """
        time.sleep(0.1)
        self.motor_speed = speed

    def get_sample_rate(self):
        """ Docstring """
        time.sleep(0.1)
        return self.sample_rate

    def set_sample_rate(self, speed):
        """ Docstring """
        self.sample_rate = speed
        time.sleep(0.1)

    def get_scans(self):
        global x11_samples
        global sampleCount

        time.sleep(0.1)
        printAllDistances = True;     # Best to set to False when code is debugged!

        sampleCount = 0
        """ Coroutine-based generator lazily returning the current XV11 scan ad infinitum"""
        print("Get XV11 scans")
        started = False
        string = ""
        foundZero = False
        byte = self.serial_dev.read(1)
        while True:
            if byte != '':
                enc = (byte.encode('hex') + ":")
                if enc == "fa:":
                    if started:
                        try:
                            # print(string)
                            speed, angles_deg, distances_mm, qualities, strengths = x11_decode_string(string)
                            if (foundZero == False):
                                # find the first report of sample 0
                                if (angles_deg[0] == 0):
                                    foundZero = True
                                    print "Found starting record"
                            # print("StringDecoded")

                            if (foundZero == True):
                                # debug line but for final code do not print this 
                                if (printAllDistances == True):
                                    print(angles_deg[0],distances_mm[0],distances_mm[1],distances_mm[2],distances_mm[3])

                                # end the scan on second time we find angle of 0
                                if ((sampleCount > 0) and (angles_deg[0] == 0)):
                                    print ("Found end of scan with sample count ", sampleCount)
                                    if (sampleCount < 85):
                                        # just not enough samples, try again (NOTE: We don't have max retries yet)
                                        print ("Not enough samples! give this scan another try")
                                        sampleCount = 0
                                        x11_samples[:] = []
                                        foundZero = false;
                                    else:
                                        print("x11 sample count ", len(x11_samples))

                                        # Return entire 2D scan of data back to caller
                                        # IMPROVEMENT: Try up to 3 revolutions or till all points we expect good scan in the 2D scan are valid
                                        #              If data points bad for 164-208 degrees it's ok, that points at scanner base
                                        yield Scan(samples=x11_samples)

                                        # reset for next time we do a full 2D scan
                                        sampleCount = 0
                                        x11_samples[:] = []

                                # output this batch of 4 sample to caller.  Angle 999 is bad data line
                                # The angular units are milli degrees
                                if (angles_deg[0] <= 360):
                                    x11_samples.append (Sample(angle=angles_deg[0]*1000, distance=distances_mm[0], signal_strength=strengths[0]))
                                    x11_samples.append (Sample(angle=angles_deg[1]*1000, distance=distances_mm[1], signal_strength=strengths[1]))
                                    x11_samples.append (Sample(angle=angles_deg[2]*1000, distance=distances_mm[2], signal_strength=strengths[2]))
                                    x11_samples.append (Sample(angle=angles_deg[3]*1000, distance=distances_mm[3], signal_strength=strengths[3]))
                                    sampleCount = sampleCount + 1

                        except Exception, e:
                            print e

                    started = True
                    string = "fa:"
                elif started:
                    string += enc
                else:
                    print "Waiting for start"
            byte = self.serial_dev.read(1)


    def reset(self):
        """ Docstring """
        time.sleep(2.0)
