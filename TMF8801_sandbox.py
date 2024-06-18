"""
Test of I2C for AMS TMF8801

April 2022

Cienna Becker beck2271@stthomas.edu

Lucas Koerner koer2434@stthomas.edu
"""
import logging
import os, sys
import time
from time import sleep
import numpy as np
import matplotlib 
import matplotlib.pyplot as plt

sys.path.append('C:\\Users\\koer2434\\Documents\\material_classification\\covg_fpga\\python\\')
from pyripherals.core import FPGA

matplotlib.use('TkAgg')
plt.ion()

sys.path.append('C:\\Users\\koer2434\\Documents\\material_classification\\dtof-materials')


hex_dir = '/Users/cienn/OneDrive/Desktop/Summer2022'
hex_dir = 'C:/Users/koer2434/Documents/material_classification/dtof_materials/'
hex_dir = 'C:\\Users\\koer2434\\Documents\\material_classification\\dtof-materials'
classifiers_dir = r"C:\plastics_classifier\saved_classifiers"
hist_dir = r"C:/Users/koer2434/OneDrive - University of St. Thomas/UST/research/tof_comparison/material_classification/data/"

FW_PATCH = True
READ_HIST = True
READ_LIVE = False
DATA_SAVE = True

# bl: boot loader commands, read .hex file and process line by line. 
def bl_intel_hex(hex_dir, filename='main_app_3v3_k2.hex'):
    ''' read intel HEX file line by line 
    Parameters
    ----------
    hex_dir : string
        directory of the hex file
    filename : string
        name of the hex file

    Returns
    -------
    line_list : list 
        list of each line in the file

    '''
    with open(os.path.join(hex_dir, filename), 'r') as f:
        line_list = [l for l in f] 

    return line_list

def bl_checksum(data):
    """ create checksum as described in section 6.3
        of the host driver manual (AN000597)
    
    Parameters
    ----------
    data : list
        bytes that will be sent

    Returns
    -------
    ones_comp : int 
        list of each line in the file    
    """

    low_byte = sum(data) & 0xff  # only use lowest byte of sum
    ones_comp = low_byte ^ 0xff  # ones complement op. via XOR
    return ones_comp

def bl_process_line(l):
    """
    interpret a HEX record line and prepare for i2c write
    
    Parameters
    ----------
    l : string
        a line from the HEX record

    Returns
    -------
    data : list 
        list of bytes to write  
    """

    # https://en.wikipedia.org/wiki/Intel_HEX

    cmd_addr = 0x08
    ram_addr_cmd = 0x43
    data_cmd = 0x41

    # data command
    if l[7:9] == '00':
        data = l[9:]
        data = data.strip()[:-2]
        data_bytes = bytearray.fromhex(data)
        data_len = len(data_bytes)
        data = [cmd_addr, data_cmd, data_len] + list(data_bytes)

    # extended address 
    elif l[7:9] == '04':
        addr = l[9:13]
        addr_bytes = bytearray.fromhex('0' + addr + '0')
        data_len = 0
        data = [cmd_addr, ram_addr_cmd] + list(addr_bytes) 

    else: 
        return None
    
    data.append(bl_checksum(data[1:]))

    return data

bl_hex_lines = bl_intel_hex(hex_dir, filename='main_app_3v3_k2.hex')
print(f'RAM patch is {len(bl_hex_lines)} lines long')

sys.path.insert(0, r'C:\covg_fpga\python')
from boards import TOF

logging.basicConfig(filename='DAC53401_test.log',
                    encoding='utf-8', level=logging.INFO)

# Initialize FPGA
f = FPGA()
f.init_device()

# Instantiate the TMF8801 controller.
tof = TOF(f)

# check ID 
id = tof.TMF.get_id()
print(f'Chip id 0x{id:04x}')
tof.TMF.cpu_reset()
app = tof.TMF.read_app()
major,minor,patch = tof.TMF.rom_fw_version()
print(f'CPU ready? {tof.TMF.cpu_ready()}')

if FW_PATCH:
    # download firmware to bootloader (i.e. RAM patch in section 7 of AN000597)
    tof.TMF.download_init()
    status = tof.TMF.ram_write_status()
    print(f'RAM write status: {status}')

    for l in bl_hex_lines:
        d = bl_process_line(l)
        if d is not None:
            addr = d[0]
            data = d[1:]

            tof.TMF.i2c_write_long(tof.TMF.ADDRESS, 
                [addr],
                (len(data)),
                data)
            # reads back 3 bytes from 'CMD_DATA7'
            status = tof.TMF.ram_write_status()
            # TODO: check that status is 00,00,FF
            # TODO: consider skipping status check to reduce time for upload
            # if not (status == [0,0,0xFF]):
            #     print(f'Bootloader status unexpected value of {status}')

    tof.TMF.ramremap_reset()

    for i in range(3):
        print(f'CPU ready? {tof.TMF.cpu_ready()}')

    major,minor,patch = tof.TMF.rom_fw_version()

# the write function reads back the register so that it is possible to write just bit-fields 

# configure the app 

# tof.TMF.load_app(app='measure')
# app = tof.TMF.read_app()

# factory calibration 
PERFORM_CAL = False
MEASURE = False

# perform factory calibration 
if PERFORM_CAL:
    tof.TMF.write(0x0A, 'COMMAND') # Command is register 0x10
    sleep(2)
    cal_done = 0
    count = 0
    while (cal_done != 0x0A) and (count < 20):
        cal_done = tof.TMF.read('REGISTER_CONTENTS')
        sleep(0.1)
        count = count + 1
        print(f'Cal done 0x{cal_done:02x}')

    cal_data = tof.TMF.read_by_addr(0x20, num_bytes=14) 
    for d in cal_data:
        print(d)

if MEASURE:
    CAL_DATA = True
    STATE_DATA = True

    if (CAL_DATA is False) and (STATE_DATA is True):
        state_addr = 0x20  # if no cal data start at register 0x20 for state data 
    else:
        state_addr = 0x2E # if both are available state starts at 0x2E

    # These registers shall be pre-loaded by the host before command=0x02 or 0x0B is executed
    if CAL_DATA:
        # factory calibration data. TODO: capture the specific calibration for this sensor?
        data = [0x01, 0x17, 0x00, 0xff, 0x04, 0x20, 0x40, 0x80, 0x00, 0x01, 0x02, 0x04, 0x00, 0xFC]
        tof.TMF.i2c_write_long(tof.TMF.ADDRESS, 
            [0x20], # cal_data
            (len(data)),
            data)

    if STATE_DATA:
        # state data 
        data = [0xb1, 0xa9, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        tof.TMF.i2c_write_long(tof.TMF.ADDRESS, 
            [state_addr], # state_data
            (len(data)),
            data)

    ### -------- Configure the sensor for measurements ---------------
    # Continuous mode, period of 100 ms (0x64)
    # S 41 W 08 03 23 00 00 00 64 D8 04 02 P
    #       cmd_data7, cmd_data6, cmd_data5, cmd_data4, cmd_data3, cmd_data2 (repeat period), iterations, interations, start-command
    data = [0x03,      0x23,      0x00,      0x00,      0x00,      0x64,  0xd8, 0x04, 0x02] 

    # data = [0x00, 0x23, 0x00, 0x00, 0x00, 0x64, 0xd8, 0x04, 0x02] # calibraion and state data is not provided -- changes first byte to 0x00 

    tof.TMF.i2c_write_long(tof.TMF.ADDRESS, 
        [0x08], # CMD_DATA7
        (len(data)),
        data)
    app = tof.TMF.read_app()

    # check configuation 
    print(tof.TMF.read_by_addr(0x1c))
    print(tof.TMF.read_by_addr(0x1d))

    # Read measurement data
    # S 41 W 1D Sr 41 R A A A A A A A A A A N P
    vals, data = tof.TMF.read_data()
    print(vals)

    # I2C pipe (for large data transfers) 
    # works although reset behaves strange
    # Note: the last 4 bytes out will remain (stale) if the pipe is 
    #       re-read after the FIFO empties 
    #       the 4 bytes out don't update until 4 I2C bytes are read.
    #       read in 4 byte multiples 

    # buf,e = read_i2c_pipe(tof.TMF)

    # to write a register without a readback use directly (with the named registers a readback happens first):
    # i2c_write_long(self, devAddr, regAddr, data_length, data)

if READ_HIST:
    from histogram_functions import write_hist, process_live, get_distance, avg_hist

    # load trained plastics model from disk
    import pickle
    from sklearn.preprocessing import StandardScaler as sc

    # filename = r"C:\plastics_classifier\saved_classifiers\final_optical_plastics_classifier.sav"
    filename = os.path.join(classifiers_dir, 'final_optical_plastics_classifier.sav')
    model = pickle.load(open(filename, 'rb'))

    # filename = r"C:\plastics_classifier\saved_classifiers\final_optical_scalar.pk1"
    filename = os.path.join(classifiers_dir, 'final_optical_scalar.pk1') # strangely the suffix ends in 1 not l 

    sc = pickle.load(open(filename,'rb'))

    polyester_arr = []

    # stop any command
    tof.TMF.write(0xFF, 'COMMAND')
    # previous command should read 0xFF
    last_cmd = tof.TMF.read('PREVIOUS')
    assert (last_cmd == 0xFF), 'previous command not FF!'

    # clear interrupt
    tof.TMF.write(0x01, 'INT_STATUS')
    tof.TMF.read('INT_STATUS')

    short_range_hist_cfg = [0x80, 0x00, 0x00, 0x00, 0x30] # bit 4 in cmd_data3 = short range hist., other commands detailed in host driver
    tof.TMF.i2c_write_long(tof.TMF.ADDRESS, 
        [0x0C], # CMD_DATA3
        (len(short_range_hist_cfg)),
        short_range_hist_cfg)  

    cyclic_meas = [0x00, 0x23, 0x00, 0x00, 0x00, 0x80, 0xD0, 0x07, 0x02]
    # cyclic_meas = [0x00, 0x23, 0x00, 0x00, 0x00, 0x20, 0xD0, 0x07, 0x02] # try to reduce the exposure time (LJK)

    tof.TMF.i2c_write_long(tof.TMF.ADDRESS, 
        [0x08], # CMD_DATA3 -> CMD_DATA7?? 
        (len(cyclic_meas)),
        cyclic_meas)    

    # loop begins here
    num_readings = 1
    all_data = []
    while (num_readings <= 10):
    # while (1): #(num_readings <= 10):
        start_time = time.time()

        cnt = 0
        bit_1 = 0
        while ((cnt < 10) and (bit_1 == 0)): # checks if bit_1 has changed
            st = tof.TMF.read('INT_STATUS')
            print(f'INT STATUS (waiting for bit1) {hex(st)}')
            bit_1 = st & 0x0002  # check bit 1 (int2 bit)
            sleep(0.1)  # warning that sometimes ipython turns this into a very long sleep
            cnt = cnt + 1
            if cnt==10:
                print('Timeout waiting for INT bit 1')


        # read out COMMAND, PREV_COMMAND ... STATE STATUS REGISTER_CONTENTS -> store TID
        cmd_etc = tof.TMF.read_by_addr(reg_addr=0x10, num_bytes=16) 
        tid = cmd_etc[-1] # last data should be TID (transaction ID)

        # setup histogram readout
        tof.TMF.write(0x80, 'COMMAND') # see command codes in the datasheet 8.9.11

        tid2 = tof.TMF.read('TID')

        # wait for tid2 to be different than tid -- not sure why we need to read this twice 
        # (but App note says so)
        tid3_etc = tof.TMF.read_by_addr(0x1C, num_bytes=4)     
        tid3 = tid3_etc[-1]

        hist_data_pipe = {}
        pipe_final = {}
        for tdc in range(5): # cycle through the 5 TDCs 
            hist_data_pipe[tdc] = np.array([], dtype=np.uint16)
            for quarter in range(4):
                # reset the FIFO, trigger data collection, then read from the filled pipe
                buf,e = tof.TMF.i2c_read_long(tof.TMF.ADDRESS, [0x20], data_length=128, data_transfer='pipe')
                hist_data_q_pipe = np.asarray(buf)
                hist_data_pipe[tdc] = np.append(hist_data_pipe[tdc], hist_data_q_pipe)

            i = 0
            temp_arr = []
            pipe_final[tdc] = np.array([], dtype=np.uint16)
            # LSB + MSB -> data gets reordered upon pipe readout, so this stores them correctly
            while(i < 509):
                temp_arr.append( (hist_data_pipe[tdc][i+3]) + (hist_data_pipe[tdc][i+2]<<8) )
                temp_arr.append( (hist_data_pipe[tdc][i+1]) + (hist_data_pipe[tdc][i+0]<<8) )
                i = i + 4
            pipe_final[tdc] = np.append(pipe_final[tdc], temp_arr)

        #clear diagnostic interrupt (bit int2)
        tof.TMF.write(0x02, 'INT_STATUS')

        # continue normal operation
        tof.TMF.write(0x32, 'COMMAND') 

        # wait for bit int1 (INT_STATUS) to be set 
        cnt = 0
        bit_0 = 0
        while ((cnt < 10) and (bit_0 == 0)): # checks if bit_0 has changed
            st = tof.TMF.read('INT_STATUS')
            print(f'INT STATUS (waiting for bit0) {hex(st)}')
            bit_0 = st & 0x0001  # check bit 0 (int1 bit)
            sleep(0.1)  # warning that sometimes ipython turns this into a very long sleep
            cnt = cnt + 1
            if cnt==10:
                print('Timeout waiting for INT bit 1')

        # clear INT_STATUS (bit int1)
        tof.TMF.write(0x01, 'INT_STATUS')

        # read out STATE STATUS REGISTER_CONTENTS TID RESULT_NUMBER
        st_field_etc = tof.TMF.read_by_addr(0x1C, num_bytes = 12)

        if READ_LIVE:
            if ( get_distance(pipe_final) >= 2 and get_distance(pipe_final) <= 26 ):
                # 2 cm buffer on either side of range

                what_material = []
                what_material.append(process_live(pipe_final, type='opt_params', data='regular'))

                if (what_material[0][0] != 'error'): # in case of runtime or optimization error when sorting using optical parameters
                    polyester_arr.append(what_material)
                    what_material = sc.transform(what_material)
                    for i in range(len(what_material)):
                        # print prediction of my survival using logistic regression classifier
                        pred = model[6].predict(what_material)

                        if (pred[i] == 0):
                            print('HDPE')
                        elif (pred[i] == 1):
                            print('LDPE')
                        elif (pred[i] == 2):
                            print('Polyester')
                        elif (pred[i] == 3):
                            print('Polypropylene')
                        elif (pred[i] == 4):
                            print('Polystyrene')
            else:
                print('no object detected')
                    
        if DATA_SAVE:
            # save histogram data to a csv file and increment num_readings
            all_data.append(pipe_final)
            print("Number " + str(num_readings) + ": " + str(get_distance(pipe_final)))
            print("Maximum value " + str(num_readings) + ": " + str(np.max(pipe_final[1]))) # max value of TDC 1 

        num_readings = num_readings + 1

    if DATA_SAVE:
        for i in range(num_readings-1):
            write_hist(all_data[i], 'test_ljk', hist_dir=hist_dir)  


def capture_histograms(num_meas, filename, period = 0x80, iterations_1k=2000, hist_dir=hist_dir):

    # stop any command
    tof.TMF.write(0xFF, 'COMMAND')
    # previous command should read 0xFF
    last_cmd = tof.TMF.read('PREVIOUS')
    assert (last_cmd == 0xFF), 'previous command not FF!'

    # clear interrupt
    tof.TMF.write(0x01, 'INT_STATUS')
    st = tof.TMF.read('INT_STATUS')
    print(f'INT_STATUS = {hex(st)}')

    short_range_hist_cfg = [0x80, 0x00, 0x00, 0x00, 0x30] # bit 4 in cmd_data3 = short range hist., other commands detailed in host driver
    tof.TMF.i2c_write_long(tof.TMF.ADDRESS, 
        [0x0C], # CMD_DATA3
        (len(short_range_hist_cfg)),
        short_range_hist_cfg)  

    iterations_low = int(iterations_1k) & 0x00ff
    iterations_high = (int(iterations_1k) & 0xff00)>>8
    cyclic_meas = [0x00, 0x23, 0x00, 0x00, 0x00, period, iterations_low, iterations_high, 0x02] # iterations was D0 07 = 2 Million
    tof.TMF.i2c_write_long(tof.TMF.ADDRESS, 
        [0x08], # CMD_DATA3 -> CMD_DATA7?? 
        (len(cyclic_meas)),
        cyclic_meas)    

    # loop begins here
    num_readings = 1
    all_data = []
    while (num_readings <= num_meas):
        start_time = time.time()

        cnt = 0
        bit_1 = 0
        cnt_limit = 100
        while ((cnt < cnt_limit) and (bit_1 == 0)): # checks if bit_1 has changed
            st = tof.TMF.read('INT_STATUS')
            bit_1 = st & 0x0002  # check bit 1 (int2 bit)
            sleep(0.02)  # warning that sometimes ipython turns this into a very long sleep
            cnt = cnt + 1
            if cnt==cnt_limit:
                print('Timeout waiting for INT')

        # read out COMMAND, PREV_COMMAND ... STATE STATUS REGISTER_CONTENTS -> store TID
        cmd_etc = tof.TMF.read_by_addr(reg_addr=0x10, num_bytes=16) 
        tid = cmd_etc[-1] # last data should be TID (transaction ID)

        # setup histogram readout
        tof.TMF.write(0x80, 'COMMAND') # see command codes in the datasheet 8.9.11

        tid2 = tof.TMF.read('TID')

        # wait for tid2 to be different than tid -- not sure why we need to read this twice 
        # (but App note says so)
        tid3_etc = tof.TMF.read_by_addr(0x1C, num_bytes=4)     
        tid3 = tid3_etc[-1]

        hist_data_pipe = {}
        pipe_final = {}
        for tdc in range(5): # cycle through the 5 TDCs 
            hist_data_pipe[tdc] = np.array([], dtype=np.uint16)
            for quarter in range(4):
                # reset the FIFO, trigger data collection, then read from the filled pipe
                buf,e = tof.TMF.i2c_read_long(tof.TMF.ADDRESS, [0x20], data_length=128, data_transfer='pipe')
                hist_data_q_pipe = np.asarray(buf)
                hist_data_pipe[tdc] = np.append(hist_data_pipe[tdc], hist_data_q_pipe)

            i = 0
            temp_arr = []
            pipe_final[tdc] = np.array([], dtype=np.uint16)
            # LSB + MSB -> data gets reordered upon pipe readout, so this stores them correctly
            while(i < 509):
                temp_arr.append( (hist_data_pipe[tdc][i+3]) + (hist_data_pipe[tdc][i+2]<<8) )
                temp_arr.append( (hist_data_pipe[tdc][i+1]) + (hist_data_pipe[tdc][i+0]<<8) )
                i = i + 4
            pipe_final[tdc] = np.append(pipe_final[tdc], temp_arr)

        #clear diagnostic interrupt (bit int2)
        tof.TMF.write(0x02, 'INT_STATUS')

        # continue normal operation
        tof.TMF.write(0x32, 'COMMAND') 

        # wait for bit int1 (INT_STATUS) to be set 
        cnt = 0
        bit_0 = 0
        while ((cnt < 200) and (bit_0 == 0)): # checks if bit_0 has changed
            st = tof.TMF.read('INT_STATUS')
            bit_0 = st & 0x0001  # check bit 0 (int1 bit)
            sleep(0.02)  # warning that sometimes ipython turns this into a very long sleep
            cnt = cnt + 1

        # clear INT_STATUS (bit int1)
        tof.TMF.write(0x01, 'INT_STATUS')

        # read out STATE STATUS REGISTER_CONTENTS TID RESULT_NUMBER
        st_field_etc = tof.TMF.read_by_addr(0x1C, num_bytes = 12)

        # save histogram data to a csv file and increment num_readings
        all_data.append(pipe_final)
        print("Number " + str(num_readings) + ": " + str(get_distance(pipe_final)))
        print("Maximum value " + str(num_readings) + ": " + str(np.max(pipe_final[1]))) # max value of TDC 1 
        print("Pileup check " + str(num_readings) + ": " + str(np.sum(pipe_final[1])/(iterations_1k*1000))) # max value of TDC 1 

        num_readings = num_readings + 1

    for i in range(num_readings-1):
        write_hist(all_data[i],  filename, hist_dir=hist_dir)  
