import numpy as np
import pdb

def startswith(line,string):
    return line[:len(string)]==string

def parse_array(string):
    # Remove surrounding braces
    string = string[1:-1]
    return np.array([float(x) for x in string.split(', ')])

def read_log(fname):
    '''
    Output for IT-DOES-THE-THING-MITERS-HALLWAY-scan.txt:
    [
      { # THE FIRST SCAN
        seq:2844.0, #float
        secs:478.0 #float
        nsecs: 137661471.0 # float, probably a unix timestamp?
        frame_id: 'laser_frame' # string
        angle_min: 3.1415... # float
        angle_max: -3.1241... # float
        angle_increment: -0.0174532... # float
        time_increment: 3.41...e-07 # float
        scan_time: 0.00012... # float
        range_min: 0.15... # float
        range_max: 6.0 # float
        intensity: <numpy array of floats, length 365>
        range: <numpy array of floats, length 365>
      },
      { # THE SECOND SCAN
        seq:...,
        ...,
        intensity,
        range
      },
      # ALL THE OTHER SCANS
    ]
    '''
            
    D_list = []
    with open(fname) as F:
        text_data = F.readlines()
    
    D = {}
    for line in text_data:
        if startswith(line,'---'):
            D_list.append(D.copy())
        else:
            x = line.index(':')
            line_key = line[:x].strip()
            line_val = line[x+1:].strip()
            if len(line_val)>0:
                if line_key in ['intensities','ranges']:
                    D[line_key] = parse_array(line_val)
                else:
                    try:
                        D[line_key] = float(line_val)
                    except:
                        D[line_key] = line_val
    return D_list
