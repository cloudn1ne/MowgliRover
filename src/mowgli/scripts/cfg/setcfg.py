#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
import getopt
import struct
from mowgli.srv import *


def set_cfg(type, name, val, quiet):    
    rospy.wait_for_service('mowgli/SetCfg')
    try:        
        func = rospy.ServiceProxy('mowgli/SetCfg', SetCfg)        
        arg = SetCfgRequest()        
        arg.type = type
        arg.name = name
        if arg.type == 0:       #  TYPE_INT32 (0)           
            arg.data = val.to_bytes(4, byteorder='little' , signed=True)      

        if arg.type == 1:       #  TYPE_UINT32 (1)
            arg.data = val.to_bytes(4, byteorder='little' , signed=False)      

        if arg.type == 2:       #  TYPE_FLOAT (2)            
            arg.data = bytearray(struct.pack("f", val))  

        if arg.type == 3:       #  TYPE_DOUBLE (3)            
            arg.data = bytearray(struct.pack("d", val))  

        if arg.type == 4:       #  TYPE_STRING (4)            
            arg.data = bytearray(val, 'ascii')              

        if arg.type == 5:       #  TYPE_BARRAY (5)            
            harray = []
            for hex_byte in val:                   
                harray.append(int(hex_byte, 16))                
            arg.data = bytearray(harray)
        resp = func(arg)
        if quiet == False:
          print(resp)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def usage():
    print("%s [-i | -u | -f | -d | -s | -a ] -v <value> -n <name>\n"%sys.argv[0])
    print(" -i = Integer32")
    print(" -u = Unsigned Integer32")
    print(" -f = Float32")
    print(" -d = Double (Float64)")
    print(" -s = String")
    print(" -a = Array")

if __name__ == "__main__":
    try:
        opts, args = getopt.getopt(sys.argv[1:], "hiufdqav:n:", ["help", "value=", "name="])
    except getopt.GetoptError as err:
        # print help information and exit:
        print(err) 
        usage()
        sys.exit(2)

    name = None
    value = None
    str_value = None    # string representation of value
    type = None
    verbose = False
    quiet = False
    for o, a in opts:
        if o == "-i":
            type = 0   # TYPE_INT32 (0)
        elif o == "-u":
            type = 1   # TYPE_UINT32 (1)          
        elif o == "-f":
            type = 2   # TYPE_FLOAT (2)
        elif o == "-d":
            type = 3   # TYPE_DOUBLE (3)            
        elif o == "-s":
            type = 4   # TYPE_STRING (4)
        elif o == "-a":
            type = 5   # TYPE_BARRAY (5)                   
        elif o == "-q":
            quiet = True
        elif o in ("-h", "--help"):
            usage()
            sys.exit()
        elif o in ("-v", "--value"):
            str_value = a
        elif o in ("-n", "--name"):            
            name = a
        else:
            print(o)
            assert False, "unhandled option"            

    if quiet == False:
       print("setting type %d %s=%s"%(type, name, str_value))    

    if type == 0 or type == 1:        
        value = int(str_value)        
    if type == 2 or type == 3:
        value = float(str_value)        
    if type == 4:
        value = str_value
    if type == 5:
        value = str_value.split(",")

    set_cfg(type, name, value, quiet)

