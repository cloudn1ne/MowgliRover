#!/usr/bin/env python3

from __future__ import print_function

import sys
import rospy
import getopt
import struct
from mowgli.srv import *


def get_cfg(name, quiet):    
    rospy.wait_for_service('mowgli/GetCfg')
    try:        
        func = rospy.ServiceProxy('mowgli/GetCfg', GetCfg)        
        arg = GetCfgRequest()                
        arg.name = name        
        resp = func(arg)
        if resp.type == 0: #  TYPE_INT32 (0)                       
            data = int.from_bytes(resp.data, byteorder='little' , signed=True)      
            print(data)

        if resp.type == 1: #  TYPE_UINT32 (1)                   
            data = int.from_bytes(resp.data, byteorder='little' , signed=False)      
            print(data)

        if resp.type == 2: #  TYPE_FLOAT (2)                             
            data =  struct.unpack("f", resp.data)
            print("%.15f"%data)

        if resp.type == 3: #  TYPE_DOUBLE (3)                             
            data =  struct.unpack("d", resp.data)
            print("%.15f"%data)

        if resp.type == 4: #  TYPE_STRING (4)                             
            data =  resp.data.decode('ascii')
            print(data)

        if resp.type == 5: #  TYPE_BARRAY(5)                             
            data =  resp.data
            print(data)
        
        if quiet == False:
          print(resp)

        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def usage():
    print("%s -n <name>\n"%sys.argv[0])
    
if __name__ == "__main__":
    try:
        opts, args = getopt.getopt(sys.argv[1:], "hqn:", ["help", "name="])
    except getopt.GetoptError as err:
        # print help information and exit:
        print(err) 
        usage()
        sys.exit(2)

    name = None    
    verbose = False
    quiet = False
    for o, a in opts:        
        if o in ("-h", "--help"):
            usage()
            sys.exit()                    
        elif o in ("-n", "--name"):            
            name = a
        elif o in ("-q"):
            quiet = True
        else:
            print(o)
            assert False, "unhandled option"            

    if quiet == False:
       print("getting name %s"%(name))    

    get_cfg(name, quiet)

