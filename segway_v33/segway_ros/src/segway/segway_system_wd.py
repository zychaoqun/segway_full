"""--------------------------------------------------------------------
COPYRIGHT 2014 Stanley Innovation Inc.

Software License Agreement:

The software supplied herewith by Stanley Innovation Inc. (the "Company") 
for its licensed Segway RMP Robotic Platforms is intended and supplied to you, 
the Company's customer, for use solely and exclusively with Stanley Innovation 
products. The software is owned by the Company and/or its supplier, and is 
protected under applicable copyright laws.  All rights are reserved. Any use in 
violation of the foregoing restrictions may subject the user to criminal 
sanctions under applicable laws, as well as to civil liability for the 
breach of the terms and conditions of this license. The Company may 
immediately terminate this Agreement upon your use of the software with 
any products that are not Stanley Innovation products.

The software was written using Python programming language.  Your use 
of the software is therefore subject to the terms and conditions of the 
OSI- approved open source license viewable at http://www.python.org/.  
You are solely responsible for ensuring your compliance with the Python 
open source license.

You shall indemnify, defend and hold the Company harmless from any claims, 
demands, liabilities or expenses, including reasonable attorneys fees, incurred 
by the Company as a result of any claim or proceeding against the Company 
arising out of or based upon: 

(i) The combination, operation or use of the software by you with any hardware, 
    products, programs or data not supplied or approved in writing by the Company, 
    if such claim or proceeding would have been avoided but for such combination, 
    operation or use.
 
(ii) The modification of the software by or on behalf of you 

(iii) Your use of the software.

 THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 
 \file   segway_system_wd.py

 \brief  This is the segway system watchdod which monitors signals
         from the embedded power system to safely shutdown the PC
         upon embedded powerdown

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
import socket
import sys
import rospy
import os
from utils import m32

class SegwayWatchdog:
    def __init__(self):
        """
        Initialize the UDP connection
        """
        self._continue = True     
        self.conn = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.conn.setblocking(False)
        self.conn.bind(('',6234))

    def Receive(self):
        """
        Try receiving the data up to a maximum size. If it fails
        empty the data
        """
        try:
            data = self.conn.recv(4)
        except:
            data = []
            
        if (len(data) == 4):
        
            rx_dat = [ord(i) for i in data]
            shutdwn_cmd = m32(rx_dat)
            if (0x8756BAEB == shutdwn_cmd):
                rospy.logerr("Platform signaled shutdown, need to shutdown the onboard PC")
                self.Close()
                os.system("shutdown now -h")
                sys.exit(0)

    def Close(self):
        self.conn.close()       
        
    
    
    
    
    
