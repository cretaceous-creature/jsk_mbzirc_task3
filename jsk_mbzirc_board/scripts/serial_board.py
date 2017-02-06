#!/usr/bin/env python

import rospy
import serial
from jsk_mbzirc_board.srv import *
from std_msgs.msg import Int16

class Serial_board:
    def __init__(self):
        self.T_HEADER = 'TH'
        self.T_TAIL = 'TT'
        self.Magdata = 'mag:!'
        self.port = '/dev/ttyUSB1'
        self.baud = 115200
        self.verbose = True
        self.serialtimeout = 0.01
        self.updaterate = 20    #20hz
        #initialize the parameters
        self.port = rospy.get_param("~port", self.port)
        self.baud = rospy.get_param('~baud', self.baud)
        self.verbose = rospy.get_param('~verbose', self.verbose)
        self.serialtimeout = rospy.get_param('~serialtimeout', self.serialtimeout)
        self.updaterate = rospy.get_param('~updaterate', self.updaterate)
        rospy.loginfo("port is : %s" % self.port)
        rospy.loginfo("baudrate is: %s " % self.baud )
        self.ser = serial.Serial(self.port, self.baud, timeout=self.serialtimeout)
        #if not (self.ser._isOpen()):
         #   rospy.ROSException("Cant open port %s" % self.baud)
        #magnets service server
        self._srv_magnet = rospy.Service("/serial_board/magnet_control", Magnet, self.__Magnet_Service)
        #magnets switch publisher
        self._pub_magnet = rospy.Publisher("/serial_board/magnet_feedback", Int16, queue_size = 1)

    #delete function
    def __del__(self):
        self.ser.close()

    #magnet call
    def __Magnet_Service(self, req):
        time = req.time_ms
        if (time > 65534 and time < 0):
            return False

        t_size = 3;
        time_high8b = chr(time/256)
        time_low8b = chr(time%256)
        onoff = chr(req.on)
        #send on order and seconds,    low byte first
        datatosend = self.T_HEADER + chr(t_size) + chr(req.on) + time_low8b + time_high8b + self.T_TAIL
        #datatosend = "mag:!5!"
        self.ser.write(datatosend)
        return True

    def Serial_Update(self):
        #always read the data
        r = rospy.Rate(self.updaterate)
        while not rospy.is_shutdown():

            data_in = self.ser.readall()
            if data_in:
                if self.verbose:
                    rospy.loginfo(data_in)
                #data will be like mag:!5!
                index = data_in.find(self.Magdata)
                if not (index < 0):
                    index +=5
                    self._pub_magnet.publish(int(data_in[index:data_in.find('!',index)]))
            r.sleep()


if __name__ == "__main__":
    rospy.init_node('serial_board')
    ex = Serial_board()
    try:
        ex.Serial_Update()
    except rospy.ROSInterruptException: pass
