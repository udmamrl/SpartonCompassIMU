#!/usr/bin/env python
# Software License Agreement (BSD License)

# Sparton Digital Compass ROS Driver for AHRS-8/GEDC-6
# Copyright (c) 2013, Cheng-Lung Lee, University of Detroit Mercy.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

# Sparton Digital Compass ROS Driver for AHRS-8/GEDC-6
# Copyright (c) 2013, Cheng-Lung Lee, University of Detroit Mercy.

# Changelog

# 2013.01.06 Add IMU message
# 2012.12.13 Use Pos2D message, normalized to 0 ~ 2*PI
#


import roslib; roslib.load_manifest('SpartonCompassIMU')
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D

import serial, string, math, time, calendar

#import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion


def wrapTo2PI(theta):
    '''Normalize an angle in radians to [0, 2*pi]
    '''
    return theta % (2.*math.pi)

def wrapToPI(theta):
    '''Normalize an angle in radians to [-pi, pi]
    '''
    return (wrapTo2PI(theta+math.pi) - math.pi)

def Spartonshutdownhook():
    global D_Compass
    global myStr1
    print "Sparton shutdown time!"
    D_Compass.write(myStr1) # stop data stream before close port
    D_Compass.flush() # flush data out
    rospy.loginfo('Closing Digital Compass Serial port')
    D_Compass.close() #Close D_Compass serial port
    # rospy.on_shutdown(Spartonshutdownhook)
if __name__ == '__main__':
    global D_Compass
    global myStr1
    rospy.init_node('SpartonDigitalCompassIMU')
    Pos_pub = rospy.Publisher('imu/HeadingTrue', Pose2D)
    Imu_pub = rospy.Publisher('imu/data', Imu)
    SpartonPose2D=Pose2D()
    SpartonPose2D.x=float(0.0)
    SpartonPose2D.y=float(0.0)
    #Init D_Compass port
    D_Compassport = rospy.get_param('~port','/dev/ttyUSB0')
    D_Compassrate = rospy.get_param('~baud',115200)
    # printmodulus set to 1 is 100 Hz. 2 : 50Hz 
    D_Compassprintmodulus = rospy.get_param('~printmodulus',1)
    #Digital compass heading offset in degree
    D_Compass_offset = rospy.get_param('~offset',0.)
    imu_data = Imu()
    imu_data = Imu(header=rospy.Header(frame_id="SpartonCompassIMU"))
    
    #TODO find a right way to convert imu acceleration/angularvel./orientation accuracy to covariance
    imu_data.orientation_covariance = [1e-6, 0, 0, 
                                       0, 1e-6, 0, 
                                       0, 0, 1e-6]
    
    imu_data.angular_velocity_covariance = [1e-6, 0, 0,
                                            0, 1e-6, 0, 
                                            0, 0, 1e-6]
    
    imu_data.linear_acceleration_covariance = [1e-6, 0, 0, 
                                               0, 1e-6, 0, 
                                               0, 0, 1e-6]
    myStr1='\r\n\r\nprinttrigger 0 set drop\r\n'
    myStr2='printmask gyrop_trigger accelp_trigger or quat_trigger or yawt_trigger or time_trigger or set drop\r\n'
        # set the number high to get lower update rate , the IMU data is 100Hz rate , the string is 130 byte with 10 bit/byte , the max sampling rate is 88Hz
        # printmodulus=2 might give us around 40Hz update rate ,set printmodulus=1 should give you the max speed. ( with auto skiping )
    myStr_printmodulus=('printmodulus %i set drop\r\n' % D_Compassprintmodulus  )
    myStr3='printtrigger printmask set drop\r\n'

    rospy.on_shutdown(Spartonshutdownhook)

    try:
        #talker()
        #ReadCompass()
        #Setup Compass serial port
        D_Compass = serial.Serial(port=D_Compassport, baudrate=D_Compassrate, timeout=.5)
        # Stop continus mode
        D_Compass.write(myStr1)
        D_Compass.flush() # flush data out
        time.sleep(0.5)
        # readout all data, if any
        rospy.loginfo("Send Stop Continus mode to Digital Compass Got bytes %i" % D_Compass.inWaiting() ) # should got OK here
        if (D_Compass.inWaiting() >0):
                #read out all datas, the response shuldbe OK
                data=D_Compass.read(D_Compass.inWaiting())
                print("Send to Digital Compass: %s Got: %s" % (myStr1 ,data)) # should got OK here

        else:
                #sned error no data in buffer error
                rospy.logerr('[1]Received No data from DigitalCompass. Shutdown!')
                rospy.signal_shutdown('Received No data from DigitalCompass')
        D_Compass.write(myStr2) # send printmask
        data = D_Compass.readline()
#        rospy.loginfo("Send to Digital Compass: %s" % myStr2 ) # should got OK here
#        rospy.loginfo("Send to Digital Compass Got: %s" % data ) # should got OK here
        if (len(data) >0):
                #read out all datas, the response shuldbe OK
                rospy.loginfo("Send to Digital Compass: %s" % myStr2 ) # should got OK here
                rospy.loginfo("Send to Digital Compass. Got: %s" % data ) # should got OK here
                D_Compass.write(myStr_printmodulus) # setup printmodule
                data = D_Compass.readline()
                rospy.loginfo("Send to Digital Compass: %s " % myStr_printmodulus) # should got OK here
                rospy.loginfo("Send to Digital Compass. Got: %s" % data) # should got OK here

                D_Compass.write(myStr3) # start the data streaming
                data = D_Compass.readline()
                rospy.loginfo("Send to Digital Compass: %s " % myStr3 ) # should got OK here
                rospy.loginfo("Send to Digital Compass. Got: %s" % data) # should got OK here
                rospy.loginfo('Digital Compass Setup Complete!')
        else:
                #sned error no data in buffer
                rospy.logerr('[2]Received No data from DigitalCompass. Shutdown!')
                rospy.signal_shutdown('Received No data from DigitalCompass')

        #data = D_Compass.readline()
        #Read in D_Compass
        #Testdata='P:,%i,ap,-6.34,-22.46,1011.71,gp,0.00,0.00,-0.00,yt,342.53,q,0.98,-0.01,0.01,-0.15\n'
        #i=0
        while not rospy.is_shutdown():
            #read D_Compass line  , The data example $HCHDT,295.5,T*2B
            #                                        [0]    [1] 
            #i+=1
            #D_Compass.write(Testdata % i) # send testdata and do loop-back in RS232 for debug
            data = D_Compass.readline()
            #rospy.loginfo("Received a sentence: %s" % data)

            #if not check_checksum(data):
            #    rospy.logerr("Received a sentence with an invalid checksum. Sentence was: %s" % data)
            #    continue

            #DatatimeNow = rospy.get_rostime()
            DataTimeSec=rospy.get_time()
            fields = data.split(',')
            #print fields[0]+fields[2]+fields[6]+fields[10]+fields[12] #P:apgpytq

            try:
                if len(fields)>16:
                            if 'P:apgpytq' == (fields[0]+fields[2]+fields[6]+fields[10]+fields[12]):

                                #      0  1 mSec 2  3Ax  4Ay     5Az     5  7Gx  8Gy  9G    10 11YawT 1213w  14x   15y  16z
                                #data='P:,878979,ap,-6.34,-22.46,1011.71,gp,0.00,0.00,-0.00,yt,342.53,q,0.98,-0.01,0.01,-0.15'

                                Ax=float(fields[3])/1000.*9.81 # convert to m/s^2 from mg/s
                                Ay=float(fields[4])/1000.*9.81
                                Az=float(fields[5])/1000.*9.81
                                Gx=float(fields[7]) * (math.pi/180.0) # convert to radians from degrees
                                Gy=float(fields[8]) * (math.pi/180.0)
                                Gz=float(fields[9]) * (math.pi/180.0)
                                w =float(fields[13])
                                x =float(fields[14])
                                y =float(fields[15])
                                z =float(fields[16])
                                
                                #imu_data.header.stamp = rospy.Time.now() # Should add an offset here
                                imu_data.header.stamp = rospy.Time.from_sec(DataTimeSec-len(data)/11520.) # this is timestamp with a bit time offset 10bit per byte @115200bps
                                imu_data.orientation = Quaternion()
                                # IMU outputs [w,x,y,z] NED, convert to [x,y,z,w] ENU
                                imu_data.orientation.x = y
                                imu_data.orientation.y = x
                                imu_data.orientation.z = -z
                                imu_data.orientation.w = w
                                
                                # again note NED to ENU converstion
                                imu_data.angular_velocity.x = Gy
                                imu_data.angular_velocity.y = Gx
                                imu_data.angular_velocity.z = -Gz
                                # again note NED to ENU converstion
                                imu_data.linear_acceleration.x = Ay
                                imu_data.linear_acceleration.y = Ax
                                imu_data.linear_acceleration.z = -Az

                                Imu_pub.publish(imu_data)

                                SpartonPose2D.y=1000./(float(fields[1])-SpartonPose2D.x) # put update rate here for debug the update rate
                                SpartonPose2D.x=float(fields[1]) # put mSec tick here for debug the speed
                                SpartonPose2D.theta = wrapToPI(math.radians(90.-float(fields[11])-D_Compass_offset))

                                Pos_pub.publish(SpartonPose2D)


                            else:
                                rospy.logerr("[3]Received a sentence but not correct. Sentence was: %s" % data)
                else:
                        rospy.logerr("[4]Received a sentence but not correct. Sentence was: %s" % data)

            except ValueError as e:
                rospy.logwarn("Value error, likely due to missing fields in the data messages.Sentence was: %s" % data)

            # no loop, delay, ROSspin() here, we try to read all the data asap
        D_Compass.write(myStr1) # stop data stream before close port
        D_Compass.flush() # flush data out

        rospy.loginfo('Closing Digital Compass Serial port')
        D_Compass.close() #Close D_Compass serial port
    except rospy.ROSInterruptException:
        pass
