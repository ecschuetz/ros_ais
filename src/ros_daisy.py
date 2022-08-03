#!/usr/bin/env python3

import rospy
from ais import decode
from ros_ais.msg import ros_AIS123
from ros_ais.msg import ros_AIS4_11
from ros_ais.msg import ros_AIS8_dac200
from sensor_msgs.msg import NavSatFix
import serial

class SensorAIS:
    def __init__(self):
        rospy.init_node('ros_dAISy_node', anonymous=False)
        # Get node name
        self.node_name = rospy.get_name()

        # Get ros params
        self.get_ros_params()

        # Create an ais instance
        self.dAISy = serial.Serial(self.serial_port,self.baudrate) #ttyAMA0 ist die serielle Schnittstelle, kann bei Verwendung von z.B. USB anders heißen

        # Internal variables
        self.ais_data_seq_counter = 0

        # Create Topics
#        self.pub_ais_data = rospy.Publisher('ais/position_report', ros_AIS123, queue_size=10)
#        self.pub_ais_data2 = rospy.Publisher('ais/Base_station_report' , ros_AIS4_11 , queue_size=10)
#        self.pub_ais_data3 = rospy.Publisher('ais/geometry_report' , ros_AIS8_dac200 , queue_size=10)
#        self.pub_NavSatFix = rospy.Publisher('ais/NavSatFix' , NavSatFix , queue_size=10)
        # Print node status
        rospy.loginfo(self.node_name + " initialized")

    def get_ros_params(self):
        self.serial_port = rospy.get_param(self.node_name + '/serial_port', '/dev/ttyAMA0')
        self.frame_id = rospy.get_param(self.node_name + '/frame_id', 'ais_link')
        self.frequency = rospy.get_param(self.node_name + '/frequency', 10)
        self.baudrate = rospy.get_param(self.node_name + '/baudrate', 38400)

    def publish_ais_data(self):

        ais_data = ros_AIS123()
        ais_data2 = ros_AIS4_11()
        ais_data3 = ros_AIS8_dac200()
        ais_nav = NavSatFix()
     #   rospy.loginfo(self.node_name + " is looking for data. . .")
        input=self.dAISy.readline() # einlesen der seriellen Daten
        input=str(input)
        fields = input.split(',')
        if int(fields[1])== 1:
                decoded=decode(str(fields[5]), int(fields[6][0]))
                if decoded['id'] == 1 or decoded['id'] == 2 or decoded['id'] == 3 or decoded['id'] == 18:
                    rospy.loginfo(self.node_name + ": Position Report received from mmsi:" + str(decoded['mmsi']))
                    self.pub_ais_data = rospy.Publisher('ais/' + str(decoded['mmsi']) + '/position_report', ros_AIS123, queue_size=10)
                    self.pub_NavSatFix = rospy.Publisher('ais/' +  str(decoded['mmsi']) + '/NavSatFix', NavSatFix , queue_size=10)
                    ais_data.header.stamp = rospy.Time.now()
                    ais_data.header.frame_id =str(decoded['mmsi'])
                    ais_data.id = decoded['id']
                    ####NAVIGATION ''''
                    ais_nav.header.stamp = rospy.Time.now()
                    ais_nav.header.frame_id = str(decoded['mmsi'])
                    ais_nav.longitude = decoded['x']
                    ais_nav.latitude = decoded['y']
                 #   ais_data.repeat_indicator = decoded['repeat_indicator']
                    ais_data.mmsi = decoded['mmsi']
                    
                #    ais_data.nav_status = decoded['nav_status']
                #    ais_data.rot_over_range = decoded['rot_over_range']
               #     ais_data.rot = decoded['rot']
                    ais_data.longitude = decoded['x']
                    ais_data.latitude = decoded['y']
                    ais_data.sog = decoded['sog']
                    ais_data.position_accuracy = decoded['position_accuracy']
                    ais_data.cog = decoded['cog']
                    ais_data.true_heading = decoded['true_heading']
                    ais_data.timestamp = decoded['timestamp']
              #      ais_data.special_manoeuvre = decoded['special_manoeuvre']
             #       ais_data.spare = decoded['spare']
            #        ais_data.raim = decoded['raim']
            #      ais_data.sync_state = decoded['sync_state']
             #       ais_data.slot_timeout = decoded['slot_timeout']
             #       ais_data.utc_hour = decoded['utc_hour']
             #      ais_data.utc_min = decoded['utc_min']
             #       ais_data.utc_spare = decoded['utc_spare']
                    self.pub_ais_data.publish(ais_data)
                    self.pub_NavSatFix.publish(ais_nav)
                elif decoded['id'] == 4 or decoded['id'] == 11:
                    rospy.loginfo(self.node_name + ": Base Station Report received from mmsi:" + str(decoded['mmsi']))
                    self.pub_ais_data2 = rospy.Publisher('ais/' + str(decoded['mmsi']) + '/Base_station_report', ros_AIS4_11 , queue_size=10)
                    self.pub_NavSatFix = rospy.Publisher('ais/' + str(decoded['mmsi']) + '/NavSatFix', NavSatFix , queue_size=10)
                    ais_data2.header.stamp = rospy.Time.now()
                    ais_data2.header.frame_id = str(decoded['mmsi'])
                    ais_data2.id = decoded['id']
                     ####NAVIGATION ''''
                    ais_nav.header.stamp = rospy.Time.now()
                    ais_nav.header.frame_id = str(decoded['mmsi'])
                    ais_nav.longitude = decoded['x']
                    ais_nav.latitude = decoded['y']


               #     ais_data2.repeat_indicator = decoded['repeat_indicator']
                    ais_data2.mmsi = decoded['mmsi']
                    ais_data2.year = decoded['year']
                    ais_data2.month = decoded['month']
                    ais_data2.day = decoded['day']
                    ais_data2.hour = decoded['hour']
                    ais_data2.minute = decoded['minute']
                    ais_data2.second = decoded['second']
                    ais_data2.position_accuracy = decoded['position_accuracy']
                    ais_data2.latitude = decoded['y']
                    ais_data2.longitude = decoded['x']
                 #   ais_data2.fix_type = decoded['fix_type']
                  #  ais_data2.transmission_ctl = decoded['transmission_ctl']
                 #   ais_data2.spare = decoded['spare']
                #    ais_data2.raim = decoded['raim']


                    self.pub_ais_data2.publish(ais_data2)
                    self.pub_NavSatFix.publish(ais_nav)


                elif decoded['id'] == 8 and decoded['dac'] == 200:
                    rospy.loginfo(self.node_name + ": Ship geometry received from mmsi:" + str(decoded['mmsi']))
                    self.pub_ais_data3 = rospy.Publisher('ais/' + str(decoded['mmsi']) + '/geometry_report', ros_AIS8_dac200 , queue_size=10)
                    self.pub_NavSatFix = rospy.Publisher('ais/' + str(decoded['mmsi']) + '/geometry_report' , NavSatFix , queue_size=10)
                    ais_data3.header.stamp = rospy.Time.now()
                    ais_data3.header.frame_id = str(decoded['mmsi'])
                    ais_data3.id = decoded['id']
                    ais_data3.mmsi = decoded['mmsi']
                    ais_data3.length = decoded['length']
                    ais_data3.beam = decoded['beam']
                    ais_data3.draught = decoded['draught']

                    self.pub_ais_data3.publish(ais_data3)


                else:
                    rospy.logwarn(self.node_name + ": unknown NMEA/AIVMD Sentence received!")
                    rospy.logwarn(input)
                    rospy.logwarn(decoded)
        else:
                rospy.logwarn(self.node_name + ": could not decode NMEA Sentence!")
                rospy.logwarn(input)

        self.ais_data_seq_counter=+1
        
        
        
        





    def run(self):




           # Set frequency
           rate = rospy.Rate(self.frequency)

           while not rospy.is_shutdown():

               self.publish_ais_data()

               rate.sleep()


if __name__ == '__main__':

    ais = SensorAIS()
    try:
        ais.run()
    except rospy.ROSInterruptException:
        pass
			
