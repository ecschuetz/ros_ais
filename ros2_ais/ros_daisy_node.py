#!/usr/bin/env python3

import rclpy
import rclpy.node
from rclpy.parameter import Parameter, ParameterType
from ais import decode
from ais_common.msg import Rosais123
from ais_common.msg import Rosais411
from ais_common.msg import Rosais8dac200
from sensor_msgs.msg import NavSatFix
import serial

class SensorAIS(rclpy.node.Node):
    def __init__(self):
        super().__init__('ros_dAISy_node')
        self.node = rclpy.create_node('ros_dAISy_node')

        # Get node name
        self.node_name = self.node.get_name()
        # Declara params
        #self.declare_parameter('serial_port', '/dev/ttyUSB0')

        self.declare_parameters(
        namespace='',
        parameters=[
            ('serial_port', 'Dev/ttyUSB0'),
            ('frame_id', 'ais_link'),
            ('frequency', 10),
            ('baudrate', 115200),
        ]
    )



        # Internal variables
        self.ais_data_seq_counter = 0

        # Create Topics
#        self.pub_ais_data = self.node.create_publisher(ros_AIS123, 'ais/position_report', 10)
#        self.pub_ais_data2 = self.node.create_publisher(ros_AIS4_11, 'ais/Base_station_report' , 10)
#        self.pub_ais_data3 = self.node.create_publisher(ros_AIS8_dac200, 'ais/geometry_report' , 10)
#        self.pub_NavSatFix = self.node.create_publisher(NavSatFix, 'ais/NavSatFix' , 10)
        # Print node status
        self.node.get_logger().info(self.node_name + " initialized")
        self.get_ros_params()
        while rclpy.ok():
            try:
                self.publish_ais_data()
            except Exception as e:
                self.node.get_logger().warn(self.node_name + ": Error while publishing AIS data!")
                self.node.get_logger().warn(e)
                #rospy.logwarn(self.node_name + ": Error while publishing AIS data!")
                #rospy.logwarn(e)
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def get_ros_params(self):
        self.node.get_logger().info(self.node_name + " search params")
        self.serial_port = self.get_parameter('serial_port').value 
       # self.serial_port = self.node.get_parameter(self.node_name + '/serial_port').value
        self.frame_id = self.get_parameter('frame_id').value
        self.frequency = self.get_parameter('frequency').value
        self.baudrate = self.get_parameter('baudrate').value
       # self.node.get_logger().info(self.node_name + "parameter getted" + self.serial_port + self.frame_id + self.frequency + self.baudrate)

    def publish_ais_data(self):
        # Create an ais serial connectioninstance
        self.node.get_logger().info(self.node_name + " connect to dAISy on port: " + self.serial_port + " with baudrate: " + str(self.baudrate) )
        self.dAISy = serial.Serial(self.serial_port,self.baudrate)
        ais_data = Rosais123()
        ais_data2 = Rosais411()
        ais_data3 = Rosais8dac200()
        ais_nav = NavSatFix()
        self.node.get_logger().info(self.node_name + " is looking for data. . .")
        # Read data from serial port for a maximum of 10 seconds  and repeat  
        self.dAISy.timeout=10       
        input=self.dAISy.readline()     
        self.node.get_logger().info(self.node_name + " input: " + str(input) + " type: " + str(type(input)))    
        if str(input) == "b''":
            self.node.get_logger().info(self.node_name + " no data received in 10 seconds")
            self.publish_ais_data()
            return  # no data found in 10 seconds       
        self.node.get_logger().info(self.node_name + " data found")
        # save undecoded input to ais_data
        #ais_data.raw = str(input)
        input=str(input)
        fields = input.split(',')
        if int(fields[1])== 1:
                decoded=decode(str(fields[5]), int(fields[6][0]))
                if decoded['id'] == 1 or decoded['id'] == 2 or decoded['id'] == 3 or decoded['id'] == 18:
                    #rospy.loginfo(self.node_name + ": Position Report received from mmsi:" + str(decoded['mmsi']))
                    self.pub_ais_data = self.node.create_publisher( Rosais123, 'ais/' + 'id_' + str(decoded['mmsi']) + '/position_report', 10)
                    self.pub_NavSatFix = self.node.create_publisher( NavSatFix , 'ais/' + 'id_' + str(decoded['mmsi']) + '/NavSatFix', 10)
                    ais_data.header.stamp = self.get_clock().now().to_msg()
                    ais_data.header.frame_id =str(decoded['mmsi'])
                    ais_data.id = decoded['id']
                    ####NAVIGATION ''''
                    ais_nav.header.stamp = self.get_clock().now().to_msg()
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
                    #rospy.loginfo(self.node_name + ": Base Station Report received from mmsi:" + str(decoded['mmsi']))
                    self.pub_ais_data2 = self.node.create_publisher( Rosais411 ,'ais/' + 'id_' + str(decoded['mmsi']) + '/Base_station_report' , 10)
                    self.pub_NavSatFix = self.node.create_publisher( NavSatFix ,'ais/' + 'id_' + str(decoded['mmsi']) + '/NavSatFix' , 10)
                    ais_data2.header.stamp = self.get_clock().now().to_msg()
                    ais_data2.header.frame_id = str(decoded['mmsi'])
                    ais_data2.id = decoded['id']
                     ####NAVIGATION ''''
                    ais_nav.header.stamp = self.get_clock().now().to_msg()
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
                    #rospy.loginfo(self.node_name + ": Ship geometry received from mmsi:" + str(decoded['mmsi']))
                    self.pub_ais_data3 = self.node.create_publisher( Rosais8dac200 , 'ais/' + 'id_' + str(decoded['mmsi']) + '/geometry_report' , 10)
                    self.pub_NavSatFix = self.node.create_publisher( NavSatFix , 'ais/' + 'id_' + str(decoded['mmsi']) + '/geometry_report' , 10)
                    ais_data3.header.stamp = self.get_clock().now().to_msg()
                    ais_data3.header.frame_id = str(decoded['mmsi'])
                    ais_data3.id = decoded['id']
                    ais_data3.mmsi = decoded['mmsi']
                    ais_data3.length = decoded['length']
                    ais_data3.beam = decoded['beam']
                    ais_data3.draught = decoded['draught']

                    self.pub_ais_data3.publish(ais_data3)


                else:
                    self.node.get_logger().warn(self.node_name + ": unknown NMEA/AIVMD Sentence received!")
                    self.node.get_logger().warn(input)
                    self.node.get_logger().warn(decoded)
                    #rospy.logwarn(self.node_name + ": unknown NMEA/AIVMD Sentence received!")
                    #rospy.logwarn(input)
                    #rospy.logwarn(decoded)
        else:
                self.node.get_logger().warn(self.node_name + ": could not decode NMEA Sentence!")
                self.node.get_logger().warn(input)
                #rospy.logwarn(self.node_name + ": could not decode NMEA Sentence!")
                #rospy.logwarn(input)

        self.ais_data_seq_counter=+1

    def run(self):
        while rclpy.ok():
            try:
                self.publish_ais_data()
            except Exception as e:
                self.node.get_logger().warn(self.node_name + ": Error while publishing AIS data!")
                self.node.get_logger().warn(e)
                #rospy.logwarn(self.node_name + ": Error while publishing AIS data!")
                #rospy.logwarn(e)
            rclpy.spin_once(self.node, timeout_sec=0.1)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = SensorAIS()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    

