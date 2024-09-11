#!/usr/bin/env python3
from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

import rclpy.time
import tf2_py
import tf2_py._tf2_py
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from builtin_interfaces.msg import Time
from tf2_msgs.msg import TFMessage
import sys
import csv

class ListenerToCsv(Node):

    def __init__(self, baseFrame, robotLeft, robotRight, humanLeft, humanRight,robotBaseFrame, dataList):
        super().__init__('ListenerToCsv')
        my_new_param = rclpy.parameter.Parameter(
            'use_sim_time',
            rclpy.Parameter.Type.BOOL,
            True
        )
        all_new_parameters = [my_new_param]
        self.set_parameters(all_new_parameters)
        #self.query_time = self.get_clock().now() 
        #self.subscription = self.create_subscription(TFMessage, '/tf', self.on_timer,10)
        #self.subscription
        # Declare and acquire `target_frame` parameter
        self.robot_left = robotLeft
        self.robot_right = robotRight

        self.base_frame = baseFrame
        self.tf_buffer = Buffer(rclpy.time.Duration(seconds=20))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.data_list = dataList
        self.human_left = humanLeft
        self.human_right = humanRight
        self.robot_base_frame = robotBaseFrame
        self.init_time = rclpy.time.Time()
        self.init_time_set = False

        # Call on_timer function every second
        self.timer = self.create_timer(0.01, self.on_timer)
        self.counter=0
        
        
    def on_timer(self):
        self.query_time = rclpy.time.Time()
        print(self.query_time)
        try:
            t1 = self.tf_buffer.lookup_transform(self.robot_base_frame,self.robot_left,   self.query_time, rclpy.time.Duration(seconds=0.1))
        except TransformException as ex:
            self.get_logger().info(
            f'Could not transform {self.robot_base_frame} to {self.robot_left}: {ex}')
            return
        at1 =  t1.header.stamp.sec + 1e-9 *t1.header.stamp.nanosec

        try:
            t2 = self.tf_buffer.lookup_transform( self.robot_base_frame, self.robot_right, self.query_time, rclpy.time.Duration(seconds=0.1))
        except TransformException as ex:
            self.get_logger().info(
            f'Could not transform {self.robot_base_frame} to {self.robot_right}: {ex}')
            return
        at2 =  t2.header.stamp.sec + 1e-9 *t2.header.stamp.nanosec

        try:
            t3 = self.tf_buffer.lookup_transform( self.robot_base_frame, self.human_left,self.query_time, rclpy.time.Duration(seconds=0.1))
        except TransformException as ex:
            self.get_logger().info(
            f'Could not transform {self.robot_base_frame} to {self.human_left}: {ex}')
            return
        ht1 =  t3.header.stamp.sec + 1e-9 *t3.header.stamp.nanosec
        try:
            t4 = self.tf_buffer.lookup_transform( self.robot_base_frame,self.human_right, self.query_time, rclpy.time.Duration(seconds=0.1))
        except TransformException as ex:
            self.get_logger().info(
            f'Could not transform {self.robot_base_frame} to {self.human_right}: {ex}')
            return
        ht2 =  t4.header.stamp.sec + 1e-9 *t4.header.stamp.nanosec
    
        try:
            t5 = self.tf_buffer.lookup_transform( self.base_frame,self.robot_base_frame, self.query_time, rclpy.time.Duration(seconds=0.1))
        except TransformException as ex:
            self.get_logger().info(
            f'Could not transform {self.base_frame} to {self.robot_base_frame}: {ex}')
            return
        self.counter = self.counter+1
        print(self.counter)

        rt1 =  t5.header.stamp.sec + 1e-9 *t5.header.stamp.nanosec
        self.data_list.append({'x1':t1.transform.translation.x,'y1':t1.transform.translation.y, 'x2':t2.transform.translation.x,'y2':t2.transform.translation.y, 'hx1':t3.transform.translation.x,'hy1':t3.transform.translation.y, 'hx2':t4.transform.translation.x,'hy2':t4.transform.translation.y,'rx1':t5.transform.translation.x, 'ry1':t5.transform.translation.y, 'at1':at1,'at2':at2,'ht1':ht1,'ht2':ht2, 'rt1':rt1})

    def clock_callback(self, msg):
        self.query_time = msg 
        self.get_logger().info('I heard: "%d"' % msg.sec) 
        self.on_timer()      

def main():
    rclpy.init()
    dataList = list()
    robotLeft = sys.argv[1]
    robotRight = sys.argv[2]
    humanLeft = sys.argv[3]
    humanRight = sys.argv[4]
    baseFrame = sys.argv[5]
    robotBaseFrame = sys.argv[6]
    filename = sys.argv[7]
    node = ListenerToCsv(baseFrame, robotLeft,robotRight, humanLeft, humanRight,robotBaseFrame, dataList)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    with open(filename,'w', newline='') as csvfile:
        fieldnames = ['x1', 'y1', 'x2', 'y2', 'hx1', 'hy1', 'hx2', 'hy2', 'rx1','ry1', 'at1','at2','ht1','ht2','rt1']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writerows(dataList)
        
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()
