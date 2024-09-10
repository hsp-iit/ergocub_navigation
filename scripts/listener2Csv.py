#!/usr/bin/env python3
from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import sys
import csv

class ListenerToCsv(Node):

    def __init__(self, baseFrame, robotLeft, robotRight, humanLeft, humanRight,robotBaseFrame, dataList):
        super().__init__('ListenerToCsv')

        # Declare and acquire `target_frame` parameter
        self.robot_left = robotLeft
        self.robot_right = robotRight

        self.base_frame = baseFrame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.data_list = dataList
        self.human_left = humanLeft
        self.human_right = humanRight
        self.robot_base_frame = robotBaseFrame 
        # Call on_timer function every second
        self.timer = self.create_timer(0.03, self.on_timer)
        
    def on_timer(self):
        try:
            t1 = self.tf_buffer.lookup_transform(self.base_frame,self.robot_left,   rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
            f'Could not transform {self.robot_left} to {self.base_frame}: {ex}')
            return
        try:
            t2 = self.tf_buffer.lookup_transform( self.base_frame, self.robot_right,  rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
            f'Could not transform {self.robot_right} to {self.base_frame}: {ex}')
            return
        
        try:
            t3 = self.tf_buffer.lookup_transform( self.base_frame, self.human_left,  rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
            f'Could not transform {self.human_left} to {self.base_frame}: {ex}')
            return
        try:
            t4 = self.tf_buffer.lookup_transform( self.base_frame,self.human_right,   rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
            f'Could not transform {self.human_right} to {self.base_frame}: {ex}')
            return
        
        try:
            t4 = self.tf_buffer.lookup_transform( self.base_frame,self.human_right,   rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
            f'Could not transform {self.human_right} to {self.base_frame}: {ex}')
            return
        try:
            t5 = self.tf_buffer.lookup_transform( self.base_frame,self.robot_base_frame,   rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
            f'Could not transform {self.human_right} to {self.base_frame}: {ex}')
            return
        
        
        self.data_list.append({'x1':t1.transform.translation.x,'y1':t1.transform.translation.y, 'x2':t2.transform.translation.x,'y2':t2.transform.translation.y, 'hx1':t3.transform.translation.x,'hy1':t3.transform.translation.y, 'hx2':t4.transform.translation.x,'hy2':t4.transform.translation.y,'rx1':t5.transform.translation.x, 'ry1':t5.transform.translation.y})

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
        fieldnames = ['x1', 'y1', 'x2', 'y2', 'hx1', 'hy1', 'hx2', 'hy2', 'rx1','ry1']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writerows(dataList)
        
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()
