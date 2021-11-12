import numpy as np
from typing import List

# ROS2 packages
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

# messages
from std_msgs.msg import Header, Float64MultiArray, MultiArrayDimension, MultiArrayLayout
from geometry_msgs.msg import Point, Pose, PoseArray
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

# services

# custom package
from needle_shape_sensing import geometry
from needle_shape_sensing.intrinsics import SHAPETYPE as NEEDLESHAPETYPE
from needle_shape_sensing.shape_sensing import ShapeSensingFBGNeedle


class SensorizedNeedleNode(Node):
    """Needle to handle sensor calibration/curvatures"""
    # TODO
    pass

# class: SensorizedNeedleNode

def main(args=None):
    rclpy.init(args=args)

    # clean-up
    rclpy.shutdown()

# main

if __name__ == "__main__":
    main()

# if __main__





