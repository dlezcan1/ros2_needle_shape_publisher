import os
import re

import numpy as np
# ROS2 packages
import rclpy
from rclpy import Parameter
# messages
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray

# current package
from . import utilities
from .sensorized_shape_sensing_needle import NeedleNode
from .shape_sensing_needle import ShapeSensingNeedleNode


class ShapeSensingNeedleDemoNode( NeedleNode ):
    """Needle to handle shape-sensing applications"""

    # - optimization options
    PARAM_OPTIMIZER = ".".join( [ NeedleNode.PARAM_NEEDLE, 'optimizer' ] )
    PARAM_KCINIT = ".".join( [ PARAM_OPTIMIZER, 'initial_kappa_c' ] )  # initial kappa_c for optimization
    PARAM_WINIT = ".".join( [ PARAM_OPTIMIZER, 'initial_w_init' ] )  # initial omega_init for optimization
    PARAM_OPTIM_MAXITER = ".".join( [ PARAM_OPTIMIZER, 'max_iterations' ] )
    PARAM_OPTIM_MAXITER_LB = 2

    # needle pose parameters
    # R_NEEDLEPOSE = geometry.rotx( -np.pi / 2 )  # +z-axis -> +y-axis
    R_NEEDLEPOSE = np.array( [ [ -1, 0, 0 ],
                               [ 0, 0, 1 ],
                               [ 0, 1, 0 ] ] )

    def __init__( self, name="ShapeSensingNeedleDemo" ):
        super().__init__( name )

        # declare ang get parameters
        needle_data_path = self.declare_parameter( "needle.demoDataDir" ).get_parameter_value().string_value

        self.kc_i = np.array( [ 0.002 ] )
        self.w_init_i = np.array( [ self.kc_i[ 0 ], 0, 0 ] )
        pd_optim_maxiter = ParameterDescriptor( name=self.PARAM_OPTIM_MAXITER, type=Parameter.Type.INTEGER.value,
                                                description="Maximum iterations for convergence" )
        optim_maxiter = self.declare_parameter( pd_optim_maxiter.name, value=15,
                                                descriptor=pd_optim_maxiter ).get_parameter_value().integer_value

        # configure shape-sensing needle
        self.ss_needle.optimizer.options[ 'options' ] = { 'maxiter': optim_maxiter }
        self.ss_needle.ref_wavelengths = np.ones_like( self.ss_needle.ref_wavelengths )
        self.ss_needle.current_depth = 0  # TODO: need to change insertion depth. Keep for testing
        # self.air_depth = 0  # the length of the needle in the air
        self.ss_needle.current_curvatures = np.zeros( (2, self.ss_needle.num_activeAreas), dtype=float )

        # configure current needle pose parameters
        # self.current_insertion_pt = np.zeros( 3 )
        # self.current_needle_pose = (np.zeros( 3 ), self.R_NEEDLEPOSE)
        # self.history_needle_pose = np.array( [ 0, 0 ] ).reshape( -1,
        #                                                          1 )  # look-up table of (insertion depth, theta rotation (rads))
        self.needle_shapes = [ ]
        self.needle_shape_counter = 0
        self.needle_shape_counter_step = 1
        self.__load_shapes( needle_data_path )

        # create publishers
        # self.pub_kc = self.create_publisher( Float64MultiArray, 'state/kappac', 1 )
        # self.pub_winit = self.create_publisher( Float64MultiArray, 'state/winit', 1 )
        self.pub_shape = self.create_publisher( PoseArray, 'state/current_shape', 1 )

        # # create subscriptions
        # self.sub_curvatures = self.create_subscription( Float64MultiArray, 'state/curvatures',
        #                                                 self.sub_curvatures_callback, 10 )
        # self.sub_entrypoint = self.create_subscription( Point, 'state/skin_entry', self.sub_entrypoint_callback, 10 )
        # self.sub_needlepose = self.create_subscription( PoseStamped, '/stage/state/needle_pose',
        #                                                 self.sub_needlepose_callback, 10 )

        # create timers
        self.pub_shape_timer = self.create_timer( 0.05, self.publish_shape )
        self.update_shape_timer = self.create_timer( 2.0, self.update_shape )

    # __init__

    # ================================ PROPERTIES =====================================================================
    @property
    def insertion_depth( self ):
        return self.ss_needle.current_depth

    # property: insertion_depth

    @insertion_depth.setter
    def insertion_depth( self, depth ):
        self.ss_needle.current_depth = depth

    # insertion_depth setter

    # =============================== PRIVATE METHODS =================================================================
    def __load_shapes( self, demo_path ):
        """ Load the demo needle shapes to cycle through"""
        files = [ f for f in os.listdir( demo_path ) if f.endswith( 'mm.csv' ) and f.startswith( 'needle_3dshape' ) ]

        re_pattern = r".*needle_3dshape_([0-9]+)-mm.csv"
        for file in files:
            depth = int( re.search( re_pattern, file ).groups()[ 0 ] )
            shape = np.loadtxt( os.path.join( demo_path, file ), delimiter=',' )
            self.get_logger().info( f"Needle shape numpy.shape: {shape.shape}" )
            self.needle_shapes.append( (depth, shape) )

        # for

        # sort the needle shapes
        self.needle_shapes = sorted( self.needle_shapes, key=lambda x: x[ 0 ] )
        self.get_logger().info( "Needle shape depths:" + str( [ depth for depth, shape in self.needle_shapes ] ) )

    # __load_shapes

    # =============================== METHODS =========================================================================
    def publish_shape( self ):
        """ Publish the needle shape. """
        pmat = self.needle_shapes[ self.needle_shape_counter ][ 1 ]
        Rmat = np.eye( 3 )[ np.newaxis ].repeat( pmat.shape[ 0 ], axis=0 )

        pmat_tf = pmat @ ShapeSensingNeedleNode.R_NEEDLEPOSE.T

        header = Header( frame_id='robot', stamp=self.get_clock().now().to_msg() )
        msg = utilities.poses2msg( pmat_tf, Rmat, header=header )

        self.pub_shape.publish( msg )
        self.get_logger().info(
                f"Published needle shape of length: {self.needle_shapes[ self.needle_shape_counter ][ 0 ]}" )

    # publish_shape

    def update_shape( self ):
        """ Update the current needle shape """
        # update the counter
        if self.needle_shape_counter >= len( self.needle_shapes ) - 1:
            self.needle_shape_counter_step = -1

        elif self.needle_shape_counter <= 0:
            self.needle_shape_counter_step = 1

        # update the counter
        self.needle_shape_counter += self.needle_shape_counter_step

    # update_shape


# class: ShapeSensingNeedleDemoNode


def main( args=None ):
    rclpy.init( args=args )

    ssneedle_demo_node = ShapeSensingNeedleDemoNode()

    try:
        rclpy.spin( ssneedle_demo_node )

    except KeyboardInterrupt:
        pass

    # clean-up
    ssneedle_demo_node.destroy_node()
    rclpy.shutdown()


# main

if __name__ == "__main__":
    main()

# if __main__
