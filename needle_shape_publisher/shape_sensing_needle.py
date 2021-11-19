import numpy as np

# ROS2 packages
import rclpy
from rclpy import Parameter

# messages
from geometry_msgs.msg import PoseArray, Point, Pose
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Float64MultiArray, Header

# services

# custom package
from needle_shape_sensing.intrinsics import SHAPETYPE as NEEDLESHAPETYPE
from . import utilities
from .sensorized_shape_sensing_needle import NeedleNode


class ShapeSensingNeedleNode( NeedleNode ):
    """Needle to handle shape-sensing applications"""

    # - optimization options
    PARAM_OPTIMIZER = ".".join( [ NeedleNode.PARAM_NEEDLE, 'optimizer' ] )
    PARAM_KCINIT = ".".join( [ PARAM_OPTIMIZER, 'initial_kappa_c' ] )  # initial kappa_c for optimization
    PARAM_WINIT = ".".join( [ PARAM_OPTIMIZER, 'initial_w_init' ] )  # initial omega_init for optimization
    PARAM_OPTIM_MAXITER = ".".join( [ PARAM_OPTIMIZER, 'max_iterations' ] )
    PARAM_OPTIM_MAXITER_LB = 2

    def __init__( self, name="ShapeSensingNeedle" ):
        super().__init__( name )

        # declare ang get parameters
        self.kc_i = np.array( [ 0.002 ] )
        self.w_init_i = np.array( [ self.kc_i[ 0 ], 0, 0 ] )
        pd_optim_maxiter = ParameterDescriptor( name=self.PARAM_OPTIM_MAXITER, type=Parameter.Type.INTEGER.value,
                                                description="Maximum iterations for convergence" )
        optim_maxiter = self.declare_parameter( pd_optim_maxiter.name, value=15,
                                                descriptor=pd_optim_maxiter ).get_parameter_value().integer_value

        # configure shape-sensing needle
        self.ss_needle.optimizer.options[ 'options' ] = { 'maxiter': optim_maxiter }
        self.ss_needle.ref_wavelengths = np.ones_like( self.ss_needle.ref_wavelengths )
        self.ss_needle.current_depth = 125  # TODO: need to change insertion depth. Keep for testing
        self.ss_needle.current_curvatures = np.zeros( (self.ss_needle.num_activeAreas, 2), dtype=float )

        # configure current needle pose paramters
        self.current_insertion_pt = None
        self.current_needle_pose = (None, None)  # Rmat, pmat

        # create publishers
        self.pub_kc = self.create_publisher( Float64MultiArray, 'state/kappac', 1 )
        self.pub_winit = self.create_publisher( Float64MultiArray, 'state/winit', 1 )
        self.pub_shape = self.create_publisher( PoseArray, 'state/winit', 1 )

        # create subscriptions
        self.sub_curvatures = self.create_subscription( Float64MultiArray, 'state/curvatures',
                                                        self.sub_curvatures_callback, 10 )
        self.sub_entrypoint = self.create_subscription( Point, 'state/skin_entry', self.sub_entrypoint_callback, 10 )
        self.sub_needlepose = self.create_subscription( Pose, 'state/pose', self.sub_needlepose_callback, 10 )

        # create timers
        self.pub_shape_timer = self.create_timer( 3, self.publish_shape )

    # __init__

    def get_needleshape( self ):
        """ Get the current needle shape"""
        if self.ss_needle.current_shapetype & NEEDLESHAPETYPE.SINGLEBEND_SINGLELAYER == NEEDLESHAPETYPE.SINGLEBEND_SINGLELAYER:  # single layer
            pmat, Rmat = self.ss_needle.get_needle_shape( self.kc_i[ 0 ], self.w_init_i )

        elif self.ss_needle.current_shapetype & NEEDLESHAPETYPE.SINGLEBEND_DOUBLELAYER == 0x02:  # 2 layers
            if len( self.kc_i ) < 2:
                pmat, Rmat = self.ss_needle.get_needle_shape( self.kc_i[ 0 ], self.kc_i[ 0 ], self.w_init_i )
            else:
                pmat, Rmat = self.ss_needle.get_needle_shape( *self.kc_i[ 0:2 ], self.w_init_i )

        # elif
        else:
            self.get_logger().error( f"Needle shape type: {self.ss_needle.current_shapetype} is not implemented." )
            self.get_logger().error( f"Resorting to shape type: {NEEDLESHAPETYPE.SINGLEBEND_SINGLELAYER}." )
            self.ss_needle.update_shapetype( NEEDLESHAPETYPE.SINGLEBEND_SINGLELAYER )
            pmat, Rmat = None, None  # pop out of the loop and redo

        # else

        return pmat, Rmat

    # get_needleshape

    def publish_shape( self ):
        """ Publish the 3D needle shape"""
        pmat, Rmat = self.get_needleshape()

        # update initial kappa_c values
        self.kc_i = self.ss_needle.current_kc
        self.w_init_i = self.ss_needle.current_winit

        # check to make sure messages are not None
        if pmat is None or Rmat is None:
            self.get_logger().warn( f"pmat or Rmat is None: {pmat}, {Rmat}" )
            self.get_logger().warn( f"Current shapetype: {self.ss_needle.current_shapetype}" )
            return

        # if

        # TODO: include straight needle section

        # generate pose message
        header = Header( stamp=self.get_clock().now().to_msg() )
        msg_shape = utilities.poses2msg( pmat, Rmat, header=header )

        # generate kappa_c and w_init message
        msg_kc = Float64MultiArray( data=self.kc_i )
        msg_winit = Float64MultiArray( data=self.w_init_i.tolist() )

        # publish the messages
        self.pub_shape.publish( msg_shape )
        self.pub_kc.publish( msg_kc )
        self.pub_winit.publish( msg_winit )

    # publish_shape

    def sub_curvatures_callback( self, msg: Float64MultiArray ):
        """ Subscription to needle sensor curvatures """
        # grab the current curvatures
        curvatures = np.reshape( msg.data, (-1, 2), order='F' )

        # update the curvatures
        self.ss_needle.current_curvatures = curvatures

        if not self.ss_needle.is_calibrated:
            self.ss_needle.ref_wavelengths = np.ones_like( self.ss_needle.ref_wavelengths )

        # if

    # sub_curvatures_callback

    def sub_entrypoint_callback( self, msg: Point ):
        """ Subscription to entrypoint topic """
        # TODO: update current ssneedle insertion depth
        self.current_insertion_pt = np.array( [ msg.x, msg.y, msg.z ] )

    # sub_entrypoint_callback

    def sub_needlepose_callback( self, msg: Pose ):
        """ Subscription to entrypoint topic """
        self.current_needle_pose = utilities.msg2pose( msg )

    # sub_needlepose_callback


# class: ShapeSensingNeedleNode

def main( args=None ):
    rclpy.init( args=args )

    ssneedle_node = ShapeSensingNeedleNode()

    try:
        rclpy.spin( ssneedle_node )

    except KeyboardInterrupt:
        pass

    # clean-up
    ssneedle_node.destroy_node()
    rclpy.shutdown()


# main

if __name__ == "__main__":
    main()

# if __main__
