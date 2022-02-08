import numpy as np
# ROS2 packages
import rclpy
from rclpy import Parameter
# messages
from geometry_msgs.msg import PoseArray, Point, Pose
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Float64MultiArray, Header
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

    # needle pose parameters
    # R_NEEDLEPOSE = geometry.rotx( -np.pi / 2 )  # +z-axis -> +y-axis
    R_NEEDLEPOSE = np.array( [ [ -1, 0, 0 ],
                               [ 0, 0, 1 ],
                               [ 0, 1, 0 ] ] )

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
        self.ss_needle.current_curvatures = np.zeros( (2, self.ss_needle.num_activeAreas), dtype=float )

        # configure current needle pose parameters
        self.current_insertion_pt = None
        self.current_needle_pose = (np.zeros( 3 ), self.R_NEEDLEPOSE)
        self.history_needle_pose = np.array( [ 0, 0 ] ).reshape( -1,
                                                                 1 )  # look-up table of (insertion depth, theta rotation (rads))

        # create publishers
        self.pub_kc = self.create_publisher( Float64MultiArray, 'state/kappac', 1 )
        self.pub_winit = self.create_publisher( Float64MultiArray, 'state/winit', 1 )
        self.pub_shape = self.create_publisher( PoseArray, 'state/current_shape', 1 )

        # create subscriptions
        self.sub_curvatures = self.create_subscription( Float64MultiArray, 'state/curvatures',
                                                        self.sub_curvatures_callback, 10 )
        self.sub_entrypoint = self.create_subscription( Point, 'state/skin_entry', self.sub_entrypoint_callback, 10 )
        self.sub_needlepose = self.create_subscription( Pose, '/stage/state/needle_pose', self.sub_needlepose_callback, 10 )

        # create timers
        self.pub_shape_timer = self.create_timer( 0.05, self.publish_shape )

    # __init__

    def __transform( self, pmat: np.ndarray, Rmat: np.ndarray ):
        """ Transforms the needle pose of an N-D array using the current needle pose
        
            :param pmat: numpy array of N x 3 size.
            :param Rmat: numpy array of orientations of size N x 3 x 3 
            
            :returns: pmat transformed by current needle pose, Rmat transformed by current needle pose
        
        """

        current_p, current_R = self.current_needle_pose

        # self.get_logger().debug(
        #         f"__transform: pmat: {pmat.shape}, Rmat: {Rmat.shape}, p: {current_p.shape}, R:{current_R.shape}" )

        # rigid body transform the current needle pose
        if pmat is not None:
            pmat_tf = pmat @ current_R.T + current_p.reshape( 1, -1 )
        else:
            pmat_tf = None

        if Rmat is not None:
            Rmat_tf = np.einsum( 'jk, ikl -> ijl', current_R, Rmat )
        else:
            Rmat_tf = None

        return pmat_tf, Rmat_tf

    # __transform

    def get_needleshape( self ):
        """ Get the current needle shape"""
        # TODO: incorporate rotation while inserted into tissue

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

        # generate the straight length section
        dL = self.ss_needle.length - self.ss_needle.current_depth
        if dL > self.ss_needle.ds and pmat is not None and Rmat is not None:
            # shift pmat z-coordinate by dL
            pmat[ :, 2 ] += dL

            # generate other needle lengths in ds increments
            L_straight = np.arange( 0, (dL // self.ss_needle.ds + 1) * self.ss_needle.ds, self.ss_needle.ds )

            # generate straight needle poses
            pmat_straight = np.hstack( (np.zeros( (len( L_straight ), 2) ), L_straight.reshape( -1, 1 )) )
            # Rmat_straight = Rmat[ 0:1 ].repeat( len( L_straight ), axis=0 )  # repeat orientation
            Rmat_straight = np.eye( 3 )[ np.newaxis ].repeat( len( L_straight ), axis=0 )  # straight needle

            # append the the current pmat and Rmat
            pmat = np.vstack( (pmat_straight, pmat) )
            Rmat = np.concatenate( (Rmat_straight, Rmat), 0 )


        # if
        elif dL > 0 and pmat is not None and Rmat is not None:
            pmat[ :, 2 ] += dL  # move base point
            # Rmat_straight = Rmat[0:1] # copy first orientation
            Rmat_straight = np.eye( 3 )[ np.newaxis ]  # straight orientation

            pmat = np.vstack( (np.zeros( 3 ), pmat) )  # append the 0 point
            Rmat = np.concatenate( (Rmat_straight, Rmat), axis=0 )  # Copy first orientation

        # elif

        # transform the current needle pose
        pmat, Rmat = self.__transform( pmat, Rmat )

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
            # self.get_logger().warn( f"pmat or Rmat is None: pmat={pmat}, Rmat={Rmat}" )
            # self.get_logger().warn( f"Current shapetype: {self.ss_needle.current_shapetype}" )
            return

        # if

        # needle shape length
        needle_L = np.linalg.norm( np.diff( pmat, axis=0 ), 2, 1 ).sum()
        self.get_logger().debug(
                f"Needle L: {self.ss_needle.length} | Needle Shape L: {needle_L} | Current Depth: {self.ss_needle.current_depth}" )

        # generate pose message
        header = Header( stamp=self.get_clock().now().to_msg(), frame_id='robot' )
        msg_shape = utilities.poses2msg( pmat, Rmat, header=header )

        # generate kappa_c and w_init message
        msg_kc = Float64MultiArray( data=self.kc_i )
        msg_winit = Float64MultiArray( data=self.w_init_i.tolist() )

        self.get_logger().debug( f"Shapes: {pmat.shape}, {Rmat.shape}, {len( msg_shape.poses )}" )

        # publish the messages
        self.pub_shape.publish( msg_shape )
        self.pub_kc.publish( msg_kc )
        self.pub_winit.publish( msg_winit )

    # publish_shape

    def sub_curvatures_callback( self, msg: Float64MultiArray ):
        """ Subscription to needle sensor curvatures """
        # grab the current curvatures
        curvatures = np.reshape( msg.data, (-1, 2), order='F' ).T

        # update the curvatures
        self.ss_needle.current_curvatures = curvatures

        if not self.ss_needle.is_calibrated:
            self.ss_needle.ref_wavelengths = np.ones_like( self.ss_needle.ref_wavelengths )

        # if

    # sub_curvatures_callback

    def sub_entrypoint_callback( self, msg: Point ):
        """ Subscription to entrypoint topic """
        # TODO: update current needle insertion depth
        self.current_insertion_pt = np.array( [ msg.x, msg.y, msg.z ] )

    # sub_entrypoint_callback

    def sub_needlepose_callback( self, msg: Pose ):
        """ Subscription to entrypoint topic """
        self.current_needle_pose = list( utilities.msg2pose( msg ) )
        # self.get_logger().info(f"pose[0]: {self.current_needle_pose[0]}")
        # self.get_logger().info(f"pose[1]: {self.current_needle_pose[1]}")
        self.current_needle_pose[ 1 ] = self.current_needle_pose[ 1 ] @ self.R_NEEDLEPOSE  # update current needle pose

        # update the insertion depth (y-coordinate is the insertion depth)
        self.ss_needle.current_depth = min( self.current_needle_pose[ 0 ][ 1 ], self.ss_needle.length )
        self.get_logger().debug( f"Current insertion depth: {self.ss_needle.current_depth}" )

        # update the history of orientations
        depth_ds = msg.position.y - msg.position.y % self.ss_needle.ds
        theta = msg.orientation.y
        if np.any( self.history_needle_pose[ 0 ] == depth_ds ):  # check if we already have this value
            idx = np.argwhere( self.history_needle_pose[ 0 ] == depth_ds ).ravel()
            self.history_needle_pose[ 1, idx ] = theta

        # if
        else:  # add a new value
            np.hstack( (self.history_needle_pose, [ [ depth_ds ], [ theta ] ]) )

        # else

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
