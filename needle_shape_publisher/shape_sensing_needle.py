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


class ShapeSensingNeedleNode( Node ):
    # PARAMETER NAMES
    PARAM_NEEDLE = "needle"

    # - needle parameters
    PARAM_NEEDLEPARAMFILE = ".".join( [ PARAM_NEEDLE, "paramFile" ] )  # needle parameter json file
    PARAM_NEEDLESN = ".".join( [ PARAM_NEEDLE, "serialNumber" ] )  # needle serial number
    PARAM_NEEDLELENGTH = ".".join( [ PARAM_NEEDLE, "length" ] )  # needle length
    PARAM_CHS = ".".join( [ PARAM_NEEDLE, "channels" ] )  # needle number of channels
    PARAM_AAS = ".".join( [ PARAM_NEEDLE, "activeAreas" ] )  # needle number of active areas
    PARAM_AAWEIGHTS = ".".join( [ PARAM_AAS, "weights" ] )  # needle AA reliability weightings
    PARAM_SLOCS = ".".join( [ PARAM_AAS, 'locations' ] )  # needle AA locations from tip of the needle
    PARAM_NEEDLESHAPE = ".".join( (PARAM_NEEDLE, "shape_type") )  # unused, needle shape type

    # - FBG signal parameters
    PARAM_NUMSIGNALS = ".".join( [ PARAM_NEEDLE, "sensor.numberSignals" ] )  # number of signals to gather for sampling

    # - optimization options
    PARAM_OPTIMIZER = ".".join( [ PARAM_NEEDLE, 'optimizer' ] )
    PARAM_KCINIT = ".".join( [ PARAM_OPTIMIZER, 'initial_kappa_c' ] )  # initial kappa_c for optimization
    PARAM_WINIT = ".".join( [ PARAM_OPTIMIZER, 'initial_w_init' ] )  # initial omega_init for optimization
    PARAM_OPTIM_MAXITER = ".".join( [ PARAM_OPTIMIZER, 'max_iterations' ] )
    PARAM_OPTIM_MAXITER_LB = 2

    def __init__( self, name="ShapeSensingNeedle" ):
        super().__init__( name )

        # declare and get parameters
        pd_needleparam = ParameterDescriptor( name=self.PARAM_NEEDLEPARAMFILE, type=Parameter.Type.STRING.value,
                                              description='needle parameter json file.', read_only=True )
        needleparam_file = self.declare_parameter( pd_needleparam.name, descriptor=pd_needleparam ). \
            get_parameter_value().string_value

        pd_numsamples = ParameterDescriptor( name=self.PARAM_NUMSIGNALS, type=Parameter.Type.INTEGER.value )
        self.num_samples = self.declare_parameter( pd_numsamples.name, descriptor=pd_numsamples,
                                                   value=200 ).get_parameter_value().integer_value

        pd_kc = ParameterDescriptor( name=self.PARAM_KCINIT, type=Parameter.Type.DOUBLE_ARRAY.value,
                                     description="kappa_c initalization values" )
        self.kc_i = self.declare_parameter( pd_kc.name, descriptor=pd_kc,
                                            value=[ 0.002 ] ).get_parameter_value().double_array_value

        pd_winit = ParameterDescriptor( name=self.PARAM_WINIT, type=Parameter.Type.DOUBLE_ARRAY.value,
                                        description="omega_init initialization values (3D vector)" )
        self.w_init_i = np.array( self.declare_parameter( pd_winit.name, descriptor=pd_winit,
                                                          value=[ self.kc_i[ 0 ], 0.0, 0.0 ]
                                                          ).get_parameter_value().double_array_value )

        pd_optim_maxiter = ParameterDescriptor( name=self.PARAM_OPTIM_MAXITER, type=Parameter.Type.INTEGER.value,
                                                description="Maximum iterations for convergence" )
        optim_maxiter = self.declare_parameter( pd_optim_maxiter.name, value=15,
                                                descriptor=pd_optim_maxiter ).get_parameter_value().integer_value

        # get shape-sensing FBG Needle object
        try:
            self.ss_needle = ShapeSensingFBGNeedle.load_json( needleparam_file )

            # make all positive since we are using processed wavelengths
            self.ss_needle.ref_wavelengths = np.ones_like( self.ss_needle.ref_wavelengths )
            self.ss_needle.current_depth = 130  # set the current depth # TODO: needle pose subscriber
            self.ss_needle.optimizer.options[ 'options' ] = { 'maxiter': optim_maxiter }
            #
            # container of wavelengths
            self.__wavelength_container = np.zeros( (self.num_samples,
                                                     self.ss_needle.num_channels * self.ss_needle.num_activeAreas) )
            self.__wavelength_container_idx = 0  # the row index to place the sample
            self.__wavelength_container_full = False  # whether we have collected a full number of samples

            self.get_logger().info( "Successfully loaded FBG Needle: \n" + str( self.ss_needle ) )

        # try
        except Exception as e:
            self.get_logger().error( f"{needleparam_file} is not a valid needle parameter file." )
            raise e

        # except

        # set (read-only) needle parameters
        pd_ndllen = ParameterDescriptor( name=self.PARAM_NEEDLELENGTH, type=Parameter.Type.DOUBLE.value,
                                         description="The length of the needle.", read_only=True )
        pd_ndlsn = ParameterDescriptor( name=self.PARAM_NEEDLESN, type=Parameter.Type.STRING.value,
                                        description="The serial number of the shape-sensing needle", read_only=True )
        pd_numchs = ParameterDescriptor( name=self.PARAM_CHS, type=Parameter.Type.INTEGER.value,
                                         description="Number of Channels in the FBG-sensorized needle", read_only=True )
        pd_numaas = ParameterDescriptor( name=self.PARAM_AAS, type=Parameter.Type.INTEGER.value,
                                         description="Number of Activa Areas in the FBG-sensorized needle",
                                         read_only=True )
        pd_slocs = ParameterDescriptor( name=self.PARAM_SLOCS, type=Parameter.Type.DOUBLE_ARRAY.value,
                                        description="Location of the active areas in (mm) from the tip of the needle",
                                        read_only=True )
        # - AA reliability weightings
        if len( self.ss_needle.weights ) > 0:
            w = list( self.ss_needle.weights.values() )
        else:
            w = (np.ones( len( self.ss_needle.num_activeAreas ) ) / self.ss_needle.num_activeAreas).tolist()

        pd_aawgts = ParameterDescriptor( name=self.PARAM_AAWEIGHTS, type=Parameter.Type.DOUBLE_ARRAY.value,
                                         description="Active Area reliability weightings", read_only=True )

        # - declarations
        self.declare_parameter( self.PARAM_NEEDLESN, descriptor=pd_ndlsn, value=self.ss_needle.serial_number )
        self.declare_parameter( self.PARAM_NEEDLELENGTH, descriptor=pd_ndllen, value=self.ss_needle.length )
        self.declare_parameter( self.PARAM_CHS, descriptor=pd_numchs, value=self.ss_needle.num_channels )
        self.declare_parameter( self.PARAM_AAS, descriptor=pd_numaas, value=self.ss_needle.num_activeAreas )
        self.declare_parameter( self.PARAM_SLOCS, descriptor=pd_slocs, value=self.ss_needle.sensor_location_tip )
        self.declare_parameter( self.PARAM_AAWEIGHTS, descriptor=pd_aawgts, value=w )

        # create publishers
        self.pub_shape = self.create_publisher( PoseArray, 'state/shape', 1 )
        self.pub_curvatures = self.create_publisher( Float64MultiArray, 'state/curvatures', 10 )
        self.pub_kc = self.create_publisher( Float64MultiArray, 'state/kappac', 1 )
        self.pub_winit = self.create_publisher( Float64MultiArray, 'state/winit', 1 )

        # create subscriptions
        self.sub_signals = self.create_subscription( Float64MultiArray, 'sensor/processed', self.sub_signals_callback,
                                                     10 )
        self.sub_entrypoint = self.create_subscription( Point, 'state/skin_entry', self.sub_entrypoint_callback,
                                                        10 )
        self.sub_needlepose = self.create_subscription( Pose, 'state/pose', self.sub_needlepose_callback,
                                                        10 )

        # create services
        # TODO: update needle shape type service

        # create timers
        shape_timer_period = 3 # seconds
        curv_timer_period = 0.01  # seconds
        self.pub_shape_timer = self.create_timer( shape_timer_period, self.publish_shape )
        self.pub_curvatures_timer = self.create_timer( curv_timer_period, self.publish_curvatures )

        # set parameter callback function
        self.add_on_set_parameters_callback( self.parameters_callback )

    # __init__

    def destroy_node( self ) -> bool:
        """ Destroy the node override"""
        self.get_logger().info( "Shutting down..." )
        retval = super().destroy_node()
        self.get_logger().info( "Shut down complete." )
        return retval

    # destroy_node

    def parameters_callback( self, parameters: List[ Parameter ] ):
        """ Parameter set calllbacks"""
        successful = True
        reasons = [ ]
        for param in parameters:
            if param.name == self.PARAM_NEEDLEPARAMFILE:  # read-only, but kep just in case.
                # grab old parameters
                shapetype = self.ss_needle.current_shapetype
                insertion_params = self.ss_needle.insertion_parameters

                # load the new FBG needle
                try:
                    ss_needle = ShapeSensingFBGNeedle.load_json( param.get_parameter_value().string_value )

                    # update new needle and its parameters
                    self.ss_needle = ss_needle
                    self.ss_needle.ref_wavelengths = np.ones_like(
                            self.ss_needle.ref_wavelengths )  # proc'd signal input
                    self.ss_needle.__current_shapetype = shapetype
                    self.ss_needle.insertion_parameters = insertion_params

                    # logging
                    self.get_logger().info( "Loaded new Shape-Sensing FBG needle\n" + str( self.ss_needle ) )

                # try
                except Exception as e:
                    successful = False
                    reasons.append( "Issue setting new needle: " + str( e ) )

                # except
            # if: parameter: NEEDLE_PARAMFILE

            elif param.name == self.PARAM_NUMSIGNALS:
                num_samples = param.get_parameter_value().integer_value
                try:
                    self.get_logger().info( f"Updating number of signals to {num_samples}..." )
                    self.update_numsamples( num_samples )
                    self.get_logger().info( f"Updated number of signals to {num_samples}." )

                # try
                except ValueError as e:
                    successful = False
                    reasons.append( f"{self.PARAM_NUMSIGNALS} must be > 0" + '\n' + str( e ) )
                    self.get_logger().error( "Update failed. Did not set the updated number of signals." )

                # except
            # elif: parameter: NUMSIGNALS

            elif param.name == self.PARAM_OPTIM_MAXITER:
                optim_maxiter = param.get_parameter_value().integer_value
                if optim_maxiter < self.PARAM_OPTIM_MAXITER_LB:
                    successful = False
                    reasons.append( f"{self.PARAM_OPTIM_MAXITER} must be >= {self.PARAM_OPTIM_MAXITER_LB}" )

                # if
                self.ss_needle.optimizer.options[ 'options' ] = { 'maxiter': optim_maxiter }

            # elif
        # for

        return SetParametersResult( successful=successful, reason="\n".join( reasons ) )

    # parameters_callback

    def publish_curvatures( self ):
        """ Publish the curvatures of the shape-sensing needle"""
        # current_curvatures are N x 2 ( columns are: x,  y ) -> ravel('F') -> (X_AA1, X_AA2, ..., Y_AA1, Y_AA2,...)
        itemsize = self.ss_needle.current_curvatures.dtype.itemsize
        dimx = MultiArrayDimension( label="x", stride=itemsize,
                                    size=self.ss_needle.current_curvatures.shape[ 0 ] * itemsize )
        dimy = MultiArrayDimension( label="y", stride=itemsize,
                                    size=self.ss_needle.current_curvatures.shape[ 0 ] * itemsize )

        msg = Float64MultiArray( data=self.ss_needle.current_curvatures.ravel( order='F' ).tolist(),
                                 layout=MultiArrayLayout( dim=[ dimx, dimy ] ) )

        self.pub_curvatures.publish(msg)

    # publish_curvatures

    def publish_shape( self ):
        """ Publish the 3D needle shape"""
        if self.__wavelength_container_full:
            # get needle shape
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
                return  # pop out of the loop and redo

            # else

            # update initial kappa_c values
            self.kc_i = self.ss_needle.current_kc
            self.w_init_i = self.ss_needle.current_winit

            # check to make sure messages are not None
            if pmat is None or Rmat is None:
                self.get_logger().warn( f"pmat or Rmat is None: {pmat}, {Rmat}" )
                self.get_logger().warn( f"Current shapetype: {self.ss_needle.current_shapetype}" )
                assert (self.ss_needle.current_shapetype == NEEDLESHAPETYPE.SINGLEBEND_SINGLELAYER)
                return

            # generate pose message
            header = Header( stamp=self.get_clock().now().to_msg() )
            msg_shape = self.poses2msg( pmat, Rmat, header=header )

            # generate kappa_c and w_init message
            msg_kc = Float64MultiArray( data=self.kc_i )
            msg_winit = Float64MultiArray( data=self.w_init_i.tolist() )

            # publish the messages
            self.pub_shape.publish( msg_shape )
            self.pub_kc.publish( msg_kc )
            self.pub_winit.publish( msg_winit )

        # if

    # publish_shape

    @staticmethod
    def pose2msg( pos: np.ndarray, R: np.ndarray ):
        """ Turn a pose into a Pose message """
        msg = Pose()

        # handle position
        msg.position.x = pos[ 0 ]
        msg.position.y = pos[ 1 ]
        msg.position.z = pos[ 2 ]

        # handle orientation
        quat = geometry.rotm2quat( R )
        msg.orientation.w = quat[ 0 ]
        msg.orientation.x = quat[ 1 ]
        msg.orientation.y = quat[ 2 ]
        msg.orientation.z = quat[ 3 ]

        return msg

    # pose2msg

    @staticmethod
    def poses2msg( pmat: np.ndarray, Rmat: np.ndarray, header: Header = Header() ):
        """ Turn a sequence of poses into a PoseArray message"""
        # determine number of elements in poses
        N = min( pmat.shape[ 0 ], Rmat.shape[ 0 ] )

        # generate the message and add the individual poses
        msg = PoseArray( header=header )
        for i in range( N ):
            msg.poses.append( ShapeSensingNeedleNode.pose2msg( pmat[ i ], Rmat[ i ] ) )

        # for

        return msg

    # poses2msg

    def set_needleparameters( self ):
        """ Set all of the needle parameters"""
        needle_params = [ Parameter( self.PARAM_SLOCS, self.ss_needle.sensor_location_tip ),
                          Parameter( self.PARAM_CHS, self.ss_needle.num_channels ),
                          Parameter( self.PARAM_NEEDLELENGTH, self.ss_needle.length ),
                          Parameter( self.PARAM_NEEDLESN, self.ss_needle.serial_number ),
                          Parameter( self.PARAM_AAS, self.ss_needle.num_activeAreas ) ]
        if len( self.ss_needle.weights ) > 0:
            weights = list( self.ss_needle.weights.values() )
        else:
            weights = (np.ones( len( self.ss_needle.num_activeAreas ) ) / self.ss_needle.num_activeAreas).tolist()
        needle_params.append( Parameter( self.PARAM_AAWEIGHTS, weights ) )

        try:
            results = self.set_parameters( needle_params )  # not able-to since read-only
            success = all( map( lambda x: x.successful, results ) )
            if not success:
                err_msg = "\n".join( map( lambda x: x.reason, results ) )
                self.get_logger().error( "Error setting needle parameters:" )
                self.get_logger().error( err_msg )

            # if
        # try

        except Exception as e:
            self.get_logger().error( "Error setting needle parameters:" )
            self.get_logger().error( e )
            success = False

        # except

        return success

    # set_needleparameters

    def sub_entrypoint_callback( self, msg: Point ):
        """ Call back to needle skin entry point"""
        # TODO: subscription entrypoint
        pass

    # sub_entrypoint_callback

    def sub_needlepose_callback( self, msg: Pose ):
        """ Call back to needle base pose"""
        # TODO: subscription needlepose
        pass

    # sub_needlepose_callback

    def sub_signals_callback( self, msg: Float64MultiArray ):
        """ Call back to processed signals subscription"""
        # get the FBG signals
        # TODO: perform appending by channel
        signals_dict = ShapeSensingNeedleNode.unpack_fbg_msg( msg )
        signals = np.array( list( signals_dict.values() ) ).ravel()  # to be improved
        self.get_logger().debug(
                f"Shape of signals: {signals.shape} | Shape of wl container: {self.__wavelength_container.shape}" )

        # add the signals to the container
        self.__wavelength_container[ self.__wavelength_container_idx ] = signals
        self.__wavelength_container_idx += 1
        if self.__wavelength_container_idx >= self.num_samples:
            self.__wavelength_container_full = True

        # if

        # update the wavelengths
        if self.__wavelength_container_full:
            self.__wavelength_container_idx %= self.num_samples
            self.ss_needle.update_wavelengths( self.__wavelength_container, processed=True )

        # if

    # sub_signals_callback

    def update_numsamples( self, num_samples: int ):
        """ Update the number of FBG samples """
        if num_samples <= 0:
            raise ValueError( "Number of samples must be > 0" )

        # if
        elif num_samples != self.num_samples:  # shrink the size
            # reshape the wavelength container
            if num_samples < self.num_samples:
                self.__wavelength_container = self.__wavelength_container[ :num_samples ]

            # if
            elif num_samples > self.num_samples:  # grow the size
                if self.__wavelength_container_full:
                    self.__wavelength_container = np.vstack(
                            (self.__wavelength_container[ self.__wavelength_container_idx: ],
                             self.__wavelength_container[
                             :self.__wavelength_container_idx ]) )  # reorder to be @ bottom
                    self.__wavelength_container_idx = self.num_samples - 1  # place pointer at the end

                # if

                # append to the wavelength container
                self.__wavelength_container = np.vstack( (self.__wavelength_container, np.zeros(
                        (num_samples - self.num_samples, self.__wavelength_container.shape[ 1 ]) )) )

                self.__wavelength_container_full = False

            # elif

            # update number of samples and check if the wavelength container is full (mainly for shrinking)
            self.num_samples = num_samples
            if self.__wavelength_container_idx >= self.num_samples:
                self.__wavelength_container_full = True

            # if
            self.__wavelength_container_idx %= self.num_samples

        # else

    # update_numsamples

    @staticmethod
    def unpack_fbg_msg( msg ) -> dict:
        """ Unpack Float64MultiArray into dict of numpy arrays """
        ret_val = { }
        idx_i = 0

        for dim in msg.layout.dim:
            ch_num = int( dim.label.strip( 'CH' ) )
            size = int( dim.size / dim.stride )

            ret_val[ ch_num ] = np.float64( msg.data[ idx_i:idx_i + size ] )

            idx_i += size  # increment size to next counter

        # for

        return ret_val

    # unpack_fbg_msg


# class: ShapeSensingNeedleNode


def main( args=None ):
    rclpy.init( args=args )

    ssneedle_node = ShapeSensingNeedleNode()

    try:
        rclpy.spin( ssneedle_node )

    except KeyboardInterrupt:
        pass

    # clean up
    ssneedle_node.destroy_node()
    rclpy.shutdown()


# main


if __name__ == "__main__":
    main()

# if: main
