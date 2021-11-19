import numpy as np

# ROS messages
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header

# custom packages
from needle_shape_sensing import geometry


def msg2pose( msg: Pose ):
    """ Convert a Pose message into a pose"""
    pos = np.array( [ msg.position.x, msg.position.y, msg.position.z ] )
    quat = np.array( [ msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z ] )
    R = geometry.quat2rotm( quat )

    return pos, R


# msg2pose

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

def poses2msg( pmat: np.ndarray, Rmat: np.ndarray, header: Header = Header() ):
    """ Turn a sequence of poses into a PoseArray message"""
    # determine number of elements in poses
    N = min( pmat.shape[ 0 ], Rmat.shape[ 0 ] )

    # generate the message and add the individual poses
    msg = PoseArray( header=header )
    for i in range( N ):
        msg.poses.append( pose2msg( pmat[ i ], Rmat[ i ] ) )

    # for

    return msg


# poses2msg

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
