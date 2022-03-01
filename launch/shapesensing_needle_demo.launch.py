from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # arguments
    arg_needleparam = DeclareLaunchArgument( 'needleParamFile',
                                             description="The shape-sensing needle parameter json file." )
    arg_needledatadir = DeclareLaunchArgument( 'needleDemoDataDir',
                                               description="The demo data directory for the needle shape publishing" )
    arg_numsignals = DeclareLaunchArgument( 'numSignals', description="The number of FBG signals to collect.",
                                            default_value="200" )
    arg_optim_maxiter = DeclareLaunchArgument( 'optimMaxIterations', default_value="15",
                                               description="The maximum number of iterations for needle shape optimizer." )

    # Nodes
    node_ssneedledemo = Node(
            package='needle_shape_publisher',
            namespace='needle',
            executable='shapesensing_needle_demo',
            output='screen',
            emulate_tty=True,
            parameters=[ {
                    'needle.paramFile'               : LaunchConfiguration( 'needleParamFile' ),
                    'needle.optimizer.max_iterations': LaunchConfiguration( 'optimMaxIterations' ),
                    'needle.demoDataDir'             : LaunchConfiguration( 'needleDemoDataDir' )
                    } ]
            )

    # add to launch description
    ld.add_action( arg_needleparam )
    ld.add_action( arg_needledatadir )
    ld.add_action( arg_numsignals )
    ld.add_action( arg_optim_maxiter )

    ld.add_action( node_ssneedledemo )

    return ld

# generate_launch_description
