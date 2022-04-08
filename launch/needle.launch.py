import sys, os
import json
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

pkg_needle_shape_publisher = get_package_share_directory('needle_shape_publisher')

def determineCHsAAs(needleParamFile: str):
    """ Determine the number of channels and active areas available """
    with open(needleParamFile, 'r') as paramFile:
        params = json.load(paramFile) 

    # with
    numChs = params['# channels']
    numAAs = params['# active areas']

    return numChs, numAAs

# determineCHsAAs

def generate_launch_description():
    ld = LaunchDescription()

    # determine #chs and numAAs
    numCHs, numAAs = None, None
    default_needleparam_file = "needle_params_2021-08-16_Jig-Calibration_best.json"
    for arg in sys.argv:
        if arg.startswith("needleParamFile:="):
            needleParamFile = arg.split(":=")[1]
            break
        # if        
    # for 

    if numCHs is None and numAAs is None: # just in-case using default value
        needleParamFile = default_needleparam_file

    numCHs, numAAs = determineCHsAAs(os.path.join(pkg_needle_shape_publisher, "needle_data", needleParamFile))

    # arguments
    arg_params = DeclareLaunchArgument( 'needleParamFile',
                                        default_value=default_needleparam_file,
                                        description="The shape-sensing needle parameter json file." )

    arg_numsignals = DeclareLaunchArgument( 'numSignals', description="The number of FBG signals to collect.",
                                            default_value="200" )

    arg_optim_maxiter = DeclareLaunchArgument( 'optimMaxIterations', default_value="15",
                                               description="The maximum number of iterations for needle shape optimizer." )

    # included launch arguments
    ld_needlepub = IncludeLaunchDescription( # needle shape publisher
             PythonLaunchDescriptionSource(
                os.path.join(pkg_needle_shape_publisher, 'sensorized_shapesensing_needle_decomposed.launch.py')),
                launch_arguments = {
                    'needleParamFile'   : PathJoinSubstitution( pkg_needle_shape_publisher, "needle_data", LaunchConfiguration( 'needleParamFile')),
                    'numSignals'        : LaunchConfiguration('numSignals'),
                    'optimMaxIterations': LaunchConfiguration('optimMaxIterations'),
                    }.items()
            )
    # configure launch description
    ld.add_action(arg_params)
    ld.add_action(arg_numsignals)
    ld.add_action(arg_optim_maxiter)

    ld.add_action(ld_needlepub)

    return ld

# generate_launch_descrtiption