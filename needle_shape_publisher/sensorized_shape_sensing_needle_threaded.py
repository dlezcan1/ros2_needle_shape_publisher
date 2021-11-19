import multiprocessing

import rclpy
from rclpy.executors import MultiThreadedExecutor

from .sensorized_needle import SensorizedNeedleNode
from .shape_sensing_needle import ShapeSensingNeedleNode

def main(args=None):
    rclpy.init(args=args)

    # configure executor and nodes
    executor = MultiThreadedExecutor(num_threads=multiprocessing.cpu_count())

    sensorized_node = SensorizedNeedleNode()
    ssneedle_node = ShapeSensingNeedleNode()

    executor.add_node(sensorized_node)
    executor.add_node(ssneedle_node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    # clean-up
    executor.shutdown()
    sensorized_node.destroy_node()
    ssneedle_node.destroy_node()

    rclpy.shutdown()

# main


if __name__ == "__main__":
    main()

# if __main__