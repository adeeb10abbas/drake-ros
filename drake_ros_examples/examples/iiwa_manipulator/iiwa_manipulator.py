#!/usr/bin/env python3
import argparse

import numpy as np
import cv2
import threading

from time import sleep
import drake_ros.core
from drake_ros.core import ClockSystem
from drake_ros.core import RosInterfaceSystem
from drake_ros.viz import RvizVisualizer

from pydrake.examples import ManipulationStation
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import Adder
from pydrake.systems.primitives import ConstantVectorSource
from pydrake.systems.primitives import Sine
import pydrake.systems.sensors

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--simulation_sec',
        type=float,
        default=float('inf'),
        help='How many seconds to run the simulation')
    args = parser.parse_args()

    builder = DiagramBuilder()

    drake_ros.core.init()
    ros_interface_system = builder.AddSystem(RosInterfaceSystem("iiwa_manipulator_node"))
    ClockSystem.AddToBuilder(builder, ros_interface_system.get_ros_interface())

    manipulation_station = builder.AddSystem(ManipulationStation())
    manipulation_station.SetupClutterClearingStation()
    manipulation_station.Finalize()


    # Make the base joint swing sinusoidally.
    constant_term = builder.AddSystem(ConstantVectorSource(
        np.zeros(manipulation_station.num_iiwa_joints())))

    amplitudes = np.zeros(manipulation_station.num_iiwa_joints())
    amplitudes[0] = np.pi / 4.  # == 45 degrees
    frequencies = np.ones(manipulation_station.num_iiwa_joints())
    phases = np.zeros(manipulation_station.num_iiwa_joints())
    variable_term = builder.AddSystem(Sine(amplitudes, frequencies, phases))

    joint_trajectory_generator = builder.AddSystem(
        Adder(2, manipulation_station.num_iiwa_joints()))

    builder.Connect(constant_term.get_output_port(),
                    joint_trajectory_generator.get_input_port(0))
    builder.Connect(variable_term.get_output_port(0),
                    joint_trajectory_generator.get_input_port(1))
    builder.Connect(joint_trajectory_generator.get_output_port(),
                    manipulation_station.GetInputPort('iiwa_position'))

    rviz_visualizer = builder.AddSystem(
        RvizVisualizer(ros_interface_system.get_ros_interface()))

    rviz_visualizer.RegisterMultibodyPlant(
        manipulation_station.get_multibody_plant())
    rviz_visualizer.ComputeFrameHierarchy()

    builder.Connect(
        manipulation_station.GetOutputPort('query_object'),
        rviz_visualizer.get_graph_query_input_port()
    )

    diagram = builder.Build()

    simulator = Simulator(diagram)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)
    simulator_context = simulator.get_mutable_context()

    default_context = manipulation_station.CreateDefaultContext()
    manipulation_station_context = \
        diagram.GetMutableSubsystemContext(
            manipulation_station, simulator_context)
    constant_term_context = \
        diagram.GetMutableSubsystemContext(
            constant_term, simulator_context)

    # Fix gripper joints' position.
    manipulation_station.GetInputPort('wsg_position').FixValue(
        manipulation_station_context, np.zeros(1))
    
    # Use default positions for every joint but the base joint.
    constants = constant_term.get_mutable_source_value(constant_term_context)
    constants.set_value(
        manipulation_station.GetIiwaPosition(manipulation_station_context))
    constants.get_mutable_value()[0] = -np.pi / 4.

## Getting the camera output - 

    # cv2.namedWindow('Live Image', cv2.WINDOW_NORMAL)
    
    # # Check if images are available
    # if manipulation_station.GetOutputPort('camera_0_rgb_image').Eval(default_context).size() > 0:
    #     image = manipulation_station.GetOutputPort('camera_0_rgb_image').Eval(default_context)
    #     image_data = image.data
    #     # image_data.shape = (480, 848, 4)
    #     # image_data.dtype = uint8
    #     # Display the image

    #     cv2.imshow('Live Image', image_data)
        
    #     # Use waitKey to introduce a delay. 1 means 1ms.
    #     # It also allows OpenCV to process GUI events.
    #     cv2.waitKey(1)


    # ##
    # print("Starting simulation...")
    # # Step the simulator in 0.1s intervals
    # step = 0.1
    # while simulator_context.get_time() < args.simulation_sec:
    #     next_time = min(
    #         simulator_context.get_time() + step, args.simulation_sec,
    #     )
    #     simulator.AdvanceTo(next_time)
    
    cv2.namedWindow('Live Image', cv2.WINDOW_NORMAL)

    # Function to display camera image
    def display_camera():
        while True:
            if manipulation_station.GetOutputPort('camera_0_rgb_image').Eval(default_context).size() > 0:
                image = manipulation_station.GetOutputPort('camera_0_rgb_image').Eval(default_context)
                image_data = image.data
                print(image_data.shape)
                print(image_data)
                cv2.imshow('Live Image', image_data)
                cv2.waitKey(1)

    # Function to run the simulation loop
    def simulation_loop():
        print("Starting simulation...")
        step = 0.1
        while simulator_context.get_time() < args.simulation_sec:
            next_time = min(
                simulator_context.get_time() + step, args.simulation_sec,
            )
            simulator.AdvanceTo(next_time)

    # Create threads
    camera_thread = threading.Thread(target=display_camera)
    simulation_thread = threading.Thread(target=simulation_loop)

    # Start threads
    camera_thread.start()
    simulation_thread.start()

    # Wait for both threads to finish
    camera_thread.join()
    simulation_thread.join()

    # Cleanup
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
