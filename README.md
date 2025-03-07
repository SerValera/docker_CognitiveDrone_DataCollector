# CognitiveDrone: A VLA Model and Evaluation Benchmark for Real-Time Cognitive Task Solving and Reasoning in UAVs 

This paper introduces CognitiveDrone, a novel Vision-Language-Action (VLA) model tailored for complex Unmanned Aerial Vehicles (UAVs) tasks that demand advanced cognitive abilities. Trained on a dataset comprising over 8,000 simulated flight trajectories across three key categories-Human Recognition, Symbol Understanding, and Reasoning-the model generates real-time 4D action commands based on first-person visual inputs and textual instructions. To further enhance performance in intricate scenarios, we propose CognitiveDrone-R1, which integrates an additional Vision-Language Model (VLM) reasoning module to simplify task directives prior to high-frequency control. Experimental evaluations using our open-source benchmark, CognitiveDroneBench, reveal that while a racing-oriented model (RaceVLA) achieves an overall success rate of 31.3%, the base CognitiveDrone model reaches 59.6%, and CognitiveDrone-R1 attains a success rate of 77.2%. These results demonstrate improvements of up to 30% in critical cognitive tasks, underscoring the effectiveness of incorporating advanced reasoning capabilities into UAV control systems. Our contributions include the development of a state-of-the-art VLA model for UAV control and the introduction of the first dedicated benchmark for assessing cognitive tasks in drone operations. The complete repository is available at https://cognitivedrone.github.io/ 

## CognitiveDrone Dataset Docker Collector

The collector uses a Docker container that integrates ROS1, ArduPilot, Gazebo Classic and the Realsense plugin to collect and process data for training and testing drone models.

## Setup Instructions

[Setup Instructions](0_docker.md)

You can also refer to the related paper on [arXiv](https://arxiv.org/abs/2503.01378).
