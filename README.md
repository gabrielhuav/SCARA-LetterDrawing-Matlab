# SCARA-LetterDrawing-Matlab

This repository contains the Matlab code for controlling a SCARA (Selective Compliant Assembly Robot Arm) robot to draw letters using inverse kinematics. The project demonstrates the practical application of inverse kinematics in controlling SCARA robots to perform precise and complex tasks such as drawing and writing.

## Abstract

The project describes the development of a SCARA robot capable of drawing letters using inverse kinematics in Matlab with the Robotics Toolbox library by Peter Corke. The SCARA robot, known for its high speed and precision, is commonly used in assembly and material handling applications. In this study, its potential to perform precise and repetitive drawing tasks is explored by implementing two methods to calculate the joint configurations necessary for the robot's end effector to reach a desired position and orientation: the geometric method and the Denavit-Hartenberg (DH) method.

## Introduction

SCARA robots are widely used in the industry for their precision, speed, and versatility in tasks such as assembly, Pick & Place, and material handling. Their compact and lightweight design makes them ideal for applications that require fast and repetitive movements in a limited workspace.

## Kinematics of Robots

Robotics kinematics refers to the study of robot motion without considering the forces and moments involved. It is divided into two main branches:

- Direct kinematics: Calculates the position and orientation of the end effector from the joint values (angles or distances).
- Inverse kinematics: Determines the angular values of the joints necessary for the end effector to reach a desired position and orientation.

## Design and Implementation

The implementation of these methods in Matlab allows the robot to successfully draw letters. The results demonstrate the effectiveness of inverse kinematics in programming robots to perform complex and precise tasks, thus contributing to advances in the field of robotics by demonstrating the practical application of inverse kinematics in controlling SCARA robots to perform precise drawing and writing tasks.

## Screenshots

![G0](https://github.com/gabrielhuav/SCARA-LetterDrawing-Matlab/assets/71236850/7f5518e4-e88c-4373-b6a1-aae1b6b4862b)
![G2](https://github.com/gabrielhuav/SCARA-LetterDrawing-Matlab/assets/71236850/83decd12-6b88-411a-a3d9-62c45c181d86)
![G4](https://github.com/gabrielhuav/SCARA-LetterDrawing-Matlab/assets/71236850/8bfefb67-cc78-4a40-b590-e0205d0503f7)
![GH0](https://github.com/gabrielhuav/SCARA-LetterDrawing-Matlab/assets/71236850/ee68d78a-9030-42f5-bd0d-ab81636019df)
![GHA2](https://github.com/gabrielhuav/SCARA-LetterDrawing-Matlab/assets/71236850/e77926fb-2dfe-4dea-b4bc-dce882d5e2d8)
![GHA3](https://github.com/gabrielhuav/SCARA-LetterDrawing-Matlab/assets/71236850/c87936a0-3f22-4771-b036-c1d537dab44a)
![GHA4](https://github.com/gabrielhuav/SCARA-LetterDrawing-Matlab/assets/71236850/72dbe9cf-5120-4d45-9ab1-82aeb94194d1)


## Installation

To run the code in this repository, you need to install the Robotics Toolbox for MATLAB. Here are the steps for local installation:

1. [Download the latest build](https://petercorke.com/toolboxes/robotics-toolbox/).
2. From within the MATLAB file browser double click on each file, it will install and configure the paths correctly.
3. Run `rtbdemo` in MATLAB.

Note: The toolbox can also be run on MATLAB Drive.

## Usage

The Matlab code can be run directly in any Matlab environment that has the Robotics Toolbox installed. The main script controls the SCARA robot to draw letters based on predefined trajectories.

## Contributing

Contributions are welcome! Please feel free to submit a pull request or open an issue. I also challenge anyone to use the [Python version of Peter Corke's Robotics Toolbox](https://github.com/petercorke/robotics-toolbox-python/) and contribute to this project.

![image](https://github.com/gabrielhuav/SCARA-LetterDrawing-Matlab/assets/71236850/f059ba1c-4919-4fd7-a2e7-5ab6bcc4015b)
