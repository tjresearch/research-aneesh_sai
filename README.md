### Training Self-driving Cars with a Genetic Algorithm

### Overview
The final product will be training self-driving cars to drive based off of a camera input, and they'll be trained using a genetic algorithm. Each car will be controlled by a neural net that will take the distances from a stereovision algorithm as an input.

### Requirements to Run
OpenCV for the stereovision algorithm
Unity for the actual simulation where the cars will be trained
Once the project is finished, there will be executable versions of both components that shouldn't require any dependencies.

### Installation instructions
First, install [OpenCV] (https://docs.opencv.org/2.4/doc/tutorials/introduction/windows_install/windows_install.html).
For stereovision, [follow this link](https://docs.opencv.org/2.4/doc/tutorials/introduction/linux_eclipse/linux_eclipse.html).  
[This tutorial](https://docs.unity3d.com/2017.2/Documentation/Manual/InstallingUnity.html) shows how to install Unity.

### Run Instructions 
The Unity project is currently not built. This can be done after opening it in Unity and going to the Build menu. After that, there should be an executable that can be directly run.

For the stereovision C++ code, use Eclipse CDT, and after g++ and the linker are set up correctly, it can be run directly through Eclipse, which will handle both the build and run commands.

### What this repo contains
As of right now, a lot of different code. However, most useful code is currently in the stereovision_cpp directory. There is also a Unity project that was used to take the training and test images, I cleaned up the root directory a bit, but there still might be some extraneous code.

### Stereovision_cpp
This folder contains the calibration right now, and will also have the code for generating the pointcloud. The code is written in C++ and requires OpenCV to run properly, which can be done relatively easily with Eclipse and the Eclipse CDT Plugin. OpenCV must also be installed and linked properly through Eclipse, but once that is completed, Eclipse can handle the build and run commands for you.
