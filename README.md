# cyberpod_sim_ros
This code was used to produce the simulation results in the IROS 2021 paper titled: "Measurement-Robust Control Barrier Functions: Certainty in Safety with Uncertainty in State" ([Arxiv link](https://arxiv.org/pdf/2104.14030.pdf)).

To install and run this package, you'll need a working ROS environment. This repo was tested using Ubuntu 18.04 and Ros Melodic. 

Clone the vision packages into your catkin_workspace's src file: 
* [**rviz_camera_stream**](https://github.com/lucasw/rviz_camera_stream.git) Rviz plugin that publishes camera streams
* [**rviz_lighting**](https://github.com/mogumbo/rviz_lighting.git) Rviz plugin that provides ambient lighting

 You'll also need the Python [**ECOS**](https://github.com/embotech/ecos-python) package which can be installed via: 
> pip install ecos

<!--
## Install OSQP_embedded 
*clone from bitbucket*
*enter directory*
mkdir build 
cd build
cmake ..
sudo make install
 
## Install ASIF++ 
*clone from bitbucket* 
*go to directory*
mkdir build
cd build
cmake ..
ccmake . 
*change osqp_embedded to true*
sudo make install
-->

## Running repo
<!--
* go to top level of catkin workspace
* 'catkin_make' whenever code changes -->
Navigate to your catkin_workspace
>cd catkin_workspace 

Make the workspace
>catkin_make

Source the setup file (this must be done in every new terminal)
> source devel/setup.bash

Adjust the epsilon value and launch the main script
> roslaunch cyberpod_sim_ros main_mrcbf.launch epsilon:=0.0

Open a new terminal, source the setup file and run the rosservice command to start the simulation:
>rosservice call /cyberpod_sim_ros/integrator/ui "cmd: 1 
data:-0"
  * The commands values are: 0 stops, 1 starts, 4 resets

## Recordings
Recordings will appear in the */bags* folder in the repo and in that folder is another readme for how to format the data.

## Relevant Portions of the Code 
* cyberpod_sim_ros/
    * include/ 
        * dynamics.hpp
    *   launch/ 
        * main_mrcbf.launch 
        * mrcbf_with_learning.launch
        * sample_state_space.launch
    * msg/ 
        * cmd.msg 
        * input.mcg
        * learning_data.msg
        * state.msg
    * src/
        * integrator_node.cpp
        * grid_state_space_node.cpp
        * sensor_node.cpp
        * controller_node.cpp
        * src_python/
            * data_logger.py
            * ecos_fitler_node.py
            * perception_node.py 
    * srv/
        * ui.srv
    * URDF/
        * cyberpod.urdf




## Key Features of Main Simulation: 
***main_marcbf.launch*** 
    launches the main simulation 

***mrcbf_with_learning.launch***
    launches the simulation with learning in the loop (this requires learning data which can be generated using the notebooks). The data couldn't be uploaded due to size. Please contact the Ryan if you would like a copy of the data. 

***dynamics.hpp*** 
    the dynamics for the cyberpod as derived from the Euler-Lagrange equations

***msg/***
    the four messages listed above are the custom messages used to relay messages between the nodes of simulation and also between the user and the system 

***srv/ui.srv***
    the service for the user to interact with the simulation

***URDF***
    the system's visual model rviz visualisations 

**Nodes:**
* integrator_node.cpp
    * serves as the main physics engine for the system, receives inputs from ecos_filter_node and outputs integrated states 
* perception_node.py 
    * recieves image data and outputs state estimates
* sensor_node.cpp
    * receives state values from integrator node state estimates from perception_node and outputs "measured states" with true state values used whenever they cannot be attained from the image, such as velocity. 
* controller_node.cpp
    * receives state estimates and output desired input based on PD feedback gains
* mrcbf_filter_node.py 
    * receives state estimates and desired input and outputs minimally adjusted input to guarantee safety 


## Main Code for Data Collection 
* sample_state_space.launch
    * launch data collection procedure
* grid_state_space_node.cpp
    * iterates through a gridding of the state space 



