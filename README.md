# Spheremap server

## Dependencies

* [octomap](https://octomap.github.io/)
* [pcl_ros](http://wiki.ros.org/pcl_ros)

%% ## Installation
%% We recommend compiling the **uav_modules** in a separated workspace from the uav_core.
%% More information in the [catkin workspace guide](https://ctu-mrs.github.io/docs/software/catkin/managing_workspaces/managing_workspaces.html).

%% 1. Clone the **uav_modules**:
%% ```bash
%% cd ~/git
%% git clone https://github.com/ctu-mrs/uav_modules
%% ```
%% 2. Call the main installation script:
%% ```bash
%% ~/git/uav_modules/installation/install.sh
%% ```
%% 3. Create a ROS workspace dedicated just to the **uav_modules** package and link the uav_modules into it:
%% ```bash
%% mkdir -p ~/modules_workspace/src && cd ~/modules_workspace
%% catkin init
%% cd ~/modules_workspace/src
%% ln -s ~/git/uav_modules
%% ```
%% 4. Extend the **mrs_workspace**:
%% ```bash
%% cd ~/modules_workspace
%% catkin config --extend ~/mrs_workspace/devel
%% ```
%% 5. Compile the workspace
%% ```bash
%% cd ~/modules_workspace
%% catkin build
%% ```

## Config and running the SphereMap server
TODO

## Using the fast safety-aware Planning services

TODO

## Changing minimal safe radius and obstacle distance planning weight

TODO

## Using the LTV-Map lightweight topological-volumetric map
TODO