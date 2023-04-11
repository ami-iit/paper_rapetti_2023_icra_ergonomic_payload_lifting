# Models
This repository contains models and worlds for running multi-agent scenarios in Gazebo.  
In order to use multiple robots in Gazebo, follow the instructions in [`two_robots_simulation.md`](https://github.com/icub-tech-iit/documentation/blob/master/docs/icub_setup_multiple_robots/two_robots_simulation.md).

## Installation
Clone the repository and create a `build` directory under the `element_wearable_sw` directory for example by using a terminal:

~~~bash
git clone https://github.com/dic-iit/element_ergonomy-control
cd element_ergonomy/
mkdir build
~~~

From the build directory, configure the CMake project and install it in the `build/install` directory:

```bash
cd build
cmake -DCMAKE_BUILD_TYPE="Release" -DCMAKE_INSTALL_PREFIX="./install" ..
cmake  --config Release --build .
cmake  --config Release --install . 
make
make install
```

Once the installation is completed, append the following lines to your `.bashrc`:

```bash
export YARP_DATA_DIRS=$YARP_DATA_DIRS:<directory-where-you-downloaded-element_ergonomy-control>/build/install/share/ergonomy-control
export YARP_DATA_DIRS=$YARP_DATA_DIRS:<directory-where-you-downloaded-element_ergonomy-control>/build/install/share/ergonomy-control/robots
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:<directory-where-you-downloaded-element_ergonomy-control>/build/install/share/ergonomy-control/robots
```
