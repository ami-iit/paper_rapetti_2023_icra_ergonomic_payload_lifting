## Dependecies
All the required dependencies can be installed using the [`robotology-superbuild`](https://github.com/robotology/robotology-superbuild).
In order to install them follow the instruction in [robotology-superbuild documentation](https://github.com/robotology/robotology-superbuild/tree/master#robotology-superbuild), and make sure that the following [options](https://github.com/robotology/robotology-superbuild/blob/master/doc/cmake-options.md#robotology-superbuild-cmake-options) are enabled (and the associated dependencies are installed):
- `ROBOTOLOGY_ENABLE_HUMAN_DYNAMICS`
- `ROBOTOLOGY_USES_MATLAB`
- `ROBOTOLOGY_USES_GAZEBO`

## Installation
Clone the repository and create a `build` directory:

~~~bash
git clone https://github.com/ami-iit/paper_rapetti_2023_icra_ergonomic_payload_lifting.git
cd paper_rapetti_2023_icra_ergonomic_payload_lifting/
mkdir build
~~~

From the build directory, configure the CMake project and install it in the chosen `<installation-path>`:

```bash
cd build
cmake -DCMAKE_BUILD_TYPE="Release" -DCMAKE_INSTALL_PREFIX=<installation-path> ..
cmake  --config Release --build .
cmake  --config Release --install . 
make
make install
```

Once the installation is completed, append the following lines to your `.bashrc`:

```bash
# YARP DIRS
export YARP_DATA_DIRS=$YARP_DATA_DIRS:<installation-path>/share/ergonomy-control
export YARP_DATA_DIRS=$YARP_DATA_DIRS:<installation-path>/share/ergonomy-control/robots

# MODELS DIRS
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:<installation-path>/share/ergonomy-control/robots

# MATLAB DIRS
export MATLABPATH=${MATLABPATH:+${MATLABPATH}:}<installation-path>/mex
export MATLABPATH=${MATLABPATH:+${MATLABPATH}:}<installation-path>/share/simulink-robot-state-library
export MATLABPATH=${MATLABPATH:+${MATLABPATH}:}<installation-path>/share/simulink-multi-body-library

# PATH
export PATH=$PATH:<installation-path>/bin
```
