
BASED ON : chadrockey/structure_core_ros

Tested on SDK version 0.7.2 (FW: 0.9.7)

To run and build:

1) From your Structure Core SDK download:

`sudo DriverAndFirmware/Linux/Install-CoreDriver-Udev-Linux.sh`

`Scripts/build.sh`

`mkdir build`

`cd build`

`cmake -G'Unix Makefiles' -DCMAKE_BUILD_TYPE=Release ..`

`make Samples`

Check the samples are working (CorePlayground)

2) copy libStructure.so to /usr/local/lib/ (x86_64 or arm, depends on the platform):

`sudo cp Libraries/Structure/Linux/x86_64/libStructure.so /usr/local/lib/`

or

`sudo cp Libraries/Structure/Linux/arm64/libStructure.so /usr/local/lib/`

then

`sudo ln /usr/local/lib/libStructure.so`

3) `catkin_make`

4) `source devel/setup.bash`

5) Run

`roslaunch structure_core structure_driver.launch` 

6) Configs can be changed with: structure_prameters.yaml
