
BASED ON : chadrockey/structure_core_ros

To run and build:


==== 1) From your Structure Core SDK download, copy the include and lib folders from:

==== StructureSDK-CrossPlatform-0.6.1-2.0/Linux/Libraries/Structure/x86_64

1) From your Structure Core SDK download:

`sudo DriverAndFirmware/Linux/Install-CoreDriver-Udev-Linux.sh`
`Scripts/build.sh`
`mkdir build`
`cd build`
`cmake -G'Unix Makefiles' -DCMAKE_BUILD_TYPE=Release ..`
`make Samples`

Check the samples are working (CorePlayground)

2) if libStructure.so is missing do:

`sudo cp StructureSDK-CrossPlatform-0.7/Libraries/Structure/Linux/x86_64/libStructure.so /usr/local/lib/`
or
`sudo cp StructureSDK-CrossPlatform-0.7/Libraries/Structure/Linux/arm64/libStructure.so /usr/local/lib/`

into this repo (choose your platforms).

3) `catkin_make`

4) Run
`roslaunch src/Structure_Core/launch/structure_driver.launch` 

5) Configs can be changed with: structure_prameters.yaml
