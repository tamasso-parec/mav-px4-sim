`ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="default_custom.sdf"
`
Then start simulation 

`MicroXRCEAgent udp4 -p 8888
`

`./QGroundControl.AppImage 
`

`env PX4_SIM_MODEL="gz_x500" "build/px4_sitl_default/bin/px4" -d -s etc/init.d-posix/rcS "build/px4_sitl_default/etc"
`

Maybe this way we can also add the ros bridge