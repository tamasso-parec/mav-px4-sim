# PX4 ROS2

## Adding models to PX4 

- Add an entry with a suitable number and name in [`~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes`](../../../PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/)
- Add the model into [`src/drone_description/models`](src/drone_description/models/)
- Update the [`CMakeLists.txt`](../../../PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt) adding the name of the airframe
- Make sure to check the appropriate flags are defined in [`~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.simulator`](../../../PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.simulator)


## Launch simulation environment

```bash
ros2 launch traj px4_sim.launch.py
```

## Takeoff

```bash
ros2 run traj arm_takeoff
```

## OFFBOARD MODE

```bash
ros2 run traj offboard_control
```

## Land

```bash
ros2 run traj land_disarm
```