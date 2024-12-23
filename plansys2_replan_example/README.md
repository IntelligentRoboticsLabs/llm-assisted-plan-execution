# PlanSys2 Behavior Tree Replan Example

## Build

Use the `rolling` branch of PlanSys2.

## Environment


```
      wp3 ------ wp4 ------ wp7 ------ wp8
      |           |          |          |
Y     |           |          |          |
^     |           |          |          |
|     wp1 ------ wp2 ------- wp5 ----- wp6
|
+--- > x

```

wp1 is in (0,0), and coordinate axis are as shown

every 25 seconds, the state of the environment changes in a cyclic FSM with 3 states, starting in OPEN_25:
1. OPEN_25: wp4 <---> wp7 connection is removed
2. TWO_OPEN: all connections are set
3. OPEN_47: wp2 <---> wp5 connection is removed


## How to run

1. Launch PlanSys2:

```
ros2 launch plansys2_replan_example plansys2_replan_example_launch.py
```

2. Run the fake Nav2 server:

```
ros2 run plansys2_replan_example nav2_sim_node
```

3. Run the controller

```
ros2 run plansys2_replan_example replan_controller_node
```

## Goal and metrics

Objects are placed in random waypoints. At initial and every time a plan is achieved, it is generated new goals set as goal move each object to any random waypoint, maybe the same for any of them.

The idea is adding al alternative `ReplanController::should_replan()` based on LLMs and compare time to complete the plan, and times that a plan fail.
