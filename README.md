# paper_plansys2_LLMs

## Install LLM Dependencies

This project uses the Gemini API, which offers a free version. To get your API key, sign up [here](https://ai.google.dev/gemini-api/docs).

Once you have your API key, create an environment variable `GOOGLE_API_KEY` and export your key:

```
export GOOGLE_API_KEY=yourapikey
```


## Using Virtual Environments (Recommended for Ubuntu 24)

To work with Python packages in a virtual environment, follow these steps from your workspace folder:

1. Create and activate a virtual environment:
   ```
   python3 -m venv my_venv_name --system-site-packages
   source my_venv_name/bin/activate
   ```

2. Install the required dependencies:
   ```
   pip install -r src/paper_plansys2_LLMs/requirements.txt
   ```

### Without Virtual Environment
If you don't wish to use a virtual environment, you can install the dependencies directly:
```
pip install -r src/paper_plansys2_LLMs/requirements.txt --break-system-packages
```

### Additional Repositories
Don't forget to install the repositories specified in the `thirdparty.repos` file.
and install nlohmann json library:

```
sudo apt-get install nlohmann-json-dev
```
## Build

Use the `rolling` branch of PlanSys2.

I had to create a new message for action `NavigateToPose` for the fake Nav2 server to know the starting point coordinates, in roder to simulate the navigation time. When using this with real robot, We should use the standard `nav2_msgs::action::NavigateToPose`, and change `Move.hpp` and `Move.hpp` to reflect this change.

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


## Running with LLMs

### 1. Launch PlanSys2 and Nav2 server:
```
ros2 launch plansys2_replan_example plansys2_replan_llm_example_launch.py
```

### 2. Launch the controller with LLMs:
```
ros2 launch plansys2_replan_example plansys2_replan_llm_controller_launch.py
```


## Goal and metrics

Objects are placed in random waypoints. At initial and every time a plan is achieved, it is generated new goals set as goal move each object to any random waypoint, maybe the same for any of them.

The idea is adding al alternative `ReplanController::should_replan()` based on LLMs and compare time to complete the plan, and times that a plan fail.
