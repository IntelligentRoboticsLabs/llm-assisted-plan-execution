# LLM-Assisted Plan Execution for Robots in Dynamic Environments

**Preprint:** [SSRN](https://papers.ssrn.com/sol3/papers.cfm?abstract_id=5350912)  
**Demo Video:** [YouTube](https://youtu.be/hloKG2jY564)

## Abstract
In recent years, planning frameworks have enabled the creation and execution of plans in robots using classical planning approaches based on the Planning Domain Definition Language (PDDL). The dynamic nature of the environments in which these robots operate requires that execution plans adapt to new conditions, either by repairing plans to improve efficiency or because they are no longer valid. Determining the appropriate moment to initiate such repairs is the focus of our research. This paper presents a novel approach to this problem by leveraging Large Language Models (LLMs) to make informed plan repair decisions during robot operation. Our approach provides reasoning capabilities that go beyond the traditional heuristic methods employed in symbolic planning frameworks, while addressing the common hallucinations associated with task planning when relying solely on generative artificial intelligence. A novel feature of our approach is that LLMs can enhance decision-making regarding when to repair a plan by employing reflection-based methods to infer repairable features and forecast potential hazards, proactively identifying dangerous situations and enabling more efficient, adaptable repairing processes. We experimentally demonstrate the validity of our approach using real robots in environments where both the environmental conditions and the goals to be achieved change dynamically.

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
You need to install the supplementary repositories specified in the `thirdparty.repos` file. Within your workspace root, run:

```bash
vcs import src < src/paper_plansys2_LLMs/thirdparty.repos
```

Also, install the `nlohmann-json` library and resolve any other ROS 2 dependencies:

```bash
sudo apt-get update
sudo apt-get install nlohmann-json-dev
rosdep install --from-paths src --ignore-src -r -y
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
ros2 run plansys2_replan_example replan_controller  --strategy <basic|improved|LLM> --experiment <dynamic_world|dynamic_goals>
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
