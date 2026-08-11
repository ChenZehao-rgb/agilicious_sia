# Thrust Mapping

This folder contains the drivers and scripts needed to run experiments on the loadcell. For instructions about setup and
operation, have a look at the [Loadcell Wiki page](https://app.gitbook.com/@rpg-uzh/s/rpg-uzh/sensors/load-cell).

## Step 0: Preliminaries
Build the load cell reader.
```
catkin build load_cell_ros_node
```

This script only publishes control commands and records force and voltage. 
What you need to provide:
1. Launch a bridge that translates an `agiros/Command` to the motors
2. Provide voltage readings to the topic `/voltage`

## Step 1: Collect Data

After you have mounted the quadrotor on the thrust test stand, launch the launch file below. **Attention!** Below
command will let the motors spin!

```
roslaunch load_cell_ros_node collect_data.launch
```

This file will publish command messages to the topic `/command` and listen to the topics `/load_cell/data` and `/voltage`. The script then
saves this data to files to perform a thrust map identification in Step 2.

During thrust mapping, the motors perform a piecewise constant oscillation to identify the motors in their full
performance envelope. You can configure the data collection by editing the arguments
in `collect_data.launch`. Specifically, you can set these properties:

```
data_dir: root directory to save data to (you can leave the default)
cycles: how many thrust cycles to perform
min_cmd: the min command during the cycle
max_cmd: the max command during the cycle
zero_cmd: the zero command (this is dependent on the bridge that you use!)
num_cmd_steps: how many steps to perform between min_cmd and max_cmd
step_duration: how long each step should last
```

**Tip:** You can launch above launch file multiple times to collect more data. Each run will create its own time-stamped
folder with data.

## Step 2: Identify the thrust map

The thrust map is identified by running this python script. The script will read all data folders in `PATH_TO_DATA_ROOT_FOLDER`.

```
python3 identify_thrust_map.py --data_dir PATH_TO_DATA_ROOT_FOLDER
```

You might have to install some python dependencies using pip. Possible missing depencies are (python will tell you what
is missing!):

```
pip3 install --user sklearn tqdm
```