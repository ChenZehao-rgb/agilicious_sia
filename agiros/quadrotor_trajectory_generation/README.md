# Quadrotor Trajectory Generation

Some python code to generate quadrotor trajectories.

## Installation

Install [casadi](https://web.casadi.org/).

## Usage

```
python generate_trajectory.py --settings_file=config/settings.yaml
```

The script either generates `symbolic` trajectories or `random` trajectories.

To edit the generated trajectories (duration, sampling frequency, ...): edit `config/settings.yaml`. 

The symbolic trajectory type allows to define an arbitrary trajectory that can be expressed as a symbolic function. To define such function, edit `utils/symbolic_traj.py` and define the position as a function of time in casadi syntax.

To edit the random trajectories, edit the length scales in the settings file to control the aggressiveness
of the generated trajectory.
