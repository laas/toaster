## tools repository

This repository is made to share tools linked to toaster.
One of the repositories is `toaster-scripts`. It contains launch files and scripts.

### toaster-scripts
It contains **roslaunch** files to launch several nodes of the toaster architecture with some parameters.
As an example, the roslaunch file at  `roslaunch/toaster_simu.launch` is running `toaster_simu`, `pdg`, `agent_monitor`, `area_manager`, `toaster_visualizer` and `rviz`. For `pdg` is uses parameters defined in `pdg/params/toaster_simu.yaml` so that pdg uses topics from `toaster_simu` as input.

This repository also provides **shell scripts**. As an example the script `/shell/spencer/area_spencer2.sh` will send requests to `area_manager` to configure areas around a guiding robot as shown in figure below.


In the `tools` repository, we also provide **python scripts**.
As an example, the python script in `python/simu_set.py` is calling services in `toaster_simu` and `agent_monitor` to configure the environment and to set an agent `Greg` as a keyboard controlled entity and a monitored agent.