#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
dt-exec roslaunch my_package saxeli.launch

# wait for app to end
dt-launchfile-join