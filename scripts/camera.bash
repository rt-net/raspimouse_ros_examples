#This script requires v4l2-ctl
#sudo apt-get install v4l-utils

#!/bin/bash
v4l2-ctl -d /dev/video0 --all
v4l2-ctl --set-ctrl white_balance_temperature_auto=0
v4l2-ctl --set-ctrl white_balance_temperature=4000
v4l2-ctl --set-ctrl focus_auto=0
v4l2-ctl --set-ctrl focus_absolute=0
#v4l2-ctl --set-ctrl exposure_auto=1
#v4l2-ctl --set-ctrl exposure_absolute=500
