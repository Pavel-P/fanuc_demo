#!/bin/bash
#Script to update model sdf's from xacro's
for dir in $(rospack find fanuc_demo)/models/*; do
    dir=${dir##*/}
    xacro_file="$(rospack find fanuc_demo)/models/$dir/$dir.urdf.xacro"
    if [ -f "$xacro_file" ]; then
        rosrun xacro xacro.py "$xacro_file" -o tmp
	
        gz sdf -p tmp > "$(rospack find fanuc_demo)/models/$dir/model.sdf"
        #gzsdf print tmp > "$(rospack find fanuc_demo)/models/$dir/model.sdf" #for gazebo version <=2 
        rm tmp
    fi
done
