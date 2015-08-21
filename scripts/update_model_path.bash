#!/bin/bash
#Script for helping gazebo find ROS packages
#To use, add the packages you want to find to the array below and source the script
declare -a packages=( \
'fanuc_lrmate200id_support' \
'fanuc_demo'
)
model_path="$GAZEBO_MODEL_PATH"
for package in "${packages[@]}"; do
    package_path="$(rospack find $package)"
    package_path=$(echo ${package_path%/*})
    case ":$model_path:" in
      *":$package_path:"*) :;;
      *) model_path="$package_path:$model_path";;
    esac
done
export GAZEBO_MODEL_PATH=$model_path
