#!/bin/bash

# Input 1: Map number
# Before running this script for the first time, make sure
# /home/${USER}/docker_share/scripts/vars/pha_carla_vars.sh and 
# $CARLA_ROOT/Unreal/CarlaUE4/Config/DefaultEngine.ini has matching
# description of the map.
declare -a map_list=("01" "02" "03" "04" "05" "06" "07" "10HD")
length=${#map_list[@]}
match_input=false
source /home/${USER}/docker_share/scripts/vars/pha_carla_vars.sh

for (( j=0; j<$length; j++ ));
do
    if [[ $1 == ${map_list[$j]} ]] 
    then
        sed -i -e "s/${PHA_CARLA_MAP}/Town${1}_Opt/g" $CARLA_ROOT/Unreal/CarlaUE4/Config/DefaultEngine.ini
        sed -i -e "s/${PHA_CARLA_MAP}/Town${1}_Opt/g" /home/${USER}/docker_share/scripts/vars/pha_carla_vars.sh
        match_input=true
    fi
done

if [ "$match_input" = false ]
then
    echo "Map is not listed!"
fi