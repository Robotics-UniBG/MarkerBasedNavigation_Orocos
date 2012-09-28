#!/bin/bash

TEST_SCRIPT="lua/youbot_test.lua"
DUMP_PARAM="lua/dump_param.lua"

export LUA_PATH="$LUA_PATH;`rospack find youbot_driver_rtt`/lua/?.lua"
RFSM_DIR=`rospack find rFSM`

if [[ ! -d $RFSM_DIR ]]; then
    echo "required rFSM package not found, please install"
    exit 1;
fi

function usage() {
    cat <<EOF
youbot test and calibration script.
usage: $1 <option>
  options:  -r: run script
            -h: print this
EOF
}

if [[ -z $1 ]]; then
    usage
elif [[ $1 == "-r" ]]; then
    rosrun ocl rttlua-gnulinux -i $TEST_SCRIPT
else
    usage
fi

