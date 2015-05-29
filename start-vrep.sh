#!/bin/bash

if [ "$1" != "" ];
then
	../dependencies/V-REP_PRO_EDU_V3_2_1_Mac/vrep.app/Contents/MacOS/vrep -g$1
else
	echo "missing instance file parameter"
fi