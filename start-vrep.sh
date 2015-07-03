#!/bin/bash

#$1 domain
#$2 algorithm
#$3 instance
#$4 headless

VREP="/Users/skiesel/gopath/src/github.com/skiesel/motionplanningtoolkit/dependencies/V-REP_PRO_EDU_V3_2_1_Mac/vrep.app/Contents/MacOS/vrep"

if [ "$1" != "" ] && [ "$2" != "" ] && [ "$3" != "" ];
then
	HEADLESS=""
	if [ "$4" == "-headless" ];
	then
		HEADLESS="-h"
	fi

	DOMAIN="${1##*/}"
	ALGORITHM="${2##*/}"
	INSTANCE="${3##*/}"

	TEMPFILE=/tmp/$DOMAIN_$ALGORITHM_$INSTANCE

	awk 'FNR==1{print ""}1' $1 $2 $3 > $TEMPFILE

	$VREP $HEADLESS -g$TEMPFILE
else
	echo "./start-vrep ENVIRONMENT.envir ALGORITHM.config INSTANCE.inst [-headless]"
fi