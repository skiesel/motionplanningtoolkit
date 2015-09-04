#!/bin/bash

ROOT="/home/aifs1/skiesel/gopath/src/github.com/skiesel/motionplanningtoolkit/"

DATA_ROOT="${ROOT}/data/"

BUILD_DIR="${ROOT}/build/"
PLANNER="${BUILD_DIR}Planner"

EXPORT_CMD="export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${BUILD_DIR}"

LIMIT_CMD="ulimit -t 1800 -m -v 6000000"

DOMS="planarlinkage kink narrowpassagefull narrowpassagehalf" #blimp
ALGS="rrt sst sstgrid rrtconnect kpiece pprm"

for DOM in $DOMS;
do

    DOM_DIR="${ROOT}description_files/${DOM}/"
    DOM_FILE="${DOM_DIR}${DOM}.dom"
    
    ALG_DIR="${ROOT}description_files/${DOM}/algorithms/"

    INST_DIR="${ROOT}description_files/${DOM}/instances/"

    touch ${DATA_ROOT}/KEY=domain
    mkdir -p ${DATA_ROOT}/${DOM}/

    for ALG in $ALGS;
    do
	touch ${DATA_ROOT}/${DOM}/KEY=algorithm
	mkdir -p ${DATA_ROOT}/${DOM}/${ALG}/
	touch ${DATA_ROOT}/${DOM}/${ALG}/KEY=inst
	
	for INST in `find $INST_DIR -type f`;
	do
	    RUN="${PLANNER} ${DOM_FILE} ${ALG_DIR}${ALG}.alg ${INST}"
	    
	    FILE=$(basename "$INST")
	    BASE_FILE="${FILE%.*}"
	    DF="${DATA_ROOT}/${DOM}/${ALG}/${BASE_FILE}"
	    echo "${EXPORT_CMD}; ${LIMIT_CMD}; $RUN > $DF"
	done
    done
done
