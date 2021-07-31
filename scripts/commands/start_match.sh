#!/bin/bash

MATCH_DELAY=${1:-120.0}
MATCH_DELAY_S=${MATCH_DELAY%.*}
MATCH_DELAY_NS=`echo 1000000000.0 \* $MATCH_DELAY | bc -l`
MATCH_DELAY_NS=`echo $MATCH_DELAY \% 1000000000.0 | bc`
MATCH_DELAY_NS=${MATCH_DELAY_NS%.*}
echo $MATCH_DELAY_S
echo $MATCH_DELAY_NS
rosservice call /tj2/start_match "delay: {secs: $MATCH_DELAY_S, nsecs: $MATCH_DELAY_NS}"

tmux a -t match
