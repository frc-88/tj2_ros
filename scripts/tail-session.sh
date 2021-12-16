#!/usr/bin/env bash

LOGS_DIR=/media/storage/logs
CURRENT_FILE=`find $LOGS_DIR -type f -printf '%T@ %p\n' | sort -n | tail -1 | cut -f2- -d" "`
echo $CURRENT_FILE
trap ' ' INT
tail -F -n 300 $CURRENT_FILE
printf "\n\n$CURRENT_FILE\n"
