#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "Specify folder with *.bag files to process"
    exit 1
fi

if [ ! -d "$1" ]; then
    echo "Folder $1 doesn't exist"
    exit 1
fi

ROSBAGS=$(find $1 -maxdepth 1 -name '*.bag')
echo $ROSBAGS | tr ' ' '\n'
echo ''

REWRITE_ROSBAG_SCRIPT=$(dirname $0)/../scripts/rewrite_rosbag.py
if [ ! -f "$REWRITE_ROSBAG_SCRIPT" ]; then
    echo "Script for rewriting doesn't exist. Expected path: $rewrite_rosbag_script"
    exit 1
fi

echo "Start processing..."
echo ''

for ROSBAG in $ROSBAGS; do
    (echo $ROSBAG && \
    echo "Reindexing..."
    ORIG_ROSBAG=${ROSBAG/%.bag/.orig.bag} && \
    rosbag reindex $ROSBAG && \
    rm $ORIG_ROSBAG && \
    echo "Rewriting..."
    REWRITTEN_ROSBAG=${ROSBAG/%.bag/_rewritten.bag} && \
    python $REWRITE_ROSBAG_SCRIPT -rosbag $ROSBAG -out $REWRITTEN_ROSBAG && \
    mv $ROSBAG $ORIG_ROSBAG && \
    mv $REWRITTEN_ROSBAG $ROSBAG && \
    rm $ORIG_ROSBAG && \
    echo "Done."
    echo '') || exit 1
done

touch $1/all_reindexed_and_rewritten.txt

echo "Success!"

