#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "Specify folder with *.bag files to process"
    exit 1
fi

if [ ! -d "$1" ]; then
    echo "Folder $1 doesn't exist"
    exit 1
fi

rosbags=$(find $1 -maxdepth 1 -name '*.bag')
echo $rosbags | tr ' ' '\n'
echo ''

rewrite_rosbag_script=$(dirname $0)/../scripts/rewrite_rosbag.py
if [ ! -f "$rewrite_rosbag_script" ]; then
    echo "Script for rewriting doesn't exist. Expected path: $rewrite_rosbag_script"
    exit 1
fi

echo "Start processing..."
echo ''

for rosbag in $rosbags; do
    (echo $rosbag && \
    echo "Reindexing..."
    orig_rosbag=${rosbag/%.bag/.orig.bag} && \
    rosbag reindex $rosbag && \
    rm $orig_rosbag && \
    echo "Rewriting..."
    rewritten_rosbag=${rosbag/%.bag/_rewritten.bag} && \
    python $rewrite_rosbag_script -rosbag $rosbag -out $rewritten_rosbag && \
    mv $rosbag $orig_rosbag && \
    mv $rewritten_rosbag $rosbag && \
    rm $orig_rosbag && \
    echo "Done."
    echo '') || exit 1
done

touch $1/all_reindexed_and_rewritten.txt

echo "Success!"

