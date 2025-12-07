#!/bin/bash

BAG_FILE="${1:-/workspaces/ros2-devenv/src/athena_localizer_test/data/kitti_2011_09_26_drive_0117_synced_0.mcap}"

cleanup() {
    pkill -P $$
    exit 0
}
trap cleanup SIGINT SIGTERM

ros2 bag play "$BAG_FILE" &
sleep 2
ros2 run athena_localizer_test athena_localizer_test_node &
sleep 3
ros2 run athena_localizer_test localizer_evaluation_node

cleanup
