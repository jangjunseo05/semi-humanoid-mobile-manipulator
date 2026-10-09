#!/usr/bin/env bash
# Kills leftover semiH_ws processes (Gazebo, ros_gz bridges, controller_manager
# spawners, SLAM/Nav2, our own perception/data-collection nodes) that survive
# a Ctrl-C or a crashed terminal. Run this before starting a new session to
# avoid "Controller already loaded" / stale-topic problems caused by orphaned
# processes fighting the new ones for CPU and DDS topic names.
#
# Usage: ./cleanup.sh [--dry-run]

set -u

DRY_RUN=false
if [[ "${1:-}" == "--dry-run" ]]; then
    DRY_RUN=true
fi

# Process patterns specific to this project. Kept narrow on purpose so this
# doesn't reach into unrelated ROS2 work the user might have running.
PATTERNS=(
    'ros2 launch semih_description'
    'ign gazebo'
    'ros_gz_bridge/parameter_bridge'
    # NOTE: the actual process argv never contains "robot_description" (that text
    # only lives inside the --params-file's contents, invisible to pgrep -f) -- an
    # earlier version of this pattern required it and so never matched anything,
    # silently leaking one robot_state_publisher per session all day. Confirmed by
    # finding 27 stray instances, oldest 4.5 hours old, during a live debugging
    # session (see docs/troubleshooting_p1-p4.md).
    'robot_state_publisher/robot_state_publisher --ros-args'
    'controller_manager/spawner'
    'slam_toolbox'
    'nav2_bringup'
    'bt_navigator|controller_server|planner_server|behavior_server|smoother_server|waypoint_follower|velocity_smoother|lifecycle_manager'
    'rviz2'
    'semih_perception/(yolo_detection_node|target_3d_node|arm_reach_node)'
    'semih_data_collection/hdf5_collector_node'
    'explore_lite/explore'
    'ros2 action send_goal.*navigate_to_pose'
)

SELF_PID=$$
MATCHED_PIDS=()

for pattern in "${PATTERNS[@]}"; do
    while read -r pid; do
        [[ -z "$pid" || "$pid" == "$SELF_PID" ]] && continue
        MATCHED_PIDS+=("$pid")
    done < <(pgrep -f "$pattern" 2>/dev/null)
done

# De-duplicate
UNIQUE_PIDS=($(printf '%s\n' "${MATCHED_PIDS[@]}" | sort -un))

if [[ ${#UNIQUE_PIDS[@]} -eq 0 ]]; then
    echo "No leftover semiH_ws processes found."
    exit 0
fi

echo "Found ${#UNIQUE_PIDS[@]} leftover process(es):"
ps -o pid,etime,%cpu,cmd -p "$(IFS=,; echo "${UNIQUE_PIDS[*]}")" 2>/dev/null

if $DRY_RUN; then
    echo "--dry-run: not killing anything."
    exit 0
fi

echo "Sending SIGTERM..."
kill -TERM "${UNIQUE_PIDS[@]}" 2>/dev/null
sleep 3

STILL_ALIVE=()
for pid in "${UNIQUE_PIDS[@]}"; do
    kill -0 "$pid" 2>/dev/null && STILL_ALIVE+=("$pid")
done

if [[ ${#STILL_ALIVE[@]} -gt 0 ]]; then
    echo "Escalating to SIGKILL for ${#STILL_ALIVE[@]} process(es) that ignored SIGTERM..."
    kill -KILL "${STILL_ALIVE[@]}" 2>/dev/null
    sleep 1
fi

STILL_ALIVE=()
for pid in "${UNIQUE_PIDS[@]}"; do
    kill -0 "$pid" 2>/dev/null && STILL_ALIVE+=("$pid")
done

if [[ ${#STILL_ALIVE[@]} -gt 0 ]]; then
    echo "WARNING: ${#STILL_ALIVE[@]} process(es) survived SIGKILL: ${STILL_ALIVE[*]}"
    exit 1
fi

echo "Done. All matched processes terminated."
exit 0
