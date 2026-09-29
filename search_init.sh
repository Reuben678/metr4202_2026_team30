# USAGE:
#   ./run.sh                              -> default TurtleBot3 world
#   ./run.sh /full/path/to/my_world.world -> custom world
#   ./run.sh /path/to/my.world x_pose:=1.0 y_pose:=0.5  -> custom world + spawn pose
#   -> 'jobs' : shows list of background processes (Nodes running)
#   -> 'fg <job_id>' : Bring job to foreground using its ID

echo "Gathering dependencies"
rosdep install --from-paths src --ignore-src -y

echo "Initialising project_search nodes..."
source install/setup.bash

# If the first argument doesn't look like a launch argument (name:=value),
# treat it as the world file path.
LAUNCH_ARGS=()
if [ -n "$1" ] && [[ "$1" != *:=* ]]; then
    WORLD="$(realpath "$1")"
    if [ ! -f "$WORLD" ]; then
        echo "World file not found: $WORLD" >&2
        exit 1
    fi
    LAUNCH_ARGS+=("world:=$WORLD")
    shift
fi
LAUNCH_ARGS+=("$@")

# Nodes whose output you want to see (add new ones here)
MY_NODES="exploration_manager|frontier_search|util_weights"

echo "Launching exploration system"
ros2 launch src/autonomous_exploration.launch.py "${LAUNCH_ARGS[@]}" 2>&1 \
  | grep --line-buffered -E "^\[(${MY_NODES})-[0-9]+\]"