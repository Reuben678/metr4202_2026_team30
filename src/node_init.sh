# USAGE:
#   -> 'jobs' : shows list of background processes (Nodes running)
#   -> 'fg <job_id>' : Bring job to foreground using it's ID
#   -> CTRL-C to the

# Initialise project nodes across the project-search directory
echo "Initialising project_search nodes..."

source install/setup.bash

echo "Init: util_weights"
ros2 run project_search util_weights &

echo "Init: frontier_search"
ros2 run project_search frontier_search &

echo "Init: mst_planner"
ros2 run project_explore mst_planner & 

echo "Init: exploration_manager"
ros2 run project_explore exploration_manager &

echo "Init: nav_handler"
ros2 run project_explore nav2_handler &

