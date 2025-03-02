

source devel/setup.bash

echo "================= Launch simulation.launch ================="
roslaunch simulation simulation.launch &
sleep 1

echo "================= Launch sem_img_proc_node ================="
rosrun image_converter_node sem_img_proc_node &
sleep 1

echo "================= Launch path_planner ================="
rosrun astar_path_planner path_planner &
sleep 1

echo "================= Launch frontier_detecter ================="
rosrun astar_path_planner frontier_detecter &
sleep 1

echo "================= Launch traj_gen.launch ================="
roslaunch trajectory_pkg traj_gen.launch &
sleep 1

echo "=== Task Begining ==="

wait
