# Controller_pkg
## Node
- **Controller node**  
This node subcribes the given `desired trajectory message` and publishes corresponding `motor action` to drone.  
- **Keyboard_input_node**  
This node recieves the input of keyboard at the terminal and map the keys to the corresponding `control signal` and publish them.
- **keyboard_traj_node.cpp**
This node subscribes the `control signal` from the keyboard_input_node and publishes `feasible trajectory` from current postion to desired position.
