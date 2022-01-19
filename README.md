# research-track-final-assignment

This code implements several nodes for controlling a robot simulation. The user can either control the robot by giving a goal or by hand using keyboard input.

##prerequisites 
To run the code you need to install the teleop_twist_keyboard package ( `sudo apt-get install ros-noetic-teleop-twist-keyboard` )
and konsole ( `sudo apt-get install konsole`).

## use

Copy the 2 packages in your ros workspace src folder. 
Compile with `catkin_make`.
Run simulation using the command `roslaunch robot_control_final_assignment sim.launch`
Run the nodes using the command `roslaunch robot_control_final_assignment robot_launch.xml`
Input 1,2 or 3 to choose your control mode.

## Nodes

### angle_filter

This node takes a range as parameters and returns the laserscan `/base_scan` filtered on this range on a chosen topic.

#### Subscribed topics

- `/scan` robot laserscan.

#### Published topics

- `/scan_topic` filtered laserscan.

#### Parameters 

- `min_angle(double)` minimum angle of the chosen range.
- `max_angle(double)` maximum angle of the chosen range.
- `scan_topic(string)` topic name for the published scan.

### control_node

This node publishes the desired twist of the robot on the `/cmd_vel` topic by correcting the one given by the teleop_twist_keyboard.

#### Subscribed topics

- `/right_scan` robot right laserscan.
- `/left_scan` robot left laserscan.
- `/front_scan` robot front laserscan.
- `/cmd_vel` current speed.

#### Published topics

- `/cmd_vel` desired velocities.

#### Algorithm

The node stops the robot if it is going straight and the front wall is too close, or if it is turning into a wall. 

### ui_node

This node implements the 3 modes (control using move_base, control by keyboard with or without help)

#### Algorithm

The node waits for a user input (between 1 and 3) and calls the corresponding nodes.

### move_base_position_node

This node publishes the desired goal using the move_base action server.It waits for a certain time the result and cancel the goal if it has not been reached.

#### Published topics

- `/move_base/cancel` cancel the goal

## flowchart of the code

[![](https://mermaid.ink/img/eyJjb2RlIjoiZ3JhcGggVERcbiAgICBBW3VpX25vZGVdIC0tPnxvcHRpb24gMSB8IEJbbW92ZV9iYXNlX3Bvc2l0aW9uX25vZGVdXG4gICAgQSAtLT4gfG9wdGlvbiAyfEpbVGVsZW9wX2tleWJvYXJkXVxuICAgIEEgLS0-IHxvcHRpb24zfEhbVGVsZW9wX2tleWJvYXJkXVxuICAgIEEgLS0-IHxvcHRpb24zfEtbQ29udHJvbCBub2RlXVxuICAgIEIgLS0-IHxnZXQgdXNlciBpbnB1dHxDW0RyaXZlIHRvIGdvYWxdXG4gICAgSiAtLT58L2NtZF92ZWx8TFtEcml2ZSB1bnRpbCBleGl0XSBcbiAgICBILS0-IHwvY21kX3ZlbHwgS1xuICAgIEstLT4gfC9jbWRfdmVsfE5bU3RvcCByb2JvdCBpZiBpdCBpcyBnb2luZyBpbiBhIHdhbGxdXG4gICAgQyAtLT58dGltZW91dCBleGNlZWRlZHwgRFtDYW5jZWxdXG4gICAgRCAtLT58R2V0IGFub3RoZXIgaW5wdXR8IEJcbiAgICBDIC0tPnxHb2FsIHJlYWNoZWR8IEJcblxuICAgIiwibWVybWFpZCI6eyJ0aGVtZSI6ImRlZmF1bHQifSwidXBkYXRlRWRpdG9yIjpmYWxzZSwiYXV0b1N5bmMiOnRydWUsInVwZGF0ZURpYWdyYW0iOmZhbHNlfQ)](https://mermaid-js.github.io/mermaid-live-editor/edit/#eyJjb2RlIjoiZ3JhcGggVERcbiAgICBBW3VpX25vZGVdIC0tPnxvcHRpb24gMSB8IEJbbW92ZV9iYXNlX3Bvc2l0aW9uX25vZGVdXG4gICAgQSAtLT4gfG9wdGlvbiAyfEpbVGVsZW9wX2tleWJvYXJkXVxuICAgIEEgLS0-IHxvcHRpb24zfEhbVGVsZW9wX2tleWJvYXJkXVxuICAgIEEgLS0-IHxvcHRpb24zfEtbQ29udHJvbCBub2RlXVxuICAgIEIgLS0-IHxnZXQgdXNlciBpbnB1dHxDW0RyaXZlIHRvIGdvYWxdXG4gICAgSiAtLT58L2NtZF92ZWx8TFtEcml2ZSB1bnRpbCBleGl0XSBcbiAgICBILS0-IHwvY21kX3ZlbHwgS1xuICAgIEstLT4gfC9jbWRfdmVsfE5bU3RvcCByb2JvdCBpZiBpdCBpcyBnb2luZyBpbiBhIHdhbGxdXG4gICAgQyAtLT58dGltZW91dCBleGNlZWRlZHwgRFtDYW5jZWxdXG4gICAgRCAtLT58R2V0IGFub3RoZXIgaW5wdXR8IEJcbiAgICBDIC0tPnxHb2FsIHJlYWNoZWR8IEJcblxuICAgIiwibWVybWFpZCI6IntcbiAgXCJ0aGVtZVwiOiBcImRlZmF1bHRcIlxufSIsInVwZGF0ZUVkaXRvciI6ZmFsc2UsImF1dG9TeW5jIjp0cnVlLCJ1cGRhdGVEaWFncmFtIjpmYWxzZX0)