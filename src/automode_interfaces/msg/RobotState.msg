# Robot state information for swarm robotics
uint32 robot_id                    # Unique identifier for the robot in the swarm
builtin_interfaces/Time stamp      # Timestamp when the message was generated
string floor_color            # Detected floor color ("black", "white", "gray", etc.)
float64 proximity_magnitude        # Strength of the detected obstacle vector
float64 proximity_angle           # Direction of the strongest proximity signal -- ROS2 convention - rad (-right, +left)
float64 light_magnitude           # Magnitude of detected ambient light
float64 light_angle              # Direction of the light source -- ROS2 convention - rad (-right, +left)
float64 attraction_angle         # Direction toward neighbouring robots -- ROS2 convention - rad (-right, +left)
uint32 neighbour_count            # Number of neighbouring robots detected