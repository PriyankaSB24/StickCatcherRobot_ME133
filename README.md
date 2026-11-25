# StickCatcherRobot_ME133

ME/CS/EE 133a final project - Atlas catches a randomly dropped stick.

Team: Priyanka, Jenny, Katie

https://docs.google.com/document/d/1YoZTJPoujT3qExUKqXEzDIid0VnyyDaPFpnXUgqOg4g/edit?tab=t.0


# --- one time ---
cd ~/robotws
colcon build --symlink-install
source ~/robotws/install/setup.bash

# tab 1 - Atlas  (first)
ros2 run robot_state_publisher robot_state_publisher \
    ~/robotws/src/atlas_description/urdf/atlas_v5.urdf

# tab 2 - RVIZ
ros2 run rviz2 rviz2 -d ~/robotws/src/stickcatcher/rviz/stickcatcher.rviz

# tab 3 - controller
ros2 run stickcatcher catch

# tab 4 - the game
ros2 run stickcatcher sticksim

# sanity check 
ros2 topic hz /hand          # catch is alive
ros2 topic echo /stick/result --once
