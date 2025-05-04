#!/usr/bin/env python3
import time
from robot_controller.start_controller import main

if __name__ == '__main__':
    print("Delaying start of move_robot by 5 seconds to let Gazebo initialize...")
    time.sleep(30)  # Adjust the delay if needed
    main()
