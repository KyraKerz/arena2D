# CMake generated Testfile for 
# Source directory: /home/joe/projects/rl/src/arena2d
# Build directory: /home/joe/projects/rl/src/arena2d/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_arena2d_rostest_launch_interactionServ.test "/home/joe/projects/rl/src/arena2d/build/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/joe/projects/rl/src/arena2d/build/test_results/arena2d/rostest-launch_interactionServ.xml" "--return-code" "/usr/bin/python2 /opt/ros/melodic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/joe/projects/rl/src/arena2d --package=arena2d --results-filename launch_interactionServ.xml --results-base-dir \"/home/joe/projects/rl/src/arena2d/build/test_results\" /home/joe/projects/rl/src/arena2d/launch/interactionServ.test ")
subdirs("gtest")
subdirs("box2d/src")
