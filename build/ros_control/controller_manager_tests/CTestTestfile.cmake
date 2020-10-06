# CMake generated Testfile for 
# Source directory: /home/faustex/catkin_ws/src/ros_control/controller_manager_tests
# Build directory: /home/faustex/catkin_ws/build/ros_control/controller_manager_tests
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_controller_manager_tests_rostest_test_cm_test.test "/home/faustex/catkin_ws/build/catkin_generated/env_cached.sh" "/usr/bin/python" "/opt/ros/kinetic/share/catkin/cmake/test/run_tests.py" "/home/faustex/catkin_ws/build/test_results/controller_manager_tests/rostest-test_cm_test.xml" "--return-code" "/opt/ros/kinetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/faustex/catkin_ws/src/ros_control/controller_manager_tests --package=controller_manager_tests --results-filename test_cm_test.xml --results-base-dir \"/home/faustex/catkin_ws/build/test_results\" /home/faustex/catkin_ws/src/ros_control/controller_manager_tests/test/cm_test.test ")
add_test(_ctest_controller_manager_tests_nosetests_test "/home/faustex/catkin_ws/build/catkin_generated/env_cached.sh" "/usr/bin/python" "/opt/ros/kinetic/share/catkin/cmake/test/run_tests.py" "/home/faustex/catkin_ws/build/test_results/controller_manager_tests/nosetests-test.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/faustex/catkin_ws/build/test_results/controller_manager_tests" "/usr/bin/nosetests-2.7 -P --process-timeout=60 --where=/home/faustex/catkin_ws/src/ros_control/controller_manager_tests/test --with-xunit --xunit-file=/home/faustex/catkin_ws/build/test_results/controller_manager_tests/nosetests-test.xml")
add_test(_ctest_controller_manager_tests_rostest_test_cm_msgs_utils_rostest.test "/home/faustex/catkin_ws/build/catkin_generated/env_cached.sh" "/usr/bin/python" "/opt/ros/kinetic/share/catkin/cmake/test/run_tests.py" "/home/faustex/catkin_ws/build/test_results/controller_manager_tests/rostest-test_cm_msgs_utils_rostest.xml" "--return-code" "/opt/ros/kinetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/faustex/catkin_ws/src/ros_control/controller_manager_tests --package=controller_manager_tests --results-filename test_cm_msgs_utils_rostest.xml --results-base-dir \"/home/faustex/catkin_ws/build/test_results\" /home/faustex/catkin_ws/src/ros_control/controller_manager_tests/test/cm_msgs_utils_rostest.test ")
add_test(_ctest_controller_manager_tests_rostest_test_controller_manager_scripts.test "/home/faustex/catkin_ws/build/catkin_generated/env_cached.sh" "/usr/bin/python" "/opt/ros/kinetic/share/catkin/cmake/test/run_tests.py" "/home/faustex/catkin_ws/build/test_results/controller_manager_tests/rostest-test_controller_manager_scripts.xml" "--return-code" "/opt/ros/kinetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/faustex/catkin_ws/src/ros_control/controller_manager_tests --package=controller_manager_tests --results-filename test_controller_manager_scripts.xml --results-base-dir \"/home/faustex/catkin_ws/build/test_results\" /home/faustex/catkin_ws/src/ros_control/controller_manager_tests/test/controller_manager_scripts.test ")
