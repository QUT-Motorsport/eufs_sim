cmake_minimum_required(VERSION 2.8.3)
project(testing_sim)

find_package(catkin REQUIRED)

find_package(rostest REQUIRED)
add_rostest(tests/acceleration.test)
add_rostest(tests/autocross.test)
add_rostest(tests/inspection.test)
add_rostest(tests/skidpad.test)
add_rostest(tests/track_drive.test)
