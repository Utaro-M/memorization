#!/bin/bash

# source ~/catkin_ws/jaxon_tutorial/devel/setup.bash
bag_file_name_def="black_left_bag_2021-08-05-17-15-20"
bag_file_name=${1:-$bag_file_name_def}
name_def="right"
name=${2:-$name_def}
echo "s/frame/$name/"

rate=${3:-3}
echo $rate #$1がなかったらhogeをデフォルト値としてfooに代入する

# (sleep 20;kill $$)&
roslaunch memorization bag2jpeg.launch rate:=$rate file_name:=$bag_file_name&

PID=$!
# Wait for 2 seconds
sleep 10
# Kill it
kill $PID

rename "s/frame/$name/" ~/.ros/frame00*
echo "rename"
mv ~/.ros/$name* /home/utaro/Desktop/dataset/test_data/
echo "move files "
