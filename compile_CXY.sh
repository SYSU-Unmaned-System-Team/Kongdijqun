catkin_make --source Modules/common/msgs --build build/msgs
catkin_make --source Modules/swarm_control --build build/swarm_control
catkin_make --source Modules/object_detection --build build/object_detection
catkin_make --source Modules/planning --build build/planning
catkin_make --source Modules/prometheus_case3 --build build/prometheus_case3
catkin_make --source Simulator/gazebo_simulator --build build/prometheus_gazebo
catkin_make --source Modules/swarm_communication --build build/swarm_communication
catkin_make --source Experiment --build build/prometheus_experiment
if [ ! -f "Modules/object_detection_yolov5openvino/CMakeLists.txt" ]; then
  # submodule object_detection_yolov5openvino not exist, skip it
  echo -e "\e[32m[INFO] SUBMODULE\e[0m \e[33mobject_detection_yolov5openvino\e[0m \e[32mNOT EXIST, Skip it!\e[0m"
else
  echo -e "\e[32m[INFO] COMPILE \e[33mobject_detection_yolov5openvino\e[0m \e[32m...\e[0m"
  # compile object_detection_landing
  catkin_make --source Modules/object_detection_yolov5openvino --build build/object_detection_yolov5openvino
fi