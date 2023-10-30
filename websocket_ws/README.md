# How to run
NOTE: This node only serves to send pointcloud and obstacle data to a backend server. If you don't have your own backend websocket reverse proxy configured (e.g NGINX) then it is useless. See "arachnida_web" for the associated backend Node.js server code (not the same as NGINX) and Next.js frontend code.

1. Have roscore running
2. Run arachnida nodes and nodelets via the arachnida_launch.launch file (roslaunch)
3. Run the websocket_ws Node.js ROS node by simply running "node ~/cavex-2023/CaveX2023/websocket_ws/src/websocket_interface/src/dist/app.js"
4. The websocket_ws ROS node will listen to the arachndia/pointcloud/pcl topic and forward the data to the backend server
