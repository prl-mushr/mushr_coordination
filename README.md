[![Build Status](https://dev.azure.com/prl-mushr/mushr_sim/_apis/build/status/prl-mushr.mushr_sim?branchName=master)](https://dev.azure.com/prl-mushr/mushr_sim/_build/latest?definitionId=5&branchName=master)

# MuSHR Coordination

## Install
Clone repo: 
``` 
cd ~/catkin_ws/src/ && git clone --recurse-submodules https://github.com/prl-mushr/mushr_coordination.git
```

## API
For adjusting params see `launch/mushr_coordination.launch` it has the environment params for the planner. Change number of Mushr Car in environment by setting parameter `num_agent` and change number of waypoints for each goal by setting parameter `num_waypoint`.

### Publishers
Topic | Type | Description
------|------|------------
`/{car_1's name}/waypoints` | [geometry_msgs/PoseArray](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseArray.htmll)| Pathway assigned to car_1
...
`/{car_n's name}/waypoints`| [geometry_msgs/PoseArray](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseArray.htmll) | Pathway assigned to car_n
`/{car_1's name}/marker` | [visualization_msgs/Marker](http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html)| Intermediate points of task(s) assigned to car_1 
...
`/{car_n's name}/marker`| [visualization_msgs/Marker](http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html) | Intermediate points of task(s) assigned to car_n 
`/mushr_coodination/border` | [visualization_msgs/Marker](http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html) | Borderline of the current setup environment for the planner


### Subscribers
Topic | Type | Description
------|------|------------
`/{car_1's name}/init_pose` | [geometry_msgs/PoseStamp](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseStamped.html)| Initial position of car_1
...
`/{car_n's name}/init_pose` | [geometry_msgs/PoseStamp](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseStamped.html)| Initial position of car_n
`/mushr_coordination/obstacles` | [geometry_msgs/PoseArray](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseArray.htmll)| List of obstacles in the map
`/mushr_coordination/goals` | [/mushr_coordination/GoalPoseArray](#mushr_coordination/GoalPoseArray ) | List of goals for {n} cars to complete


### Message Definition
#### mushr_coordination/GoalPoseArray  
[`std_msgs/Header`](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Header.html) Header \
`float64` scale `#scala for converting continous space to grid space`   
`float64` minx \
`float64` miny \
`float64` maxx \
`float64` maxy \
[`geometry_msgs/PoseArray`](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseArray.html)[] goals 

