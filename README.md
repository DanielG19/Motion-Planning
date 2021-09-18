# Motion-Planning
The starter code is really similar to the backyard code, there are certain difference in the states. But the most important difference is the plan_path function, which is the one in charge of planning the waypoints the dron will take. Also there is the planning_utils.py which contains all the different functions that i can use for my path planning.
PLAN_PATH:
![image](https://user-images.githubusercontent.com/29236973/133879570-1479ce5c-458a-4c52-a7d0-17a453bb8af5.png)

This function is the one in charge of planning the path, that mean that we will read the map, we will set home (local and global positions), also we assign the altitude of the drone (for our 2.5d map, but mainly for our "configuration space"), the safety distance (for avoiding collitions), the start point and the goal point. Once everything is set up inside this function, we start calling the functions inside "planning_utils" so that we can get the plan that the drone will follow, finally we get the waypoints which are added to the self.waypoints = [], and we start processing the states for the movement of the drone.
PLANNING_UTILS:

![image](https://user-images.githubusercontent.com/29236973/133879588-05782d7f-646a-45e9-b067-664bd7b40a6a.png)

First function that we have is "create_grid" which is the same one from the past examples, which will create a grid with the data, the drone-altitude and the safety distance.

![image](https://user-images.githubusercontent.com/29236973/133879623-3fedafe3-e1a9-4d81-8b90-7b20ff356f4e.png)

The class action is an Enum which will give me the movements that my drone can do and the cost, in this starter function there are only 4 movements, which mean that i will have to add the other 4.

![image](https://user-images.githubusercontent.com/29236973/133879631-78707f13-e199-477d-a315-76fb1bd267f1.png)

"Valid-Actions", will check if the movement inside my Queue is valid, if it is  not then it removes the action.

![image](https://user-images.githubusercontent.com/29236973/133879646-9a0a6f35-295b-4736-9b88-5a269dd54adb.png)

The "a-star" Algorithm will create the most efficient path depending of the actions, costs and heuristic.  In this case this A will work ok with grid enviorement, but it can be modified for solving in graph approximation.

![image](https://user-images.githubusercontent.com/29236973/133879694-01c43dab-6738-40d9-9bd0-162cd5b587bb.png)

Finally the heuristic function, which is using the "Norm 2", will give me the total values so that i can compare which one of them is the minimal.

# Implementing Your Path Planning Algorithm
*TODO: read lat0, lon0 from colliders into floating point values
Using panda as the csv reader, I extract only the first row and separate latitude and longitude.

![image](https://user-images.githubusercontent.com/29236973/133879850-f53633c5-5b1c-4fcc-b593-3ff613f3e70c.png)

*TODO: determine your local position relative to global home
Using Global_to_local function from Planning_utils.py, I used the global position and the global home, in order to get the local position.

![image](https://user-images.githubusercontent.com/29236973/133880076-2e6cc159-ccf1-4a5e-bc12-066c97074bca.png)

*TODO: convert start position to current position rather than map center
Knowing my local position. Then by adding my own offset to the center of the map. I can get the coordinates of where the dron is.

![image](https://user-images.githubusercontent.com/29236973/133880159-09aa0cd2-2249-47a3-a2b7-581932a184cc.png)


