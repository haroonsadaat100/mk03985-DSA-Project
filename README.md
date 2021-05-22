# mk03985-DSA-Project

In this project we have applied graph theory and shortest path finding algorithms to detect the most efficient path from a source point to a destination point for a mobile robot under the presence of obstacles in between the robot and goal point.

This project has 5 key stages:
1, Read the image containing obstacle environment information.
2, Erode the image using an appropriate disk.
3, Compute convex hull of original read image and eroded image.
4, Plot the original obstacles and eroded obstacles.
5, Apply the edge extraction function.
6, Create appropriate edges and nodes.
7, Apply disktra to find the minimum distance between robot start and goal point.
8, PLot the shortest path on the graph containing all the edges and obstacles.


In order to run this code you should have the following libraries and version of python installed on your machines,
1, Python 3.0 or higher version.
2, cv2
3, numpy
4, matplotlib
5, math
6, shapely


Once all the libraries and apporpriate version of python has been installed. Follow these steps,
1, Open the main script named as Path_Planning, using idle.
2, In this script you will have  multiple image files containing obstcales. Select the one you wish to observe by uncommenting that file_name.
3, You can change the robot and goal postion by modifying RX,RY and GX,GY.
4, Cheers, you are done. Keep on trying different obstacle files and observe the results.
