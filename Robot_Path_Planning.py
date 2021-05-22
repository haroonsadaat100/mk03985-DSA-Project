import cv2
import numpy as np
from matplotlib import pyplot as plt
import math
from shapely.geometry import LineString
import graphs



#We assign the name of the image if in the same folder of code or path of the image if it is in a different folder into a variable
file_name = 'square2.png' 
#file_name = 'square5.png'
#file_name = 'square_new2.png'
#file_name = 'square_new4.png'
#file_name = 'square_new5.png'



#Here we read the image and dilate it in order to expand the obstacles
img = cv2.imread(file_name, 0)
kernel = np.ones((50,50), np.uint8)
img_dilation = cv2.erode(img, kernel, iterations=1)


kernel = np.ones((40,40), np.uint8)
img_dilation2 = cv2.dilate(img, kernel, iterations=1)
hull = img_dilation



#These lines can be uncommented if you want to observe the result of dilation on the obstacle image or if you want to save the dilated image
##cv2.imshow('Input', img)
##cv2.imshow('Eroded',hull)
##cv2.imwrite('Eroded.png',hull)


#This function, name distance calculates the euclidean distanc between two points in a 2D plane
def distance(A,B):
    return math.sqrt((round(A[0]-B[0]))**2+round((A[1]-B[1]))**2)


#Here we use some image processing algorithms for calculating the contours or obstacle boundaries
img = cv2.imread(file_name) 
#converting image into grayscale
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#setting threshold of gray image
_, threshold = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)



# contours for obstacles
contours_obs, _ = cv2.findContours(
    threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

# contours for dilated obstacles
contours_bound, _ = cv2.findContours(
    hull, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


#Here we define robot's start and goal position
sx = -5 # [m]
sy = -5 # [m]
gx = 600 # [m]
gy = 600  # [m]
grid_size = 2.0  # [m]
robot_radius = 1.0  # [m]


#We plot the robot's start and goal postion on the 2D-plot.
plt.plot(sx, sy, "og")
plt.plot(gx, gy, "xb")
plt.grid(True)
plt.axis("equal")


#This function plots lines on the 2D-plot and returns the corner points along with a list containing all the elements, which are convex hull of the given obstacles 
def Find_Corners(contours,color):
    final_contours = []
    corners = []
    i = 0
    for contour in contours:         
        # here we are ignoring first counter because 
        # findcontour function detects whole image as shape
        if i == 0:
            i = 1
            continue
        else:
            contour = list(contour)
            num = 0
            new_con = []
            for j in contour:
                #print(j[0])
                if num%2 == 0:
                    #we convert the corner points into list
                    tem = list((j[0][0],j[0][1]))
                    new_con.append(tem)
                    corners.append(tem)

                num+=1
            check = len(new_con)-1
            final_contours.append(new_con)
            for k in range(len(new_con)):
                if k==check:
                    A = new_con[k]
                    B = new_con[0]
                else:
                    A = new_con[k]
                    B = new_con[k+1]
                x_values = [A[0], B[0]]
                y_values = [A[1], B[1]]
                plt.plot(x_values, y_values,color)

    #A list of individual corner points  and a list containing corner points of each obstacle as a single element is returned
    return corners,final_contours




#Using the Find_Corners function we calculate the corner points and a list containing set of corner points corresponding to each object,
#the obstacle and eroded obstacle image.
#The plotting of these obstacle boundaries and expanded obstacle boundaries is done inside the Find_Corners function.
Robot = [sx,sy]
Goal  = [gx,gy]
corners_obs,final_contours_obs = Find_Corners(contours_obs,'-g')
plt.pause(1)
corners_bound,final_contours_bound = Find_Corners(contours_bound,'-b')


##You can print corners of obstacles and expanded obstacles
##print(corners_obs)
##print(corners_bound)


#This is the most crucial function, as it creates valid edges for graph creation.
#The conditions that we have to check here is that for any line between two valid nodes
#If they cut through an obstacles then the edge is invalid other wise it is valid.
def plot_edges(start,corners,final_contours):
    edges = []
    R = start
    for i in corners:
        result = 0
        line = LineString([R, [i[0],i[1]]])
        collision = False
        for j in final_contours:
            other = LineString(j)
            result = line.intersects(other)
            #print("New_Iter---------------------------------------")
            #print(i)
            #print(j)
            #print(result)
            if result == True:
                collision = True
        if collision == False:
            edges.append((R,i,distance(R,i)))
            x_values = [R[0], i[0]]
            y_values = [R[1], i[1]]
            plt.plot(x_values, y_values, "-b")
            plt.pause(0.05)
    return edges




#This variable will contain all the valid edges we have to include in our graph.
final_edges = []
#These are the edges between robot start position and expanded obstacle boundary corner points
edges_robot = plot_edges(Robot,corners_bound,final_contours_obs)
#These are the edges between goal and expanded obstacle boundary corner points
edges_goal = plot_edges(Goal,corners_bound,final_contours_obs)

edge_between = plot_edges(Robot,[Goal],final_contours_obs)

final_edges = final_edges + edge_between
#If either of the edge lists are empty this means that the robot or the goal position is bounded inside and obstacle hence in
#this case our goal is not reachable and we stop further processing
if edges_robot==[] or edges_goal==[]:
    print("___________________THE ROBOT WILL NOT REACH GOAL______________________")
else:
    #We calculate the edges between expanded obstacle corner points
    final_edges = final_edges+edges_robot
    final_edges = final_edges+edges_goal
    for i in corners_bound:
        edges = plot_edges(i,corners_bound,final_contours_obs)
        final_edges = final_edges+edges


    nodes = [Robot,Goal]
    for i in corners_bound:
        nodes.append(i)

##    you can observe the final edges and nodes for graph
##    print(final_edges)
##    print(nodes)



    #Shortest Path Finding Calculations using dikstra's algorithm
    #Here we are using graph functions from the imported graph file
    #that we have implemented separately and it contains all the ne,
    #cessary graph functions including dikstra and shortest path

    #Here we conver final edges and nodes into tuples beacsue in py,
    #then we can not feed lists as keys of dictionary
        
    nodes = [tuple(i) for i in nodes]
    final_edges = [(tuple(i[0]),tuple(i[1]),i[2]) for i in final_edges]
    Robot = tuple([sx,sy])
    Goal  = tuple([gx,gy])


    #Graph Creation
    G = {}
    G = graphs.addNodes(G,nodes)
    G = graphs.addEdges(G,final_edges,False)

    #Shortest path calculation using respective functions in graphs
    lst = graphs.shortestPath(G,Robot,Goal)

    #Once we have obtained the shortest path, we then plot it using plt
    for i in lst:
        x_values = [i[0][0], i[1][0]]
        y_values = [i[0][1], i[1][1]]
        plt.plot(x_values, y_values, "-r")
        plt.pause(1)

    #Showing whatever we have obtained on plt
    plt.show()
