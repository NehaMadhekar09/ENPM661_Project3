#================================================================================================================================
# Github link of the code repository:
# 
# 
#================================================================================================================================
import pygame
import numpy as np
from collections import deque
from shapely.geometry import LineString
import time
import math
from pygame import gfxdraw
#================================================================================================================================
# Functions for creating obstacle space 

def CreateObstacles(canvas,clerarance,robot_radius):
    distance= clerarance+robot_radius

    # For Border clerance
    canvas.fill((255,255,255))
    pygame.draw.rect(canvas, (0,0,0), pygame.Rect(distance, distance, 600-2*distance, 200-2*distance))
    wall_clearance=[(distance,distance),(600-distance,distance),(600-distance,200-distance),(distance,200-distance)]

    # For Rectangular Obstacles
    pygame.draw.rect(canvas, (255,255,255), pygame.Rect(150, 0, 15, 125).inflate(2*distance,2*distance))
    pygame.draw.rect(canvas, (80,208,255), pygame.Rect(150, 0, 15, 125))
    pygame.draw.rect(canvas, (255,0,0), pygame.Rect(150, 0, 15, 125),2)
    rectangle_1=[(150-distance,0-distance),(165+distance,0-distance),(165+distance,125+distance),(150-distance,125+distance)]

    pygame.draw.rect(canvas, (255,255,255), pygame.Rect(250, 75, 15, 125).inflate(2*distance,2*distance))
    pygame.draw.rect(canvas, (80,208,255), pygame.Rect(250, 75, 15, 125))
    pygame.draw.rect(canvas, (255,0,0), pygame.Rect(250, 75, 15, 125),2)
    rectangle_2=[(250-distance,75-distance),(265+distance,75-distance),(265+distance,200+distance),(250-distance,200+distance)]

    # For Circular Obstacle
    pygame.draw.circle(canvas, (255,255,255), (400,90), 50+distance)
    pygame.draw.circle(canvas, (80,208,255), (400,90), 50)
    pygame.draw.circle(canvas, (255,0,0), (400,90), 50,2)
    circle=[400,90,50+distance]

    pygame.display.flip()

    return wall_clearance,rectangle_1,rectangle_2,circle
    

#================================================================================================================================
# Functions used for A* Algorithm

def ActionVel(curr_node_xytheta,UL,UR,wall_clearance,rectangle_1,rectangle_2,circle):
    t = 0
    r = 3.8
    L = 16
    dt = 0.1
    Xn=curr_node_xytheta[0]
    Yn=curr_node_xytheta[1]
    Thetan = 3.14 * curr_node_xytheta[2] / 180
    going_in_obstacle=False

# Xi, Yi,Thetai: Input point's coordinates
# Xs, Ys: Start point coordinates for plot function
# Xn, Yn, Thetan: End point coordintes
    D=0
    while t< 0.5:
        t = t + dt
        Xs = Xn
        Ys = Yn
        Xn = Xs+0.5*r * (UL + UR) * math.cos(Thetan) * dt
        Yn = Ys+0.5*r * (UL + UR) * math.sin(Thetan) * dt
        Thetan += (r / L) * (UR - UL) * dt
        D=D+ math.sqrt(math.pow((0.5*r * (UL + UR) * math.cos(Thetan) * dt),2)+math.pow((0.5*r * (UL + UR) * math.sin(Thetan) * dt),2))
        if(IsNodeInObstacleSpace((Xn,Yn),wall_clearance,rectangle_1,rectangle_2,circle)):
            going_in_obstacle=True

    Thetan = 180 * (Thetan) / 3.14
    return Xn, Yn, Thetan, D,going_in_obstacle


def plot_curve(canvas,Xi,Yi,Thetai,UL,UR,color=(0, 0, 255)):
    t = 0
    r = 3.8
    L = 16
    dt = 0.1
    Xn=Xi
    Yn=Yi
    Thetan = 3.14 * Thetai / 180

# Xi, Yi,Thetai: Input point's coordinates
# Xs, Ys: Start point coordinates for plot function
# Xn, Yn, Thetan: End point coordintes
    D=0
    while t<0.5:
        t = t + dt
        Xs = Xn
        Ys = Yn
        Xn += 0.5*r * (UL + UR) * math.cos(Thetan) * dt
        Yn += 0.5*r * (UL + UR) * math.sin(Thetan) * dt
        Thetan += (r / L) * (UR - UL) * dt
        D=D+ math.sqrt(math.pow((0.5*r * (UL + UR) * math.cos(Thetan) * dt),2)+math.pow((0.5*r * (UL + UR) * math.sin(Thetan) * dt),2))
        pygame.draw.line(canvas, color, (Xs, Ys), (Xn, Yn))
        pygame.display.flip()
        
    Thetan = 180 * (Thetan) / 3.14
    return Xn, Yn, Thetan, D


# Using half planes to find whether a point is in a given obstacle
def IsPointOnOneSideOfLineSet(point,pointset):
    is_on_right_side=[]
    for i in range(len(pointset)-1):
        x1=pointset[i][0]
        y1=pointset[i][1]
        x2=pointset[i+1][0]
        y2=pointset[i+1][1]
        d=(point[0]-x1)*(y2-y1)-(point[1]-y1)*(x2-x1)
        if d >= 0:
            is_on_right_side.append(True)
        else:
            is_on_right_side.append(False)

    d=(point[0]-pointset[-1][0])*(pointset[0][1]-pointset[i][1])-(point[1]-pointset[-1][1])*(pointset[0][0]-pointset[-1][0])
    if d >= 0:
        is_on_right_side.append(True)
    else:
        is_on_right_side.append(False)

    isAllSame = all(ele == is_on_right_side[0] for ele in is_on_right_side)
    return isAllSame

# Determines whether the given node is in any of the given obstacle shapes
def IsNodeInObstacleSpace(point,wall_clearance,rectangle_1,rectangle_2,circle):
    is_in_obstacle_space=False
    if not IsPointOnOneSideOfLineSet(point,wall_clearance):
        is_in_obstacle_space= True
    elif IsPointOnOneSideOfLineSet(point,rectangle_1):
        is_in_obstacle_space= True
    elif IsPointOnOneSideOfLineSet(point,rectangle_2):
        is_in_obstacle_space= True
    elif ComputeDistance(point,(circle[0],circle[1])) <= circle[2]:
        is_in_obstacle_space= True
    return is_in_obstacle_space

# Determines whether a move at given angle is possible 
def IsActionPossible(current_node,RPM_left, RPM_right,closed_queue,closed_Q_matrix,wall_clearance,rectangle_1,rectangle_2,circle):
    x_n,y_n,theta_n,action_cost,going_in_obstacle=ActionVel(current_node[1][5],RPM_left,RPM_right,wall_clearance,rectangle_1,rectangle_2,circle)
    if theta_n < 0:
        theta_n=360+theta_n
    if theta_n >= 360:
        theta_n=theta_n-360
    new_node_xytheta=(x_n,y_n,theta_n)
    if 0<=x_n<600 and 0<=y_n<200:
        if not going_in_obstacle:
            new_node_xythetaD=DiscretizeNodeCoordinates((new_node_xytheta[0],new_node_xytheta[1]))   
            if closed_Q_matrix[int(new_node_xythetaD[1]/5)][int(new_node_xythetaD[0]/5)]==0:
                return [True,new_node_xythetaD, new_node_xytheta,action_cost] 
    return[False]

# Computes Euclidean distance between given two points
def ComputeDistance(node_xy,goal_node_xytheta):
    return np.sqrt((goal_node_xytheta[0]-node_xy[0])*(goal_node_xytheta[0]-node_xy[0])+(goal_node_xytheta[1]-node_xy[1])*(goal_node_xytheta[1]-node_xy[1]))
    
# Updates open list 
def UpdateOpenList(current_node,new_node_xythetaD,new_node_xytheta,action_cost,open_queue,open_matrix,index,goal_node_xy,action):
    cost_to_come=current_node[1][1] + action_cost
    if open_matrix[int(new_node_xythetaD[1]/5)][int(new_node_xythetaD[0]/5)]==1:
        if(cost_to_come < open_queue[new_node_xythetaD][1]):
            id=open_queue[new_node_xythetaD][3]
            cost_to_go=open_queue[new_node_xythetaD][2]
            total_cost=cost_to_come+cost_to_go
            open_queue[new_node_xythetaD]=(total_cost,cost_to_come,cost_to_go, id, current_node[1][3],new_node_xytheta,action)
    else:
        cost_to_go=ComputeDistance(new_node_xytheta,goal_node_xy)
        total_cost=cost_to_come+cost_to_go
        open_queue[new_node_xythetaD]=(total_cost,cost_to_come,cost_to_go,index,current_node[1][3],new_node_xytheta,action)
    # new_node=open_queue[new_node_xythetaD]
    # return new_node

# This function generates path by backtracking.
def BacktrackPath(closed_queue,goal_xy_theta,goal_parent_index,goal_action):
    path=deque()
    path.appendleft((goal_xy_theta,goal_action))
    parent_index=goal_parent_index
    index=1
    while(index!=0):
        for key,value in closed_queue.items():
            if parent_index==value[3]:
                path.appendleft((value[5],value[6]))
                parent_index=value[4]
                index=value[3]
    return path

# Discretize node coordinates or find the node region
def DiscretizeNodeCoordinates(actual_coordinates,thresholds=(5,5)):
    discretized_coordinates=[]
    for i in range(len(actual_coordinates)):
        remainder=actual_coordinates[i]%thresholds[i]
        answer=int(actual_coordinates[i]/thresholds[i])
        if(remainder <= thresholds[i]/2):
            discretized_coordinates.append(answer*thresholds[i])
        else:
            discretized_coordinates.append((answer+1)*thresholds[i])
    return (discretized_coordinates[0],discretized_coordinates[1])

# Checks whether a goal node matches with the current node considering tolerance
def IsGoalReached(node_xy, goal_node_xy,goal_threshold):
    goal_reached=False
    distance=ComputeDistance(node_xy,goal_node_xy)
    if distance<goal_threshold:
        goal_reached=True
    return goal_reached

# A* Algorithm
def A_Star_Algorithm(start_node_xytheta, goal_node_xy,RPM1,RPM2,canvas,wall_clearance,rectangle_1,rectangle_2,circle):
    visited_nodes=[]
    open_Q = {}
    open_Q_matrix=np.zeros(shape=(40,120))
    closed_Q={}
    closed_Q_matrix=np.zeros(shape=(40,120))
    actions=[]
    
    # creating tuple with total cost,cost to come,cost to go,node index, parent node index and tuple with actual position(x,y,theta)
    # Adding it to the dictionary with key value as descritized node coordinates(x_d,y_d,theta_d)
    start_node_xythetaD=DiscretizeNodeCoordinates((start_node_xytheta[0],start_node_xytheta[1]))
    open_Q[start_node_xythetaD]=(0,0,0,0,0,start_node_xytheta,(0,0))
    open_Q_matrix[int(start_node_xythetaD[1]/5)][int(start_node_xythetaD[0]/5)]=1
    visited_nodes.append((0,start_node_xytheta,(0,0)))
    actions.append((0,0))

    goal_threshold=5
    index=1
    # Run for maximum 100000 iterations
    isConverged=True
    while True:
        if index > 100000:
            print("Failed to reach the goal.Check whether the goal is in reachable area. Max Iterations Reached.")
            isConverged=False
            break
        open_Q=dict(sorted(open_Q.items(),key=lambda x:x[1][0],reverse = True))
        current_node=open_Q.popitem()
        closed_Q[current_node[0]]=current_node[1]
        closed_Q_matrix[int(current_node[0][1]/5)][int(current_node[0][0]/5)]=1
        
        # Check if action1 [0, RPM1] possible
        Action_1=IsActionPossible(current_node,0,RPM1,closed_Q,closed_Q_matrix,wall_clearance,rectangle_1,rectangle_2,circle)
        if(Action_1[0]):
            new_node_xythetaD=Action_1[1]
            new_node_xytheta=Action_1[2]
            action_cost=Action_1[3]
            UpdateOpenList(current_node,new_node_xythetaD,new_node_xytheta,action_cost,open_Q,open_Q_matrix,index,goal_node_xy,(0,RPM1))
            open_Q_matrix[int(new_node_xythetaD[1]/5)][int(new_node_xythetaD[0]/5)]=1
            visited_nodes.append((current_node[1][3],new_node_xytheta,(0,RPM1)))
            plot_curve(canvas,current_node[1][5][0],current_node[1][5][1],current_node[1][5][2],0,RPM1)
            index+=1
            if IsGoalReached(new_node_xytheta,goal_node_xy,goal_threshold):
                goal_parent_index=current_node[1][3]
                goal_action=(0,RPM1)
                break
        
        # Check if action2 [RPM1,0] possible
        Action_2=IsActionPossible(current_node,RPM1,0,closed_Q,closed_Q_matrix,wall_clearance,rectangle_1,rectangle_2,circle)
        if(Action_2[0]):
            new_node_xythetaD=Action_2[1]
            new_node_xytheta=Action_2[2]
            action_cost=Action_2[3]
            UpdateOpenList(current_node,new_node_xythetaD,new_node_xytheta,action_cost,open_Q,open_Q_matrix,index,goal_node_xy,(RPM1,0))
            open_Q_matrix[int(new_node_xythetaD[1]/5)][int(new_node_xythetaD[0]/5)]=1
            visited_nodes.append((current_node[1][3],new_node_xytheta,(RPM1,0)))
            plot_curve(canvas,current_node[1][5][0],current_node[1][5][1],current_node[1][5][2],RPM1,0)
            index+=1
            if IsGoalReached(new_node_xytheta,goal_node_xy,goal_threshold):
                goal_parent_index=current_node[1][3]
                goal_action=(RPM1,0)
                break

         # Check if action3 [RPM1,RPM1] possible
        Action_3=IsActionPossible(current_node,RPM1,RPM1,closed_Q,closed_Q_matrix,wall_clearance,rectangle_1,rectangle_2,circle)
        if(Action_3[0]):
            new_node_xythetaD=Action_3[1]
            new_node_xytheta=Action_3[2]
            action_cost=Action_3[3]
            UpdateOpenList(current_node,new_node_xythetaD,new_node_xytheta,action_cost,open_Q,open_Q_matrix,index,goal_node_xy,(RPM1,RPM1))
            open_Q_matrix[int(new_node_xythetaD[1]/5)][int(new_node_xythetaD[0]/5)]=1
            visited_nodes.append((current_node[1][3],new_node_xytheta,(RPM1,RPM1)))
            plot_curve(canvas,current_node[1][5][0],current_node[1][5][1],current_node[1][5][2],RPM1,RPM1)
            index+=1
            if IsGoalReached(new_node_xytheta,goal_node_xy,goal_threshold):
                goal_parent_index=current_node[1][3]
                goal_action=(RPM1,RPM1)
                break

        # Check if action4 [0, RPM2] possible
        Action_4=IsActionPossible(current_node,0,RPM2,closed_Q,closed_Q_matrix,wall_clearance,rectangle_1,rectangle_2,circle)
        if(Action_4[0]):
            new_node_xythetaD=Action_4[1]
            new_node_xytheta=Action_4[2]
            action_cost=Action_4[3]
            UpdateOpenList(current_node,new_node_xythetaD,new_node_xytheta,action_cost,open_Q,open_Q_matrix,index,goal_node_xy,(0,RPM2))
            open_Q_matrix[int(new_node_xythetaD[1]/5)][int(new_node_xythetaD[0]/5)]=1
            visited_nodes.append((current_node[1][3],new_node_xytheta,(0,RPM2)))
            plot_curve(canvas,current_node[1][5][0],current_node[1][5][1],current_node[1][5][2],0,RPM2)
            index+=1
            if IsGoalReached(new_node_xytheta,goal_node_xy,goal_threshold):
                goal_parent_index=current_node[1][3]
                goal_action=(0,RPM2)
                break
        
        # Check if action5 [RPM2,0] possible
        Action_5=IsActionPossible(current_node,RPM2,0,closed_Q,closed_Q_matrix,wall_clearance,rectangle_1,rectangle_2,circle)
        if(Action_5[0]):
            new_node_xythetaD=Action_5[1]
            new_node_xytheta=Action_5[2]
            action_cost=Action_5[3]
            UpdateOpenList(current_node,new_node_xythetaD,new_node_xytheta,action_cost,open_Q,open_Q_matrix,index,goal_node_xy,(RPM2,0))
            open_Q_matrix[int(new_node_xythetaD[1]/5)][int(new_node_xythetaD[0]/5)]=1
            visited_nodes.append((current_node[1][3],new_node_xytheta,(RPM2,0)))
            plot_curve(canvas,current_node[1][5][0],current_node[1][5][1],current_node[1][5][2],RPM2,0)
            index+=1
            if IsGoalReached(new_node_xytheta,goal_node_xy,goal_threshold):
                goal_parent_index=current_node[1][3]
                goal_action=(RPM2,0)
                break

         # Check if action6 [RPM2,RPM2] possible
        Action_6=IsActionPossible(current_node,RPM2,RPM2,closed_Q,closed_Q_matrix,wall_clearance,rectangle_1,rectangle_2,circle)
        if(Action_6[0]):
            new_node_xythetaD=Action_6[1]
            new_node_xytheta=Action_6[2]
            action_cost=Action_6[3]
            UpdateOpenList(current_node,new_node_xythetaD,new_node_xytheta,action_cost,open_Q,open_Q_matrix,index,goal_node_xy,(RPM2,RPM2))
            open_Q_matrix[int(new_node_xythetaD[1]/5)][int(new_node_xythetaD[0]/5)]=1
            visited_nodes.append((current_node[1][3],new_node_xytheta,(RPM2,RPM2)))
            plot_curve(canvas,current_node[1][5][0],current_node[1][5][1],current_node[1][5][2],RPM2,RPM2)
            index+=1
            if IsGoalReached(new_node_xytheta,goal_node_xy,goal_threshold):
                goal_parent_index=current_node[1][3]
                goal_action=(RPM2,RPM2)
                break
        
        # Check if action7 [RPM1,RPM2] possible
        Action_7=IsActionPossible(current_node,RPM1,RPM2,closed_Q,closed_Q_matrix,wall_clearance,rectangle_1,rectangle_2,circle)
        if(Action_7[0]):
            new_node_xythetaD=Action_7[1]
            new_node_xytheta=Action_7[2]
            action_cost=Action_7[3]
            UpdateOpenList(current_node,new_node_xythetaD,new_node_xytheta,action_cost,open_Q,open_Q_matrix,index,goal_node_xy,(RPM1,RPM2))
            open_Q_matrix[int(new_node_xythetaD[1]/5)][int(new_node_xythetaD[0]/5)]=1
            visited_nodes.append((current_node[1][3],new_node_xytheta,(RPM1,RPM2)))
            plot_curve(canvas,current_node[1][5][0],current_node[1][5][1],current_node[1][5][2],RPM1,RPM2)
            index+=1
            if IsGoalReached(new_node_xytheta,goal_node_xy,goal_threshold):
                goal_parent_index=current_node[1][3]
                goal_action=(RPM1,RPM2)
                break
        
        # Check if action8 [RPM2,RPM1] possible
        Action_8=IsActionPossible(current_node,RPM2,RPM1,closed_Q,closed_Q_matrix,wall_clearance,rectangle_1,rectangle_2,circle)
        if(Action_8[0]):
            new_node_xythetaD=Action_8[1]
            new_node_xytheta=Action_8[2]
            action_cost=Action_8[3]
            UpdateOpenList(current_node,new_node_xythetaD,new_node_xytheta,action_cost,open_Q,open_Q_matrix,index,goal_node_xy,(RPM2,RPM1))
            open_Q_matrix[int(new_node_xythetaD[1]/5)][int(new_node_xythetaD[0]/5)]=1
            visited_nodes.append((current_node[1][3],new_node_xytheta,(RPM2,RPM1)))
            plot_curve(canvas,current_node[1][5][0],current_node[1][5][1],current_node[1][5][2],RPM2,RPM1)
            index+=1
            if IsGoalReached(new_node_xytheta,goal_node_xy,goal_threshold):
                goal_parent_index=current_node[1][3]
                goal_action=(RPM2,RPM1)
                break
            
        if(len(open_Q)==0):
            print("Failed to find solution")
            isConverged=False
            break

    if(isConverged):
        # Backtrach path
        path=BacktrackPath(closed_Q,goal_node_xy,goal_parent_index,goal_action)
        return [isConverged,path]

    return [isConverged]


#================================================================================================================================
# Function Calls

clearance=int(input("Enter the clearance value: "))

robot_radius=10.5 #in cm

pygame.init()
canvas=pygame.display.set_mode((600,200))

wall_clearance,rectangle_1,rectangle_2,circle=CreateObstacles(canvas,clearance,robot_radius)

# Get correct inputs from user
while True:
    x=int(input("Enter x coordinate of the start node: "))
    y=int(input("Enter y coordinate of the start node: "))
    start_x=x+50
    start_y=100-y
    start_theta=int(input("Enter the orientation of the robot in degrees at the start node(It should be multiple of 30): "))
    if (0<=start_x<600 and 0<=start_y<200):
        # Check whether the inputs are not in collision space
        is_node_in_obstacle_space=IsNodeInObstacleSpace((start_x,start_y),wall_clearance,rectangle_1,rectangle_2,circle)
        if not is_node_in_obstacle_space:
            break
        else:
            print("The start node lie in the obstacle space. Enter the values again")
    

while True:
    x=int(input("Enter x coordinate of the goal node: "))
    y=int(input("Enter y coordinate of the goal node: "))
    goal_x=x+50
    goal_y=100-y
    if (0<=goal_x<600 and 0<=goal_y<200):
        # Check whether the inputs are not in collision space
        is_node_in_obstacle_space=IsNodeInObstacleSpace((goal_x,goal_y),wall_clearance,rectangle_1,rectangle_2,circle)
        if not is_node_in_obstacle_space:
            break
        else:
            print("The goal node lie in the obstacle space. Enter the values again")

RPM1=int(input("Enter the value of RPM1 (roughly between 5 to 20)"))
RPM2=int(input("Enter the value of RPM2 (roughly between 5 to 20)"))

# Highlight start and end point
pygame.draw.circle(canvas, (255,0,0), (start_x,start_y),3,1)
pygame.draw.circle(canvas, (255,0,0),(goal_x,goal_y),3,1)
pygame.display.flip()

print("A* in progress...")

# Convert start and goal node to correct coordinate system
if start_theta < 0:
    start_theta=360+start_theta
if start_theta >= 360:
    start_theta=start_theta-360

start_time = time.time()

result=A_Star_Algorithm((start_x,start_y,start_theta),(goal_x,goal_y),RPM1,RPM2,canvas,wall_clearance,rectangle_1,rectangle_2,circle)

end_time = time.time()
print("Total time taken in seconds: ",end_time - start_time)

if result[0]:
    path=result[1]
    # Show optimal path
    pygame.draw.circle(canvas, (255,255,0), (path[0][0][0],path[0][0][1]),1)
    for i in range (1,len(path)):
        pygame.draw.circle(canvas, (255,255,0), (path[i][0][0],path[i][0][1]),1)
        plot_curve(canvas,path[i-1][0][0],path[i-1][0][1],path[i-1][0][2],path[i][1][0],path[i][1][1],(255,255,0))
        pygame.display.flip()


running = True
  
while running:
    for event in pygame.event.get():
        # Check for QUIT event      
        if event.type == pygame.QUIT:
            running = False

#================================================================================================================================