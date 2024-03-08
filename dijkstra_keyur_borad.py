# Consolidating the above steps intp one as building the canvas with ostacles
import numpy as np
import cv2

# Define canvas size
canvas_height = 500
canvas_width = 1200

# Create canvas
canvas = np.zeros((canvas_height, canvas_width, 3), dtype=np.uint8)


# Define side length of the hexagon
side_length = 155


# Calculate center of the canvas
center_x = 650
center_y = 250

# Calculate coordinates of the vertices
angles = np.linspace(np.pi/6,  2*np.pi+np.pi/6, 7)[:-1]  # Angles for each vertex
x = np.int32(center_x + side_length * np.cos(angles))  # X coordinates
y = np.int32(center_y + side_length * np.sin(angles))  # Y coordinates

# Reshape the coordinates for OpenCV's fillPoly function
pts = np.array([x, y], np.int32).T.reshape((-1, 1, 2))

# Draw the hexagon on the canvas
cv2.fillPoly(canvas, [pts], color=(255, 255, 255))  # Fill the hexag

#Creating different rectangular obstacles
for i in range(406):
    canvas[i][95:181][:]=[255,255,255]
for i in range(95,500):
    canvas[i][270:356][:]=[255,255,255]
for i in range(45,131):
    canvas[i][895:1106][:]=[255,255,255]
for i in range(120,381):
    canvas[i][1015:1106][:]=[255,255,255]
for i in range(370,456):
    canvas[i][895:1106][:]=[255,255,255]

print("Canvas shape:", canvas.shape)
# cv2.imshow('window',canvas)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

#using Sets and creating Obstacle space
Obstacle_Space=set()
for i in range(canvas.shape[1]):
    for j in range(canvas.shape[0]):
        if np.array_equal(canvas[j][i], [255, 255, 255]):
            Obstacle_Space.add((i,-j+500))


while True:
    init_x=int(input(print(" Enter initial x values: ")))
    init_y=int(input(print(" Enter initial y values: ")))
    if {(init_x,init_y)}.issubset(Obstacle_Space)==False:
        break
    else: 
        print(" enter valid coordinare \n")
print(init_x,init_y)

while True:
    final_x=int(input(print(" Enter final x values: ")))
    final_y=int(input(print(" Enter final y values: ")))
    if {(final_x,final_y)}.issubset(Obstacle_Space)==False:
        break
    else: 
        print(" enter valid coordinare \n")

print(final_x,final_y)

import heapq as hq
def move_up(node):
    return node[3][0],(node[3][1]+1),1
def move_up_right(node):
    return (node[3][0]+1),(node[3][1]+1),1.4
def move_right(node):
    return (node[3][0]+1),(node[3][1]),1
def move_down_right(node):
    return (node[3][0]+1),(node[3][1]-1),1.4
def move_down(node):
    return (node[3][0]),(node[3][1]-1),1
def move_down_left(node):
    return (node[3][0]-1),(node[3][1]-1),1.4
def move_left(node):
    return (node[3][0]-1),(node[3][1]),1
def move_up_left(node):
    return (node[3][0]-1),(node[3][1]+1),1.4

listt=[move_up,move_right,move_down,move_left,move_up_left,move_down_left,move_down_right,move_up_right]

# Creating open and closed lists

open_nodes=[]
closed_nodes=[]
visited_nodes={}
named_node={}
hq.heapify(open_nodes)
hq.heapify(closed_nodes)
closed_set=set()
closed_set1=set()

c2c=0
node_index=1
parent_node_index=0
cost_to_come=0
start_node=(cost_to_come,node_index,parent_node_index,(init_x,init_y))
hq.heappush(open_nodes,start_node)

#Flag for goal node found
found_goal=False
def visited_node(node):
    visited_nodes.update({node[1]:node[2]})
    named_node.update({node[1]:node[3]})

while (open_nodes and found_goal!=True):
    ######################
    #Checking termination of while loop
    # print(hq.heappop(open_nodes))
    #Checking termination of while loop
    # i+=1
    # print(i)
    # if i==10:
    #     found_goal=True
    ######################
    # Fetching the node with lowest c2c from openlist
    current_node=hq.heappop(open_nodes)
    # Pushing the node into closed list
    hq.heappush(closed_nodes,current_node)
    visited_node(current_node)
    closed_set.add(current_node[3])
    closed_set1.add(current_node)
    


    # Verifying the fetched node is goal node or not
    if current_node[3]==(final_x,final_y):
        print(current_node)
        found_goal=True
        ##Backtracking after exiting this while loop
    # If not following code is executed
    else:
        
        parent_node_index=current_node[2]
        cost_to_come=current_node[0]
        for action in listt:
            action_output=action(current_node)
            if action_output[0]<=1194 and action_output[0]>=5 and action_output[1]<=494 and action_output[1]>=5:

                new_node=action_output[:2]
                action_c2c=action_output[2]
                if ({new_node}.issubset(closed_set)==False):
                
                
                    if {new_node}.issubset(Obstacle_Space)==False:
                            open_node_flag=False
                            for i in range(len(open_nodes)):
                                    if open_nodes[i][3]==new_node:
                                        open_node_flag=True
                                        if open_nodes[i][0]>(cost_to_come+action_c2c):
                                            open_nodes[i]=(cost_to_come+action_c2c,open_nodes[i][1],current_node[1],open_nodes[i][3])
                                            break
                                        else:
                                            break
                            
                            if open_node_flag==False:
                                node_index+=1
                                hq.heappush(open_nodes,(cost_to_come+action_c2c,node_index,current_node[1],new_node))
                            else:#FOr Open_node_falg 
                                pass
                    else:
                        continue    
                else:#For closed_node_flag condition
                    continue        

if found_goal==True:
    #Backtracking
    #Visualising the exploration of node

    # out = cv2.VideoWriter('keyur_borad_project2.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 10, (1200,500))

    canvas_1=canvas
    count=0
    while closed_nodes:
        plot=hq.heappop(closed_nodes)
        
        cv2.circle(canvas_1, (init_x,500-init_y), 4, [0,0,252], -1)
        cv2.circle(canvas_1, (final_x,500-final_y), 4, [0,0,250], -1)
        cv2.circle(canvas_1, (plot[3][0],500-plot[3][1]), 1, [250,0,0], -1)
        
        if count%100==0:
            # out.write(canvas_1)
            cv2.imshow('window',canvas_1)
            cv2.waitKey(1)
        count+=1

    # Visualising Backtracking 
    parent= current_node[2]
    print(parent)
    backtrack=[]
    while parent!=0:
        backtrack.append(named_node[parent])
        parent=visited_nodes[parent]
    backtrack.reverse()
    for coord in (backtrack):
        cv2.circle(canvas_1, (coord[0],500-coord[1]), 1, [0,250,0], -1)
        # out.write(canvas_1)
        cv2.imshow('window',canvas_1)
        cv2.waitKey(1)
    cv2.waitKey(0)
    # out.release()
    cv2.destroyAllWindows()