'''
@author Keyur Borad
dir id: kborad
ENPM661 Project2
'''
#https://github.com/keyurborad5/Path-Planning-Project-2.git
import heapq as hq
import numpy as np
import time
import cv2

start_time = time.time()  
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

#Creating different walls
for i in range(0,5):
    canvas[i][0:1201][:]=[255,255,255]
for i in range(495,500):
    canvas[i][0:1201][:]=[255,255,255]

for i in range(0,500):
    canvas[i][0:5][:]=[255,255,255]
    canvas[i][1196:1201][:]=[255,255,255]


print("Generated with Canvas shape:", canvas.shape)


#using Sets and creating Obstacle space
Obstacle_Space=set()
for i in range(canvas.shape[1]):
    for j in range(canvas.shape[0]):
        if np.array_equal(canvas[j][i], [255, 255, 255]):
            Obstacle_Space.add((i,-j+500))

# def get_input(Obstacle_Space):
while True:
    init_x=int(input(" Enter initial x values: "))
    init_y=int(input(" Enter initial y values: "))
    if {(init_x,init_y)}.issubset(Obstacle_Space)==False and 5<init_x<1195 and 5<init_y<495:
        break
    else: 
        print(" enter valid coordinates \n")
print("Initial coordinates: ",init_x,init_y)

while True:
    final_x=int(input(" Enter final x values: "))
    final_y=int(input(" Enter final y values: "))
    if {(final_x,final_y)}.issubset(Obstacle_Space)==False  and 5<final_x<1195 and 5<final_y<495:
        break
    else: 
        print(" enter valid coordinates \n")

print("Final coordinates",final_x,final_y)

# defining action steps
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
listt=[move_up,move_right,move_down,move_left,move_up_left,move_down_left,move_down_right,move_up_right]

start_node=(cost_to_come,node_index,parent_node_index,(init_x,init_y))
hq.heappush(open_nodes,start_node)

#Flag for goal node found
found_goal=False
def visited_node(node):
    visited_nodes.update({node[1]:node[2]})
    named_node.update({node[1]:node[3]})

while (open_nodes and found_goal!=True):
   
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
        #for exiting the loop
        found_goal=True
        ##Backtracking after exiting this while loop
    # If not following code is executed
    else:
        
        parent_node_index=current_node[2]
        cost_to_come=current_node[0]
        #iterating all eight actions
        for action in listt:
            action_output=action(current_node)
            # Checking the explored node in the canvas region
            if action_output[0]<=1194 and action_output[0]>=5 and action_output[1]<=494 and action_output[1]>=5:

                new_node=action_output[:2]
                action_c2c=action_output[2]
                #Checking if the explored node is not in closed set
                if ({new_node}.issubset(closed_set)==False):
                
                    #Checking if the explored node is not part of obstacle space set
                    if {new_node}.issubset(Obstacle_Space)==False:
                            open_node_flag=False
                            #Iterating explored node through open list
                            for i in range(len(open_nodes)):
                                    if open_nodes[i][3]==new_node:
                                        open_node_flag=True
                                        # updateing the node cost if cost to come is smaller than existing cost to come
                                        if open_nodes[i][0]>(cost_to_come+action_c2c):
                                            open_nodes[i]=(cost_to_come+action_c2c,open_nodes[i][1],current_node[1],open_nodes[i][3])
                                            break
                                        else:
                                            break
                            # if explored node not found in open list then added
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

    out = cv2.VideoWriter('keyur_borad_project2.mp4',cv2.VideoWriter_fourcc(*'mp4v'), 166, (1200,500))

    canvas_1=canvas
    count=0
    while closed_nodes:
        plot=hq.heappop(closed_nodes)
        #plotting initial point
        cv2.circle(canvas_1, (init_x,500-init_y), 4, [0,0,252], -1)
        #plotting final point
        cv2.circle(canvas_1, (final_x,500-final_y), 4, [0,255,0], -1)
        #PLotting the explored node
        cv2.circle(canvas_1, (plot[3][0],500-plot[3][1]), 1, [250,0,0], -1)
        # counter for speeding up the display
        if count%100==0:
            out.write(canvas_1)
            cv2.imshow('window',canvas_1)
            cv2.waitKey(1)
        count+=1

    # Visualising Backtracking 
    parent= current_node[2]
    # print(parent)
    backtrack=[]
    while parent!=0:
        backtrack.append(named_node[parent])
        parent=visited_nodes[parent]
    backtrack.reverse()
    for coord in (backtrack):
        #ploting the backtracked points
        cv2.circle(canvas_1, (coord[0],500-coord[1]), 1, [0,250,0], -1)
        out.write(canvas_1)
        cv2.imshow('window',canvas_1)
        cv2.waitKey(1)
    cv2.waitKey(0)
    out.release()
    cv2.destroyAllWindows()
end_time = time.time()
print(f"The runtime of my program is {end_time - start_time} seconds.")

   