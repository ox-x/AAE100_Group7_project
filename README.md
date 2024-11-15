## Presentation Link :link:
[![image](https://img.shields.io/badge/YouTube-FF0000?style=for-the-badge&logo=youtube&logoColor=white)](https://www.youtube.com)


#Task 1 (4 person)

 Yu Mengzhi 24102853d (Step 1)

Liang Chi Ho 24082636d (Step 1)
                            
Deng Yuchun Harry 24112212d (Step 2)

Xiao Suen 24107803d  (Step 2)

#Task 2 (2 person)

Huang Liangtai 24133635D

Yu Qinxiang 24128609D


#Task 3 (2 person)

Liu Cheuk Kei Yuki 24123003d (Step 1)

He Yinxi 24109157d (Step 2)

#Additional Task

Yu Mengzhi 24102853d 

# AAE1001-Group 7-Project

## TABLE OF CONTENTS
- [Background of Path Planning to Aviation Engineering :flight_departure:	](#Background-of-Path-Planning-to-Aviation-Engineering-flight_departure)
- [Theory of Path Planning Algorithm :](#Theory-of-Path-Planning-Algorithm-)
- [How the project simulate the Path-Planning process in Aviation](#How-the-project-simulate-the-Path-Planning-process-in-Aviation)
  - [Task 1](#Task-1)
  - [Task 2](#Task-2)
  - [Task 3](#Task-3)
  - [Additional Task](#Additional-Task)
- [Individual reflective essay 📜](#Individual-reflective-essay-)



## Background of Path Planning to Aviation Engineering :flight_departure:	
Path planning (also known as the navigation problem),can be simply said as its literal meaning, implies
moving the sepcific object from the source to destination.The term is
widely used in robotics and computer games, espcailly in Aviation :airplane:.

Aa aviation has become important,if not essential in the transportation system,Path Planning is needed in 
Private pilots do the path plan before the flight to make sure the
navigation aid is available
For ATC near airports, collaborative path planning is required to make
the best use of the crowded airspace
Commercial pilot follow the path that plan based on different cost index
designed by airlines.


## Theory of Path Planning Algorithm :
A* search algorithm is the pathfinding algorithm which we are going to use.Simply put,it's a path finder by "cost" calculation,comoparsion.Three varbles included in the algorithm to find the "cost".To begin with, the f(x,y)=g(x,y)+h(x,y) ,which is the total cost of a neighboring node (𝑥, 𝑦).g(x,y) represents the exact cost of the path from the Start node to node (x,y) and the h(x,y) represents the
heuristic estimated cost from node (x,y) to the Goal node.

The algorithm starts with testing each possible nodes one-by-one from Start node,and record its path distance between Start node and it as "g".Also, recording the path distance between Goal node and it as "h". Afterall, calculate all nodes' "f" in the open list (searched nodes) and choose the shortest one and input into close list(arrived nodes).Repeat the mentioned steps until it reaches the goal nodes, then tracing back the arrived nodes to do the double checking.

> [!IMPORTANT]
> The objective of A* search algorithm is to find the shortest path from Start node to Goal node.
## How the project simulate the Path-Planning process in Aviation
### Project Tasks for Flight Path Planning
1. Find an appropriate aircraft model that achieve the minimum cost for
the challenge ( Map of Group 7) assigned to our group .
2. Design a new cost area that can reduce the cost of the route.
3. Design a new aircraft model within the constrains to achieve
minimum cost for your group challenge.

#### Map Set-Up  🗺️
Our porject uses x-y coordinate plane as a simplfied map for Path-planning simulation. All potential position you can go across with a unique
position (𝑥, 𝑦). Blue and Green points on the map are respectively indicated as the Start and Goal nodes,the khaki filled places represents Fuel-consuming area and Pink filled places represents Time-consuming area.Bold lines such as the inclined line from (10,40) to (20,20), or the vertical line from (40,60) to (40,35) are the obstalces which the path can't pass through.
![image](https://github.com/ox-x/AAE100_Group7_project/blob/main/GROUP%207%20MAP.png "GROUP 7 MAP")

> [!NOTE]
> Both  Fuel-consuming area and Time-consuming area can be passed through,but the cost for flying
through such area is increased due to airflow, legal restrictions and other reasons.
### Task 1
In aircraft operation, path selection is crucial. Correct path selection can not only reduce time consumption, but also significantly reduce fuel costs.Task 1 is to select the correct path for the aircraft based on the environment and calculate the lowest-cost model of aircraft in different situations.
#### Step 1
For our map scenario (map set-up), the end node lies on the border of the whole map ( their x-coordinates are all -10), which in the path finding code, the code identify the end node as a part of the border/ obstalces which unable to form the route in red colour. (even though the code make it to the end node, check the [python code file](https://github.com/ox-x/AAE100_Group7_project/blob/main/task1%20step1%20oringin.py)). To deal with this issue, we set the border on x=-12 which end node is no longer boarder the path finding consequence.

This is how the final code find the path, the light blue crosses shown on the map indicate the different tries/tests in path finding procedures , while crosses successfully connect the start and end nodes, a red line formed which indicates the path and stop the the path finding testes.(check the [python code file](https://github.com/ox-x/AAE100_Group7_project/blob/main/task1%20step1%20finnal.py) for more details).

![image](https://github.com/ox-x/AAE100_Group7_project/blob/main/PATH%20FINDING.gif )
#### Step 2
Firstly,count number of flights for aircraft models according to different scenario.Secondly,calculate trip cost from available numbers.Finally,choose the most suitable aircraft by comparing the costs of different aircraft models.The following is the calculation process.
![image](https://github.com/ox-x/AAE100_Group7_project/blob/main/handwriting.png)
We also do it inside our programme.
![image](https://github.com/ox-x/AAE100_Group7_project/blob/main/s1.png)
![image](https://github.com/ox-x/AAE100_Group7_project/blob/main/s2.png)
![image](https://github.com/ox-x/AAE100_Group7_project/blob/main/s3.png)
### Task 2
As there are cost intensive areas in the map, there are also certain areas where aircrafts
could consume relatively less fuel (Jet stream). Task 2 is to recreate a jet stream that could benefit our flight route the most.

The cost along the jet stream is reduced by 5%.
```python
if self.calc_grid_position(node.x, self.min_x) in self.lc_x:
                    if self.calc_grid_position(node.y, self.min_y) in self.lc_y:
                        node.cost -= 0.05 * self.motion[i][2]
```
The area of the jet stream span acrossthe map laterally and span 5-unit length vertically.

The following is the code to create the jet stream area which y_offset is from -10 to 55 and calculate the path for each jet stream area to find the minimum cost path.
```python
# set low cost area
best_time = float('inf')
best_lc_position = None

# try different y_offset for the low cost area
for y_offset in range(-10, 56):  # y_offset ranges from -10 to 55 (inclusive of -10 and 55)
    lc_x, lc_y = [], []
    for i in range(-12, 60):  # x stays between -12 and 60
        for j in range(y_offset, y_offset + 5):
            lc_x.append(i)
            lc_y.append(j)  # y value is the current y_offset
    plt.plot(ox, oy, ".k") # plot the obstacle
    plt.plot(sx, sy, "og") # plot the start position 
    plt.plot(gx, gy, "xb") # plot the end position
    
    plt.plot(fc_x, fc_y, "oy") # plot the cost intensive area 1
    plt.plot(tc_x, tc_y, "or") # plot the cost intensive area 2
    #plt.plot(lc_x, lc_y, "ob") # plot the low cost area

    plt.grid(True) # plot the grid to the plot panel
    plt.axis("equal") # set the same resolution for x and y axis 
    plt.plot(lc_x, lc_y, "ob")
    
    a_star = AStarPlanner(ox, oy, grid_size, robot_radius, fc_x, fc_y, tc_x, tc_y, lc_x, lc_y)
    rx, ry = a_star.planning(sx, sy, gx, gy)
    plt.plot(rx, ry, "-r") # show the route 
    plt.pause(0.001) # pause 0.001 seconds
    #plt.show() # show the plot

    total_time = flight_time

    # compare the total time with the best time
    if total_time < best_time:
        best_time = total_time
        best_lc_position = y_offset  # record the y_offset of the low cost area

print(f"the lowest cost: {best_time}, the y_offset of low cost area: {best_lc_position}")
```
![image](https://github.com/ox-x/AAE100_Group7_project/blob/main/Task%202%20calculate.gif)

[**calculation code file**](https://github.com/ox-x/AAE100_Group7_project/blob/main/Task%202%20calculate.py)

Below is the lowest cost path for the jet stream area with y_offset of 20.
![image](https://github.com/ox-x/AAE100_Group7_project/blob/main/Task%202%20result.gif)
![image](https://github.com/ox-x/AAE100_Group7_project/blob/main/Task%202.png)

[**result code file**](https://github.com/ox-x/AAE100_Group7_project/blob/main/Task2%20result.py)

### Task 3 
In our real life, aircrafts are designed based on industry needs.For example, A380 is for large global transport hubs and Boeing 737 is developed to cater for short and thin routes. Task 3 is to design a new aircraft by finding out its
parameters based on the restrictions
#### Trip Cost of Flight 💸
The fundamental rationale of the cost index concept is to achieve
minimum trip cost by means of a trade-off between operating costs
per hour and incremental fuel burn.

To keep it simple in our project, costs of operating an aircraft can be approximated by:
![image](https://github.com/ox-x/AAE100_Group7_project/blob/main/Cost%20Formula.png "Cost Formula")

|                 |     Case(2 Engine    |
| -------------   |    --------------    |
| Capacity        |          273         |
|Fuel Cost($/kg)  |        0.8823        |
|Trip Fuel(kg)    |  3181.0764773832456  |
|Time Cost($/min) |          23          |  
|Fixed Cost($)    |      2000            |
|Total Cost($)    |370,163.5241683685335 |

By calculating the cost with the equation, the overall cost for flying the 2-engine aircraft would be $370,163.5241683685335

Based on the calculation in the above, we named this new designed aircraft as A322 ,which is an upgraded version of the A321-XLR and has a similar design with the A330 family. The A322 features a foldable wingtip design similar to the B777-9X, which allows for longer wingtips in-flight, reducing induced drag. Also,the A322 can carry all 3000 passengers in 11 flights, with a total seat availability of 3003, maximizing the load factor. A twin-engine aircraft burns less fuel, the cost will be lower as a result. Thua this A322 designed by our group is an aircraft model that best fit Scenario 1 in task 1.

### Additional Task

#### Additional Task 1
Assume the aircraft is a supply craft that must reach 2 drop-off points to drop supplies before heading to base.

Below is the code to plot the checkpoint positions on the map.
```python
# checkpoint position
cpx1 = 55.0 
cpy1 = 20.0
cpx2 = 5.0
cpy2 = 10.0

# ~~~ ~~~

plt.plot(cpx1, cpy1, marker='^', color='yellow', markersize=10) # plot the first checkpoint position as a yellow triangle
plt.plot(cpx2, cpy2, marker='^', color='yellow', markersize=10) # plot the second checkpoint position as a yellow triangle
```
Below is the code to plan the path from start to goal and from first checkpoint to second checkpoint and from second checkpoint to goal.
```python
rx1, ry1 = a_star.planning(sx, sy, cpx1, cpy1) #plan the path from start to first checkpoint
rx2, ry2 = a_star.planning(cpx1, cpy1, cpx2, cpy2) #plan the path from first checkpoint to second checkpoint
rx3, ry3 = a_star.planning(cpx2, cpy2, gx, gy) #plan the path from second checkpoint to goal
```
Below is the final path from start to goal with two drop-off points.
![image](https://github.com/ox-x/AAE100_Group7_project/blob/main/Task%20a1.png)
![image](https://github.com/ox-x/AAE100_Group7_project/blob/main/Task%20a1%20output.png)

[**code file**](https://github.com/ox-x/AAE100_Group7_project/blob/main/Task%20a1.py)

#### Additional Task 2
Assume the mission and the environment keep changing for each operation

Only the fuel-consuming area remains and generate it randomly with a fixed area (40x40)
```python
# set random cost intesive area 
fc_x, fc_y = [], []
bottom_left_x = random.randint(-12, 20) # random position for the bottom left corner of the cost intensive area 
bottom_left_y = random.randint(-10, 20) 
for i in range(bottom_left_x,bottom_left_x+40):
    for j in range(bottom_left_y,bottom_left_y+40):
        fc_x.append(i)
        fc_y.append(j)
```
Diagonal movement is disabled, change parameter(s) so that the object could travel within one grid size
```python
def get_motion_model(): # the cost of the surrounding 8 points
    # dx, dy, cost
    motion = [[1, 0, 1],
              [0, 1, 1],
              [-1, 0, 1],
              [0, -1, 1],]
              # diagonal move is not allowed               
              # [-1, -1, math.sqrt(2)],
              # [-1, 1, math.sqrt(2)],
              # [1, -1, math.sqrt(2)],
              # [1, 1, math.sqrt(2)]]
            
    return motion
```
Obstacles are generated randomly with the number of 600 and the obstacles are not generated around the start and goal position.
```python
# generate random obstacles
for _ in range(600):
        while True:
            x = random.randint(-12, 60)
            y = random.randint(-10, 60)
            
            # check if the point is inside or around the start and goal position
            if not (sx - 5 <= x <= sx + 5 and sy - 5 <= y <= sy + 5) and \
                not (gx - 5 <= x <= gx + 5 and gy - 5 <= y <= gy + 5):
                ox.append(x)
                oy.append(y)
                break
```
Destination and starting points are generated randomly with at least a 40-unit distance in-between
```python
def generate_random_points(): # generate random start and goal position
    sx = random.uniform(-10, 58)
    sy = random.uniform(-8, 58)
    while True:
        gx = random.uniform(-10, 58)
        gy = random.uniform(-8, 58)
        distance = math.sqrt((gx - sx) ** 2 + (gy - sy) ** 2)
        if distance >= 40:
            break

    return sx, sy, gx, gy

sx, sy, gx, gy = generate_random_points()
```

Below is the final path

![image](https://github.com/ox-x/AAE100_Group7_project/blob/main/Task%20a2.png)
![image](https://github.com/ox-x/AAE100_Group7_project/blob/main/Task%20a2%20output.png)
[**code file**](https://github.com/ox-x/AAE100_Group7_project/blob/main/Task%20a2.py)

#### Additional Task 3
We use DijkstraPlanner BFSPlanner and A*Planner to find the shortest path in a same scenario and compare their performance in terms of time and path length.
```python
import time

#~~~ ~~~

start_time = time.time()

#~~~ ~~~

end_time = time.time()
print(f"Time taken: {end_time - start_time:.4f} seconds")
```

1. DijkstraPlanner
##### Algorithm Functionality:
Dijkstra's algorithm operates by iteratively selecting the node with the smallest tentative distance from the start node and exploring its neighboring nodes. It updates the distances to these neighbors and continues until all nodes have been explored.
##### Optimal Path Finding:
The DijkstraPlanner guarantees finding the shortest path in terms of total weight, which is critical in various applications such as route optimization, logistics, and navigation.
##### Performance:
![image](https://github.com/ox-x/AAE100_Group7_project/blob/main/Dijkstra.png)
![image](https://github.com/ox-x/AAE100_Group7_project/blob/main/Dijkstra%20fig.png)

[**code file**](https://github.com/ox-x/AAE100_Group7_project/blob/main/Task%20a3.1%20Dijkstra's%20Algorithm.py)

2. BFSPlanner
##### Algorithm Functionality:
Breadth-first search (BFS) is a graph traversal algorithm that starts at the root node (or some arbitrary node of a graph) and explores all of its neighbor nodes at the present depth prior to moving on to the nodes at the next depth level.
##### Optimal Path Finding:
In unweighted graphs, BFS guarantees finding the shortest path from the start to the goal node because it visits nodes in layers, treating all edges as having equal weight (or treating them as equivalent).
##### Performance:
![image](https://github.com/ox-x/AAE100_Group7_project/blob/main/BFS.png)
![image](https://github.com/ox-x/AAE100_Group7_project/blob/main/BFS%20fig.png)

[**code file**](https://github.com/ox-x/AAE100_Group7_project/blob/main/Task%20a3.2%20Breadth-First%20Search.py)

3. A*Planner
##### Algorithm Functionality:
A* is a graph traversal and path search algorithm that combines the best of BFS and Dijkstra's algorithm. It uses a heuristic function to estimate the distance from the current node to the goal node, and combines this with the Dijkstra's algorithm's idea of using a priority queue to select the node with the smallest tentative distance.
##### Optimal Path Finding:
A* uses a cost function f(n) = g(n) + h(n) to evaluate the priority of each node:
g(n): The actual cost from the start node to the current node.
h(n): The heuristic estimate of the cost from the current node to the goal node, often calculated using Manhattan distance or Euclidean distance.

This combination allows the A* algorithm to efficiently search while ensuring an optimal solution is found.
##### Performance:
![image](https://github.com/ox-x/AAE100_Group7_project/blob/main/Astar.png)
![image](https://github.com/ox-x/AAE100_Group7_project/blob/main/Astar%20fig.png)

[**code file**](https://github.com/ox-x/AAE100_Group7_project/blob/main/Task%20a3.3%20A*%20Algorithm.py)

4. Comparison

| Performance   | Path Length    | Time Cost    |
|----------|----------|----------|
| DijkstraPlanner  | 79.5269  | 0.7591  |
| BFSPlanner  | 79.7754  | 0.7280  |
| A* Planner  | 79.5269  | 0.4478  |

DijkstraPlanner and BFSPlanner have a similar time complexity and path length.

However, A* Planner has a better performance in terms of time complexity and path length. 

Therefore, A* Planner is the best choice for this scenario.

## Individual reflective essay 📜

#### 1. Liu Cheuk Kei Yuki 24123003d
  I am respondsible for task 3 aircraft design in this project. This assessment enhanced my creativity. I found it quite difficult at the beginning since I believe the market is saturated, aircraft manufacurers have already designed aircraft for almost all different scenario. Although there is currently existing aircraft that can be used in our scenario, there is no existing aircraft that is perfect for our scenario, and by perfect I mean maximising the load factor and minimising cost. Designing a new aircraft which doesn't exist currently is quite charllenging for me and I believe the main goal of the task is requiring me to think out of the box. In the process, I have understood why a tread of retiring four engines aircrafts exist after considering the fuel cost. I have also learnt and applied aerodynamics know-how during the design process in minimising time and fuel cost so I have put new technologies, foldable wingtips and Riblets surface in to our aircraft design. Apart from that, I would also like to apply knowledge that I have gained from other AAE subjects.Therefore, I have also took maintanance cost, training cost and airport expenses in to consideration. 

#### 2.Liang Chi Ho 24082636d
I am responsible for the README and a part of task 1. Throughout the whole project, everyone didn't know each other from the beginning but finally became cooperative with the other groupmates effectively; I can feel the enhancement of my communication, work processing, and collaboration skills. At first, the project seemed so difficult, if not demanding (since phrases like path-finding algorithm sounds to be unable to handle for a year 1 student like me) that I was afraid I would only help a little in these seemingly professional matters. However, when I dug into it deeper, it turned out to be quite easy to comprehend. It reminded me of the importance of beginning since all things are difficult before they are easy. Moreover, during these lectures' group discussions, I learned the importance of proactive questioning, listening and effective communication. While every groupmate is pre-assigned to do a different part and has their own procedures and procession, only by more asking and more listening to ensure everyone is doing great, no misunderstanding happens, and the cooperation is practical.

#### 3.Deng Yuchun 24112212d

#### 4.He Yinxi 24109157d
Completing the AAE 1001 Flight Path Planning Project has significantly expanded my knowledge and skill set. This course has been instrumental in broadening my understanding of these technologies. I've had to learn how to use GitHub, Visual Studio Code and Python, which were all new to me. Although I did not contribute to the coding part, because my partners are so capable, I played a role in the project by assisting with the calculations for Task 1 scenarios, where our objective was to design a new aircraft. 
Lastly，I would like to thank all my groupmates. I really enjoyed working on the project with them and I learnt a lot of new things which will definitely be beneficial ahead in my study.

#### 5. XIAO Suen  24107803d
I am responsible for the README of Task 1 step 2 and check the answer by calculate. When I first encountered this group task, to be honest, I was a bit scared. I was not familiar with using computers, let alone solving practical problems with code. However, the enthusiastic and responsible group members gave me a lot of confidence, encouraging me to try. As I gradually became familiar with using the program, I found that the task was not as difficult as I had imagined, and I became more adept at using the software. It turns out that writing code is quite interesting, and seeing the program I wrote run successfully gives me a great sense of accomplishment. This task taught me a lot, I not only learned basic coding but also boosted my confidence. More importantly, I learned about teamwork and gained deep friendships.

#### 6. Yu Mengzhi  24102853d
I was the leader of the project, and was mainly responsible for the code part in this project. I gained a deeper understanding of a* and other path algorithms in the process of writing the code. Completing various tasks in a real airplane operation scenario greatly improved my practical ability. One person cannot accomplish such a large amount of tasks, so teamwork is necessary. Decentralizing tasks to each member and allowing each person to fully utilize his/her abilities is the core problem in team management, and while solving this problem, I have stronger team management skills.

#### 7..

#### 8..
