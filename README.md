## Presentation Link :link:
(https://www.youtube.com/)

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


# AAE1001-Group 7-Project

<!-- TABLE OF CONTENTS -->
<details open="open">
  <summary><h2 style="display: inline-block">Table of Contents</h2></summary>
    <li><a href="##Background-of-Path-Planning-to-Aviation-Engineering">Background of Path Planning to Aviation Engineering</a></li>
    <li><a href="##Theory-of-Path-Planning-Algorithm">Theory of Path Planning Algorithm</a></li>
    <li><a href="##How the project simulate the Path-Planning process in Aviation">How the project simulate the Path-Planning process in Aviation</a></li>
  </ol>
</details>

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

![image](https://github.com/ox-x/AAE100_Group7_project/blob/main/PATH%20FINDING.gif =250x250)
### Task 2
As there are cost intensive areas in the map, there are also certain areas where aircrafts
could consume relatively less fuel (Jet stream). Task 2 is to recreate a jet stream that could benefit our flight route the most.
### Task 3
In our real life, aircrafts are designed based on industry needs.For example, A380 is for large global transport hubs and Boeing 737 is developed to cater for short and thin routes. Task 3 is to design a new aircraft by finding out its
parameters based on the restrictions
#### Trip Cost of Flight
The fundamental rationale of the cost index concept is to achieve
minimum trip cost by means of a trade-off between operating costs
per hour and incremental fuel burn.

To keep it simple in our project, costs of operating an aircraft can be approximated by:
![image](https://github.com/ox-x/AAE100_Group7_project/blob/main/Cost%20Formula.png "Cost Formula")
