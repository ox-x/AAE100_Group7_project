## Presentation Link
(https://www.youtube.com/)

#git tut 
https://gitee.com/opensource-guide/git-tutorial/getting-started/how-to-install-git.html. 
https://learngitbranching.js.org/?locale=zh_CN. 

#Task 1 (4 person)

 Yu Mengzhi 24102853d (Step 1)

Liang Chi Ho 24082636d (Step 1)
                                                  
Xiao Suen 24107803d  (Step 2)
                            
Deng Yuchun Harry 24112212d (Step 2)

#Task 2 (2 person)

Huang Liangtai 24133635D

Yu Qinxiang 24128609D


#Task 3 (2 person)

He Yinxi 24109157d (Calculation)

Liu Cheuk Kei Yuki 24123003d (Aircraft design)

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

Aa aviation has become important,if not essential in the transportation system,Path PLanning is needed in 
Private pilots do the path plan before the flight to make sure the
navigation aid is available
For ATC near airports, collaborative path planning is required to make
the best use of the crowded airspace
Commercial pilot follow the path that plan based on different cost index
designed by airlines.


## Theory of Path Planning Algorithm
A* search algorithm is the pathfinding algorithm which we are going to use.Simply put,it's a path finder by "cost" calculation,comoparsion.Three varbles included in the algorithm to find the "cost".To begin with, the f(x,y)=g(x,y)+h(x,y) ,which is the total cost of a neighboring node (𝑥, 𝑦).g(x,y) represents the exact cost of the path from the Start node to node (x,y) and the h(x,y) represents the
heuristic estimated cost from node (x,y) to the Goal node.

The algorithm starts with testing each possible nodes one-by-one from Start node,and record its path distance between Start node and it as "g".Also, recording the path distance between Goal node and it as "h". Afterall, calculate all nodes' "f" in the open list (searched nodes) and choose the shortest one and input into close list(arrived nodes).Repeat the mentioned steps until it reaches the goal nodes, then tracing back the arrived nodes to do the double checking.

> [!IMPORTANT]
> The objective of A* search algorithm is to find the shortest path from Start node to Goal node.
## How the project simulate the Path-Planning process in Aviation


#### Map Set-Up  🗺️
Our porject uses x-y coordinate plane as a simplfied map for Path-planning simulation. All potential position you can go across with a unique
position (𝑥, 𝑦). Blue and Green points on the map are respectively indicated as the Start and Goal nodes,the khaki filled places represents Fuel-consuming area and Pink filled places represents Time-consuming area.Bold lines such as the inclined line from (10,40) to (20,20), or the vertical line from (40,60) to (40,35) are the obstalces which the path can't pass through.
![image](https://github.com/ox-x/AAE100_Group7_project/blob/main/GROUP%207%20MAP.png "GROUP 7 MAP")
> [!NOTE]
> Both  Fuel-consuming area and Time-consuming area can be passed through,but the cost for flying
through such area is increased due to
airflow, legal restrictions and other reasons.
#### 

