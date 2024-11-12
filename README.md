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

#### 6..

#### 7..

#### 8..
