# EightPuzzleSolver
The 8-puzzle is a classic search problem defined on a 3×3 grid containing tiles numbered from 1 to 8 and a single blank space. The goal is to transform an initial configuration into the ordered goal state by sliding tiles horizontally or vertically into the blank space, where each move represents a state transition. The main challenge of the problem lies in efficiently exploring the large state space to find an optimal or near-optimal solution.

In this project, the 8-puzzle problem is solved and analyzed using several search algorithms, including Breadth-First Search (BFS), Depth-First Search (DFS), Iterative Deepening Search (IDS), and A* search. For the A* algorithm, both Manhattan distance and Euclidean distance heuristics are employed to guide the search toward the goal more efficiently. This allows for a comparative evaluation of uninformed and informed search strategies in terms of solution optimality, time complexity, and memory usage.

GUI
<img width="631" height="767" alt="image" src="https://github.com/user-attachments/assets/2d807108-06fd-40cd-b885-488fe960d91f" />

Results: 
Test 1
[3 0 2
 6 1 8
 7 5 4]
Algorithm	   Path Cost	Nodes Expanded	Search Depth	Running Time (s) /n
BFS	            9	          319	            9	           0.01       /n
DFS	          28009	        29312	         28009	       0.15
IDS	            9	          473	            9            0.01
A* (Manhattan)	9	          12	            9	           0.0
A* (Euclidean)	9	          12	            9	           0.0

Test 2
[■(8&6&3@0&5&7@1&2&4)]
Algorithm	Path Cost	Nodes Expanded	Search Depth	Running Time (s)
BFS	27	173855	27	0.34
DFS	20215	20946	20215	0.10
IDS	27	9616296	27	42.63
A* (Manhattan)	27	2930	27	0.02
A* (Euclidean)	27	6797	27	0.04
Test 3
[■(2&3&6@1&0&5@4&7&8)]
Algorithm	Path Cost	Nodes Expanded	Search Depth	Running Time (s)
BFS	20	47993	20	0.10
DFS	49996	56356	49996	0.27
IDS	20	237256	20	1.05
A* (Manhattan)	20	534	20	0.00
A* (Euclidean)	20	718	20	0.01

