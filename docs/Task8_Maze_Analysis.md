**Task 8**

Left wall follower robots is memoryless and can solve mazes but will get stuck in loops. Also, we are getting a dry run to solve maze so I believe that DFS is just better approach because it tackles loops.



Also , because we have a dry run , we can map out whole maze and find optimum path in first run and then take the absolute shortest path in final run and operates at maximum speed as it already knows turns and arrays.

SO, DFS is just way better.

so Phase 1 :Discover the path and store the info , handle dead ends through backtracking.

Phase 2: Use the stored info to find shortest path and implement it



**Pseuodocode:**

INITIALIZE:

&#x20;   maze\_graph = empty graph

&#x20;   visited\_nodes = empty set

&#x20;   visited\_edges = empty set

&#x20;   current\_node = START

&#x20;   heading = initial direction



FUNCTION EXPLORE(current\_node):



&#x20;   Mark current\_node as visited



&#x20;   IF current\_node is GOAL:

&#x20;       RETURN SUCCESS



&#x20;   Detect open directions using ultrasonic sensors

&#x20;   Store the available corridors at current\_node



&#x20;   Sort directions by priority:

&#x20;       LEFT → STRAIGHT → RIGHT → BACK



&#x20;   FOR each direction in priority order:



&#x20;       IF direction is open:

&#x20;           Turn towards direction

&#x20;           Move through the corridor to the next junction or dead end



&#x20;           next\_node = identify\_or\_create\_node()



&#x20;           Add edge (current\_node, next\_node) to maze\_graph



&#x20;           IF edge (current\_node, next\_node)

&#x20;              has not been explored:



&#x20;               Mark edge as explored



&#x20;               IF next\_node has not been visited:

&#x20;                   IF EXPLORE(next\_node) == SUCCESS:

&#x20;                       RETURN SUCCESS



&#x20;               Turn around

&#x20;               Return to current\_node



&#x20;   RETURN FAILURE





\# PHASE 1: DRY RUN — EXPLORE AND MAP



EXPLORE(START)



Store the discovered maze\_graph

Store the detected GOAL node



\# PHASE 2: FIND THE SHORTEST ROUTE



route = BFS(maze\_graph, START, GOAL)



Store the ordered route from START to GOAL





\# PHASE 3: ACTUAL RUN — REPLAY THE ROUTE



current\_node = START



FOR each next\_node in route, excluding START:



&#x20;   Determine the turn needed to reach next\_node

&#x20;   Turn using calibrated motor commands



&#x20;   WHILE next\_node has not been reached:

&#x20;       Read front, left and right ultrasonic sensors

&#x20;       Move forward while maintaining safe wall clearance

&#x20;       Correct alignment if necessary



&#x20;   current\_node = next\_node



STOP the robot at GOAL



**#AI is used for pseudocode.**



**#Solution of maze provide by IIT Bombay techfest is simulated and attached in the files. Ai is used for simulation**

