

AUTHOR - SAKET SESHADRI GUDIMETLA HANUMATH

NAME OF PRJECT - ENPM 661 PROJECT 2 : Implementation of Dijkstra and A* Algorithm for point and rigid robot in a given workspace

DESCRIPTION - The python program included in the same folder is used to run both the Dijkstra and A* algorithm for a point/rigid robot based on user's input.
              The algorithms are run in an obstacle filled environment composed of a rectangle, circle, ellipse and a polygon. The algorithm finds the shortest
	      path to goal node from the start node(both user provided) and displays the explored nodes and the path as well as outputs the computation time.

USAGE - The program was written using python 3.7(works with 3.5 but not legacy version like 2.7).
	Upon the running the code the user is prompted to provide a number of inputs. The sample usecase has been provided below -

NOTE - (**please note that it might take a few seconds ~ 25-30seconds for the workspace to be printed, thus wait**) 
	(**the program will run for most inputs but it might take a few seconds ~30-60 seconds to display messages in case of erroneous input)
	(**The program's output has a warning by matplotlib, please ignore it as it does not affect program's performance**)
	
	SCENARIO 1 -->
	Enter the start point x-coordinate -->
	0(**then press Enter)

	Enter the start point y--coordinate -->
	0(**then press Enter)
	
	Enter the goal point x-coordinate -->
	50(**then press Enter)
	
	Enter the goal point y--coordinate -->
	130(**then press Enter)
	
	Enter robot's radius (enter zero if point robot)-->
	0(**then press Enter)
	**** for rigid robot enter a non zero radius****
	
	Enter clearance (enter zero if point robot)-->
	0(**then press Enter)
	**** for rigid robot you can enter any value****
	
	Etner grid size/resolution --?
	1. Dijkstra 2. Astar	
	**** Enter either 1 or 2 for running either of the algorithm. If you enter anything else it prints "Invalid Option"****
	2

	Found goal in --> seconds 11.930627

	****The program will display "points entered are either invalid or lie in obstacle" if the coordinates entered are 
	    not present in free space, for example **** 
	
	SCENARIO 2-->
	Enter the start point x-coordinate -->
	0

	Enter the start point y--coordinate -->
	0
	
	Enter the goal point x-coordinate -->
	30
	
	Enter the goal point y--coordinate -->
	40
	
	Enter robot's radius (enter zero if point robot)-->
	5
	**** for rigid robot enter a non zero radius****
	
	Enter clearance (enter zero if point robot)-->
	1
	**** for rigid robot you can enter any value****
	
	Etner grid size/resolution --?
	1. Dijkstra 2. Astar	
	**** Enter either 1 or 2 for running either of the algorithm. If you enter anything else it prints "Invalid Option"****
	1

	points entered are either invalid or lie in obstacle
