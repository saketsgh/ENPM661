

AUTHOR - SAKET SESHADRI GUDIMETLA HANUMATH

UID : 116332293

NAME OF PRJECT - ENPM 661 PROJECT 3 : Implementation of A* Algorithm for turtlebot

DESCRIPTION - The python program included in the same folder is used to run A* algorithm for turtlebot based on user's input.
              The algorithms are run in a given map of RRL lab. The algorithm finds the shortest
	      path to goal node from the start node(both user provided) and displays the explored nodes and the path as well as outputs the computation time. 
	      It also contains a script for connecting to vrep using remote API through which the final path's (x, y) as well as the corresponding RPMs at those instants are sent to 
	      V-Rep for simulation. 

USAGE - The program was written using python 3.7(works with 3.5 but not legacy version like 2.7). The program can be run using an IDE like PyCharm or in the default IDLE of python.
	Upon the running the code the user is prompted to provide a number of inputs. The sample usecase has been provided below -

NOTE - (**please note that it might take a few seconds ~ 60-90seconds for the workspace to be printed, thus wait**) 
	(**the program will run for most inputs but it might take a few seconds ~30-60 seconds to display messages in case of erroneous input)
	
	example-->

	Enter the start point x-coordinate -->
	40(**then press Enter)

	Enter the start point y--coordinate -->
	40(**then press Enter)
	
	Enter the goal point x-coordinate -->
	400(**then press Enter)
	
	Enter the goal point y--coordinate -->
	620(**then press Enter)
	
	Connected to remote API server
	Found goal in --> seconds 89.94546341896057
	algorithm execution successful

	After this point the program will print the output, the time it took to reach the goal and then upon closing the output window the data is sent to V-Rep for simulation.