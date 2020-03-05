import numpy as np
import time
import collections
import copy

#######################################################################################################################
# ENPM 661 PROJECT 1
# code submitted by : SAKET SESHADRI GUDIMETLA HANUMATH
# UID : 116332293
#######################################################################################################################

# Initialisation of variables used throughout the code

Nodes_Goal = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 0]])  # goal state
NodesInfo = []          # stores the values as follows [current node ID, parent node ID, cost to reach that node]
ID = 1                  # denotes the index number of a particular node
Nodes = []              # list of all explored nodes
NodesString = set()     # stores the string equivalents of Nodes. It makes computation easier
cost = 0                # cost variable for each node

######################################################################################################################

# The Node CLASS is used to declare Node objects that possess following attributes ->
# 1. values - it stores the 3X3 array of one node
# 2. id - it denotes the index number of a particular node in the Nodes set that contains all the explored node values
# 3. parentID - stores the index number of the parent of current node
# 4. cost2come - shows the number of steps to reach that particular node
#
# Contains the following member functions -
# 1. conv2string
# 2. node2string
# 3. action_move
# 4. output_text_file
# 5. goal_check
# 6. bfs_method
# What it does - Creates Node objects like current_node which has the above mentioned attributes that can be accessed
#                anytime.


class Node:

	def __init__(self, init_val=None, node_id=None, parent_id=None, Cost=None):
		self.values = init_val
		self.id = node_id
		self.parentID = parent_id
		self.cost2come = Cost

	# Function name - conv2string
	# Inputs - state2sting
	# Inputs type - a 2D array
	# What it does - converts the state or value of a Node ie 3X3 2D array into a string of numbers
	# eg - 1 2 3  is converted to "123456780"
	#      4 5 6
	#      7 8 0
	# returns - a string cf numbers that depict the node values
	# It makes the computation easier
	def conv2string(self, state2string):
		s = ''
		for i in state2string:
			for j in i:
				s += str(j)
		return s

	# Function name - node2string
	# Inputs - state2sting
	# Inputs type - a 2D array
	# What it does - converts the state or value of a Node ie 3X3 2D array into a string of numbers
	# eg - 1 2 3  is converted to "1 2 3 4 5 6 7 8 0"
	#      4 5 6
	#      7 8 0
	# returns - a string cf numbers that are separated by space
	# It is same as the above function. This is one is used during the output text file generation only
	def node2string(self, state2string):
		s = ''
		for i in state2string:
			for j in i:
				s += str(j) + " "
		return s

	# Function name - action_move
	# Inputs - parent_node, visited_nodes
	# Inputs type - parent_node is a Node object that contains the information of the parent node
	# # # # # # # # visited_ nodes is a list of strings that contain the values of all explored nodes in a string format
	#
	# What it does - finds the location of the blank tile from the parent node 2D array and generates children nodes by
	# moving the tile up, down, left or right based on where the tile is located
	#
	# returns - a list of child nodes

	def action_move(self, parent_node, visited_nodes):
		child_nodes = []
		global ID                 # ID and NodesInfo are made global so that all the changes made are reflected globally
		global NodesInfo
		x_0, y_0 = 0, 0

		# find the location of the blank tile
		for i in range(0, 3):
			for j in range(0, 3):
				if parent_node.values[i][j] == 0:
					x_0 = i                 # x coordinate of blank tile
					y_0 = j                 # y coordinate of blank tile

		# MOVE THE BLANK TILE DOWN
		if x_0 is not 2:  # that is for row indices 0 and 1. Move Down

			# Generate a child node object called node_child by copying parent_node. Deepcopy is used because
			# any changes made to a copy of object do not reflect in the original object.
			node_child = Node(copy.deepcopy(parent_node.values), None, parent_node.id, parent_node.cost2come+1)

			# update the blank tile position by swapping its value from the node below it
			temp = node_child.values[x_0 + 1][y_0]
			node_child.values[x_0 + 1][y_0] = node_child.values[x_0][y_0]
			node_child.values[x_0][y_0] = temp

			# We need to check now whether the created child node already exists
			if self.conv2string(node_child.values) not in visited_nodes:
				ID += 1
				node_child.id = ID
				child_nodes.append(node_child)  # child nodes contain all the child node objects that are then sent to bfs_method

		# MOVE THE BLANK TILE UP
		if x_0 is not 0:

			node_child = Node(copy.deepcopy(parent_node.values), None, parent_node.id, parent_node.cost2come+1)

			# update the blank tile position by swapping its value from the node above it
			temp = node_child.values[x_0 - 1][y_0]
			node_child.values[x_0 - 1][y_0] = node_child.values[x_0][y_0]
			node_child.values[x_0][y_0] = temp

			# We need to check now whether the created child node already exists
			if self.conv2string(node_child.values) not in visited_nodes:
				ID += 1
				node_child.id = ID
				child_nodes.append(node_child)

		# MOVE THE BLANK TILE LEFT
		if y_0 is not 0:

			node_child = Node(copy.deepcopy(parent_node.values), None, parent_node.id, parent_node.cost2come+1)
			# update the blank tile position by swapping its value from the node to the left of it
			temp = node_child.values[x_0][y_0 - 1]
			node_child.values[x_0][y_0 - 1] = node_child.values[x_0][y_0]
			node_child.values[x_0][y_0] = temp

			# We need to check now whether the created child node already exists
			if self.conv2string(node_child.values) not in visited_nodes:
				ID += 1
				node_child.id = ID
				child_nodes.append(node_child)

		# MOVE THE BLANK TILE RIGHT
		if y_0 is not 2:

			node_child = Node(copy.deepcopy(parent_node.values), None, parent_node.id, parent_node.cost2come+1)
			# update the blank tile position by swapping its value from the node to the right of it
			temp = node_child.values[x_0][y_0 + 1]
			node_child.values[x_0][y_0 + 1] = node_child.values[x_0][y_0]
			node_child.values[x_0][y_0] = temp

			# We need to check now whether the created child node already exists
			if self.conv2string(node_child.values) not in visited_nodes:
				ID += 1
				node_child.id = ID
				child_nodes.append(node_child)

		return child_nodes

	# Function name - output text file
	# Inputs - visited_nodes, option
	# Inputs type - visited_ nodes is a list of strings that contain the values of all explored nodes in a string format
	# # # # # # # # option is just a number used for case selection
	#
	# What it does - generates text files Nodes, NodeInfo and nodePath in the same folder as the file is located in
	#
	# returns - nothing

	def output_text_file(self, visited_nodes, option):

		if option == 1:

			bfs_op = open('Nodes.txt', 'w')
			for i in range(0, len(visited_nodes)):
				transposed_nodes = visited_nodes[i].transpose()  # because the output has to be printed column wise,
				nodes_string = self.node2string(transposed_nodes)  # thus a transpose of the array is done and then,
				bfs_op.write("%s \n" % nodes_string)               # sent to the text file
			bfs_op.close()

		if option == 2:

			bfs_op = open('nodePath.txt', 'w')
			for i in range(0, len(visited_nodes)):
				transposed_nodes = visited_nodes[i].transpose()
				nodes_string = self.node2string(transposed_nodes)
				bfs_op.write("%s \n" % nodes_string)
			bfs_op.close()

		if option == 3:

			bfs_op = open('NodesInfo.txt', 'w')
			for i in range(0, len(visited_nodes)):
				for j in range(0, 3):
					nodes_string = visited_nodes[i][j]
					bfs_op.write("%s " % nodes_string)
				bfs_op.write("\n")
			bfs_op.close()

		if option == 4:                                   # in case of no solution the three files are blank
			bfs_op = open('Nodes.txt', 'w')
			bfs_op.write(" ")
			bfs_op.close()
			bfs_op = open('nodePath.txt', 'w')
			bfs_op.write(" ")
			bfs_op.close()
			bfs_op = open('NodesInfo.txt', 'w')
			bfs_op.write(" ")
			bfs_op.close()

	# Function name - goal_check
	# Inputs - current_node
	# Inputs type - Node object that contains information of current node
	# # # # # # # #
	#
	# What it does - checks if the current node is same as the goal node and computes the path of nodes from
	#                start to goal
	#
	# returns - flag = 1 or flag = 0 depending on whether the current node matches the goal node
	def goal_check(self, current_node):
		node_path = []      # stores the values of nodes from goal to start(Reversed later)

		if self.conv2string(current_node.values) == self.conv2string(Nodes_Goal):

			cID = current_node.id
			pID = current_node.parentID
			node_path.append(Nodes[cID - 1])
			node_path.append(Nodes[pID - 1])

			while pID != 1:   # when pID is 1 it means it has reached the penultimate node
				cID = [x for x in NodesInfo if pID in x][0][0]  # checks for the element in NodesInfo matching parent ID
				pID = [x for x in NodesInfo if pID in x][0][1]
				node_path.append(Nodes[cID - 1])
				node_path.append(Nodes[pID - 1])

			node_path.reverse()                # in order to print from start to goal rather than goal to start
			self.output_text_file(Nodes, 1)
			self.output_text_file(node_path, 2)
			self.output_text_file(NodesInfo, 3)

			# uncomment if you want to see the output on the console
			# print("The explored nodes are -->")
			# print(Nodes)
			# print("The information of nodes is as follows -->")
			# print(NodesInfo)
			# print("The node path is as follows -->")
			# print(node_path)

			flag = 1
			return flag

		else:
			flag = 0
			return flag

	# Function name - bfs_method
	# Inputs - none
	# Inputs type - none
	#
	# What it does - takes user input and finds out all possible states from start to goal node
	#
	# returns - none
	def bfs_method(self):

		global ID
		global cost
		global Nodes

		# take user input
		print("Enter the value of first node --> \n")
		first_node_val = np.array([[int(j) for j in input().split()] for i in range(3)])

		init_time = time.time()
		time_flag = 0
		flag = 0

		stack_of_nodes = collections.deque()            # stacks up new nodes
		init_node = Node(first_node_val, ID, 0, cost)   # initial node is created from user input data
		stack_of_nodes.append(init_node)

		while stack_of_nodes:

			current_time = time.time()
			if current_time-init_time >= 20*60:             # to detect a timeout
				time_flag = 1
				break

			current_node = stack_of_nodes.popleft()

			Nodes.append(current_node.values)          # Adding the current node values to Nodes list
			NodesString.add(self.conv2string(current_node.values))   # convert the grid into a string for easier computation
			NodesInfo.append([current_node.id, current_node.parentID, current_node.cost2come])

			flag = self.goal_check(current_node)

			if flag == 1:
				print("Node goal was reached")
				print("Total number of explored nodes -> " + str(len(Nodes)))
				print("time elapsed -> "+str(current_time-init_time))
				break

			new_nodes = self.action_move(current_node, NodesString)
			stack_of_nodes.extend(new_nodes)            # add the newly formed nodes

		current_time = time.time()
		if flag == 0:
			print("No sol found")
			print("time elapsed -> " + str(current_time - init_time))
			self.output_text_file(Nodes, 4)

		if time_flag == 1:
			print("Search Timed Out")


def main():
	node_obj = Node()
	node_obj.bfs_method()


main()






