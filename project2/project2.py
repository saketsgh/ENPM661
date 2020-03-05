#######################################################################################################################
# ENPM 661 PROJECT 2 : Implementation of Dijkstra and A* Algorithm for point and rigid robot in a given workspace
# code submitted by : SAKET SESHADRI GUDIMETLA HANUMATH
# UID : 116332293
#######################################################################################################################

import matplotlib.pyplot as plt
import numpy as np
import math
import time


# a class that contains all the necessary attributes that a node has while performing search
class Node:

	def __init__(self, x, y, cost, id, par_id, hcost):
		self.x = x
		self.y = y
		self.cost = cost
		self.id = id
		self.parent_id = par_id
		self.hcost = hcost      # only applicable for A*. It is set to zero for dijkstra nodes


# defining the astar algorithm for point and rigid robot. If checking for point robot enter radius and clearance as 0 when prompted
def astar(start, goal, rad_bot, clearance, grid_reso):

	init = time.time()

	# plotting the start and goal points
	plt.plot(start[0], start[1], "cx")
	plt.plot(goal[0], goal[1], "cx")

	obstacle_points = draw((0, 0), (250, 150), rad_bot, clearance, grid_reso)

	# checking if the start and goal points are valid
	if start in obstacle_points or goal in obstacle_points or start[0]>(250)/grid_reso or start[1]>(150)/grid_reso or goal[0]>(250)/grid_reso or goal[1]>(150)/grid_reso:
		return "points entered are either invalid or lie in obstacle"

	# open_set is a dictionary of unexplored nodes/nodes that still need to have their cost updated to optimum value
	open_set = dict()

	# closed_set is a dictionary of explored ndes
	closed_set = dict()

	plot_points = list()

	hcost_src = euclidian_distance(start[0], start[1], goal[0], goal[1])
	source_node = Node(start[0], start[1], 0, 0, -1, hcost_src)
	goal_node = Node(goal[0], goal[1], 0, 0, 0, 0)

	source_node.id = generate_id(source_node.x, source_node.y)

	# storing the source node in open_set
	open_set[source_node.id] = source_node

	# continuing the search till all the nodes are explored
	while len(open_set) != 0:

		# taking the element with minimum Heuristic cost value
		id_current_node = min(open_set, key=lambda i: open_set[i].hcost)
		current_node = open_set[id_current_node]

		# storing the explored points
		plot_points.append((current_node.x, current_node.y))
		x, y = zip(*plot_points)

		# marking the current node as explored by adding it to the closed set
		del open_set[id_current_node]
		closed_set[current_node.id] = current_node

		if current_node.x == goal_node.x and current_node.y == goal_node.y:
			goal_node.parent_id = current_node.parent_id
			goal_node.cost = current_node.cost
			plt.plot(x, y, "r.")
			break

		# printing the explored points for every 100/grid_reso nodes. This helps speed up the animation.
		# CHANGE THE NUMBER FOR VARYING SPEED
		if len(plot_points) % (100/grid_reso) == 0:
			plt.plot(x, y, "r.")
			plt.pause(0.0001)
			plot_points.clear()

		neighbor_nodes = create_neighbor_astar(current_node, goal_node, obstacle_points, grid_reso, rad_bot)

		for neighbor in neighbor_nodes:

			# ignore if already present in explored nodes
			if neighbor.id in closed_set:
				continue

			# if present in open_set then check if the cost in the current iteration is better than the previous
			if neighbor.id in open_set:
				if open_set[neighbor.id].cost > neighbor.cost:
					open_set[neighbor.id].cost = neighbor.cost
					open_set[neighbor.id].hcost = neighbor.hcost
					open_set[neighbor.id].parent_id = id_current_node

			else:
				open_set[neighbor.id] = neighbor

	X_list, Y_list = calc_path(closed_set, goal_node)

	# plotting the calculated path
	for i in range(len(X_list)):
		plt.plot(X_list[i], Y_list[i], "b.")

	final = time.time()
	print("Found goal in --> seconds", final - init)
	plt.show()


# defining the dijkstra algorithm for point and rigid robot. If checking for point robot enter radius and clearance as 0 when prompted
def dijkstra(start, goal, rad_bot, clearance, grid_reso):

	# same logic as A* with the exception of Heuristic cost that is set to zero in Dijkstra.
	# also elements in the open_set are sorted with respect to cost from source node and the not the heuristic cost

	init = time.time()

	plt.plot(start[0], start[1], "cx")
	plt.plot(goal[0], goal[1], "cx")
	obstacle_points = draw((0, 0), (251, 151), rad_bot, clearance, grid_reso)

	if start in obstacle_points or goal in obstacle_points or start[0]>(250)/grid_reso or start[1]>(150)/grid_reso or goal[0]>(250)/grid_reso or goal[1]>(150)/grid_reso:
		return "points entered are either invalid or lie in obstacle"

	open_set = dict()
	closed_set = dict()
	plot_points = list()

	source_node = Node(start[0], start[1], 0, 0, -1, 0)
	goal_node = Node(goal[0], goal[1], 0, 0, 0, 0)

	source_node.id = generate_id(source_node.x, source_node.y)
	open_set[source_node.id] = source_node

	while len(open_set) != 0:

		# the element with the least cost is returned rather than heuristic cost as seen in A*
		id_current_node = min(open_set,key=lambda i: open_set[i].cost)
		current_node = open_set[id_current_node]

		plot_points.append((current_node.x, current_node.y))
		x, y = zip(*plot_points)

		del open_set[id_current_node]
		closed_set[current_node.id] = current_node

		if current_node.x == goal_node.x and current_node.y == goal_node.y:
			goal_node.parent_id = current_node.parent_id
			goal_node.cost = current_node.cost
			plt.plot(x, y, "r.")
			break

		if len(plot_points) % (100/grid_reso) == 0:
			plt.plot(x, y, "r.")
			plt.pause(0.0001)
			plot_points.clear()

		neighbor_nodes = create_neighbor(current_node, obstacle_points,grid_reso, rad_bot)

		for neighbor in neighbor_nodes:

			if neighbor.id in closed_set:
				continue

			if neighbor.id in open_set:
				if open_set[neighbor.id].cost > neighbor.cost:
					open_set[neighbor.id].cost = neighbor.cost
					open_set[neighbor.id].parent_id = id_current_node


			else:
				open_set[neighbor.id] = neighbor

	X_list, Y_list = calc_path(closed_set, goal_node)
	for i in range(len(X_list)):
		plt.plot(X_list[i], Y_list[i], "b.")

	final = time.time()
	print("Found goal in --> %f seconds", final - init)
	plt.show()


# function to calculate the path from goal to start by retracing using parent IDs
def calc_path(closed_set, goal_node):

	# X_list and Y_list store the nodes from goal node to start node
	X_list = [goal_node.x]
	Y_list = [goal_node.y]
	p_id = goal_node.parent_id

	# retracing the path until the parent ID of the node in consideration is -1 which was the ID of source node
	while p_id != -1:
		node = closed_set[p_id]
		X_list.append(node.x)
		Y_list.append(node.y)
		p_id = node.parent_id

	return X_list, Y_list


# creates VALID neighbor nodes for Dijkstra algorithm and returns them
def create_neighbor(current_node, obstacle_points, grid_reso, rad_bot):

	X = current_node.x
	Y = current_node.y
	cost = current_node.cost

	# creates neighbors by moving the current node in the following directions :
	# right, up, left, down, left-diagonally-down, left-diagonally-up, right-diagonally-down, right-diagonally-up
	# structure of move --> (x coord, y coord, cost to move)
	move = [(1, 0, 1), (0, 1, 1),(-1, 0, 1), (0, -1, 1), (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
	(1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]
	neighbor_nodes = []

	for i in range(0, 8):
		X_new = X + move[i][0]
		Y_new = Y + move[i][1]
		cost_new = cost + move[i][2]
		coord = (X_new, Y_new)
		if X_new in range(0, round((250)/grid_reso)) and Y_new in range(0, round((150)/grid_reso)):
			if coord not in obstacle_points:
				ID = generate_id(X_new, Y_new)
				neighbor_node = Node(X_new, Y_new, cost_new, ID, current_node.id, 0)
				neighbor_nodes.append(neighbor_node)

	return neighbor_nodes


# function to compute euclidian distance between two given points
def euclidian_distance(x1, y1, x2, y2):

	dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
	return dist


# creates VALID neighbor nodes for A* algorithm and returns them
def create_neighbor_astar(current_node, goal_node, obstacle_points, grid_reso, rad_bot):

	X = current_node.x
	Y = current_node.y
	X_goal = goal_node.x
	Y_goal = goal_node.y
	cost = current_node.cost

	# creates neighbors by moving the current node in the following directions :
	# right, up, left, down, left-diagonally-down, left-diagonally-up, right-diagonally-down, right-diagonally-up
	# structure of move --> (x coord, y coord, cost to move)
	move = [(1, 0, 1), (0, 1, 1), (-1, 0, 1), (0, -1, 1), (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
	(1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]
	neighbor_nodes = []

	for i in range(0, 8):

		X_new = X + move[i][0]
		Y_new = Y + move[i][1]
		cost_new = cost + move[i][2]
		hcost = cost_new + euclidian_distance(X_new, Y_new, X_goal, Y_goal)
		cord = (X_new, Y_new)
		if X_new in range(0, round((250)/grid_reso)) and Y_new in range(0, round((150)/grid_reso)):
			if cord not in obstacle_points:
				ID = generate_id(X_new, Y_new)
				neighbor_node = Node(X_new, Y_new, cost_new, ID, current_node.id, hcost)
				neighbor_nodes.append(neighbor_node)

	return neighbor_nodes


#  function to create unique id for a node (x, y). Used for storing in a unique way in the open_set and closed_set
def generate_id(x, y):
	return y*300 + x


# plots the configuration space and returns the location of the obstacle points in the configuration space
def draw(start, end, rad_bot, clearance, grid_reso):

	# grid_reso is the resolution value that acts as a scaling factor
	x_start = round(start[0]/grid_reso)
	y_start = round(start[1]/grid_reso)
	x_end = round(end[0]/grid_reso)
	y_end = round(end[1]/grid_reso)

	obstacle_points = set()

	rad_bot = rad_bot/grid_reso
	clearance = clearance/grid_reso
	extra = rad_bot+clearance
	rad_circle = (15 / grid_reso) + extra

	# plotting the workspace
	plt.axes()
	rectangle = plt.Rectangle((x_start, y_start), x_end, y_end, fc='w')
	plt.gca().add_patch(rectangle)

	# plotting the grid lines
	plt.gca().set_xticks(np.arange(0, 250/grid_reso))
	plt.gca().set_yticks(np.arange(0, 150/grid_reso))
	plt.grid(True)

	# running the loop to check for various half plane and semi algebraic conditions and storing obstacle points in a set
	for x in np.arange(x_start, x_end):
		for y in np.arange(y_start, y_end):

			################################################################################################################
			# semi algebraic model for circle
			eq_circle = ((x - (190/grid_reso)) ** 2) + ((y - (130/grid_reso)) ** 2) - (rad_circle ** 2)
			################################################################################################################

			###############################################
			# Half plane equations of circle
			hp1 = -y + (67.5/grid_reso) - extra
			hp2 = x - (100 / grid_reso) - extra
			hp3 = y - (112.5/grid_reso) - extra
			hp4 = -x + (50/grid_reso) - extra
			################################################

			################################################################################################################
			# semi algebraic model for ellipse
			major_ax = (15/grid_reso) + extra
			minor_ax = (6/grid_reso) + extra

			eq_ellipse = (((x - (140/grid_reso)) ** 2) / major_ax ** 2) + (((y - (120/grid_reso)) ** 2) / minor_ax ** 2) - 1

			################################################################################################################
			# Defining the first polygon's half planes
			# L3
			hpL3 = (37 * x) - (20 * y) - (6101/grid_reso) - (extra*math.sqrt(37**2 + 20**2))

			# L4
			hpL4 = (38 * x) + (23 * y) - (8530/grid_reso) - (extra*math.sqrt(38**2 + 23**2))

			# -L7
			hpL7_1 = (-37 * x) - (10 * y) + (6551/grid_reso)

			# -L5
			hpL5 = (-38 * x) + (7 * y) + (5830/grid_reso) - (extra*math.sqrt(38**2 + 7**2))
			################################################################################################################
			# Defining the second polygon's half planes
			# L1
			hpL1 = (-41 * x) - (25 * y) + (6525/grid_reso) - (extra*math.sqrt(41**2 + 25**2))

			# L2
			hpL2 = -y + (15/grid_reso) - extra

			# L7
			hpL7 = (37 * x) + (10 * y) - (6551/grid_reso)

			# L6
			hpL6 = (2 * x) + (19 * y) - (1314/grid_reso) - (extra*math.sqrt(2**2 + 19**2))

			################################################################################################################
			tup = (x, y)

			if eq_circle <= 0:
				obstacle_points.add(tup)

			elif hp1 <= 0 and hp2 <= 0 and hp3 <= 0 and hp4 <= 0:
				obstacle_points.add(tup)

			elif hpL1 <= 0 and hpL2 <= 0 and hpL7 < 0 and hpL6 <= 0:
				obstacle_points.add(tup)

			elif eq_ellipse <= 0:
				obstacle_points.add(tup)

			elif hpL3 <= 0 and hpL4 <= 0 and hpL7_1 <= 0 and hpL5 <= 0:
				obstacle_points.add(tup)

			# condition for avoiding walls by considering them as obstacles
			elif x < extra or y < extra or x > (250/grid_reso) - extra or y > (150/grid_reso) - extra:
				obstacle_points.add(tup)

	# plotting the obstacle space
	for i in obstacle_points:
		plt.plot(i[0], i[1], 'k.')

	return obstacle_points


def main():

	print("Enter the start point x-coordinate --> ")
	x_s = int(input())
	print("Enter the start point y-coordinate --> ")
	y_s = int(input())
	print("Enter the goal point x-coordinate --> ")
	x_g = int(input())
	print("Enter the goal point y-coordinate --> ")
	y_g = int(input())
	print("Enter robot's radius (enter zero if point robot)-->")
	rad_bot = int(input())
	print("Enter clearance (enter zero if point robot)--> ")
	clearance = int(input())
	print("Enter grid size/resolution -->")
	grid_reso = int(input())
	print("Enter algorithm to be used to find path --> ")
	print("1. Dijkstra 2. Astar ")
	select = int(input())

	x_s = round(x_s/grid_reso)
	y_s = round(y_s/grid_reso)
	x_g = round(x_g/grid_reso)
	y_g = round(y_g/grid_reso)

	if select is 1:
		string = dijkstra((x_s, y_s), (x_g, y_g), rad_bot, clearance, grid_reso)
		if string == "points entered are either invalid or lie in obstacle":
			print(string)
	elif select is 2:
		string = astar((x_s, y_s), (x_g, y_g), rad_bot, clearance, grid_reso)
		if string == "points entered are either invalid or lie in obstacle":
			print(string)
	else:
		print("Invalid option")


if __name__ == '__main__':
	main()
