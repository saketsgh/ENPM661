# project 3 ENPM661
# code submitted by : Saket Seshadri Gudimetla Hanumath
# UID : 116332293

import numpy as np
import math as m
import vrep
import sys
import time
import matplotlib.pyplot as plt


def circle(x0, y0, rad, x, y, margin):

	cir = (x - x0) ** 2 + (y - y0) ** 2 - ((margin+rad) ** 2) <= 0
	return cir


def rect(x1, x2, y1, y2, x, y, margin):

	# x1, x2 bottom left, bottom right points
	# y1, y2 top left, top right points

	# Ax + By + C = 0 is the equation of straight line
	# here A, B and C are lists of line parameters a, b and c for the 4 lines of rectangles
	A = [-1, 1, 0, 0]
	B = [0, 0, -1, 1]
	C = [x1, -x2, y1, -y2]

	rect = ((A[0] * x + B[0] * y + C[0] - margin) <= 0) and ((A[1] * x + B[1] * y + C[1] - margin) <= 0) and (
			(A[2] * x + B[2] * y + C[2] - margin) <= 0) and ((A[3] * x + B[3] * y + C[3] - margin) <= 0)

	return rect


def draw():

	rad_bot = 17.7
	clearance = 19.3
	obstacle_points = dict()
	margin = rad_bot + clearance

	for i in np.arange(0, 1110, 1):
		for j in np.arange(0, 1010, 1):

			coordinate = (i, j)
			obstacle_points[coordinate] = False
			if circle(149.95, 830.05, 79.95, i, j, margin):
				obstacle_points[coordinate] = True
			elif circle(309.73, 830.05, 79.95, i, j, margin):
				obstacle_points[coordinate] = True
			elif circle(390, 965, 40.5, i, j, margin):
				obstacle_points[coordinate] = True
			elif circle(438, 736, 40.5, i, j, margin):
				obstacle_points[coordinate] = True
			elif circle(438, 274, 40.5, i, j, margin):
				obstacle_points[coordinate] = True
			elif circle(390, 45, 40.5, i, j, margin):
				obstacle_points[coordinate] = True
			elif rect(149.95, 309.73, 750.1, 910, i, j, margin):
				obstacle_points[coordinate] = True
			elif rect(438, 529, 315, 498, i, j, margin):
				obstacle_points[coordinate] = True
			elif rect(529, 712, 265, 341, i, j, margin):
				obstacle_points[coordinate] = True
			elif rect(474, 748, 35, 187, i, j, margin):
				obstacle_points[coordinate] = True
			elif rect(685, 1110, 0, 35, i, j, margin):
				obstacle_points[coordinate] = True
			elif rect(927, 1110, 35, 111, i, j, margin):
				obstacle_points[coordinate] = True
			elif rect(779, 896, 35, 93, i, j, margin):
				obstacle_points[coordinate] = True
			elif rect(1052, 1110, 187, 304, i, j, margin):
				obstacle_points[coordinate] = True
			elif rect(784.5, 936.5, 267, 384, i, j, margin):
				obstacle_points[coordinate] = True
			elif rect(1019, 1110, 362.5, 448.5, i, j, margin):
				obstacle_points[coordinate] = True
			elif rect(1052, 1110, 448.5, 565.5, i, j, margin):
				obstacle_points[coordinate] = True
			elif rect(744, 1110, 621, 697, i, j, margin):
				obstacle_points[coordinate] = True
			elif rect(832, 918, 827, 1010, i, j, margin):
				obstacle_points[coordinate] = True
			elif rect(983, 1026, 919, 1010, i, j, margin):
				obstacle_points[coordinate] = True
			elif i <= margin or j <= margin or i >= 1110 - margin or j >= 1010 - margin:
				obstacle_points[coordinate] = True

	return obstacle_points, margin


# a class that contains all the necessary attributes that a node has while performing search
class Node:

	def __init__(self, x, y, theta, cost, id, par_id, f_cost):
		self.x = x
		self.y = y
		self.theta = theta
		self.cost = cost
		self.id = id
		self.parent_id = par_id
		self.f_cost = f_cost
		self.ur = 0
		self.ul = 0


def euclidean_distance(x1, y1, x2, y2):

	dist = m.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
	return dist


def generate_id(x, y):
	return y*3000 + x*800


# function to calculate the path from goal to start by retracing using parent IDs
def calc_path(closed_set, goal_node):

	p_id = goal_node.parent_id
	node_list = list()
	node_list.append(goal_node)

	# retracing the path until the parent ID of the node in consideration is -1 which was the ID of source node
	while p_id != -1:

		node = closed_set[p_id]
		node_list.append(node)
		p_id = node.parent_id

	return node_list


def create_neighbor(current_node, goal_node,obstacle_points, margin):

	RPM1 = 20
	sampling_time = 3.8  # in seconds
	delta_t = sampling_time/100  # split equally into 100 intervals
	RPM2 = 35
	X_new = 0
	Y_new = 0
	T_new = 0
	j = 0
	r = 3.8
	L = 23

	X_old = current_node.x
	Y_old = current_node.y
	T_old = current_node.theta
	X_goal = goal_node.x
	Y_goal = goal_node.y
	cost_old = current_node.cost

	action_space = [(0, RPM1), (RPM1, 0), (RPM1, RPM1), (0, RPM2), (RPM2, 0), (RPM2, RPM2), (RPM1, RPM2), (RPM2, RPM1)]
	neighbor_nodes = []

	for i in range(0, 8):

		left_u = (action_space[i][0])*2*m.pi/60
		right_u = (action_space[i][1])*2*m.pi/60
		x, y, t = X_old, Y_old, T_old

		for j in range(100):

			T_new = t + (right_u - left_u)*delta_t*r/L
			X_new = (x + (left_u + right_u)*m.cos(T_new)*delta_t*(r/2))
			Y_new = (y + (left_u + right_u)*m.sin(T_new)*delta_t*(r/2))
			cord = (int(X_new), int(Y_new))

			if obstacle_points[cord]:
				break

			t = T_new
			x = X_new
			y = Y_new

		if j is 99:

			# checking once more for the last point
			X_new = int(X_new)
			Y_new = int(Y_new)
			cord = (X_new,Y_new)

			if obstacle_points[cord]:
				continue
			cost_new = cost_old + euclidean_distance(X_new, Y_new, X_old, Y_old)
			f_cost = cost_new + euclidean_distance(X_new, Y_new, X_goal, Y_goal)

			if (X_new < 1110-margin and X_new > margin) and (Y_new > margin and Y_new < 1010-margin):

				ID = generate_id(X_new, Y_new)
				neighbor_node = Node(X_new, Y_new, T_new, cost_new, ID, current_node.id, f_cost)
				neighbor_node.ur = right_u
				neighbor_node.ul = left_u
				neighbor_nodes.append(neighbor_node)

	return neighbor_nodes


def a_star(start, goal):

	init = time.time()

	# plotting the start and goal points
	plt.plot(start[0], start[1], "cx")
	plt.plot(goal[0], goal[1], "cx")

	obstacle_points, margin = draw()

	x = list()
	y = list()

	points = [x for x in obstacle_points.keys() if (obstacle_points[x])]
	x = [i[0] for i in points]
	y = [i[1] for i in points]
	plt.scatter(x, y, s=1, c='b', marker='x')

	# checking if the start and goal points are valid
	if obstacle_points[start] or start[0] > 1110 or start[1] > 1010:
		return "start point is either out of scope or inside obstacle"
	elif obstacle_points[goal] or goal[0] > 1110 or goal[1] > 1010:
		return "goal point is either out of scope or inside obstacle"

	# open_set is a dictionary of unexplored nodes/nodes that still need to have their cost updated to optimum value
	open_set = dict()

	# closed_set is a dictionary of explored nodes
	closed_set = dict()

	plot_points = list()

	f_cost_src = euclidean_distance(start[0], start[1], goal[0], goal[1])
	source_node = Node(start[0], start[1], 0, 0, 0, -1, f_cost_src)
	goal_node = Node(goal[0], goal[1], 0, 0, 0, 0, 0)

	# setting the initial and final rpm of left and right wheel as 0
	source_node.ul = 0
	source_node.ur = 0
	goal_node.ul = 0
	goal_node.ur = 0

	source_node.id = generate_id(source_node.x, source_node.y)

	# storing the source node in open_set
	open_set[source_node.id] = source_node

	# continuing the search till all the nodes are explored
	while len(open_set) != 0:

		# taking the element with minimum Heuristic cost value
		id_current_node = min(open_set, key=lambda i: open_set[i].f_cost)
		current_node = open_set[id_current_node]

		# storing the explored points
		plot_points.append((current_node.x, current_node.y))
		x, y = zip(*plot_points)

		# marking the current node as explored by adding it to the closed set
		del open_set[id_current_node]
		closed_set[current_node.id] = current_node

		if euclidean_distance(current_node.x, current_node.y, goal_node.x, goal_node.y) <= 10:
			goal_node.parent_id = current_node.parent_id
			goal_node.cost = current_node.cost
			plt.plot(x, y, "r.")
			break

		# printing the explored points for every 500 nodes
		if len(plot_points) % 500 == 0:
			plt.plot(x, y, "r.")
			plt.pause(0.01)
			plot_points.clear()

		neighbor_nodes = create_neighbor(current_node, goal_node, obstacle_points, margin)

		for neighbor in neighbor_nodes:

			# ignore if already present in explored nodes
			if neighbor.id in closed_set:
				continue

			# if present in open_set then check if the cost in the current iteration is better than the previous
			if neighbor.id in open_set:
				if open_set[neighbor.id].cost > neighbor.cost:
					open_set[neighbor.id].cost = neighbor.cost
					open_set[neighbor.id].f_cost = neighbor.f_cost
					open_set[neighbor.id].parent_id = id_current_node

			else:
				open_set[neighbor.id] = neighbor

	# plotting the calculated path
	node_list = calc_path(closed_set, goal_node)
	X_list = [node.x for node in node_list]
	Y_list = [node.y for node in node_list]
	plt.plot(X_list, Y_list, c='k')

	# store the generated path points for simulation
	simulation_points = [[node.x, node.y, node.ul, node.ur] for node in node_list]
	simulation_points.reverse()

	final = time.time()
	print("Found goal in --> seconds", final - init)

	plt.show()
	print("algorithm execution successful")
	return simulation_points


def main():

	print("Enter the start point x-coordinate(in cm) --> ")
	x_s = int(input())
	print("Enter the start point y-coordinate --> ")
	y_s = int(input())
	print("Enter the goal point x-coordinate --> ")
	x_g = int(input())
	print("Enter the goal point y-coordinate --> ")
	y_g = int(input())

	# close opened connections
	vrep.simxFinish(-1)

	clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
	vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)

	# check for successful connection
	if clientID != -1:
		print('Connected to remote API server')

	else:
		print('Connection not successful')
		sys.exit('Could not connect')

	# retrieve motor  handles
	errorCode, left_motor_handle = vrep.simxGetObjectHandle(clientID, 'wheel_left_joint', vrep.simx_opmode_oneshot_wait)
	errorCode, right_motor_handle = vrep.simxGetObjectHandle(clientID, 'wheel_right_joint', vrep.simx_opmode_oneshot_wait)

	errorCode, handle = vrep.simxGetObjectHandle(clientID, 'Turtlebot2', vrep.simx_opmode_oneshot_wait)
	vrep.simxSetObjectPosition(clientID, handle, -1, [((x_s/100) - 5.1870), ((y_s/100) - 4.8440), (0.06)], vrep.simx_opmode_oneshot_wait)

	# print(handle)
	emptyBuff = bytearray()

	simulation_points = a_star((x_s, y_s), (x_g, y_g))

	# start the simulation -->
	init = time.time()
	vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot_wait)

	for i in range(0, (len(simulation_points))):

		errorcode, position = vrep.simxGetObjectPosition(clientID, 189, -1, vrep.simx_opmode_streaming)

		# X_Point = (((simulation_points[i][0]) / (100)) - (5.1870))
		# Y_Point = (((simulation_points[i][1]) / (100)) - (4.8440))

		ul = simulation_points[i][2]
		ur = simulation_points[i][3]

		errorCode = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, ul, vrep.simx_opmode_streaming)
		errorCode = vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, ur, vrep.simx_opmode_streaming)

		# setting the sampling time for simulation. It will be the same as sampling time used in create_neighbors function
		time.sleep(3.8)

		# print("Velocities --> ", ur, ul)

	ul = 0
	ur = 0
	errorCode = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, ul, vrep.simx_opmode_streaming)
	errorCode = vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, ur, vrep.simx_opmode_streaming)

	final = time.time()
	print("time taken for simulation --> ", final-init)


if __name__ == '__main__':
	main()
