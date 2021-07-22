#!/usr/bin/env python3
import rospy

from planning.srv import GetCSpace, GetCSpaceResponse

def index_to_grid(mapdata, index):
	"""
	Returns the cell coordinate given an index into the OccupancyGrid.
	:param mapdata  [OccupancyGrid] The map information.
	:param index    [int]           The OccupancyGrid index
	:return         [(int,int)]     The cell coordinate.
	"""

	# Calculate the y cell coordinate
	y = int(index / mapdata.info.width)
	# Calculate the x cell coordinate
	x = index % mapdata.info.width

	return x, y

def grid_to_index(mapdata, x, y):
	"""
	Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
	:param mapdata  [OccupancyGrid] The map information.
	:param x        [int]           The cell X coordinate.
	:param y        [int]           The cell Y coordinate.
	:return         [int]           The index.
	"""

	# Converts grid coordinate to index into occupancy grid
	# If the coordinate is out of map bounds, return None
	if x < 0 or x > (mapdata.info.width - 1) or y < 0 or y > (mapdata.info.height - 1):
		return None
	
	return y * mapdata.info.width + x

def calc_cspace(req):
	"""
	Calculates the C-Space, i.e., makes the obstacles in the map thicker.
	Publishes the list of cells that were added to the original map.
	:param msg:    [OccupancyGrid] from Gmapping
	:return        [OccupancyGrid] The C-Space.
	"""

	# Define array for to later apply the mask over mapdata obstacles
	padding = req.cspace_length
	mapdata = req.map

	# 5x5 for a padding of 2
	mask_width = padding * 2 + 1
	offsets = []

	# Iterate through every cell obstacle in mask_width
	for row in range(mask_width):
		for col in range(mask_width):
			# Account for the offset needed for the C-Space
			offsets.append((row - padding, col - padding))

	padded_cells = set()

	# Iterate through every cell in mapdata
	for i in range(len(mapdata.data)):

		# Do nothing if the cell is empty or unknown
		if mapdata.data[i] <= 0:
			continue

		# Get the coordinate of the current cell
		cell_coord = index_to_grid(mapdata, i)

		# Calculate all neighbors on the mask
		neighbors = [(cell[0] + cell_coord[0], cell[1] + cell_coord[1]) for cell in offsets]

		# Iterate through cell of valid neighbors
		for cell in neighbors:
			padded_cells.add(cell)

	occupancy_copy = list(mapdata.data)

	# Iterate through cell in padded cells
	for cell in padded_cells:
		# Index the current cell in the array
		# If this returns none, then the requested (x,y) is off the map
		index = grid_to_index(mapdata, cell[0], cell[1])

		if index is not None:
			# Assign cost
			occupancy_copy[index] = 100

	# Update costs in mapdata
	mapdata.data = tuple(occupancy_copy)

	# Return the C-space
	return GetCSpaceResponse(mapdata)

if __name__ == "__main__":
	rospy.init_node("cspace_node")

	s = rospy.Service("calc_cspace", GetCSpace, calc_cspace)

	rospy.loginfo('CSpace service node started')
	
	rospy.spin()