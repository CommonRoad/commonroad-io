from commonroad.prediction.prediction import Occupancy, SetBasedPrediction
from spot import *
from commonroad.scenario.obstacle import Obstacle, DynamicObstacle, StaticObstacle, ObstacleType
from commonroad.scenario.scenario import Scenario
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.lanelet import Lanelet
from commonroad.geometry.shape import Polygon
from typing import Union, List
from commonroad.geometry.shape import Rectangle, Circle, ShapeGroup
import os
from numpy import array
import math



class SpotPrediction():
	def __init__(self, t0: float, time_steps: int, dt: float, net: Union[None, cpp_roadNetwork] = None, myCar: Union[None, cpp_egoVehicle] = None, cpp_obstacles: Union[None, cpp_dynamicObstacle] = None, pyobstacles: Union[None, DynamicObstacle] = None, elapsed: float = None):
		self.t0 = t0
		self.time_steps = time_steps
		self.dt = dt
		self.roadNetwork = None
		self.cpp_myCar = None
		self.cpp_obstacles = []
		self.py_obstacles = []
		self.elapsed_time = None
		self.velo_dict = {}

	def createRoadNetwork(self, py_lanelets: List[Lanelet]):

		# convert python-lanelets to cpp-lanelets
		lanelets = []
		for item in py_lanelets:
			laneletObj = cpp_vehicularLanelet()
			laneletObj.setId(item.lanelet_id)
			if item.speed_limit == None:
				item.speed_limit = 999999.0
			laneletObj.setSpeedLimit(item.speed_limit)
			v = cpp_vertice()
			for vert in item.left_vertices:
				v.x = vert[0]
				v.y = vert[1]
				laneletObj.addLeftVertice(v)
			for vert in item.right_vertices:
				v.x = vert[0]
				v.y = vert[1]
				laneletObj.addRightVertice(v)
			for vert in item.center_vertices:
				v.x = vert[0]
				v.y = vert[1]
				laneletObj.addCenterVertice(v)

			lanelets.append(laneletObj)

		# assign adjacent lanelets
		i = 0
		for item in py_lanelets:
			for pred in item.predecessor:
				for l in lanelets:
					if l.getId() == pred:
						lanelets[i].addPredecessor(l)
						break
				
			for suc in item.successor:
				for l in lanelets:
					if l.getId() == suc:
						lanelets[i].addSuccessor(l)
						break
			if item.adj_left != None:
				if item.adj_left_same_direction:
					for l in lanelets:
						if l.getId() == item.adj_left:
							lanelets[i].setLeftAdjacent(l, 'same')
							break
				else:
					for l in lanelets:
						if l.getId() == item.adj_left:
							lanelets[i].setLeftAdjacent(l, 'opposite')
							break

			if item.adj_right != None:
				if item.adj_right_same_direction:
					for l in lanelets:
						if l.getId() == item.adj_right:
							lanelets[i].setRightAdjacent(l, 'same')
							break
				else:
					for l in lanelets:
						if l.getId() == item.adj_right:
							lanelets[i].setRightAdjacent(l, 'opposite')
							break
			i = i + 1

		self.roadNetwork = getRoadNetwork(lanelets)

	def setEgoVehicle(self, planning_problem: PlanningProblem, a_min: float, a_max: float, rect_length: float, rect_width):
		myCar = cpp_egoVehicle()
		myCar.setId(planning_problem.planning_problem_id)
		myCar.setVelocity(planning_problem.initial_state.velocity)
		myCar.setOrientation(planning_problem.initial_state.orientation)
		myCar.setPosition(planning_problem.initial_state.position[0], planning_problem.initial_state.position[1])
		myCar.setAccComfortMin(a_min)
		myCar.setAccComfortMax(a_max)
		rect = cpp_rectangle()
		rect.setLength(rect_length)
		rect.setWidth(rect_width)
		myCar.setShape(rect)
		self.cpp_myCar = myCar
		
	def setObstacles(self, scenario: Scenario):
		# convert python-obstacles to cpp-obstacles
		obstacles = []
		for key, item in scenario._dynamic_obstacles.items():
			if item.obstacle_type == ObstacleType.CAR:
				dynamicObj = cpp_vehicle()
				dynamicObj.setId(item.obstacle_id)
				position_uncertainty = 0
				if isinstance(item.initial_state.position, Rectangle):
					dynamicObj.setPosition(item.initial_state.position.center[0], item.initial_state.position.center[1])
					position_uncertainty = math.sqrt(item.initial_state.position.length**2+item.initial_state.position.width**2)
				elif isinstance(item.initial_state.position, Circle):
					dynamicObj.setPosition(item.initial_state.position.center[0], item.initial_state.position.center[1])
					position_uncertainty = item.initial_state.position.radius*2
				elif isinstance(item.initial_state.position, Polygon):
					minx, miny, maxx, maxy = item.initial_state.position._shapely_polygon.bounds
					length_x = abs(maxx-minx)
					length_y = abs(maxy-miny)
					dynamicObj.setPosition(minx+length_x/2.0, miny+length/2.0)
					position_uncertainty = math.sqrt(length_x**2+length_y**2)
				else:
					dynamicObj.setPosition(item.initial_state.position[0], item.initial_state.position[1])
				if hasattr(item.initial_state.orientation, 'start') and hasattr(item.initial_state.orientation, 'end'):
					meanOrientation = (item.initial_state.orientation.start + item.initial_state.orientation.end)/2.0
					dynamicObj.setOrientation(meanOrientation)
					dynamicObj.setOrientationError(abs(meanOrientation-item.initial_state.orientation.start))
				else:
					dynamicObj.setOrientation(item.initial_state.orientation)
				if hasattr(item.initial_state.velocity, 'start') and hasattr(item.initial_state.velocity, 'end'):
					meanVelocity = (item.initial_state.velocity.start + item.initial_state.velocity.end)/2.0
					dynamicObj.setVelocity(meanVelocity)
					dynamicObj.setVelocityError(abs(meanVelocity-item.initial_state.velocity.start))
				else:
					dynamicObj.setVelocity(item.initial_state.velocity)
				if isinstance(item._obstacle_shape, Rectangle):
					rect = cpp_rectangle()
					rect.setLength(item._obstacle_shape.length + position_uncertainty)
					rect.setWidth(item._obstacle_shape.width + position_uncertainty)
				dynamicObj.setShape(rect)
				obstacles.append(dynamicObj)
			elif item.obstacle_type == ObstacleType.BICYCLE:
				dynamicObj = cpp_cyclist()
				dynamicObj.setId(item.obstacle_id)
				position_uncertainty = 0
				if isinstance(item.initial_state.position, Rectangle):
					dynamicObj.setPosition(item.initial_state.position.center[0], item.initial_state.position.center[1])
					position_uncertainty = math.sqrt(item.initial_state.position.length**2+item.initial_state.position.width**2)
				elif isinstance(item.initial_state.position, Circle):
					dynamicObj.setPosition(item.initial_state.position.center[0], item.initial_state.position.center[1])
					position_uncertainty = item.initial_state.position.radius*2
				elif isinstance(item.initial_state.position, Polygon):
					minx, miny, maxx, maxy = item.initial_state.position._shapely_polygon.bounds
					length_x = abs(maxx-minx)
					length_y = abs(maxy-miny)
					dynamicObj.setPosition(minx+length_x/2.0, miny+length/2.0)
					position_uncertainty = math.sqrt(length_x**2+length_y**2)
				else:
					dynamicObj.setPosition(item.initial_state.position[0], item.initial_state.position[1])
				if hasattr(item.initial_state.orientation, 'start') and hasattr(item.initial_state.orientation, 'end'):
					meanOrientation = (item.initial_state.orientation.start + item.initial_state.orientation.end)/2.0
					dynamicObj.setOrientation(meanOrientation)
					dynamicObj.setOrientationError(abs(meanOrientation-item.initial_state.orientation.start))
				else:
					dynamicObj.setOrientation(item.initial_state.orientation)
				if hasattr(item.initial_state.velocity, 'start') and hasattr(item.initial_state.velocity, 'end'):
					meanVelocity = (item.initial_state.velocity.start + item.initial_state.velocity.end)/2.0
					dynamicObj.setVelocity(meanVelocity)
					dynamicObj.setVelocityError(abs(meanVelocity-item.initial_state.velocity.start))
				else:
					dynamicObj.setVelocity(item.initial_state.velocity)
				if isinstance(item._obstacle_shape, Rectangle):
					rect = cpp_rectangle()
					rect.setLength(item._obstacle_shape.length + position_uncertainty)
					rect.setWidth(item._obstacle_shape.width + position_uncertainty)
				dynamicObj.setShape(rect)
				obstacles.append(dynamicObj)
			elif item.obstacle_type == ObstacleType.PEDESTRIAN:
				dynamicObj = cpp_pedestrian()
				position_uncertainty = 0
				dynamicObj.setId(item.obstacle_id)
				if isinstance(item.initial_state.position, Rectangle):
					dynamicObj.setPosition(item.initial_state.position.center[0], item.initial_state.position.center[1])
					position_uncertainty = math.sqrt(item.initial_state.position.length**2+item.initial_state.position.width**2)/2.0
				elif isinstance(item.initial_state.position, Circle):
					dynamicObj.setPosition(item.initial_state.position.center[0], item.initial_state.position.center[1])
					position_uncertainty = item.initial_state.position.radius
				elif isinstance(item.initial_state.position, Polygon):
					minx, miny, maxx, maxy = item.initial_state.position._shapely_polygon.bounds
					length_x = abs(maxx-minx)
					length_y = abs(maxy-miny)
					dynamicObj.setPosition(minx+length_x/2.0, miny+length/2.0)
					position_uncertainty = math.sqrt(length_x**2+length_y**2)/2.0
				else:
					dynamicObj.setPosition(item.initial_state.position[0], item.initial_state.position[1])
				if hasattr(item.initial_state.orientation, 'start') and hasattr(item.initial_state.orientation, 'end'):
					meanOrientation = (item.initial_state.orientation.start + item.initial_state.orientation.end)/2.0
					dynamicObj.setOrientation(meanOrientation)
					dynamicObj.setOrientationError(abs(meanOrientation-item.initial_state.orientation.start))
				else:
					dynamicObj.setOrientation(item.initial_state.orientation)
				if hasattr(item.initial_state.velocity, 'start') and hasattr(item.initial_state.velocity, 'end'):
					meanVelocity = (item.initial_state.velocity.start + item.initial_state.velocity.end)/2.0
					dynamicObj.setVelocity(meanVelocity)
					dynamicObj.setVelocityError(abs(meanVelocity-item.initial_state.velocity.start))
				else:
					dynamicObj.setVelocity(item.initial_state.velocity)
				if isinstance(item._obstacle_shape, Circle):
					circ = cpp_circle()
					circ.setRadius(item._obstacle_shape.radius + position_uncertainty)
					circ.setCenter(item._obstacle_shape.center[0], item.obstacle_shape.center[1])
				elif isinstance(item._obstacle_shape, Rectangle):
					circ = cpp_circle()
					circ.setRadius(math.sqrt(item._obstacle_shape.length**2+item._obstacle_shape.width**2)/2.0 + position_uncertainty)
				else:
					raise TypeError('Pedestrian with ID %s must have appropriate shape. Got shape: %s' % (
						(item._obstacle_id, item._obstacle_shape.__class__)))
				dynamicObj.setShape(circ)
				obstacles.append(dynamicObj)
		self.cpp_obstacles = obstacles
		self.py_obstacles = scenario._dynamic_obstacles

	def calcOccupancies(self, num_threads: int, verbose: int = None):
		if len(self.cpp_obstacles) == 0:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")
			return
		elif self.roadNetwork == None:
			print("No lanes! Please create lanes with SpotPredicition.createRoadNetwork(scenario.lanelet_network.lanelets)")
			return
		# call spot with converted elements	
		returned_objects = call_spot(self.cpp_obstacles, self.cpp_myCar, self.roadNetwork, [self.t0, self.dt, self.time_steps*self.dt+self.t0], num_threads)
		self.elapsed_time = returned_objects[1]
		if verbose is not None:
			print("Calculated occupancies for dynamic obstacles:", " ".join(str(item.getId()) for item in returned_objects[0]))
		# assign occupancies to python obstacles
		for item in returned_objects[0]:
			obst = self.py_obstacles[item.getId()]
			occMatrix = item.getOccupancyMatrix()
			occupancy_list = []
			for j in range(self.time_steps):
				sh = ShapeGroup([])
				occ = Occupancy(j+1, sh)
				occupancy_list.append(occ)
			i = 0
			for timeStep in occMatrix:
				for laneOcc in timeStep:
					lane_id = 0
					if len(laneOcc.forLane) > 0:
						lane_id = laneOcc.forLane[0].getId()
					self.velo_dict[(item.getId(),i,lane_id)] = [laneOcc.minVelo, laneOcc.maxVelo]
					for poly in convertPolygon(laneOcc):
						vertices = []
						for vert in poly:
							vertices.append([vert.x, vert.y])
						if len(vertices) < 4:
							continue
						shape_obj = Polygon(array(vertices))
						occupancy_list[i].shape.shapes.append(shape_obj)
						#t_s = laneOcc.timeInterval.startingTime
						#t_e = laneOcc.timeInterval.ending
				i = i + 1
			occupancies = occupancy_list.copy()
			obst.prediction = SetBasedPrediction(1, occupancies[0:])

	def setSpeedLimitOfLanes(self, limit: float):
		if self.roadNetwork is not None:
			for laneObj in self.roadNetwork.lanes:
				speedLimit = []
				for i in range(len(laneObj.getSpeedLimit())):
					speedLimit.append(limit)
				laneObj.setSpeedLimit(speedLimit)
		else:
			print("No lanes! Please create lanes with SpotPredicition.createRoadNetwork(scenario.lanelet_network.lanelets)")

	def setSpeedLimitOfLanelets(self, limit: float):
		if self.roadNetwork is not None:
			for laneletObj in self.roadNetwork.lanelets:
				laneletObj.setSpeedLimit(limit)
		else:
			print("Lanelets not set!")

	
	def restrictSpeedLimitOfLanes(self, limit: float):
		if self.roadNetwork is not None:
			for laneObj in self.roadNetwork.lanes:
				speedLimit = []
				for i in range(len(laneObj.getSpeedLimit())):
					if laneObj.getSpeedLimit()[i] < limit:
						speedLimit.append(laneObj.getSpeedLimit()[i])
					else:
						speedLimit.append(limit)
				laneObj.setSpeedLimit(speedLimit)
		else:
			print("No lanes! Please create lanes with SpotPredicition.createRoadNetwork(scenario.lanelet_network.lanelets)")

	def setAmaxOfVehicles(self, a_max: float):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if type(item) == cpp_vehicle:
					item.setAmax(a_max)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")

	def setAminLongOfVehicles(self, amin_long: float):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if type(item) == cpp_vehicle:
					item.setAminLong(amin_long)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")

	def setOnlyInLaneOfVehicles(self, val: bool):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if type(item) == cpp_vehicle:
					item.setOnlyInLane(val)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")

	def setAmaxLongOfVehicles(self, amax_long: float):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if type(item) == cpp_vehicle:
					item.setAmaxLong(amax_long)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")

	def setVmaxOfVehicles(self, v_max: float):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if type(item) == cpp_vehicle:
					item.setVmax(v_max)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")				


	def setSpeedingFactorOfVehicles(self, factor: float):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if type(item) == cpp_vehicle:
					item.setSpeedingFactor(factor)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")		

	# constraint C3 (not allowing backward driving)
	def setConstraint3OfVehicles(self, boolean: bool):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if type(item) == cpp_vehicle:
					item.setConstraint3(boolean)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")	

	# changing and crossing lanes is forbidden unless allowed by traffic regulations
	def setConstraint5OfVehicles(self, boolean: bool):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if type(item) == cpp_vehicle:
					item.setConstraint5(boolean)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")

	def setComputeM1(self, boolean: bool):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if type(item) == cpp_vehicle:
					item.setCompute_Occ_M1(boolean)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")

	def setComputeM2(self, boolean: bool):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if type(item) == cpp_vehicle:
					item.setCompute_Occ_M2(boolean)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")	

	def setComputeM3(self, boolean: bool):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if type(item) == cpp_vehicle:
					item.setCompute_Occ_M3(boolean)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")		

	def setVsOfVehicles(self, v_s: float):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if type(item) == cpp_vehicle:
					item.setVs(v_s)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")	

	def setAmaxOfCyclist(self, a_max: float):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if type(item) == cpp_cyclist:
					item.setAmax(a_max)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")

	def setAminLongOfCyclist(self, amin_long: float):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if type(item) == cpp_cyclist:
					item.setAminLong(amin_long)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")

	def setOnlyInLaneOfCyclist(self, val: bool):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if type(item) == cpp_cyclist:
					item.setOnlyInLane(val)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")

	def setAmaxLongOfCyclist(self, amax_long: float):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if type(item) == cpp_cyclist:
					item.setAmaxLong(amax_long)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")

	def setVmaxOfCyclist(self, v_max: float):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if type(item) == cpp_cyclist:
					item.setVmax(v_max)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")


	def setSpeedingFactorOfCyclist(self, factor: float):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if type(item) == cpp_cyclist:
					item.setSpeedingFactor(factor)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")

	# constraint C3 (not allowing backward driving)
	def setConstraint3OfCyclist(self, boolean: bool):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if type(item) == cpp_cyclist:
					item.setConstraint3(boolean)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")

	# changing and crossing lanes is forbidden unless allowed by traffic regulations
	def setConstraint5OfCyclist(self, boolean: bool):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if type(item) == cpp_cyclist:
					item.setConstraint5(boolean)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")

	def setComputeM1Cyclist(self, boolean: bool):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if type(item) == cpp_cyclist:
					item.setCompute_Occ_M1(boolean)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")

	def setComputeM2Cyclist(self, boolean: bool):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if type(item) == cpp_cyclist:
					item.setCompute_Occ_M2(boolean)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")

	def setComputeM3Cyclist(self, boolean: bool):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if type(item) == cpp_cyclist:
					item.setCompute_Occ_M3(boolean)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")

	def setVsOfCyclist(self, v_s: float):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if type(item) == cpp_cyclist:
					item.setVs(v_s)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")

	def setbStopOfPedestrians(self, boolean: bool):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if isinstance(item, cpp_pedestrian):
					item.setbStop(boolean)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")	

	def setbCrossOfPedestrians(self, boolean: bool):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if isinstance(item, cpp_pedestrian):
					item.setbCross(boolean)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")

	def setbCrossOfPedestrians(self, boolean: bool):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if isinstance(item, cpp_pedestrian):
					item.setbCross(boolean)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")

	def setbSlackOfPedestrians(self, boolean: bool):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if isinstance(item, cpp_pedestrian):
					item.setbSlack(boolean)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")

	def setComputeBstop(self, val: bool):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if isinstance(item, cpp_pedestrian):
					item.setCompute_bStop(val)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")

	def setComputeBperp(self, val: bool):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if isinstance(item, cpp_pedestrian):
					item.setCompute_bPerp(val)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")

	def setComputeBslack(self, val: bool):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if isinstance(item, cpp_pedestrian):
					item.setCompute_bSlack(val)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")

	def setNotOnRoad(self, val: bool):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if isinstance(item, cpp_pedestrian):
					item.setNotOnRoad(val)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")

	def setAlpha_perp(self, val: float):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if isinstance(item, cpp_pedestrian):
					item.setAlpha_perp(val)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")

	def setdSlackOfPedestrians(self, boolean: bool):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if isinstance(item, cpp_pedestrian):
					item.setdSlack(boolean)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")


	def setdPerpOfPedestrians(self, val: float):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if isinstance(item, cpp_pedestrian):
					item.setdPerp(val)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")

	def setComputeOccRuleBasedOfPedestrians(self, boolean: bool):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if isinstance(item, cpp_pedestrian):
					item.setComputeOccRuleBased(boolean)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")


	def setComputeOccDynamicBasedOfPedestrians(self, boolean: bool):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if isinstance(item, cpp_pedestrian):
					item.setComputeOccDynamicBased(boolean)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")

	def setNumVertOfPedestrians(self, num: int): 
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if isinstance(item, cpp_pedestrian):
					item.setNumVert(num)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")

	def setAmaxOfPedestrians(self, a_max: float):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if isinstance(item, cpp_pedestrian):
					item.setAmax(a_max)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")

	def setAstopOfPedestrians(self, a_stop: float):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if isinstance(item, cpp_pedestrian):
					item.setAstop(a_stop)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")

	def setVmaxOfPedestrians(self, v_max: float):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if isinstance(item, cpp_pedestrian):
					item.setVmax(v_max)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")

