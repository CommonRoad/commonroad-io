

class car_time():
    def __init__(self,scenario):
        self.scenario=scenario

        self.car_time= [ [ 0 for y in range( 180 ) ]
                     for x in range(len(self.scenario.dynamic_obstacles)) ]

        for i in range(len(self.scenario.dynamic_obstacles)):

            for t in range(self.scenario.dynamic_obstacles[i].prediction.final_time_step):
                if t==0:
                    self.car_time[i][t]=self.scenario.lanelet_network.find_most_likely_lanelet_by_state([self.scenario.dynamic_obstacles[i].state_at_time(t)])[0]
                if t!=0:
                    self.car_time[i][t]=find_current_lanelet(t,self.car_time,i,self.scenario)





def point_in_lan(position,lanelet,scenario,j):
    for i in range(int(len(lanelet)/2)):
        a = (lanelet[i][0], lanelet[i][1])

        if abs(position[0]-a[0])<3.1699 and abs(position[1]-a[1])<3.1699:
                return True

    return False




def find_current_lanelet(t,list_car,i,scenario):

    if scenario.lanelet_network.find_lanelet_by_id(list_car[i][t-1]).successor==[]:
        return list_car[i][t-1]
    center=scenario.dynamic_obstacles[i].prediction.occupancy_at_time_step(t).shape.center
    for successor in scenario.lanelet_network.find_lanelet_by_id(list_car[i][t-1]).successor:
        if successor in scenario.lanelet_network.find_most_likely_lanelet_by_state([scenario.dynamic_obstacles[i].state_at_time(t)]):
            return successor
        if point_in_lan(center,scenario.lanelet_network.find_lanelet_by_id(successor).polygon.vertices,scenario,i):
            return successor
    if point_in_lan(center,scenario.lanelet_network.find_lanelet_by_id(list_car[i][t-1]).polygon.vertices,scenario,i):
        return list_car[i][t-1]

    return list_car[i][t-1]

