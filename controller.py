"""

MAPF Simulator

author: Justin Li (@justin871030)

"""

import sys
sys.path.insert(0, './')
from math import fabs
from environment import Agent_Task

class Order:
    def __init__(self, shelf_location):
        self.shelf_location = shelf_location
    def set_shelf_location(location):
        self.shelf_location = shelf_location
        
class Station:
    def __init__(self, locations):
        self.idle_locations = locations
        self.busy_locations = []
    
    def get_idle_location(self):
        if not self.idle_locations:
            return False
        location = self.idle_locations.pop(-1)
        self.busy_locations.append(location)
        return location
    
    def free_busy_location(self, location):
        for busy_location_index in range(len(self.busy_locations)):
            if(location == self.busy_locations[busy_location_index]):
                self.idle_locations.append(self.busy_locations.pop(busy_location_index))
                break

class Parking_Place:
    def __init__(self, locations):
        self.idle_locations = locations
        self.busy_locations = []
    
    def get_nearest_idle_location(self, start_location):
        if not self.idle_locations:
            return False
        
        temp_distance = float('inf')
        temp_index = -1

        for index in range(len(self.idle_locations)):
            distance = self.location_distance(self.idle_locations[index], start_location)
            if(distance < temp_distance):
                temp_distance = distance
                temp_index = index
        location = self.idle_locations.pop(temp_index)
        self.busy_locations.append(location)
        
        return location
    
    def get_idle_location(self):
        if not self.idle_locations:
            return False
        
        location = self.idle_locations.pop(-1)
        self.busy_locations.append(location)
        return location
    
    def free_busy_location(self, location):
        for busy_location_index in range(len(self.busy_locations)):
            if(location == self.busy_locations[busy_location_index]):
                self.idle_locations.append(self.busy_locations.pop(busy_location_index))
                break
                    
    def location_distance(self, location1, location2):
        return fabs(location1.x - location2.x) + fabs(location1.y - location2.y)

class Shelf_Place:
    def __init__(self, locations):
        self.idle_locations = locations
        self.busy_locations = []
    
    def get_location(self, target):
        for index in range(len(self.idle_locations)):
            if(self.idle_locations[index] == target):
                location = self.idle_locations.pop(index)
                self.busy_locations.append(location)
                return location
        return False
    
    def free_busy_location(self, location):
        for busy_location_index in range(len(self.busy_locations)):
            if(location == self.busy_locations[busy_location_index]):
                self.idle_locations.append(self.busy_locations.pop(busy_location_index))
                break

    
class Controller:
    def __init__(self, agent_dict, parking_place, shelf_place, station_list):
        self.agent_dict = agent_dict
        self.parking_place = parking_place
        self.shelf_place = shelf_place
        self.station_list = station_list
        self.order_queue = []
        
    def init_parking_places_with_agents(self):
        for agent_name, agent in self.agent_dict.items():
            for index in range(len(self.parking_place.idle_locations)):
                if(agent.location == self.parking_place.idle_locations[index]):
                    self.parking_place.busy_locations.append(self.parking_place.idle_locations.pop(index))
                    break
    
    def add_orders(self, orders):
        self.order_queue = self.order_queue + orders
        
    def deal_with_orders(self):
        if not self.order_queue:
            return {}
        
        except_list = []
        
        task_dict = {}
        
        index = 0 
        
        while(index < len(self.order_queue)):
            task = self.deal_with_one_order(self.order_queue[index], except_list)
            if task:
                task_dict.update(task)
                self.order_queue.pop(index)
            else:
                index += 1

        return task_dict
                
        
    def deal_with_one_order(self, order, except_list):
        
        shelf_location = self.shelf_place.get_location(order.shelf_location)

        if(shelf_location):
            agent_name = self.get_nearest_agent(order.shelf_location, except_list)
            if(agent_name):
                parking_location = self.agent_dict[agent_name].location
                if(parking_location):
                    for station in self.station_list:
                        station_location = station.get_idle_location()
                        if(station_location):
                            except_list.append(agent_name)
                            return {agent_name: Agent_Task(shelf_location, station_location, parking_location)}
                        
                    #if fail, free
                    self.parking_place.free_busy_location(parking_location)
            else:
                #if fail, free
                self.shelf_place.free_busy_location(shelf_location)
                
        return False
    
    def deal_with_finished_tasks(self, tasks):
        for task in tasks:
            self.shelf_place.free_busy_location(task.shelf_location)
            for station in self.station_list:
                station.free_busy_location(task.station_location)
        
    def location_distance(self, location1, location2):
        return fabs(location1.x - location2.x) + fabs(location1.y - location2.y)
    
    def get_nearest_agent(self, target, except_list):
        temp_distance = float('inf')
        temp_name = ""
        
        for agent_name, agent in self.agent_dict.items():
            if(agent.is_idle() and agent_name not in except_list):
                distance = self.location_distance(agent.location, target)
                if(distance < temp_distance):
                    temp_distance = self.location_distance(agent.location, target)
                    temp_name = agent_name
        if(temp_name == ""):
            return False
        return temp_name
    
    def __str__(self):
        string = ""
        for i in self.order_queue:
            string += str(i.shelf_location) + "\n"
        return string