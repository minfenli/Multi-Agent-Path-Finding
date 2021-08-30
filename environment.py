"""

MAPF Simulator

author: Justin Li (@justin871030)

"""

import sys
sys.path.insert(0, './')


class Location:
    def __init__(self, x=-1, y=-1):
        self.x = x
        self.y = y
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    def __str__(self):
        return str((self.x, self.y))

class State:
    def __init__(self, time, location):
        self.time = time
        self.location = location
    def __eq__(self, other):
        return self.time == other.time and self.location == other.location
    def __hash__(self):
        return hash(str(self.time)+str(self.location.x) + str(self.location.y))
    def is_equal_except_time(self, state):
        return self.location == state.location
    def __str__(self):
        return str((self.time, self.location.x, self.location.y))
    
class Agent:
    def __init__(self, name, location, target):
        self.name = name
        self.location = location
        self.target = target
        self.path_list = []
        self.state = Agent_State(self)
        self.task = Agent_Task()
        self.finished_task = []

    def set_location(self, location):
        self.location = location
    
    def set_target(self, target):
        self.target = target
    
    def update_path(self, path_list):
        self.path_list = path_list
    
    def assign_task(self, task):
        self.task = task
        self.target = self.task.shelf_location
        self.state.state = 1
    
    def update_state(self): 
        if self.state.next(): # true if finish atask
            self.finished_task.append(self.task)

    def is_idle(self):
        return self.state.state == 0
    
    def __str__(self):
        return "Name: " + self.name  + \
            "Location: (" + str(self.location.x) + ", " + str(self.location.y) + ") " + \
            "Target: (" + str(self.target.x) + ", " + str(self.target.y) + ") " + \
            "State: (" + str(self.state) + ") "

class Agent_Task:
    def __init__(self, shelf_location = Location(), station_location = Location(), parking_location = Location()):
        self.shelf_location = shelf_location
        self.station_location = station_location
        self.parking_location = parking_location
    
class Agent_State:
    def __init__(self, agent, state = 0, hold_time_cost = 3, wait_time_cost = 3, put_time_cost = 3):
        
        self.state_dict = {0: "Idle", 1: "Move", 2:"Hold", 3:"Carry", 4:"Wait", 5: "Return", 6: "Put", 7: "Back"}
        self.state = state
        self.agent = agent
        self.hold_time_cost = hold_time_cost
        self.hold_time_count = 0
        self.wait_time_cost = wait_time_cost
        self.wait_time_count = 0
        self.put_time_cost = put_time_cost
        self.put_time_count = 0
    
    def __str__(self):
        return self.state_dict[self.state]
    
    def next(self): #True if finish a task
        if (self.state == 0):
            return False
        if (self.state == 1):
            if len(self.agent.path_list):
                self.agent.location = self.agent.path_list.pop(0)
                if self.agent.location == self.agent.target:
                    self.state = 2
                return False
            return False
        if (self.state == 2):
            if(self.hold_time_count < self.hold_time_cost):
                self.hold_time_count += 1
                return False
            self.agent.target = self.agent.task.station_location
            self.hold_time_count = 0
            self.state = 3
        if (self.state == 3):
            if len(self.agent.path_list):
                self.agent.location = self.agent.path_list.pop(0)
                if self.agent.location == self.agent.target:
                    self.state = 4
                return False
            return False
        if (self.state == 4):
            if(self.wait_time_count < self.wait_time_cost):
                self.wait_time_count += 1
                return False
            self.agent.target = self.agent.task.shelf_location
            self.wait_time_count = 0
            self.state = 5
        if (self.state == 5):
            if len(self.agent.path_list):
                self.agent.location = self.agent.path_list.pop(0)
                if self.agent.location == self.agent.target:
                    self.state = 6
                return False
            return False
        if (self.state == 6):
            if(self.put_time_cost < self.put_time_cost):
                self.put_time_cost += 1
                return False
            self.agent.target = self.agent.task.parking_location
            self.put_time_cost = 0
            self.state = 7
        if (self.state == 7):
            if len(self.agent.path_list):
                self.agent.location = self.agent.path_list.pop(0)
                if self.agent.location == self.agent.target:
                    self.state = 0
                    return True
            return False
            
    


class Environment:
    def __init__(self, dimension = [0,0], agents = [], obstacles = [], window_size = 10,\
                 time_step_per_planning = 10, total_run_time = 100):
        
        self.dimension = dimension
        
        self.obstacles = obstacles
        
        self.window_size = window_size
        
        self.time_step_per_planning = time_step_per_planning
        
        self.total_run_time = total_run_time
        
        self.agent_dict = {}

        self.make_agent_dict(agents)
        
    def read_map_by_2d_list(self, map_list = []):
        if not len(map_list):
            self.dimension = [0,0]
            self.obstacles = []
            return
        self.dimension = [len(map_list[0]), len(map_list)]
        self.obstacles = []
        for y in range(len(map_list)):
            for x in range(len(map_list[0])):
                if (map_list[y][x] == 1):
                    self.obstacles.append((x,y))
                    
    def update_one_timestep(self):
        for agent_name, agent in self.agent_dict.items():
            agent.update_state()
#             if(agent_name == "agent0"):
#                 print(agent)
        
    def set_agents(self, agents):
        self.make_agent_dict(agents)
        
    def assign_tasks(self, tasks):
        for agent_name, task in tasks.items():
            self.agent_dict[agent_name].assign_task(task)

    def return_finish_tasks(self):
        finish_task_list = []
        for agent_name, agent in self.agent_dict.items():
            finish_task_list += agent.finished_task
            agent.finished_task = []
        return finish_task_list
    
    def set_agent_target(self, agent_name, target):
        self.agent_dict[agent_name].target = target

    def make_agent_dict(self, agents):
        for agent in agents:
            self.agent_dict.update({agent.name : agent})
    
    def print_map_env(self):
        map_list = [['O']*self.dimension[0] for _ in range(self.dimension[1])]
        for obstacle in self.obstacles:
            map_list[obstacle[1]][obstacle[0]] = 'X'
        for agent_name, agent in self.agent_dict.items():
            map_list[agent.location.y][agent.location.x] = agent['name'][-1]
            map_list[agent.target.y][agent.target.x] = agent['name'][-1]
        return map_list