"""

MAPF Simulator

author: Justin Li (@justin871030)

"""

import sys
sys.path.insert(0, './')
from a_star import AStar
from environment import State, Location
from math import fabs
from itertools import combinations
from copy import deepcopy
from random import shuffle

class HighLevelNode:
    def __init__(self): #depth is number of tree node
        self.solution = {}
        self.constraint_dict = {}
        self.priority_list = []
        self.cost = 0

    def __eq__(self, other):
        if not isinstance(other, type(self)): return NotImplemented
        return self.solution == other.solution and self.cost == other.cost

    def __hash__(self):
        return hash(self.cost)
    
    def __lt__(self, other):
        return self.cost < other.cost

class Conflict:
    VERTEX = 1
    EDGE = 2
    def __init__(self):
        self.time = -1
        self.type = -1

        self.agent_1 = ''
        self.agent_2 = ''

        self.location_1 = Location()
        self.location_2 = Location()

    def __str__(self):
        return '(' + str(self.time) + ', ' + self.agent_1 + ', ' + self.agent_2 + \
             ', '+ str(self.location_1) + ', ' + str(self.location_2) + ')'

class VertexConstraint:
    def __init__(self, time, location):
        self.time = time
        self.location = location

    def __eq__(self, other):
        return self.time == other.time and self.location == other.location
    def __hash__(self):
        return hash(str(self.time)+str(self.location))
    def __str__(self):
        return '(' + str(self.time) + ', '+ str(self.location) + ')'

class EdgeConstraint:
    def __init__(self, time, location_1, location_2):
        self.time = time
        self.location_1 = location_1
        self.location_2 = location_2
    def __eq__(self, other):
        return self.time == other.time and self.location_1 == other.location_1 \
            and self.location_2 == other.location_2
    def __hash__(self):
        return hash(str(self.time) + str(self.location_1) + str(self.location_2))
    def __str__(self):
        return '(' + str(self.time) + ', '+ str(self.location_1) +', '+ str(self.location_2) + ')'

class Constraints:
    def __init__(self):
        self.vertex_constraints = set()
        self.edge_constraints = set()

    def add_constraint(self, other):
        self.vertex_constraints |= other.vertex_constraints
        self.edge_constraints |= other.edge_constraints

    def __str__(self):
        return "VC: " + str([str(vc) for vc in self.vertex_constraints])  + \
            "EC: " + str([str(ec) for ec in self.edge_constraints])

class CBS:
    def __init__(self, environment):
        self.env = environment
        self.a_star = AStar(self)
        self.constraints = Constraints()
        self.constraint_dict = {}
        
    def search(self):
        
        open_set = set()
        closed_set = set()
        start = HighLevelNode()
        # TODO: Initialize it in a better way
        start.constraint_dict = {}
        for agent in self.env.agent_dict.keys():
            start.constraint_dict[agent] = Constraints()
        start.solution = self.compute_solution()
        
        #self.print_solution(start.solution)
        
        if not start.solution:
            return {}
        start.cost = self.compute_solution_cost(start.solution)

        open_set |= {start}

        while open_set:
            P = min(open_set)
            open_set -= {P}
            closed_set |= {P}

            self.constraint_dict = P.constraint_dict
            conflict_dict = self.get_first_conflict(P.solution)
            if not conflict_dict:
                print("solution found")
                
                self.update_path_list(P.solution)

                return self.generate_plan(P.solution)

            constraint_dict = self.create_constraints_from_conflict(conflict_dict)

            for agent in constraint_dict.keys():
                new_node = deepcopy(P)
                new_node.constraint_dict[agent].add_constraint(constraint_dict[agent])

                self.constraint_dict = new_node.constraint_dict
                new_node.solution = self.compute_solution()
                if not new_node.solution:
                    continue
                new_node.cost = self.compute_solution_cost(new_node.solution)

                # TODO: ending condition
                if new_node not in closed_set:
                    open_set |= {new_node}
        for agent in self.env.agent_dict.values():
            print(agent.location,agent.target)

        return {}
    
    def get_neighbors(self, state, agent_name):
        neighbors = []

        # Wait action
        n = State(state.time + 1, state.location)
        if self.state_valid(n, agent_name):
            neighbors.append(n)
        # Up action
        n = State(state.time + 1, Location(state.location.x, state.location.y+1))
        if self.state_valid(n, agent_name) and self.transition_valid(state, n):
            neighbors.append(n)
        # Down action
        n = State(state.time + 1, Location(state.location.x, state.location.y-1))
        if self.state_valid(n, agent_name) and self.transition_valid(state, n):
            neighbors.append(n)
        # Left action
        n = State(state.time + 1, Location(state.location.x-1, state.location.y))
        if self.state_valid(n, agent_name) and self.transition_valid(state, n):
            neighbors.append(n)
        # Right action
        n = State(state.time + 1, Location(state.location.x+1, state.location.y))
        if self.state_valid(n, agent_name) and self.transition_valid(state, n):
            neighbors.append(n)
        return neighbors
    
    def state_wait(self, state):
        return State(state.time + 1, state.location)


    def get_first_conflict(self, solution):
        #print(solution['agent1'])
        max_t = max([len(plan) for plan in solution.values()])
        result = Conflict()
        for t in range(max_t):
            for agent_1, agent_2 in combinations(solution.keys(), 2):
                state_1 = self.get_state(agent_1, solution, t)
                state_2 = self.get_state(agent_2, solution, t)
                if state_1.is_equal_except_time(state_2):
                    result.time = t
                    result.type = Conflict.VERTEX
                    result.location_1 = state_1.location
                    result.agent_1 = agent_1
                    result.agent_2 = agent_2
                    return result

            for agent_1, agent_2 in combinations(solution.keys(), 2):
                state_1a = self.get_state(agent_1, solution, t)
                state_1b = self.get_state(agent_1, solution, t+1)

                state_2a = self.get_state(agent_2, solution, t)
                state_2b = self.get_state(agent_2, solution, t+1)

                if state_1a.is_equal_except_time(state_2b) and state_1b.is_equal_except_time(state_2a):
                    result.time = t
                    result.type = Conflict.EDGE
                    result.agent_1 = agent_1
                    result.agent_2 = agent_2
                    result.location_1 = state_1a.location
                    result.location_2 = state_1b.location
                    return result
        return False

    def create_constraints_from_conflict(self, conflict):
        constraint_dict = {}
        if conflict.type == Conflict.VERTEX:
            v_constraint = VertexConstraint(conflict.time, conflict.location_1)
            constraint = Constraints()
            constraint.vertex_constraints |= {v_constraint}
            constraint_dict[conflict.agent_1] = constraint
            constraint_dict[conflict.agent_2] = constraint

        elif conflict.type == Conflict.EDGE:
            constraint1 = Constraints()
            constraint2 = Constraints()

            e_constraint1 = EdgeConstraint(conflict.time, conflict.location_1, conflict.location_2)
            e_constraint2 = EdgeConstraint(conflict.time, conflict.location_2, conflict.location_1)

            constraint1.edge_constraints |= {e_constraint1}
            constraint2.edge_constraints |= {e_constraint2}

            constraint_dict[conflict.agent_1] = constraint1
            constraint_dict[conflict.agent_2] = constraint2

        return constraint_dict
    
    def get_state(self, agent_name, solution, t):
        if t < len(solution[agent_name]):
            return solution[agent_name][t]
        else:
            return solution[agent_name][-1]

    def state_valid(self, state, agent_name):
        if(self.env.agent_dict[agent_name].is_idle()):
            if State(0, self.env.agent_dict[agent_name].location).is_equal_except_time(state):
                return True
        else:
            if State(0, self.env.agent_dict[agent_name].task.shelf_location).is_equal_except_time(state):
                return True
            if State(0, self.env.agent_dict[agent_name].task.parking_location).is_equal_except_time(state):
                return True
            if State(0, self.env.agent_dict[agent_name].task.station_location).is_equal_except_time(state):
                return True
        return state.location.x >= 0 and state.location.x < self.env.dimension[0] \
            and state.location.y >= 0 and state.location.y < self.env.dimension[1] \
            and VertexConstraint(state.time, state.location) not in self.constraints.vertex_constraints \
            and (state.location.x, state.location.y) not in self.env.obstacles

    def transition_valid(self, state_1, state_2):
        return EdgeConstraint(state_1.time, state_1.location, state_2.location) not in self.constraints.edge_constraints

    def is_solution(self, agent_name):
        pass

    def admissible_heuristic(self, state, agent_name):
        goal = self.env.agent_dict[agent_name].target
        return fabs(state.location.x - goal.x) + fabs(state.location.y - goal.y)

    def get_location_state(self, agent_name):
        return State(0, self.env.agent_dict[agent_name].location)
    
    def is_at_goal(self, state, agent_name):
        goal_state = State(0, self.env.agent_dict[agent_name].target)
        return state.is_equal_except_time(goal_state)
    
    def compute_solution(self):
        solution = {}
        for agent in self.env.agent_dict.keys():
            self.constraints = self.constraint_dict.setdefault(agent, Constraints())
            local_solution = self.a_star.search(agent)
            if not local_solution:
                return False
            solution.update({agent:local_solution})
        return solution
    

    def compute_solution_cost(self, solution):
        return sum([len(path) for path in solution.values()])
    
    def update_states(self, solution, time):
        for agent, path in solution.items():
            for state in path:
                if(state['t'] == time):
                    self.env.agent_dict[agent].location = Location(state['x'], state['y'])
    
    def generate_plan(self, solution):
        plan = {}
        for agent, path in solution.items():
            path_dict_list = [{'t':state.time, 'x':state.location.x, 'y':state.location.y} for state in path]
            plan[agent] = path_dict_list
        return plan
    
    def update_path_list(self, solution):
        plan = {}
        for agent, path in solution.items():
            path_list = [state.location for state in path]
            path_list.pop(0)
            self.env.agent_dict[agent].update_path(path_list)
    
    def print_solution(self, solution):
        for agent, path in solution.items():
            path_dict_list = [{'t':state.time, 'x':state.location.x, 'y':state.location.y} for state in path]
            print(agent)
            print(path_dict_list)


class PBS:
    def __init__(self, environment):
        self.env = environment
        self.a_star = AStar(self)
        self.constraints = Constraints()
        self.constraint_dict = {}
        self.retry_time_if_fail = 5
        
    def search(self):
        
        start = HighLevelNode()
        
        for agent_name in self.env.agent_dict.keys():
            start.priority_list.append(agent_name) 
        print(start.priority_list)
#        print(self.constraints)
        
        start.solution = self.compute_solution(start.priority_list)
        
        if start.solution:
            
            print("solution found")
                
            self.update_path_list(start.solution)
            
            return self.generate_plan(start.solution)
        
        count = 0
        
        while count < self.retry_time_if_fail:
            
            shuffle(start.priority_list)
            print(start.priority_list)
            start.solution = self.compute_solution(start.priority_list)
        
            if start.solution:
            
                print("solution found")
                    
                self.update_path_list(start.solution)
                
                return self.generate_plan(start.solution)
            
            count += 1
        

        return {}
    
    def get_neighbors(self, state, agent_name):
        neighbors = []

        # Wait action
        n = State(state.time + 1, state.location)
        if self.state_valid(n, agent_name):
            neighbors.append(n)
        # Up action
        n = State(state.time + 1, Location(state.location.x, state.location.y+1))
        if self.state_valid(n, agent_name) and self.transition_valid(state, n):
            neighbors.append(n)
        # Down action
        n = State(state.time + 1, Location(state.location.x, state.location.y-1))
        if self.state_valid(n, agent_name) and self.transition_valid(state, n):
            neighbors.append(n)
        # Left action
        n = State(state.time + 1, Location(state.location.x-1, state.location.y))
        if self.state_valid(n, agent_name) and self.transition_valid(state, n):
            neighbors.append(n)
        # Right action
        n = State(state.time + 1, Location(state.location.x+1, state.location.y))
        if self.state_valid(n, agent_name) and self.transition_valid(state, n):
            neighbors.append(n)
        return neighbors
    
    def state_wait(self, state):
        return State(state.time + 1, state.location)


    def get_first_conflict(self, solution):
        print(solution['agent1'])
        max_t = max([len(plan) for plan in solution.values()])
        result = Conflict()
        for t in range(max_t):
            for agent_1, agent_2 in combinations(solution.keys(), 2):
                state_1 = self.get_state(agent_1, solution, t)
                state_2 = self.get_state(agent_2, solution, t)
                if state_1.is_equal_except_time(state_2):
                    result.time = t
                    result.type = Conflict.VERTEX
                    result.location_1 = state_1.location
                    result.agent_1 = agent_1
                    result.agent_2 = agent_2
                    return result

            for agent_1, agent_2 in combinations(solution.keys(), 2):
                state_1a = self.get_state(agent_1, solution, t)
                state_1b = self.get_state(agent_1, solution, t+1)

                state_2a = self.get_state(agent_2, solution, t)
                state_2b = self.get_state(agent_2, solution, t+1)

                if state_1a.is_equal_except_time(state_2b) and state_1b.is_equal_except_time(state_2a):
                    result.time = t
                    result.type = Conflict.EDGE
                    result.agent_1 = agent_1
                    result.agent_2 = agent_2
                    result.location_1 = state_1a.location
                    result.location_2 = state_1b.location
                    return result
        return False

    def create_constraints_from_path(self, solution):
        if(not len(solution)):
            return
        self.constraints.vertex_constraints |= {VertexConstraint(solution[0].time, solution[0].location)}
        for i in range(1,len(solution)):
            self.constraints.vertex_constraints |= {VertexConstraint(solution[i].time, solution[i].location)}
            self.constraints.edge_constraints |= {EdgeConstraint(solution[i-1].time, solution[i].location, solution[i-1].location)}

    
    def get_state(self, agent_name, solution, t):
        if t < len(solution[agent_name]):
            return solution[agent_name][t]
        else:
            return solution[agent_name][-1]

    def state_valid(self, state, agent_name):
        if(self.env.agent_dict[agent_name].is_idle()):
            if State(0, self.env.agent_dict[agent_name].location).is_equal_except_time(state):
                return True
        else:
            if State(0, self.env.agent_dict[agent_name].task.shelf_location).is_equal_except_time(state):
                return True
            if State(0, self.env.agent_dict[agent_name].task.parking_location).is_equal_except_time(state):
                return True
            if State(0, self.env.agent_dict[agent_name].task.station_location).is_equal_except_time(state):
                return True
        return state.location.x >= 0 and state.location.x < self.env.dimension[0] \
            and state.location.y >= 0 and state.location.y < self.env.dimension[1] \
            and VertexConstraint(state.time, state.location) not in self.constraints.vertex_constraints \
            and (state.location.x, state.location.y) not in self.env.obstacles

    def transition_valid(self, state_1, state_2):
        return EdgeConstraint(state_1.time, state_1.location, state_2.location) not in self.constraints.edge_constraints

    def is_solution(self, agent_name):
        pass

    def admissible_heuristic(self, state, agent_name):
        goal = self.env.agent_dict[agent_name].target
        return fabs(state.location.x - goal.x) + fabs(state.location.y - goal.y)

    def get_location_state(self, agent_name):
        return State(0, self.env.agent_dict[agent_name].location)
    
    def is_at_goal(self, state, agent_name):
        goal_state = State(0, self.env.agent_dict[agent_name].target)
        return state.is_equal_except_time(goal_state)
    
    def compute_solution(self, priority_list):
        solution = {}
        self.constraints = Constraints()
        for agent in priority_list:
            local_solution = self.a_star.search(agent)
            if not local_solution:
                return False #被卡住還有優先級高的人想塞進來
            solution.update({agent:local_solution})
            self.create_constraints_from_path(local_solution)
        return solution
    

    def compute_solution_cost(self, solution):
        return sum([len(path) for path in solution.values()])
    
    def update_states(self, solution, time):
        for agent, path in solution.items():
            for state in path:
                if(state['t'] == time):
                    self.env.agent_dict[agent].location = Location(state['x'], state['y'])
    
    def generate_plan(self, solution):
        plan = {}
        for agent, path in solution.items():
            path_dict_list = [{'t':state.time, 'x':state.location.x, 'y':state.location.y} for state in path]
            plan[agent] = path_dict_list
        return plan
    
    def update_path_list(self, solution):
        plan = {}
        for agent, path in solution.items():
            path_list = [state.location for state in path]
            path_list.pop(0)
            self.env.agent_dict[agent].update_path(path_list)
    
    def print_solution(self, solution):
        for agent, path in solution.items():
            path_dict_list = [{'t':state.time, 'x':state.location.x, 'y':state.location.y} for state in path]
            print(agent)
            print(path_dict_list)