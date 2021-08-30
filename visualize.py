"""

MAPF Simulator

author: Justin Li (@justin871030)

"""

import sys
import time
sys.path.insert(0, './')
from cbs import CBS, PBS
from environment import Environment, Location, Agent, Agent_Task
from controller import Shelf_Place, Parking_Place, Station, Order, Controller
from copy import deepcopy
import matplotlib
from matplotlib.patches import Circle, Rectangle
import matplotlib.pyplot as plt
from matplotlib import animation

import argparse

from random import randint

def time_use(time_list):
    count = 0 
    for i in time_list:
        count += i
    print("used time: " + str(count) + "s")
    return

def save_map_information_at_the_time(env, shelfs, stations, parkings):
    shelf_locations = deepcopy(shelfs)
    shelf_now_locations = deepcopy(shelfs)
    station_locations = []
    for station in stations:
        station_locations += deepcopy(station)
    parking_locations = deepcopy(parkings)
    agent_locations = []
    for agent_name, agent in env.agent_dict.items():
        agent_locations.append(agent.location)
        if not agent.is_idle() and not agent.state.state == 1 and not agent.state.state == 7:
            for i in range(len(shelf_now_locations)):
                if(shelf_now_locations[i] == agent.task.shelf_location):
                    shelf_now_locations[i] = agent.location
    return [shelf_locations, station_locations, parking_locations, agent_locations, shelf_now_locations]
    
def make_order_list(shelfs, num): #random choose a shelf that a order need
    list = []
    for i in range(num):
        list.append(Order(shelfs[randint(0,len(shelfs)-1)]))
    return list

def get_default_test_data(order_num = 50):
    # 0s are the positions that the robot can reach, 1s are the forbidden positions
    # the stations, shelfs, parking places should all be obstacles
    map_obstacle_list = [
        [0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0],
        [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [1,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0],
        [1,0,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1],
        [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1],
        [1,0,1,1,0,1,1,0,1,1,0,1,1,0,0,1,1,1,1],
        [1,0,1,1,0,1,1,0,1,1,0,1,1,0,0,1,1,1,1],
        [1,0,1,1,0,1,1,0,1,1,0,1,1,0,0,0,0,0,0],
        [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],   
        [0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0] 
    ]
    #    0         5         10        15  

    #station_list is a list of stations that each station have a series of positions to place shelfs
    station_list = [
        [
        Location(15,3), Location(15,4), Location(15,5), Location(15,6), Location(16,3), Location(17,3),
        Location(18,3), Location(16,6), Location(17,6), Location(18,6)
        ]
    ]

    #station_list is a list of shelfs' positions where the shelfs are set
    shelf_list = [
        Location(2,2), Location(3,2), Location(4,2), Location(5,2), Location(6,2), Location(7,2),
        Location(8,2), Location(9,2), Location(10,2), Location(11,2), Location(12,2), Location(13,2),
        Location(2,3), Location(3,3), Location(4,3), Location(5,3), Location(6,3), Location(7,3),
        Location(8,3), Location(9,3), Location(10,3), Location(11,3), Location(12,3), Location(13,3),
        Location(2,5), Location(2,6), Location(2,7), Location(3,5), Location(3,6), Location(3,7),
        Location(5,5), Location(5,6), Location(5,7), Location(6,5), Location(6,6), Location(6,7),
        Location(8,5), Location(8,6), Location(8,7), Location(9,5), Location(9,6), Location(9,7),
        Location(11,5), Location(11,6), Location(11,7), Location(12,5), Location(12,6), Location(12,7),
    ]

    #parking_list is a list of parking places where the robots should "start from" and "back to" during a assignment of the task
    parking_list = [
        Location(0,1), Location(0,2), Location(0,3), Location(0,4), Location(0,5), Location(0,6), Location(0,7), Location(0,8)
    ]

    #agent_list is a list of robots which should be placed at the parking places at start that be set as the default position.
    agent_list = [
        Agent("agent0", parking_list[0], parking_list[0]),
        Agent("agent1", parking_list[1], parking_list[1]),
        Agent("agent2", parking_list[2], parking_list[2]),
        Agent("agent3", parking_list[3], parking_list[3]),
        Agent("agent4", parking_list[4], parking_list[4]),
        Agent("agent5", parking_list[5], parking_list[5])
    ]


    #agent_list is a list of orders including target shelves
    order_list = make_order_list(shelf_list, order_num)
    
    return map_obstacle_list, station_list, shelf_list, parking_list, agent_list, order_list

def run(env, map_obstacle_list, station_list, shelf_list, parking_list, agent_list, order_list , use_pbs = False):
    
    map_time_location_data = []
    map_time_count_data = []
    
    env.read_map_by_2d_list(map_obstacle_list)
    env.set_agents(deepcopy(agent_list))
    
    stations = []
    for station in station_list:
        stations.append(Station(deepcopy(station)))
        
    controller = Controller(env.agent_dict, Parking_Place(deepcopy(parking_list)), Shelf_Place(deepcopy(shelf_list)), stations)
    controller.init_parking_places_with_agents()
    controller.add_orders(order_list)
    
    if(use_pbs):
        cbs = PBS(env) 
    else:
        cbs = CBS(env) 

    time_count = 0
    
    map_time_location_data.append(save_map_information_at_the_time(env, shelf_list, station_list, parking_list))

    while time_count < env.total_run_time and controller.order_queue:
        env.assign_tasks(controller.deal_with_orders())
        
        time_start = time.time() #Timecount start

        solution = cbs.search()

        if not solution:
            print("solution not found")
            input()

        # Time count
        time_end = time.time()

        time_c= time_end - time_start 

        map_time_count_data.append(time_c)

        for i in range(env.time_step_per_planning):
            env.update_one_timestep()
            map_time_location_data.append(save_map_information_at_the_time(env, shelf_list, station_list, parking_list))

        tasks = env.return_finish_tasks()
        controller.deal_with_finished_tasks(tasks)
        
        time_count += env.time_step_per_planning
    
    return map_time_location_data, map_time_count_data 

def visualize(env, location_data, interval = 200):
    
    dimension = env.dimension

    aspect = dimension[0] / dimension[1]

    fig = plt.figure(frameon=False, figsize=(4 * aspect, 4))
    ax = fig.add_subplot(111, aspect='equal')
    fig.subplots_adjust(left=0,right=1,bottom=0,top=1, wspace=None, hspace=None)

    xmin = -0.5
    ymin = -0.5
    xmax = dimension[0] - 0.5
    ymax = dimension[1] - 0.5

    plt.xlim(xmin, xmax)
    plt.ylim(ymin, ymax)

    def animate(i):
        ax.patches = []
        ax.texts = []

        ax.add_patch(Rectangle((xmin, ymin), xmax - xmin, ymax - ymin, facecolor='white', edgecolor='black'))

        text = ax.text(0, 0, str(i))
        text.set_horizontalalignment('center')
        text.set_verticalalignment('center')

        time_data = location_data[i]

        for location in time_data[0]:
            x, y = location.x, location.y
            ax.add_patch(Rectangle((x - 0.5, y - 0.5), 1, 1, facecolor='none', edgecolor='blue', alpha = 1))

        for location in time_data[1]:
            x, y = location.x, location.y
            ax.add_patch(Rectangle((x - 0.5, y - 0.5), 1, 1, facecolor='none', edgecolor='red', alpha = 1))

        for location in time_data[2]:
            x, y = location.x, location.y
            ax.add_patch(Rectangle((x - 0.5, y - 0.5), 1, 1, facecolor='none', edgecolor='green', alpha = 1))

        for i in range(len(time_data[3])):
            x, y = time_data[3][i].x, time_data[3][i].y
            ax.add_patch(Circle((x, y), 0.3, facecolor= 'orange', edgecolor='black'))
            text = ax.text(x, y, str(i))
            text.set_horizontalalignment('center')
            text.set_verticalalignment('center')

        for location in time_data[4]:
            x, y = location.x, location.y
            ax.add_patch(Rectangle((x - 0.5, y - 0.5), 1, 1, facecolor='blue', edgecolor='black', alpha = 0.5))


    def init():
        ax.patches = []
        ax.texts = []
        
        ax.add_patch(Rectangle((xmin, ymin), xmax - xmin, ymax - ymin, facecolor='white', edgecolor='black'))

        time_data = location_data[0]

        for location in time_data[0]:
            x, y = location.x, location.y
            ax.add_patch(Rectangle((x - 0.5, y - 0.5), 1, 1, facecolor='none', edgecolor='blue', alpha = 1))

        for location in time_data[1]:
            x, y = location.x, location.y
            ax.add_patch(Rectangle((x - 0.5, y - 0.5), 1, 1, facecolor='none', edgecolor='red', alpha = 1))

        for location in time_data[2]:
            x, y = location.x, location.y
            ax.add_patch(Rectangle((x - 0.5, y - 0.5), 1, 1, facecolor='none', edgecolor='green', alpha = 1))

        for location in time_data[3]:
            x, y = location.x, location.y
            ax.add_patch(Circle((x, y), 0.3, facecolor= 'orange', edgecolor='black'))

        for location in time_data[4]:
            x, y = location.x, location.y
            ax.add_patch(Rectangle((x - 0.5, y - 0.5), 1, 1, facecolor='blue', edgecolor='black', alpha = 0.5))


    return animation.FuncAnimation(fig=fig, func=animate, frames=len(location_data), init_func=init,
                                  interval=interval, blit=False)

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("--pbs", type=bool, default=False, help="False: CBS, True: PBS")
  parser.add_argument("--window_size", type=int, default=15, help="window_size-factor")
  parser.add_argument("--time_step_per_planning", type=int, default=10, help="time_step_per_planning-factor")
  parser.add_argument("--total_run_time", type=int, default=50, help="total_run_time-factor")
  parser.add_argument("--order_num", type=int, default=20, help="order_num-factor")
  parser.add_argument('--video', dest='video', default=None, help="output video file (or leave empty to show on screen)")
  parser.add_argument("--speed", type=int, default=1, help="speedup-factor")
  args = parser.parse_args()



  map_obstacle_list, station_list, shelf_list, parking_list, agent_list, order_list = get_default_test_data(order_num = args.order_num)

  # see follow 20 steps without collisions and re-plan each 10 sec, search for following 100 sec from start
  env = Environment(window_size= args.window_size, time_step_per_planning= args.time_step_per_planning, total_run_time= args.total_run_time)

  map_time_location_data, map_time_count_data = run(env, map_obstacle_list, station_list, shelf_list, parking_list, agent_list, order_list, use_pbs= args.pbs)

  animation = visualize(env, map_time_location_data, 1000/args.speed)

  time_use(map_time_count_data)


  if args.video:
    animation.save(args.video)
  else:
    plt.show()