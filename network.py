import traci
import xml.etree.ElementTree as ET
import numpy as np

from agent_based_control.signal_agent import get_movement_num


def check_band_coverage(g_range, g_bar_range, total_cycle_length=60, max_coverage_ratio=0.8):
    """检查绿波区段是否覆盖过多时间，避免覆盖整个周期从而使 fallback 失效。

    Args:
        g_range (list of tuple): 主 green band 时间段 [(start, end), ...]
        g_bar_range (list of tuple): 次级 green band 时间段 [(start, end), ...]
        total_cycle_length (int): 总周期长度（秒）
        max_coverage_ratio (float): 最大允许覆盖比例（例如 0.8 表示最多覆盖 80%）
    """
    # 合并所有区间
    combined_ranges = g_range + g_bar_range

    # 构建一个周期时间轴数组，每秒是否被某个区间覆盖
    covered = [0] * total_cycle_length

    for start, end in combined_ranges:
        for t in range(start, end + 1):
            if 0 <= t < total_cycle_length:
                covered[t] = 1

    covered_seconds = sum(covered)
    coverage_ratio = covered_seconds / total_cycle_length

    if coverage_ratio > max_coverage_ratio:
        print(
            f"❌ Green band time covers too much: {covered_seconds}s out of {total_cycle_length}s "
            f"({coverage_ratio * 100:.1f}%), which exceeds the allowed limit of {max_coverage_ratio * 100:.0f}%."
            f"{g_range, g_bar_range, total_cycle_length}"
        )

def range_check(a, range_list, step_length):
    a = (a//step_length) * step_length
    for r in range_list:
        aligned_start = (r[0] // step_length) * step_length
        aligned_end = (r[1] // step_length) * step_length

        if a >= aligned_start and a <= aligned_end:
            return True, aligned_start
    return False, -1



def range_check_saturated(a, range_list, step_length):
    if a < range_list[0][0]:
        return False, False, -1
    a_aligned = (a // step_length) * step_length

    '''
    a_aligned = (a // step_length) * step_length
    first_start_aligned = (range_list[0][0] // step_length) * step_length

    if a_aligned < first_start_aligned:
        return False, False, -1
    '''
    for r in range_list:
        aligned_start = (r[0] // step_length) * step_length
        aligned_end = (r[1] // step_length) * step_length

        if a_aligned >= aligned_start and a_aligned <= aligned_end:
            return True, True, aligned_start

    return False, True, -1


class Coordinated_plan:
    def __init__(self, id=0, start_time=0, duration=3600, type=None, coordinated_path=None):
        self.id = id
        self.start_time = start_time
        self.duration = duration
        self.end_time = self.start_time+duration
        self.type = type
        self.coordinated_path = coordinated_path

class trafficlight:
    def __init__(self,id,program=None,phase=None,duration=None):
        self.id = id
        self.program=program
        self.phase=phase
        self.duration=duration

class Connection:
    def __init__(self,id=None,name=None,lane=None, outlane = None, cfrom=None, cto = None, fromlane = None, tolane = None, dir = None, type = None, tl = None, linkindex =0):

        self.id=id
        self.name=name
        self.lane=lane
        self.outlane=outlane
        self.cfrom=cfrom
        self.cto=cto
        self.fromlane=fromlane
        self.tolane=tolane
        self.dir=dir
        self.type=type
        self.tl=tl
        self.linkindex=linkindex
        self.lanenum=1 #1 1/2 1/3...取决于有几个connection共用该lane
        self.throughput=0

class Edge:
    def __init__(self,id, efrom=None, eto=None, frompos=0, topos=0,length=0, numlane=0,max_speed=None):
        self.id=id
        self.reverse=0
        self.efrom=efrom
        self.eto=eto
        self.frompos=frompos
        self.topos=topos
        self.length =length
        self.vehicle_number=[]
        self.density=0
        self.twodir_density=0
        self.density_interval=[]
        self.twodir_density_interval=[]
        self.numlane=numlane
        self.max_speed=max_speed

        self.speed_list=[]
        self.travel_time=0
        self.num_speed=5 #5 to get the mean speed

    def update_speed(self,speed):
        if len(self.speed_list)<self.num_speed:
            self.speed_list.append(speed)
        else:
            self.speed_list=self.speed_list[1:]
            self.speed_list.append(speed)

class Node:
    def __init__(self, id,  step_length=1, position=None, index_number=0, initial_phases_list=None, connections = None, tl_node_list = None):
        self.tl_type = 'isolated_control'
        self.step_length = step_length
        self.id=id
        self.position = position
        self.index_number = index_number
        self.initial_phases_list = initial_phases_list
        self.connections = connections
        self.tl_node_list = tl_node_list

        #self.neighbor =[]
        #self.neighbor_nodes = []
        self.arrivals = {}
        self.arrivals_perlane = {}
        self.arrival_vehicles={}
        self.connections_by_movement = {}
        self.lanes_by_movement = {}
        self.outlanes_by_movement = {}
        self.tl_index = {}
        self.phases = {}
        self.phase_names = []
        self.last_phase_names = ['s_bars']

        self.prev_phase = []

        self.coordinate_start_time=0
        self.C=0
        self.g_range = []
        self.g_bar_range = []
        self.b_start = 0
        self.b_bar_start = 0

        self.reward_queue = 0.0
        self.reward_vehwait = 0.0

        self.last_step_running_veh = {}
        self.reward_main_line_avg_speed = 0
        self.reward_main_line_stop = 0
        self.state_main_line_queue = [0,0]
        #self.state_main_line_stop = [0,0]
        #self.state_main_line_speed = [0,0]


        self.state_queue = []
        self.state_arrival = []
        #self.state_coord = []
        self.state_related_node = []
        self.state_speed = []

        #self.state_outlane_veh_num = []

        self.travel_time = 0
        self.waiting_time = 0
        self.one_dir_coord_path_travel_time = 0
        self.two_dir_coord_path_travel_time = 0
        self.one_dir_coord_path_waiting_time = 0
        self.two_dir_coord_path_waiting_time = 0

        self.right_movements = ['r', 'bar_r','branch_r','branch_bar_r']
        self.one_dir_coord_path_movement = ['s']
        self.two_dir_coord_path_movement = ['s','bar_s']


        self.record_window=600

    def update_arrival_rate(self):
        # 初始化到达车辆集合
        #self.arrival_vehicles = {name: {} for name in self.lanes_by_movement.keys()}
        self.arrival_vehicles = {name: set() for name in self.lanes_by_movement.keys()}

    def record_arrival_rate(self):
        for name in self.lanes_by_movement:
            for lane in self.lanes_by_movement[name]:
                self.arrival_vehicles[name].add(traci.lane.getLastStepVehicleIDs(lane))
            '''
            if name not in self.arrival_vehicles:
                self.arrival_vehicles[name] = {}

            if self.arrival_vehicles[name]:
                self.arrival_vehicles[name] = {
                    veh: ts for veh, ts in self.arrival_vehicles[name].items()
                    if cur_sec - ts <= self.record_window
                }

            for lane in self.lanes_by_movement[name]:
                vehs = traci.lane.getLastStepVehicleIDs(lane)
                for veh in vehs:
                    if veh not in self.arrival_vehicles[name]:
                        self.arrival_vehicles[name][veh] = cur_sec
            '''
    def update_reward(self):
        queues = []
        vehwaits = []

        all_lanes = [lane for lanes in self.lanes_by_movement.values() for lane in lanes]

        for lane in all_lanes:
            # 队列长度
            halting_num = traci.lane.getLastStepHaltingNumber(lane)
            queues.append(halting_num)

            max_wait = 0
            if halting_num > 0:
                vehs = traci.lane.getLastStepVehicleIDs(lane)
                max_pos = -1
                for vid in vehs:
                    pos = traci.vehicle.getLanePosition(vid)
                    if pos > max_pos:
                        max_pos = pos
                        max_wait = traci.vehicle.getWaitingTime(vid)
            vehwaits.append(max_wait)

        self.reward_queue = float(np.sum(queues)) if queues else 0.0
        self.reward_vehwait = float(np.sum(vehwaits)) if vehwaits else 0.0

    def update_high_level_state_and_reward(self):
        # main_lanes: dict, key=方向名, value=该方向上的 lane 列表
        main_lanes = {dir_: self.lanes_by_movement[dir_] for dir_ in self.two_dir_coord_path_movement}

        self.state_main_line_queue = []
        #self.state_main_line_stop = []
        #self.state_main_line_speed = []

        total_speed = []  # 用于 reward
        total_stops = 0

        for dir_, lanes in main_lanes.items():
            #dir_speed = []
            #dir_stops = 0

            for lane in lanes:
                vehs = traci.lane.getLastStepVehicleIDs(lane)
                running_veh = []
                for veh in vehs:
                    veh_speed = traci.vehicle.getSpeed(veh)
                    #dir_speed.append(veh_speed)
                    total_speed.append(veh_speed)

                    if veh_speed < 0.1:
                        # 车辆在上一 step 也是 running_veh 才算 stop
                        if veh in self.last_step_running_veh.get(lane, []):
                            #dir_stops += 1
                            total_stops += 1
                    else:
                        running_veh.append(veh)
                self.last_step_running_veh[lane] = running_veh
            '''
            # 每个方向的状态值
            if len(dir_speed) > 0:
                avg_speed = np.mean(dir_speed)
            else:
                avg_speed = traci.lane.getMaxSpeed(lanes[0])

            self.state_main_line_speed.append(avg_speed)
            self.state_main_line_stop.append(dir_stops)
            '''
            self.state_main_line_queue.append(sum([traci.lane.getLastStepHaltingNumber(lane) for lane in lanes]))

        # Reward 仍然使用所有主干道车辆的平均速度
        if len(total_speed) > 0:
            self.reward_main_line_avg_speed = np.mean(total_speed)
        else:
            # 选第一个方向的第一个 lane 的 max speed 作为缺省
            first_dir = list(main_lanes.keys())[0]
            self.reward_main_line_avg_speed = traci.lane.getMaxSpeed(main_lanes[first_dir][0])

        self.reward_main_line_stop = total_stops


    def update_state(self):
        self.state_queue = []
        self.state_arrival = []
        self.state_speed = []

        #self.state_outlane_veh_num = []

        self.travel_time = 0
        self.waiting_time = 0
        self.one_dir_coord_path_travel_time = 0
        self.two_dir_coord_path_travel_time = 0
        self.one_dir_coord_path_waiting_time = 0
        self.two_dir_coord_path_waiting_time = 0
        '''
        for name in self.outlanes_by_movement:
            outlane_veh_num = []
            for lane in self.outlanes_by_movement[name]:
                veh_num = traci.lane.getLastStepVehicleNumber(lane)
                outlane_veh_num.append(veh_num)
            self.state_outlane_veh_num.append(np.mean(outlane_veh_num))
        '''
        for name in self.lanes_by_movement:
            arrivals = []
            queues = []
            speeds = []
            for lane in self.lanes_by_movement[name]:
                arrivals.append(traci.lane.getLastStepVehicleNumber(lane))
                queues.append(traci.lane.getLastStepHaltingNumber(lane))
                speeds.append(traci.lane.getLastStepMeanSpeed(lane))

            waiting_time = np.sum(queues) * self.step_length
            self.waiting_time += waiting_time
            travel_time = np.sum(arrivals) * self.step_length
            self.travel_time += travel_time

            if name in self.one_dir_coord_path_movement:
                self.one_dir_coord_path_travel_time += travel_time
                self.one_dir_coord_path_waiting_time += waiting_time
            if name in self.two_dir_coord_path_movement:
                self.two_dir_coord_path_travel_time += travel_time
                self.two_dir_coord_path_waiting_time += waiting_time

            self.state_arrival.append(np.mean(arrivals))
            self.state_queue.append(np.mean(queues))
            self.state_speed.append(np.mean(speeds))


    def get_info(self):

        info_dict = {"waiting_time": self.waiting_time, "travel_time": self.travel_time,
                     "one_dir_coord_path_waiting_time": self.one_dir_coord_path_waiting_time,
                     "two_dir_coord_path_waiting_time":self.two_dir_coord_path_waiting_time,
                     "one_dir_coord_path_travel_time": self.one_dir_coord_path_travel_time,
                     "two_dir_coord_path_travel_time": self.two_dir_coord_path_travel_time,
                     "reward_queue": self.reward_queue, "reward_vehwait": self.reward_vehwait}

        return info_dict

    def average_arrival_rate(self):
        queues = {name: 0 for name in self.lanes_by_movement.keys()}
        for name in self.lanes_by_movement:
            for lane in self.lanes_by_movement[name]:
                queues[name] += traci.lane.getLastStepVehicleNumber(lane)

        self.arrivals = {}
        self.arrivals_perlane = {}
        for name in self.lanes_by_movement:
            num_vehicles = max(len(self.arrival_vehicles.get(name, {})) - queues.get(name, 0),0)
            num_lanes = len(self.lanes_by_movement[name])
            self.arrivals[name] = num_vehicles / self.record_window
            self.arrivals_perlane[name] = num_vehicles / (num_lanes * self.record_window)


    def coordinated_feasible_phases(self,cur_sec):

        if self.tl_type == 'green_band_control':
            timing = cur_sec - self.coordinate_start_time
            timing = timing % self.C
            phase_names= self.get_phase_green_band(timing)
            return phase_names
        elif self.tl_type=='saturated_control':
            timing = cur_sec - self.coordinate_start_time
            print(14565432,timing, cur_sec, self.coordinate_start_time)
            phase_names = self.get_phase_saturated_control(timing)
            return phase_names
        else:#isolated_control
            return self.phase_names

    def set_phase(self,phase_name):

        self.phase_name = phase_name
        phase = self.phases.get(self.phase_name)

        actuate_phase = self.get_actuated_phase(phase)
        traci.trafficlight.setRedYellowGreenState(self.id, actuate_phase)

    def get_actuated_phase(self, cur_phase):
        if not self.prev_phase or cur_phase == self.prev_phase:
            return cur_phase

        yellow_phase = list(cur_phase)
        for i, (p0, p1) in enumerate(zip(self.prev_phase, cur_phase)):
            if p0 in 'Gg' and p1 == 'r':
                yellow_phase[i] = 'y'  # green -> red: need yellow
            elif p0 == 'r' and p1 in 'Gg':
                yellow_phase[i] = 'r'  # red -> green: delay for one frame

        self.prev_phase = cur_phase
        return ''.join(yellow_phase)
    '''
    def transfer_to_coord_start(self, range_list):
        if isinstance(range_list, int):  # ✅ 单个数字（如 42）
            return self.coordinate_start_time + range_list
        elif isinstance(range_list, list):
            if all(isinstance(x, list) and len(x) == 2 for x in range_list):
                #  区间列表：[[36, 49]]、[[0, 27], [60, 66]]、[[1218, 1221], ...]
                return [[self.coordinate_start_time + a, self.coordinate_start_time + b] for a, b in range_list]
            else:
                # 普通数值列表：[2, 4, 6]（虽然你目前没提到，但也支持）
                return [self.coordinate_start_time + a for a in range_list]
        else:
            raise ValueError("range_list must be an int, a list of ints, or a list of [a, b] pairs.")
    '''
    def get_phase_saturated_control(self,timing):

        in_g_range,if_coordinated_start,g_start = range_check_saturated(timing, self.g_range,self.step_length)

        if in_g_range:
            phase_names = ['s_bars', 's_l']
            return phase_names
        else:
            if if_coordinated_start:
                phase_names = ['l_barl', 'bars_barl','branch_s_bars', 'branch_l_barl', 'branch_s_l', 'branch_bars_barl']
            else:
                phase_names = ['s_bars','s_l','l_barl', 'bars_barl','branch_s_bars', 'branch_l_barl', 'branch_s_l', 'branch_bars_barl']

            return phase_names


    def get_phase_green_band(self,timing):
        in_g_range,g_start = range_check(timing,self.g_range, self.step_length)
        in_g_bar_range,g_bar_start = range_check(timing,self.g_bar_range, self.step_length)
        check_band_coverage(self.g_range, self.g_bar_range, total_cycle_length=self.C, max_coverage_ratio=0.8)

        if in_g_range or in_g_bar_range:
            phase_names=[]
            if timing==g_start:
                if in_g_bar_range:
                    phase_names=['s_bars']
                else:
                    phase_names=['s_l']
            if timing==g_bar_start:
                if in_g_range:
                    phase_names=['s_bars']
                else:
                    phase_names=['bars_barl']
            if timing == self.g_range[0][1] + 1 or (len(self.g_range)>1 and timing == self.g_range[1][1]+1):
                phase_names = ['bars_barl']
            if timing == self.g_bar_range[0][1] + 1 or (len(self.g_bar_range)>1 and timing == self.g_bar_range[1][1]+1):
                phase_names=['s_l']

            if len(phase_names)>0:
                self.last_phase_names = phase_names
            else:
                phase_names = self.last_phase_names
            return phase_names

        else:
            phase_names = ['l_barl','branch_s_bars', 'branch_l_barl', 'branch_s_l', 'branch_bars_barl']
            return phase_names
    '''
    def isolated_control(self):
        phase = self.phases.get(self.phase_name)
        return phase
    
    
    def get_phase_list(self,phase_names=None):
        phases_list = []
        for name, phase in self.phases.items():
            if name in phase_names:
                phases_list.append(phase)
        return phases_list
    '''
    def max_weight_phase(self, phase_names=None):
        max_weight = -100000
        max_weight_phase_name = 0

        #expect green phase
        for name in phase_names:
            phase = self.phases[name]
            weight = self._calculate_weight(phase)

            if weight>=max_weight:
                max_weight_phase_name = name
                max_weight=weight
        return max_weight_phase_name

    def _calculate_weight(self, phase):
        # self.saturated_flow=1 #饱和流率，由于所有相位设置同一流率，取1
        controlledlinks = traci.trafficlight.getControlledLinks(self.id)

        weight = {}
        for i in range(len(phase)):
            if phase[i] != 'G' and phase[i] != 'g':
                continue
            if controlledlinks[i] == []:
                continue

            inputlane = controlledlinks[i][0][0]
            outputlane = controlledlinks[i][0][1]
            upstream_queue = traci.lane.getLastStepHaltingNumber(inputlane)
            outputedge = traci.lane.getEdgeID(outputlane)
            downstream_queue = int(
                traci.edge.getLastStepHaltingNumber(outputedge) / traci.edge.getLaneNumber(outputedge))

            if inputlane not in weight:
                weight[inputlane] = []
            weight[inputlane].append((upstream_queue - downstream_queue))

        sumweight = 0
        for inputlane in weight:
            weight[inputlane] = np.mean(weight[inputlane])
            sumweight += weight[inputlane]

        return sumweight

    def init_connections_by_movement(self):
        self.connections_by_movement={'l':[],
        's':[],
        'r':[],
        'bar_l':[],
        'bar_s':[],
        'bar_r':[],
        'branch_l':[],
        'branch_s':[],
        'branch_r':[],
        'branch_bar_l':[],
        'branch_bar_s':[],
        'branch_bar_r':[]}

    def update_lanes(self):
        self.lanes_by_movement={}
        self.outlanes_by_movement={}
        for n in self.connections_by_movement:
            if n in self.right_movements:
                continue
            lanes=[]
            outlanes=[]
            for connection in self.connections_by_movement[n]:
                lanes.append(connection.lane)
                outlanes.append(connection.outlane)
            self.lanes_by_movement[n] = lanes
            self.outlanes_by_movement[n] = outlanes

    def get_structured_edges(self, all_nodes, all_edges):
        straight_conns = [conn for conn in self.connections.values() if conn.dir == 's']

        e_to_w_entry = None
        w_to_e_entry = None
        e_to_w_exit = None
        w_to_e_exit = None

        for conn in straight_conns:
            edge_from = conn.cfrom
            edge_to = conn.cto

            node_from = all_edges[edge_from].efrom
            node_to = all_edges[edge_to].eto

            from_pos = all_nodes[node_from].position
            to_pos = all_nodes[node_to].position

            dx = to_pos[0] - from_pos[0]
            dy = to_pos[1] - from_pos[1]

            if abs(dx) > abs(dy):  # 是东西方向
                if dx < 0:
                    # 东→西方向
                    e_to_w_entry = edge_from
                    e_to_w_exit = edge_to
                else:
                    # 西→东方向
                    w_to_e_entry = edge_from
                    w_to_e_exit = edge_to

        if None in [e_to_w_entry, w_to_e_entry, e_to_w_exit, w_to_e_exit]:
            raise ValueError(f"Node {self.id} does not have complete east-west structured connections.")

        return [
            w_to_e_entry,  # coordinated_edge
            e_to_w_entry,  # coordinated_edge_opposite
            w_to_e_exit,  # output_edge
            e_to_w_exit  # output_edge_opposite
        ]

    def update_movement_info(self, edges=None, default=False, all_nodes = None, all_edges=None):
        if default:
            edges = self.get_structured_edges(all_nodes,all_edges)

        if edges is None or len(edges) < 4:
            raise ValueError("edges must be provided with at least 4 elements.")

        coordinated_edge, coordinated_edge_opposite, output_edge, output_edge_opposite = edges[:4]

        self.init_connections_by_movement()
        for cid in self.connections:
            connection = self.connections[cid]
            if connection.tl != self.id:
                continue

            if connection.cfrom == coordinated_edge:
                if connection.dir == 'l':
                    self.connections_by_movement['l'].append(connection)
                elif connection.dir == 's':
                    self.connections_by_movement['s'].append(connection)
                else:
                    self.connections_by_movement['r'].append(connection)

            if connection.cfrom == coordinated_edge_opposite:
                if connection.dir == 'l':
                    self.connections_by_movement['bar_l'].append(connection)
                elif connection.dir == 's':
                    self.connections_by_movement['bar_s'].append(connection)
                else:
                    self.connections_by_movement['bar_r'].append(connection)

            if connection.cto == output_edge and connection.dir == 'l':
                self.connections_by_movement['branch_l'].append(connection)
                for cid1 in self.connections:
                    connection1 = self.connections[cid1]
                    if connection1.cfrom==connection.cfrom:
                        if connection1.dir=='s':
                            self.connections_by_movement['branch_s'].append(connection1)
                        elif connection1.dir=='r':
                            self.connections_by_movement['branch_r'].append(connection1)

            if connection.cto == output_edge_opposite and connection.dir == 'l':
                self.connections_by_movement['branch_bar_l'].append(connection)
                for cid1 in self.connections:
                    connection1 = self.connections[cid1]
                    if connection1.cfrom==connection.cfrom:
                        if connection1.dir=='s':
                            self.connections_by_movement['branch_bar_s'].append(connection1)
                        elif connection1.dir=='r':
                            self.connections_by_movement['branch_bar_r'].append(connection1)

        self.update_related_nodes()
        self.update_lanes()
        self.update_tl_index()
        self.update_phases()
        self.update_arrival_rate()

    def update_related_nodes(self):
        move_to_nodes = {'s':'upstream_node','bar_s':'downstream_node',
                         'branch_s':'branch_node','branch_bar_s':'branch_bar_node'}

        self.related_nodes = {'upstream_node':None,'downstream_node':None,'branch_node':None,
                              'branch_bar_node':None}

        for move,node in move_to_nodes.items():
            connection = self.connections_by_movement[move]
            edge = connection[0].cfrom
            from_node = edge.split('_')[0]
            if from_node in self.tl_node_list:
                self.related_nodes[node] = from_node

    def update_tl_index(self):
        self.tl_index={}
        for n in self.connections_by_movement:
            tl_index=[]
            for connection in self.connections_by_movement[n]:
                tl_index.append(connection.linkindex)
            self.tl_index[n]=tl_index

    def update_phases(self):
        correlation = {'s_bars': ['s', 'bar_s'],
                       'l_barl': ['l', 'bar_l'],
                       's_l': ['s', 'l'],
                       'bars_barl': ['bar_s', 'bar_l'],
                       'branch_s_bars': ['branch_s', 'branch_bar_s',],
                       'branch_l_barl': ['branch_l', 'branch_bar_l'],
                       'branch_s_l':['branch_s','branch_l'],
                       'branch_bars_barl':['branch_bar_s','branch_bar_l']
                       }

        all_red = ['r'] * self.index_number
        self.phases = {}

        for phase_key,lane_keys in correlation.items():
            phase_list= all_red.copy()
            for lane_key in lane_keys:
                indices=self.tl_index[lane_key]
                for index in indices:
                    phase_list[index] = 'G'

            for lane_key in self.right_movements:
                indices = self.tl_index[lane_key]
                for index in indices:
                    phase_list[index] = 'g'

            self.phases[phase_key]=''.join(phase_list)
            self.phase_names = list(self.phases.keys())

class Network():
    def __init__(self, step_length=1,sumocfg_file=None,net_file=None,seed=None):
        self.sumocfg_file = sumocfg_file
        self.net_file=net_file
        self.seed=seed
        self.step_length=step_length


    def measure_for_coordination(self):

        for node_id in self.tl_nodes:
            node = self.tl_nodes[node_id]
            node.record_arrival_rate()


    def update_state_and_reward(self,enable_high_level= False):

        for node_id in self.tl_nodes:

            node = self.tl_nodes[node_id]
            #node.record_arrival_rate()
            node.update_state()
            node.update_reward()
            if enable_high_level:
                node.update_high_level_state_and_reward()

        move_num = get_movement_num()
        for node_id,node in self.tl_nodes.items():
            related_node_state = []
            for _, related_node_id in node.related_nodes.items():
                if related_node_id in self.tl_nodes:
                    related_node = self.tl_nodes[related_node_id]
                    related_node_state += related_node.state_arrival
                else:
                    related_node_state += [0] * move_num

            node.state_related_node = related_node_state

    def _init_network(self,sim=True):
        if sim:
            self._init_sim()

        self._init_trafficlights()
        self._init_connections()
        self._init_nodes()

        self._init_edges()


    def _init_trafficlights(self):
        tree = ET.parse(self.net_file)
        root = tree.getroot()
        self.trafficlights = {}
        for child in root:
            if child.tag != 'tlLogic':
                continue
            data = child.attrib
            id=data['id']
            program=data['programID']
            phase=[]
            duration=[]
            for i in child:
                if i.tag != 'phase':
                    continue
                #筛选
                state=i.attrib['state']
                temp=0
                for p in state:
                    if p=='G' or p=='g':
                        temp=1
                if temp==1:
                    phase.append(i.attrib['state'])
                    duration.append(i.attrib['duration'])
            self.trafficlights[id] = trafficlight(id, program=program, phase=phase, duration=duration)


    def _init_sim(self):
        # 访问sumo
        # sumoBinary = 'sumo-gui'
        sumoBinary = 'sumo'
        sumoCmd = [sumoBinary, "-c", self.sumocfg_file]
        # 如果有种子，则传递种子到 SUMO，否则使用默认的随机性
        if self.seed is not None:
            sumoCmd += ["--seed", str(self.seed)]
            print(f"SUMO started with seed: {self.seed}")
        else:
            print("SUMO started with default random seed")

        traci.start(sumoCmd)

    def _init_nodes(self):
        tree = ET.parse(self.net_file)
        root = tree.getroot()
        self.nodes = {}
        self.tl_nodes = {}

        for child in root:
            if child.tag != 'junction':
                continue


            data = child.attrib

            if data['type'] == "internal":
                continue

            id=data['id']
            position = [float(data['x']),float(data['y'])]
            index_number=0
            for i in child:
                if i.tag=='request':
                    index_number+=1

            if data['type']=="dead_end":
                self.nodes[id] = Node(id, step_length=self.step_length, position=position, index_number=index_number,
                                          initial_phases_list=None)
            else:
                initial_phases_list = self.trafficlights[id].phase
                connections = {cid: conn for cid, conn in self.connections.items() if conn.tl == id}
                self.nodes[id] = Node(id,step_length=self.step_length, position=position, index_number=index_number,
                                      initial_phases_list=initial_phases_list, connections=connections)
                self.tl_nodes[id] = self.nodes[id]

        tl_node_list = self.tl_nodes.keys()
        for node in self.tl_nodes.values():
            node.tl_node_list = tl_node_list


    '''
    def get_neighbor_nodes(self, name):
        tree = ET.parse(self.net_file)
        root = tree.getroot()
        neighbor_nodes = []

        # 解析相邻junction的信息
        for edge in root.findall('edge'):
            if (edge.get('from') == name and edge.get('to') in self.tl_nodes):
                neighbor_nodes.append(edge.get('to'))
            if (edge.get('to') == name and edge.get('from') in self.tl_nodes):
                neighbor_nodes.append(edge.get('from'))

        return neighbor_nodes
    '''


    def _init_connections(self):
        tree = ET.parse(self.net_file)
        root = tree.getroot()
        self.connections = {}
        index = 0
        for child in root:
            if child.tag != 'connection':
                continue
            data = child.attrib
            if data['from'][0]==':':
                continue

            name = data['from'] + '_' + data['fromLane'] + '*' + data['to'] + '_' + data['toLane']
            id = index
            cfrom = data['from']
            cto = data['to']
            fromlane = data['fromLane']
            tolane = data['toLane']

            lane = cfrom + '_' + fromlane

            outlane = cto + '_' + tolane
            dir = data['dir']

            if 'tl' in data:
                type = 'tl'
                tl = data['tl']
                linkindex = int(data['linkIndex'])
                self.connections[id] = Connection(id, name=name, lane=lane, outlane = outlane, cfrom=cfrom, cto=cto, fromlane=fromlane, tolane=tolane,
                                                  dir=dir, type=type, tl=tl, linkindex=linkindex)
                index += 1
            else:
                type = 'notl'
                self.connections[id] = Connection(id, name=name, lane=lane, outlane = outlane,cfrom=cfrom, cto=cto, fromlane=fromlane, tolane=tolane,
                                                  dir=dir, type=type)
                index += 1

        for i in self.connections:
            num = 0
            for j in self.connections:
                if self.connections[i].cfrom == self.connections[j].cfrom and self.connections[i].fromlane == \
                        self.connections[j].fromlane:
                    num += 1
            self.connections[i].lanenum = float(1. / num)

    def _init_edges(self):
        tree = ET.parse(self.net_file)
        root = tree.getroot()
        self.edges = {}
        self.edges_id = []
        self.edges_IIE_id = []
        for child in root:
            if child.tag != 'edge':
                continue
            data = child.attrib
            id = data['id']
            self.edges_IIE_id.append(id)
            if not (('function' in child.attrib) and (child.attrib['function'] == "internal")):
                self.edges_id.append(id)
                efrom = data['from']
                eto = data['to']

                #length = []
                speed = []
                for i in child:
                    if i.tag == 'lane':
                        #length.append(float(i.attrib['length']))
                        speed.append(float(i.attrib['speed']))
                #length = np.mean(np.array(length))
                numlane = len(speed)
                speed = np.mean(np.array(speed))

                frompos = self.nodes[efrom].position
                topos = self.nodes[eto].position

                length=((frompos[0]-topos[0])**2+(frompos[1]-topos[1])**2)**0.5
                '''
                if eto not in self.nodes[efrom].neighbor:
                    self.nodes[efrom].neighbor.append(eto)
                if efrom not in self.nodes[eto].neighbor:
                    self.nodes[eto].neighbor.append(efrom)
                '''
                self.edges[id] = Edge(id, efrom=efrom, eto=eto, frompos=frompos, topos=topos, length=length,
                                      numlane=numlane, max_speed=speed)

        for i in self.edges:
            edge = self.edges[i]
            for j in self.edges:
                edge_r = self.edges[j]
                if edge_r.eto == edge.efrom and edge_r.efrom == edge.eto:
                    edge.reverse = j

    def _terminate(self):
        traci.close()

