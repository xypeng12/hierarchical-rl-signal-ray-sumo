from green_band import green_band
from saturated_control import saturated_control
import traci
import matplotlib.pyplot as plt
from network import network, Coordinated_plan

import numpy as np
class run_coordination(network):
    def __init__(self,coordinated_plans=None,sumocfg_file=None,net_file=None,seed=None,record_coordinated_path=None):

        self.set_coordinated_plans(coordinated_plans)

        #self.coordinated_path=coordinated_path

        self.sumocfg_file = sumocfg_file
        self.net_file = net_file
        self.seed = seed

        self.time_length = 2000
        self.step_length = 1
        self.time_for_measure_arrival = 200  # time_for_measure_arrival_before_green_band

        self.saturated_flow_rate_left = 0.3 #veh/s 1111111111111
        self.saturated_flow_rate_straight = 0.4 #veh/s

        self.speed_reduction_ratio = float(1/0.7)
        self.C_max = 100
        self.C_min = 40
        self.C=0

        self.M = 1000 #large_number
        self.stop_headway = 7.5 #h
        self.q_branch_max_ratio = 1.2
        self.q_branch_min_ratio = 0.6

        self._init_network()

        self.init_record_settings()

        self.run()
        self.plot_trajectories()

    def init_record_settings(self):
        self.veh_data = {}
        self.veh_data_opposite = {}
        self.record_coordinated_edges = {}
        self.record_coordinated_edges_opposite = {}
        self.record_nodepos = {}

        for id, path in self.record_coordinated_paths.items():
            self.veh_data[id] = {}
            self.veh_data_opposite[id] = {}
            self.record_coordinated_edges[id], self.record_coordinated_edges_opposite[id], self.record_nodepos[
                id] = self.get_coordinated_information(path)


    def set_coordinated_plans(self,coordinated_plans_information):
        self.record_coordinated_paths = {}
        self.coordinated_plans = {}

        for id in range(len(coordinated_plans_information)):
            plan_information = coordinated_plans_information[id]
            start_time = plan_information['start_time']
            duration = plan_information['duration']
            type = plan_information['type']
            path = plan_information['path']

            coordianted_plan = Coordinated_plan(id=id, start_time=start_time, duration=duration, type=type, coordinated_path=path)
            self.coordinated_plans[id] = coordianted_plan
            self.record_coordinated_paths[id] = path

    def get_coordinated_information(self,coordinated_path):
        coordinated_edges = []
        coordinated_edges_opposite = []
        for i in range(len(coordinated_path) - 1):
            coordinated_edges.append(coordinated_path[i] + '_' + coordinated_path[i + 1])
            coordinated_edges_opposite.append(coordinated_path[i + 1] + '_' + coordinated_path[i])

        nodepos = {node: 0 for node in coordinated_path}
        for i in range(len(coordinated_edges)):
            edge = coordinated_edges[i]
            edge_length = self.edges[edge].length

            for j in range(i, len(coordinated_edges)):
                node = coordinated_edges[j].split('_')[1]
                nodepos[node] += edge_length
        return coordinated_edges,coordinated_edges_opposite, nodepos

    def record_veh_data(self,step,veh_data,veh_data_opposite,coordinated_edges,coordinated_edges_opposite,nodepos):

        veh_list = traci.vehicle.getIDList()
        for veh in veh_list:
            lane = traci.vehicle.getLaneID(veh)
            edge = lane[:-2]
            if lane[-1] == '2':
                # at left turn or not belong to the coordinated path
                continue

            if edge in set(coordinated_edges):
                edge_startpos = nodepos[edge.split('_')[0]]
                position = edge_startpos + traci.vehicle.getLanePosition(veh)
                if veh in veh_data:
                    veh_data[veh]['lane'].append(lane[-1])
                    veh_data[veh]['edge'].append(edge)
                    veh_data[veh]['position'].append(position)
                    veh_data[veh]['step'].append(step)
                else:
                    veh_data[veh] = {}
                    veh_data[veh]['lane'] = []
                    veh_data[veh]['edge'] = []
                    veh_data[veh]['position'] = []
                    veh_data[veh]['step'] = []

            if edge in set(coordinated_edges_opposite):
                edge_startpos = nodepos[edge.split('_')[0]]
                position = edge_startpos - traci.vehicle.getLanePosition(veh)
                if veh in veh_data_opposite:
                    veh_data_opposite[veh]['lane'].append(lane[-1])
                    veh_data_opposite[veh]['edge'].append(edge)
                    veh_data_opposite[veh]['position'].append(position)
                    veh_data_opposite[veh]['step'].append(step)
                else:
                    veh_data_opposite[veh] = {}
                    veh_data_opposite[veh]['lane'] = []
                    veh_data_opposite[veh]['edge'] = []
                    veh_data_opposite[veh]['position'] = []
                    veh_data_opposite[veh]['step'] = []

    def run(self):
        step = 0

        '''
        total_travel_time = 0
        total_delay = 0
        '''

        while True:
            traci.simulation.step(time=step)

            for id in self.veh_data.keys():
                self.record_veh_data(step, self.veh_data[id], self.veh_data_opposite[id], self.record_coordinated_edges[id], self.record_coordinated_edges_opposite[id],
                                self.record_nodepos[id])

            coordinated_node_set=set()

            for id in self.coordinated_plans:
                coordinated_plan = self.coordinated_plans[id]
                start_time = coordinated_plan.start_time
                end_time = coordinated_plan.end_time
                self.tl_type = coordinated_plan.type

                if step >= start_time and step <= end_time:

                    self.coordinated_path = coordinated_plan.coordinated_path

                    self.coordinated_edges, self.coordinated_edges_opposite, self.nodepos = self.get_coordinated_information(self.coordinated_path)

                    if step == start_time:
                        self.update_coordinated_nodes()

                    if step < start_time + self.time_for_measure_arrival:
                        for node_id in self.coordinated_path:
                            node = self.nodes[node_id]
                            node.record_arrival_rate()

                    if step == start_time + self.time_for_measure_arrival:
                        for node_id in self.coordinated_path:
                            node = self.nodes[node_id]
                            node.average_arrival_rate(self.time_for_measure_arrival)
                        if self.tl_type == 'green_band_control':
                            self.set_greed_band()
                        elif self.tl_type == 'saturated_control':
                            duration = coordinated_plan.duration - self.time_for_measure_arrival
                            self.set_saturated_control(duration)

                    if step >= start_time + self.time_for_measure_arrival:
                        timing = step - start_time - self.time_for_measure_arrival
                        for node_id in self.coordinated_path:
                            coordinated_node_set.update([node_id])
                            node = self.nodes[node_id]
                            node.tl_type = self.tl_type
                            node.set_phase(timing)

            tl_nodes=traci.trafficlight.getIDList()
            for node_id in tl_nodes:
                if node_id not in coordinated_node_set:
                    node = self.nodes[node_id]
                    node.tl_type='isolated_control'
                    node.set_phase(step)

            '''
            travel_time = 0
            delay = 0
            for edge in traci.edge.getIDList():
                travel_time += traci.edge.getLastStepVehicleNumber(edge)
                delay += traci.edge.getLastStepHaltingNumber(edge)
            total_travel_time += travel_time
            total_delay += delay
            '''

            step += self.step_length
            if step > self.time_length:
                break

    def update_node_information(self,node_id,coordinated_edge,coordinated_edge_opposite,output_edge,output_edge_opposite):
        node = self.nodes[node_id]
        node.init_connections()

        for cid in self.connections:
            connection = self.connections[cid]
            if connection.tl != node_id:
                continue

            if connection.cfrom == coordinated_edge:
                if connection.dir == 'l':
                    node.connections['l'].append(connection)
                elif connection.dir == 's':
                    node.connections['s'].append(connection)
                else:
                    node.connections['r'].append(connection)

            if connection.cfrom == coordinated_edge_opposite:
                if connection.dir == 'l':
                    node.connections['bar_l'].append(connection)
                elif connection.dir == 's':
                    node.connections['bar_s'].append(connection)
                else:
                    node.connections['bar_r'].append(connection)

            if connection.cto == output_edge and connection.dir == 'l':
                node.connections['branch_l'].append(connection)
                for cid1 in self.connections:
                    connection1 = self.connections[cid1]
                    if connection1.cfrom==connection.cfrom:
                        if connection1.dir=='s':
                            node.connections['branch_s'].append(connection1)
                        elif connection1.dir=='r':
                            node.connections['branch_r'].append(connection1)

            if connection.cto == output_edge_opposite and connection.dir == 'l':
                node.connections['branch_bar_l'].append(connection)
                for cid1 in self.connections:
                    connection1 = self.connections[cid1]
                    if connection1.cfrom==connection.cfrom:
                        if connection1.dir=='s':
                            node.connections['branch_bar_s'].append(connection1)
                        elif connection1.dir=='r':
                            node.connections['branch_bar_r'].append(connection1)
        node.update_lanes()
        node.update_tl_index()
        node.update_phases()
        node.update_arrival_rate()

    def update_coordinated_nodes(self):
        for i in range(len(self.coordinated_path)):
            if i==0:
                coordinated_edge_opposite = self.coordinated_edges_opposite[i]
                output_edge = self.coordinated_edges[i]

                coordinated_edge=-1
                output_edge_opposite=-1
                for c_id in self.connections:
                    connection = self.connections[c_id]
                    if connection.cto == output_edge and connection.dir=='s':
                        coordinated_edge=connection.cfrom
                    if connection.cfrom== coordinated_edge_opposite and connection.dir=='s':
                        output_edge_opposite=connection.cto

            elif i==len(self.coordinated_path) - 1:
                coordinated_edge=self.coordinated_edges[i-1]
                output_edge_opposite = self.coordinated_edges_opposite[i-1]
                coordinated_edge_opposite = -1
                output_edge=-1
                for c_id in self.connections:
                    connection = self.connections[c_id]
                    if connection.cfrom== coordinated_edge and connection.dir=='s':
                        output_edge=connection.cto
                    if connection.cto == output_edge_opposite and connection.dir=='s':
                        coordinated_edge_opposite=connection.cfrom

            else:
                coordinated_edge = self.coordinated_edges[i - 1]
                coordinated_edge_opposite = self.coordinated_edges_opposite[i]
                output_edge= self.coordinated_edges[i]
                output_edge_opposite=self.coordinated_edges_opposite[i-1]

            node_id = self.coordinated_path[i]

            self.update_node_information(node_id, coordinated_edge, coordinated_edge_opposite, output_edge, output_edge_opposite)
    def set_saturated_control(self,duration):

        n = len(self.coordinated_path)-1
        t = []
        L = []

        qs = [self.saturated_flow_rate_straight] * n # TO D0: also has saturated_left flow


        for i in range(len(self.coordinated_edges)):
            edge = self.coordinated_edges[i]
            length = self.edges[edge].length

            L.append(length)

            speed = self.edges[edge].max_speed * self.speed_reduction_ratio
            t.append(length / speed)

        f = []
        l_initial = []
        q_branch = []

        g_max_list = []
        g_min_list = []

        node = self.nodes[self.coordinated_path[0]]
        q_in_1 = node.arrivals_perlane['s']*1.5 #higher estimation

        qs_1=self.saturated_flow_rate_straight

        signalized_nodes=self.coordinated_path[1:]
        for node_id in signalized_nodes:
            node = self.nodes[node_id]

            f.append(node.arrivals['s']/(node.arrivals['s']+node.arrivals['l'])) #ignore right cause right in the same lane of straight

            queues=[]
            for lane in node.lanes['s']:
                queues.append(traci.lane.getLastStepHaltingNumber(lane))
            l_initial.append(np.mean(queues))

            g_branch_min = max(
                node.arrivals_perlane['branch_s'] / self.saturated_flow_rate_straight + node.arrivals_perlane[
                    'branch_bar_l'] / self.saturated_flow_rate_left,
                node.arrivals_perlane['branch_bar_s'] / self.saturated_flow_rate_straight + node.arrivals_perlane[
                    'branch_l'] / self.saturated_flow_rate_left)

            g_bar_l_min = node.arrivals_perlane['bar_l']/self.saturated_flow_rate_left
            g_bar_l_max = g_bar_l_min * 2

            g_l_min = node.arrivals_perlane['l']/self.saturated_flow_rate_left
            g_bar_s_min = node.arrivals_perlane['bar_s']/self.saturated_flow_rate_straight

            g_max = 1 - g_branch_min - g_bar_l_min
            g_min = min(g_bar_s_min + g_l_min - g_bar_l_max, g_max*0.5)

            g_max = max(g_max, 0.3)
            g_min = max(g_min, 0)

            g_max_list.append(g_max)
            g_min_list.append(g_min)

        for node_id in self.coordinated_path[:-1]:
            node = self.nodes[node_id]
            q_branch.append(node.arrivals['branch_l'])  # ignore branch bar right

        q_branch_max = [q * self.q_branch_max_ratio for q in q_branch]
        q_branch_min = [q * self.q_branch_min_ratio for q in q_branch]

        C, g_range, q_branch = saturated_control(n, L, l_initial, f, qs, g_max_list, g_min_list, q_branch_max, q_branch_min, self.C_max,
                                                 self.C_min, q_in_1, qs_1, self.stop_headway, self.M, duration, t)

        self.C = C

        for i in range(len(self.coordinated_path)):
            node_id=self.coordinated_path[i]
            node = self.nodes[node_id]
            node.C = C
            node.g_range = g_range[i]
            if i !=len(self.coordinated_path)-1:
                node.q_branch = q_branch[i]
            print('node', node_id, 'g_range', [[x[0] + 1200, x[1] + 1200] for x in g_range[i]])

        print('cycle', C)

    def set_greed_band(self):
        n = len(self.coordinated_path)  # Example number of intersections

        t = []
        t_bar = []
        g_min_l=[]
        g_min_bar_l=[]
        g_min_branch=[]

        for i in range(len(self.coordinated_edges)):
            edge = self.coordinated_edges[i]
            length=self.edges[edge].length

            speed = self.edges[edge].max_speed * self.speed_reduction_ratio
            t.append(length / speed)

            edge_opposite = self.coordinated_edges_opposite[i]
            speed = self.edges[edge_opposite].max_speed*self.speed_reduction_ratio
            t_bar.append(length / speed)

        for node_id in self.coordinated_path:
            node=self.nodes[node_id]

            g_min_l.append(node.arrivals_perlane['l']/self.saturated_flow_rate_left)
            g_min_bar_l.append(node.arrivals_perlane['bar_l']/self.saturated_flow_rate_left)

            g_min_branch.append(max(node.arrivals_perlane['branch_s']/self.saturated_flow_rate_straight + node.arrivals_perlane['branch_bar_l']/self.saturated_flow_rate_left,
                                    node.arrivals_perlane['branch_bar_s']/self.saturated_flow_rate_straight + node.arrivals_perlane['branch_l']/self.saturated_flow_rate_left))

        C,g_range,g_bar_range,b_start,b_bar_start = green_band(n, g_min_l, g_min_bar_l,
                                                                                     g_min_branch, t, t_bar, self.C_max,
                                                                                     self.C_min)
        self.C=C


        for i in range(len(self.coordinated_path)):
            node_id = self.coordinated_path[i]
            node = self.nodes[node_id]
            node.C=C
            node.g_range=g_range[i]
            node.g_bar_range=g_bar_range[i]

            node.b_start=b_start[i]
            node.b_bar_start=b_bar_start[i]

            print('node', node_id, 'g_range', [[x[0], x[1]] for x in g_range[i]])

        print('cycle', C)
    def plot_trajectories(self):
        for id, path in self.record_coordinated_paths.items():
            self.plot_trajectories_for_a_coordinated_path(self.veh_data[id], path)

    def plot_trajectories_for_a_coordinated_path(self,veh_data,path):
        # 创建一个图表
        plt.figure(figsize=(10, 6))

        for veh, data in veh_data.items():
            plt.plot(data['step'], data['position'], label=veh)

        # 添加标题和轴标签
        plt.title("trajectories%s" % path)
        plt.xlabel("Step")
        plt.ylabel("Position")
        plt.grid(True)
        plt.xlim(0, 1600)

        # 显示图表
        plt.show()

coordinated_plans=[#{'start_time':1000, 'duration':600, 'type': 'saturated_control', 'path':['n1','n2','n3','n4','n5','n6']}] #,
                  {'start_time':0, 'duration':3600, 'type': 'saturated_control', 'path':['n1','n2','n3','n4','n5','n6']}]

sumocfg_file = '../simulation/low_level_GWC/exp.sumocfg'
net_file= '../simulation/low_level_GWC/exp.net.xml'
run=run_coordination(coordinated_plans=coordinated_plans,sumocfg_file=sumocfg_file,net_file=net_file)