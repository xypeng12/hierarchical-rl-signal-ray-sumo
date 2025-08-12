import traci
from network import Coordinated_plan
from coordination.green_band import green_band
from coordination.saturated_control import saturated_control
import numpy as np


def coordinated_path_dict():
    coordinated_path_dict = {0: [{'type': 'green_band_control', 'path': ['n1', 'n2', 'n3', 'n4', 'n5', 'n6']}],
                             1: [{'type': 'saturated_control', 'path': ['n1', 'n2', 'n3', 'n4', 'n5', 'n6']}],
                             2: []}
    return coordinated_path_dict

def coordinated_paths_GWC():
    coordinated_paths_GWC = [{'type': 'green_band_control',
                            'path': ['n1', 'n2', 'n3', 'n4', 'n5', 'n6']}]
                            
    return coordinated_paths_GWC

def coordinated_paths_SCC():

    coordinated_paths_SCC = [{'type': 'saturated_control','path': ['n1', 'n2', 'n3','n4', 'n5', 'n6']}]

    return coordinated_paths_SCC


class Coordinator:
    def __init__(self, coordinated_paths=None, nodes=None, tl_nodes=None, edges=None, connections=None, cur_sec=0):
        self.load_coordinated_plans(coordinated_paths,cur_sec=cur_sec)
        self.nodes = nodes
        self.tl_nodes = tl_nodes
        self.edges = edges
        self.connections = connections
        self.cur_sec = cur_sec

        self.saturated_flow_rate_straight = 0.4  # veh/s

        self.speed_reduction_ratio = 1
        self.C_max = 180
        self.C_min = 40
        self.C = 0

        self.M = 1000  # large_number
        self.stop_headway = 7.5  # h
        self.q_branch_max_ratio = 1.2
        self.q_branch_min_ratio = 0.6


        self.step_length = 1

    def load_coordinated_plans(self,coordinated_paths,cur_sec=0,duration = 3600):
        self.coordinated_plans = {}

        for id in range(len(coordinated_paths)):
            plan_information = coordinated_paths[id]
            start_time = cur_sec
            type = plan_information['type']
            path = plan_information['path']
            duration = duration
            coordianted_plan = Coordinated_plan(id=id, start_time=start_time, type=type, coordinated_path=path, duration=duration)
            self.coordinated_plans[id] = coordianted_plan

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
        return coordinated_edges, coordinated_edges_opposite, nodepos


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
                        output_edge = connection.cto
                    if connection.cto == output_edge_opposite and connection.dir=='s':
                        coordinated_edge_opposite=connection.cfrom

            else:
                coordinated_edge = self.coordinated_edges[i - 1]
                coordinated_edge_opposite = self.coordinated_edges_opposite[i]
                output_edge= self.coordinated_edges[i]
                output_edge_opposite=self.coordinated_edges_opposite[i-1]

            node_id = self.coordinated_path[i]
            node = self.tl_nodes[node_id]
            node.update_movement_info(edges=[coordinated_edge, coordinated_edge_opposite, output_edge,
                                   output_edge_opposite])



    def update_coordinated_paths(self,coordinated_paths,cur_sec, left_time = 3600):
        self.cur_sec = cur_sec

        self.load_coordinated_plans(coordinated_paths=coordinated_paths, cur_sec=cur_sec, duration = left_time)

        coordinated_node_set = set()

        for id in self.coordinated_plans:

            coordinated_plan = self.coordinated_plans[id]
            self.tl_type = coordinated_plan.type


            self.coordinated_path = coordinated_plan.coordinated_path


            self.coordinated_edges, self.coordinated_edges_opposite, self.nodepos = self.get_coordinated_information(
                self.coordinated_path)

            self.update_coordinated_nodes()

            for node_id in self.coordinated_path:
                coordinated_node_set.update([node_id])
                node = self.tl_nodes[node_id]
                node.tl_type = self.tl_type
                node.node_location = self.coordinated_path.index(node_id)

        for node_id in self.tl_nodes:
            if node_id not in coordinated_node_set:
                node = self.tl_nodes[node_id]
                node.tl_type = 'isolated_control'
                node.update_movement_info(default=True, all_nodes=self.nodes, all_edges=self.edges)

    def set_coordinated_plans(self,cur_sec):

        for id in self.coordinated_plans:
            coordinated_plan = self.coordinated_plans[id]

            self.coordinated_path = coordinated_plan.coordinated_path
            self.coordinated_edges, self.coordinated_edges_opposite, _ = self.get_coordinated_information(
                self.coordinated_path)

            for node_id in coordinated_plan.coordinated_path:
                node = self.tl_nodes[node_id]
                node.average_arrival_rate()
                node.coordinate_start_time = cur_sec

            print(34234,id)
            print(coordinated_plan.coordinated_path)

            if  coordinated_plan.type in ['green_band_control','saturated_control']:
                if  coordinated_plan.type == 'green_band_control':
                    obs = self.set_green_band()
                else:# coordinated_plan.type == 'saturated_control':
                    duration = coordinated_plan.duration

                    print(65432,id,self.coordinated_edges)
                    obs = self.set_saturated_control(duration)
                for node_id in coordinated_plan.coordinated_path:
                    node = self.tl_nodes[node_id]
                    node.state_coord = obs + [node.node_location]




    
    def set_saturated_control(self,duration):

        n = len(self.coordinated_path)-1
        t = []
        L = []

        qs = [self.saturated_flow_rate_straight] * n


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

        lane_num = []
        flow_keys = ['s', 'l', 'bar_s', 'bar_l', 'branch_s', 'branch_l', 'branch_bar_s', 'branch_bar_l']


        node = self.nodes[self.coordinated_path[0]]

        print(self.coordinated_path,2222,node.id,node.arrivals,node.lanes_by_movement)

        q_in_1 = node.arrivals['s']
        lane_num_1 = len(node.lanes_by_movement['s'])
        qs_1=self.saturated_flow_rate_straight

        total_incoming = sum(node.arrivals_perlane.get(k, 0.0) for k in flow_keys)
        inflow_ratio = node.arrivals_perlane['s'] / total_incoming if total_incoming > 0 else 0.0

        g_0 = max(inflow_ratio, node.arrivals_perlane['s'] / qs_1, 0.2)

        signalized_nodes=self.coordinated_path[1:]
        for node_id in signalized_nodes:
            node = self.tl_nodes[node_id]

            lane_num.append(len(node.lanes_by_movement['s']))
            f.append(node.arrivals['s'] / (node.arrivals['s'] + node.arrivals['l']) if node.arrivals['s'] +
                                                                                           node.arrivals[
                                                                                               'l'] > 0 else 1)

            queues=[]
            for lane in node.lanes_by_movement['s']:
                queues.append(traci.lane.getLastStepHaltingNumber(lane))
            l_initial.append(np.mean(queues))

            total_incoming = sum(node.arrivals_perlane.get(k, 0.0) for k in flow_keys)

            l_ratio = node.arrivals_perlane['l'] / total_incoming if total_incoming > 0 else 0
            g_min_l= min(l_ratio * 0.8, 0.2)

            bar_l_ratio = node.arrivals_perlane['bar_l'] / total_incoming if total_incoming > 0 else 0
            g_min_bar_l = min(bar_l_ratio * 0.8, 0.2)
            g_max_bar_l = g_min_bar_l * 2

            branch_demand = max(node.arrivals_perlane['branch_s'] + node.arrivals_perlane['branch_bar_l'],
                                    node.arrivals_perlane['branch_bar_s'] + node.arrivals_perlane['branch_l'])

            branch_ratio = branch_demand / total_incoming if total_incoming > 0 else 0
            g_min_branch = min(branch_ratio * 0.8, 0.3)


            bar_s_ratio = node.arrivals_perlane['bar_s'] / total_incoming if total_incoming > 0 else 0
            g_min_bar_s = min(bar_s_ratio * 0.8, 0.3)


            g_max = 1 - g_min_branch - g_min_bar_l
            g_min = g_min_bar_s + g_min_l - g_max_bar_l

            g_max = max(g_max, 0.3)
            g_min = max(min(g_min, g_max*0.5), 0)

            g_max_list.append(g_max)
            g_min_list.append(g_min)

        for node_id in self.coordinated_path[:-1]:
            node = self.tl_nodes[node_id]
            q_branch.append(node.arrivals['branch_l'])  # ignore branch bar right

        q_branch_max = [q * self.q_branch_max_ratio for q in q_branch]
        q_branch_min = [q * self.q_branch_min_ratio for q in q_branch]

        C, g_range, q_branch = saturated_control(n, L, l_initial, f, qs, g_max_list, g_min_list, q_branch_max, q_branch_min, self.C_max,
                                                 self.C_min, q_in_1, g_0, qs_1, self.stop_headway, self.M, duration, t,lane_num_1, lane_num)



        self.C = C

        for i in range(len(self.coordinated_path)):
            node_id=self.coordinated_path[i]
            node = self.tl_nodes[node_id]
            node.C = C
            node.g_range = g_range[i]
            if i !=len(self.coordinated_path)-1:
                node.q_branch = q_branch[i]
                print('node', node_id, 'q_branch', node.q_branch)

            print('node', node_id, 'g_range', [[node.coordinate_start_time+a, node.coordinate_start_time+b] for a,b in node.g_range])

        print('cycle', C)

       
        q = []
        for node_id in self.coordinated_path:
            node = self.nodes[node_id]
            q.append(node.arrivals_perlane['s'])

        assert len(t)+1 == len(q)
        obs = t + q + [n] #2n+2
        return obs

    def set_green_band(self):
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
            node=self.tl_nodes[node_id]

            flow_keys = ['s', 'l', 'bar_s', 'bar_l', 'branch_s', 'branch_l', 'branch_bar_s', 'branch_bar_l']
            total_incoming = sum(node.arrivals_perlane.get(k, 0.0) for k in flow_keys)

            l_ratio = node.arrivals_perlane['l'] / total_incoming if total_incoming > 0 else 0
            g_min_l.append(min(l_ratio * 0.8, 0.2))

            bar_l_ratio = node.arrivals_perlane['bar_l'] / total_incoming if total_incoming > 0 else 0
            g_min_bar_l.append(min(bar_l_ratio * 0.8, 0.2))

            branch_demand = max(node.arrivals_perlane['branch_s'] + node.arrivals_perlane['branch_bar_l'],
                                    node.arrivals_perlane['branch_bar_s'] + node.arrivals_perlane['branch_l'])

            branch_ratio = branch_demand / total_incoming if total_incoming > 0 else 0
            g_min_branch.append(min(branch_ratio * 0.8, 0.3))


        C,g_range,g_bar_range,b_start,b_bar_start = green_band(n, g_min_l, g_min_bar_l,
                                                                                     g_min_branch, t, t_bar, self.C_max,
                                                                                     self.C_min)
        self.C=C


        for i in range(len(self.coordinated_path)):
            node_id = self.coordinated_path[i]
            node = self.tl_nodes[node_id]
            node.C=C
            node.g_range = g_range[i]
            node.g_bar_range= g_bar_range[i]
            node.b_start = b_start[i]
            node.b_bar_start = b_bar_start[i]

            print('node', node_id, 'g_range', [[x[0], x[1]] for x in node.g_range])

        print('cycle', C)
        

        obs = t + t_bar + [n]  # 2n+1

        return obs
