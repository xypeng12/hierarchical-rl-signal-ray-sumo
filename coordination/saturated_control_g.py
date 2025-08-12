import pulp
import re

from pulp import PULP_CBC_CMD

from coordination.saturated_control_cycle import check_solution_status

def saturated_control_green(n, t_T, L, l_initial, f, qs, g_max, g_min, q_branch_max, q_branch_min, q_in_1, h, M, z, lane_num_1, lane_num):
    problem = saturated_control_model_green(n, t_T, L, l_initial, f, qs, g_max, g_min,q_branch_max, q_branch_min, q_in_1, h, M, z, lane_num_1, lane_num)
    solver = PULP_CBC_CMD(msg=True, gapRel = 0.05, timeLimit = 60)  
    problem.solve(solver)
    check_solution_status(problem)

    g, l, q_out, q_branch, _ = read_solution(problem, n, t_T)

    return g, l, q_out, q_branch

def saturated_control_model_green(n, t_T, L, l_initial, f, qs, g_max, g_min,q_branch_max,q_branch_min, q_in_1, h, M, z, lane_num_1, lane_num):
    prob = pulp.LpProblem("Maximize_Outflow", pulp.LpMaximize)

    q_out = pulp.LpVariable.dicts("q_out", ((i, t) for i in range(n) for t in range(t_T)), lowBound=0)
    x = pulp.LpVariable.dicts("x", ((i, t) for i in range(n) for t in range(t_T)), cat='Binary')
    q_branch = pulp.LpVariable.dicts("q_branch", (i for i in range(n)), lowBound=0)
    l = pulp.LpVariable.dicts("l", ((i, t) for i in range(n) for t in range(t_T)), lowBound=0)

    g = {}
    for i in range(n):
        for t in range(t_T):
            g[i, t] = pulp.LpVariable(f"g_{i}_{t}", lowBound=g_min[i], upBound=g_max[i])

    for i in range(n):
        prob += l[i, 0] == l_initial[i], f"initial_l_{i}"

    if l_initial[0] * z + q_in_1 / lane_num_1 + q_branch_min[0]/ lane_num_1 > g_max[0] * qs[0]:
        print('first_intersection_will_increase_queue')
    else:
        print('first_intersection_will_not_increase_queue')

    for i in range(n):
        for t in range(t_T):
            if i == 0:
                q_in_i_t = q_in_1
            else:
                q_in_i_t = q_out[i-1, t] * f[i]

            prob += q_out[i, t] <= g[i, t] * qs[i] * lane_num[i], f"q_out1{i}_{t}"
            prob += q_out[i, t] <= l[i, t] * z * lane_num[i] + q_in_i_t + q_branch[i], f"q_out2{i}_{t}"
            prob += q_out[i, t] >= g[i, t] * qs[i] * lane_num[i]  - M * (1 - x[i, t]), f"q_out3{i}_{t}"
            prob += q_out[i, t] >= l[i, t] * z * lane_num[i] + q_in_i_t + q_branch[i] - M * x[i, t], f"q_out4{i}_{t}"

            if f[i] < 0.99:   # low-level  if f[i] > 0.99:  seems not revised
                prob += L[i] * z / h >= (q_out[i, t] * 1/lane_num[i] - g[i, t] * f[i] * qs[i]) / (1 - f[i]), f"Lz_h_constraint_1_{i}_{t}"
            elif q_out[i, t]* 1/lane_num[i] == g[i, t] * qs[i]:
                prob += L[i] * z / h >= q_out[i, t] * 1/lane_num[i], f"Lz_h_constraint_1_{i}_{t}"

            prob += L[i] * z / h >= q_branch[i] * 1/lane_num[i] + l[i, t] * z, f"Lz_h_constraint_2_{i}_{t}"
            prob += L[i] * z / h >= l[i, t] * z + q_in_i_t * 1/lane_num[i] + q_branch[i] * 1/lane_num[i]- g[i, t] * qs[i], f"Lz_h_constraint_3_{i}_{t}"

            if t < t_T - 1:
                prob += l[i, t + 1] >= l[i, t] + (q_in_i_t * 1/lane_num[i] + q_branch[i] * 1/lane_num[i] - g[i, t] * qs[i]) * (1/z), f"li_update_{i}_{t}"
                prob += l[i, t + 1] >= 0, f"li_non_negative_{i}_{t}"
                prob += l[i, t + 1] <= l[i, t] + (q_in_i_t * 1/lane_num[i] + q_branch[i] * 1/lane_num[i] - g[i, t] * qs[i]) * (1/z) + M * (1 - x[i, t]), f"li_upper_bound_{i}_{t}"
                prob += l[i, t + 1] <= M * x[i, t], f"li_upper_bound_2_{i}_{t}"


        prob += q_branch[i] <= q_branch_max[i], f"q_branch_max_constraint_{i}"
        prob += q_branch[i] >= q_branch_min[i], f"q_branch_min_constraint_{i}"

    prob += pulp.lpSum(q_out[i, t] for i in range(n) for t in range(t_T))

    return prob
def read_solution(problem,n,t_T):
    g_values = [[0 for _ in range(t_T)] for _ in range(n)]
    l_values = [[0 for _ in range(t_T)] for _ in range(n)]
    q_out_values = [[0 for _ in range(t_T)] for _ in range(n)]
    q_branch_values = [0 for _ in range(n)]
    x_values = [[0 for _ in range(t_T)] for _ in range(n)]

    for v in problem.variables():
        name_parts = v.name.split('_')
        var_type = name_parts[0]  
        if var_type=='q':
            var_type=name_parts[0]+'_'+name_parts[1]

        if var_type in ['g', 'l', 'q_out', 'x']:
            indices = re.findall(r'\d+', v.name)
            i = int(indices[0])  
            t = int(indices[1])  

            if var_type == 'g':
                g_values[i][t] = v.varValue
            elif var_type == 'l':
                l_values[i][t] = v.varValue
            elif var_type == 'q_out':
                q_out_values[i][t] = v.varValue
            elif var_type == 'x':
                x_values[i][t] = v.varValue
        elif var_type == 'q_branch':
            i = int(name_parts[2])  
            q_branch_values[i] = v.varValue
    return g_values, l_values, q_out_values, q_branch_values, x_values

if __name__ == '__main__':
    n = 4
    t_T = 4
    L = [400] * n
    l_initial = [2] * n
    f = [0.2] * n
    qs = [0.5] * n
    g_max = [0.5] * n
    g_min = [0] * n
    q_in_1 = 0.1
    h = 5
    M = 10000
    z = 0.01
    g, l, q_out, q_branch, x = saturated_control_green(n, t_T, L, l_initial, f, qs, g_max, g_min, q_in_1, h, M, z)

    print("g_values:", g)
    print("l_values:", l)
    print("q_out_values:", q_out)
    print("q_branch_values:", q_branch)
    print("x_values:", x)

