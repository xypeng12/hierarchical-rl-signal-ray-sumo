import pulp
import numpy as np

def saturated_control_cycle(n,L,l,f,qs,g_max,g_min,q_branch_max,q_branch_min,Cmax,Cmin,q_in_1,h,M,lane_num):
    _, _, _,_, z_value = saturated_control_model_cycle(L, l, f, qs, g_max, g_min, q_branch_max,q_branch_min,Cmax, Cmin, q_in_1, h, M, n,lane_num)
    return z_value

def saturated_control_model_cycle(L, l, f, qs, g_max, g_min, q_branch_max,q_branch_min,Cmax, Cmin, q_in_1, h, M, n, lane_num):
    prob = pulp.LpProblem("OptimizationProblem", pulp.LpMaximize)

    q_out = [pulp.LpVariable(f'q_out_{i}', 0) for i in range(n)]
    q_branch=[pulp.LpVariable(f'q_branch{i}', float(q_branch_min[i]),float(q_branch_max[i])) for i in range(n)]
    g = [pulp.LpVariable(f'g_{i}', float(g_min[i]), float(g_max[i])) for i in range(n)]
    x = [pulp.LpVariable(f'x_{i}', cat='Binary') for i in range(n)]
    z = pulp.LpVariable('z', 1 / Cmax, 1 / Cmin)
    q_in = [q_in_1] + [0] * (n - 1)  

    for i in range(n):
        assert g_min[i] <= g_max[i], f"g_min[{i}] > g_max[{i}]: {g_min[i]} > {g_max[i]}"
        assert q_branch_min[i] <= q_branch_max[
            i], f"q_branch_min[{i}] > q_branch_max[{i}]: {q_branch_min[i]} > {q_branch_max[i]}"
    assert 1 / Cmax <= 1 / Cmin or 1 / Cmin <= 1 / Cmax, "Invalid z bounds: check Cmax and Cmin"

    assert all(np.isfinite(q_branch_min)), "Invalid q_branch_min values"
    assert all(np.isfinite(q_branch_max)), "Invalid q_branch_max values"
    assert all(np.isfinite(g_min)), "Invalid g_min values"
    assert all(np.isfinite(g_max)), "Invalid g_max values"

    prob += pulp.lpSum(q_out)

    for i in range(n):
        #constraint 1
        prob += q_out[i] <= g[i] * qs[i] * lane_num[i]
        prob += q_out[i] <= l[i] * z * lane_num[i] + q_in[i] + q_branch[i]
        prob += q_out[i] >= g[i] * qs[i] * lane_num[i] - M * (1 - x[i])
        prob += q_out[i] >= l[i] * z * lane_num[i] + q_in[i] + q_branch[i] - M * x[i]

        if f[i]!=1:
            prob += L[i] * z / h >= (q_out[i] * 1/lane_num[i] - g[i] * f[i] * qs[i]) / (1 - f[i])
        elif q_out[i] * 1/lane_num[i] == g[i] * qs[i]:
            prob += L[i] * z / h >= q_out[i] * 1/lane_num[i]

        prob += L[i] * z / h >= q_branch[i] * 1/lane_num[i] +l[i]*z  #constraint 5

        if i < n - 1:  
            q_in[i + 1] = q_out[i] * f[i] #constrain 2

    prob.solve()

    check_solution_status(prob)
    q_out_values = [q_out[i].varValue for i in range(n)]
    q_branch_values=[q_branch[i].varValue for i in range(n)]
    g_values = [g[i].varValue for i in range(n)]
    x_values = [x[i].varValue for i in range(n)]
    z_value = z.varValue

    return q_out_values,q_branch_values, g_values, x_values, z_value


def check_solution_status(prob):
    if prob.status == pulp.LpStatusOptimal:
        print("解是最优的.")
    elif prob.status == pulp.LpStatusInfeasible:
        print("问题无可行解.")
    elif prob.status == pulp.LpStatusUnbounded:
        print("问题解无界.")
    elif prob.status == pulp.LpStatusNotSolved:
        print("问题未解决.")
    else:
        print("求解状态未知.")
