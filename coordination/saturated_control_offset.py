import numpy as np
def saturated_control_offset(q_branch, l, z, qs, q_out, g, f, t_set, lane_num):
    """
    Generates a set of phi values based on lists of parameters.

    :param list_q_branch: List of q_branch values
    :param list_l: List of l(t) values
    :param z: Constant z
    :param list_qs: List of q^s values
    :param list_q_out: List of q_out(t) values
    :param list_g: List of g(t) values
    :param list_f: List of f values
    :param list_g_minus_1: List of g_{i-1}(t) values
    :param list_t: List of t_i values
    :return: Set of calculated phi values
    """
    phi_set = [[] for _ in range(len(l[0]))]

    for t in range(len(l[0])):
        for i in range(len(q_branch)):

            # Extract values for each i, t combination
            q_branch_i = q_branch[i] / lane_num[i]
            l_i_t = l[i][t]
            qs_i = qs[i]
            q_out_i_t = q_out[i][t] / lane_num[i]
            g_i_t = g[i][t]
            f_i = f[i]
            g_i_minus_1_t = g[i-1][t]
            if t != len(l[i])-1:
                g_i_t_plus_1 = g[i][t+1]
            else:
                g_i_t_plus_1 = g[i][-1]
            t_i = t_set[i-1]

            # Calculate phi_i_t and add to the set
            phi = calculate_phi_i_t(q_branch_i, l_i_t, z, qs_i, q_out_i_t, g_i_t, f_i, g_i_minus_1_t, g_i_t_plus_1, t_i)

            if t!=0:
                if i==1:
                    min_phi = phi_set[0][0] + (g[i][0]-g[i][t])/2
                    max_phi = phi_set[0][0] + (g[i][t]+g[i][0])/2
                    #max_phi = phi_set[0][0]-(g[i][t]+g[i][0])/2+1
                    #min_phi = phi_set[0][0]+(g[i][t]-g[i][0])/2
                else:
                    min_phi = sum(phi_set[0][0:i]) - sum(phi_set[t][0:i-1]) + (g[i][0] - g[i][t]) / 2
                    max_phi = sum(phi_set[0][0:i]) - sum(phi_set[t][0:i-1]) + (g[i][t] + g[i][0]) / 2
                    #max_phi = sum(phi_set[0][0:i - 1]) - sum(phi_set[t][0:i - 2]) - (g[i][t] + g[i][0]) / 2 + 1
                    #min_phi = sum(phi_set[0][0:i - 1]) - sum(phi_set[t][0:i - 2]) + (g[i][t] - g[i][0]) / 2
                phi = max(min_phi, min(phi, max_phi))

            phi_set[t].append(phi)


    phi_set_transfer = []
    # 首先确保phi_set_transfer有足够的子列表
    for i in range(len(phi_set[0])):
        phi_set_transfer.append([])

    for t in range(len(phi_set)):
        for i in range(len(phi_set[t])):
            phi_set_transfer[i].append(phi_set[t][i])

    return phi_set_transfer

def calculate_phi_i_t(q_branch_i, l_i_t, z, qs_i, q_out_i_t, g_i_t, f_i, g_i_minus_1_t, g_i_t_plus_1, t_i):
    """
    Calculates the value of phi_{i, t} based on the provided formulas and scenarios.

    :param q_branch_i: Value of q_branch for the i-th term
    :param l_i_t: Value of l_i at time t
    :param z: Constant z
    :param qs_i: Value of q^s_i
    :param q_out_i_t: Value of q_out_i at time t
    :param g_i_t: Value of g_i at time t
    :param f_i: Value of f_i
    :param g_i_minus_1_t: Value of g_{i-1}(t) at time t
    :param t_i: Constant t_i
    :return: The calculated value of phi_{i, t} or its bounds
    """
    # Calculating t_{c,i}(t), t_{s,i}(t), and t_{u,i}(t)
    t_ci_t = (q_branch_i + l_i_t * z) / qs_i

    if f_i!=1:
        t_si_t = (q_out_i_t - g_i_t * f_i * qs_i) / (qs_i * (1 - f_i))
        t_ui_t = (g_i_t * qs_i - q_out_i_t) / (qs_i * (1 - f_i))
    else:
        t_si_t = q_out_i_t / qs_i
        t_ui_t = (g_i_t * qs_i - q_out_i_t) / qs_i


    # Applying scenarios to calculate phi_i_t
    if t_si_t >= t_ci_t and t_ui_t != 0:
        # Scenario 1.1
        phi = t_i * z - g_i_t / 2 + g_i_minus_1_t / 2

    elif t_si_t < t_ci_t and t_ui_t != 0:
        # Scenario 1.2
        phi = t_i * z - g_i_t / 2 + g_i_minus_1_t / 2
    elif t_si_t >= t_ci_t and t_ui_t == 0:
        # Scenario 2.1
        phi = ((1 / f_i) - 1) * t_si_t - t_ci_t / f_i + g_i_t / 2 - g_i_minus_1_t / 2 + t_i * z
    else: #t_si_t < t_ci_t and t_ui_t == 0:
        # Scenario 2.2
        phi= g_i_t/2-g_i_minus_1_t/2+g_i_t_plus_1 + t_i * z-t_ci_t - q_branch_i/qs_i-1

    return phi
'''
# Applying scenarios to calculate phi_i_t
    if t_si_t >= t_ci_t and t_ui_t != 0:
        # Scenario 1.1
        phi = t_i * z - g_i_t / 2 + g_i_minus_1_t / 2

    elif t_si_t < t_ci_t and t_ui_t != 0:
        # Scenario 1.2        
        lower_bound = t_i * z - g_i_t / 2 + g_i_minus_1_t / 2
        upper_bound = t_i * z - t_ci_t + g_i_t / 2 - g_i_minus_1_t / 2   
        if lower_bound >= min_phi:
            if lower_bound <= max_phi:
                phi = lower_bound
            else:
                phi = max_phi
        else: #lower_bound < min_phi:
            if upper_bound >= min_phi:
                phi = min_phi
            else:
                phi = min_phi
        
    elif t_si_t >= t_ci_t and t_ui_t == 0:
        # Scenario 2.1
        phi = ((1 / f_i) - 1) * t_si_t - t_ci_t / f_i + g_i_t / 2 - g_i_minus_1_t / 2 + t_i * z
    else: #t_si_t < t_ci_t and t_ui_t == 0:
        # Scenario 2.2
        upper_bound=g_i_t / 2 - g_i_minus_1_t / 2 + t_i * z - t_ci_t
        phi=(np.NaN,upper_bound)

    phi=max(min_phi,min(phi,max_phi))

    return phi
'''
