from coordination.saturated_control_g import saturated_control_green
from coordination.saturated_control_cycle import saturated_control_cycle
from coordination.saturated_control_offset import saturated_control_offset
from coordination.saturated_control_calculation import saturated_control_calculation


def saturated_control(n,L,l_initial,f,qs,g_max,g_min,q_branch_max,q_branch_min,Cmax,Cmin,q_in_1,g_0,qs_1,h,M,duration,t,lane_num_1,lane_num):
    print(f"\n[Intersection number {n}]")
    print(f"  L = {L}, l_initial = {l_initial}")
    print(f"  f = {f}, qs = {qs}")
    print(f"  g_max = {g_max}, g_min = {g_min}")
    print(f"  q_branch_max = {q_branch_max}, q_branch_min = {q_branch_min}")
    print(f"  Cmax = {Cmax}, Cmin = {Cmin}")
    print(f"  q_in_1 = {q_in_1}, qs_1 = {qs_1}")
    print(f"  lane_num_1 = {lane_num_1}, lane_num = {lane_num}")
    print(f"  h = {h}, M = {M}")
    print(f"  duration = {duration}, t = {t}\n")

    z = saturated_control_cycle(n, L, l_initial, f, qs, g_max, g_min, q_branch_max, q_branch_min, Cmax, Cmin, q_in_1, h, M,lane_num)
    C = int(1/z)

    print("C",C)
    t_T = int(duration//C)
    g, l, q_out, q_branch = saturated_control_green(n, t_T, L, l_initial, f, qs, g_max, g_min, q_branch_max, q_branch_min, q_in_1, h, M,z,lane_num_1,lane_num)

    print(f"  g = {g}")
    print(f"  q_out = {q_out}")
    print(f"  q_branch = {q_branch}\n")

    g_0_list = [[g_0] * len(g[0])]

    g = g_0_list + g

    phi_set = saturated_control_offset(q_branch, l, z, qs, q_out, g, f, t, lane_num)


    g_range_list = saturated_control_calculation(C, phi_set, g)

    return C, g_range_list, q_branch

if __name__ == '__main__':
    n = 3
    L = [400.0] * n
    l_initial = [9.0, 9.0, 9.5]

    f = [0.9934640522875817, 0.9940119760479043, 0.9937106918238995]
    qs = [0.5] * n

    g_max = [0.6394366197183099, 0.6164744645799012, 0.7055028462998102]

    g_min = [0.14225352112676057, 0.15683690280065896, 0.18216318785578747]

    q_branch_max = [0.046, 0.054, 0.026]
    q_branch_min = [0.023, 0.027, 0.013]


    Cmax = 180
    Cmin = 40
    q_in_1 = 0.23916666666666667
    qs_1 = 0.5

    lane_num_1 = 2
    lane_num = [2, 2, 2]

    h = 7.5
    M = 1000

    duration = 3297
    t = [20.0] * n

    C, g_range_list, q_branch = saturated_control(n, L, l_initial, f, qs, g_max, g_min, q_branch_max, q_branch_min,
                                                  Cmax, Cmin, q_in_1,qs_1, h, M, duration, t, lane_num_1, lane_num)
    print("cycle", C)
    print("g_range_list", g_range_list)
    print("q_branch", q_branch)
