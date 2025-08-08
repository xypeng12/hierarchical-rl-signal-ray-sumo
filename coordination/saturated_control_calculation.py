def saturated_control_calculation(C, phi_set, g):
    n = len(g)  # Assuming g has a length of n
    g_start = [[] for _ in range(n)]
    g_end = [[] for _ in range(n)]
    for i in range(n):
        for k in range(len(g[i])):  # Looping over k for each i
            if i == 0:  # Corresponds to i = 1 in mathematical notation
                g_start_value = (1 - g[i][k]) / 2 + k
                g_end_value = (1 + g[i][k]) / 2 + k
            else:  # Corresponds to i = 2, 3, ..., n in mathematical notation
                sum_phi = sum(phi_set[j][k] for j in range(i))  # Sum up to i-1

                g_start_value = sum_phi + (1 - g[i][k]) / 2 + k
                g_end_value = sum_phi + (1 + g[i][k]) / 2 + k

            g_start_value = int(g_start_value*C)
            g_end_value = int(g_end_value*C)

            g_start[i].append(g_start_value)
            g_end[i].append(g_end_value)

    g_range_list = []
    for i in range(len(g_start)):
        g_range = []
        for t in range(len(g_start[i])):
            g_range.append([g_start[i][t], g_end[i][t]])

        g_range_list.append(g_range)

    adjusted_g_range_list = adjust_ranges(g_range_list)

    return adjusted_g_range_list

def adjust_ranges(g_range_list):
    # 找出列表中的最小值
    min_value = float('inf')
    for sub_list in g_range_list:
        for range_pair in sub_list:
            min_value = min(min_value, *range_pair)  # 展开 range_pair 并更新最小值

    # 如果最小值小于0，计算其绝对值，否则设置增加量为0
    increment = abs(min_value) if min_value < 0 else 0

    # 如果需要，增加每个元素
    if increment > 0:
        for sub_list in g_range_list:
            for range_pair in sub_list:
                range_pair[0] += increment
                range_pair[1] += increment

    return g_range_list
