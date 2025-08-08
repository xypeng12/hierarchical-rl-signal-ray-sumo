from coordination.green_band_model import green_band_model
from coordination.green_band_calculation import calculation

def green_band(n,g_min_l,g_min_bar_l,g_min_branch,t,t_bar,C_max,C_min):
    g,g_bar,w,w_bar,b,b_bar,triangle,z = green_band_model(n, g_min_l, g_min_bar_l, g_min_branch, t, t_bar, C_max, C_min)
    print(22222,g, g_bar, g_min_l, g_min_bar_l, g_min_branch)
    return calculation(n,g,g_bar,w,w_bar,b,b_bar,t, t_bar,triangle,z)

if __name__ == '__main__':
    # Example usage
    n = 5  # Example number of intersections
    g_min_l = [0.15] * n  # Example values
    g_min_bar_l = [0.2] * n  # Example values
    g_min_branch = [0.3] * n  # Example values
    t = [60, 40, 100, 80]  # Example values
    t_bar = [40] * (n - 1)  # Example values
    C_max = 100
    C_min = 50
    C,g_range,g_bar_range,b_start,b_bar_start = green_band(n, g_min_l, g_min_bar_l, g_min_branch,
                                                                                 t, t_bar, C_max, C_min)

    print(C)
    print(g_range)
    print(g_bar_range)
    print(b_start)
    print(b_bar_start)