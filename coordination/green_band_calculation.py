b_nan=-1000
def calculation(n,g,g_bar,w,w_bar,b,b_bar,t, t_bar,triangle,z):
    phi, phi_bar = get_offset(n, g, g_bar, w, w_bar, t, t_bar, z)
    C = get_cycle_length(z)
    r, r_bar = get_red_time(n,g,g_bar)

    #print('red_time', r, r_bar)
    g_start, g_end, g_bar_start, g_bar_end = get_green_time(n, r, r_bar, w, w_bar, b, b_bar, phi, triangle)
    #print('green1', g_start,g_end, g_bar_start, g_bar_end)
    g_range = transfer_green_time(C, g_start, g_end)
    g_bar_range = transfer_green_time(C, g_bar_start, g_bar_end)
    #print('green2', g_range, g_bar_range)

    b_start,b_bar_start = get_clearance_time(n, r, r_bar, w, w_bar, b, b_bar, phi, triangle)

    b_start = transfer_b(C, b_start)
    b_bar_start = transfer_b(C, b_bar_start)

    from network import check_band_coverage
    for i in range(len(g_range)):
        g=g_range[i]
        g_bar=g_bar_range[i]
        check_band_coverage(g, g_bar, total_cycle_length=C, max_coverage_ratio=0.8)

    return C, g_range, g_bar_range, b_start, b_bar_start


def transfer_green_time(C,g_start,g_end):
    g_norm_range=[]

    for i in range(len(g_start)):
        start=int(g_start[i]*C)
        end=int(g_end[i]*C)
        while start<0 or start>=C:
            #let start between [0,C)
            if start<0:
                start+=C
                end+=C
            else:
                start-=C
                end-=C
        if end>0 and end<=C:
            # end between (0,C]
            g_norm_range.append([[start,end]])
        elif end>C and end<2*C:
            #if end between (C,2C)
            if end-C==start:
                g_norm_range.append([[0,C]])
            else:
                g_norm_range.append([[0,end-C],[start,C]])
        else:
            print('error_g_range',g_start[i],g_end[i],start,end,C)

    return g_norm_range



def transfer_b(C, numbers):
    #transfer_to_0~1
    normalized_numbers = []
    for num in numbers:
        if num==b_nan:
            normalized_numbers.append(num)
            continue
        while num < 0 or num > 1:
            if num < 0:
                num += 1
            else:
                num -= 1
        normalized_numbers.append(num)

    cyclize_numbers=[]
    for num in normalized_numbers:
        cyclize_numbers.append(int(num*C))
    return cyclize_numbers

def get_offset(n,g,g_bar,w,w_bar,t, t_bar,z):
    phi=[]
    phi_bar=[]
    for i in range(n-1):
        phi.append(-g[i]/2 + g[i + 1]/2 + w[i]- w[i + 1] + t[i] * z )
        phi_bar.append(-g_bar[i]/2 + g_bar[i + 1]/2 + w_bar[i]- w_bar[i + 1] + t_bar[i] * z )

    return phi, phi_bar

def get_cycle_length(z):
    C = int(1 / z)
    return C

def get_red_time(n,g,g_bar):
    r = []
    r_bar = []
    for i in range(n):
        r.append(1 - g[i])
        r_bar.append(1 - g_bar[i])
    return r, r_bar

def get_green_time(n,r,r_bar,w,w_bar,b,b_bar,phi,triangle):
    g_start=[]
    g_end=[]
    for i in range(n):
        if i == 0:
            g_start.append(r[i] / 2 + w[i] - b[i]/2)
            g_end.append((r[i] / 2 + w[i] + b[i]/2))
        else:
            sum_phi = sum(phi[:i])  # Sum of phi_j for j=1 to i-1
            if i != n-1:
                temp = max(b[i-1]/2, b[i]/2)
            else:
                temp = b[i-1]/2
            g_start.append(sum_phi+r[i]/2+w[i]-temp)
            g_end.append(sum_phi+r[i]/2+w[i]+temp)

    g_bar_start = []
    g_bar_end = []
    for i in range(n):
        if i==0:
            g_bar_start.append(- triangle[i] - r_bar[i] / 2 - w_bar[i] - b_bar[i] / 2)
            g_bar_end.append(- triangle[i] - r_bar[i] / 2 - w_bar[i] + b_bar[i] / 2)
        else:
            sum_phi = sum(phi[:i])
            if i!=n-1:
                temp=max(b_bar[i-1]/2,b_bar[i]/2)
            else:
                temp=b_bar[i-1]/2

            g_bar_start.append(sum_phi - triangle[i] - r_bar[i] / 2 - w_bar[i]- temp)
            g_bar_end.append(sum_phi - triangle[i] - r_bar[i] / 2 - w_bar[i] + temp)
    return g_start,g_end,g_bar_start,g_bar_end

def get_clearance_time(n, r, r_bar, w, w_bar, b, b_bar, phi, triangle):
    b_start = []
    for i in range(n):
        if i!=0:
            sum_phi = sum(phi[:i])  # Sum of phi_j for j=1 to i-1
            b_start.append(sum_phi+r[i]/2+w[i]-b[i-1]/2)
        else:
            b_start.append(b_nan)

    b_bar_start = []
    for i in range(n):
        if i==0:
            b_bar_start.append(- triangle[i] - r_bar[i] *0.5 - w_bar[i] -b_bar[i]*0.5)
        elif i==n-1:
            b_bar_start.append(b_nan)
        else:
            sum_phi = sum(phi[:i])  # Sum of phi_j for j=1 to i-1
            b_bar_start.append(sum_phi- triangle[i] - r_bar[i] *0.5 - w_bar[i] -b_bar[i]*0.5)

    return b_start,b_bar_start