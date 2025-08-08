from pulp import LpMaximize, LpProblem, LpVariable, lpSum

def green_band_model(n, g_min_l, g_min_bar_l, g_min_branch, t, t_bar, C_max, C_min):

    solution = multiband_model(n, g_min_l, g_min_bar_l, g_min_branch, t, t_bar, C_max, C_min)
    g, g_bar, w, w_bar, b, b_bar, triangle, z = read_solution(n, solution)

    return g, g_bar, w, w_bar, b, b_bar, triangle, z

def multiband_model(n, g_min_l, g_min_bar_l, g_min_branch, t, t_bar, C_max, C_min):
    # Create the problem
    model = LpProblem(name="maximize-sum-of-b-and-b-bar", sense=LpMaximize)

    # Define the decision variables
    b = {i: LpVariable(name=f"b_{i}", lowBound=0) for i in range(n-1)}
    b_bar = {i: LpVariable(name=f"b_bar_{i}", lowBound=0) for i in range(n-1)}
    w = {i: LpVariable(name=f"w_{i}", lowBound=0) for i in range(n)}
    w_bar = {i: LpVariable(name=f"w_bar_{i}", lowBound=0) for i in range(n)}
    g = {i: LpVariable(name=f"g_{i}", lowBound=0) for i in range(n)}
    g_bar = {i: LpVariable(name=f"g_bar_{i}", lowBound=0) for i in range(n)}
    triangle = {i: LpVariable(name=f"triangle_{i}", lowBound=0) for i in range(n)}
    z = LpVariable(name="z", lowBound=1/C_max, upBound=1/C_min)
    m = {i: LpVariable(name=f"m_{i}", cat='Integer') for i in range(n-1)}

    # Add objective function
    model += lpSum(b[i] + b_bar[i] for i in range(n-1))

    # Constraint 1
    for i in range(n-1):
        model += w[i] >= b[i] *0.5
        model += w_bar[i] >= b_bar[i] *0.5
        model += w[i+1] >= b[i] *0.5
        model += w_bar[i+1] >= b_bar[i] *0.5

    # Constraint 2
    for i in range(n-1):
        model += w[i] + b[i] *0.5<= g[i]
        model += w[i+1] + b[i] *0.5<= g[i+1]
        model += w_bar[i] + b_bar[i] *0.5<= g_bar[i]
        model += w_bar[i+1] + b_bar[i] *0.5 <= g_bar[i+1]

    # Constraint 3 4
    for i in range(n-1):
        model += (
            -g[i]*0.5+g[i+1]*0.5-g_bar[i]*0.5+g_bar[i+1]*0.5+
            w[i] - w[i+1] +w_bar[i] - w_bar[i+1]+
            t[i] * z + t_bar[i] * z+
            triangle[i] - triangle[i+1] +
            m[i] == 0)

    # Constraint 5
    for i in range(n):
        model += g[i]*0.5+ g_bar[i]*0.5 + triangle[i] <= 1 - g_min_branch[i]
        model += g[i] <= 1 - g_min_bar_l[i] - g_min_branch[i]
        model += g_bar[i] <= 1 - g_min_l[i] - g_min_branch[i]

    # Constraint 6 and 7 are implicitly handled by variable bounds

    # Solve the problem
    model.solve()

    solution={v.name: v.varValue for v in model.variables()}
    return solution

def read_solution(n,solution):
    g=[]
    g_bar=[]
    w = []
    w_bar = []
    triangle=[]

    for i in range(n):
        g.append(solution['g_'+str(i)])
        g_bar.append(solution['g_bar_'+str(i)])
        w.append(solution['w_' + str(i)])
        w_bar.append(solution['w_bar_' + str(i)])
        triangle.append(solution['triangle_'+str(i)])

    b=[]
    b_bar=[]
    for i in range(n-1):
        b.append(solution['b_'+str(i)])
        b_bar.append(solution['b_bar_'+str(i)])

    z=solution['z']
    return g,g_bar,w,w_bar,b,b_bar,triangle,z