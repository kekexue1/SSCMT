# THIS PROBLEM IS DISTANCE MINIMIZATION WIT NAME OF CITIES
# n is the number of clients to visit
# N is the set of clients, with N = {1, 2,...n}
# V set of vertices (or nodes), with V = {0} U N
# A set of arcs, with A = {(i,j) E V**2 : i != j}
# cij is cost of travel over arc(i,j) E A
# Q is the vehicle capacity
# qi is demand of each customer

from gurobipy import *
import numpy as np
import matplotlib.pyplot as plt

rnd = np.random
rnd.seed(0)

n = 5 # number of clients
xc = rnd.rand(n+1) * 200  # n location of clients + 1 for depot
yc = rnd.rand(n+1) * 100  # 200 x 100 rectangle
print(xc)
print(yc)

city_names = ["Depot", "Guadalajara", "Tijuana", "Mexico City", "Cancun", "Merida"] # Create array of cities

for i, txt in enumerate(city_names):
    plt.annotate(txt, (xc[i], yc[i]), textcoords="offset points", xytext=(0, 10), ha='center') # Just add cities names randomly

N = [i for i in range(1, n + 1)] # Set of clients [1,2,3,4,5,6,7,8,9,10]
V = [0] + N # The union of 0 + N [0,1,2,3,4,5,6,7,8,9,10]
A = [(i, j) for i in V for j in V if i != j] # [(0,1), (0,2],...,(10,9]]
# Create dictionary with cost (we could change this for distance)
c = {(i, j): np.hypot(xc[i] - xc[j], yc[i] - yc[j]) for i, j in A} # [(0,1): 33.50, (0,2):41:111,...,(10,9):83.60]
Q = 30 # vehicle capacity
q = {i: rnd.randint(1, 10) for i in N} # just random demand

# Create a new model
mdl = Model('CVRP')

# Create variables
x = mdl.addVars(A, vtype=GRB.BINARY, name="x") # if a vehicle travels in an arc
u = mdl.addVars(N, vtype=GRB.CONTINUOUS, name="u")

# Create additional binary variables to represent flow
# Binary decision variables representing the flow between vertices (arcs). These variables are used to enforce the balance of flow constraint.
flow = mdl.addVars(A, vtype=GRB.BINARY)

# Add the balance of flow constraint (13)
mdl.addConstrs((quicksum(flow[i, j] for j in V if j != i) == x.sum(i, '*')) for i in N)
mdl.addConstrs((quicksum(flow[j, i] for j in V if j != i) == x.sum('*', i)) for i in N)

# Set the objective function
mdl.modelSense = GRB.MINIMIZE
mdl.setObjective(quicksum(x[i, j] * c[i, j] for i, j in A)) # for each arc * the cost(distance), for all the arcs in A

# Add the constraints
mdl.addConstrs(quicksum(x[i, j] for j in V if j != i) == 1 for i in N) # (11) each customer is visited at least once
mdl.addConstrs(quicksum(x[i, j] for i in V if i != j) == 1 for j in N) # (12) each customer is visited at least once

mdl.addConstrs((x[i, j] == 1) >> (u[i] + q[i] == u[j]) for i, j in A if i != 0 and j != 0) # elimination constraint, subtours

mdl.addConstrs(u[i] >= q[i] for i in N)  # (14)
mdl.addConstrs(u[i] <= Q for i in N)     # (14)

# Add constraint to limit the number of vehicles (10)
mdl.addConstr(quicksum(x[0, j] for j in N) == 1) # Just one vehicle

mdl.optimize()

# Get active arcs
active_arcs = [(i, j) for i, j in A if x[i, j].X > 0.9] # Find arcs where x[i, j] is approximately equal to 1

# Plot solution with active arcs
for i, j in active_arcs:
    plt.plot([xc[i], xc[j]], [yc[i], yc[j]], color="g", zorder=0)
    plt.annotate(city_names[j], (xc[j], yc[j]), textcoords="offset points", xytext=(0, 10), ha='center')

plt.plot(xc[0], yc[0], c='r', marker='s') # That is the depot
plt.scatter(xc[1:], yc[1:], c='b')        # These are the clients

#Print graphical solution
plt.xlabel("Distance X")
plt.ylabel("Distance Y")
plt.title("The Pollition-Routing Problem")

plt.show()