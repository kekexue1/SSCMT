
# Weighted load minimization
# n is the number of clients to visit
# N is the set of clients, with N = {1, 2,...n}
# V set of vertices (or nodes), with V = {0} U N
# A set of arcs, with A = {(i,j) E V**2 : i != j}
# cij is distance of travel over arc(i,j)
# Q is the vehicle capacity
# qi is demand of each client


from gurobipy import *
import matplotlib.pyplot as plt
import numpy as np

# parameters
rnd = np.random
rnd.seed(0)

n = 5 # number of clients
w = 3 # weight of vehicles
g = 9.81 # gravity acceleration
Cr = 0.01 #
a = 0 # acceleration
ar = 0
z = a + g*math.sin(ar) + g*Cr*math.cos(ar) # constant

# graph
xc = rnd.rand(n+1) * 200  # x coordinate
yc = rnd.rand(n+1) * 100  # y coordinate
print(xc)
print(yc)

city_names = ["Depot", "Guadalajara", "Tijuana", "Mexico City", "Cancun", "Merida"] # Create array of cities


for i, txt in enumerate(city_names):
    plt.annotate(txt, (xc[i], yc[i]), textcoords="offset points", xytext=(0, 10), ha='center') # Just add cities names randomly


N = [i for i in range(1, n + 1)] # Set of clients [1,2,3,4,5,6,7,8,9,10]
V = [0] + N # The union of 0 + N [0,1,2,3,4,5,6,7,8,9,10]
A = [(i, j) for i in V for j in V if i != j]  # possible route among all clients and depot

c = {(i, j): np.hypot(xc[i] - xc[j], yc[i] - yc[j]) for i, j in A} # the distance between two point
Q = 30 # vehicle capacity
q = {i: rnd.randint(1, 10) for i in N} # just random demand


# Create a new model
mdl = Model('CVRP')

# Create variables
x = mdl.addVars(A, vtype=GRB.BINARY, name="x") # if a vehicle travels in an arc
f = mdl.addVars(A, vtype=GRB.CONTINUOUS, name="f") # the amount of commodity flowing at which a vehicle travels on this arc
u = mdl.addVars(N, vtype=GRB.CONTINUOUS, name="u")

# objective function
mdl.modelSense = GRB.MINIMIZE
mdl.setObjective(quicksum(z * c[i, j] * w * x[i, j] for i, j in A) + quicksum(z * c[i, j] * f[i, j] for i, j in A))

# constraint 10 //one vehicle
mdl.addConstr(quicksum(x[0, j] for j in N) == 1)

# constraint 11
mdl.addConstrs(quicksum(x[i, j] for j in V if j != i) == 1 for i in V)

# constraint 12
mdl.addConstrs(quicksum(x[i, j] for i in V if i != j) == 1 for j in V)

# constraint 13
mdl.addConstrs(quicksum(f[j, i] for i, j in A) - quicksum(f[i,j] for i, j in A) == q[i] for i in N)


# constraint 14
mdl.addConstrs(u[i] >= q[i] for i in N)
mdl.addConstrs(u[i] <= Q for i in N)

# constraint 20
mdl.addConstrs(f[i,j] >= 0 for i, j in A)

mdl.optimize()

# Get active arcs
active_arcs = [(i, j) for i, j in A if x[i, j]. X > 0]

#active_arcs = [(i, j) for i, j in A if x[i, j].X > 0.9] # Find arcs where x[i, j] is approximately equal to 1

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
