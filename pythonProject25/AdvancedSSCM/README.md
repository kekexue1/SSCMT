
# DISTANCE MINIMIZATION

This model represents the Capacitated Vehicle Routing Problem (CVRP), which is a problem of finding the optimal routes for a fleet of vehicles to serve a set of clients with known demands. The objective is to minimize the total distance traveled by the vehicles while satisfying certain constraints.

Here is an explanation of the key components of the model:

- Variables:
  - `x`: Binary decision variables that indicate whether a vehicle travels in an arc (edge) between two vertices (nodes).
  - `u`: Continuous variables representing the cumulative load carried by each vehicle.

- Sets and Parameters:
  - `n`: The number of clients to visit (excluding the depot).
  - `N`: The set of clients (1 to n).
  - `V`: The set of vertices or nodes, including the depot (0) and the clients (N).
  - `A`: The set of arcs, defined as all possible combinations of vertices where the start and end vertices are not the same.
  - `c`: A dictionary representing the cost of travel (or distance) for each arc in A.
  - `Q`: The vehicle capacity.
  - `q`: The demand of each customer.

- Constraints:
  - Balance of flow constraint: The flow into and out of each vertex (except the depot) must be equal to the number of times it is visited by the vehicles.
  - Each customer is visited at least once: Each customer node must be visited by at least one vehicle.
  - Elimination constraint (subtour elimination): Ensures that subtours, where a vehicle visits multiple customers and returns to the depot without serving all the customers, are not allowed.
  - Vehicle load constraints: The cumulative load carried by each vehicle must be greater than or equal to the demand of each customer and less than or equal to the vehicle capacity.
  - Constraint to limit the number of vehicles: Ensures that only one vehicle departs from the depot.

- Objective:
  - Minimize the total distance traveled by the vehicles. The objective function is defined as the sum of the distance (cost) multiplied by the binary decision variable `x` for each arc in A.

The code also includes visualization of the solution using Matplotlib. It plots the depot, clients, and the active arcs representing the optimal routes found by the model. Additionally, it annotates the cities' names near their respective locations on the plot.

Overall, the model aims to solve the CVRP by finding the optimal routes for vehicles to visit customers, minimizing the total distance traveled while satisfying capacity and visitation constraints.
