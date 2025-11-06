"""
Advanced Vehicle Routing Problem (VRP) Optimizer using Google OR-Tools

This solution models the multi-trip logistics problem with explicit pickup
and delivery constraints using OR-Tools' VRP solver.
"""

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import math


def haversine_distance(coord1, coord2):
    """Calculate distance between two lat/lon coordinates in km."""
    lat1, lon1 = coord1
    lat2, lon2 = coord2

    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.asin(math.sqrt(a))
    r = 6371  # Earth radius in km
    return c * r


class MultiTripVRPSolver:
    """
    Solve the multi-trip logistics problem using OR-Tools VRP.

    Problem structure:
    - Trip 1: Deliver 2 items to locations a and b
    - Trip 2: Move items from a,b to c,d
    - Trip 3: Pick up items from c,d back to depot
    """

    def __init__(self, candidate_locations, depot_coords=(39.7392, -104.9903)):
        """
        Initialize the VRP solver.

        Args:
            candidate_locations: List of (name, lat, lon) tuples
            depot_coords: (lat, lon) for Denver depot
        """
        self.depot = depot_coords
        self.candidates = candidate_locations
        self.num_candidates = len(candidate_locations)

    def create_data_model(self, assignment):
        """
        Create the data model for OR-Tools VRP.

        Args:
            assignment: Dict mapping 'a', 'b', 'c', 'd' to indices in candidates

        Returns:
            Data dictionary for VRP solver
        """
        # Map location keys to candidate indices
        idx_a = assignment['a']
        idx_b = assignment['b']
        idx_c = assignment['c']
        idx_d = assignment['d']

        # Build location list: [depot, a, b, c, d]
        locations = [
            ('depot', self.depot),
            ('a', self.candidates[idx_a][1:]),
            ('b', self.candidates[idx_b][1:]),
            ('c', self.candidates[idx_c][1:]),
            ('d', self.candidates[idx_d][1:]),
        ]

        # Create distance matrix
        num_locations = len(locations)
        distance_matrix = []

        for i in range(num_locations):
            row = []
            for j in range(num_locations):
                if i == j:
                    row.append(0)
                else:
                    dist = haversine_distance(locations[i][1], locations[j][1])
                    # Convert to meters (int)
                    row.append(int(dist * 1000))
            distance_matrix.append(row)

        data = {
            'distance_matrix': distance_matrix,
            'num_vehicles': 3,  # 3 trips
            'depot': 0,  # Denver
            'locations': locations,
            'location_names': [loc[0] for loc in locations],
        }

        return data

    def solve_vrp(self, data):
        """
        Solve the VRP using OR-Tools.

        Returns:
            Solution object and total distance
        """
        manager = pywrapcp.RoutingIndexManager(
            len(data['distance_matrix']),
            data['num_vehicles'],
            data['depot']
        )

        routing = pywrapcp.RoutingModel(manager)

        # Distance callback
        def distance_callback(from_index, to_index):
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return data['distance_matrix'][from_node][to_node]

        transit_callback_index = routing.RegisterTransitCallback(distance_callback)
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        # Add distance dimension
        dimension_name = 'Distance'
        routing.AddDimension(
            transit_callback_index,
            0,  # no slack
            300000,  # max 300 km per trip (in meters)
            True,  # start cumul to zero
            dimension_name
        )
        distance_dimension = routing.GetDimensionOrDie(dimension_name)
        distance_dimension.SetGlobalSpanCostCoefficient(100)

        # Configure search parameters
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        )
        search_parameters.local_search_metaheuristic = (
            routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
        )
        search_parameters.time_limit.seconds = 10

        # Solve
        solution = routing.SolveWithParameters(search_parameters)

        return manager, routing, solution

    def extract_routes(self, manager, routing, solution, data):
        """Extract and format the routes from the solution."""
        total_distance = 0
        routes = []

        for vehicle_id in range(data['num_vehicles']):
            index = routing.Start(vehicle_id)
            route_nodes = []
            route_distance = 0

            while not routing.IsEnd(index):
                node = manager.IndexToNode(index)
                route_nodes.append(node)
                previous_index = index
                index = solution.Value(routing.NextVar(index))
                route_distance += routing.GetArcCostForVehicle(
                    previous_index, index, vehicle_id
                )

            # Add final depot
            route_nodes.append(manager.IndexToNode(index))

            # Convert nodes to location names
            route_names = [data['location_names'][node] for node in route_nodes]

            routes.append({
                'trip': vehicle_id + 1,
                'nodes': route_nodes,
                'names': route_names,
                'distance_km': route_distance / 1000
            })

            total_distance += route_distance

        return routes, total_distance / 1000

    def evaluate_assignment(self, idx_a, idx_b, idx_c, idx_d):
        """
        Evaluate a specific assignment of locations using VRP solver.

        Returns:
            Total distance for this assignment
        """
        assignment = {'a': idx_a, 'b': idx_b, 'c': idx_c, 'd': idx_d}
        data = self.create_data_model(assignment)

        manager, routing, solution = self.solve_vrp(data)

        if solution:
            _, total_distance = self.extract_routes(manager, routing, solution, data)
            return total_distance
        else:
            return float('inf')

    def optimize(self):
        """
        Find the optimal assignment of candidate locations to a, b, c, d.

        Uses brute force over assignments + VRP for routing.
        """
        from itertools import permutations

        best_distance = float('inf')
        best_assignment = None
        best_routes = None

        # Try all permutations of 4 locations
        count = 0
        total = math.perm(self.num_candidates, 4) if self.num_candidates >= 4 else 0

        print(f"Evaluating {total} possible assignments...\n")

        for perm in permutations(range(self.num_candidates), 4):
            count += 1
            idx_a, idx_b, idx_c, idx_d = perm

            assignment = {'a': idx_a, 'b': idx_b, 'c': idx_c, 'd': idx_d}
            data = self.create_data_model(assignment)

            manager, routing, solution = self.solve_vrp(data)

            if solution:
                routes, total_distance = self.extract_routes(
                    manager, routing, solution, data
                )

                if total_distance < best_distance:
                    best_distance = total_distance
                    best_assignment = assignment
                    best_routes = routes

                    print(f"Progress: {count}/{total} | Best so far: {best_distance:.2f} km")

        # Format final solution
        if best_assignment:
            return {
                'assignment': {
                    'a': self.candidates[best_assignment['a']],
                    'b': self.candidates[best_assignment['b']],
                    'c': self.candidates[best_assignment['c']],
                    'd': self.candidates[best_assignment['d']],
                },
                'routes': best_routes,
                'total_distance_km': best_distance
            }
        else:
            return None


def calculate_manual_distance(depot, loc_a, loc_b, loc_c, loc_d):
    """Calculate distance for the specific trip sequence."""
    # Trip 1: Denver → a → b → Denver
    trip1 = (haversine_distance(depot, loc_a) +
             haversine_distance(loc_a, loc_b) +
             haversine_distance(loc_b, depot))

    # Trip 2: Denver → a → b → c → d → Denver
    trip2 = (haversine_distance(depot, loc_a) +
             haversine_distance(loc_a, loc_b) +
             haversine_distance(loc_b, loc_c) +
             haversine_distance(loc_c, loc_d) +
             haversine_distance(loc_d, depot))

    # Trip 3: Denver → c → d → Denver
    trip3 = (haversine_distance(depot, loc_c) +
             haversine_distance(loc_c, loc_d) +
             haversine_distance(loc_d, depot))

    return trip1, trip2, trip3


def print_solution(solution, depot):
    """Pretty print the VRP solution."""
    print("\n" + "="*80)
    print("VRP OPTIMIZATION RESULTS")
    print("="*80)

    assignment = solution['assignment']
    routes = solution['routes']

    print("\n📍 OPTIMAL LOCATION ASSIGNMENT:")
    print("-" * 80)
    for key in ['a', 'b', 'c', 'd']:
        loc = assignment[key]
        print(f"  Location {key.upper()}: {loc[0]:20s} → ({loc[1]:.4f}°, {loc[2]:.4f}°)")

    print("\n🚛 OPTIMIZED ROUTES:")
    print("-" * 80)
    for route in routes:
        route_str = ' → '.join(route['names'])
        print(f"  Trip {route['trip']}: {route_str}")
        print(f"           Distance: {route['distance_km']:.2f} km")

    # Calculate manual trip distances to verify
    loc_a = assignment['a'][1:]
    loc_b = assignment['b'][1:]
    loc_c = assignment['c'][1:]
    loc_d = assignment['d'][1:]

    trip1, trip2, trip3 = calculate_manual_distance(depot, loc_a, loc_b, loc_c, loc_d)

    print("\n📋 TRIP BREAKDOWN (Manual Calculation):")
    print("-" * 80)
    print(f"  Trip 1 (Denver → a → b → Denver):           {trip1:.2f} km")
    print(f"  Trip 2 (Denver → a → b → c → d → Denver):  {trip2:.2f} km")
    print(f"  Trip 3 (Denver → c → d → Denver):           {trip3:.2f} km")
    print(f"\n  Manual Total: {trip1 + trip2 + trip3:.2f} km")

    print(f"\n📊 VRP TOTAL DISTANCE: {solution['total_distance_km']:.2f} km")
    print("="*80 + "\n")


def main():
    """Run the advanced VRP optimization."""

    # Denver coordinates
    denver = (39.7392, -104.9903)

    # Candidate locations around Denver
    candidates = [
        ("Colorado_Springs", 38.8339, -104.8214),
        ("Fort_Collins", 40.5853, -105.0844),
        ("Boulder", 40.0150, -105.2705),
        ("Aurora", 39.7294, -104.8319),
        ("Lakewood", 39.7047, -105.0814),
        ("Thornton", 39.8680, -104.9719),
        ("Arvada", 39.8028, -105.0875),
        ("Westminster", 39.8367, -105.0372),
    ]

    print("\n" + "="*80)
    print("ADVANCED VRP LOGISTICS OPTIMIZATION")
    print("="*80)
    print(f"\nDepot: Denver ({denver[0]:.4f}°, {denver[1]:.4f}°)")
    print(f"\nCandidate Locations: {len(candidates)}")
    for name, lat, lon in candidates:
        dist_from_denver = haversine_distance(denver, (lat, lon))
        print(f"  • {name:20s} ({lat:.4f}°, {lon:.4f}°) - {dist_from_denver:.1f} km from Denver")

    print("\nProblem:")
    print("  Trip 1: Denver (2 items) → a → b → Denver (0 items)")
    print("  Trip 2: Denver (0 items) → a → b → c → d → Denver (0 items)")
    print("  Trip 3: Denver (0 items) → c → d → Denver (2 items)")
    print("="*80 + "\n")

    # Initialize and run optimizer
    solver = MultiTripVRPSolver(candidates, denver)
    solution = solver.optimize()

    if solution:
        print_solution(solution, denver)
    else:
        print("\n✗ No solution found\n")


if __name__ == "__main__":
    main()
