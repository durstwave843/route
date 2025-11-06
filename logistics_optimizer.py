"""
Logistics Optimization using Google OR-Tools

Problem:
- Trip 1: Denver (2 items) → location a → location b → Denver (0 items)
- Trip 2: Denver (0 items) → location a → location b → location c → location d → Denver (0 items)
- Trip 3: Denver (0 items) → location c → location d → Denver (2 items)

Goal: Find optimal locations for a, b, c, d to minimize total travel distance
"""

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import math


class LogisticsOptimizer:
    """Optimize location assignments for a multi-trip delivery problem."""

    def __init__(self, candidate_locations, denver_coords=(39.7392, -104.9903)):
        """
        Initialize the optimizer.

        Args:
            candidate_locations: List of tuples (name, lat, lon) for potential locations
            denver_coords: Tuple of (lat, lon) for Denver depot
        """
        self.denver = denver_coords
        self.candidate_locations = candidate_locations

    def distance(self, coord1, coord2):
        """Calculate Euclidean distance between two coordinates (simplified)."""
        # For more accuracy, use Haversine formula for lat/lon
        lat1, lon1 = coord1
        lat2, lon2 = coord2
        # Approximate distance (in arbitrary units)
        return math.sqrt((lat1 - lat2)**2 + (lon1 - lon2)**2)

    def haversine_distance(self, coord1, coord2):
        """Calculate great circle distance between two points in km."""
        lat1, lon1 = coord1
        lat2, lon2 = coord2

        # Convert to radians
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

        # Haversine formula
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a))
        r = 6371  # Radius of earth in kilometers
        return c * r

    def calculate_total_distance(self, loc_a, loc_b, loc_c, loc_d):
        """
        Calculate total distance for all three trips.

        Args:
            loc_a, loc_b, loc_c, loc_d: Coordinates (lat, lon) for each location

        Returns:
            Total distance for all trips
        """
        # Trip 1: Denver → a → b → Denver
        trip1 = (self.haversine_distance(self.denver, loc_a) +
                self.haversine_distance(loc_a, loc_b) +
                self.haversine_distance(loc_b, self.denver))

        # Trip 2: Denver → a → b → c → d → Denver
        trip2 = (self.haversine_distance(self.denver, loc_a) +
                self.haversine_distance(loc_a, loc_b) +
                self.haversine_distance(loc_b, loc_c) +
                self.haversine_distance(loc_c, loc_d) +
                self.haversine_distance(loc_d, self.denver))

        # Trip 3: Denver → c → d → Denver
        trip3 = (self.haversine_distance(self.denver, loc_c) +
                self.haversine_distance(loc_c, loc_d) +
                self.haversine_distance(loc_d, self.denver))

        return trip1 + trip2 + trip3

    def optimize_brute_force(self):
        """
        Find optimal location assignment using brute force.
        Works well for small number of candidate locations.
        """
        best_distance = float('inf')
        best_assignment = None

        n = len(self.candidate_locations)

        # Try all permutations of 4 locations from candidates
        from itertools import permutations

        for perm in permutations(range(n), min(4, n)):
            if len(perm) < 4:
                continue

            loc_a = self.candidate_locations[perm[0]][1:]  # (lat, lon)
            loc_b = self.candidate_locations[perm[1]][1:]
            loc_c = self.candidate_locations[perm[2]][1:]
            loc_d = self.candidate_locations[perm[3]][1:]

            total_dist = self.calculate_total_distance(loc_a, loc_b, loc_c, loc_d)

            if total_dist < best_distance:
                best_distance = total_dist
                best_assignment = {
                    'a': self.candidate_locations[perm[0]],
                    'b': self.candidate_locations[perm[1]],
                    'c': self.candidate_locations[perm[2]],
                    'd': self.candidate_locations[perm[3]],
                    'total_distance_km': total_dist
                }

        return best_assignment

    def optimize_with_vrp(self):
        """
        Optimize using OR-Tools Vehicle Routing Problem solver.
        This approach models the multi-trip constraint problem.
        """
        # Create routing index manager
        # Locations: 0=Denver, then candidate locations
        num_locations = len(self.candidate_locations) + 1
        num_vehicles = 3  # 3 trips
        depot = 0  # Denver is depot

        manager = pywrapcp.RoutingIndexManager(num_locations, num_vehicles, depot)
        routing = pywrapcp.RoutingModel(manager)

        # Create distance callback
        def distance_callback(from_index, to_index):
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)

            from_coord = self.denver if from_node == 0 else self.candidate_locations[from_node - 1][1:]
            to_coord = self.denver if to_node == 0 else self.candidate_locations[to_node - 1][1:]

            # Return distance in meters (int)
            return int(self.haversine_distance(from_coord, to_coord) * 1000)

        transit_callback_index = routing.RegisterTransitCallback(distance_callback)
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        # Add capacity constraints for items
        def demand_callback(from_index):
            # This is a simplified model - capacity tracking is complex for pickup/delivery
            return 0

        demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)

        # Set search parameters
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
        search_parameters.local_search_metaheuristic = (
            routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
        search_parameters.time_limit.seconds = 30

        # Solve
        solution = routing.SolveWithParameters(search_parameters)

        if solution:
            return self._extract_solution(manager, routing, solution)
        else:
            return None

    def _extract_solution(self, manager, routing, solution):
        """Extract solution from OR-Tools solver."""
        total_distance = 0
        routes = []

        for vehicle_id in range(routing.vehicles()):
            index = routing.Start(vehicle_id)
            route = []
            route_distance = 0

            while not routing.IsEnd(index):
                node = manager.IndexToNode(index)
                route.append(node)
                previous_index = index
                index = solution.Value(routing.NextVar(index))
                route_distance += routing.GetArcCostForVehicle(
                    previous_index, index, vehicle_id)

            route.append(manager.IndexToNode(index))
            routes.append({
                'trip': vehicle_id + 1,
                'route': route,
                'distance_m': route_distance
            })
            total_distance += route_distance

        return {
            'routes': routes,
            'total_distance_km': total_distance / 1000
        }


def main():
    """Example usage of the logistics optimizer."""

    # Example candidate locations around Denver
    candidate_locations = [
        ("Location_1", 39.7, -105.0),    # West of Denver
        ("Location_2", 39.8, -104.9),    # North of Denver
        ("Location_3", 39.7, -104.8),    # East of Denver
        ("Location_4", 39.6, -104.9),    # South of Denver
        ("Location_5", 39.75, -105.1),   # Northwest
        ("Location_6", 39.65, -104.8),   # Southeast
        ("Location_7", 39.8, -105.0),    # North-Northwest
        ("Location_8", 39.6, -105.0),    # South-Southwest
    ]

    print("="*70)
    print("LOGISTICS OPTIMIZATION PROBLEM")
    print("="*70)
    print("\nProblem Description:")
    print("- Trip 1: Denver (2 items) → a → b → Denver (0 items)")
    print("- Trip 2: Denver (0 items) → a → b → c → d → Denver (0 items)")
    print("- Trip 3: Denver (0 items) → c → d → Denver (2 items)")
    print("\nObjective: Find optimal locations for a, b, c, d")
    print("="*70)

    # Initialize optimizer
    optimizer = LogisticsOptimizer(candidate_locations)

    print("\n[1] Running Brute Force Optimization...")
    print("-" * 70)
    result = optimizer.optimize_brute_force()

    if result:
        print("\n✓ Optimal Assignment Found:")
        print(f"\n  Location a: {result['a'][0]} at ({result['a'][1]:.4f}, {result['a'][2]:.4f})")
        print(f"  Location b: {result['b'][0]} at ({result['b'][1]:.4f}, {result['b'][2]:.4f})")
        print(f"  Location c: {result['c'][0]} at ({result['c'][1]:.4f}, {result['c'][2]:.4f})")
        print(f"  Location d: {result['d'][0]} at ({result['d'][1]:.4f}, {result['d'][2]:.4f})")

        print(f"\n  Total Distance: {result['total_distance_km']:.2f} km")

        # Calculate individual trip distances
        loc_a = result['a'][1:]
        loc_b = result['b'][1:]
        loc_c = result['c'][1:]
        loc_d = result['d'][1:]

        trip1 = (optimizer.haversine_distance(optimizer.denver, loc_a) +
                optimizer.haversine_distance(loc_a, loc_b) +
                optimizer.haversine_distance(loc_b, optimizer.denver))

        trip2 = (optimizer.haversine_distance(optimizer.denver, loc_a) +
                optimizer.haversine_distance(loc_a, loc_b) +
                optimizer.haversine_distance(loc_b, loc_c) +
                optimizer.haversine_distance(loc_c, loc_d) +
                optimizer.haversine_distance(loc_d, optimizer.denver))

        trip3 = (optimizer.haversine_distance(optimizer.denver, loc_c) +
                optimizer.haversine_distance(loc_c, loc_d) +
                optimizer.haversine_distance(loc_d, optimizer.denver))

        print("\n  Trip Breakdown:")
        print(f"    Trip 1 (Denver → a → b → Denver): {trip1:.2f} km")
        print(f"    Trip 2 (Denver → a → b → c → d → Denver): {trip2:.2f} km")
        print(f"    Trip 3 (Denver → c → d → Denver): {trip3:.2f} km")
    else:
        print("\n✗ No solution found")

    print("\n" + "="*70)
    print("\n[2] Running OR-Tools VRP Optimization...")
    print("-" * 70)
    vrp_result = optimizer.optimize_with_vrp()

    if vrp_result:
        print("\n✓ OR-Tools Solution:")
        print(f"\n  Total Distance: {vrp_result['total_distance_km']:.2f} km")
        print("\n  Routes:")
        for route_info in vrp_result['routes']:
            route_nodes = route_info['route']
            route_names = []
            for node in route_nodes:
                if node == 0:
                    route_names.append("Denver")
                else:
                    route_names.append(candidate_locations[node - 1][0])
            print(f"    Trip {route_info['trip']}: {' → '.join(route_names)} ({route_info['distance_m']/1000:.2f} km)")
    else:
        print("\n✗ No VRP solution found")

    print("\n" + "="*70)


if __name__ == "__main__":
    main()
