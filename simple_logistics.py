"""
Simplified Logistics Optimization using Google OR-Tools

Direct solution to the 3-trip problem with optimal location selection.
"""

import math
from itertools import permutations


def haversine_distance(coord1, coord2):
    """Calculate distance between two lat/lon coordinates in km."""
    lat1, lon1 = coord1
    lat2, lon2 = coord2

    # Convert to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.asin(math.sqrt(a))
    r = 6371  # Earth radius in km
    return c * r


def calculate_trip_distances(denver, loc_a, loc_b, loc_c, loc_d):
    """
    Calculate distances for all three trips.

    Trip 1: Denver → a → b → Denver
    Trip 2: Denver → a → b → c → d → Denver
    Trip 3: Denver → c → d → Denver
    """
    # Trip 1: Deliver items to a and b
    trip1 = (haversine_distance(denver, loc_a) +
             haversine_distance(loc_a, loc_b) +
             haversine_distance(loc_b, denver))

    # Trip 2: Move items from a,b to c,d
    trip2 = (haversine_distance(denver, loc_a) +
             haversine_distance(loc_a, loc_b) +
             haversine_distance(loc_b, loc_c) +
             haversine_distance(loc_c, loc_d) +
             haversine_distance(loc_d, denver))

    # Trip 3: Pickup items from c and d
    trip3 = (haversine_distance(denver, loc_c) +
             haversine_distance(loc_c, loc_d) +
             haversine_distance(loc_d, denver))

    return trip1, trip2, trip3, trip1 + trip2 + trip3


def optimize_locations(candidate_locations, denver_coords=(39.7392, -104.9903)):
    """
    Find the optimal assignment of candidate locations to a, b, c, d.

    Args:
        candidate_locations: List of tuples (name, lat, lon)
        denver_coords: Denver depot coordinates (lat, lon)

    Returns:
        Dictionary with optimal assignment and distances
    """
    best_distance = float('inf')
    best_solution = None

    # Try all possible assignments of 4 locations from candidates
    n = len(candidate_locations)

    if n < 4:
        raise ValueError("Need at least 4 candidate locations")

    print(f"Evaluating {math.perm(n, 4)} possible location assignments...")

    for perm in permutations(range(n), 4):
        # Extract coordinates for this assignment
        loc_a = candidate_locations[perm[0]][1:]  # (lat, lon)
        loc_b = candidate_locations[perm[1]][1:]
        loc_c = candidate_locations[perm[2]][1:]
        loc_d = candidate_locations[perm[3]][1:]

        # Calculate total distance
        trip1, trip2, trip3, total = calculate_trip_distances(
            denver_coords, loc_a, loc_b, loc_c, loc_d
        )

        # Track best solution
        if total < best_distance:
            best_distance = total
            best_solution = {
                'locations': {
                    'a': candidate_locations[perm[0]],
                    'b': candidate_locations[perm[1]],
                    'c': candidate_locations[perm[2]],
                    'd': candidate_locations[perm[3]],
                },
                'trips': {
                    'trip1_km': trip1,
                    'trip2_km': trip2,
                    'trip3_km': trip3,
                },
                'total_distance_km': total
            }

    return best_solution


def print_solution(solution):
    """Pretty print the optimization solution."""
    print("\n" + "="*80)
    print("OPTIMAL LOGISTICS SOLUTION")
    print("="*80)

    locations = solution['locations']
    trips = solution['trips']

    print("\n📍 LOCATION ASSIGNMENTS:")
    print("-" * 80)
    for key in ['a', 'b', 'c', 'd']:
        loc = locations[key]
        print(f"  Location {key.upper()}: {loc[0]:20s} → ({loc[1]:.4f}°, {loc[2]:.4f}°)")

    print("\n🚛 TRIP DETAILS:")
    print("-" * 80)
    print(f"  Trip 1: Denver → a → b → Denver")
    print(f"          Distance: {trips['trip1_km']:.2f} km")
    print(f"          Purpose: Deliver 2 items to locations a and b")

    print(f"\n  Trip 2: Denver → a → b → c → d → Denver")
    print(f"          Distance: {trips['trip2_km']:.2f} km")
    print(f"          Purpose: Relocate items from a,b to c,d")

    print(f"\n  Trip 3: Denver → c → d → Denver")
    print(f"          Distance: {trips['trip3_km']:.2f} km")
    print(f"          Purpose: Pick up 2 items from c and d")

    print(f"\n📊 TOTAL DISTANCE: {solution['total_distance_km']:.2f} km")
    print("="*80 + "\n")


def main():
    """Run the logistics optimization."""

    # Define Denver location
    denver = (39.7392, -104.9903)

    # Define candidate locations (these would be your actual warehouse/depot options)
    # Format: (name, latitude, longitude)
    candidates = [
        # Colorado Springs area
        ("Colorado_Springs", 38.8339, -104.8214),
        # Fort Collins area
        ("Fort_Collins", 40.5853, -105.0844),
        # Boulder area
        ("Boulder", 40.0150, -105.2705),
        # Aurora area
        ("Aurora", 39.7294, -104.8319),
        # Lakewood area
        ("Lakewood", 39.7047, -105.0814),
        # Thornton area
        ("Thornton", 39.8680, -104.9719),
        # Arvada area
        ("Arvada", 39.8028, -105.0875),
        # Westminster area
        ("Westminster", 39.8367, -105.0372),
        # Centennial area
        ("Centennial", 39.5807, -104.8767),
        # Littleton area
        ("Littleton", 39.6133, -105.0166),
    ]

    print("\n" + "="*80)
    print("LOGISTICS OPTIMIZATION PROBLEM")
    print("="*80)
    print(f"\nDepot: Denver ({denver[0]:.4f}°, {denver[1]:.4f}°)")
    print(f"\nCandidate Locations: {len(candidates)}")
    for name, lat, lon in candidates:
        print(f"  • {name:20s} ({lat:.4f}°, {lon:.4f}°)")

    print("\nProblem Structure:")
    print("  Trip 1: Denver (2 items) → a → b → Denver (0 items)")
    print("  Trip 2: Denver (0 items) → a → b → c → d → Denver (0 items)")
    print("  Trip 3: Denver (0 items) → c → d → Denver (2 items)")
    print("\nObjective: Minimize total distance across all trips")
    print("="*80)

    # Run optimization
    solution = optimize_locations(candidates, denver)

    # Print results
    print_solution(solution)


if __name__ == "__main__":
    main()
