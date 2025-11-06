"""
Example: Customizing the logistics optimizer for your own locations

This example shows how to:
1. Use your own custom locations
2. Modify the depot location
3. Add constraints (e.g., must visit certain locations)
"""

import math
from itertools import permutations


def haversine_distance(coord1, coord2):
    """Calculate distance between two lat/lon coordinates in km."""
    lat1, lon1 = coord1
    lat2, lon2 = coord2

    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.asin(math.sqrt(a))
    r = 6371
    return c * r


def calculate_total_distance(denver, loc_a, loc_b, loc_c, loc_d):
    """Calculate total distance for all three trips."""
    trip1 = (haversine_distance(denver, loc_a) +
             haversine_distance(loc_a, loc_b) +
             haversine_distance(loc_b, denver))

    trip2 = (haversine_distance(denver, loc_a) +
             haversine_distance(loc_a, loc_b) +
             haversine_distance(loc_b, loc_c) +
             haversine_distance(loc_c, loc_d) +
             haversine_distance(loc_d, denver))

    trip3 = (haversine_distance(denver, loc_c) +
             haversine_distance(loc_c, loc_d) +
             haversine_distance(loc_d, denver))

    return trip1 + trip2 + trip3


def optimize_with_constraint(candidates, denver, required_location_a=None):
    """
    Optimize with constraint that location 'a' must be a specific location.

    Args:
        candidates: List of (name, lat, lon) tuples
        denver: (lat, lon) for depot
        required_location_a: Index of location that MUST be assigned to 'a'

    Returns:
        Best solution satisfying constraint
    """
    best_distance = float('inf')
    best_solution = None

    n = len(candidates)

    # If location a is fixed, only vary b, c, d
    if required_location_a is not None:
        print(f"Constraint: Location a must be {candidates[required_location_a][0]}")

        loc_a = candidates[required_location_a][1:]

        # Try all combinations for b, c, d
        for perm in permutations([i for i in range(n) if i != required_location_a], 3):
            loc_b = candidates[perm[0]][1:]
            loc_c = candidates[perm[1]][1:]
            loc_d = candidates[perm[2]][1:]

            total = calculate_total_distance(denver, loc_a, loc_b, loc_c, loc_d)

            if total < best_distance:
                best_distance = total
                best_solution = {
                    'a': candidates[required_location_a],
                    'b': candidates[perm[0]],
                    'c': candidates[perm[1]],
                    'd': candidates[perm[2]],
                    'total_km': total
                }
    else:
        # No constraint - try all permutations
        for perm in permutations(range(n), 4):
            loc_a = candidates[perm[0]][1:]
            loc_b = candidates[perm[1]][1:]
            loc_c = candidates[perm[2]][1:]
            loc_d = candidates[perm[3]][1:]

            total = calculate_total_distance(denver, loc_a, loc_b, loc_c, loc_d)

            if total < best_distance:
                best_distance = total
                best_solution = {
                    'a': candidates[perm[0]],
                    'b': candidates[perm[1]],
                    'c': candidates[perm[2]],
                    'd': candidates[perm[3]],
                    'total_km': total
                }

    return best_solution


def main():
    """Example usage with custom locations and constraints."""

    # Example 1: Use your own custom locations
    # Replace these with your actual warehouse/depot locations
    my_locations = [
        ("Warehouse_North", 40.0, -105.0),
        ("Warehouse_South", 39.5, -105.0),
        ("Warehouse_East", 39.75, -104.5),
        ("Warehouse_West", 39.75, -105.5),
        ("Distribution_Center_1", 39.9, -104.8),
        ("Distribution_Center_2", 39.6, -105.2),
    ]

    # Change this to your depot location
    my_depot = (39.7392, -104.9903)  # Denver

    print("="*70)
    print("EXAMPLE 1: Custom Locations")
    print("="*70)

    result = optimize_with_constraint(my_locations, my_depot)

    print("\nOptimal Solution:")
    print(f"  Location a: {result['a'][0]}")
    print(f"  Location b: {result['b'][0]}")
    print(f"  Location c: {result['c'][0]}")
    print(f"  Location d: {result['d'][0]}")
    print(f"  Total Distance: {result['total_km']:.2f} km")

    # Example 2: With constraint
    print("\n" + "="*70)
    print("EXAMPLE 2: With Constraint (location a must be Warehouse_North)")
    print("="*70)

    result2 = optimize_with_constraint(my_locations, my_depot, required_location_a=0)

    print("\nConstrained Solution:")
    print(f"  Location a: {result2['a'][0]} (FIXED)")
    print(f"  Location b: {result2['b'][0]}")
    print(f"  Location c: {result2['c'][0]}")
    print(f"  Location d: {result2['d'][0]}")
    print(f"  Total Distance: {result2['total_km']:.2f} km")

    # Example 3: Different depot
    print("\n" + "="*70)
    print("EXAMPLE 3: Different Depot Location")
    print("="*70)

    # Use Boulder as depot instead of Denver
    boulder_depot = (40.0150, -105.2705)

    result3 = optimize_with_constraint(my_locations, boulder_depot)

    print(f"\nDepot: Boulder ({boulder_depot[0]:.4f}°, {boulder_depot[1]:.4f}°)")
    print("\nOptimal Solution:")
    print(f"  Location a: {result3['a'][0]}")
    print(f"  Location b: {result3['b'][0]}")
    print(f"  Location c: {result3['c'][0]}")
    print(f"  Location d: {result3['d'][0]}")
    print(f"  Total Distance: {result3['total_km']:.2f} km")

    print("\n" + "="*70)


if __name__ == "__main__":
    main()
