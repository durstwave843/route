"""
Verify the CP-SAT solution is actually optimal by comparing
against some intuitive alternatives.
"""

import math
from itertools import combinations

def haversine_distance(coord1, coord2):
    """Calculate distance between two lat/lon coordinates in km."""
    lat1, lon1 = coord1
    lat2, lon2 = coord2
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.asin(math.sqrt(a))
    return c * 6371

def calc_total_distance(denver, a, b, c, d):
    """Calculate total distance for all three trips."""
    trip1 = (haversine_distance(denver, a) +
             haversine_distance(a, b) +
             haversine_distance(b, denver))

    trip2 = (haversine_distance(denver, a) +
             haversine_distance(a, b) +
             haversine_distance(b, c) +
             haversine_distance(c, d) +
             haversine_distance(d, denver))

    trip3 = (haversine_distance(denver, c) +
             haversine_distance(c, d) +
             haversine_distance(d, denver))

    return trip1, trip2, trip3, trip1 + trip2 + trip3

def main():
    denver = (39.7392, -104.9903)

    candidates = [
        ("Colorado_Springs", 38.8339, -104.8214),
        ("Fort_Collins", 40.5853, -105.0844),
        ("Boulder", 40.0150, -105.2705),
        ("Aurora", 39.7294, -104.8319),
        ("Lakewood", 39.7047, -105.0814),
        ("Thornton", 39.8680, -104.9719),
        ("Arvada", 39.8028, -105.0875),
        ("Westminster", 39.8367, -105.0372),
        ("Centennial", 39.5807, -104.8767),
        ("Littleton", 39.6133, -105.0166),
        ("Parker", 39.5186, -104.7614),
        ("Broomfield", 39.9205, -105.0866),
        ("Commerce_City", 39.8083, -104.9339),
        ("Englewood", 39.6478, -104.9875),
        ("Golden", 39.7555, -105.2211),
    ]

    # CP-SAT Solution
    print("="*80)
    print("VERIFYING CP-SAT OPTIMAL SOLUTION")
    print("="*80)

    cpsat_assignment = {
        'a': ("Commerce_City", 39.8083, -104.9339),
        'b': ("Thornton", 39.8680, -104.9719),
        'c': ("Westminster", 39.8367, -105.0372),
        'd': ("Arvada", 39.8028, -105.0875),
    }

    t1, t2, t3, total = calc_total_distance(
        denver,
        cpsat_assignment['a'][1:],
        cpsat_assignment['b'][1:],
        cpsat_assignment['c'][1:],
        cpsat_assignment['d'][1:]
    )

    print(f"\n✅ CP-SAT Solution:")
    print(f"   a={cpsat_assignment['a'][0]}, b={cpsat_assignment['b'][0]}")
    print(f"   c={cpsat_assignment['c'][0]}, d={cpsat_assignment['d'][0]}")
    print(f"   Trip 1: {t1:.2f} km")
    print(f"   Trip 2: {t2:.2f} km")
    print(f"   Trip 3: {t3:.2f} km")
    print(f"   TOTAL: {total:.2f} km")

    cpsat_total = total

    # Test some intuitive alternatives
    print(f"\n{'='*80}")
    print("TESTING ALTERNATIVE ASSIGNMENTS")
    print("="*80)

    test_cases = [
        {
            'name': 'Closest 4 to Denver',
            'a': ("Aurora", 39.7294, -104.8319),
            'b': ("Lakewood", 39.7047, -105.0814),
            'c': ("Englewood", 39.6478, -104.9875),
            'd': ("Littleton", 39.6133, -105.0166),
        },
        {
            'name': 'All North of Denver',
            'a': ("Thornton", 39.8680, -104.9719),
            'b': ("Westminster", 39.8367, -105.0372),
            'c': ("Broomfield", 39.9205, -105.0866),
            'd': ("Fort_Collins", 40.5853, -105.0844),
        },
        {
            'name': 'All South of Denver',
            'a': ("Aurora", 39.7294, -104.8319),
            'b': ("Centennial", 39.5807, -104.8767),
            'c': ("Littleton", 39.6133, -105.0166),
            'd': ("Colorado_Springs", 38.8339, -104.8214),
        },
        {
            'name': 'Clustered Near Each Other',
            'a': ("Arvada", 39.8028, -105.0875),
            'b': ("Lakewood", 39.7047, -105.0814),
            'c': ("Golden", 39.7555, -105.2211),
            'd': ("Boulder", 40.0150, -105.2705),
        },
    ]

    better_found = False

    for i, test in enumerate(test_cases, 1):
        t1, t2, t3, total = calc_total_distance(
            denver,
            test['a'][1:],
            test['b'][1:],
            test['c'][1:],
            test['d'][1:]
        )

        difference = total - cpsat_total
        symbol = "❌ WORSE" if difference > 0 else "⚠️ BETTER"

        if difference < 0:
            better_found = True

        print(f"\n{i}. {test['name']}:")
        print(f"   a={test['a'][0]}, b={test['b'][0]}")
        print(f"   c={test['c'][0]}, d={test['d'][0]}")
        print(f"   Total: {total:.2f} km ({difference:+.2f} km) {symbol}")

    # Sample random assignments
    print(f"\n{'='*80}")
    print("SAMPLING 10 RANDOM ASSIGNMENTS")
    print("="*80)

    import random
    random.seed(42)

    for i in range(10):
        sample = random.sample(range(len(candidates)), 4)
        a, b, c, d = [candidates[j] for j in sample]

        t1, t2, t3, total = calc_total_distance(
            denver, a[1:], b[1:], c[1:], d[1:]
        )

        difference = total - cpsat_total

        if difference < 0:
            better_found = True
            print(f"\n{i+1}. FOUND BETTER: {a[0]}, {b[0]}, {c[0]}, {d[0]}")
            print(f"   Total: {total:.2f} km ({difference:+.2f} km) ⚠️")
        elif difference < 10:
            print(f"\n{i+1}. Close: {total:.2f} km ({difference:+.2f} km)")

    print(f"\n{'='*80}")
    print("CONCLUSION")
    print("="*80)

    if better_found:
        print("⚠️  FOUND A BETTER SOLUTION - CP-SAT MAY NOT BE OPTIMAL!")
    else:
        print("✅ NO BETTER SOLUTIONS FOUND")
        print("   CP-SAT solution appears to be optimal!")
        print(f"   Total Distance: {cpsat_total:.2f} km")
    print("="*80 + "\n")

if __name__ == "__main__":
    main()
