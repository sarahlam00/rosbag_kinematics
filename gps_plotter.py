import gpxpy
import gpxpy.gpx

from geopy.distance import geodesic

import matplotlib.pyplot as plt

gpx_file = open('test_1.gpx', 'r')

gpx = gpxpy.parse(gpx_file)

gps_coordinates = []
cartesian_coordinates = []

x_values = []
y_values = []
elevations = []

for track in gpx.tracks:
    for segment in track.segments:
        for point in segment.points:
            gps_coordinates.append((point.time.timestamp(), point.latitude, point.longitude, point.elevation))

# Reference point (lat, lon)
reference_latitude = gps_coordinates[0][1]
reference_longitude = gps_coordinates[0][2]

for time, latitude, longitude, elevation in gps_coordinates:

    # Y (north-south)
    north_m = geodesic((reference_latitude, reference_longitude), (latitude, reference_longitude)).meters
    if latitude < reference_latitude:
        north_m *= -1

    # X (east-west)
    east_m = geodesic((reference_latitude, reference_longitude), (reference_latitude, longitude)).meters
    if longitude < reference_longitude:
        east_m *= -1

    cartesian_coordinates.append((time, east_m, north_m, elevation))

    x_values.append(east_m)
    y_values.append(north_m)
    elevations.append(elevation)

for point in cartesian_coordinates:
    print(f"Time: {point[0]:.1f}, X: {point[1]:.3f} m, Y: {point[2]:.3f} m")

# Plotting
plt.figure(figsize=(8, 8))
plt.plot(x_values, y_values, marker='o')

plt.xlabel('X (meters from start, East)')
plt.ylabel('Y (meters from start, North)')
plt.title('Rover GPS Path')
plt.grid(True)

# Make axes equal
plt.gca().set_aspect('equal', adjustable='box')
plt.axis('equal')  # This ensures 1 unit on x == 1 unit on y

plt.tight_layout()
plt.show()