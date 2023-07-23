import math
import json
import requests
import pathlib
import warnings
import geopy.distance
from urllib import parse
from constants import MAPBOX_API_KEY


CONFIGURATION_FILEPATH = pathlib.Path("config.json")
MOST_ALLOWED_COORDINATES = 90
LOOP_SIMULATION_INSTEAD_OF_ENDING = True

# Vehicle Dynamics Variables
SPEED_METERS_PER_SECOND = 12.0
LATERAL_ACCELERATION_METERS_PER_SECOND = 12.0
MAX_LINEAR_SPEED_METERS_PER_SECOND = 12.0
MAX_LINEAR_ACCELERATION_METERS_PER_SECOND_SQUARED = 10.0
MAX_LINEAR_JERK_METERS_PER_SECOND_CUBED = 1.0
MAX_LATERAL_ACCELERATION_METERS_PER_SECOND_SQUARED = 10.0
MAX_LATERAL_JERK_METERS_PER_SECOND_CUBED = 1.0


class GeoCoordinate:
    def __init__(self, latitude: float, longitude: float):
        self.lat = latitude
        self.lon = longitude

    def calculate_distance(self, other, unit="km") -> float:
        distance = geopy.distance.geodesic((self.lat, self.lon), (other.lat, other.lon))
        return getattr(distance, unit)

    def calculate_bearing(self, other) -> float:
        lat1 = self.lat
        lon1 = self.lon
        lon2 = other.lon
        lat2 = other.lat

        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)

        diff_lon = lon2_rad - lon1_rad

        y = math.sin(diff_lon) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(diff_lon)
        bearing_rad = math.atan2(y, x)

        bearing_deg = math.degrees(bearing_rad)

        bearing_normalized = (bearing_deg + 360) % 360

        return bearing_normalized


def get_configuration(config_file):
    if not config_file.is_file():
        raise FileNotFoundError(f"{config_file.absolute()} is not a file.")

    with open(config_file, 'r') as file:
        config = json.load(file)

    starting_point = GeoCoordinate(**config["Starting Point"])
    config_stops = config.get("Stops")
    stops = []
    if config_stops is not None:
        for stop in config_stops:
            stops.append(GeoCoordinate(**stop))
    ending_point = GeoCoordinate(**config["Ending Point"])
    output_filename = config["Output Filename"]

    return starting_point, stops, ending_point, output_filename


def generate_waypoints(starting_point: GeoCoordinate, stops: list[GeoCoordinate],
                       ending_point: GeoCoordinate) -> list[GeoCoordinate]:
    domain = "https://api.mapbox.com"

    stops_string = f"{';'.join([f'{stop.lon},{stop.lat}' for stop in stops])}{';' if len(stops) else ''}"

    path = f"/directions/v5/mapbox/driving/" \
           f"{starting_point.lon},{starting_point.lat};" \
           f"{stops_string}" \
           f"{ending_point.lon},{ending_point.lat}"
    query_string_seperator = "?"
    params = {
        "alternatives": "true",
        "geometries": "geojson",
        "language": "en",
        "overview": "full",
        "steps": "true",
        "access_token": MAPBOX_API_KEY
    }
    querystring = parse.urlencode(params)
    url = domain + path + query_string_seperator + querystring
    resp = requests.get(url)

    if not resp.ok:
        raise ConnectionError(f"Generate Waypoints API returned {resp.status_code=}")

    routes = resp.json()["routes"]

    if len(routes) == 0:
        raise ValueError("This route is not possible")

    coordinates_list = []
    coordinates = routes[0]["geometry"]["coordinates"]
    for coordinate in coordinates:
        coordinates_list.append(GeoCoordinate(latitude=coordinate[1], longitude=coordinate[0]))

    return coordinates_list


def get_bearing_list_from_coordinates_list(coordinates: list[GeoCoordinate]) -> list[float]:
    bearings = []
    for i in range(len(coordinates) - 1):
        bearing = GeoCoordinate.calculate_bearing(coordinates[i], coordinates[i + 1])
        bearings.append(bearing)
    bearings.append(0.0)  # the bearings list is one shorter than the coordinate list, so it must be appended with a 0.0
    return bearings


def filter_waypoints(waypoints, bearings):
    # filters the points to remove points that are have a change of direction by less than 2 degrees
    new_waypoints = []
    current_bearing = float('nan')
    for bearing, waypoint in zip(bearings, waypoints):
        if not (current_bearing - 2 < bearing < current_bearing + 2):
            new_waypoints.append(waypoint)
            current_bearing = bearing
    if new_waypoints[-1] != waypoints[-1]:
        new_waypoints.append(waypoints[-1])

    waypoints = new_waypoints

    if MOST_ALLOWED_COORDINATES is None or len(waypoints) <= MOST_ALLOWED_COORDINATES:
        return waypoints
    else:
        warnings.warn(f"MOST_ALLOWED_COORDINATES exceeded - {len(waypoints) - MOST_ALLOWED_COORDINATES} points lost")

    # filters the points to restrict the waypoints to the eliminate points that are the closest together
    # as they are the least important
    distances = [0.0]
    for i in range(len(waypoints) - 1):
        distance_km = GeoCoordinate.calculate_distance(waypoints[i], waypoints[i + 1], unit="km")
        distances.append(distance_km)

    sorted_distance_km, sorted_coordinates = zip(*sorted(zip(distances, waypoints), key=lambda x: x[0], reverse=True))
    most_important_coordinates = sorted_coordinates[:MOST_ALLOWED_COORDINATES - 2]

    new_waypoints_list = [waypoints[0]]
    for coordinate in waypoints:
        if coordinate in most_important_coordinates:
            new_waypoints_list.append(coordinate)
    new_waypoints_list.append(waypoints[-1])

    return new_waypoints_list


def get_elevation_list_from_coordinates_list(coordinates: list[GeoCoordinate]) -> list[float]:
    domain = "https://api.open-meteo.com/v1/elevation"
    query_string_seperator = "?"

    elevation_list = []

    # the open-meteo api used only allows for up to 100 elevations to be returned from one
    # call, so this loop calls the api 100 coordinates at a time until the coordinates list is exhausted
    for i in range(len(coordinates) // 100 + 1):
        left_index = i * 100
        right_index = i * 100 + 100

        next_hundred_latitude_list = [str(cord.lat) for cord in coordinates[left_index: right_index]]
        next_hundred_longitude_list = [str(cord.lon) for cord in coordinates[left_index: right_index]]

        latitude_string = ",".join(next_hundred_latitude_list)
        longitude_string = ",".join(next_hundred_longitude_list)

        params = {
            "latitude": latitude_string,
            "longitude": longitude_string
        }
        querystring = parse.urlencode(params)
        url = domain + query_string_seperator + querystring
        url = url.replace("%2C", ",")
        resp = requests.get(url)

        if not resp.ok:
            raise ConnectionError(f"Elevation API returned {resp.status_code=}")

        elevations = resp.json()["elevation"]
        elevation_list.extend(elevations)

    return elevation_list


def output(waypoints, elevations, bearings, *, filename=None):
    motion_command = f"SIM:POS:MOTION:WRITE {{line_number}}, {{command}}"

    all_commands = []
    starting_commands = [
        f"DYN, {LATERAL_ACCELERATION_METERS_PER_SECOND}, {MAX_LINEAR_SPEED_METERS_PER_SECOND}, "
        f"{MAX_LINEAR_ACCELERATION_METERS_PER_SECOND_SQUARED}, {MAX_LINEAR_JERK_METERS_PER_SECOND_CUBED}, "
        f"{MAX_LATERAL_ACCELERATION_METERS_PER_SECOND_SQUARED}, {MAX_LATERAL_JERK_METERS_PER_SECOND_CUBED}",

        f"REF, {waypoints[0].lat}, {waypoints[0].lon}, {elevations[0]}, {bearings[0]:.3f}, {SPEED_METERS_PER_SECOND}"
    ]
    all_commands.extend(starting_commands)
    for index, (waypoint, altitude, bearing) in enumerate(zip(waypoints, elevations, bearings)):
        command = f"WAYPT, {waypoint.lat}, {waypoint.lon}, {altitude}, {bearing:.3f}, " \
                  f"{LATERAL_ACCELERATION_METERS_PER_SECOND}"
        all_commands.append(command)
    ending_commands = [
        f"{'GOTO' if LOOP_SIMULATION_INSTEAD_OF_ENDING else 'END'}, {len(starting_commands)+1}"
    ]
    all_commands.extend(ending_commands)

    if filename is None or filename == "":
        for index, command in enumerate(all_commands):
            full_command = motion_command.format(line_number=index+1, command=command)
            print(full_command)

    elif filename.endswith('.kml'):
        import simplekml
        kml = simplekml.Kml()
        for index, (cord, altitude, heading) in enumerate(zip(waypoints, elevations, bearings)):
            kml.newpoint(name=f"Point: {index+1}", coords=[(cord.lon, cord.lat, altitude)], description=str(heading))
        kml.save(filename)

    else:
        with open(filename, 'w') as f:
            for index, command in enumerate(all_commands):
                full_command = motion_command.format(line_number=index+1, command=command)
                f.write(f"{full_command}\n")


def make_gps_simulation_commands():
    starting_point, stops, ending_point, output_filename = get_configuration(CONFIGURATION_FILEPATH)
    waypoints = generate_waypoints(starting_point, stops, ending_point)
    bearings = get_bearing_list_from_coordinates_list(waypoints)
    waypoints = filter_waypoints(waypoints, bearings)
    bearings = get_bearing_list_from_coordinates_list(waypoints)
    elevations = get_elevation_list_from_coordinates_list(waypoints)
    output(waypoints, elevations, bearings, filename=output_filename)


if __name__ == '__main__':
    make_gps_simulation_commands()
