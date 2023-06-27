import math
import geopy.distance
import requests
from urllib import parse
import constants


OUTPUT_FILENAME = "output.txt"
MOST_ALLOWED_COORDINATES = 90
LATERAL_ACCELERATION = 12.0


class GeoCoordinate:
    def __init__(self, latitude, longitude):
        self.lat = latitude
        self.lon = longitude

    def calculate_distance(self, other):
        distance = geopy.distance.geodesic((self.lat, self.lon), (other.lat, other.lon)).km
        return distance

    def calculate_bearing(self, other):
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


def generate_waypoints(starting_coordinate: GeoCoordinate, stops: list, ending_coordinate: GeoCoordinate):
    domain = "https://api.mapbox.com"

    stops_string = f"{';'.join([f'{stop.lon},{stop.lat}' for stop in stops])}{';' if len(stops) else ''}"

    path = f"/directions/v5/mapbox/driving/" \
           f"{starting_coordinate.lon},{starting_coordinate.lat};" \
           f"{stops_string}" \
           f"{ending_coordinate.lon},{ending_coordinate.lat}"
    query_string_seperator = "?"
    params = {
        "alternatives": "true",
        "geometries": "geojson",
        "language": "en",
        "overview": "full",
        "steps": "true",
        "access_token": constants.MAPBOX_API_KEY
    }
    querystring = parse.urlencode(params)
    url = domain + path + query_string_seperator + querystring
    resp = requests.get(url)

    if not resp.ok:
        raise ConnectionError(f"{resp.status_code=}")

    coordinates_list = [starting_coordinate]
    legs = resp.json()["routes"][0]["legs"]
    for leg in legs:
        steps = leg["steps"]
        for step in steps:
            coordinate = step["geometry"]["coordinates"][0]
            coordinates_list.append(GeoCoordinate(latitude=coordinate[1], longitude=coordinate[0]))
    return coordinates_list


def filter_waypoints(waypoints_list):
    """This function filters the waypoints to the MOST_ALLOWED_COORDINATES number of longest paths between points"""
    distances_list = [float('inf')]
    for i in range(len(waypoints_list) - 1):
        distance = GeoCoordinate.calculate_distance(waypoints_list[i], waypoints_list[i + 1])
        distances_list.append(distance)

    sorted_distance, sorted_coordinates = zip(*sorted(zip(distances_list, waypoints_list)))
    most_important_coordinates = sorted_coordinates[:MOST_ALLOWED_COORDINATES - 2]

    new_waypoints_list = [waypoints_list[0]]
    for coordinate in waypoints_list:
        if coordinate in most_important_coordinates:
            new_waypoints_list.append(coordinate)
    new_waypoints_list[-1] = waypoints_list[-1]

    return new_waypoints_list


def get_bearing_list_from_coordinates_list(coordinates_list: list[GeoCoordinate]):
    bearing_list = []
    for i in range(len(coordinates_list) - 1):
        bearing = GeoCoordinate.calculate_bearing(coordinates_list[i], coordinates_list[i + 1])
        bearing_list.append(bearing)
    return bearing_list


def get_elevation_list_from_coordinates_list(coordinates_list: list[GeoCoordinate]):
    domain = "https://api.open-meteo.com/v1/elevation"
    query_string_seperator = "?"
    latitude_string = ",".join([str(cord.lat) for cord in coordinates_list])
    longitude_string = ",".join([str(cord.lon) for cord in coordinates_list])
    params = {
        "latitude": latitude_string,
        "longitude": longitude_string
    }
    querystring = parse.urlencode(params)
    url = domain + query_string_seperator + querystring
    url = url.replace("%2C", ",")
    resp = requests.get(url)

    if not resp.ok:
        raise ConnectionError(f"{resp.status_code=}")

    elevation_list = resp.json()["elevation"]

    return elevation_list


def output(waypoints_list, elevation_list, bearing_list, *, filename=None):
    motion_command = f"SIM:POS:MOTION:WRITE {{line_number}}, "
    starting_coordinate = waypoints_list[0]
    starting_commands = [
        "DYN, 12.000, 10.000, 1.000, 10.000, 1.000",
        f"REF, {starting_coordinate.lat}, {starting_coordinate.lon}, {elevation_list[0]}, {bearing_list[0]}, "
        f"{LATERAL_ACCELERATION}"
    ]
    all_commands = starting_commands.copy()
    for index, (waypoint, altitude, bearing) in enumerate(zip(waypoints_list, elevation_list, bearing_list)):
        command = f"WAYPT, {waypoint.lat}, {waypoint.lon}, {altitude}, {bearing}, {LATERAL_ACCELERATION}"
        all_commands.append(command)
    ending_commands = [
        f"WAYPT, {waypoints_list[-1].lat}, {waypoints_list[-1].lon}, {elevation_list[-1]},, {LATERAL_ACCELERATION}",
        f"GOTO, {len(starting_commands) + 1}"
    ]
    all_commands.extend(ending_commands)

    if filename is None:
        for index, command in enumerate(all_commands):
            full_command = motion_command.format(line_number=index + 1) + command
            print(full_command)
    else:
        with open(filename, 'w') as f:
            for index, command in enumerate(all_commands):
                full_command = motion_command.format(line_number=index + 1) + command
                f.write(f"{full_command}\n")


def make_gps_simulation():
    starting_coordinate = GeoCoordinate(40.751723, -72.986082)
    stops = [
    ]
    ending_coordinate = GeoCoordinate(40.7484665, -73.985542)

    waypoints_list = generate_waypoints(starting_coordinate, stops, ending_coordinate)
    waypoints_list = filter_waypoints(waypoints_list)

    bearing_list = get_bearing_list_from_coordinates_list(waypoints_list)

    elevation_list = get_elevation_list_from_coordinates_list(waypoints_list)
    # TODO Add functionality to prevent duplicate runs
    output(waypoints_list, bearing_list, elevation_list, filename=OUTPUT_FILENAME)


if __name__ == '__main__':
    make_gps_simulation()
