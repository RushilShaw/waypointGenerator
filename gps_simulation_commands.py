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


class CommandPoint(GeoCoordinate):
    def __init__(self, latitude: float, longitude: float, altitude: float, heading: float):
        super().__init__(latitude, longitude)
        self.alt = altitude
        self.hdg = heading
        self.function = "WAYPT"

    @property
    def command(self):
        if self.function == "WAYPT":
            return f"{self.function}, {self.lat}, {self.lon}, {self.alt}, {self.hdg:.3f}, " \
                   f"{LATERAL_ACCELERATION_METERS_PER_SECOND}"


class Path:
    def __init__(self, starting_point: GeoCoordinate, stops: list[GeoCoordinate], ending_point: GeoCoordinate):
        self.starting_point = starting_point
        self.stops = stops
        self.ending_point = ending_point
        self.command_points: list[CommandPoint] = []

    def generate_waypoints(self):
        domain = "https://api.mapbox.com"

        stops_string = f"{';'.join([f'{stop.lon},{stop.lat}' for stop in self.stops])}{';' if len(self.stops) else ''}"

        path = f"/directions/v5/mapbox/driving/" \
               f"{self.starting_point.lon},{self.starting_point.lat};" \
               f"{stops_string}" \
               f"{self.ending_point.lon},{self.ending_point.lat}"
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

    def generate_command_points(self):
        waypoints = self.generate_waypoints()
        elevations = get_elevation_list_from_coordinates_list(waypoints)
        bearings = get_bearing_list_from_coordinates_list(waypoints)
        for waypoint, elevation, bearing in zip(waypoints, elevations, bearings):
            command_point = CommandPoint(waypoint.lat, waypoint.lon, elevation, bearing)
            self.command_points.append(command_point)

    def filter_command_points(self):
        tolerance_degrees = 5.0
        tolerance_distance_meters = 10.0

        current_index = 0
        while current_index < len(self.command_points) - 1:
            next_index = current_index + 1
            current_bearing = self.command_points[current_index].hdg
            next_bearing = self.command_points[next_index].hdg
            heading_difference = min(abs(current_bearing - next_bearing), 360 - abs(current_bearing - next_bearing))
            similar_heading = heading_difference < tolerance_degrees
            if similar_heading:
                self.command_points.pop(next_index)
            else:
                current_index += 1

        """
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
            warnings.warn(
                f"MOST_ALLOWED_COORDINATES exceeded - {len(waypoints) - MOST_ALLOWED_COORDINATES} points lost")

        # filters the points to restrict the waypoints to the eliminate points that are the closest together
        # as they are the least important
        distances = [0.0]
        for i in range(len(waypoints) - 1):
            distance_km = GeoCoordinate.calculate_distance(waypoints[i], waypoints[i + 1], unit="km")
            distances.append(distance_km)

        sorted_distance_km, sorted_coordinates = zip(
            *sorted(zip(distances, waypoints), key=lambda x: x[0], reverse=True))
        most_important_coordinates = sorted_coordinates[:MOST_ALLOWED_COORDINATES - 2]

        new_waypoints_list = [waypoints[0]]
        for coordinate in waypoints:
            if coordinate in most_important_coordinates:
                new_waypoints_list.append(coordinate)
        new_waypoints_list.append(waypoints[-1])

        return new_waypoints_list
        """

    def output(self, *, filename=None):
        motion_command = f"SIM:POS:MOTION:WRITE {{line_number}}, {{command}}"

        first_point = self.command_points[0]

        all_commands = []
        starting_commands = [
            f"DYN, {MAX_LINEAR_SPEED_METERS_PER_SECOND}, {MAX_LINEAR_ACCELERATION_METERS_PER_SECOND_SQUARED}, "
            f"{MAX_LINEAR_JERK_METERS_PER_SECOND_CUBED}, {MAX_LATERAL_ACCELERATION_METERS_PER_SECOND_SQUARED}, "
            f"{MAX_LATERAL_JERK_METERS_PER_SECOND_CUBED}",

            f"REF, {first_point.lat}, {first_point.lon}, {first_point.alt}, {first_point.hdg:.3f}, "
            f"{SPEED_METERS_PER_SECOND}"
        ]
        all_commands.extend(starting_commands)

        for index, command_point in enumerate(self.command_points):
            all_commands.append(command_point.command)

        ending_commands = [
            f"GOTO, {len(starting_commands) + 1}" if LOOP_SIMULATION_INSTEAD_OF_ENDING else "END"
        ]
        all_commands.extend(ending_commands)

        if filename is None or filename == "":
            for index, command in enumerate(all_commands):
                full_command = motion_command.format(line_number=index + 1, command=command)
                print(full_command)

        elif filename.endswith('.kml'):
            import simplekml
            kml = simplekml.Kml()
            for index, point in enumerate(self.command_points):
                kml.newpoint(name=f"Point: {index + 1}", coords=[(point.lon, point.lat, point.alt)],
                             description=str(point.hdg))
            kml.save(filename)

        else:
            with open(filename, 'w') as f:
                for index, command in enumerate(all_commands):
                    full_command = motion_command.format(line_number=index + 1, command=command)
                    f.write(f"{full_command}\n")


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


def get_bearing_list_from_coordinates_list(coordinates: list[GeoCoordinate]) -> list[float]:
    bearings = []
    for i in range(len(coordinates) - 1):
        bearing = GeoCoordinate.calculate_bearing(coordinates[i], coordinates[i + 1])
        bearings.append(bearing)
    bearings.append(0.0)  # the bearings list is one shorter than the coordinate list, so it must be appended with a 0.0
    return bearings


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


def make_gps_simulation_commands():
    starting_point, stops, ending_point, output_filename = get_configuration(CONFIGURATION_FILEPATH)
    path = Path(starting_point, stops, ending_point)
    path.generate_command_points()
    path.filter_command_points()
    path.output(filename=output_filename)


if __name__ == '__main__':
    make_gps_simulation_commands()
