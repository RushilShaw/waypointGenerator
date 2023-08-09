import math
import json
import requests
import pathlib
import geopy.distance
from urllib import parse


MAPBOX_API_KEY = r"sk.eyJ1IjoicnVzaGlsc2hhaCIsImEiOiJjbGo3YTc3cGgwanljM2VzYzF5cTQ0dTduIn0.6HX8RTGZHVbmqrCnUxAXWA"
CONFIGURATION_FILEPATH = pathlib.Path("config.json")
MOST_ALLOWED_WAYPOINTS = 90


class CommandPoint:
    def __init__(self, latitude: float, longitude: float, altitude: float, heading: float):
        self.lat = latitude
        self.lon = longitude
        self.alt = altitude
        self.hdg = heading
        self.function = "WAYPT"
        self.arguments = {
            "lateral_acceleration_meters_per_second": 12.0
        }

    @property
    def command(self):
        lat_acc = self.arguments["lateral_acceleration_meters_per_second"]
        return f"{self.function}, {self.lat}, {self.lon}, {self.alt}, {self.hdg:.3f}, {lat_acc}"


class GeoCoordinate:
    def __init__(self, latitude: float, longitude: float):
        self.lat = latitude
        self.lon = longitude

    @staticmethod
    def calculate_distance(self, other, unit="km") -> float:
        distance = geopy.distance.geodesic((self.lat, self.lon), (other.lat, other.lon))
        return getattr(distance, unit)

    @staticmethod
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

    def filter_command_points(self, maximum_command_points=None):
        tolerance_degrees = 5.0
        tolerance_distance_meters = 10.0

        if maximum_command_points is None or len(self.command_points) <= maximum_command_points:
            return

        # removes coordinates that have similar heading to prevent too many points in straight paths
        current_index = 0
        while current_index < len(self.command_points) - 2:
            next_index = current_index + 1
            current_command_point = self.command_points[current_index]
            next_command_point = self.command_points[next_index]
            current_bearing = current_command_point.hdg
            next_bearing = next_command_point.hdg
            heading_difference = min(abs(current_bearing - next_bearing), 360 - abs(current_bearing - next_bearing))
            similar_heading = heading_difference < tolerance_degrees
            if similar_heading:
                self.command_points.pop(next_index)
            else:
                current_index += 1

        if maximum_command_points is None or len(self.command_points) <= maximum_command_points:
            return

        # removes coordinates that have distances to prevent too many points in on top of each other
        current_index = 0
        while current_index < len(self.command_points) - 2:
            next_index = current_index + 1
            current_command_point = self.command_points[current_index]
            next_command_point = self.command_points[next_index]
            distance_meters = GeoCoordinate.calculate_distance(current_command_point, next_command_point, unit="m")
            close_coordinates = distance_meters < tolerance_distance_meters
            if close_coordinates:
                self.command_points.pop(next_index)
            else:
                current_index += 1

        if maximum_command_points is None or len(self.command_points) <= maximum_command_points:
            return

        # filters the points to remove the closest points
        distances = [0.0]
        for i in range(len(self.command_points) - 1):
            distance_km = GeoCoordinate.calculate_distance(self.command_points[i], self.command_points[i + 1])
            distances.append(distance_km)

        sorted_distances, sorted_coordinates = zip(
            *sorted(zip(distances, self.command_points), key=lambda x: x[0], reverse=True))
        most_important_command_points = sorted_coordinates[:maximum_command_points - 2]

        current_index = 1
        while current_index < len(self.command_points) - 2:
            current_command_point = self.command_points[current_index]
            if current_command_point in most_important_command_points:
                current_index += 1
            else:
                self.command_points.pop(current_index)

    def output(self, *, filename=None):
        loop_simulation_instead_of_ending = True

        speed_meters_per_second = 12.0
        max_linear_speed_meters_per_second = 12.0
        max_linear_acceleration_meters_per_second_squared = 10.0
        max_linear_jerk_meters_per_second_cubed = 1.0
        max_lateral_acceleration_meters_per_second_squared = 10.0
        max_lateral_jerk_meters_per_second_cubed = 1.0
        
        motion_command = f"SIM:POS:MOTION:WRITE {{line_number}}, {{command}}"

        first_point = self.command_points[0]

        all_commands = []
        starting_commands = [
            f"DYN, {max_linear_speed_meters_per_second}, {max_linear_acceleration_meters_per_second_squared}, "
            f"{max_linear_jerk_meters_per_second_cubed}, {max_lateral_acceleration_meters_per_second_squared}, "
            f"{max_lateral_jerk_meters_per_second_cubed}",

            f"REF, {first_point.lat}, {first_point.lon}, {first_point.alt}, {first_point.hdg:.3f}, "
            f"{speed_meters_per_second}"
        ]
        all_commands.extend(starting_commands)

        for index, command_point in enumerate(self.command_points):
            all_commands.append(command_point.command)

        ending_commands = [
            f"GOTO, {len(starting_commands) + 1}" if loop_simulation_instead_of_ending else "END"
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


def main():
    starting_point, stops, ending_point, output_filename = get_configuration(CONFIGURATION_FILEPATH)

    path = Path(starting_point, stops, ending_point)
    path.generate_command_points()
    path.filter_command_points(MOST_ALLOWED_WAYPOINTS)
    path.output(filename=output_filename)


if __name__ == '__main__':
    main()
