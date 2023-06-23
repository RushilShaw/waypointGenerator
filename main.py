import json
import math
import geopy.distance
import requests
from urllib import parse
import constants


MOST_ALLOWED_COORDINATES = 90


def calculate_bearing(coordinate1, coordinate2):
    lon1, lat1 = coordinate1
    lon2, lat2 = coordinate2

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


def main():
    starting_coordinates = (40.751723, -72.986082)
    stops = [
    ]
    ending_coordinates = (40.7484665, -73.985542)

    stops_string = f"{';'.join([f'{stop[0]},{stop[1]}' for stop in stops])}{';' if len(stops) else ''}"

    domain = "https://api.mapbox.com"
    path = f"/directions/v5/mapbox/driving/" \
           f"{starting_coordinates[1]},{starting_coordinates[0]};" \
           f"{stops_string}" \
           f"{ending_coordinates[1]},{ending_coordinates[0]}"

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
        raise ConnectionRefusedError(f"{resp.status_code=}")

    coordinates_list = [starting_coordinates]
    distances = []  # km
    legs = resp.json()["routes"][0]["legs"]
    for leg in legs:
        steps = leg["steps"]
        for step in steps:
            prev_coordinate = coordinates_list[-1]
            coordinate = step["geometry"]["coordinates"][0]
            distance = geopy.distance.geodesic(prev_coordinate, coordinate).km
            coordinates_list.append(coordinate)
            distances.append(distance)

    sorted_distance, sorted_coordinates = zip(*sorted(zip(distances, coordinates_list)))
    most_important_coordinates = sorted_coordinates[:MOST_ALLOWED_COORDINATES - 1]

    new_coordinates_list = []
    for coordinate in coordinates_list:
        if coordinate in most_important_coordinates:
            new_coordinates_list.append(coordinate)
    new_coordinates_list.append(ending_coordinates)

    bearing_list = []
    for i in range(len(new_coordinates_list) - 1):
        bearing = calculate_bearing(new_coordinates_list[i], new_coordinates_list[i + 1])
        bearing_list.append(bearing)
    bearing_list.append(0.0)

    domain = "https://api.open-meteo.com/v1/elevation"
    query_string_seperator = "?"
    params = {
        "latitude": ",".join([str(cord[1]) for cord in new_coordinates_list]),
        "longitude": ",".join([str(cord[0]) for cord in new_coordinates_list])
    }
    querystring = parse.urlencode(params)
    url = domain + query_string_seperator + querystring
    url = url.replace("%2C", ",")
    resp = requests.get(url)
    elevation_list = resp.json()["elevation"]

    for cord, alt, bearing in zip(new_coordinates_list, elevation_list, bearing_list):
        print(cord, alt, bearing)


if __name__ == '__main__':
    main()
