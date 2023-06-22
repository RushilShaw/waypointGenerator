import json
import math
import geopy.distance
import requests
from urllib import parse
import constants


MOST_ALLOWED_COORDINATES = 90


def calculate_bearing(coordinate1, coordinate2):
    lat1, lon1 = coordinate1
    lat2, lon2 = coordinate2

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
    starting_coordinates = (-72.986082, 40.751723)
    stops = [
        (-73.986082, 40.751723),
        (-73.985542, 40.7484665)
    ]
    ending_coordinates = (-73.985542, 40.7484665)

    stops_string = f"{';'.join([f'{stop[0]},{stop[1]}' for stop in stops])}{';' if len(stops) else ''}"

    domain = "https://api.mapbox.com"
    path = f"/directions/v5/mapbox/driving/" \
           f"{starting_coordinates[0]},{starting_coordinates[1]};" \
           f"{stops_string}" \
           f"{ending_coordinates[0]},{ending_coordinates[1]}"

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
        bearing = calculate_bearing(*new_coordinates_list[i], *new_coordinates_list[i + 1])
        bearing_list.append(bearing)
    bearing_list.append(0.0)

    domain = "https://maps.googleapis.com/maps/api/elevation/json"
    query_string_seperator = "?"
    params = {
        "locations": "|".join([f"{cord[0]},{cord[1]}" for cord in new_coordinates_list]),
        "key": constants.GOOGLEMAPS_ELEVATION_API_KEY
    }
    querystring = parse.urlencode(params)
    url = domain + path + query_string_seperator + querystring
    resp = requests.get(url)
    results = resp.json()["results"]

    elevation_list = []
    for result in results:
        elevation_list.append(result["elevation"])


if __name__ == '__main__':
    main()
