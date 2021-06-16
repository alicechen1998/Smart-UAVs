import math
from math import radians, cos, sin, asin, sqrt
class HaversineDistance:
    """Represents great-circle distance between two points given coordinates; user inputs coordinates"""
    def __init__(self, _x_coord_1, _y_coord_1, _x_coord_2, _y_coord_2):
        """Initializes x and y coordinates and sets starting distance to 0"""
        self._x_coord_1 = math.radians(_x_coord_1)
        self._y_coord_1 = math.radians(_y_coord_1)
        self._x_coord_2 = math.radians(_x_coord_2)
        self._y_coord_2 = math.radians(_y_coord_2)
        self._distance = 0     #no parameter needed for starting distance as it is set to 0

    def get_change_in_x_coord(self):
        """Returns change in x-coordinate"""
        return self._x_coord_2 - self._x_coord_1

    def get_change_in_y_coord(self):
        """Returns change in y-coordinate"""
        return self._y_coord_2 - self._y_coord_1

    def get_distance(self):
        """Returns distance between the two points using Haversine formula"""
        R = 6373.0      #approximate radius of Earth in km
        lon_diff = self._x_coord_2 - self._x_coord_1       #longitudinal difference between the two points
        lat_diff = self._y_coord_2 - self._y_coord_1       #latitudinal difference between the two points

        #a computes the square of half the chord length between the two points

        a = (math.sin(lat_diff / 2)) ** 2 + math.cos(self._y_coord_1) * math.cos(self._y_coord_2) * (math.sin(lon_diff / 2)) ** 2

        #c computes angular distance in radians

        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        distance = R * c
        self._distance += distance
        return self._distance

    def miles(self):
        """Returns distance between the two points in miles"""
        return self._distance * 0.62137119

print("Input x-coordinate from point 1 (longitude):")
point_1_x = float(input())
print("Input y-coordinate from point 1 (latitude):")
point_1_y = float(input())

print("Input x-coordinate from point 2 (longitude):")
point_2_x = float(input())
print("Input y-coordinate from point 2 (latitude):")
point_2_y = float(input())

dist = HaversineDistance(point_1_x, point_1_y, point_2_x, point_2_y)

print("Change in x-coordinate:", round(dist.get_change_in_x_coord(), 2), "radians")
print("Change in y-coordinate:", round(dist.get_change_in_y_coord(), 2), "radians")
print("Distance in km:", round(dist.get_distance(), 2))
print("Distance in mi:", round(dist.miles(), 2))
