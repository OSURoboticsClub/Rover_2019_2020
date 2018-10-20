'''
MappingSettings.py: Objected Orientated Google Maps for Python
ReWritten by Chris Pham

Copyright OSURC, orginal code from GooMPy by Alec Singer and Simon D. Levy

This code is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.
This code is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with this code.  If not, see <http://www.gnu.org/licenses/>.
'''

#####################################
# Imports
#####################################
# Python native imports
import math
import urllib2
from io import StringIO, BytesIO
import os
import time
import PIL.ImageDraw
import PIL.Image
import PIL.ImageFont
import signing
import RoverMapHelper as MapHelper
import cv2
import numpy as np

#####################################
# Constants
#####################################
_KEYS = []
# Number of pixels in half the earth's circumference at zoom = 21
_EARTHPIX = 268435456
# Number of decimal places for rounding coordinates
_DEGREE_PRECISION = 6
_PRECISION_FORMAT = '%.' + str(_DEGREE_PRECISION) + 'f'
# Larget tile we can grab without paying
_TILESIZE = 640
# Fastest rate at which we can download tiles without paying
_GRABRATE = 4
# Pixel Radius of Earth for calculations
_PIXRAD = _EARTHPIX / math.pi
_DISPLAYPIX = _EARTHPIX / 2000

file_pointer = open('key', 'r')
for i in file_pointer:
    _KEYS.append(i.rstrip())
file_pointer.close()


class GMapsStitcher(object):
    def __init__(self, width, height,
                 latitude, longitude, zoom,
                 maptype, radius_meters=None, num_tiles=4, debug=False):
        self.helper = MapHelper.MapHelper()
        self.latitude = latitude
        self.longitude = longitude
        self.start_latitude = latitude
        self.start_longitude = longitude
        self.width = width
        self.height = height
        self.zoom = zoom
        self.maptype = maptype
        self.radius_meters = radius_meters
        self.num_tiles = num_tiles
        self.display_image = self.helper.new_image(width, height)
        self.debug = debug

        # Get the big image here
        self._fetch()
        self.center_display(latitude, longitude)

    def __str__(self):
        """
        This string returns when used in a print statement
        Useful for debugging and to print current state

        returns STRING
        """
        string_builder = ""
        string_builder += ("Center of the displayed map: %4f, %4f\n" %
                           (self.center_x, self.center_y))
        string_builder += ("Center of the big map: %4fx%4f\n" %
                           (self.start_longitude, self.start_longitude))
        string_builder += ("Current latitude is: %4f, %4f\n" %
                           (self.longitude, self.latitude))
        string_builder += ("The top-left of the box: %dx%d\n" %
                           (self.left_x, self.upper_y))
        string_builder += ("Number of tiles genreated: %dx%d\n" %
                           (self.num_tiles, self.num_tiles))
        string_builder += "Map Type: %s\n" % (self.maptype)
        string_builder += "Zoom Level: %s\n" % (self.zoom)
        string_builder += ("Dimensions of Big Image: %dx%d\n" %
                           (self.big_image.size[0], self.big_image.size[1]))
        string_builder += ("Dimensions of Displayed Image: %dx%d\n" %
                           (self.width, self.height))
        string_builder += ("LatLong of Northwest Corner: %4f, %4f\n" %
                           (self.northwest))
        string_builder += ("LatLong of Southeast Corner: %4f, %4f\n" %
                           (self.southeast))
        return string_builder

    def _grab_tile(self, longitude, latitude, sleeptime=0):
        """
        This will return the tile at location longitude x latitude.
        Includes a sleep time to allow for free use if there is no API key

        returns PIL.IMAGE OBJECT
        """
        # Make the url string for polling
        # GET request header gets appended to the string
        urlbase = 'https://maps.googleapis.com/maps/api/staticmap?'
        urlbase += 'center=' + _PRECISION_FORMAT + ',' + _PRECISION_FORMAT + '&zoom=%d&maptype=%s'
        urlbase += '&size=%dx%d&format=png&key=%s'

        # Fill the formatting
        specs = (self.helper.fast_round(latitude, _DEGREE_PRECISION),
                 self.helper.fast_round(longitude, _DEGREE_PRECISION),
                 self.zoom, self.maptype, _TILESIZE, _TILESIZE, _KEYS[0])
        filename = 'Resources/Maps/' + ((_PRECISION_FORMAT + '_' + _PRECISION_FORMAT + '_%d_%s_%d_%d_%s') % specs)
        filename += '.png'

        # Tile Image object
        tile_object = None

        if os.path.isfile(filename):
            tile_object = PIL.Image.open(filename)

        # If file on filesystem
        else:
            # make the url
            url = urlbase % specs
            url = signing.sign_url(url, _KEYS[1])
            try:
                result = urllib2.urlopen(urllib2.Request(url)).read()
            except urllib2.HTTPError, e:
                print "Error accessing url for reason:", e
                print url
                return

            tile_object = PIL.Image.open(BytesIO(result))
            if not os.path.exists('Resources/Maps'):
                os.mkdir('Resources/Maps')
            tile_object.save(filename)
            # Added to prevent timeouts on Google Servers
            time.sleep(sleeptime)

        return tile_object

    def _pixels_to_lon(self, iterator, lon_pixels):
        """
        This converts pixels to degrees to be used in
        fetching squares and generate correct squares

        returns FLOAT(degrees)
        """
        # Magic Lines, no idea
        degrees = self.helper.pixels_to_degrees(
            (iterator - self.num_tiles / 2) * _TILESIZE, self.zoom)
        return math.degrees((lon_pixels + degrees - _EARTHPIX) / _PIXRAD)

    def _pixels_to_lat(self, iterator, lat_pixels):
        """
        This converts pixels to latitude using meridian projection
        to get the latitude to generate squares

        returns FLOAT(degrees)
        """
        # Magic Lines
        return math.degrees(math.pi / 2 - 2 * math.atan(math.exp(((lat_pixels +
                                                                   self.helper.pixels_to_degrees(
                                                                       (iterator - self.num_tiles /
                                                                        2) * _TILESIZE, self.zoom)) -
                                                                  _EARTHPIX) / _PIXRAD)))

    def fetch_tiles(self):
        """
        Function that handles fetching of files from init'd variables

        returns PIL.IMAGE OBJECT, (WEST, NORTH), (EAST, SOUTH)

        North/East/South/West are in FLOAT(degrees)
        """
        # cap floats to precision amount
        self.latitude = self.helper.fast_round(self.latitude,
                                               _DEGREE_PRECISION)
        self.longitude = self.helper.fast_round(self.longitude,
                                                _DEGREE_PRECISION)

        # number of tiles required to go from center
        # latitude to desired radius in meters
        if self.radius_meters is not None:
            self.num_tiles = (int(
                round(2 * self.helper.pixels_to_meters(
                    self.latitude, self.zoom) /
                      (_TILESIZE / 2. / self.radius_meters))))

        lon_pixels = _EARTHPIX + self.longitude * math.radians(_PIXRAD)

        sin_lat = math.sin(math.radians(self.latitude))
        lat_pixels = _EARTHPIX - _PIXRAD * math.log((1 + sin_lat) / (1 - sin_lat)) / 2
        self.big_size = self.num_tiles * _TILESIZE
        big_image = self.helper.new_image(self.big_size, self.big_size)

        for j in range(self.num_tiles):
            lon = self._pixels_to_lon(j, lon_pixels)
            for k in range(self.num_tiles):
                lat = self._pixels_to_lat(k, lat_pixels)
                tile = self._grab_tile(lon, lat)
                big_image.paste(tile, (j * _TILESIZE, k * _TILESIZE))

        west = self._pixels_to_lon(0, lon_pixels)
        east = self._pixels_to_lon(self.num_tiles - 1, lon_pixels)

        north = self._pixels_to_lat(0, lat_pixels)
        south = self._pixels_to_lat(self.num_tiles - 1, lat_pixels)
        return big_image, (north, west), (south, east)

    def move_pix(self, dx, dy):
        """
        Function gets change in x and y (dx, dy)
        then displaces the displayed map that amount

        NO RETURN
        """
        self._constrain_x(dx)
        self._constrain_y(dy)
        self.update()

    def _constrain_x(self, diff):
        """
        Helper for move_pix
        """
        new_value = self.left_x - diff

        if ((not new_value > 0) and
                (new_value < self.big_image.size[0] - self.width)):
            return self.left_x
        else:
            return new_value

    def _constrain_y(self, diff):
        """
        Helper for move_pix
        """
        new_value = self.upper_y - diff

        if ((not new_value > 0) and
                (new_value < self.big_image.size[1] - self.height)):
            return self.upper_y
        else:
            return new_value

    def update(self):
        """
        Function remakes display image using top left corners
        """
        self.display_image.paste(self.big_image, (-self.left_x, -self.upper_y))
        # self.display_image.resize((self.image_zoom, self.image_zoom))

    def _fetch(self):
        """
        Function generates big image
        """
        self.big_image, self.northwest, self.southeast = self.fetch_tiles()

    def move_latlon(self, lat, lon):
        """
        Function to move the object/rover
        """
        x, y = self._get_cartesian(lat, lon)
        self._constrain_x(self.center_x - x)
        self._constrain_y(self.center_y - y)
        self.update()

    def _get_cartesian(self, lat, lon):
        """
        Helper for getting the x, y given lat and lon

        returns INT, INT (x, y)
        """
        viewport_lat_nw, viewport_lon_nw = self.northwest
        viewport_lat_se, viewport_lon_se = self.southeast
        # print "Lat:", viewport_lat_nw, viewport_lat_se
        # print "Lon:", viewport_lon_nw, viewport_lon_se

        viewport_lat_diff = viewport_lat_nw - viewport_lat_se
        viewport_lon_diff = viewport_lon_se - viewport_lon_nw

        # print viewport_lon_diff, viewport_lat_diff

        bigimage_width = self.big_image.size[0]
        bigimage_height = self.big_image.size[1]

        pixel_per_lat = bigimage_height / viewport_lat_diff
        pixel_per_lon = bigimage_width / viewport_lon_diff
        # print "Pixel per:", pixel_per_lat, pixel_per_lon

        new_lat_gps_range_percentage = (viewport_lat_nw - lat)
        new_lon_gps_range_percentage = (lon - viewport_lon_nw)
        # print lon, viewport_lon_se

        x = new_lon_gps_range_percentage * pixel_per_lon
        y = new_lat_gps_range_percentage * pixel_per_lat

        return int(x), int(y)

    def add_gps_location(self, lat, lon, shape, size, fill):
        """
        Function adds a shape at lat x lon
        """
        x, y = self._get_cartesian(lat, lon)
        draw = PIL.ImageDraw.Draw(self.big_image)
        if shape is "ellipsis":
            draw.ellipsis((x - size, y - size, x + size, y + size), fill)
        else:
            draw.rectangle([x - size, y - size, x + size, y + size], fill)
        self.update()

    def center_display(self, lat, lon):
        """
        Function centers the display image
        """
        x, y = self._get_cartesian(lat, lon)
        self.center_x = x
        self.center_y = y

        self.left_x = (self.center_x - (self.width / 2))
        self.upper_y = (self.center_y - (self.height / 2))
        self.update()

    # def update_rover_map_location(self, lat, lon):
    #     print "I did nothing"

    # def draw_circle(self, lat, lon, radius, fill):
    #     print "I did nothing"

    def connect_signals_and_slots(self):
        pass


class OverlayImage(object):
    def __init__(self, latitude, longitude, northwest, southeast,
                 big_width, big_height, width, height):
        self.northwest = northwest
        self.southeast = southeast
        self.latitude = latitude
        self.longitude = longitude
        self.big_width = big_width
        self.big_height = big_height
        self.width = width
        self.height = height
        self.big_image = None
        self.big_image_copy = None
        self.display_image = None
        self.display_image_copy = None
        self.indicator = None
        self.helper = MapHelper.MapHelper()

        x, y = self._get_cartesian(latitude, longitude)
        self.center_x = x
        self.center_y = y

        self.left_x = (self.center_x - (self.width / 2))
        self.upper_y = (self.center_y - (self.height / 2))

        self.generate_image_files()
        self.write_once = True

        # Text Drawing Variables
        self.font = cv2.FONT_HERSHEY_TRIPLEX
        self.font_thickness = 1
        self.font_baseline = 0

        self.nav_coordinates_text_image = None

    def generate_image_files(self):
        """
        Creates big_image and display image sizes

        Returns NONE
        """
        self.big_image = self.helper.new_image(self.big_width, self.big_height,
                                               True)

        self.big_image_copy = self.big_image.copy()

        self.display_image = self.helper.new_image(self.width, self.height,
                                                   True)

        self.display_image_copy = self.display_image.copy()

        self.load_rover_icon()
        self.indicator.save("location.png")

    def _get_cartesian(self, lat, lon):
        """
        Helper for getting the x, y given lat and lon

        returns INT, INT (x, y)
        """
        viewport_lat_nw, viewport_lon_nw = self.northwest
        viewport_lat_se, viewport_lon_se = self.southeast
        # print "Lat:", viewport_lat_nw, viewport_lat_se
        # print "Lon:", viewport_lon_nw, viewport_lon_se

        viewport_lat_diff = viewport_lat_nw - viewport_lat_se
        viewport_lon_diff = viewport_lon_se - viewport_lon_nw

        # print viewport_lon_diff, viewport_lat_diff

        pixel_per_lat = self.big_height / viewport_lat_diff
        pixel_per_lon = self.big_width / viewport_lon_diff
        # print "Pixel per:", pixel_per_lat, pixel_per_lon

        new_lat_gps_range_percentage = (viewport_lat_nw - lat)
        new_lon_gps_range_percentage = (lon - viewport_lon_nw)
        # print lon, viewport_lon_se

        x = new_lon_gps_range_percentage * pixel_per_lon
        y = new_lat_gps_range_percentage * pixel_per_lat

        return int(x), int(y)

    def update_new_location(self, latitude, longitude,
                            compass, navigation_list, landmark_list):
        self.big_image = self.big_image_copy.copy()
        self.display_image = self.display_image_copy.copy()

        size = 5
        draw = PIL.ImageDraw.Draw(self.big_image)

        for element in navigation_list:
            x, y = self._get_cartesian(float(element[1]), float(element[2]))
            draw.text((x + 10, y - 5), str(element[0]))
            draw.ellipse((x - size, y - size, x + size, y + size), fill=(element[3].red(), element[3].green(), element[3].blue()))

        for element in landmark_list:
            x, y = self._get_cartesian(element[1], element[2])
            draw.text((x + 10, y - 5), str(element[0]))
            draw.ellipse((x - size, y - size, x + size, y + size), fill=(element[3].red(), element[3].green(), element[3].blue()))

        self._draw_rover(latitude, longitude, compass)
        self.update(latitude, longitude)

        return self.display_image

    def load_rover_icon(self):
        self.indicator = PIL.Image.open("Resources/Images/rover.png").resize((40, 40))

    def _draw_rover(self, lat, lon, angle=0):
        x, y = self._get_cartesian(lat, lon)

        x -= 25 # Half the height of the icon
        y -= 25

        rotated = self.indicator.copy()
        rotated = rotated.rotate(-angle, resample=PIL.Image.BICUBIC)
        # rotated.save("rotated.png")
        self.big_image.paste(rotated, (x, y), rotated)
        if self.write_once:
            # self.display_image.save("Something.png")
            self.write_once = False

    def update(self, latitude, longitude):
        # self.left_x -= 50
        # self.upper_y -= 50
        self.display_image.paste(self.big_image, (-self.left_x, -self.upper_y))
        # self._draw_coordinate_text(latitude, longitude)

    def connect_signals_and_slots(self):
        pass
