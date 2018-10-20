import PIL.Image
import math


class MapHelper(object):

    @staticmethod
    def new_image(width, height, alpha=False):
        """
        Generates a new image using PIL.Image module

        returns PIL.IMAGE OBJECT
        """
        if alpha is True:
            return PIL.Image.new('RGBA', (width, height), (0, 0, 0, 0))
        else:
            return PIL.Image.new('RGBA', (width, height))

    @staticmethod
    def fast_round(value, precision):
        """
        Function to round values instead of using python's

        return INT
        """
        return int(value * 10 ** precision) / 10. ** precision

    @staticmethod
    def pixels_to_degrees(pixels, zoom):
        """
        Generates pixels to be expected at zoom levels

        returns INT
        """
        return pixels * 2 ** (21-zoom)

    @staticmethod
    def pixels_to_meters(latitude, zoom):
        """
        Function generates how many pixels per meter it
        should be from the projecction

        returns FLOAT
        """
        # https://groups.google.com/forum/#!topic/google-maps-js-api-v3/hDRO4oHVSeM
        return 2 ** zoom / (156543.03392 * math.cos(math.radians(latitude)))
