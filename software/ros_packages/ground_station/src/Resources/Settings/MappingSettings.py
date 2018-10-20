# Note that the lat and lon positions below correspond to the center point of the maps you want to download
# Proper zoom level selection determines total viewable area

MAPPING_LOCATIONS = {
    "Graf Hall": {
        "latitude": 44.5675721667,
        "longitude": -123.2750535,
        "default_zoom": 18,
        "valid_zoom_options": [16, 17, 18, 19, 20],
        "pre_cache_map_zoom_levels": [18, 19, 20]
    },

    "Crystal Lake": {
        "latitude": 44.547155,
        "longitude": -123.251438,
        "default_zoom": 18,
        "valid_zoom_options": [16, 17, 18, 19, 20],
        "pre_cache_map_zoom_levels": [18, 19, 20]
    },

    "Bend's \"The Pit\"": {
        "latitude": 43.9941317,
        "longitude": -121.4150066,
        "default_zoom": 17,
        "valid_zoom_options": [16, 17, 18, 19, 20],
        "pre_cache_map_zoom_levels": [18, 19, 20]
    },

    "McMullen": {
        "latitude": 51.470326,
        "longitude": -112.773995,
        "default_zoom": 17,
        "valid_zoom_options": [16, 17, 18, 19, 20],
        "pre_cache_map_zoom_levels": [18, 19, 20]
    },

    "Compound": {
        "latitude": 51.470941,
        "longitude": -112.752322,
        "default_zoom": 17,
        "valid_zoom_options": [16, 17, 18, 19, 20],
        "pre_cache_map_zoom_levels": [18, 19, 20]
    },

    "1st Avenue": {
        "latitude": 51.453744,
        "longitude": -112.715879,
        "default_zoom": 17,
        "valid_zoom_options": [16, 17, 18, 19, 20],
        "pre_cache_map_zoom_levels": [18, 19, 20]
    },

    "Rosedale": {
        "latitude": 51.421368,
        "longitude": -112.641666,
        "default_zoom": 17,
        "valid_zoom_options": [16, 17, 18, 19, 20],
        "pre_cache_map_zoom_levels": [18, 19, 20]
    },
}

LAST_SELECTION = "Graf Hall"
LAST_ZOOM_LEVEL = MAPPING_LOCATIONS[LAST_SELECTION]["default_zoom"]

# ##### This is the offset from magnetic north to true north
DECLINATION_OFFSET = 0  # We set this to 0 so we can use a phone to calibrate
