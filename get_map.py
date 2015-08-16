import cv2
import numpy as np
import urllib2

MAP_HEIGHT = 640
SCALE = 1
ZOOM = 20
LAT = 51.195340
LONG = 4.465070
CENTER = (LAT, LONG)
BASE_URL = 'http://maps.googleapis.com/maps/api/staticmap'
FILE_NAME = 'staticmap.png'
# roadmap, satellite, hybrid, terrain
MAP_TYPE = 'roadmap'

def plot_lat_lon(lat, lon):
    offset_lat_degrees = lat - CENTER[0]
    offset_lon_degrees = lon - CENTER[1]
    degrees_in_map = (MAP_HEIGHT / 256.0) * (360.0 / pow(2, ZOOM))
    grid_x = (offset_lon_degrees / degrees_in_map) * MAP_HEIGHT
    grid_y = (offset_lat_degrees / degrees_in_map) * MAP_HEIGHT
    x, y = grid_x + ((MAP_HEIGHT * SCALE) / 2), grid_y + ((MAP_HEIGHT * SCALE) / 2)
    return int(x), int(y)

def get_map():
    url = (BASE_URL + '?center=' + str(LAT) + ',' + str(LONG) + '&zoom=' + str(ZOOM) + '&size=' + str(MAP_HEIGHT) + 'x' + str(MAP_HEIGHT)
                + '&scale=' + str(SCALE) + '&maptype=' + MAP_TYPE)
    response = urllib2.urlopen(url)
    png_bytes = np.asarray([ord(char) for char in response.read()], dtype=np.uint8)
    cv_array = cv2.imdecode(png_bytes, cv2.CV_LOAD_IMAGE_UNCHANGED)
    cv2.imwrite(FILE_NAME, cv_array)

try:
    get_map()
    cv2.namedWindow('Map')
    cv2.startWindowThread()
    map = cv2.imread('staticmap.png', 1)
    locations = []
    locations.append(CENTER)
    for (x, y) in locations:
        coords = plot_lat_lon(x, y)
    while True:
        cv2.circle(map, coords, 2, (0, 0, 255), 2)
        cv2.imshow('Map', map)
except:
    print 'No map generated. Maybe check you network connection?'
