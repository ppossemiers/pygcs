import libardrone
import cv2 # brew install opencv --with-ffmpeg
import numpy as np
import pygame
import time
import sys
import socket
import threading

# https://cranklin.wordpress.com/2014/11/14/artificial-intelligence-applied-to-your-drone
class PID:
    def __init__(self, Kpx=0.25, Kpy=0.25, Kdx=0.25, Kdy=0.25, Kix=0.0, Kiy=0.0):
        self.Kpx = Kpx
        self.Kpy = Kpy
        self.Kdx = Kdx
        self.Kdy = Kdy
        self.Kix = Kix
        self.Kiy = Kiy

        self.errx_1 = 0
        self.erry_1 = 0

    def compute_control_command(self, errx, erry):
        # left / right
        x = self.Kpx * errx + self.Kix * (errx + self.errx_1) + self.Kdx * (errx - self.errx_1)
        # up / down
        y = self.Kpy * erry + self.Kiy * (erry + self.erry_1) + self.Kdy * (erry - self.erry_1)
        # remember values
        self.errx_1 = errx
        self.erry_1 = erry

        return (x, y)

# Ground Control Station
class GCS:
    def __init__(self, fps=30, map_center=(51.195323, 4.464865), map_zoom=20, map_scale=1,
                map_height=640, map_name='staticmap.png', use_gps=True):

        self.drone = libardrone.ARDrone()
        self.drone.config('general:navdata_demo', 'TRUE')
        self.drone.config('general:ardrone_name', 'DronePhil')
        self.drone.config('control:outdoor', 'TRUE')
        self.drone.config('control:flight_without_shell', 'TRUE')
        self.drone.config('control:altitude_max', '25000')

        self.pid = PID()
        self.fps = fps
        self.map_center = map_center
        self.map_zoom = map_zoom
        self.map_scale = map_scale
        self.map_height = map_height
        self.map_name = map_name
        self.use_gps = use_gps
        self.flight_data_bg = 'flight_data.png'
        self.gps_data = []
        self.waypoints = []

        # gps tracking
        if (self.use_gps):
            cv2.namedWindow('Map')
            cv2.moveWindow('Map', self.map_height, 0)
            cv2.cv.SetMouseCallback('Map', self.mouse_click)
            self.gps_thread = threading.Thread(target=self.process_gps)
            self.gps_thread.start()

        # flight data
        cv2.namedWindow('Flight Data')
        cv2.moveWindow('Flight Data', 0, 405)
        self.data_thread = threading.Thread(target=self.process_flight_data)
        self.data_thread.start()

        # object tracking
        cv2.namedWindow('Video')
        cv2.moveWindow('Video', 0, 0)
        self.process_video()

        cv2.startWindowThread()

    # get distance from center
    def distance(self, ctr, dim, siz):
        siz = (siz / 2) * 1.0
        return (ctr[dim] - siz) / siz

    # callback function when map is clicked
    def mouse_click(self, event, x, y, flag=0, param=None):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.waypoints.append((x, y))

    # http://scottlobdell.me/2014/03/basic-google-maps-manager-python-opencv
    # get x, y coordinates from lat, lon
    def get_x_y(self, lat, lon):
        offset_lat_degrees = lat - self.map_center[0]
        offset_lon_degrees = lon - self.map_center[1]
        # 256 pixels in google maps image, 360 degrees in a circle
        # every zoomlevel halves the degrees
        degrees_in_map = (self.map_height / 256.0) * (360.0 / pow(2, self.map_zoom))
        grid_x = (offset_lon_degrees / degrees_in_map) * self.map_height
        grid_y = (offset_lat_degrees / degrees_in_map) * self.map_height
        x, y = grid_x + ((self.map_height * self.map_scale) / 2), grid_y + ((self.map_height * self.map_scale) / 2)
        return int(x), int(y)

    # get lat and lon from x, y coordinates
    def get_lat_lon(self, x, y):
        grid_x, grid_y = x - (self.map_height / 2), -1 * (y - (self.map_height / 2))
        degrees_in_map = (self.map_height / 256.0) * (360.0 / pow(2, self.map_zoom))
        offset_x_degrees = (float(grid_x) / self.map_height) * degrees_in_map
        offset_y_degrees = (float(grid_y) / self.map_scale) * degrees_in_map
        return self.map_center[0] + offset_y_degrees, self.map_center[1] + offset_x_degrees

    # find center of qr code in frame
    def find_qr(self, frame):
        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (7, 7), 0)
            edged = cv2.Canny(blurred, 50, 150)

            contours, hierarchy = cv2.findContours(edged.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            hierarchy = hierarchy[0]
            z = zip(contours, hierarchy)
            corner_a, corner_b, corner_c = None, None, None

            for component in z:
                currentContour = component[0]
                currentHierarchy = component[1]
                peri = cv2.arcLength(currentContour, True)
                approx = cv2.approxPolyDP(currentContour, 0.01 * peri, True)

                # this could be a square
                if len(approx) >= 4 and len(approx) <= 6:
                    (x, y, w, h) = cv2.boundingRect(approx)
                    aspectRatio = w / float(h)

                    area = cv2.contourArea(currentContour)
                    hullArea = cv2.contourArea(cv2.convexHull(currentContour))
                    solidity = area / float(hullArea)

                    # it's a square!
                    if ((solidity > 0.9)
                        and (aspectRatio >= 0.9 and aspectRatio <= 1.1)):
                        child_count = 0

                        # find all of it's children
                        while currentHierarchy[2] > -1:
                            child_count += 1
                            currentHierarchy = z[currentHierarchy[2]][1]

                        # it looks like a QR corner
                        if child_count > 4:
                            # get the center
                            M = cv2.moments(approx)
                            cntr = [int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])]

                            if corner_a == None and corner_b == None and corner_c == None:
                                corner_a = cntr
                            elif corner_a != None and corner_b == None and corner_c == None:
                                corner_b = cntr
                            elif corner_a != None and corner_b != None and corner_c == None:
                                corner_c = cntr

                            # we have 3 corner points
                            if corner_a != None and corner_b != None and corner_c != None:
                                pts = np.array([corner_a, corner_b, corner_c], np.int32)
                                center_qr = None #TODO : check if necessary

                                # get length of sides of the triangle
                                AB = np.sqrt((corner_a[0] - corner_b[0]) * (corner_a[0] - corner_b[0]) + (corner_a[1] - corner_b[1]) * (corner_a[1] - corner_b[1]))
                                BC = np.sqrt((corner_b[0] - corner_c[0]) * (corner_b[0] - corner_c[0]) + (corner_b[1] - corner_c[1]) * (corner_b[1] - corner_c[1]))
                                CA = np.sqrt((corner_c[0] - corner_a[0]) * (corner_c[0] - corner_a[0]) + (corner_c[1] - corner_a[1]) * (corner_c[1] - corner_a[1]))

                                # find hypotenuse center
                                if BC > AB and BC > CA:
                                    center_qr = ((corner_b[0] + corner_c[0]) / 2, (corner_b[1] + corner_c[1]) / 2)
                                elif CA > AB and CA > BC:
                                    center_qr = ((corner_c[0] + corner_a[0]) / 2, (corner_c[1] + corner_a[1]) / 2)
                                elif AB > BC and AB > CA:
                                    center_qr = ((corner_a[0] + corner_b[0]) / 2, (corner_a[1] + corner_b[1]) / 2)

                                cv2.polylines(frame, [pts], True, (0, 255, 0), 4)
                                if center_qr != None:
                                    # draw a circle around the center
                                    cv2.circle(frame, center_qr, 6, (0, 0, 255), 4)

            cv2.imshow('Video', frame)
        except:
            pass

    # find simple red object in frame
    def find_red_object(self, frame):
        try:
            lower_red = np.array([17, 15, 100], dtype = "uint8")
            upper_red = np.array([50, 56, 200], dtype = "uint8")
            mask = cv2.inRange(frame, lower_red, upper_red)
            filtered = cv2.bitwise_and(frame, frame, mask = mask)

            gray = cv2.cvtColor(filtered, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (7, 7), 0)
            edged = cv2.Canny(blurred, 50, 150)
            contours, hierarchy = cv2.findContours(edged.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            cnt = contours[0]
            M = cv2.moments(cnt)

            cntr = (int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))
            # apply PID control
            errx =  self.distance(cntr, 0, 640)
            erry = -self.distance(cntr, 1, 360)
            print self.pid.compute_control_command(errx, erry)

            cv2.circle(frame, cntr, 6, (255, 255, 255), 4)
            cv2.imshow('Video', frame)
        except:
            cv2.imshow('Video', frame)

    def process_video(self):
        pygame.init()
        clock = pygame.time.Clock()
        running = True
        camera = cv2.VideoCapture('tcp://192.168.1.1:5555')
        while running:
            try:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                    elif event.type == pygame.KEYUP:
                        self.drone.hover()
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            self.drone.land()
                            running = False
                        # takeoff / land
                        elif event.key == pygame.K_RETURN:
                            self.drone.takeoff()
                        elif event.key == pygame.K_SPACE:
                            self.drone.land()
                        # reset
                        elif event.key == pygame.K_r:
                            self.drone.reset()
                        # forward / backward
                        elif event.key == pygame.K_w:
                            self.drone.move_up()
                        elif event.key == pygame.K_s:
                            self.drone.move_down()
                        # left / right
                        elif event.key == pygame.K_a:
                            self.drone.turn_left()
                        elif event.key == pygame.K_d:
                            self.drone.turn_right()
                        # trim
                        elif event.key == pygame.K_t:
                            self.drone.trim()
                        # up / down
                        elif event.key == pygame.K_UP:
                            self.drone.move_forward()
                        elif event.key == pygame.K_DOWN:
                            self.drone.move_backward()
                        # turn left / turn right
                        elif event.key == pygame.K_LEFT:
                            self.drone.move_left()
                        elif event.key == pygame.K_RIGHT:
                            self.drone.move_right()
                        # front camera
                        elif event.key == pygame.K_f:
                            self.drone.config('video:video_channel','2')
                        # bottom camera
                        elif event.key == pygame.K_b:
                            self.drone.config('video:video_channel','1')

                # grab extra frames to empty buffer and avoid lag
                camera.grab()
                camera.grab()
                _, frame = camera.read()
                #self.find_qr(frame)
                self.find_red_object(frame)
                # limit fps
                clock.tick(self.fps)
            except:
                pass

        print 'Shutting down...'
        camera.release()
        self.drone.halt()
        cv2.destroyAllWindows()
        if (self.use_gps): self.gps_thread._Thread__stop()
        self.data_thread._Thread__stop()
        print 'Finished'
        sys.exit()

    def process_gps(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect(('192.168.1.1', 4567))
        locations = []
        locations.append(self.map_center)
        prev_coords = None
        wp_color = (0, 255, 0)
        loc_color = (0, 0, 255)
        while True:
            try:
                # reread map to clear everything
                map = cv2.imread(self.map_name, 1)

                # draw all predefined waypoints
                prev_wp = None
                for wp in self.waypoints:
                    cv2.circle(map, wp, 2, wp_color, 2)
                    if (prev_wp): cv2.line(map, wp, prev_wp, wp_color, 1)
                    prev_wp = wp

                gps_string = sock.recv(1024).rstrip('\n')
                # lat, lon, alt, course, speed, satellites
                self.gps_data = gps_string.split()
                # update frequency of gps is 1Hz, but we force the process
                # to 5Hz so that the drone updates (pitch, roll) are faster
                current_coords = (self.gps_data[0], self.gps_data[1])

                # if we are moving and it's a new position
                if (float(self.gps_data[4]) > 1.0 and current_coords != prev_coords):
                    locations.append(current_coords)

                prev_coords = current_coords
                l = len(locations)

                # draw all visited locations
                prev_loc = None
                for i, loc in enumerate(locations):
                    coords = self.get_x_y(loc[0], loc[1])
                    if (i < l - 1):
                        cv2.circle(map, coords, 5, (0, 255, 255), 2)
                    elif (i == l - 1):
                        cv2.circle(map, coords, 1, loc_color, 1)
                    if (prev_loc): cv2.line(map, coords, prev_loc, loc_color, 1)
                    prev_loc = coords

                cv2.imshow('Map', map)
            except:
                pass

    def process_flight_data(self):
        txt_color = (255, 255, 255)
        txt_size = 0.7
        sky_color = (200, 120, 79)
        ground_color = (42, 112, 76)
        font = cv2.FONT_HERSHEY_SIMPLEX
        space = 25
        start = 10
        while True:
            try:
                # reread background to clear everything
                flight_data = cv2.imread(self.flight_data_bg, 1)
                bat = self.drone.navdata.get(0).get('battery')
                alt1 = self.drone.navdata.get(0).get('altitude')
                pitch = self.drone.navdata.get(0).get('theta') * 2
                roll = self.drone.navdata.get(0).get('phi') * 4
                yaw = self.drone.navdata.get(0).get('psi')

                cntr1 = np.array([(0, 0), (0, 160 + roll + pitch), (640, 160 - roll + pitch), (640, 0)])
                cntr2 = np.array([(0, 160 + roll + pitch), (0, 320), (640, 320), (640, 160 - roll + pitch)])
                cv2.fillPoly(flight_data, pts =[cntr1], color=sky_color)
                cv2.fillPoly(flight_data, pts =[cntr2], color=ground_color)
                cv2.line(flight_data, (300, 160), (340, 160), txt_color)
                cv2.putText(flight_data, 'Bat : ' + str(bat) + '%', (10, start + space), font, txt_size, txt_color, 1, 1, False)
                cv2.putText(flight_data, 'Alt : ' + str(alt1) + 'm', (10, start + (space * 2)), font, txt_size, txt_color, 1, 1, False)

                if (self.use_gps):
                    lat = self.gps_data[0]
                    lon = self.gps_data[1]
                    alt2 = self.gps_data[2]
                    course = self.gps_data[3]
                    speed = self.gps_data[4]
                    sats = self.gps_data[5]

                    cv2.putText(flight_data, 'Alt GPS : ' + str(alt2) + 'm', (10, start + (space * 3)), font, txt_size, txt_color, 1, 1, False)
                    cv2.putText(flight_data, 'Speed : ' + str(speed) + 'km/h', (10, start + (space * 4)), font, txt_size, txt_color, 1, 1, False)
                    cv2.putText(flight_data, 'Sats : ' + str(sats), (10, start + (space * 5)), font, txt_size, txt_color, 1, 1, False)
                    cv2.putText(flight_data, 'Lat : ' + str(lat), (10, start + (space * 6)), font, txt_size, txt_color, 1, 1, False)
                    cv2.putText(flight_data, 'Lon : ' + str(lon), (10, start + (space * 7)), font, txt_size, txt_color, 1, 1, False)
                    cv2.putText(flight_data, 'Course : ' + str(course) + 'deg', (10, start + (space * 8)), font, txt_size, txt_color, 1, 1, False)

                cv2.imshow('Flight Data', flight_data)
            except:
                pass

if __name__ == '__main__':
    gcs = GCS()
