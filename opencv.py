import libardrone
import cv2
import pygame
import sys

# https://cranklin.wordpress.com/2014/11/14/artificial-intelligence-applied-to-your-drone/
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
        self.phi_1 = 0
        self.gaz_1 = 0

    def compute_control_command(self, errx, erry):
        # roll angle
        phi = self.Kpx * errx + self.Kix * (errx + self.errx_1) + self.Kdx * (errx - self.errx_1)
        # vertical speed
        gaz = self.Kpy * erry + self.Kiy * (erry + self.erry_1) + self.Kdy * (erry - self.erry_1)
        # remember values
        self.errx_1 = errx
        self.erry_1 = erry
        self.phi_1 = phi
        self.gaz_1 = gaz

        return (phi, gaz)

class UserCode:
    def __init__(self, use_camera=True):
        self.pid = PID()
        self.use_camera = use_camera
        #self.video  = cv2.VideoWriter('video.avi' + str(datetime.now()), -1, 25, (320, 240))

    def distance(self, ctr, dim, siz):
        siz = (siz / 2) * 1.0
        return (ctr[dim] - siz) / siz

    def process_frame(self, frame):
        #video.write(frame)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)
        edged = cv2.Canny(blurred, 50, 150)

        (cnts, _) = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for c in cnts:
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.01 * peri, True)

            if len(approx) >= 4 and len(approx) <= 6:
                (x, y, w, h) = cv2.boundingRect(approx)
                aspectRatio = w / float(h)

                area = cv2.contourArea(c)
                hullArea = cv2.contourArea(cv2.convexHull(c))
                solidity = area / float(hullArea)

                keepDims = w > 25 and h > 25
                keepSolidity = solidity > 0.9
                keepAspectRatio = aspectRatio >= 0.8 and aspectRatio <= 1.2

                if keepDims and keepSolidity and keepAspectRatio:
                    cv2.drawContours(frame, [approx], -1, (0, 0, 255), 4)

                    # apply PID control
                    M = cv2.moments(approx)
                    ctr = (int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))
                    errx =  self.distance(ctr, 0, 320)
                    erry = -self.distance(ctr, 1, 240)
                    print self.pid.compute_control_command(errx, erry)

        cv2.imshow('Feed', frame)

    def run(self):
        alt_limit = 3
        bat = 0
        alt = 0

        pygame.init()
        clock = pygame.time.Clock()
        drone = libardrone.ARDrone()
        running = True

        if self.use_camera:
            camera = cv2.VideoCapture('tcp://192.168.1.1:5555')

        cv2.startWindowThread()
        window = cv2.namedWindow('Feed')

        while running:
            try:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                    elif event.type == pygame.KEYUP:
                        drone.hover()
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            drone.reset()
                            running = False
                        # takeoff / land
                        elif event.key == pygame.K_RETURN:
                            drone.takeoff()
                        elif event.key == pygame.K_SPACE:
                            drone.land()
                        # emergency
                        elif event.key == pygame.K_BACKSPACE:
                            drone.reset()
                        # forward / backward
                        elif event.key == pygame.K_w:
                            drone.move_forward()
                        elif event.key == pygame.K_s:
                            drone.move_backward()
                        # left / right
                        elif event.key == pygame.K_a:
                            drone.move_left()
                        elif event.key == pygame.K_d:
                            drone.move_right()
                        # trim
                        elif event.key == pygame.K_t:
                            drone.trim()
                        # up / down
                        elif event.key == pygame.K_UP:
                            if alt < alt_limit:
                                drone.move_up()
                        elif event.key == pygame.K_DOWN:
                            drone.move_down()
                        # turn left / turn right
                        elif event.key == pygame.K_LEFT:
                            drone.turn_left()
                        elif event.key == pygame.K_RIGHT:
                            drone.turn_right()

                alt = round(drone.navdata.get(0).get('altitude') / 1000.0, 2)
                if alt > alt_limit:
                    drone.move_down()
                    drone.hover()
                bat = drone.navdata.get(0).get('battery')
                if bat < 5:
                    drone.land()

                if self.use_camera:
                    ret, frame = camera.read()

                    cv2.putText(frame, 'Bat : ' + str(bat) + '%', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, 2, False)
                    cv2.putText(frame, 'Alt : ' + str(alt) + 'm', (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, 2, False)

                    self.process_frame(frame)

                    clock.tick(35)
            except:
                pass

        print 'Shutting down...'
        drone.halt()
        camera.release()
        cv2.destroyAllWindows()
        print 'Finished'
        sys.exit()

if __name__ == '__main__':
    uc = UserCode()
    uc.run()
