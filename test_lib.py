import libardrone
import pygame
import sys

# Ground Control Center
class GCC:
    def __init__(self):
        self.process_video()

    def process_video(self):
        pygame.init()
        drone = libardrone.ARDrone()
        running = True

        while running:
            try:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                    elif event.type == pygame.KEYUP:
                        drone.hover()
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            #drone.reset()
                            running = False
                        # takeoff / land
                        elif event.key == pygame.K_RETURN:
                            drone.takeoff()
                        elif event.key == pygame.K_SPACE:
                            drone.land()
                        # emergency
                        elif event.key == pygame.K_BACKSPACE:
                            print 'reset'
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
                            drone.move_up()
                        elif event.key == pygame.K_DOWN:
                            drone.move_down()
                        # turn left / turn right
                        elif event.key == pygame.K_LEFT:
                            drone.turn_left()
                        elif event.key == pygame.K_RIGHT:
                            drone.turn_right()

            except:
                pass

        print 'Shutting down...'
        drone.halt()
        print 'Finished'
        sys.exit()

if __name__ == '__main__':
    gcc = GCC()
