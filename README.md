# ARDrone Python ground control station

## About
Pygcs is a ground control station for the Parrot ARDrone 2.0. It uses a heavily modified version
of python-ardrone (https://github.com/venthur/python-ardrone) to communicate with the drone.
There is support for gps through an Arduino Pro Mini which is connected to the USB port of the
ARDrone with an FTDI device. The gps data is mapped on a map wich can be downloaded from google maps.

The video stream is read from a tcp connection with the ARDrone and captured in OpenCV. This allows for
a number of image processing algorithms and can be used to track colored objects or QR codes. The ultimate
goal is to implement a VSLAM solution.

There is also support for autonomous flight. The Arduino can send commands to the ARDrone through sockets. For
this to work, arduinoProxy.c must be compiled, uploaded and started on the ARDrone.

Here is a screenshot :

![alt tag](https://raw.github.com/ppossemiers/pygcs/master/screenshot.png)
