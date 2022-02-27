#!/usr/bin/env python
# -*- coding: utf-8 -*-
from pygame.locals import *
import posenet
import argparse


from utils import CvFpsCalc

from gestures import *


import time
import sys
import cyberbee
import datetime
import os
import keyboard
import pygame
import cv2
import numpy
import av
import threading
import traceback
from simple_pid import PID
import tensorflow.compat.v1 as tf


import json
import socket
import pygame

from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import parse_qs
import cgi


from sys import stdout

import cv2
from PIL import Image
import threading
from socketserver import ThreadingMixIn
from io import StringIO, BytesIO
import time


# Initialize pygame for joystick support
pygame.display.init()
pygame.joystick.init()

controller = pygame.joystick.Joystick(0)
controller.init()

tf.disable_v2_behavior()


prev_flight_data = None
run_controller_thread = True
shutdown = False
# drone control inputs
drone_cc = 0  # rotation command
drone_ud = 0  # up/down command
drone_fb = 0  # forward/backward command

framerate = 30.0

filename = "temp"

start_hand_landing = False  # true when gesture recognized
ready_to_land = False  # true when size of body match the requested
# number of pixel for size of body in frame (distance control)
desiredHeight = 200
# number of pixel for size of body in frame for landing in hand  (distance control)
landing_height = 500
landing_timeout = 0
up_count = 0  # number of up gestures recognized
take_of_from_gesture_count = 5
last_up_time_mili = 0
gesture_take_off = False
last_locked_position = [0, 0]
last_body_height = 0
took_off = False
new_image_ready = False
gesture_start_control = False
stopped_lr = False
current_height, speed, battery, wifi_quality = 0, 0, 0, 0
lr_timeout = 0
time_out_LR = 500
json_to_send = json.loads(
    '{ "Height":-1, "Speed":-1, "Battery":-1, "Wifi_quality":-1}')
tracker_on = False
bbox = (0, 0, 0, 0)
face_detected_front = False
face_detected_back = False
enable_gesture_control = False


class CamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        global new_image_ready, new_frame
        if (self.path.endswith('.mjpg')):
            self.send_response(200)
            self.send_header(
                'Content-type', 'multipart/x-mixed-replace; boundary=--jpgboundary')
            self.end_headers()
            while not shutdown:
                if(new_image_ready):
                    imgRGB = cv2.cvtColor(new_frame, cv2.COLOR_BGR2RGB)
                    jpg = Image.fromarray(imgRGB)
                    tmpFile = BytesIO()
                    jpg.save(tmpFile, 'JPEG')
                    self.wfile.write("--jpgboundary".encode())
                    self.send_header('Content-type', 'image/jpeg')
                    self.send_header('Content-length',
                                     str(tmpFile.getbuffer().nbytes))
                    self.end_headers()
                    jpg.save(self.wfile, 'JPEG')
                    time.sleep(0.05)
        if self.path.endswith('.html'):
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write('<html><head></head><body>')
            self.wfile.write('<img src="http://localhost:8000/cam.mjpg"/>')
            self.wfile.write('</body></html>')
            return


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Handle requests in a separate thread."""


class GP(BaseHTTPRequestHandler):
    def _set_headers(self):
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

    def do_HEAD(self):
        self._set_headers()

    def do_GET(self):
        self._set_headers()
        print(self.path)
        print(parse_qs(self.path[2:]))
        string_to_send = json_to_send.encode(encoding='utf_8')
        self.wfile.write(string_to_send)

    def do_POST(self):
        self._set_headers()
        form = cgi.FieldStorage(
            fp=self.rfile,
            headers=self.headers,
            environ={'REQUEST_METHOD': 'POST'}
        )
        print(form.getvalue("foo"))
        print(form.getvalue("bin"))
        self.wfile.write(
            "<html><body><h1>POST Request Received!</h1></body></html>")


def Stream_Video():
    time.sleep(15)
    server = ThreadedHTTPServer(('', 8000), CamHandler)
    server.serve_forever()


def controller_thread():
    global drone

    global shutdown
    global took_off
    global ready_to_land
    global start_hand_landing
    global control_on
    global lr_timeout
    global stopped_lr
    global face_detected_front, face_detected_back
    # initialize previous drone control inputs
    control_on = True  # allows you to toggle control so that you can force landing

    global run_controller_thread
    print('start controller_thread()')
    try:
        time.sleep(5)
        while run_controller_thread:
            time.sleep(.1)
            # takeoff
            if keyboard.is_pressed('space'):
                drone.takeoff()
                took_off = True
            # land
            elif keyboard.is_pressed('l'):
                drone.land()


            elif keyboard.is_pressed('m'):
                start_hand_landing = True

            elif keyboard.is_pressed('q'):
                drone.counter_clockwise(40)
            elif keyboard.is_pressed('e'):
                drone.clockwise(40)
            elif keyboard.is_pressed('d'):
                drone.right(40)
            elif keyboard.is_pressed('a'):
                drone.left(40)
            elif keyboard.is_pressed('w'):
                drone.forward(40)
            elif keyboard.is_pressed('s'):
                drone.backward(40)
            elif keyboard.is_pressed('r'):
                drone.clockwise(0)
                drone.forward(0)
                drone.left(0)
            elif keyboard.is_pressed('t'):  # toggle controls
                control_on = not control_on
            elif keyboard.is_pressed('f'):  # toggle Face detected
                face_detected_front = True
            elif keyboard.is_pressed('b'):  # toggle Face detected
                face_detected_back = True
            elif keyboard.is_pressed('esc'):
                drone.land()
                break

            if(took_off):
                pygame.event.pump()
                roll = 90 * controller.get_axis(0)
                pitch = 90 * controller.get_axis(1)
                yaw = 120 * controller.get_axis(3)
                gaz = 120 * controller.get_axis(2)
                # print(controller.get_axis(6))

                # print("control_on is false" )
                drone.clockwise(yaw)
                drone.up(gaz)
                drone.backward(pitch)
                drone.right(roll)

    except KeyboardInterrupt as e:
        print(e)
    except Exception as e:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        traceback.print_exception(exc_type, exc_value, exc_traceback)
        print(e)
    finally:
        run_controller_thread = False


def URL_ResponceThread(server_class=HTTPServer, handler_class=GP, port=8088):
    time.sleep(15)
    server_address = ('', port)
    httpd = server_class(server_address, handler_class)
    print('Server running at localhost:8088...')
    httpd.serve_forever()


def handler(event, sender, data, **args):
    global prev_flight_data
    global current_height, speed, battery, wifi_quality
    global json_to_send
    global face_detected_front, face_detected_back

    drone = sender
    if event is drone.EVENT_FLIGHT_DATA:
        if prev_flight_data != str(data):
            mylist = str(data).split(" ")
            mylist = list(filter(None, mylist))
            current_height = int(mylist[1])+2
            speed = int(mylist[4])
            battery = int(mylist[7])
            wifi_quality = int(mylist[10])
            json_string = {
                "Height": current_height,
                "Speed": speed,
                "Battery": battery,
                "Wifi_quality": wifi_quality,
                "Face_Recognition_Front": face_detected_front,
                "Face_Recognition_Back": face_detected_back
            }

            json_to_send = json.dumps(json_string)
            print(json_to_send)
            prev_flight_data = str(data)
    else:
        print('event="%s" data=%s' % (event.getname(), str(data)))


def ShowVideos(image):
    global new_frame, new_image_read
    new_image_read = True
    cv2.imshow('posenet', image)
    new_frame = image

    cv2.waitKey(1)


def main():
    global drone
    global shutdown
    global battery_status
    global new_frame
    global new_image_ready

    battery_status = -1
    drone_class = cyberbee.CyberBee()
    drone = drone_class.GetDrone()

    drone.connect()
    drone.wait_for_connection(2.0)
    drone.start_video()

    drone.subscribe(drone.EVENT_FLIGHT_DATA, handler)

    # video = cv2.VideoWriter('test_out.avi', -1, 1, (320, 240))
    # drone.subscribe(drone.EVENT_VIDEO_FRAME,handler)
    print("Start Running")

    try:
        # threading.Thread(target=recv_thread).start()
        threading.Thread(target=controller_thread).start()

        threading.Thread(target=URL_ResponceThread).start()
        threading.Thread(target=Stream_Video).start()

        container = av.open(drone.get_video_stream())
        frame_count = 0
        while not shutdown:
            for frame in container.decode(video=0):
                frame_count = frame_count + 1

                # skip first 100 frames
                if frame_count < 200:
                    continue

                im = numpy.array(frame.to_image())
                image = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)

                # cv2.imshow('after control', overlay_image)

                ShowVideos(image)

    except KeyboardInterrupt as e:
        print(e)

    except Exception as e:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        traceback.print_exception(exc_type, exc_value, exc_traceback)

        print(e)

    cv2.destroyAllWindows()

    drone.quit()
    exit(1)


if __name__ == '__main__':

    main()
