#!/usr/bin/env python
# -*- coding: utf-8 -*-
from pygame.locals import *
import posenet
import argparse
import configargparse


from utils import CvFpsCalc

from gestures import *


import time
import sys
import cyberbee

import keyboard
import pygame
import cv2
import numpy
import av
import threading
import traceback
from simple_pid import PID
import tensorflow.compat.v1 as tf
tf.disable_v2_behavior()


parser = argparse.ArgumentParser()
parser.add_argument('--model', type=int, default=101)
parser.add_argument('--cam_id', type=int, default=0)
# defaults need to be changed?
parser.add_argument('--cam_width', type=int, default=1280)
parser.add_argument('--cam_height', type=int, default=720)
parser.add_argument('--scale_factor', type=float, default=0.7125)
parser.add_argument('--file', type=str, default=None,
                    help="Optionally use a video file instead of a live camera")
args = parser.parse_args()


prev_flight_data = None
run_controller_thread = True
shutdown = False
# drone control inputs
drone_cc = 0  # rotation command
drone_ud = 0  # up/down command
drone_fb = 0  # forward/backward command
out_video_save = cv2.VideoWriter('outpy.avi', cv2.VideoWriter_fourcc(
    'M', 'J', 'P', 'G'), 10, (1280, 720))
start_hand_landing = False  # true when gesture recognized
ready_to_land = False  # true when size of body match the requested
# number of pixel for size of body in frame (distance control)
desiredHeight = 70
# number of pixel for size of body in frame for landing in hand  (distance control)
landing_height = 130
landing_timeout = 0
up_count = 0  # number of up gestures recognized
take_of_from_gesture_count = 2
last_up_time_mili = 0
gesture_take_off = False
last_locked_position = [0, 0]
last_body_height = 0
took_off = False


def controller_thread():
    global drone
    global drone_cc
    global drone_ud
    global drone_fb
    global shutdown
    global gesture_take_off
    global took_off

    # initialize previous drone control inputs
    control_on = True  # allows you to toggle control so that you can force landing
    pdrone_cc = -111
    pdrone_ud = -111
    pdrone_fb = -111

    global run_controller_thread
    print('start controller_thread()')
    try:
        while run_controller_thread:
            time.sleep(.02)
            # takeoff
            if keyboard.is_pressed('space'):
                drone.takeoff()
                took_off = True
            # land
            elif keyboard.is_pressed('l'):
                drone.land()
                control_on = False  # disable control
                shutdown = True
                took_off = False

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
            elif keyboard.is_pressed('esc'):
                drone.land()
                break

            # set commands based on PID output
            if control_on and (pdrone_cc != drone_cc):
                if drone_cc < 0:
                    drone.clockwise(min([20, (int(drone_cc)*-1)]))
                else:
                    drone.counter_clockwise(min([20, int(drone_cc)]))
                pdrone_cc = drone_cc
            if control_on and (pdrone_ud != drone_ud):
                if drone_ud < 0:
                    # easily moving downwards requires control output to be magnified
                    drone.down(min([100, (int(drone_ud)*-2)]))
                else:
                    drone.up(min([200, int(drone_ud)]))
                pdrone_ud = drone_ud
            if control_on and (pdrone_fb != drone_fb):
                if drone_fb < 0:
                    # easily moving backwards requires control output to be magnified
                    drone.backward(int(drone_fb)*-1)
                else:
                    drone.forward(int(drone_fb))
                pdrone_fb = drone_fb

            if(gesture_take_off):
                gesture_take_off = False
                drone.takeoff()
                took_off = True
                time.sleep(0.5)
                drone.backward(40)
                time.sleep(0.5)
                drone.down(40)
                
            if(ready_to_land ):
                drone.land()
                control_on = False  # disable control
                shutdown = True
                run_controller_thread = False
                took_off = False
            
                

    except KeyboardInterrupt as e:
        print(e)
    except Exception as e:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        traceback.print_exception(exc_type, exc_value, exc_traceback)
        print(e)
    finally:
        run_controller_thread = False


def handler(event, sender, data, **args):
    global prev_flight_data
    drone = sender
    if event is drone.EVENT_FLIGHT_DATA:
        if prev_flight_data != str(data):
            print(data)
            prev_flight_data = str(data)
    else:
        print('event="%s" data=%s' % (event.getname(), str(data)))


def get_args():
    print('## Reading configuration ##')
    parser = configargparse.ArgParser(default_config_files=['config.txt'])

    parser.add('-c', '--my-config', required=False,
               is_config_file=True, help='config file path')
    parser.add("--device", type=int)
    parser.add("--width", help='cap width', type=int)
    parser.add("--height", help='cap height', type=int)
    parser.add("--is_keyboard",
               help='To use Keyboard control by default', type=bool)
    parser.add('--use_static_image_mode', action='store_true',
               help='True if running on photos')
    parser.add("--min_detection_confidence",
               help='min_detection_confidence',
               type=float)
    parser.add("--min_tracking_confidence",
               help='min_tracking_confidence',
               type=float)
    parser.add("--buffer_len",
               help='Length of gesture buffer',
               type=int)

    args = parser.parse_args()

    return args


def gesture_control(gesture_buffer):
    global drone_cc, drone_ud, drone_fb, control_on, shutdown, run_controller_thread
    global desiredHeight, landing_timeout, start_hand_landing
    global up_count  # safe from false gesture
    global last_up_time_mili
    global gesture_take_off

    gesture_id = gesture_buffer.get_gesture()
    print("GESTURE", gesture_id)

    #     # if gesture_id == 0:  # Forward

    #     if gesture_id == 1:  # STOP
    #         print("stop")
    #         drone_cc = drone_ud= drone_fb=0
    #     # if gesture_id == 5:  # Back

    if (gesture_id == 2 or gesture_id == 0):  # UP
        print("-------got up ------------")
        if(1000 < round(time.time() * 1000)-last_up_time_mili):
            print("-------too slow ------------")
            up_count = 0
        last_up_time_mili = round(time.time() * 1000)
        up_count += 1
        if(take_of_from_gesture_count < up_count):
            print("-------taking of ------------")
            gesture_take_off = True

    #     # elif gesture_id == 4:  # DOWN

    if (gesture_id == 3 ) :  # LAND
        desiredHeight = landing_height
        landing_timeout = round(time.time() * 1000)
        start_hand_landing = True

    #     # elif gesture_id == 6:  # LEFT

    #     # elif gesture_id == 7:  # RIGHT

    # elif gesture_id == -1:
    #         print("stop - error recognition")
    #         drone_cc = drone_ud= drone_fb=0


def CalcWidthHight(keypoint_coords):
    global ready_to_land
    global last_body_height
    leftSholy_y = int(keypoint_coords[0, 5, 0])
    rightSholy_y = int(keypoint_coords[0, 6, 0])
    leftHipy_y = int(keypoint_coords[0, 11, 0])
    rightHipy_y = int(keypoint_coords[0, 12, 0])

    # print("leftSholy",leftSholy,"rightSholy",rightSholy,"leftHipy",leftHipy,"rightHipy",rightHipy)
    # technically arbitrary
    meanHeight = int(((rightHipy_y - rightSholy_y) +
                      (leftHipy_y - leftSholy_y))/2)
    leftSholy_x = int(keypoint_coords[0, 5, 1])
    rightSholy_x = int(keypoint_coords[0, 6, 1])
    leftHipy_x = int(keypoint_coords[0, 11, 1])
    rightHipy_x = int(keypoint_coords[0, 12, 1])

    # technically arbitrary
    meanWidth = int(((rightHipy_x - leftHipy_x) +
                     (rightSholy_x - leftSholy_x))/2)
    # print("x: ",keypoint_coords[0,:,1])
    # print("y: ",keypoint_coords[0,:,0])
    print("meanHeight", meanHeight)
    # print("meanWidth", meanWidth)

    if(start_hand_landing):
        if(meanHeight >= landing_height):
            ready_to_land = True

    center_body_x = int(leftSholy_x+(meanWidth/2))
    center_body_y = int(leftSholy_y+(meanHeight/2))
    last_body_height = meanHeight
    errorFB = meanHeight - desiredHeight
    return center_body_x, center_body_y, errorFB


def CalculateControl(keypoint_scores, keypoint_coords, pid_cc, pid_ud, pid_fb, overlay_image):
    global drone_cc
    global drone_ud
    global drone_fb
    global last_locked_position

    centerx = int(overlay_image.shape[1]/2)
    centery = int(overlay_image.shape[0]/2)

    ctrl_out_cc = 0
    ctrl_out_ud = 0

    ctrl_out_fb = 0

    errorFB = 0
    # overlay_image = cv2.putText(overlay_image, str(nosey), (120,50), cv2.FONT_HERSHEY_SIMPLEX ,   1, (55,255,45), 2)
    errorx = 0
    errory = 0

    if keypoint_scores[0, 5] > .03 and keypoint_scores[0, 6] > .03 and keypoint_scores[0, 11] > .03 and keypoint_scores[0, 12] > .03:
        center_body_x, center_body_y, errorFB = CalcWidthHight(keypoint_coords)

        last_locked_position = [center_body_x, center_body_y]

        overlay_image = cv2.line(overlay_image, (centerx, centery - 10),
                                 (center_body_x, center_body_y), (255, 255, 0), 2)
        errorx = center_body_x - centerx
        errory = center_body_y - centery - 10
        if abs(errorx) > 2:
            ctrl_out_cc = pid_cc(errorx)
            drone_cc = ctrl_out_cc
        else:
            drone_cc = 0
        if abs(errory) > 4:
            ctrl_out_ud = pid_ud(errory)
            drone_ud = ctrl_out_ud
        else:
            drone_ud = 0

            # error can be within +/- 15 without caring
        if abs(errorFB) > 5:
            ctrl_out_fb = pid_fb(errorFB)
            drone_fb = ctrl_out_fb
        else:
            drone_fb = 0

    # out_img = cv2.putText(out_img, str(keypoint_scores[ii,kpoint]), (50,50), cv2.FONT_HERSHEY_SIMPLEX ,   1, (255,255,45), 2)
    else:
        drone_cc = 0
        drone_ud = 0
        drone_fb = 0
        pid_fb.reset()
        pid_cc.reset()
        pid_ud.reset()
        print("-----------coudn't find body -------------")
        # reset pid
        if(took_off):
            if(desiredHeight*2 < last_body_height):
                drone_fb = -40
            elif (desiredHeight/2 > last_body_height):
                drone_fb = 40
            if(240 < last_locked_position[0]):
                drone_cc = -30
            elif(80 > last_locked_position[0]):
                drone_cc = 30
            if(180 < last_locked_position[1]):
                drone_ud = -40
            elif(60 > last_locked_position[1]):
                drone_ud = 40
        # don't let the hips lie
        # if keypoint_scores[0,11] < .04 and keypoint_scores[0,12] < .04:
        #     drone_fb = -20
        #     drone_ud = -20

        print("drone_cc ", drone_cc, "drone_ud ",
              drone_ud, "drone_fb ", drone_fb)


def ShowVideos(frame, overlay_image, gesture_detector, number, mode, video):
    # overlay_image = cv2.putText(overlay_image, str(ctrl_out_fb), (30,110), cv2.FONT_HERSHEY_SIMPLEX ,   1, (55,255,45), 2)
    # overlay_image = cv2.putText(overlay_image, str(ctrl_out_ud), (30,30), cv2.FONT_HERSHEY_SIMPLEX ,   1, (55,255,45), 2)
    # overlay_image = cv2.putText(overlay_image, str(errory), (30,70), cv2.FONT_HERSHEY_SIMPLEX ,   1, (55,255,45), 2)
    im = numpy.array(frame.to_image())
    im = cv2.resize(im, (1280, 720))  # resize frame
    image = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
    cv2.imshow('posenet', overlay_image)
    # cv2.imshow('Input video', image)

    debug_image, gesture_id = gesture_detector.recognize(
        image, number, mode)
    gesture_buffer.add_gesture(gesture_id)
    gesture_control(gesture_buffer)
    cv2.imshow('Tello Gesture Recognition', debug_image)

    out_video_save.write(image)
    video.write(overlay_image)
    # cv2.imshow('Original', image)
    # cv2.imshow('Canny', cv2.Canny(image, 100, 200))

    cv2.waitKey(1)


def main():
    global drone

    global shutdown
    global gesture_buffer
    global gesture_id
    global battery_status
    global drone_cc
    global drone_ud
    global drone_fb
    global last_locked_position
    args_gest = get_args()
    gesture_detector = GestureRecognition(args_gest.use_static_image_mode, args_gest.min_detection_confidence,
                                          args_gest.min_tracking_confidence)
    gesture_buffer = GestureBuffer(buffer_len=args_gest.buffer_len)
    # FPS Measurement
    cv_fps_calc = CvFpsCalc(buffer_len=10)

    mode = 0
    number = -1
    battery_status = -1
    drone_class = cyberbee.CyberBee()
    drone = drone_class.GetDrone()

    drone.connect()
    drone.wait_for_connection(2.0)
    drone.start_video()

    drone.subscribe(drone.EVENT_FLIGHT_DATA, handler)
    pid_cc = PID(0.40, 0.12, 0.2, setpoint=0, output_limits=(-100, 100))
    pid_ud = PID(0.3, 0.05, 0.15, setpoint=0, output_limits=(-80, 80))
    pid_fb = PID(0.4, 0.10, 0.25, setpoint=0, output_limits=(-50, 50))

    video = cv2.VideoWriter('test_out.mp4', -1, 1, (320, 240))
    # drone.subscribe(drone.EVENT_VIDEO_FRAME,handler)
    print("Start Running")
    with tf.Session() as sess:
        model_cfg, model_outputs = posenet.load_model(args.model, sess)
        output_stride = model_cfg['output_stride']

        try:
            # threading.Thread(target=recv_thread).start()
            threading.Thread(target=controller_thread).start()
            container = av.open(drone.get_video_stream())
            frame_count = 0
            while not shutdown:
                for frame in container.decode(video=0):
                    frame_count = frame_count + 1
                    # skip first 300 frames
                    if frame_count < 100:
                        continue
                    if frame_count % 3 == 0:
                        im = numpy.array(frame.to_image())
                        im = cv2.resize(im, (320, 240))  # resize frame
                        image = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
                        # image = cv2.cvtColor(numpy.array(frame.to_image()), cv2.COLOR_RGB2BGR)
                        input_image, display_image, output_scale = posenet.process_input(
                            image, scale_factor=args.scale_factor, output_stride=output_stride)

                        heatmaps_result, offsets_result, displacement_fwd_result, displacement_bwd_result = sess.run(
                            model_outputs,
                            feed_dict={'image:0': input_image}
                        )

                        pose_scores, keypoint_scores, keypoint_coords = posenet.decode_multi.decode_multiple_poses(
                            heatmaps_result.squeeze(axis=0),
                            offsets_result.squeeze(axis=0),
                            displacement_fwd_result.squeeze(axis=0),
                            displacement_bwd_result.squeeze(axis=0),
                            output_stride=output_stride,
                            max_pose_detections=10,
                            min_pose_score=0.15)

                        keypoint_coords *= output_scale

                        # TODO this isn't particularly fast, use GL for drawing and display someday...
                        overlay_image = posenet.draw_skel_and_kp(
                            display_image, pose_scores, keypoint_scores, keypoint_coords,
                            min_pose_score=0.15, min_part_score=0.1)
                        

                        centerx = int(overlay_image.shape[1]/2)
                        centery = int(overlay_image.shape[0]/2)

                        ctrl_out_cc = 0
                        ctrl_out_ud = 0

                        ctrl_out_fb = 0

                        errorFB = 0
                        # overlay_image = cv2.putText(overlay_image, str(nosey), (120,50), cv2.FONT_HERSHEY_SIMPLEX ,   1, (55,255,45), 2)
                        errorx = 0
                        errory = 0

                        if keypoint_scores[0, 5] > .03 and keypoint_scores[0, 6] > .03 and keypoint_scores[0, 11] > .03 and keypoint_scores[0, 12] > .03:
                            center_body_x, center_body_y, errorFB = CalcWidthHight(keypoint_coords)

                            last_locked_position = [center_body_x, center_body_y]

                            overlay_image = cv2.line(overlay_image, (centerx, centery - 10),
                                                    (center_body_x, center_body_y), (255, 255, 0), 2)
                            errorx = center_body_x - centerx
                            errory = center_body_y - centery - 10
                            if abs(errorx) > 2:
                                ctrl_out_cc = pid_cc(errorx)
                                drone_cc = ctrl_out_cc
                            else:
                                drone_cc = 0
                            if abs(errory) > 4:
                                ctrl_out_ud = pid_ud(errory)
                                drone_ud = ctrl_out_ud
                            else:
                                drone_ud = 0

                                # error can be within +/- 15 without caring
                            if abs(errorFB) > 5:
                                ctrl_out_fb = pid_fb(errorFB)
                                drone_fb = ctrl_out_fb
                            else:
                                drone_fb = 0

                        # out_img = cv2.putText(out_img, str(keypoint_scores[ii,kpoint]), (50,50), cv2.FONT_HERSHEY_SIMPLEX ,   1, (255,255,45), 2)
                        else:
                            drone_cc = 0
                            drone_ud = 0
                            drone_fb = 0
                            pid_fb.reset()
                            pid_cc.reset()
                            pid_ud.reset()
                            print("-----------coudn't find body -------------")
                            # reset pid
                            if(took_off):
                                if(desiredHeight*2 < last_body_height):
                                    drone_fb = -40
                                elif (desiredHeight/2 > last_body_height):
                                    drone_fb = 40
                                if(240 < last_locked_position[0]):
                                    drone_cc = -30
                                elif(80 > last_locked_position[0]):
                                    drone_cc = 30
                                if(180 < last_locked_position[1]):
                                    drone_ud = -40
                                elif(60 > last_locked_position[1]):
                                    drone_ud = 40
                            # don't let the hips lie
                            # if keypoint_scores[0,11] < .04 and keypoint_scores[0,12] < .04:
                            #     drone_fb = -20
                            #     drone_ud = -20

                            print("drone_cc ", drone_cc, "drone_ud ",
                                drone_ud, "drone_fb ", drone_fb)
                        # CalculateControl(
                        #     keypoint_scores, keypoint_coords, pid_cc, pid_ud, pid_fb, overlay_image)
                        ShowVideos(frame, overlay_image,
                                   gesture_detector, number, mode, video)

        except KeyboardInterrupt as e:
            print(e)
        except Exception as e:
            exc_type, exc_value, exc_traceback = sys.exc_info()
            traceback.print_exception(exc_type, exc_value, exc_traceback)
            print(e)

    cv2.destroyAllWindows()
    video.release()
    drone.quit()
    exit(1)


if __name__ == '__main__':
    main()
