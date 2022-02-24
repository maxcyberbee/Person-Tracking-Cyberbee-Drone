
import configargparse
import datetime
import os
import cv2
import numpy

landing_height = 130
last_locked_position = [0, 0]
ready_to_land = False


def CalcWidthHight(keypoint_coords, start_hand_landing, desiredHeight):

    global last_body_height, ready_to_land
    bbox = (0,0,0,0)

    center_body_x, center_body_y, meanHeight,bbox = find_body_xy(keypoint_coords)

    if(start_hand_landing):
        if(meanHeight >= landing_height):
            ready_to_land = True

    last_body_height = meanHeight
    errorFB = meanHeight - desiredHeight
    return center_body_x, center_body_y, errorFB, ready_to_land,bbox


def CalculateControl(control_on, keypoint_scores, keypoint_coords, pid_cc, pid_ud, pid_fb, overlay_image, start_hand_landing, desiredHeight):
    global last_locked_position
    global ready_to_land
    #print("------!!! calculate control!!! ----------------")
    centerx = int(overlay_image.shape[1]/2)
    centery = int(overlay_image.shape[0]/2)
    ctrl_out_cc = 0
    ctrl_out_ud = 0
    ctrl_out_fb = 0
    errorFB = 0
    errorx = 0
    errory = 0
    bbox = (0,0,0,0)

    if control_on and (keypoint_scores[0, 5] > .1 and keypoint_scores[0, 6] > .1 and keypoint_scores[0, 11] > .1 and keypoint_scores[0, 12] > .1):

        center_body_x, center_body_y, errorFB, ready_to_land,bbox = CalcWidthHight(
            keypoint_coords, start_hand_landing, desiredHeight)

        last_locked_position = [center_body_x, center_body_y]

        # overlay_image = cv2.line(overlay_image, (centerx, centery - 10),
        #                          (center_body_x, center_body_y), (255, 255, 0), 2)
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

    return drone_cc, drone_ud, drone_fb, control_on, last_locked_position, ready_to_land,bbox


def find_body_xy(keypoint_coords):
    # find body y coords
    leftSholy_y = int(keypoint_coords[0, 5, 0])
    rightSholy_y = int(keypoint_coords[0, 6, 0])
    leftHipy_y = int(keypoint_coords[0, 11, 0])
    rightHipy_y = int(keypoint_coords[0, 12, 0])

    meanHeight = int(((rightHipy_y - rightSholy_y) +
                      (leftHipy_y - leftSholy_y))/2)

    # find body x coords
    leftSholy_x = int(keypoint_coords[0, 5, 1])
    rightSholy_x = int(keypoint_coords[0, 6, 1])
    leftHipy_x = int(keypoint_coords[0, 11, 1])
    rightHipy_x = int(keypoint_coords[0, 12, 1])

    meanWidth = int(((rightHipy_x - leftHipy_x) +
                     (rightSholy_x - leftSholy_x))/2)

    center_body_x = int(leftSholy_x+(meanWidth/2))
    center_body_y = int(leftSholy_y+(meanHeight/2))
    body_width = meanWidth*2*3
    body_height = meanHeight *2*3
    bbox = (leftSholy_x*3,leftSholy_y*3,body_width,body_height)
    return center_body_x*3, center_body_y*3, meanHeight*3,bbox


def getAviNameWithDate(nameIn="output.avi"):
    """Needs a file ending on .avi, inserts _<date> before .avi.

    If file exists, it appends a additional _number after the <date>
    ensuring filename uniqueness at this time."""
    if not nameIn.endswith(".avi"):
        raise ValueError("filename must end on .avi")

    filename = nameIn.replace(".avi", "_{0}.avi").format(
        datetime.datetime.now().strftime("%Y-%m-%d"))

    if os.path.isfile(filename):             # if already exists
        # modify pattern to include a number
        fn2 = filename[0:-4]+'_{0}.avi'
        count = 1
        # increase number until file not exists
        while os.path.isfile(fn2.format(count)):
            count += 1
        # return file with number in it
        return fn2.format(count)

    else:   # filename ok, return it
        return filename


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
    parser.add_argument('--scale_factor', type=float, default=0.7125)
    parser.add_argument('--model', type=int, default=101)
    args = parser.parse_args()
    return args

#parser = argparse.ArgumentParser()
#
# parser.add_argument('--cam_id', type=int, default=0)
# # defaults need to be changed?
# parser.add_argument('--cam_width', type=int, default=1280)
# parser.add_argument('--cam_height', type=int, default=720)
# parser.add_argument('--scale_factor', type=float, default=0.7125)
# parser.add_argument('--file', type=str, default=None,
#                     help="Optionally use a video file instead of a live camera")
# args = parser.parse_args()
