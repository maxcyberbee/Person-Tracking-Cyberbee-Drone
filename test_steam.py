import imagezmq
import cv2
import time
import numpy as np

imageHub = imagezmq.ImageHub(open_port='tcp://127.0.0.1:5000')

lastActive = {}

while True:
    # receive the device name and decoded image
    (name, frame) = imageHub.recv_image()

    # update the frame at ~50fps
    cv2.imshow("Frame", frame.astype(np.uint8))
    cv2.waitKey(20) 

    # send back confirmation message
    imageHub.send_reply(b'OK')

    if name not in lastActive.keys():
        print("[INFO] receiving data from {}...".format("test"))
    lastActive[name] = time.time()
