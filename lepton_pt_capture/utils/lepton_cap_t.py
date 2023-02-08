
# !/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
from datetime import datetime
import numpy as np

ir_camera_index = 2


def raw_to_8bit(data):
    cv2.normalize(data, data, 0, 65535, cv2.NORM_MINMAX)
    np.right_shift(data, 8, data)
    return cv2.cvtColor(np.uint8(data), cv2.COLOR_GRAY2RGB)


def main():
    # use V4l2 pipe otherwise CV2 returns 8bit image
    cap = cv2.VideoCapture(ir_camera_index , cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('Y', '1', '6', ' '))
    # cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('B', 'G', 'R', '3'))
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    # force resolution otherwise weirdness happens (extra pixels)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 120)

    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    while True:
        # Capture frame-by-frame
        code, frame = cap.read()
        # if frame is read correctly ret is True
        if not code:
            print("Can't receive frame (stream end?). Exiting ...")
            break

        # Save frame
        # cv2.imwrite('irData/LR_%s.png' % datetime.now().strftime('%H_%M_%S_%f')[:-3], frame)
        # Display the resulting frame
        frame = cv2.resize(frame[:, :], (640, 480))
        frame = raw_to_8bit(frame)


        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()