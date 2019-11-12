from stereovision.blockmatchers import StereoBM, StereoSGBM
from stereovision.stereo_cameras import CalibratedPair
from stereovision.ui_utils import STEREO_BM_FLAG
from stereovision.calibration import *
import cv2
rows=7
columns=7
square_size=1
image_size=(960,540)
calib_dir="calib_images/"
out_dir="calibration/"
skip=[15,17,19]
calibrator = StereoCalibrator(rows, columns, square_size, image_size)
for i in range(30):
    try:
        print(i+1)
        if (i+1) not in skip:
            leftimg=cv2.imread("{}left_{}.png".format(calib_dir,i+1))
            rightimg=cv2.imread("{}right_{}.png".format(calib_dir,i+1))
            temp=cv2.cvtColor(leftimg, cv2.COLOR_BGR2GRAY)
            calibrator.add_corners((leftimg,rightimg),show_results=False)
    except:
        print('No chessboard found')
calibration = calibrator.calibrate_cameras()
avg_error = calibrator.check_calibration(calibration)
print(avg_error)
calibration.export(out_dir)
