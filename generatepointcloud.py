import cv2
from stereovision.blockmatchers import StereoSGBM, StereoBM
from stereovision.calibration import StereoCalibration
from stereovision.stereo_cameras import CalibratedPair
test_dir="test_images/"
number=32
left=test_dir+"left_"+str(number)+".png"
right=test_dir+"right_"+str(number)+".png"

image_pair = [cv2.imread(image) for image in [left, right]]
calib_folder="calibration/"
block_matcher = StereoBM()
camera_pair = CalibratedPair(None,
                            StereoCalibration(input_folder=calib_folder),
                            block_matcher)
rectified_pair = camera_pair.calibration.rectify(image_pair)

#block_matcher.compute(image_pair[0],image_pair[1]).astype(np.float32) / 16.0
#disparity = block_matcher.get_disparity(rectified_pair)
#points = block_matcher.get_3d(disparity,
#                                  self.calibration.disp_to_depth_mat)
#colors = cv2.cvtColor(pair[0], cv2.COLOR_BGR2RGB)
points = camera_pair.get_point_cloud(rectified_pair)
#points = points.filter_infinity()
#points.write_ply("output.png")
