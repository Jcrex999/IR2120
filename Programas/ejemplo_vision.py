from pyniryo import *

observation_pose = PoseObject(
    x=0.18, y=0.0, z=0.35,
    roll=0.0, pitch=1.57, yaw=-0.2,
)

# Connecting to robot
robot = NiryoRobot("169.254.200.200")
robot.calibrate_auto()

# Getting calibration param
mtx, dist = robot.get_camera_intrinsics()
# Moving to observation pose
robot.move_pose(observation_pose)

blue_min_hsv = [90, 85, 70]
blue_max_hsv = [125, 255, 255]

while "User do not press Escape neither Q":
    # Getting image
    img_compressed = robot.get_img_compressed()
    # Uncompressing image
    img_test = uncompress_image(img_compressed)

    img_threshold_red = threshold_hsv(img_test, *ColorHSV.RED.value)

    img_threshold_blue = threshold_hsv(img_test, list_min_hsv=blue_min_hsv,
                                       list_max_hsv=blue_max_hsv, reverse_hue=False)

    show_img("img_threshold_red", img_threshold_red)

    show_img_and_wait_close("img_threshold_blue", img_threshold_blue)

    # - Display
    # Concatenating raw image and undistorted image
    concat_ims = concat_imgs((img_raw, img_threshold_red, img_threshold_blue))

    # Showing images
    key = show_img("Images raw & undistorted", concat_ims, wait_ms=30)
    if key in [27, ord("q")]:  # Will break loop if the user press Escape or Q
        break