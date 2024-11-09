from pyniryo import *


class MovimientoBrazo:
    def __init__(self):
        self.robot = NiryoRobot("169.254.200.200")
        self.robot.calibrate_auto()
        self.mtx, self.dist = self.robot.get_camera_intrinsics()
        self.workspace_name = "bloque1"
        self.observation_pose1 = PoseObject(
            x=0.18, y=0.0, z=0.35,
            roll=0.0, pitch=1.57, yaw=-0.2,
        )
        self.observation_pose2 = PoseObject(
            x=0.18, y=1.0, z=0.35,
            roll=0.0, pitch=1.57, yaw=0.2,
        )
        self.place_pose = PoseObject(
            x=0.0, y=-0.2, z=0.12,
            roll=0.0, pitch=1.57, yaw=-1.57
        )

    def get_img(self):
        img_compressed = self.robot.get_img_compressed()
        img = uncompress_image(img_compressed)
        return img

    def mover_observation_pose(self):
        self.robot.move_pose(self.observation_pose1)

    def detect_color_red(self):
        red_min_hsv = [0, 100, 100]
        red_max_hsv = [10, 255, 255]
        img_test = self.get_img()
        img_threshold_red = threshold_hsv(img_test, list_min_hsv=red_min_hsv,
                                           list_max_hsv=red_max_hsv, reverse_hue=False)
        return img_threshold_red

    def detect_color_blue(self):
        blue_min_hsv = [90, 85, 70]
        blue_max_hsv = [125, 255, 255]
        img_test = self.get_img()
        img_threshold_blue = threshold_hsv(img_test, list_min_hsv=blue_min_hsv,
                                           list_max_hsv=blue_max_hsv, reverse_hue=False)
        return img_threshold_blue

    def detect_color_green(self):
        green_min_hsv = [35, 100, 100]
        green_max_hsv = [85, 255, 255]
        img_test = self.get_img()
        img_threshold_green = threshold_hsv(img_test, list_min_hsv=green_min_hsv,
                                           list_max_hsv=green_max_hsv, reverse_hue=False)
        return img_threshold_green

    def vision_pick_and_place(self):
        obj_found, shape_ret, color_ret = self.robot.vision_pick(self.workspace_name)
        if obj_found:
            self.robot.place_from_pose(self.place_pose)

        self.robot.set_learning_mode(True)

    def run(self):
        self.mover_observation_pose()
        while True:
            img_raw = self.get_img()
            img_threshold_red = self.detect_color_red()
            img_threshold_blue = self.detect_color_blue()
            img_threshold_green = self.detect_color_green()
            concat_ims = concat_imgs((img_raw, img_threshold_red, img_threshold_blue, img_threshold_green))
            key = show_img("Images raw & undistorted", concat_ims, wait_ms=30)
            if key in [27, ord("q")]:
                break

if __name__ == "__main__":
    movimiento_brazo = MovimientoBrazo()
    movimiento_brazo.run()