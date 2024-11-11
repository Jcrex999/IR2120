from PIL.ImageChops import offset
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

        self.center_conditioning_pose = PoseObject(
            x=0.0, y=-0.2, z=0.12,
            roll=0.0, pitch=1.57, yaw=-1.57
        )

        self.display_stream = True
        self.grid_dimension = (3, 3)
        self.count = 0

    def get_img(self):
        img_compressed = self.robot.get_img_compressed()
        img = uncompress_image(img_compressed)
        return img

    def get_img_workspace(self):
        img_compressed = self.robot.get_img_compressed()
        img = uncompress_image(img_compressed)
        img = undistort_image(img, self.mtx, self.dist)
        # extracting working area
        im_work = extract_img_workspace(img, workspace_ratio=1.0)
        if im_work is None:
            print("Unable to find markers")
            if self.display_stream:
                cv2.imshow("Last image saw", img)
                cv2.waitKey(25)
        return im_work

    def mover_observation_pose(self):
        self.robot.move_pose(self.observation_pose1)

    def detect_color_red(self):
        img_test = self.get_img_workspace()
        color_hsv_setting = ColorHSV.RED.value
        img_thresh = threshold_hsv(img_test, *color_hsv_setting)
        return img_thresh

    def detect_color_blue(self):
        img_test = self.get_img_workspace()
        color_hsv_setting = ColorHSV.BLUE.value
        img_thresh = threshold_hsv(img_test, *color_hsv_setting)
        return img_thresh

    def detect_color_green(self):
        img_test = self.get_img_workspace()
        color_hsv_setting = ColorHSV.GREEN.value
        img_thresh = threshold_hsv(img_test, *color_hsv_setting)
        return img_thresh

    def vision_pick(self):
        ret = self.robot.get_target_pose_from_cam(self.workspace_name,
                                                   height_offset=0.0,
                                                   shape=ObjectShape.RECTANGLE,
                                                   color=ObjectColor.ANY)
        obj_found, obj_pose, shape, color = ret

        if not obj_found:
            print("No object found")
            return False
        else:
            self.robot.pick_from_pose(obj_pose)

    def mover_to_grid(self):
        offset_x = self.count % self.grid_dimension[0] - self.grid_dimension[0] // 2
        offset_y = (self.count // self.grid_dimension[1]) % 3 - self.grid_dimension[1] // 2
        offset_z = self.count // (self.grid_dimension[0] * self.grid_dimension[1])
        place_pose = self.center_conditioning_pose.copy_with_offsets(0.05 * offset_x, 0.05 * offset_y, 0.025 * offset_z)

        # Placing
        self.robot.place_from_pose(place_pose)

    def run(self):
        self.mover_observation_pose()
        while True:
            img_raw = self.get_img()
            img_threshold_red = self.detect_color_red()
            img_threshold_blue = self.detect_color_blue()
            img_threshold_green = self.detect_color_green()

            self.vision_pick()
            self.mover_to_grid()

            concat_ims = concat_imgs((img_raw, img_threshold_red, img_threshold_blue, img_threshold_green))
            key = show_img("Images raw & undistorted", concat_ims, wait_ms=1)
            if key in [27, ord("q")]:
                break


class Seguimiento:
    def __init__(self):
        self.robot = NiryoRobot("169.254.200.200")
        self.robot.calibrate_auto()
        self.mtx, self.dist = self.robot.get_camera_intrinsics()

    def get_img(self):
        img_compressed = self.robot.get_img_compressed()
        img = uncompress_image(img_compressed)
        return img

    def detect_color(self, color):
        img_test = self.get_img()
        if color == "red":
            color_hsv_setting = ColorHSV.RED.value
        elif color == "blue":
            color_hsv_setting = ColorHSV.BLUE.value
        elif color == "green":
            color_hsv_setting = ColorHSV.GREEN.value
        else:
            color_hsv_setting = ColorHSV.ANY.value

        img_thresh = threshold_hsv(img_test, *color_hsv_setting)
        return img_thresh

    def centrar_objeto(self):
        img = self.get_img()
        # height, width = img.shape[:2]
        img_thresh = self.detect_color("red")
        contour = biggest_contour_finder(img_thresh)
        if contour is None or len(contour) == 0:
            print("No blob found")
            return False
        else:
            cx, cy = get_contour_barycenter(contour)
            cx_rel, cy_rel = relative_pos_from_pixels(img, cx, cy)
            angle = get_contour_angle(contour)
            self.robot.move_pose(PoseObject(x=cx_rel, y=cy_rel, z=0.35, roll=0.0, pitch=1.57, yaw=angle))

        """
        if cx < width/2 - 10:
            #moverse a la derecha
            self.robot.move_pose(PoseObject(x=cx_rel, y=cy_rel, z=0.35, roll=0.0, pitch=1.57, yaw=angle))
        elif cx > width/2 + 10:
            #moverse a la izquierda
            self.robot.move_pose(PoseObject(x=0.18, y=0.0, z=0.35, roll=0.0, pitch=1.57, yaw=0.2))

        if cy < height/2 - 10:
            #moverse hacia arriba
            self.robot.move_pose(PoseObject(x=0.18, y=0.0, z=0.35, roll=0.0, pitch=1.57, yaw=-0.2))
        elif cy > height/2 + 10:
            #moverse hacia abajo
            self.robot.move_pose(PoseObject(x=0.18, y=0.0, z=0.35, roll=0.0, pitch=1.57, yaw=0.2))
        """

    def run(self):
        while True:
            img = self.detect_color("red")
            self.centrar_objeto()

            key = show_img("Image thresh", img, wait_ms=1)
            if key in [27, ord("q")]:
                break


class 


if __name__ == "__main__":
    movimiento_brazo = MovimientoBrazo()
    movimiento_brazo.run()