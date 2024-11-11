from pyniryo import *
import cv2
import numpy as np


"""
robot = NiryoRobot("169.254.200.200")
robot.calibrate_auto()
mtx, dist = robot.get_camera_intrinsics()
workspace_name = "bloque2"

observation_pose = PoseObject(
    x=0.18, y=1.0, z=0.35,
    roll=0.0, pitch=1.57, yaw=0.2,
)
"""

def get_img():
    img = cv2.imread("grid/grid_prueba.png")
    return img

def detectar_work_grid():
    img = get_img()
    cv2.imshow("Original", img)
    cv2.waitKey(0)
    # Definir manualmente la matriz de intrínsecos de la cámara y los coeficientes de distorsión
    mtx = np.array([[535.4, 0, 320.1],
                    [0, 539.2, 247.6],
                    [0, 0, 1]])
    dist = np.array([0.2, -0.4, 0.0, 0.0, 0.0])

    img_undi = undistort_image(img, mtx, dist)
    im_work = extract_img_workspace(img_undi, workspace_ratio=1.0)
    if im_work is None:
        print("No se han encontrado marcadores")
        return None
    else:
        return im_work

def detect_grid(im_work):
    img_result = im_work.copy()
    img_hsv = cv2.cvtColor(im_work, cv2.COLOR_BGR2HSV)
    lower = np.array([0, 0, 100])
    upper = np.array([179, 255, 200])
    mask = cv2.inRange(img_hsv, lower, upper)
    cv2.imshow("Mask", mask)
    cv2.waitKey(0)

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    cont = 0
    vacios = []
    tablero = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]

    grid_size = (3, 3)
    cell_width = im_work.shape[1] // grid_size[1]
    cell_height = im_work.shape[0] // grid_size[0]

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 1000:
            cv2.drawContours(img_result, cnt, -1, (255, 0, 0), 3)
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            obj_cor = len(approx)
            x, y, w, h = cv2.boundingRect(approx)
            if obj_cor == 4:
                cv2.rectangle(img_result, (x, y), (x+w, y+h), (0, 255, 0), 3)
                print("Rectangulo")

                # Calcular el centro del rectángulo
                x_center, y_center = get_contour_barycenter(cnt)
                cv2.circle(img_result, (x_center, y_center), 5, (0, 0, 255), -1)

                # Calcular la posición en la matriz grid
                grid_x = x_center // cell_width
                grid_y = y_center // cell_height

                # Detectar si dentro del rectangulo hay un objeto
                roi = im_work[y:y+h, x:x+w]
                cv2.imshow("ROI", roi)
                cv2.waitKey(1)

                # Guardar la imagen del ROI
                #cv2.imwrite(f"roi{cont}.png", roi)

                roi_thresh_red = threshold_hsv(roi, *ColorHSV.RED.value)
                cv2.imshow("ROI Threshold red", roi_thresh_red)
                cv2.waitKey(1)

                roi_thresh_blue = threshold_hsv(roi, *ColorHSV.BLUE.value)
                cv2.imshow("ROI Threshold blue", roi_thresh_blue)
                cv2.waitKey(0)

                # Si detecta el color rojo, entonces hay un objeto
                if cv2.countNonZero(roi_thresh_red) > 0:
                    print("Objeto detectado")
                    tablero[grid_y][grid_x] = 1
                    for i in range(3):
                        print(tablero[i])
                    #enctontrados.append([x_center, y_center])
                    #print(enctontrados)
                elif cv2.countNonZero(roi_thresh_blue) > 0:
                    print("No hay objeto")
                    cx_rel, cy_rel = relative_pos_from_pixels(im_work, x_center, y_center)
                    angle = get_contour_angle(cnt)
                    vacios.append([[x_center, y_center],[cx_rel, cy_rel, angle]])
                    tablero[grid_y][grid_x] = 2
                    print(vacios)
                    for i in range(3):
                        print(tablero[i])
                else:
                    print("No hay objeto")
                    cx_rel, cy_rel = relative_pos_from_pixels(im_work, x_center, y_center)
                    angle = get_contour_angle(cnt)
                    vacios.append([[x_center, y_center],[cx_rel, cy_rel, angle]])
                    tablero[grid_y][grid_x] = 0
                    print(vacios)
                    for i in range(3):
                        print(tablero[i])

                cont += 1

            else:
                print("No es un rectangulo")
    cv2.imshow("Grid resultado", img_result)
    cv2.waitKey(0)





if __name__ == "__main__":
    img = detectar_work_grid()
    if img is not None:
        cv2.imshow("Grid", img)
        cv2.waitKey(0)

    else:
        print("No se han encontrado marcadores")

    #detect
    detect_grid(img)
    cv2.destroyAllWindows()