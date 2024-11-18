
from pyniryo import *
import cv2
import numpy as np

SIMULATION = False

class ControlNiryo:
    def __init__(self):
        # Definir el robot y calibrar
        if SIMULATION:
            self.robot = NiryoRobot("127.0.0.1")
            self.workspace_name1 = "gazebo_1"
            self.workspace_name2 = "gazebo_2"
        else:
            self.robot = NiryoRobot("169.254.200.200")
            self.workspace_name1 = "bloque1"
            self.workspace_name2 = "bloque2"

        self.robot.calibrate_auto()

        # Obtener la matriz de la cámara y los coeficientes de distorsión
        self.mtx, self.dist = self.robot.get_camera_intrinsics()

        # Definir las variables específicas para las actividades
        self.variables_especificas = {
            "display_stream": True,
            "grid_dimension": (3, 3),
            "count": 0,
            "try_without_success": 0,
            "vision_process_on_robot": False,
            "tablero": [[0, 0, 0], [0, 0, 0], [0, 0, 0]],
            "jugador": 1,
            "Colocar_grid": {
                "Disponibles": [],
                "Ocupados": []
            },
            "best_move": [],
            "detectado": False
        }

        # Definir las posiciones de observación
        self.observation_poses = {
            "bloque1": PoseObject(
                x=0.18, y=0.0, z=0.35,
                roll=0.0, pitch=1.67, yaw=0,
            ),
            "bloque2": PoseObject(
                x=0.0, y=0.2, z=0.35,
                roll=0.0, pitch=1.75, yaw=1.57,
            ),
            "gazebo_1": PoseObject(
                x=0.18, y=0.0, z=0.35,
                roll=0.0, pitch=1.57, yaw=0.0,
            ),
            "gazebo_2": PoseObject(
                x=0.0, y=0.2, z=0.35,
                roll=0.0, pitch=1.67, yaw=1.57,
            )
        }

        # Definir las posiciones de colocación
        self.place_pose = {
            "center_conditioning_pose": PoseObject(
                x=0.0, y=0.2, z=0.12,
                roll=0.0, pitch=1.57, yaw=1.57
            ),
            "dejar_obj": PoseObject(
                x=0.0, y=0.25, z=0.12,
                roll=0.0, pitch=1.57, yaw=1.57
            )
        }

        self.grid_pose = {}

    def get_img(self):
        img_compressed = self.robot.get_img_compressed()
        img = uncompress_image(img_compressed)
        return img

    def get_img_workspace(self, workspace_name):
        img_compressed = self.robot.get_img_compressed()
        img = uncompress_image(img_compressed)
        img = undistort_image(img, self.mtx, self.dist)
        im_work = extract_img_workspace(img, workspace_ratio=1.0)
        if im_work is None:
            print("Unable to find markers")
            if self.variables_especificas["display_stream"]:
                cv2.imshow("Last image saw", img)
                cv2.waitKey(25)
        return im_work

    def mover_observation_pose(self, workspace_name):
        self.robot.move_pose(self.observation_poses[workspace_name])

    def detect_color(self, color, workspace_name=None):
        color_mapping = {
            "RED": ColorHSV.RED.value,
            "BLUE": ColorHSV.BLUE.value,
            "GREEN": ColorHSV.GREEN.value,
            "ANY": ColorHSV.ANY.value
        }

        if workspace_name is None:
            img_test = self.get_img()
        else:
            img_test = self.get_img_workspace(workspace_name)

        color_hsv_setting = color_mapping.get(color.upper(), ColorHSV.ANY.value)
        img_thresh = threshold_hsv(img_test, *color_hsv_setting)
        return img_thresh

    def vision_pick(self):
        # Inicializar variables
        obj_pose = None
        self.variables_especificas["try_without_success"] = 0

        # Loop
        while self.variables_especificas["try_without_success"] < 5:
            # Mover a la posición de observación
            self.mover_observation_pose(self.workspace_name1)

            # Procesar la imagen desde el robot
            if self.variables_especificas["vision_process_on_robot"]:
                ret = self.robot.get_target_pose_from_cam(self.workspace_name1,
                                                           height_offset=0.0,
                                                           shape=ObjectShape.RECTANGLE,
                                                           color=ObjectColor.ANY)
                obj_found, obj_pose, shape, color = ret

            # Procesar la imagen manualmente
            else:
                img = self.get_img()
                img = undistort_image(img, self.mtx, self.dist)
                im_work = extract_img_workspace(img, workspace_ratio=1.0)
                if im_work is None:
                    print("Unable to find markers")
                    self.variables_especificas["try_without_success"] += 1
                    if self.variables_especificas["display_stream"]:
                        cv2.imshow("Last image saw", img)
                        cv2.waitKey(25)
                    continue

                color_hsv_setting = ColorHSV.ANY.value
                img_thresh = threshold_hsv(im_work, *color_hsv_setting)

                if self.variables_especificas["display_stream"]:
                    cv2.imshow("Last image saw", img)
                    cv2.waitKey(25)
                    cv2.imshow("Thresholded image", img_thresh)
                    cv2.waitKey(25)

                contour = biggest_contour_finder(img_thresh)
                if contour is None or len(contour) == 0:
                    print("No blob found")
                    obj_found = False
                else:
                    img_thresh_rgb_w_contour = draw_contours(img_thresh, [contour])

                    cx, cy = get_contour_barycenter(contour)
                    img_thresh_rgb_w_contour = draw_barycenter(img_thresh_rgb_w_contour, cx, cy)
                    cx_rel, cy_rel = relative_pos_from_pixels(im_work, cx, cy)

                    angle = get_contour_angle(contour)
                    img_thresh_rgb_w_contour = draw_angle(img_thresh_rgb_w_contour, cx, cy, angle)

                    cv2.imshow("Thresholded image", img_thresh_rgb_w_contour)
                    cv2.waitKey(25)

                    obj_pose = self.robot.get_target_pose_from_rel(self.workspace_name1,
                                                                    height_offset=0.0,
                                                                    x_rel=cx_rel, y_rel=cy_rel,
                                                                    yaw_rel=angle)
                    obj_found = True

            if not obj_found:
                print("No object found")
                self.variables_especificas["try_without_success"] += 1
                continue

            self.robot.pick_from_pose(obj_pose)
            self.variables_especificas["count"] += 1

            break

    def mover_pick(self, place_pose):
        while True:
            self.vision_pick()
            if self.variables_especificas["try_without_success"] >= 5:
                break
            self.robot.move_pose(self.place_pose[place_pose])
            self.robot.open_gripper(100)
            self.robot.move_pose(self.observation_poses[self.workspace_name1])

    # ==================================================
    # Seguimiento de objetos
    def centrar_objeto(self):
        img = self.get_img()
        img_thresh = self.detect_color("green")
        contour = biggest_contour_finder(img_thresh)
        if contour is None or len(contour) == 0:
            print("No blob found")
            return False
        else:
            cx, cy = get_contour_barycenter(contour)
            cx_rel, cy_rel = relative_pos_from_pixels(img, cx, cy)
            print(cx_rel, cy_rel)

            # Obtener el ángulo actual de la cámara
            pose = self.robot.get_pose().to_list()
            pitch = pose[4]  # Yaw es el sexto elemento en la lista de pose

            # Ajustar movimientos según el ángulo de la cámara
            if 1.4 < pitch < 1.8:
                # Esta duplicado, pero mas adelante se puede ajustar
                if cx < img.shape[1] // 2 - 10:
                    if pose[1] < 0.5:
                        print("Mover a la derecha")
                        pose[1] -= 0.01
                elif cx > img.shape[1] // 2 + 10:
                    if pose[1] > -0.5:
                        print("Mover a la izquierda")
                        pose[1] += 0.01

                # Hasta aquí se repite

                if cy < img.shape[0] // 2 - 10:
                    if pose[0] < 0.5:
                        print("Mover hacia abajo")
                        pose[0] -= 0.01
                elif cy > img.shape[0] // 2 + 10:
                    if pose[0] > -0.5:
                        print("Mover hacia arriba")
                        pose[0] += 0.01
            elif pitch == 0:
                # Esta duplicado, pero mas adelante se puede ajustar
                if cx < img.shape[1] // 2:
                    if pose[1] < 0.5:
                        pose[1] += 0.001
                elif cx > img.shape[1] // 2:
                    if pose[1] > -0.5:
                        pose[1] -= 0.001
                # Hasta aquí se repite

                if cy < img.shape[0] // 2:
                    if pose[2] < 0.5:
                        pose[2] += 0.001
                elif cy > img.shape[0] // 2:
                    if pose[2] > -0.5:
                        pose[2] -= 0.1

            print("\n", pose, "Situacion actual\n")
            #img_contour = draw_barycenter(img, cx, cy)
            cv2.imshow("Centrado", img)
            self.robot.move_pose(PoseObject(*pose))


    # ==================================================
    # Juego del tres en raya

    def detectar_work_grid(self):
        img = self.get_img()
        img_undi = undistort_image(img, self.mtx, self.dist)
        im_work = extract_img_workspace(img_undi, workspace_ratio=1.0)
        if im_work is None:
            print("No se han encontrado marcadores")
            return None
        else:
            return im_work

    def comprobar_ganador(self):
        """Comprueba si hay un ganador en el tablero."""
        # Comprobar filas

        for a in range(3):
            if self.variables_especificas["tablero"][a][0] == self.variables_especificas["tablero"][a][1] == self.variables_especificas["tablero"][a][2] != 0:
                return self.variables_especificas["tablero"][a][0]

        # Comprobar columnas
        for a in range(3):
            if self.variables_especificas["tablero"][0][a] == self.variables_especificas["tablero"][1][a] == self.variables_especificas["tablero"][2][a] != 0:
                return self.variables_especificas["tablero"][0][a]

        # Comprobar diagonales
        if self.variables_especificas["tablero"][0][0] == self.variables_especificas["tablero"][1][1] == self.variables_especificas["tablero"][2][2] != 0:
            return self.variables_especificas["tablero"][0][0]

        if self.variables_especificas["tablero"][0][2] == self.variables_especificas["tablero"][1][1] == self.variables_especificas["tablero"][2][0] != 0:
            return self.variables_especificas["tablero"][0][2]

        return 0  # Si no hay ganador, devolver 0

    def detect_grid(self, im_work):
        img_result = im_work.copy()
        img_hsv = cv2.cvtColor(im_work, cv2.COLOR_BGR2HSV)
        lower = np.array([0, 0, 0])
        upper = np.array([0, 0, 44])
        mask = cv2.inRange(img_hsv, lower, upper)

        cv2.imshow("Mask", mask)
        cv2.waitKey(100)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        grid_size = self.variables_especificas["grid_dimension"]
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

                    cv2.imshow("Grid", img_result)
                    cv2.waitKey(100)

                    # Calcular la posición en la matriz grid
                    grid_x = x_center // cell_width
                    grid_y = y_center // cell_height

                    # Detectar si dentro del rectangulo hay un objeto
                    roi = im_work[y:y + h, x:x + w]
                    #cv2.imshow("ROI", roi)
                    #cv2.waitKey(1)

                    roi_thresh_red = threshold_hsv(roi, *ColorHSV.RED.value)
                    #cv2.imshow("ROI Threshold red", roi_thresh_red)
                    #cv2.waitKey(1)

                    roi_thresh_blue = threshold_hsv(roi, *ColorHSV.BLUE.value)
                    #cv2.imshow("ROI Threshold blue", roi_thresh_blue)
                    #cv2.waitKey(0)

                    # Si detecta el color rojo, entonces hay un objeto
                    if cv2.countNonZero(roi_thresh_red) > 0:
                        print("Ocupado por el jugador 1")
                        self.variables_especificas["Colocar_grid"]["Ocupados"].append([x_center, y_center])
                        self.variables_especificas["tablero"][grid_y][grid_x] = 1
                        for i in range(3):
                            print(self.variables_especificas["tablero"][i])
                        # enctontrados.append([x_center, y_center])
                        # print(enctontrados)
                    elif cv2.countNonZero(roi_thresh_blue) > 0:
                        print("Ocupado por el jugador 2")
                        self.variables_especificas["Colocar_grid"]["Ocupados"].append([x_center, y_center])
                        self.variables_especificas["tablero"][grid_y][grid_x] = 2
                        print(self.variables_especificas["Colocar_grid"]["Ocupados"])
                        for i in range(3):
                            print(self.variables_especificas["tablero"][i])
                    else:
                        print("No hay objeto")
                        cx_rel, cy_rel = relative_pos_from_pixels(im_work, x_center, y_center)
                        angle = get_contour_angle(cnt)
                        self.variables_especificas["Colocar_grid"]["Disponibles"].append([[grid_y, grid_x],cx_rel, cy_rel, angle])
                        self.variables_especificas["tablero"][grid_y][grid_x] = 0
                        print(self.variables_especificas["Colocar_grid"]["Disponibles"])
                        for i in range(3):
                            print(self.variables_especificas["tablero"][i])

    def calcular_jugada(self):
        def minimax(tablero, depth, is_maximizing):
            score = self.comprobar_ganador()
            if score == 1:
                return 10 - depth
            elif score == 2:
                return depth - 10
            elif not any(0 in row for row in tablero):
                return 0

            if is_maximizing:
                best_score = float('-inf')
                for i in range(3):
                    for j in range(3):
                        if tablero[i][j] == 0:
                            tablero[i][j] = 1
                            score = minimax(tablero, depth + 1, False)
                            tablero[i][j] = 0
                            best_score = max(score, best_score)
                return best_score
            else:
                best_score = float('inf')
                for i in range(3):
                    for j in range(3):
                        if tablero[i][j] == 0:
                            tablero[i][j] = 2
                            score = minimax(tablero, depth + 1, True)
                            tablero[i][j] = 0
                            best_score = min(score, best_score)
                return best_score

        def find_best_move(tablero):
            best_score = float('-inf')
            best_move = None
            for i in range(3):
                for j in range(3):
                    if tablero[i][j] == 0:
                        tablero[i][j] = 1
                        score = minimax(tablero, 0, False)
                        tablero[i][j] = 0
                        if score > best_score:
                            best_score = score
                            best_move = (i, j)
            return best_move

        best_move = find_best_move(self.variables_especificas["tablero"])
        print(f"Mejor movimiento: {best_move}")
        if best_move is not None:
            self.variables_especificas["best_move"] = list(best_move)
            self.variables_especificas["tablero"][best_move[0]][best_move[1]] = 1
            print(f"Jugador 1 coloca en la posicion {best_move}")
            for fila in self.variables_especificas["tablero"]:
                print(fila)

    def move_to_grid(self):
        """
        # Tomamos los valores de Colocar_grid y los guardamos en una variable
        disponibles = self.variables_especificas["Colocar_grid"]["Disponibles"]

        print("\nDisponibles: ", disponibles)
        print("\nMejor movimiento: ", self.variables_especificas["best_move"])

        for move in disponibles:
            print("\n", move[0], self.variables_especificas["best_move"])
            if move[0] == self.variables_especificas["best_move"]:
                print("\n\nMovimiento encontrado", move)
                self.robot.move_pose(self.observation_poses[self.workspace_name2])
                place_pose = PoseObject(x=move[1], y=move[2], z=0.35, roll=0.0, pitch=1.57, yaw=move[3])
                self.robot.move_to_object(self.workspace_name2, place_pose)
                break
        """
        best_move = self.variables_especificas["best_move"]
        """
        if best_move == [0,0]:
            # hacer un movimiento definido para la posicion 0,0
            #self.robot.move_pose(PoseObject(-0.033, 0.285, 0.1, 0, 1.57, 1.57))
            self.robot.move_pose(self.robot.get_pose_saved("grid_0_0"))
        elif best_move == [0,1]:
            # hacer un movimiento definido para la posicion 0,1
            #self.robot.move_pose(PoseObject(0.0, 0.285, 0.1, 0, 1.57, 1.57))
            self.robot.move_pose(self.robot.get_pose_saved("grid_0_1"))
        elif best_move == [0,2]:
            # hacer un movimiento definido para la posicion 0,2
            #self.robot.move_pose(PoseObject(0.033, 0.285, 0.1, 0, 1.57, 1.57))
            self.robot.move_pose(self.robot.get_pose_saved("grid_0_2"))
            # [0.0, 0.29, 0.1, 0, 1.57, 1.57]
        elif best_move == [1, 0]:
            # hacer un movimiento definido para la posicion 1,0
            #self.robot.move_pose([-0.033, 0.253, 0.1, 0, 1.57, 1.57])
            self.robot.move_pose(self.robot.get_pose_saved("grid_1_0"))
        elif best_move == [1,1]:
            # hacer un movimiento definido para la posicion 1,1
            #self.robot.move_pose([0.0, 0.253, 0.1, 0, 1.57, 1.57])
            self.robot.move_pose(self.robot.get_pose_saved("grid_1_1"))
        elif best_move == [1,2]:
            # hacer un movimiento definido para la posicion 1,2
            #self.robot.move_pose([0.033, 0.253, 0.1, 0, 1.57, 1.57])
            self.robot.move_pose(self.robot.get_pose_saved("grid_1_2"))
        elif best_move == [2,0]:
            # hacer un movimiento definido para la posicion 2,0
            #self.robot.move_pose([-0.033, 0.217, 0.1, 0, 1.57, 1.57])
            self.robot.move_pose(self.robot.get_pose_saved("grid_2_0"))
        elif best_move == [2,1]:
            # hacer un movimiento definido para la posicion 2,1
            #self.robot.move_pose([0.0, 0.217, 0.1, 0, 1.57, 1.57])
            self.robot.move_pose(self.robot.get_pose_saved("grid_2_1"))
        elif best_move == [2,2]:
            # hacer un movimiento definido para la posicion 2,2
            #self.robot.move_pose([0.033, 0.217, 0.1, 0, 1.57, 1.57])
            self.robot.move_pose(self.robot.get_pose_saved("grid_2_2"))
        else:
            print("No se ha encontrado el mejor movimiento")
        """
        try:
            self.robot.move_pose(self.robot.get_pose_saved(f"grid_{best_move[0]}_{best_move[1]}"))
        except Exception as e:
            print(e)
            print("Hubo un error al encntrar el pose guardado")

    def save_pose_grid(self):
        for i in range(3):
            for j in range(3):
                self.robot.set_learning_mode(True)
                print(input(f"Mover el niryo y preciona enter para guardar la pose de la casilla ({i}, {j})"))
                self.robot.save_pose(f"grid_{i}_{j}", self.robot.get_pose())
                self.robot.set_learning_mode(False)

    def load_pose_grid(self):
        for i in range(3):
            for j in range(3):
                self.grid_pose[f"grid_{i}_{j}"] = self.robot.get_pose_saved(f"grid_{i}_{j}")

    def save_pose_observation(self):
        opcion = input("Selecciona un workspace: \n"
                       "1. Bloque1 \n"
                       "2. Bloque2 \n")
        try:
            self.robot.set_learning_mode(True)
            self.robot.save_pose(f"bloque{opcion}", self.robot.get_pose())
            self.observation_poses[f"bloque{opcion}"] = self.robot.get_pose_saved(f"bloque{opcion}")
            self.robot.set_learning_mode(False)
        except Exception as e:
            print(e)
            print("Hubo un error al guardar la pose")

    def hacer_jugada(self):
        img = self.detectar_work_grid()
        if img is not None:
            cv2.imshow("Grid", img)
            self.detect_grid(img)
            self.calcular_jugada()
            self.vision_pick()
            self.move_to_grid()
            self.robot.open_gripper(400)

    def filtrado_img(self):
        img = self.get_img_workspace("gazebo_2")
        cv2.namedWindow('Trackbars')

        # Create trackbars for color change
        cv2.createTrackbar('H Lower', 'Trackbars', 0, 179, nothing)
        cv2.createTrackbar('S Lower', 'Trackbars', 0, 255, nothing)
        cv2.createTrackbar('V Lower', 'Trackbars', 0, 255, nothing)
        cv2.createTrackbar('H Upper', 'Trackbars', 179, 179, nothing)
        cv2.createTrackbar('S Upper', 'Trackbars', 255, 255, nothing)
        cv2.createTrackbar('V Upper', 'Trackbars', 255, 255, nothing)

        while True:
            # Get current positions of the trackbars
            h_lower = cv2.getTrackbarPos('H Lower', 'Trackbars')
            s_lower = cv2.getTrackbarPos('S Lower', 'Trackbars')
            v_lower = cv2.getTrackbarPos('V Lower', 'Trackbars')
            h_upper = cv2.getTrackbarPos('H Upper', 'Trackbars')
            s_upper = cv2.getTrackbarPos('S Upper', 'Trackbars')
            v_upper = cv2.getTrackbarPos('V Upper', 'Trackbars')

            lower = np.array([h_lower, s_lower, v_lower])
            upper = np.array([h_upper, s_upper, v_upper])

            img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(img_hsv, lower, upper)
            result = cv2.bitwise_and(img, img, mask=mask)

            cv2.imshow('Original', img)
            cv2.imshow('Mask', mask)
            cv2.imshow('Filtered', result)

            if cv2.waitKey(1) & 0xFF == 27:  # Press 'ESC' to exit
                break

        cv2.destroyAllWindows()

    def menu(self):
        salir = False

        while not salir:
            print("1. Hacer ejercicio 1 (Mover y dejar objeto)")
            print("2. Hacer ejercicio 2 (Hacer Hacer jugada)")
            print("3. Hacer ejercicio 3 (Centrar objeto)")
            print("4. Guardar pose")
            print("5. Cargar pose")
            print("6. Mostrar lista de poses")
            print("7. Filtrar imagen")
            print("8. Salir")
            opcion = input("Selecciona una opcion: ")

            if opcion == "1":
                self.mover_pick("dejar_obj")
            elif opcion == "2":
                while True:
                    if SIMULATION:
                        robot.robot.move_pose(robot.observation_poses["gazebo_2"])
                    else:
                        robot.robot.move_pose(robot.observation_poses["bloque2"])

                    print(input("Jugador 2, empieza"))
                    robot.hacer_jugada()
                    if robot.comprobar_ganador() == 1:
                        print("Gano el jugador 1")
                        break
                    elif robot.comprobar_ganador() == 2:
                        print("Gano el jugador 2")
                        break
            elif opcion == "3":
                if SIMULATION:
                    robot.robot.move_pose(robot.observation_poses["gazebo_1"])
                else:
                    robot.robot.move_pose(robot.observation_poses["bloque1"])
                self.centrar_objeto()
            elif opcion == "4":
                self.save_pose_grid()
            elif opcion == "5":
                self.load_pose_grid()
            elif opcion == "6":
                n_pose = self.robot.get_saved_pose_list()
                for pose in n_pose:
                    print(pose)
            elif opcion == "7":
                self.filtrado_img()
            elif opcion == "8":
                salir = True

        print("Fin del programa")
        exit()

"""
    Nota: hay una funcion llamada say en NiryoRobot que funciona de la siguiente forma:
    
    say(self, text, language=0)
    
    Use gtts (Google Text To Speech) to interprete a string as sound
        Languages available are:
        * English: 0
        * French: 1
        * Spanish: 2
        * Mandarin: 3
        * Portuguese: 4

        Example ::

            robot.say("Hello", 0)
            robot.say("Bonjour", 1)
            robot.say("Hola", 2)
            
    Mas adelante usarlo para que en la interfas de menu hable
"""

def nothing(x):
    pass

if __name__ == "__main__":
    robot = ControlNiryo()
    robot.menu()
