# Inicializamos el tablero con una matriz 3x3 llena de ceros
tablero = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]


def intro_pieza(jugador, x, y):
    """Inserta la pieza del jugador en la posición (x, y) si está libre."""
    if tablero[x][y] == 0:
        tablero[x][y] = jugador
        return True
    else:
        return False


def comprobar_ganador():
    """Comprueba si hay un ganador en el tablero."""
    # Comprobar filas
    for a in range(3):
        if tablero[a][0] == tablero[a][1] == tablero[a][2] != 0:
            return tablero[a][0]

    # Comprobar columnas
    for a in range(3):
        if tablero[0][a] == tablero[1][a] == tablero[2][a] != 0:
            return tablero[0][a]

    # Comprobar diagonales
    if tablero[0][0] == tablero[1][1] == tablero[2][2] != 0:
        return tablero[0][0]

    if tablero[0][2] == tablero[1][1] == tablero[2][0] != 0:
        return tablero[0][2]

    return 0  # Si no hay ganador, devolver 0


def tres_en_raya():
    """Función principal para ejecutar el juego de tres en raya."""
    jugador = 1
    num_piezas = 0
    juego_terminado = False

    while not juego_terminado:
        print(f"Turno del jugador {jugador}")
        x = int(input("Introduce la fila (0, 1, 2): "))
        y = int(input("Introduce la columna (0, 1, 2): "))

        # Validar la entrada
        if 0 <= x < 3 and 0 <= y < 3:
            if intro_pieza(jugador, x, y):
                num_piezas += 1

                # Imprimir el tablero actual
                for fila in tablero:
                    print(fila)

                # Comprobar si hay un ganador después de la 5ª pieza
                if num_piezas >= 5:
                    ganador = comprobar_ganador()
                    if ganador != 0:
                        print(f"¡El jugador {ganador} ha ganado!")
                        juego_terminado = True

                # Cambiar de jugador
                jugador = 2 if jugador == 1 else 1
            else:
                print("¡Esa casilla ya está ocupada!")
        else:
            print("Entrada no válida, intenta de nuevo.")

        # Comprobar si se llenó el tablero y no hay ganador
        if num_piezas == 9 and not juego_terminado:
            print("¡Empate! No hay más movimientos posibles.")
            juego_terminado = True


# Ejecutar el juego
tres_en_raya()
