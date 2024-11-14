# NiryoStudio Docker Setup

Este repositorio contiene un Dockerfile para configurar un entorno de desarrollo con **NiryoStudio** y **ROS Melodic** utilizando Ubuntu 18.04. A continuación se presentan las instrucciones para construir y ejecutar el contenedor.

## Requisitos Previos

Asegúrate de tener Docker instalado en tu máquina. Puedes instalar Docker siguiendo las instrucciones en [la documentación oficial de Docker](https://docs.docker.com/get-docker/).

## Construcción del Contenedor

1. Clona este repositorio (o copia el Dockerfile a tu máquina local).

2. Navega al directorio donde se encuentra el Dockerfile:

   ```bash
   cd /ruta/a/tu/directorio
   ```

3. Construye la imagen de Docker con el siguiente comando:

   ```bash
   docker build -t ros_melodic_niryo_ned<modelo> .
   ```
   Remplace `<modelo>` por el número de modelo de la estación de trabajo, 1 para Ned1 y 2 para Ned2.

## Ejecución del Contenedor

### Permisos para X11

Antes de ejecutar el contenedor, debes permitir el acceso a tu servidor X11. Ejecuta el siguiente comando:

```bash
xhost +local:
```

Ahor hay que ejecutar el contenedor interactivo y configurar la variable de entorno `DISPLAY` para la interfaz gráfica de NiryoStudio, para ello utiliza el siguiente comando:

```bash
docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ros_melodic_niryo_ned<modelo> bash
```
Esto permite que el contenedor acceda a la pantalla de tu sistema.

## Uso de NiryoStudio

Una vez que estés dentro del contenedor, puedes iniciar NiryoStudio ejecutando:

```bash
./opt/NiryoStudio/NiryoStudio --no-sandbox &
```

El argumento `--no-sandbox` es necesario para evitar errores de configuración del sandbox.

Sin embago, ya esta configurado para que se ejecute automáticamente al iniciar el contenedor.

## Ejecucion del simulador

Para ejecutar la simulacion del niryo robot, ejecuta el siguiente comando:

```bash
roslaunch niryo_robot_bringup desktop_gazebo_simulation.launch
```

## Notas

- Asegúrate de que tu sistema tiene suficientes recursos para ejecutar Docker y NiryoStudio.
- Cualquier problema o error puede ser reportado en este repositorio.

## Contribuciones

Las contribuciones son bienvenidas. Si deseas mejorar este proyecto, no dudes en enviar un pull request o abrir un issue.

## Licencia

Este proyecto está bajo la Licencia MIT. Para más detalles, consulta el archivo `LICENSE`.
