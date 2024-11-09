
# Proyecto NiryoStudio en Docker

Este proyecto configura un contenedor Docker que incluye ROS Melodic y NiryoStudio, utilizando una versión empaquetada en formato .zip.

## Requisitos Previos

Asegúrate de tener instalado Docker en tu máquina. Puedes encontrar instrucciones para instalar Docker [aquí](https://docs.docker.com/get-docker/).

## Construcción del Contenedor

1. Clona este repositorio o copia los archivos necesarios en un directorio local.

2. Navega al directorio donde se encuentra el `Dockerfile` utilizando la terminal.

3. Construye la imagen Docker con el siguiente comando:

   ```bash
   docker build -t ros_melodic_niryo_ned1 .
   ```

## Ejecución del Contenedor

Para ejecutar el contenedor y permitir el uso de la interfaz gráfica, usa el siguiente comando:

```bash
docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ros_melodic_niryo_ned1 bash
```

Esto iniciará una sesión interactiva en el contenedor.

## Uso de NiryoStudio

Una vez dentro del contenedor, ejecuta el siguiente comando para iniciar NiryoStudio:

```bash
/start.sh
```

## Solución de Problemas

- Si encuentras problemas relacionados con la interfaz gráfica, asegúrate de que el servidor X esté en funcionamiento y que el contenedor tenga acceso a él.

- Para cualquier otro problema, revisa los registros de Docker para obtener más información sobre los errores.

## Notas Adicionales

- Este contenedor está basado en Ubuntu 18.04 y es compatible con ROS Melodic.
- El script `start.sh` configura el entorno de ROS y ejecuta NiryoStudio.

