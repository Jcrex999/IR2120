# Nombre de la imagen Docker
IMAGE_NAME="davidballesterpirchmoser/ubuntu-niryo"

# Nombre del contenedor (opcional)
CONTAINER_NAME="niryo_container"

# Variables para el entorno gráfico y soporte de GPU
XSOCK="/tmp/.X11-unix"
XAUTH="$HOME/.Xauthority"

# Ejecutar el contenedor con todas las condiciones necesarias
docker run --rm -it \
    --name $CONTAINER_NAME \
    --network host \
    --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$(pwd)":/workspace \
    --privileged \
    $IMAGE_NAME

# Comprobar si el contenedor se ejecutó correctamente
if [ $? -eq 0 ]; then
    echo "El contenedor '$CONTAINER_NAME' se está ejecutando con éxito."
else
    echo "Hubo un error al intentar ejecutar el contenedor."
    exit 1
fi
