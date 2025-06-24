# Build the Docker image
docker build -t dev/automodel_linedetector -f docker/Dockerfile .

# Run the container (with GUI support if needed)
``` docker
docker run -it --rm \
    --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --device="/dev/video0:/dev/video0" \
    --volume $(pwd):/home/docker/catkin_ws \
    --name dev_automodel_linedetector dev/automodel_linedetector bash
```
