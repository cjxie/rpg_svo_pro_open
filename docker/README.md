# Docker deployment instructions

1. Build the docker image

    docker build . -t svo

2. Ensure the forwarding is activated for docker

    xhost + local:docker

3. Launch the docker image. The `--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"` part is necessary for X11, and the `-v "$PWD/data:/data"` part is necessary for docker accessing the data on host. 

    docker run -e "DISPLAY" -e "QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" -v "$PWD/data:/data" -it svo:latest

<!-- 
5. Launch the docker image. The `--volume=$HOME/.Xauthority:/root/.Xauthority:rw` part is necessary for X11, and the `--volume /dev:/dev` part is necessary for openGL. 
```sh
docker run --net=host --env="DISPLAY" --volume=$HOME/.Xauthority:/root/.Xauthority:rw --volume /dev:/dev --volume $PWD/test_data:/test_data -it vio:latest -->
