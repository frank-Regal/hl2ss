SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PARENT_DIR="$(dirname "$SCRIPT_DIR")"

IMAGE_NAME="hl2ss/ros2"
CONTAINER_NAME="hl2ss_ros2"

hl2ss-ros2-start(){
    #add xhost permissions for docker to use display
    xhost +local:docker
    echo "added docker xhost permissions"
    export UID_GID=$(id -u):$(id -g)
    export UNAME=$(whoami)
    echo "gathering host user info... $UNAME $UID_GID"
    docker compose -f $SCRIPT_DIR/docker-compose.yaml up -d
}

hl2ss-ros2-stop(){
    # stop the container and remove it
    docker compose -f $SCRIPT_DIR/docker-compose.yaml down
}

hl2ss-ros2-rm(){
    # remove the docker image
    docker image rm -f $IMAGE_NAME
}

hl2ss-ros2-shell() {
    # open a shell in the container
    docker exec -it -u $USER -w ~/ $CONTAINER_NAME /bin/bash -l
}

hl2ss-ros2-build() {
    # if the container is running, stop it
    if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
        echo "stopping container $CONTAINER_NAME..."
        docker compose -f $SCRIPT_DIR/docker-compose.yaml down
    fi
    # build the container from the docker-compose file
    docker compose -f $SCRIPT_DIR/docker-compose.yaml build
}

hl2ss-ros2-log() {
    # view the logs of the picogk container
    docker logs $CONTAINER_NAME
}
