SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

source $SCRIPT_DIR/Docker/bash_utils.sh

# First ensure container is stopped
docker compose -f $SCRIPT_DIR/docker-compose.yaml down

if hl2ss-ros2-build; then
    echo "hl2ss-ros2-build completed successfully"
else
    echo "hl2ss-ros2-build failed. Aborting."
    exit 1
fi