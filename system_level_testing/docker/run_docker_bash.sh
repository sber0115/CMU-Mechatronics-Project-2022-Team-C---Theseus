docker run -it --net=host --env="DISPLAY" --volume="$HOME/.Xauthority:/root/.Xauthority:rw" --device=/dev/ttyACM0 rosserial_ydlidar_arduino bash
