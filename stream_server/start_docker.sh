#! /bin/bash
docker run --rm -it --network=host -p 8189:8189/udp -p 8189:8189/udp -p 8000:8000/udp -p 8001:8001/udp --privileged --tmpfs /dev/shm:exec -v /run/udev:/run/udev:ro -v ./mediamtx.yml:/mediamtx.yml bluenviron/mediamtx:latest-rpi
