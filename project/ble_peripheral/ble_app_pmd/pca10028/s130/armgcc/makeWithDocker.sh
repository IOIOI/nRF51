#!/bin/bash

eval "$(docker-machine env default)"

docker run -i  -v $(pwd)/../../../../../..:/src -w /src/project/ble_peripheral/ble_app_pmd/pca10028/s130/armgcc --net="host"  polybassa/pmd:latest make "$@"

