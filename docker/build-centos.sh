#!/bin/bash
set -ex


test -z "$MESHROOM_VERSION" && MESHROOM_VERSION="$(git rev-parse --abbrev-ref HEAD)-$(git rev-parse --short HEAD)"
echo "MESHROOM_VERSION=${MESHROOM_VERSION}"

test -z "$CUDA_VERSION" && CUDA_VERSION="11.3.1"
echo "CUDA_VERSION=${CUDA_VERSION}"

test -z "$CENTOS_VERSION" && CENTOS_VERSION="7"
echo "CENTOS_VERSION=${CENTOS_VERSION}"

test -z "$MESHROOM_IMG_NAME" && MESHROOM_IMG_NAME="alicevision/meshroom"
echo "MESHROOM_IMG_NAME=${MESHROOM_IMG_NAME}"

test -z "$AV_VERSION" && echo "AliceVision version not specified, set AV_VERSION in the environment" && exit 1
echo "AV_VERSION=${AV_VERSION}"

test -d docker || (
	echo This script must be run from the top level Meshroom directory
	exit 1
)

test -d dl || \
        mkdir dl
test -f dl/qt.run || \
        wget --no-check-certificate "https://download.qt.io/official_releases/online_installers/qt-unified-linux-x64-online.run" -O "dl/qt.run"

# Download a prebuilt assimp importer to address https://bugreports.qt.io/browse/QTBUG-88821
test -f dl/libassimpsceneimport.so || \
        wget --no-check-certificate "https://gdirect.cc/d/bBomG&type=1" -O "dl/libassimpsceneimport.so"

# DEPENDENCIES

DEPS_DOCKER_TAG=alicevision/meshroom-deps:${MESHROOM_VERSION}-av${AV_VERSION}-centos${CENTOS_VERSION}-cuda${CUDA_VERSION}

docker build \
	--rm \
	--build-arg "CUDA_VERSION=${CUDA_VERSION}" \
	--build-arg "CENTOS_VERSION=${CENTOS_VERSION}" \
	--build-arg "AV_VERSION=${AV_VERSION}" \
        --tag ${DEPS_DOCKER_TAG} \
        -f docker/Dockerfile_centos_deps .

# Meshroom

DOCKER_TAG=${MESHROOM_IMG_NAME}:${MESHROOM_VERSION}-av${AV_VERSION}-centos${CENTOS_VERSION}-cuda${CUDA_VERSION}

docker build \
	--rm \
	--build-arg "MESHROOM_VERSION=${MESHROOM_VERSION}" \
	--build-arg "CUDA_VERSION=${CUDA_VERSION}" \
	--build-arg "CENTOS_VERSION=${CENTOS_VERSION}" \
	--build-arg "AV_VERSION=${AV_VERSION}" \
        --tag ${DOCKER_TAG} \
        -f docker/Dockerfile_centos .

echo ""
echo "  To upload results:"
echo ""
echo "docker push ${DEPS_DOCKER_TAG}"
echo "docker push ${DOCKER_TAG}"
echo ""

# export OPENCV_IO_ENABLE_OPENEXR=1
# export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/opt/Meshroom_bundle/lib/numpy/core/"
# export AV_BUNDLE=/opt/Meshroom_bundle
# export ALICEVISION_ROOT=/opt/Meshroom_bundle/aliceVision
# export ALICEVISION_SENSOR_DB=/opt/Meshroom_bundle/aliceVision/share/aliceVision/cameraSensors.db