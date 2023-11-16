#!/bin/bash

export MESHROOM_ROOT="$(dirname "$(readlink -f "${BASH_SOURCE[0]}" )" )"
export PYTHONPATH=$MESHROOM_ROOT:$PYTHONPATH
export OPENCV_IO_ENABLE_OPENEXR=1

# using existing alicevision release
#export LD_LIBRARY_PATH=/foo/Meshroom-2023.2.0/aliceVision/lib/
#export PATH=$PATH:/foo/Meshroom-2023.2.0/aliceVision/bin/

# using alicevision built source
#export PATH=$PATH:/foo/build/Linux-x86_64/

python "$MESHROOM_ROOT/meshroom/ui"
