#!/bin/bash
# The picture for the two-way mirror: the garments alone on black, drawn from the user's eye point
# so they land on the reflection. A second renderer beside the normal view on the camera image:
# both run the same garment code off the same perception service, so whatever is improved in the
# camera view shows on the mirror too.
#
#   ./run_perception.sh           (once, as usual)
#   ./run_mirror_output.sh        (this; move the window to the mirror's display. Esc quits)
#
# The mirror's display size and where the camera sits on it (position and downward tilt) are
# "screen" and "camera_pose" in perception/calib/config.json. Extra arguments go to the renderer,
# e.g. -windowed -ResX=540 -ResY=960 to try it in a window.
cd "$(dirname "$0")"
UE_ROOT="${UE_ROOT:-/Users/Shared/Epic Games/UE_5.8}"
APP=unreal/Build/Mac/ReflectAR.app
[ $# -eq 0 ] && set -- -fullscreen
if [ -d "$APP" ]; then
    exec "$APP/Contents/MacOS/ReflectAR" -ReflectARMirrorOutput "$@"
fi
exec "$UE_ROOT/Engine/Binaries/Mac/UnrealEditor.app/Contents/MacOS/UnrealEditor" "$PWD/unreal/ReflectAR.uproject" \
    -game -nosplash -ReflectARMirrorOutput "$@"
