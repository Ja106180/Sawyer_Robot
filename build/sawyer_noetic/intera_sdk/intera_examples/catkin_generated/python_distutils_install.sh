#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/mycar/catkin_ws/src/sawyer_noetic/intera_sdk/intera_examples"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/mycar/catkin_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/mycar/catkin_ws/install/lib/python3/dist-packages:/home/mycar/catkin_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/mycar/catkin_ws/build" \
    "/usr/bin/python3" \
    "/home/mycar/catkin_ws/src/sawyer_noetic/intera_sdk/intera_examples/setup.py" \
    egg_info --egg-base /home/mycar/catkin_ws/build/sawyer_noetic/intera_sdk/intera_examples \
    build --build-base "/home/mycar/catkin_ws/build/sawyer_noetic/intera_sdk/intera_examples" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/mycar/catkin_ws/install" --install-scripts="/home/mycar/catkin_ws/install/bin"
