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

echo_and_run cd "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/src/dynamixel_motor/dynamixel_controllers"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/install/lib/python2.7/dist-packages:/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/build/dynamixel_controllers/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/build/dynamixel_controllers" \
    "/usr/bin/python2" \
    "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/src/dynamixel_motor/dynamixel_controllers/setup.py" \
     \
    build --build-base "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/build/dynamixel_controllers" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/install" --install-scripts="/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/install/bin"
