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
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/zheng/ros/src/crazyflie-clients-python"

# snsure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/zheng/ros/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/zheng/ros/install/lib/python2.7/dist-packages:/home/zheng/ros/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/zheng/ros/build" \
    "/usr/bin/python" \
    "/home/zheng/ros/src/crazyflie-clients-python/setup.py" \
    build --build-base "/home/zheng/ros/build/crazyflie-clients-python" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/zheng/ros/install" --install-scripts="/home/zheng/ros/install/bin"
