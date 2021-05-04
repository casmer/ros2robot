#!/bin/sh
. /opt/cotsbot/cotsbot_env_vars.sh
/opt/cotsbot/bin/simple-rtsp-server "( v4l2src device=${COTSBOT_MAIN_VIDEO_DEV} !  rtph264pay name=pay0 pt=96 )"
