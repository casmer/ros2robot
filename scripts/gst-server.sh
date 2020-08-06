#!/bin/bash
#gst-launch-1.0 v4l2src device=/dev/video0 ! image/jpeg,width=640,height=480,framerate=30/1 ! jpegparse ! rtpjpegpay ! application/x-rtp, width=640, height=480, framerate=30/1, media=video, payload=26, clock-rate=90000, encoding-name=JPEG ! udpsink host=192.168.0.25 port=8090
export GST_DEBUG=3
./test-launch "( v4l2src device=/dev/video0 !  rtph264pay name=pay0 pt=96 )"
