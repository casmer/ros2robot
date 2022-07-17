# ros2robot
This is a simple two wheeled robot controlled by a remote terminal using an xbox controller. 

I have it working on two platforms currently.
1. a Tank base from amazon with an RPI and a Motor Hat.
2. a Jazzy wheelchair base witn an RPI and a 30A motor drive controller.

Video is served up via a simple rtsp server using gstreamer.
Video is played on the controlling computer using gst-launch and playbin for low latency video.
The Drive controls are on an xbox controller connected to the controlling computer.
