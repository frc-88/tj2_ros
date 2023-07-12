# This ROS node connects to one or more UVC cameras, applies the requested modees and controls,
# and publishes the images to ROS topics. In addition, it will take the supplied camera info if provided and
# publish it to the camera_info topic.

# Node tasks:
#   match device info to address
#   apply frame mode
#   set controls
#   connect
#   get frame
#   publish
