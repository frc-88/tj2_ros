v4l2-ctl -d /dev/video0 -c frame_rate=60
v4l2-ctl -d /dev/video0 -c frame_timeout=200
v4l2-ctl -d /dev/video0 -c low_latency_mode=0
v4l2-ctl -d /dev/video0 -c exposure=1000
v4l2-ctl -d /dev/video0 -c analogue_gain=200

# v4l2-ctl -d /dev/video0 --all
# Driver Info (not using libv4l2):
# 	Driver name   : tegra-video
# 	Card type     : vi-output, arducam-csi2 10-000c
# 	Bus info      : platform:15c10000.vi:2
# 	Driver version: 4.9.253
# 	Capabilities  : 0x84200001
# 		Video Capture
# 		Streaming
# 		Extended Pix Format
# 		Device Capabilities
# 	Device Caps   : 0x04200001
# 		Video Capture
# 		Streaming
# 		Extended Pix Format
# Priority: 2
# Video input : 0 (Camera 2: ok)
# Format Video Capture:
# 	Width/Height      : 5120/800
# 	Pixel Format      : 'GREY'
# 	Field             : None
# 	Bytes per Line    : 5120
# 	Size Image        : 4096000
# 	Colorspace        : sRGB
# 	Transfer Function : Default (maps to sRGB)
# 	YCbCr/HSV Encoding: Default (maps to ITU-R 601)
# 	Quantization      : Default (maps to Full Range)
# 	Flags             : 

# User Controls

#                        exposure 0x00980911 (int)    : min=1 max=65523 step=1 default=681 value=50
#                 horizontal_flip 0x00980914 (bool)   : default=0 value=0
#                   vertical_flip 0x00980915 (bool)   : default=0 value=0
#                    trigger_mode 0x00981901 (bool)   : default=0 value=0
#           disable_frame_timeout 0x00981902 (bool)   : default=0 value=0
#                   frame_timeout 0x00981903 (int)    : min=100 max=12000 step=1 default=2000 value=200
#                      frame_rate 0x00981906 (int)    : min=5 max=60 step=1 default=50 value=60

# Camera Controls

#            sensor_configuration 0x009a2032 (u32)    : min=0 max=0 step=0 default=0 flags=read-only, volatile, has-payload
#          sensor_mode_i2c_packet 0x009a2033 (u32)    : min=0 max=0 step=0 default=0 flags=read-only, volatile, has-payload
#       sensor_control_i2c_packet 0x009a2034 (u32)    : min=0 max=0 step=0 default=0 flags=read-only, volatile, has-payload
#                     bypass_mode 0x009a2064 (intmenu): min=0 max=1 default=0 value=0
#                 override_enable 0x009a2065 (intmenu): min=0 max=1 default=0 value=0
#                    height_align 0x009a2066 (int)    : min=1 max=16 step=1 default=1 value=1
#                      size_align 0x009a2067 (intmenu): min=0 max=2 default=0 value=0
#                write_isp_format 0x009a2068 (int)    : min=1 max=1 step=1 default=1 value=1
#        sensor_signal_properties 0x009a2069 (u32)    : min=0 max=0 step=0 default=0 flags=read-only, has-payload
#         sensor_image_properties 0x009a206a (u32)    : min=0 max=0 step=0 default=0 flags=read-only, has-payload
#       sensor_control_properties 0x009a206b (u32)    : min=0 max=0 step=0 default=0 flags=read-only, has-payload
#               sensor_dv_timings 0x009a206c (u32)    : min=0 max=0 step=0 default=0 flags=read-only, has-payload
#                low_latency_mode 0x009a206d (bool)   : default=0 value=1
#                preferred_stride 0x009a206e (int)    : min=0 max=65535 step=1 default=0 value=0
#                    sensor_modes 0x009a2082 (int)    : min=0 max=30 step=1 default=30 value=1 flags=read-only

# Image Source Controls

#               vertical_blanking 0x009e0901 (int)    : min=0 max=0 step=1 default=0 value=0
#             horizontal_blanking 0x009e0902 (int)    : min=0 max=0 step=1 default=0 value=0
#                   analogue_gain 0x009e0903 (int)    : min=100 max=1590 step=1 default=100 value=1000

# Image Processing Controls

#                      pixel_rate 0x009f0902 (int64)  : min=0 max=0 step=0 default=0 value=0 flags=read-only

# v4l2-ctl -d /dev/video0 --list-formats-ext
# ioctl: VIDIOC_ENUM_FMT
# 	Index       : 0
# 	Type        : Video Capture
# 	Pixel Format: 'GREY'
# 	Name        : 8-bit Greyscale
# 		Size: Discrete 5120x800
# 		Size: Discrete 5120x720
# 		Size: Discrete 2560x400

# 	Index       : 1
# 	Type        : Video Capture
# 	Pixel Format: 'Y10 '
# 	Name        : 10-bit Greyscale
# 		Size: Discrete 5120x800
# 		Size: Discrete 5120x720
# 		Size: Discrete 2560x400

# 	Index       : 2
# 	Type        : Video Capture
# 	Pixel Format: 'Y16 '
# 	Name        : 16-bit Greyscale
# 		Size: Discrete 5120x800
# 		Size: Discrete 5120x720
# 		Size: Discrete 2560x400

