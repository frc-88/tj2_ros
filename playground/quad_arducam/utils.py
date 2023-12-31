import os
from enum import Enum
from typing import Optional, Tuple
import v4l2
import fcntl
import ctypes
import cv2
import numpy as np

_IOC_NRBITS = 8
_IOC_TYPEBITS = 8
_IOC_SIZEBITS = 14
_IOC_DIRBITS = 2

_IOC_NRSHIFT = 0
_IOC_TYPESHIFT = _IOC_NRSHIFT + _IOC_NRBITS
_IOC_SIZESHIFT = _IOC_TYPESHIFT + _IOC_TYPEBITS
_IOC_DIRSHIFT = _IOC_SIZESHIFT + _IOC_SIZEBITS

_IOC_NONE = 0
_IOC_WRITE = 1
_IOC_READ = 2


def _IOC(dir_, type_, nr, size):
    return (
        ctypes.c_int32(dir_ << _IOC_DIRSHIFT).value
        | ctypes.c_int32(ord(type_) << _IOC_TYPESHIFT).value
        | ctypes.c_int32(nr << _IOC_NRSHIFT).value
        | ctypes.c_int32(size << _IOC_SIZESHIFT).value
    )


def _IOC_TYPECHECK(t):
    return ctypes.sizeof(t)


def _IO(type_, nr):
    return _IOC(_IOC_NONE, type_, nr, 0)


def _IOW(type_, nr, size):
    return _IOC(_IOC_WRITE, type_, nr, _IOC_TYPECHECK(size))


def _IOR(type_, nr, size):
    return _IOC(_IOC_READ, type_, nr, _IOC_TYPECHECK(size))


def _IOWR(type_, nr, size):
    return _IOC(_IOC_READ | _IOC_WRITE, type_, nr, _IOC_TYPECHECK(size))


BASE_VIDIOC_PRIVATE = 192


class arducam_i2c(ctypes.Structure):
    _fields_ = [
        ("reg", ctypes.c_uint16),
        ("val", ctypes.c_uint16),
    ]


class arducam_dev(ctypes.Structure):
    _fields_ = [
        ("reg", ctypes.c_uint16),
        ("val", ctypes.c_uint32),
    ]


VIDIOC_R_I2C = _IOWR("V", BASE_VIDIOC_PRIVATE + 0, arducam_i2c)
VIDIOC_W_I2C = _IOWR("V", BASE_VIDIOC_PRIVATE + 1, arducam_i2c)
VIDIOC_R_DEV = _IOWR("V", BASE_VIDIOC_PRIVATE + 2, arducam_dev)
VIDIOC_W_DEV = _IOWR("V", BASE_VIDIOC_PRIVATE + 3, arducam_dev)


def to_fourcc(a: str, b: str, c: str, d: str) -> int:
    return ord(a) | (ord(b) << 8) | (ord(c) << 16) | (ord(d) << 24)


def to_format_code(string: str) -> int:
    if len(string) == 3:
        return to_fourcc(string[0], string[1], string[2], " ")
    elif len(string) == 4:
        return to_fourcc(string[0], string[1], string[2], string[3])
    else:
        raise ValueError(f"{string} is not a pixel format")


class ArduCamPixelFormat(Enum):
    RAW8 = "raw8"
    RAW10 = "raw10"


FORMAT_FOURCC_MAPPING = {
    "raw8": "GREY",
    "raw10": "Y16",
}


class Arducam:
    NUM_CAMERAS = 4
    pixfmt_map = {
        v4l2.V4L2_PIX_FMT_SBGGR10: {
            "depth": 10,
            "cvt_code": cv2.COLOR_BAYER_RG2BGR,
            "convert2rgb": 0,
        },
        v4l2.V4L2_PIX_FMT_SGBRG10: {
            "depth": 10,
            "cvt_code": cv2.COLOR_BAYER_GR2BGR,
            "convert2rgb": 0,
        },
        v4l2.V4L2_PIX_FMT_SGRBG10: {
            "depth": 10,
            "cvt_code": cv2.COLOR_BAYER_GB2BGR,
            "convert2rgb": 0,
        },
        v4l2.V4L2_PIX_FMT_SRGGB10: {
            "depth": 10,
            "cvt_code": cv2.COLOR_BAYER_BG2BGR,
            "convert2rgb": 0,
        },
        v4l2.V4L2_PIX_FMT_Y10: {"depth": 10, "cvt_code": -1, "convert2rgb": 0},
    }
    pixfmt_map_xavier_nx = {
        v4l2.V4L2_PIX_FMT_SBGGR10: {
            "depth": 16,
            "cvt_code": cv2.COLOR_BAYER_RG2BGR,
            "convert2rgb": 0,
        },
        v4l2.V4L2_PIX_FMT_SGBRG10: {
            "depth": 16,
            "cvt_code": cv2.COLOR_BAYER_GR2BGR,
            "convert2rgb": 0,
        },
        v4l2.V4L2_PIX_FMT_SGRBG10: {
            "depth": 16,
            "cvt_code": cv2.COLOR_BAYER_GB2BGR,
            "convert2rgb": 0,
        },
        v4l2.V4L2_PIX_FMT_SRGGB10: {
            "depth": 16,
            "cvt_code": cv2.COLOR_BAYER_BG2BGR,
            "convert2rgb": 0,
        },
        v4l2.V4L2_PIX_FMT_Y10: {"depth": 16, "cvt_code": -1, "convert2rgb": 0},
    }

    pixfmt_map_raw8 = {
        v4l2.V4L2_PIX_FMT_SBGGR8: {
            "depth": 8,
            "cvt_code": cv2.COLOR_BAYER_RG2BGR,
            "convert2rgb": 0,
        },
        v4l2.V4L2_PIX_FMT_SGBRG8: {
            "depth": 8,
            "cvt_code": cv2.COLOR_BAYER_GR2BGR,
            "convert2rgb": 0,
        },
        v4l2.V4L2_PIX_FMT_SGRBG8: {
            "depth": 8,
            "cvt_code": cv2.COLOR_BAYER_GB2BGR,
            "convert2rgb": 0,
        },
        v4l2.V4L2_PIX_FMT_SRGGB8: {
            "depth": 8,
            "cvt_code": cv2.COLOR_BAYER_BG2BGR,
            "convert2rgb": 0,
        },
    }

    AUTO_CONVERT_TO_RGB = {"depth": -1, "cvt_code": -1, "convert2rgb": 1}

    DEVICE_REG_BASE = 0x0100
    PIXFORMAT_REG_BASE = 0x0200
    FORMAT_REG_BASE = 0x0300
    CTRL_REG_BASE = 0x0400
    SENSOR_REG_BASE = 0x500

    STREAM_ON = DEVICE_REG_BASE | 0x0000
    FIRMWARE_VERSION_REG = DEVICE_REG_BASE | 0x0001
    SENSOR_ID_REG = DEVICE_REG_BASE | 0x0002
    DEVICE_ID_REG = DEVICE_REG_BASE | 0x0003
    FIRMWARE_SENSOR_ID_REG = DEVICE_REG_BASE | 0x0005
    SERIAL_NUMBER_REG = DEVICE_REG_BASE | 0x0006
    CHANNEL_SWITCH_REG = DEVICE_REG_BASE | 0x0008

    PIXFORMAT_INDEX_REG = PIXFORMAT_REG_BASE | 0x0000
    PIXFORMAT_TYPE_REG = PIXFORMAT_REG_BASE | 0x0001
    PIXFORMAT_ORDER_REG = PIXFORMAT_REG_BASE | 0x0002
    MIPI_LANES_REG = PIXFORMAT_REG_BASE | 0x0003

    RESOLUTION_INDEX_REG = FORMAT_REG_BASE | 0x0000
    FORMAT_WIDTH_REG = FORMAT_REG_BASE | 0x0001
    FORMAT_HEIGHT_REG = FORMAT_REG_BASE | 0x0002

    CTRL_INDEX_REG = CTRL_REG_BASE | 0x0000
    CTRL_ID_REG = CTRL_REG_BASE | 0x0001
    CTRL_MIN_REG = CTRL_REG_BASE | 0x0002
    CTRL_MAX_REG = CTRL_REG_BASE | 0x0003
    CTRL_STEP_REG = CTRL_REG_BASE | 0x0004
    CTRL_DEF_REG = CTRL_REG_BASE | 0x0005
    CTRL_VALUE_REG = CTRL_REG_BASE | 0x0006

    SENSOR_RD_REG = SENSOR_REG_BASE | 0x0001
    SENSOR_WR_REG = SENSOR_REG_BASE | 0x0002

    NO_DATA_AVAILABLE = 0xFFFFFFFE

    DEVICE_ID = 0x0030

    def __init__(
        self,
        device_num: int,
        pixel_format: ArduCamPixelFormat,
        width: int = -1,
        height: int = -1,
        channel: int = -1,
    ):
        self.device_num = device_num
        if self.is_nx():
            Arducam.pixfmt_map = Arducam.pixfmt_map_xavier_nx

        # set in refresh(), see __getattr__
        self.depth = -1
        self.cvt_code = -1
        self.convert2rgb = 0

        self.cv_capture = self.opencv_capture(self.device_num, pixel_format)
        self.vd = open("/dev/video{}".format(self.device_num), "w")
        self.refresh()

        if self.convert2rgb == 0:
            self.cv_capture.set(cv2.CAP_PROP_CONVERT_RGB, self.convert2rgb)
        # set width
        if width > 0:
            self.cv_capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        # set height
        if height > 0:
            self.cv_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        if channel in range(0, 4):
            self.write_dev(Arducam.CHANNEL_SWITCH_REG, channel)

    def is_nx(self) -> bool:
        path = "/proc/device-tree/model"
        if not os.path.isfile(path):
            return False
        with open(path, "r") as file:
            contents = file.read()
        return "NVIDIA" in contents and "NX" in contents

    def opencv_capture(
        self, device_num: int, pixel_format: ArduCamPixelFormat
    ) -> cv2.VideoCapture:
        capture = cv2.VideoCapture(device_num, cv2.CAP_V4L2)

        code = to_format_code(FORMAT_FOURCC_MAPPING[pixel_format.value])
        capture.set(cv2.CAP_PROP_FOURCC, code)
        return capture

    def get_combined(self) -> Optional[np.ndarray]:
        success, frame = self.cv_capture.read()
        if not success:
            return None

        if self.convert2rgb == 0:
            width = self.cv_capture.get(cv2.CAP_PROP_FRAME_WIDTH)
            height = self.cv_capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
            frame = frame.reshape(int(height), int(width))

        frame = self.convert(frame)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return frame

    def split_frame(self, frame: np.ndarray) -> Tuple[np.ndarray, ...]:
        height, width = frame.shape[0:2]

        split_frames = []
        single_width = width // self.NUM_CAMERAS
        for index in range(self.NUM_CAMERAS):
            x0 = int(single_width * index)
            x1 = int(single_width * (index + 1))
            split_frames.append(frame[:, x0:x1])

        return tuple(split_frames)

    def refresh(self):
        config = self.get_pixfmt_cfg()
        self.depth = int(config.get("depth"))  # type: ignore
        self.cvt_code = int(config.get("cvt_code"))  # type: ignore
        self.convert2rgb = int(config.get("convert2rgb"))  # type: ignore

    def read_sensor(self, reg):
        i2c = arducam_i2c()
        i2c.reg = reg
        fcntl.ioctl(self.vd, VIDIOC_R_I2C, i2c)
        return i2c.val

    def write_sensor(self, reg, val):
        i2c = arducam_i2c()
        i2c.reg = reg
        i2c.val = val
        return fcntl.ioctl(self.vd, VIDIOC_W_I2C, i2c)

    def read_dev(self, reg):
        dev = arducam_dev()
        dev.reg = reg
        ret = fcntl.ioctl(self.vd, VIDIOC_R_DEV, dev)
        return ret, dev.val

    def write_dev(self, reg, val):
        dev = arducam_dev()
        dev.reg = reg
        dev.val = val
        return fcntl.ioctl(self.vd, VIDIOC_W_DEV, dev)

    def get_device_info(self):
        return {
            "fw_sensor_id": self.read_dev(Arducam.FIRMWARE_SENSOR_ID_REG),
            "sensor_id": self.read_dev(Arducam.SENSOR_ID_REG),
            "fw_version": self.read_dev(Arducam.FIRMWARE_VERSION_REG),
            "serial_number": self.read_dev(Arducam.SERIAL_NUMBER_REG),
        }

    def show_info(self):
        info = self.get_device_info()
        _, firmware_version = info["fw_version"]
        _, sensor_id = info["sensor_id"]
        _, serial_number = info["serial_number"]
        print("Firmware Version: {}".format(firmware_version))
        print("Sensor ID: 0x{:04X}".format(sensor_id))
        print("Serial Number: 0x{:08X}".format(serial_number))

    def convert(self, frame):
        if self.convert2rgb == 1:
            return frame

        if self.depth != -1:
            frame = cv2.convertScaleAbs(frame, alpha=256.0 / (1 << self.depth))
            frame = frame.astype(np.uint8)

        if self.cvt_code != -1:
            frame = cv2.cvtColor(frame, self.cvt_code)

        return frame

    def get_pixelformat(self):
        fmt = v4l2.v4l2_format()
        fmt.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
        ret = fcntl.ioctl(self.vd, v4l2.VIDIOC_G_FMT, fmt)
        return ret, fmt.fmt.pix.pixelformat

    # Find the actual pixel format
    def get_pixfmt_cfg(self):
        ret, pixfmt = self.get_pixelformat()

        pf = Arducam.pixfmt_map_raw8.get(pixfmt, None)
        if pf is not None:
            return pf

        if pixfmt != v4l2.V4L2_PIX_FMT_Y16:
            return Arducam.AUTO_CONVERT_TO_RGB
        fmtdesc = v4l2.v4l2_fmtdesc()
        fmtdesc.index = 0
        fmtdesc.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
        while True:
            try:
                fcntl.ioctl(self.vd, v4l2.VIDIOC_ENUM_FMT, fmtdesc)
                pixfmt = Arducam.pixfmt_map.get(fmtdesc.pixelformat, None)
                if pixfmt is not None:
                    return pixfmt
                fmtdesc.index += 1
            except Exception:
                break
        return Arducam.AUTO_CONVERT_TO_RGB

    def get_pixelformats(self):
        pixfmts = []
        fmtdesc = v4l2.v4l2_fmtdesc()
        fmtdesc.index = 0
        fmtdesc.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
        while True:
            try:
                fcntl.ioctl(self.vd, v4l2.VIDIOC_ENUM_FMT, fmtdesc)
                pixfmts.append((fmtdesc.pixelformat, fmtdesc.description))
                fmtdesc.index += 1
            except Exception:
                break
        return pixfmts

    def get_framesizes(self, pixel_format=v4l2.V4L2_PIX_FMT_Y16):
        framesizes = []
        framesize = v4l2.v4l2_frmsizeenum()
        framesize.index = 0
        framesize.pixel_format = pixel_format
        while True:
            try:
                fcntl.ioctl(self.vd, v4l2.VIDIOC_ENUM_FRAMESIZES, framesize)
                framesizes.append((framesize.discrete.width, framesize.discrete.height))
                framesize.index += 1
            except Exception:
                break
        return framesizes

    def set_focus(self, value: float) -> None:
        if not (0.0 <= value <= 1.0):
            raise ValueError("Focus value must be between 0.0 and 1.0")
        focus_value = int(value * 1000.0)
        os.system(
            f"v4l2-ctl -d /dev/v4l-subdev{self.device_num} -c focus_absolute={focus_value}"
        )

    def close(self):
        self.cv_capture.release()
