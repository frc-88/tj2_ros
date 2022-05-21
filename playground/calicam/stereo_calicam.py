import os
import cv2
import math
import pickle
import numpy as np

# https://stackoverflow.com/questions/56212229/how-to-improve-stereosgbm-results-with-texture
# https://github.com/k22jung/stereo_vision/blob/master/src/stereo_vision.cpp
# https://stackoverflow.com/questions/62627109/how-do-you-use-opencvs-disparitywlsfilter-in-python
# https://github.com/tobybreckon/python-examples-cv/blob/master/stereo_sgbm.py



class StereoCaliCam:
    def __init__(self, capture_num, param_path):
        self.rectify_map_path = param_path + ".pkl"
        self.capture_num = capture_num
        self.capture = cv2.VideoCapture(self.capture_num)

        self.mode = "kRectPerspective"
        # self.mode = "kRectLonglat"
        # self.mode = "kRectFisheye"
        self.Kl = np.array([])
        self.Dl = np.array([])
        self.Rl = np.array([])
        self.xil = np.array([])
        self.Kr = np.array([])
        self.Dr = np.array([])
        self.Rr = np.array([])
        self.xir = np.array([])
        self.T = np.array([])
        self.cam_model = ""
        self.cap_cols = 0
        self.cap_rows = 0
        self.img_width = 0

        self.current_vfov = 140.0
        self.current_width = 640
        self.current_height = 480
        self.current_ndisp = 32
        self.current_wsize = 7
        self.max_speckle_size = 5000

        self.baseline = 0.085  # distance between cameras in meters

        self.mapx = [None, None]
        self.mapy = [None, None]

        self.smoothing_factor = 2
        self.max_x_grad = 25

        self.stereo = cv2.StereoSGBM_create(0, self.current_ndisp, self.current_wsize)
        self.stereo.setP1(24 * self.current_wsize * self.current_wsize * self.smoothing_factor)
        self.stereo.setP2(96 * self.current_wsize * self.current_wsize * self.smoothing_factor)
        self.stereo.setPreFilterCap(self.max_x_grad)
        self.stereo.setMode(cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        self.right_matcher = cv2.ximgproc.createRightMatcher(self.stereo)
        self.wls_filter = cv2.ximgproc.createDisparityWLSFilter(self.stereo)
        self.wls_filter.setLambda(8000.0)
        self.wls_filter.setSigmaColor(0.1)

        self.load_parameters(param_path)
        self.load_rectify_map()

        # it seems setting this to full resolution causes USB errors.
        # I'm setting the camera to its lower resolution and up scaling it later
        # so the camera parameters still apply
        # self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.cap_cols)
        # self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cap_rows)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        read_width = self.capture.get(cv2.CAP_PROP_FRAME_WIDTH)
        read_height = self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
        print(read_width, read_height)
        # assert read_width == self.cap_cols, read_width
        # assert read_height == self.cap_rows, read_height

    def load_parameters(self, param_path):
        fs = cv2.FileStorage(param_path, cv2.FILE_STORAGE_READ)

        self.cam_model = fs.getNode("cam_model").string()
        cap_size_node = fs.getNode("cap_size")
        self.cap_cols = int(cap_size_node.at(0).real())
        self.cap_rows = int(cap_size_node.at(1).real())
        # self.img_width = self.cap_cols

        self.Kl = fs.getNode("Kl").mat()
        self.Dl = fs.getNode("Dl").mat()
        self.Rl = np.identity(3, dtype=np.float64)
        self.xil = fs.getNode("xil").mat()

        self.Rl = fs.getNode("Rl").mat()

        self.Kr = fs.getNode("Kr").mat()
        self.Dr = fs.getNode("Dr").mat()
        self.Rr = fs.getNode("Rr").mat()
        self.xir = fs.getNode("xir").mat()

        self.T = fs.getNode("T").mat()

        self.img_width = self.cap_cols // 2

    def init_undistort_rectify_map(self, k, d, r, knew, xi0, size, mode):
        fx = k[0, 0]
        fy = k[1, 1]
        cx = k[0, 2]
        cy = k[1, 2]
        s = k[0, 1]

        k1 = d[0, 0]
        k2 = d[0, 1]
        p1 = d[0, 2]
        p2 = d[0, 3]

        ki = np.linalg.inv(knew)
        ri = np.linalg.inv(r)
        kri = np.linalg.inv(np.matmul(knew, r))

        rows = size[0]
        cols = size[1]

        mapx = np.zeros((rows, cols), dtype=np.float32)
        mapy = np.zeros((rows, cols), dtype=np.float32)

        print("Wait, this takes a while ... ")
        for r in range(rows):
            for c in range(cols):
                xc = 0.0
                yc = 0.0
                zc = 0.0

                if mode == 'kRectPerspective':
                    cr1 = np.array([c, r, 1.])
                    xc = np.dot(kri[0, :], cr1)
                    yc = np.dot(kri[1, :], cr1)
                    zc = np.dot(kri[2, :], cr1)

                if mode == 'kRectLonglat':
                    tt = (c * 1. / (cols - 1) - 0.5) * math.pi
                    pp = (r * 1. / (rows - 1) - 0.5) * math.pi

                    xn = math.sin(tt)
                    yn = math.cos(tt) * math.sin(pp)
                    zn = math.cos(tt) * math.cos(pp)

                    cr1 = np.array([xn, yn, zn])
                    xc = np.dot(ri[0, :], cr1)
                    yc = np.dot(ri[1, :], cr1)
                    zc = np.dot(ri[2, :], cr1)

                if mode == 'kRectFisheye':
                    cr1 = np.array([c, r, 1.])
                    ee = np.dot(ki[0, :], cr1)
                    ff = np.dot(ki[1, :], cr1)
                    zz = 2. / (ee * ee + ff * ff + 1.)

                    xn = zz * ee
                    yn = zz * ff
                    zn = zz - 1.

                    cr1 = np.array([xn, yn, zn])
                    xc = np.dot(ri[0, :], cr1)
                    yc = np.dot(ri[1, :], cr1)
                    zc = np.dot(ri[2, :], cr1)

                if zc < 0.0:
                    mapx[r, c] = np.float32(-1.)
                    mapy[r, c] = np.float32(-1.)

                    continue

                rr = math.sqrt(xc * xc + yc * yc + zc * zc)
                xs = xc / rr
                ys = yc / rr
                zs = zc / rr

                xu = xs / (zs + xi0)
                yu = ys / (zs + xi0)

                r2 = xu * xu + yu * yu
                r4 = r2 * r2
                xd = (1 + k1 * r2 + k2 * r4) * xu + 2 * p1 * xu * yu + p2 * (r2 + 2 * xu * xu)
                yd = (1 + k1 * r2 + k2 * r4) * yu + 2 * p2 * xu * yu + p1 * (r2 + 2 * yu * yu)

                u = fx * xd + s * yd + cx
                v = fy * yd + cy

                mapx[r, c] = np.float32(u)
                mapy[r, c] = np.float32(v)

        return mapx, mapy

    def load_rectify_map(self):
        if os.path.isfile(self.rectify_map_path):
            with open(self.rectify_map_path, 'rb') as file:
                map_settings = pickle.load(file)
        else:
            map_settings = {}
        if self.did_settings_change(map_settings):
            self.init_rectify_map()
            map_settings = self.make_mapping_settings()
            with open(self.rectify_map_path, 'wb') as file:
                pickle.dump(map_settings, file)
        else:
            self.load_mapping_settings(map_settings)

    def did_settings_change(self, map_settings):
        comparison_map = {
            "current_vfov": self.current_vfov,
            "current_height": self.current_height,
            "current_width": self.current_width,
            "mode": self.mode,
        }
        for key, value in comparison_map.items():
            if value != map_settings.get(key, None):
                return True
        return False

    def load_mapping_settings(self, map_settings):
        self.current_vfov = map_settings.get("current_vfov")
        self.current_height = map_settings.get("current_height")
        self.current_width = map_settings.get("current_width")
        self.mapx = map_settings.get("mapx")
        self.mapy = map_settings.get("mapy")
        self.Kl = map_settings.get("Kl")
        self.Dl = map_settings.get("Dl")
        self.Rl = map_settings.get("Rl")
        self.xil = map_settings.get("xil")
        self.Kr = map_settings.get("Kr")
        self.Dr = map_settings.get("Dr")
        self.Rr = map_settings.get("Rr")
        self.xir = map_settings.get("xir")
        self.mode = map_settings.get("mode")

    def make_mapping_settings(self):
        map_settings = {}
        map_settings["current_vfov"] = self.current_vfov
        map_settings["current_height"] = self.current_height
        map_settings["current_width"] = self.current_width
        map_settings["mapx"] = self.mapx
        map_settings["mapy"] = self.mapy
        map_settings["Kl"] = self.Kl
        map_settings["Dl"] = self.Dl
        map_settings["Rl"] = self.Rl
        map_settings["xil"] = self.xil
        map_settings["Kr"] = self.Kr
        map_settings["Dr"] = self.Dr
        map_settings["Rr"] = self.Rr
        map_settings["xir"] = self.xir
        map_settings["mode"] = self.mode
        return map_settings

    def init_rectify_map(self):
        vfov_rad = math.radians(self.current_vfov)
        focal = self.current_height * 0.5 / math.tan(vfov_rad * 0.5)

        Knew = np.identity(3, dtype=np.float64)
        Knew[0, 0] = focal
        Knew[1, 1] = focal
        Knew[0, 2] = (self.current_width - 1.0) * 0.5
        Knew[1, 2] = (self.current_height - 1.0) * 0.5

        img_size = [self.current_height, self.current_width]

        self.mapx[0], self.mapy[0] = self.init_undistort_rectify_map(
            self.Kl, self.Dl, self.Rl, Knew, self.xil, img_size, 'kRectPerspective')

        print('Width: {}, Height: {}, V.FoV: {}'.format(self.current_width, self.current_height, self.current_vfov))
        print('K Matrix:')
        print(Knew)

        self.mapx[1], self.mapy[1] = self.init_undistort_rectify_map(
            self.Kr, self.Dr, self.Rr, Knew, self.xir, img_size, 'kRectPerspective')
        print('Ndisp: {}, Wsize: {}'.format(self.current_ndisp, self.current_wsize))

        print('')

    def disparity_image(self, rect_imgl, rect_imgr):
        gray_imgl = cv2.cvtColor(rect_imgl, cv2.COLOR_BGR2GRAY)
        gray_imgr = cv2.cvtColor(rect_imgr, cv2.COLOR_BGR2GRAY)

        left_disparity = self.stereo.compute(gray_imgl, gray_imgr)
        right_disparity = self.right_matcher.compute(gray_imgr, gray_imgl)
        disparity = self.wls_filter.filter(left_disparity, gray_imgl, disparity_map_right=right_disparity)

        # cv2.filterSpeckles(disparity, 0, self.max_speckle_size, self.current_ndisp)
        return disparity

    def normalize_disparity_float(self, disparity):
        return cv2.normalize(disparity, None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)

    def normalize_disparity_int(self, disparity):
        disparity = np.clip(disparity, 0, None)
        return cv2.normalize(disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

    def get_raw(self):
        success, frame = self.capture.read()
        return success, cv2.resize(frame, (self.cap_cols, self.cap_rows), interpolation=cv2.INTER_NEAREST)
        # return success, frame

    def get_images(self):
        success, raw_img = self.get_raw()
        if not success:
            return None, None, success
        if raw_img.shape[0:2] != (self.cap_rows, self.cap_cols):
            raise RuntimeError("Capture size doesn't match config! %s != %s" % (raw_img.shape[0:2], (self.cap_rows, self.cap_cols)))
        raw_imgl = raw_img[:, : self.img_width]
        raw_imgr = raw_img[:, self.img_width: self.img_width * 2]

        rect_imgl = cv2.remap(raw_imgl, self.mapx[0], self.mapy[0], cv2.INTER_LINEAR)
        rect_imgr = cv2.remap(raw_imgr, self.mapx[1], self.mapy[1], cv2.INTER_LINEAR)
        return rect_imgl, rect_imgr, success

    def get_distance(self, disparity, px, py):
        fx = self.Kl[0, 0]
        return self.baseline * fx / disparity[py, px]
