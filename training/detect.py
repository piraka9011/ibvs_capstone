from ctypes import *
import math
import random
import cv2
import time
import numpy as np
import sys
import os

class BOX(Structure):
    _fields_ = [("x", c_float),
                ("y", c_float),
                ("w", c_float),
                ("h", c_float)]

class DETECTION(Structure):
    _fields_ = [("bbox", BOX),
                ("classes", c_int),
                ("prob", POINTER(c_float)),
                ("mask", POINTER(c_float)),
                ("objectness", c_float),
                ("sort_class", c_int)]


class IMAGE(Structure):
    _fields_ = [("w", c_int),
                ("h", c_int),
                ("c", c_int),
                ("data", POINTER(c_float))]

class METADATA(Structure):
    _fields_ = [("classes", c_int),
                ("names", POINTER(c_char_p))]

class YOLODetector:
    __instance = None

    def __init__(self):
        # (cfg_path, meta_path, object_weight_path)
        self.model_to_path = {
            "yolo3_tiny": ("yolo3-obj-tiny.cfg", "obj.data", "yolo3-obj-tiny_10000.weights"),
            "yolo3": ("yolo3-objv2.cfg", "objv2-py.data", "yolo3-objv2_10000.weights"),
            "yolo3_bunny_plushie": ("yolo3_bunny_plushie.cfg", "obj_bunny_plushie.data", "yolo3_bunny_plushie_10000.weights"),
            "yolo3_bunny_plushie_tiny": ("yolo3_bunny_plushie-tiny.cfg", "obj_bunny_plushie.data", "yolo3-tiny_bunny_plushie_10000.weights"),
            "yolo3_final": ("yolo3-5th.cfg", "obj_v5.data", "yolo3-5th_30000.weights")
        }
        self.load_net()

    def _get_path(self, model_name):
        try:
            _cfg_path, _meta_path, _object_weight_path = self.model_to_path[model_name]
            cfg_path = os.path.abspath(os.path.join(__file__, '..', 'darknet', 'cfg', _cfg_path))
            meta_path = os.path.abspath(os.path.join(__file__, '..', 'darknet', 'cfg', _meta_path))
            object_weight_path = os.path.abspath(os.path.join(__file__, '..', 'darknet', 'backup', _object_weight_path))

            return cfg_path, meta_path, object_weight_path
        except Exception as e:
            print "Cannot get model path! Error: {}".format(e.message)
            raise

    # Singleton Pattern
    def __new__(cls):
        if YOLODetector.__instance is None:
            YOLODetector.__instance = object.__new__(cls)
        return YOLODetector.__instance

    def load_net(self):
        libdarknet_path = os.path.abspath(os.path.join(__file__, '..', 'darknet', 'libdarknet.so'))
        self.lib = CDLL(libdarknet_path, RTLD_GLOBAL)
        self.lib.network_width.argtypes = [c_void_p]
        self.lib.network_width.restype = c_int
        self.lib.network_height.argtypes = [c_void_p]
        self.lib.network_height.restype = c_int

        self.predict = self.lib.network_predict
        self.predict.argtypes = [c_void_p, POINTER(c_float)]
        self.predict.restype = POINTER(c_float)

        self.set_gpu = self.lib.cuda_set_device
        self.set_gpu.argtypes = [c_int]

        self.make_image = self.lib.make_image
        self.make_image.argtypes = [c_int, c_int, c_int]
        self.make_image.restype = IMAGE

        self.get_network_boxes = self.lib.get_network_boxes
        self.get_network_boxes.argtypes = [c_void_p, c_int, c_int, c_float, c_float, POINTER(c_int), c_int, POINTER(c_int)]
        self.get_network_boxes.restype = POINTER(DETECTION)

        self.make_network_boxes = self.lib.make_network_boxes
        self.make_network_boxes.argtypes = [c_void_p]
        self.make_network_boxes.restype = POINTER(DETECTION)

        self.free_detections = self.lib.free_detections
        self.free_detections.argtypes = [POINTER(DETECTION), c_int]

        self.free_ptrs = self.lib.free_ptrs
        self.free_ptrs.argtypes = [POINTER(c_void_p), c_int]

        self.network_predict = self.lib.network_predict
        self.network_predict.argtypes = [c_void_p, POINTER(c_float)]

        self.reset_rnn = self.lib.reset_rnn
        self.reset_rnn.argtypes = [c_void_p]

        self.load_net = self.lib.load_network
        self.load_net.argtypes = [c_char_p, c_char_p, c_int]
        self.load_net.restype = c_void_p

        self.do_nms_obj = self.lib.do_nms_obj
        self.do_nms_obj.argtypes = [POINTER(DETECTION), c_int, c_int, c_float]

        self.do_nms_sort = self.lib.do_nms_sort
        self.do_nms_sort.argtypes = [POINTER(DETECTION), c_int, c_int, c_float]

        self.free_image = self.lib.free_image
        self.free_image.argtypes = [IMAGE]

        self.letterbox_image = self.lib.letterbox_image
        self.letterbox_image.argtypes = [IMAGE, c_int, c_int]
        self.letterbox_image.restype = IMAGE

        self.load_meta = self.lib.get_metadata
        self.lib.get_metadata.argtypes = [c_char_p]
        self.lib.get_metadata.restype = METADATA

        self.load_image = self.lib.load_image_color
        self.load_image.argtypes = [c_char_p, c_int, c_int]
        self.load_image.restype = IMAGE

        self.rgbgr_image = self.lib.rgbgr_image
        self.rgbgr_image.argtypes = [IMAGE]

        self.predict_image = self.lib.network_predict_image
        self.predict_image.argtypes = [c_void_p, IMAGE]
        self.predict_image.restype = POINTER(c_float)

    def _array_to_image(self, arr):
        # need to return old values to avoid python freeing memory
        arr = arr.transpose(2,0,1)
        c, h, w = arr.shape[0:3]
        arr = np.ascontiguousarray(arr.flat, dtype=np.float32) / 255.0
        data = arr.ctypes.data_as(POINTER(c_float))
        im = IMAGE(w,h,c,data)
        return im, arr

    def _detect(self, net, meta, image, thresh=.5, hier_thresh=.5, nms=.45):
        im, image = self._array_to_image(image)
        self.rgbgr_image(im)
        num = c_int(0)
        pnum = pointer(num)
        self.predict_image(net, im)
        dets = self.get_network_boxes(net, im.w, im.h, thresh,
                                hier_thresh, None, 0, pnum)
        num = pnum[0]
        if nms:
            self.do_nms_obj(dets, num, meta.classes, nms)

        res = []
        for j in range(num):
            a = dets[j].prob[0:meta.classes]
            if any(a):
                ai = np.array(a).nonzero()[0]
                for i in ai:
                    b = dets[j].bbox
                    res.append((meta.names[i], dets[j].prob[i],
                            (b.x, b.y, b.w, b.h)))

        res = sorted(res, key=lambda x: -x[1])
        if isinstance(image, bytes):
            self.free_image(im)
        self.free_detections(dets, num)
        return res


    def run(self, img, model_name="yolo3_final", _meta_path=None):
        # Using yolov3 and self-trained weights
        cfg_path, meta_path, object_weight_path = self._get_path(model_name)
        print("After get path")
        if _meta_path: # load from __main__
            meta_path = _meta_path
        net = self.load_net(cfg_path, object_weight_path, 0)
        print(cfg_path)
        print("After load net")
        print(meta_path)
        print(os.path.abspath(__file__))
        meta = self.load_meta(meta_path)
        print("After load meta")

        res = self._detect(net, meta, img)
        print('res: ', res)
        print("After detect")
        return res

if __name__ == '__main__':
    yolo3_bunny_meta_path = os.path.abspath(os.path.join(__file__, '..', 'darknet', 'cfg', 'obj_bunny_plushie-py_main.data'))
    img = cv2.imread('./bunny_plushie_1.jpg')
    d = YOLODetector()
    print d.run(img, "yolo3_final", yolo3_bunny_meta_path)
