# Training

## How to Run

From command line:

```
# Full Yolov3, 20 normal + 40 augmented images
./darknet detector test cfg/objv2.data cfg/yolo3-objv2.cfg backup/yolo3-objv2_10000.weights data/IMG_CUP.jpeg

# Yolov3 Tiny, 20 normal images
./darknet detector test cfg/obj.data cfg/yolo3-obj-tiny.cfg backup/yolo3-obj-tiny_10000.weights data/IMG_CUP.jpeg
```

Replace `data/IMG_CUP.jpeg` with other test images. 

From `python`:

```
python ./python/darknet_cup.py
```

Tested with python2, currently using Full Yolov3. Change to Yolov3 Tiny by changing L155 and L156.