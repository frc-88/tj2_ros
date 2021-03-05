#MODEL_DIR=/home/ben/jetson-inference/python/training/detection/ssd/models/tj2_2020_game
MODEL_DIR=/home/ben/jetson-inference/python/training/detection/ssd/models/tj2_2020_game

#MODEL_NAME=mb2-ssd-lite-Epoch-116-Loss-3.3456220626831055.pth
#MODEL_NAME=vgg16-ssd-Epoch-119-Loss-1.5007062611125765.pth
#MODEL_NAME=vgg16-ssd-Epoch-20-Loss-2.185383092789423.pth
#MODEL_NAME=vgg16-ssd-Epoch-0-Loss-6.025318622589111.pth
#MODEL_NAME=""  # picks lowest loss

#OUTPUT=$MODEL_DIR/tj2-2020-vgg16-ssd.onnx
OUTPUT=$MODEL_DIR/tj2-2020-mb2-ssd-lite.onnx

#BASE_NET=mb1-ssd
#BASE_NET=mb1-lite-ssd
BASE_NET=mb2-ssd-lite
#BASE_NET=vgg16-ssd

/usr/bin/python3 /home/ben/jetson-inference/python/training/detection/ssd/onnx_export.py \
        --net=$BASE_NET \
        --model-dir=$MODEL_DIR \
        --input=$MODEL_NAME \
        --output=$OUTPUT
