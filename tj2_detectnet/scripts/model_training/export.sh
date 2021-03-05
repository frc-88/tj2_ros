MODEL_DIR=/home/ben/jetson-inference/python/training/detection/ssd/models/tj2_2020_game
MODEL=mb2-ssd-lite-Epoch-116-Loss-3.3456220626831055.pth
OUTPUT=/home/ben/jetson-inference/python/training/detection/ssd/models/tj2_2020_game/tj2-2020-mb2-ssd-lite.onnx


/usr/bin/python3 /home/ben/jetson-inference/python/training/detection/ssd/onnx_export.py \
        --net=mb2-ssd-lite \
        --model-dir=$MODEL_DIR \
        --input=$MODEL_NAME \
        --output=$OUTPUT
