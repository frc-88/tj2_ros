
MODEL_DIR=model
MODEL_NAME=mb2-ssd-lite-Epoch-60-Loss-1.8050242725171541.pth
# MODEL_NAME=""  # picks lowest loss
OUTPUT=limelight-mb2-ssd-lite.onnx

BASE_NET=mb2-ssd-lite

JETSON_INFERENCE_SSD_DIR=/home/ben/object-recognition/jetson-inference/python/training/detection/ssd

/usr/bin/python3 $JETSON_INFERENCE_SSD_DIR/onnx_export.py \
        --net=$BASE_NET \
        --model-dir=$MODEL_DIR \
        --input=$MODEL_NAME \
        --output=$MODEL_DIR/$OUTPUT
