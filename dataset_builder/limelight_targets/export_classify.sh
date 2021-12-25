
MODEL_DIR=./outputs/classify_model
MODEL_NAME=""  # picks lowest loss
OUTPUT=limelight-classify.onnx

JETSON_INFERENCE_CLASSIFY_DIR=/home/ben/object-recognition/jetson-inference/python/training/classification

/usr/bin/python3 $JETSON_INFERENCE_CLASSIFY_DIR/onnx_export.py \
        --model-dir=$MODEL_DIR \
        --output=$MODEL_DIR/$OUTPUT
