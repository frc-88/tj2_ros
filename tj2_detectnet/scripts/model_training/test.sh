#MODEL_DIR=/home/ben/jetson-inference/python/training/detection/ssd/models/tj2_2020_game_mb2
#MODEL=$MODEL_DIR/tj2-2020-mb2-ssd-lite.onnx

MODEL_DIR=/home/ben/jetson-inference/python/training/detection/ssd/models/tj2_2020_game
#MODEL=$MODEL_DIR/tj2-2020-vgg16-ssd.onnx
MODEL=$MODEL_DIR/tj2-2020-mb2-ssd-lite.onnx

LABELS=$MODEL_DIR/labels.txt
#INPUT=/dev/video0
#INPUT=/home/ben/tensorflow_workspace/2020Game/data/videos/5172_POV-Great_Northern_2020_Quals_22.mp4
INPUT=/home/ben/tensorflow_workspace/2020Game/data/videos/5172_POV-Great_Northern_2020_Quals_60.mp4
#INPUT=/home/ben/tensorflow_workspace/2020Game/data/videos/Monterrey_Regional_2020_Practice15.mp4
#INPUT=/home/ben/tensorflow_workspace/2020Game/data/videos/VID_20200215_145921.mp4
#INPUT=/home/ben/tensorflow_workspace/2020Game/data/videos/video_day2.mp4

OUTPUT=display://0
#OUTPUT=/home/ben/Desktop/output.png
#OUTPUT=/home/ben/Desktop/output.mp4

THRESHOLD=0.3

# relevant fix for running video on x86 systems:
# https://github.com/dusty-nv/jetson-inference/issues/810

detectnet \
	--model=$MODEL \
	--labels=$LABELS \
	--input-blob=input_0 \
	--output-cvg=scores \
	--output-bbox=boxes \
        --threshold=$THRESHOLD \
	$INPUT $OUTPUT
