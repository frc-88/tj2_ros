MODEL_NAME=yolov5n

detectnet \
	--model=resources/pretrained/$MODEL_NAME.onnx \
	--labels=models/pretrained/$MODEL_NAME-labels.txt \
	--input-blob=input_0 \
	--output-cvg=scores \
	--output-bbox=boxes \
	/dev/video0 display://0

