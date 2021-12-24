DATABASE=./classify_dataset
OUTPUT=./classify_model
#RESUME=
RESUME=""

JETSON_INFERENCE_CLASSIFY_DIR=/home/ben/object-recognition/jetson-inference/python/training/classification

tmux new -s train -d
tmux send -t train "/usr/bin/python3 $JETSON_INFERENCE_CLASSIFY_DIR/train.py \
	--model-dir=$OUTPUT \
	--batch-size=24 --epochs=35 \
	--workers=2 \
	$DATABASE" ENTER
#	--resume=$RESUME

echo "Started training session in tmux. Check progress with 'tmux a -t train' "
echo "Monitor GPU usage: 'watch -n1 nvidia-smi' "
echo "If training receives a kill signal, reduce num-workers to 1"
tmux a -t train
