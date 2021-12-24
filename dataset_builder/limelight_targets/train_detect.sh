DATABASE=detect_dataset
OUTPUT=detect_model
BASE_NET=mb2-ssd-lite
PRETRAINED_NET=pretrained/mb2-ssd-lite-mp-0_686.pth
#RESUME=/home/ben/jetson-inference/python/training/detection/ssd/models/tj2_2020_game/mb2-ssd-lite-Epoch-99-Loss-1.019632302224636.pth
RESUME=""

JETSON_INFERENCE_SSD_DIR=/home/ben/object-recognition/jetson-inference/python/training/detection/ssd

tmux new -s train -d
tmux send -t train "/usr/bin/python3 $JETSON_INFERENCE_SSD_DIR/train_ssd.py \
	--data=$DATABASE \
	--model-dir=$OUTPUT \
	--net=$BASE_NET \
	--batch-size=24 --epochs=150 \
	--dataset-type=voc \
	--pretrained-ssd=$PRETRAINED_NET \
	--num-workers=1" ENTER
#	--resume=$RESUME

echo "Started training session in tmux. Check progress with 'tmux a -t train' "
echo "Monitor GPU usage: 'watch -n1 nvidia-smi' "
echo "If training receives a kill signal, reduce num-workers to 1"
tmux a -t train
