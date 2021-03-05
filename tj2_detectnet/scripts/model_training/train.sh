#DATABASE=/home/ben/jetson-inference/python/training/detection/ssd/data/tj2_2020_voc_image_database
DATABASE=/home/ben/jetson-inference/python/training/detection/ssd/data/tj2_2020_voc_image_database_artificial_warps
OUTPUT=/home/ben/jetson-inference/python/training/detection/ssd/models/tj2_2020_game
#BASE_NET=mb1-ssd
#BASE_NET=mb1-lite-ssd
BASE_NET=mb2-ssd-lite
#BASE_NET=vgg16-ssd
PRETRAINED_NET=/home/ben/jetson-inference/python/training/detection/ssd/models/mb2-ssd-lite-mp-0_686.pth
#PRETRAINED_NET=/home/ben/jetson-inference/python/training/detection/ssd/models/vgg16-ssd-mp-0_7726.pth
#RESUME=/home/ben/jetson-inference/python/training/detection/ssd/models/tj2_2020_game/mb2-ssd-lite-Epoch-99-Loss-1.019632302224636.pth
RESUME=""

tmux new -s train -d
tmux send -t train "/usr/bin/python3 /home/ben/jetson-inference/python/training/detection/ssd/train_ssd.py \
	--data=$DATABASE \
	--model-dir=$OUTPUT \
	--net=$BASE_NET \
	--batch-size=24 --epochs=120 \
	--dataset-type=voc \
	--pretrained-ssd=$PRETRAINED_NET \
	--num-workers=1" ENTER
#	--resume=$RESUME

echo "Started training session in tmux. Check progress with 'tmux a -t train' "
echo "Monitor GPU usage: 'watch -n1 nvidia-smi' "
echo "If training receives a kill signal, reduce num-workers to 1"
tmux a -t train
