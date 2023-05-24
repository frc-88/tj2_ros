#!/bin/bash

export DIR=$(dirname $(realpath ${BASH_SOURCE[0]}))

cd ${DIR}

which nvidia-smi > /dev/null
if [[ $? -eq 1 ]]; then
	HAS_GPU="disable"
else
	nvidia-smi -L > /dev/null
	num_gpus=$?
	if ((num_gpus == 0)); then
		HAS_GPU="enable"
	else
		HAS_GPU="disable"
	fi
fi

ln -sf docker-compose.gpu-${HAS_GPU}.yaml docker-compose.gpu.yaml

docker image list | grep -q training_tj2_ros
if [ $? -eq 0 ]; then
	echo "training"
	DEVCONTAINER_IMAGE=training
else
	echo "workstation"
	DEVCONTAINER_IMAGE=workstation
fi
ln -sf docker-compose.${DEVCONTAINER_IMAGE}.yaml docker-compose.yaml

cat <<EOT > ${DIR}/.env
TJ2_IMAGE_VERSION=$(${DIR}/../docker/get_image_tag)
EOT
