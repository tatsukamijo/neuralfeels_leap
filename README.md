# NeuralFeels with LEAP Hand in Real

## Installation
```bash
git submodule update --init --recursive
```
Download the IsaacGym Preview 4 from [here](https://developer.nvidia.com/isaac-gym/download) and put it under the `./LEAP_Hand_Sim` directory.

## Mounting your dataset directory

Before running docker compose, set the DATASET_PATH environment variable to the absolute path of your dataset directory. This directory will be mounted to /workspace/neuralfeels/data inside the neuralfeels container.

Example:
```bash
export DATASET_PATH=/absolute/path/to/your/dataset
```
Then run:
```bash
docker compose up --build -d
```

You can then access your dataset at /workspace/neuralfeels/data inside the neuralfeels container.

Download the FeelSight dataset
```bash
cd YOUR_DATASET_PATH && git clone https://huggingface.co/datasets/suddhu/Feelsight
mv Feelsight/* . && rm -r Feelsight
find . -name "*.tar.gz" -exec tar -xzf {} \; -exec rm {} \; && cd ..
```

Download the tactile transformer model
```bash
cd YOUR_DATASET_PATH && git clone https://huggingface.co/suddhu/tactile_transformer && cd ..
```
Get the Segment-anything weights:
```bash
mkdir -p YOUR_DATASET_PATH/segment-anything && cd YOUR_DATASET_PATH/segment-anything
for model in sam_vit_h_4b8939.pth sam_vit_l_0b3195.pth sam_vit_b_01ec64.pth; do
  wget https://dl.fbaipublicfiles.com/segment_anything/$model
done
```

```bash
docker exec -it leap_hand_sim bash
cd /workspace/catkin_ws
catkin_make
source devel/setup.bash
```
```bash
# Open another terminal
docker exec -it neuralfeels bash
./install.sh -e neuralfeels
eval "$(micromamba shell hook --shell bash)"
micromamba activate neuralfeels  
export PYTHONPATH=/workspace/catkin_ws/devel/lib/python3/dist-packages:$PYTHONPATH
pip install rospy rospkg
```