# NeuralFeels with LEAP Hand in Real

## Installation
```bash
git submodule update --init --recursive
```
Download the IsaacGym Preview 4 from [here](https://developer.nvidia.com/isaac-gym/download) and put it under the `./LEAP_Hand_Sim` directory.

```bash
docker compose up --build -d
```

```bash
docker exec -it leap_hand_sim bash
```
```bash
# Open another terminal
docker exec -it neuralfeels bash
```