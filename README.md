<div align="center">  
 
# Autonomous Manipulation
</div>


<h3 align="center">
  <a href="#">arXiv</a> |
  <a href="#">Video</a> |
  <a href="#">Slides</a>
</h3>


<br><br>

## Table of Contents:
1. [Highlights](#high)
2. [Getting Started](#start)
   - [Installation](docs/INSTALL.md)
   - [Starting UR5 Simulation](docs/UR5Sim.md)
   - [Starting UR5 Real Robot](docs/UR5Real.md)
   - [Calibration](docs/Calibration.md)
   - [Running](docs/Running.md)
4. [Results and Models](#models)
5. [TODO List](#todos)
6. [License](#license)


## Highlights <a name="high"></a>

- :oncoming_automobile: **Autonomous Manipulation**: Our work focuses on TBD ... 
- :trophy: **SOTA performance**: All tasks achieve SOTA performance .... 



## Getting Started <a name="start"></a>
   - [Installation](docs/INSTALL.md)
   - [Starting UR5 Simulation](docs/UR5Sim.md)
   - [Starting UR5 Real Robot](docs/UR5Real.md)
   - [Callibration](docs/Callibration.md)
   - [Running](docs/Running.md)

## Results and Pre-trained Models <a name="models"></a>
**To fix later**

### Mask-RCNN training
> **To fix later** We first train the perception modules (i.e., track and map) to obtain a stable weight initlization for the next stage. BEV features are aggregated with 5 frames (queue_length = 5).

| Method | Encoder | Tracking<br>AMOTA | Mapping<br>IoU-lane | config | Download |
| :---: | :---: | :---: | :---: | :---:|:---:| 
| UniAD-B | R101 | 0.390 | 0.297 |  [base-stage1](projects/configs/stage1_track_map/base_track_map.py) | [base-stage1](https://github.com/OpenDriveLab/UniAD/releases/download/v1.0/uniad_base_track_map.pth) |


### Checkpoint Usage
* Download the checkpoints you need into ....



## TODO List <a name="todos"></a>
**To be updated**
- [x] Finish README
- [ ] Add vision part
- [ ] Visualization codes 


## License <a name="license"></a>

All assets and code are under the [Apache 2.0 license](./LICENSE) unless specified otherwise.

