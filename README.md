<h1 align="center">
 A Control Approach for Human-Robot <br /> Ergonomic Payload Lifting
</h1>

<div align="center">

L. Rapetti, C. Sartore, M. Elobaid, Y. Tirupachuri, F. Draicchio, T. Kawakami,
T. Yoshiike, and D. Pucci

</div>

</div>

<p align="center">

https://github.com/ami-iit/paper_rapetti_2023_icra_ergonomic_payload_lifting/assets/35487806/6af0b2d7-571b-46d3-bc99-665455f33f1b


<div align="center">
2023 International Conference on Robotics and Automation (ICRA)
</div>
 
<div align="center">
  <a href="INSTALL.md"><b>Installation</b></a> |
  <a href="https://arxiv.org/abs/2305.08499"><b>arXiv</b></a> | 
  <a href="https://youtu.be/wJTRQpjeHMc"><b>YouTube</b></a>
</div>


## Abstract
Collaborative robots can relief human operators from excessive efforts during payload lifting activities. Modelling the human partner allows the design of safe and efficient collaborative strategies.
In this paper, we present a control approach for human-robot collaboration based on human monitoring through whole-body wearable sensors, and interaction modelling through coupled rigid-body dynamics. Moreover, a trajectory advancement strategy is proposed, allowing for online adaptation of the robot trajectory depending on the human motion. The resulting framework allows us to perform payload lifting tasks,  taking into
account the ergonomic requirements of the agents. Validation has been performed in an experimental scenario using the iCub3 humanoid robot and a human subject sensorized with the iFeel wearable system.

## Reproducing the experiments
The instruction to install the required software can be found in [INSTALL documentation](INSTALL.md). Following the instruction, you will be able to use:
- [Gazebo models for collaborative tasks](app/models/README.md)
- [Visualizer for multi-agent interaction](modules/MultiRobotVisualizer/README.md)
- [Simulink controllers for human-robot collaboration](whole-body-controllers/README.md)

Human models can be generated using the [`human-model-generator`](https://github.com/ami-iit/human-model-generator) python tool.

## Citing this work

If you find the work useful, please consider citing:

```bibtex
@INPROCEEDINGS{rapetti2023control,
   author={Lorenzo Rapetti and Carlotta Sartore and Mohamed Elobaid and Yeshasvi Tirupachuri and Francesco Draicchio and Tomohiro Kawakami and Takahide Yoshiike and Daniele Pucci},
   booktitle={2023 International Conference on Robotics and Automation (ICRA)},
   title={A Control Approach for Human-Robot Ergonomic Payload Lifting},
   year={2023},
```



## Maintainer

This repository is maintained by:

|                                                              |                                                      |
| :----------------------------------------------------------: | :--------------------------------------------------: |
| [<img src="https://github.com/lrapetti.png" width="40">](https://github.com/lrapetti) | [@lrapetti](https://github.com/lrapetti) |
