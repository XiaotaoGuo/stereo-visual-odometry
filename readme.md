# Stereo Visual Odometry

## Introduction

This is my practice project after learning [14 lectures on visual SLAM](https://github.com/gaoxiang12/slambook2). It utilizes stereo images(frame) in each timestamp and compute a relative pose between each frame. Currently it is implemented KITTI stereo odometry dataset and provide a limited visualization of odometry process.

## Refactory plan

- [ ] testing and visualizaion
  - [ ] Decouple date reading method to support more dataset
  - [ ] Decouple and change visualizaion framework to show more information
  - [ ] Add testing functionality to calculating
- [ ] functionaliyu
  - [ ] Add close-loop detection
  - [ ] Add more option for pose estimation
- [ ] Instrucion
  - [ ] Polish readme to have more information