# Gate Sizing Contest

本项目主要针对 [CAD Contest 24 Problem C](https://www.iccad-contest.org/Problems.html)，目标是实现一个高质量，高效的的Gate Sizer。本次竞赛有一些特点：

- 使用ASAP7作为本次竞赛的工艺库，工艺较为先进，过去的sizer不一定对先进工艺可以产生好的结果。
- 使用 OpenROAD 

## 1. 技术路线

### 1.1 使用iEDA parser + iSTA + usizer 完成竞赛要求

CMakeLists 中ROAD 1

### 1.2 使用OpenRoad parser + OpenSTA + usizer 完成竞赛要求

CMakeLists 中ROAD 2

###  1.3 使用修改版的TritonSizer parser + OpenSTA + TritonSizer 完成竞赛要求

CMakeLists 中ROAD 3

