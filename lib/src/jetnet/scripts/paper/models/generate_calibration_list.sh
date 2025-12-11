#!/bin/bash
# generate calibration dataset for 8-bit calibration

visdrone_train_list="/path/to/visdrone/train.txt"
coco_train_list="/path/to/coco/train.txt"

{ cat $visdrone_train_list | sort -R | head -n 500 & cat $coco_train_list | sort -R | head -n 500; } > cal_coco_visdrone.txt
