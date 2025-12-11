#!/bin/bash

NAME="surveillance"
WEIGHTS_LEAKY="yolov3_leaky.weights"
WEIGHTS_RELU="yolov3_relu.weights"
INPUT_WIDTH=608
INPUT_HEIGHT=352
CLASSES=6
BATCH=1

INT8BATCH=50
INT8CALFILES="cal_coco_visdrone.txt"

# FP32 models
jetnet_build_darknet_model --maxbatch=$BATCH --width=$INPUT_WIDTH --height=$INPUT_HEIGHT --classes=$CLASSES yolov3_leaky_plugin $WEIGHTS_LEAKY \
    "${NAME}_${INPUT_WIDTH}x${INPUT_HEIGHT}_leaky_plugin_fp32.model"
jetnet_build_darknet_model --maxbatch=$BATCH --width=$INPUT_WIDTH --height=$INPUT_HEIGHT --classes=$CLASSES yolov3_leaky_native $WEIGHTS_LEAKY \
    "${NAME}_${INPUT_WIDTH}x${INPUT_HEIGHT}_leaky_native_fp32.model"
jetnet_build_darknet_model --maxbatch=$BATCH --width=$INPUT_WIDTH --height=$INPUT_HEIGHT --classes=$CLASSES yolov3_relu $WEIGHTS_RELU \
    "${NAME}_${INPUT_WIDTH}x${INPUT_HEIGHT}_relu_fp32.model"

# FP16 models
jetnet_build_darknet_model --maxbatch=$BATCH --width=$INPUT_WIDTH --height=$INPUT_HEIGHT --classes=$CLASSES --fp16 yolov3_leaky_plugin $WEIGHTS_LEAKY \
    "${NAME}_${INPUT_WIDTH}x${INPUT_HEIGHT}_leaky_plugin_fp16.model"
jetnet_build_darknet_model --maxbatch=$BATCH --width=$INPUT_WIDTH --height=$INPUT_HEIGHT --classes=$CLASSES --fp16 yolov3_leaky_native $WEIGHTS_LEAKY \
    "${NAME}_${INPUT_WIDTH}x${INPUT_HEIGHT}_leaky_native_fp16.model"
jetnet_build_darknet_model --maxbatch=$BATCH --width=$INPUT_WIDTH --height=$INPUT_HEIGHT --classes=$CLASSES --fp16 yolov3_relu $WEIGHTS_RELU \
    "${NAME}_${INPUT_WIDTH}x${INPUT_HEIGHT}_relu_fp16.model"

# INT8 models (not for TX2)
jetnet_build_darknet_model --maxbatch=$BATCH --width=$INPUT_WIDTH --height=$INPUT_HEIGHT --classes=$CLASSES --int8batch=$INT8BATCH --int8cache="${NAME}_leaky_plugin.cache" \
    --int8calfiles=$INT8CALFILES yolov3_leaky_plugin $WEIGHTS_LEAKY "${NAME}_${INPUT_WIDTH}x${INPUT_HEIGHT}_leaky_plugin_int8.model"
jetnet_build_darknet_model --maxbatch=$BATCH --width=$INPUT_WIDTH --height=$INPUT_HEIGHT --classes=$CLASSES --int8batch=$INT8BATCH --int8cache="${NAME}_leaky_native.cache" \
    --int8calfiles=$INT8CALFILES yolov3_leaky_native $WEIGHTS_LEAKY "${NAME}_${INPUT_WIDTH}x${INPUT_HEIGHT}_leaky_native_int8.model"
jetnet_build_darknet_model --maxbatch=$BATCH --width=$INPUT_WIDTH --height=$INPUT_HEIGHT --classes=$CLASSES --int8batch=$INT8BATCH --int8cache="${NAME}_relu.cache" \
    --int8calfiles=$INT8CALFILES yolov3_relu $WEIGHTS_RELU "${NAME}_${INPUT_WIDTH}x${INPUT_HEIGHT}_relu_int8.model"

