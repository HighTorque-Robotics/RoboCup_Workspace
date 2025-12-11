#!/bin/bash

if [ $# -ne 1 ]; then
    echo "Usage: $0 <model file list>"
    exit 1
fi

BATCH=1
MODELLIST=$1

# validate for all models (take model list file as input) for visdrone and for coco
while read modelfile; do
    modelname=$(basename "$modelfile")
    modelname=${modelname%.*}
    echo $modelname

    # visdrone.map and coco.map can be used to remap class label ids
    jetnet_validate_yolo --profile --batch=$BATCH --anchors=anchors.txt --catmap=visdrone.map --imgpath yolov3 ${modelfile} \
        class_labels.names visdrone_val.txt "visdrone/${modelname}.json" > "visdrone/${modelname}.log"
    jetnet_validate_yolo --profile --batch=$BATCH --anchors=anchors.txt --catmap=coco.map yolov3 ${modelfile} class_labels.names \
        coco_val.txt "coco/${modelname}.json" > "coco/${modelname}.log"
done < ${MODELLIST}
