# Summary

This README explains how to reproduct the TensorRT models and detections evaluated in our paper
"Super accurate low latency object detection on a surveillance UAV".

We assume you already installed Jetnet on your system.

# Building the models

## creating the calibration list

First, create a list file with paths of calibration images. You can pick calibration images from the training set.
See ```scripts/models/generate_calibration_list.sh``` as an example to create a calibration list file.

NOTE: the calibration list is only needed for INT8 models.

## getting the weight files

Download the darknet weight files used in our experiments [yolov3_leaky.weights](https://kuleuven.app.box.com/s/wiiehwqod5clap3ohj5fsmoq4aaehvkq)
and [yolov3_relu.weights](https://kuleuven.box.com/s/qb4goxr55gh9iufge95vum8g2oi8sy1j) and put them in the ```scipts/paper/models``` folder.

## generating the models

```
cd scripts/paper/models
generate_models.sh
```

## 8-bit models

For INT8 models, calibration is done first during the compilation stage (which might take a long time).
The calibration result of each model is stored in a so called '.cache' file. A ''.cache' file contains a set of scaling factors, one for each layer.
The TensorRT framework only supports symmetric 8-bit integer activations, meaning that it assumes that the input feature map values of each layer (including the input)
are balanced around 0. All models trained in darknet assume that input values of the first layer range between 0 and 1 (balanced around 0.5).
This results in one scaling factor (that of the input layer) to be calculated wrong by TensorRT's calibration algorithm. To be precise, its value must be
multiplied by 2. We therefore patch the calibration file after it has been generated with the following script:

```
python patch_trt_cache.py <model.cache> <patched_model.cache>
```

So first, build the INT8 model to get the calibration cache file, then patch the cache file and rebuild the model, providing the builder program with the patched
cache file (this will go much faster since the calibration cache is already created).

For convenience, we already included the patched .cache files of the provided models.

# Generating detections on a validation set

The script ```scripts/paper/validate/validate.sh``` is an example to generate detection results and timing benchmarks for a
given list of compiled models. It will run each model on the images listed in ```visdrone_val.txt``` and ```coco_val.txt```.
The generated detection results are in MS COCO format (json).

## create the model list file

The first argument of ```validate.sh``` is a model list file that can be created with:

```
find `pwd`/models -name "*.model" > models/models.txt
```
