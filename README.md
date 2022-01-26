# CyberBee track person + gesture control

<img alt="demo_gif" src="https://user-images.githubusercontent.com/13486777/111168690-fb2e9280-85aa-11eb-894f-fe70633072fd.gif">

## Introduction
This project relies on Mediapipe fast hand keypoints recognition.

Mediapipe is an amazing ML platform with many robust solutions like Face mesh, Hand Keypoints detection and Objectron. Moreover, their model can be used on the mobile platforms with on-device acceleration.

## Setup
### 1. Installing pip packages
First, we need to install python dependencies. Make sure you that you are using `python3.7`

List of packages
```sh
ConfigArgParse == 1.2.3
djitellopy == 1.5
numpy == 1.19.3
opencv_python == 4.5.1.48
tensorflow == 2.4.1
mediapipe == 0.8.2
```

Install

pip3 install -r requirements.txt
## Requirements/Setup

### [OpenCV-Python](https://pypi.org/project/opencv-python/):
`pip install opencv-contrib-python` If you receive an error related to Qt binaries that prevents the script from running, try installing the headless version instead with `pip install opencv-contrib-python-headless`. If you are still getting errors, also try the main version instead of contrib.

### [Simple-PID](https://github.com/m-lundberg/simple-pid):
Basic PID controller - `pip install simple-pid`.

### [Tensorflow](https://www.tensorflow.org/install/pip):
This code works with Tensorflow 1.15. It might work with Tensorflow 2 but you would need to make some modifications both in my script and the posenet python code.

### [Posenet-Python](https://github.com/rwightman/posenet-python):
For pose detection, I use this python port (made by [rwightman](https://github.com/rwightman) of the Tensorflow.js Posenet models created by Google. The models and code are already included here so it should work as long as you have Tensorflow set up properly.

### Other:
There are a few other packages you might need to install if you don't have them already, such as keyboard, pygame, and av.

## Usage
To start the script, run `track_person.py` with admin privileges. The automatic control is on from the start, but you can toggle it as well as control the drone manually using the keyboard controls below:
* **T** - toggle automatic control
* **Space** - take off (it won't take off automatically)
* **L** - land
* **Q** - rotate counter clockwise
* **E** - rotate clockwise
* **D** - move right
* **A** - move left
* **W** - move forward
* **S** - move backward
* **R** - reset commands (hover/attempt to stay still)

## How it works:
There are two main processes happening: receiving and processing video input, and sending commands back to the drone. The input is processed to predict the key points of your body, which are used to calculate the error passed into the PID controllers. The location of your nose is used to calculate the error for yaw and vertical movement, and the distance between your shoulders and hips are used to calculate the error for forwards/backwards movement. Commands are then sent back to the drone based on the PID outputs.

## Adding new gestures
Hand recognition detector can add and change training data to retrain the model on the own gestures. But before this,
there are technical details of the detector to understand how it works and how it can be improved
### Technical details of gesture detector
Mediapipe Hand keypoints recognition is returning 3D coordinated of 20 hand landmarks. For our
model we will use only 2D coordinates.

<img alt="gestures_list" width="80%" src="https://user-images.githubusercontent.com/13486777/110933339-49d2f700-8335-11eb-9588-5f68a2677ff0.png">


Then, these points are preprocessed for training the model in the following way.

<img alt="preprocessing" width="80%" src="https://user-images.githubusercontent.com/13486777/111294503-11902900-8653-11eb-9856-a50fe96e750e.png">


After that, we can use data to train our model. Keypoint classifier is a simple Neural network with such 
structure

<img alt="model_structure" width="80%" src="https://user-images.githubusercontent.com/13486777/112172879-c0a5a500-8bfd-11eb-85b3-34ccfa256ec3.jpg">



_check [here](#Grid-Search) to understand how the architecture was selected_
### Creating dataset with new gestures
First, pull datasets from Git LFS. [Here](https://github.com/git-lfs/git-lfs/wiki/Installation) is the instruction of how 
to install LFS. Then, run the command to pull default csv files
```sh
git lfs install
git lfs pull
```

After that, run `main.py` and press "n" to enter the mode to save key points
(displayed as **MODE:Logging Key Point**）

<img width="60%" alt="writing_mode" src="https://user-images.githubusercontent.com/13486777/111301228-a185a100-865a-11eb-8a3c-fa4d9ee96d6a.png">


If you press "0" to "9", the key points will be added to [model/keypoint_classifier/keypoint.csv](model/keypoint_classifier/keypoint.csv) as shown below.<br>
1st column: Pressed number (class ID), 2nd and subsequent columns: Keypoint coordinates

<img width="90%" alt="keypoints_table" src="https://user-images.githubusercontent.com/13486777/111295338-ec4fea80-8653-11eb-9bb3-4d27b519a14f.png">

In the initial state, 7 types of learning data are included as was shown [here](#Gesture-control). If necessary, add 3 or later, or delete the existing data of csv to prepare the training data.
### Notebook for retraining model
[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/kinivi/tello-gesture-control/blob/main/Keypoint_model_training.ipynb)

Open [Keypoint_model_training.ipynb](Keypoint_model_training.ipynb) in Jupyter Notebook or Google Colab.
Change the number of training data classes,the value of **NUM_CLASSES = 3**, and path to the dataset. Then, execute all cells
and download `.tflite` model

<img width="60%" alt="notebook_gif" src="https://user-images.githubusercontent.com/13486777/111295516-1ef9e300-8654-11eb-9f59-6f7a85b99076.gif">


Do not forget to modify or add labels in `"model/keypoint_classifier/keypoint_classifier_label.csv"`

#### Grid Search
❗️ Important ❗️ The last part of the notebook is an experimental part of the notebook which main functionality is to test hyperparameters of the model structure. In a nutshell: grid search using TensorBoard visualization. Feel free to use it for your experiments.


<img width="70%" alt="grid_search" src="https://user-images.githubusercontent.com/13486777/111295521-228d6a00-8654-11eb-937f-a15796a3024c.png">


## Repository structure
<pre>
│  main.py
│  Keypoint_model_training.ipynb
│  config.txt
│  requirements.txt
│  
├─model
│  └─keypoint_classifier
│      │  keypoint.csv
│      │  keypoint_classifier.hdf5
│      │  keypoint_classifier.py
│      │  keypoint_classifier.tflite
│      └─ keypoint_classifier_label.csv
│ 
├─gestures
│   │  gesture_recognition.py
│   │  tello_gesture_controller.py
│   └─ tello_keyboard_controller.py
│          
├─tests
│   └─connection_test.py
│ 
└─utils
    └─cvfpscalc.py
</pre>
### app.py
Main app which controls the functionality of drone control and gesture recognition<br>
App also includes mode to collect training data for adding new gestures.<br>

### keypoint_classification.ipynb
This is a model training script for hand sign recognition.

### model/keypoint_classifier
This directory stores files related to gesture recognition.<br>

* Training data(keypoint.csv)
* Trained model(keypoint_classifier.tflite)
* Label data(keypoint_classifier_label.csv)
* Inference module(keypoint_classifier.py)

### gestures/
This directory stores files related to drone controllers and gesture modules.<br>

* Keyboard controller (tello_keyboard_controller.py)
* Gesture controller(tello_keyboard_controller.py)
* Gesture recognition module(keypoint_classifier_label.csv)

### utils/cvfpscalc.py
Module for FPS measurement.