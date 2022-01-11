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

