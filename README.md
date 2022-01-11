## Project Title
Automatic Radar-Camera Dataset Generation for Sensor-Fusion Applications

---
#### Authors: [Arindam Sengupta](http://u.arizona.edu/~sengupta/), Atshushi Yoshizawa, [Dr. Siyang Cao](https://uweb.engr.arizona.edu/~caos/)
#### Organization: University of Arizona
#### Email: sengupta@email.arizona.edu | caos@email.arizona.edu
#### This work is supported by Sony Research Award Program
---

## Project Overview
With heterogeneous sensors offering complementary advantages in perception, there has been a significant growth in sensor-fusion based research and development in object perception and tracking using classical or deep neural networks based approaches. However, supervised learning requires massive labeled data-sets, that require expensive manual labor to generate. This paper presents a novel approach that leverages YOLOv3 based highly accurate object detection from camera to automatically label point cloud data obtained from a co-calibrated radar sensor to generate labeled [radar-image](https://github.com/radar-lab/autolabelling_radar#generating-radar-camera-labelled-dataset) and  radar-only data-sets to aid learning algorithms for different applications. To achieve this we first co-calibrate the vision and radar sensors and obtain a radar-to-camera transformation matrix. The collected radar returns are segregated by different targets using a density based clustering scheme and the cluster centroids are projected onto the camera image using the transformation matrix. The Hungarian Algorithm is then used to associate the radar cluster centroids with the YOLOv3 generated bounding box centroids, and are labeled with the predicted class. The proposed approach is efficient, easy to implement and aims to encourage rapid development of multi-sensor data-sets, which are extremely limited currently, compared to the optical counterparts. 

## Data Collection
### Hardware Used
```
NVidia Jetson AGX Xavier
TI mmWave AWR1843 BOOST 
USB8MP02G Camera Module

```
<p float="center">
  <img src="https://github.com/radar-lab/autolabelling_radar/blob/main/Auxiliary/Setup.jpg" width="400" />
</p>

### Software/OS Used
```
Jetpack 4.5.1 (Note: OpenCV4)
ROS Kinetic and Melodic
Ubuntu 18 (Bionic Beaver)
```

### ROS Packages
```
mm_radar [Link: https://github.com/radar-lab/ti_mmwave_rospkg)
usb_webcam [Link: https://github.com/radar-lab/usb_webcam)
darknet_ros [Link: https://github.com/leggedrobotics/darknet_ros)
```
<p float="center">
  <img src="https://github.com/radar-lab/autolabelling_radar/blob/main/Auxiliary/Pipeline.png" width="700" />
</p>

The data from radar and camera was acquired using a ROS pipeline using three main packages, namely (i) `mm_radar`; (ii) `usb_webcam`; and (iii) `darknet_ros`. The `mm_radar` package loads the chirp configuration onto the radar and buffers the processed data from the radar and publishes it to the `/radar_scan` topic.  Every message in this topic corresponds to a single radar return, that can be bifurcated into Point Specific parameters and Cluster Specific parameters. The Point Specific Parameters pertain to the position, motion and SNR information of a specific reflection point, denoted by the `PointID`, from the target following the 3D-FFT, MTI and CFAR stages. The `PointID`s are assigned in an ascending order in a frame, always starting at 0, and is therefore used as a marker in identifying and separating radar frames. The radar frame rate was \approx10 frames-per-second (fps).  Also, the on-board radar signal processing module has a DBSCAN stage that clusters radar reflections from the same target, denoted by the `TargetID`, which along with the target centroid position and motion information constitute the Cluster Specific Parameters. Points that have no associated clusters or identified as noise are assigned with ` TargetID>253`. In this study we have not relied on the Cluster Specific Parameters and the inbuilt DBSCAN, and have only saved the Point Specific Parameters and applied DBSCAN retroactively in post-processing for ease of re-configurability and debugging.

The `usb_webcam` package reads the raw image from the camera (30fps) and uses the estimated camera intrinsic parameters to rectify and undistort the image, published as a compressed message `/image_rect/compressed`, that also accompanies a time-stamp header. The `darknet_ros` package uses an OpenCV bridge to subscribe to the rectified image and subjects it to a YOLO classification network that outputs the bounding boxes and class of the objects in the image via `/bounding_boxes` message, along with the image acquisition and prediction time-stamps. Besides the bounding box co-ordinates, this package also publishes an image output via. `/detection_image`, with the bounding boxes overlaid on the rectified image input. During the data collection phase, these four message topics are subscribed and saved in ROS bag files for the next steps.

## Autolabelling Schemes
#### Note: While the current source code is constructed to work with ROSbag files, it can be duly modified to fit your input data specifications.
### Requirements
```
MATLAB R2019 (or higher)
ROS Toolbox
Custom ROS message [https://www.mathworks.com/help/ros/ug/create-custom-messages-from-ros-package.html]
Matlab-Numpy Interface [https://github.com/kwikteam/npy-matlab]
```
### Sample Data
A sample intersection data can be downloaded from [here](https://drive.google.com/file/d/1inC5DblWC84UBXTW3gZWK6VQwGN3Kwx1/view?usp=sharing).

### Generating Radar-Camera Labelled Dataset
#### Run Source File
```
Run Source Codes/main_Rad_Cam.m
Notes: 
1. Modify Line 24 to set path to your input file
2. Modify Lines 30-39 to set path for your generated dataset 
```
#### Underlying Algorithm
<p float="center">
  <img src="https://github.com/radar-lab/autolabelling_radar/blob/main/Auxiliary/algo1.png" width="700" />
</p>

The first data-set consists of labeled radar-image pairs from the same objects in a given frame, over all the N frames. The radar points are first congregated to form frame-wise radar PCL data using the `PointID` parameter. Recall that the `PointID`s are always in an ascending order in a frame, starting at 0. The radar time-stamp (TS<sub>R</sub>) of a given frame is set to be the time-stamp at which was PointID=0 was received. Similarly (TS<sub>I</sub>) and (TS<sub>BB</sub>) represent the time-stamps of the image and the YOLO bounding box (YOLOBBox) predictions, respectively. In each of the radar frames, DBSCAN is performed on the 3-D PCL data to segregate multiple target clusters. The 3-D centroids of the clusters are then projected onto the corresponding (using TS<sub>R</sub> and TS<sub>I</sub>) camera image of the scene by using the Radar-to-Camera transformation matrix TM estimated during the calibration process. Similarly, the corresponding YOLO bounding boxes and classes are then determined (using TS<sub>R</sub>, TS<sub>I</sub> and TS<sub>BB</sub>). The pixel indices of the YOLO centroids and the projected radar cluster centroids are then subjected to the Hungarian Algorithm for intra-frame radar-to-image association. For every associated YOLO-cluster pair (i) the image-region inside the bounding box is cropped and reshaped to a 64X64X3 png file (CropImg), and (ii) the X,Y,Z, Doppler and SNR of all the points from the radar cluster, are saved to disk as Numpy arrays, with the YOLO class as the label. 

The intermediate steps described above has been depicted through an example below. 