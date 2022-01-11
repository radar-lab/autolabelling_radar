## Project Title
**Automatic Radar-Camera Dataset Generation for Sensor-Fusion Applications**

---
#### Authors: [Arindam Sengupta](http://u.arizona.edu/~sengupta/), Atshushi Yoshizawa, [Dr. Siyang Cao](https://uweb.engr.arizona.edu/~caos/)
#### Organization: University of Arizona
#### Email: sengupta@email.arizona.edu | caos@email.arizona.edu
#### This work is supported by Sony Research Award Program
---

## Project Overview
With heterogeneous sensors offering complementary advantages in perception, there has been a significant growth in sensor-fusion based research and development in object perception and tracking using classical or deep neural networks based approaches. However, supervised learning requires massive labeled data-sets, that require expensive manual labor to generate. This paper presents a novel approach that leverages YOLOv3 based highly accurate object detection from camera to automatically label point cloud data obtained from a co-calibrated radar sensor to generate labeled [radar-image](https://github.com/radar-lab/autolabelling_radar#generating-radar-camera-labelled-dataset) and  [radar-only](https://github.com/radar-lab/autolabelling_radar#generating-radar-camera-labelled-dataset-1) data-sets to aid learning algorithms for different applications. To achieve this we first co-calibrate the vision and radar sensors and obtain a radar-to-camera transformation matrix. The collected radar returns are segregated by different targets using a density based clustering scheme and the cluster centroids are projected onto the camera image using the transformation matrix. The Hungarian Algorithm is then used to associate the radar cluster centroids with the YOLOv3 generated bounding box centroids, and are labeled with the predicted class. The proposed approach is efficient, easy to implement and aims to encourage rapid development of multi-sensor data-sets, which are extremely limited currently, compared to the optical counterparts.

## Related Work
Several vision-radar sensor-fusion based object recognition using supervised techniques have been proposed in the recent past. Two of the most challenging stages in such approaches are (i) obtaining simultaneous vision and segregated radar data from individual targets in the scene; and (ii) manually labeling each of the data points with the object class. While some of the aforementioned approaches use one of the existing data-sets, the downside to that is the inability or degraded performance of their system when different radar parameters than the one used to collect the data-set is used, subsequently having to resort back to manual annotation.

While there are several labeled image data-sets for object identification, annotating radar point-cloud data is comparatively challenging primarily due to (i) the sparse nature of the distribution, (ii) difficulty in segregating multiple target clusters, and (iii) identifying the object class from the distribution alone, thereby often requiring an optical co-sensing modality to serve as a visual reference. With the advent of machine learning and the ubiquitous realm of neural networks, developing application-specific supervised learning models for radars typically require highly accurate and large annotated point-cloud data-sets. In 2018, __Schumann et al.__ presented a PointNet and PointNet++ inspired semantic segmentation approach for radar point cloud data, followed by a convolutional neural network-custom neural network driven static and dynamic object radar semantic segmentation in 2020. In 2021, the same authors also presented __RadarScenes__-an automotive-application-driven data-set with labeled radar point-cloud data. A generative modeling approach was adopted by __Lekic et al.__ for radar-camera fusion in 2019, attempting to generate camera-like images of the scene, containing all the environment features, using radar data. However, all the approaches required data-sets that were generated via the tedious task of completely-manual annotation.

Besides complete-manual approaches, several automated and semi-automated radar annotation techniques have also been explored in the recent past. __Wang et al.__ developed the CRUW radar data-set using the proposed RODNet architecture using monocular and stereo camera. __Chadwick et al.__ used a monocular camera and a stereo camera pair to label distant vehicle and pedestrian radar data using YOLO. __Zendar Inc.__ developed a high resolution radar data-set for object detection in complex urban environment, by leveraging semantic dense lidar data stream for automatic annotation followed by manual fine-tuning. Another semi-automatic labeling approach using lidar and Camera data was presented by __Meyer et al.__ that used a combination of active learning and manually fine-tuning low-confidence predictions to yield an automotive radar data-set. A similar cross-modal learning based annotation was proposed by __Major et al.__ that used lidar annotations to label 3-D processed radar data-cube. 

## Advantages of this method

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

<p float="center">
  <img src="https://github.com/radar-lab/autolabelling_radar/blob/main/Auxiliary/exampleRC.gif" width="700" />
</p>

### Generating Radar-Camera Labelled Dataset
#### Run Source File
```
Run Source Codes/main_Rad_Only.m
Notes: 
1. Modify Line 24 to set path to your input file
2. Modify Lines 30-38 to set path for your generated dataset 
```
#### Underlying Algorithm
<p float="center">
  <img src="https://github.com/radar-lab/autolabelling_radar/blob/main/Auxiliary/algo2.png" width="700" />
</p>
The second data-set consists of labeled consecutive frames of just the radar data, to aid applications that aim to identify the class of the object using only radar data. This is extremely useful especially for scenarios where the Image-YOLO pipeline fails to detect an object either due to poor illumination or occlusion, while the radar continues to detect the object as shown in the figure below. Just like the previous case the radar points are first combined to yield frame-wise 3-D radar PCL data. In each frame, DBSCAN is performed to segregate PCL clusters from different targets. At every frame i (i>1), the 3-D cluster centroids are compared and assigned to the 3-D cluster centroids from the previous frame i-1 using Hungarian Algorithm for an inter-frame cluster-to-cluster association, almost in a pseudo-tracking fashion. Every time a centroid is detected for the first time, a unique track ID is assigned to it, and any subsequent associated clusters in the forthcoming frames are appended to the same track ID. Parallely, another Hungarian Algorithm module checks to see if a projected cluster centroid can be associated with any available YOLO prediction in a given frame. If a YOLO assignment is possible, the track ID gets labeled with the associated YOLO class information in that frame. 

<p float="center">
  <img src="https://github.com/radar-lab/autolabelling_radar/blob/main/Auxiliary/pedestrian.png" width="700" />
</p>

Note that a track ID can have radar detections for say N consecutive frames, of which only M\>N frames can have an associated YOLO based label. The idea here is that if an object has been continuously tracked by the radar over several frames, and has been identified by the image-aided YOLO anytime during the track, the entirety of the radar data in that track can be labeled with that specific YOLO class. However, to make the labeling more robust, we take the statistical mode() of all the YOLO predictions associated to a given track ID over all the frames and assign the most frequently predicted class as the final label to a track. The data-set is then generated by extracting 3 consecutive frames, sliding from 1-3, 2-4, 3-5 ... to (N-2)-N frames from each track ID labeled with the mode(YOLO class) and saved to disk as Numpy arrays. Note that objects or `track_ID` s with \< 3 frames andor track IDs with just a single YOLO prediction throughout its course, are excluded from this data-set.

