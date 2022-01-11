## Project Title
Automatic Radar-Camera Dataset Generation for Sensor-Fusion Applications

---
#### Authors: [Arindam Sengupta](http://u.arizona.edu/~sengupta/), Atshushi Yoshizawa, [Dr. Siyang Cao](https://uweb.engr.arizona.edu/~caos/)
#### Organization: University of Arizona
#### Email: sengupta@email.arizona.edu | caos@email.arizona.edu
#### This work is supported by Sony Research Award Program
---

## Project Overview
With heterogeneous sensors offering complementary advantages in perception, there has been a significant growth in sensor-fusion based research and development in object perception and tracking using classical or deep neural networks based approaches. However, supervised learning requires massive labeled data-sets, that require expensive manual labor to generate. This paper presents a novel approach that leverages YOLOv3 based highly accurate object detection from camera to automatically label point cloud data obtained from a co-calibrated radar sensor to generate labeled radar-image and  radar-only data-sets to aid learning algorithms for different applications. To achieve this we first co-calibrate the vision and radar sensors and obtain a radar-to-camera transformation matrix. The collected radar returns are segregated by different targets using a density based clustering scheme and the cluster centroids are projected onto the camera image using the transformation matrix. The Hungarian Algorithm is then used to associate the radar cluster centroids with the YOLOv3 generated bounding box centroids, and are labeled with the predicted class. The proposed approach is efficient, easy to implement and aims to encourage rapid development of multi-sensor data-sets, which are extremely limited currently, compared to the optical counterparts. 

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
mm_radar [Link]()
usb_webcam [Link]()
darknet_ros [Link]()
```
The data from radar and camera was acquired using a ROS pipeline using three main packages, namely (i) `mm_radar`; (ii) `usb_webcam`; and (iii) `darknet_ros`. The `mm_radar` package loads the chirp configuration onto the radar and buffers the processed data from the radar and publishes it to the `/radar_scan` topic.  Every message in this topic corresponds to a single radar return, that can be bifurcated into Point Specific parameters and Cluster Specific parameters. The Point Specific Parameters pertain to the position, motion and SNR information of a specific reflection point, denoted by the `PointID`, from the target following the 3D-FFT, MTI and CFAR stages. The `PointID`s are assigned in an ascending order in a frame, always starting at 0, and is therefore used as a marker in identifying and separating radar frames. The radar frame rate was $\approx$10 frames-per-second (fps).  Also, the on-board radar signal processing module has a DBSCAN stage that clusters radar reflections from the same target, denoted by the `TargetID`, which along with the target centroid position and motion information constitute the Cluster Specific Parameters. Points that have no associated clusters or identified as noise are assigned with ` TargetID>253`. In this study we have not relied on the Cluster Specific Parameters and the inbuilt DBSCAN, and have only saved the Point Specific Parameters and applied DBSCAN retroactively in post-processing for ease of re-configurability and debugging.

The `usb_webcam` package reads the raw image from the camera (30fps) and uses the estimated camera intrinsic parameters to rectify and undistort the image, published as a compressed message `/image_rect/compressed`, that also accompanies a time-stamp header. The `darknet_ros` package uses an OpenCV bridge to subscribe to the rectified image and subjects it to a YOLO classification network that outputs the bounding boxes and class of the objects in the image via `/bounding_boxes` message, along with the image acquisition and prediction time-stamps. Besides the bounding box co-ordinates, this package also publishes an image output via. `/detection_image`, with the bounding boxes overlaid on the rectified image input. During the data collection phase, these four message topics are subscribed and saved in ROS bag files for the next steps.

<p float="center">
  <img src="https://github.com/radar-lab/autolabelling_radar/blob/main/Auxiliary/Pipeline.png" width="400" />
</p>