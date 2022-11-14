
# SDCND : Sensor Fusion and Tracking
This is the project for the second course in the  [Udacity Self-Driving Car Engineer Nanodegree Program](https://www.udacity.com/course/c-plus-plus-nanodegree--nd213) : Sensor Fusion and Tracking. 

## License
[License](LICENSE.md)

## Project Lineup and Setup
The project setup can be found in [README.md](https://github.com/udacity/nd013-c2-fusion-starter)

# 3D Object Detection (Mid-Term Project)
This project is done by using the Waymo Open Dataset's real-world data collected from lidar sensor. Here are the project's requirements that should be done by each section.

- Section 1: Computer Lidar Point-Cloud from Range Image
  - Convert the range image "range" and "intensity" channel to 8-bit
  - Able to visualize the result using OpenCV (ID_S1_EX1)
  - Visualize the point-cloud using open3d module
  - Identify 10 images of vehicles (ID_S1_EX2)

<br>

- Section 2: Create Birds-Eye View (BEV) from Lidar PCL
  - Convert coordinates in x,y [m] into x,y [pixel] based on width and height of the bev map (ID_S2_EX1)
  - Adjust instensity so that the vehicles are clearly visible (ID_S2_EX2)
  - Compare the to results from just normalizing the height in each BEV map pixel and filling the "height" channel. (ID_S2_EX3)

<br>

- Section 3: Model-based Object Detection in BEV Image
  - In addition to Complex YOLO, extract the code for output decoding and post-processing from the GitHub repo. (ID_S3_EX1)
  - Tranform BEV coordinates and convert model output to expected bounding box format. (ID_S3_EX2)

<br>

- Section 4: Performance Evaluation for Object Detection
  - Compare the ground-truth label and detected objects and count the true positive (ID_S4_EX1)
  - Compute the false negative and false positive based on IoU and ground-truth labels (ID_S4_EX2)
  - Compute the precision and recall for all frames (ID_S4_EX3)

All the projects are run in Ubuntu20.04 machine with RTX 3060Ti GPU device. The running code for this project is 

<code> python loop_over_dataset.py </code>

## Section 1: Computer Lidar Point-Cloud from Range Image
There are some changes in 'loop_over_dataset.py' to run this section
![Alt text](code_image/section_1_1.png)
![Alt text](code_image/section_1_2.png)

Also, there are changes in 'objdet_pcl.py'
![Alt text](code_image/section_1_3.png)

The visualized result by using OpenCV
![Alt text](result_image/range_image.png)

For the (ID_S1_EX2) task, the code changes in 'loop_over_dataset.py' and 'objdet_pcl.py' are shown as below:
![Alt text](code_image/section1_4.png)
![Alt text](code_image/section_1_5.png)
The frame will change to next by just clicking "Q".

The sample of vehicles from BEV map (one consists of pedestrian):

![Alt text](result_image/pcd_1.png)
![Alt text](result_image/pcd_2.png)
![Alt text](result_image/pcd_3.png)
![Alt text](result_image/pcd_4.png)
![Alt text](result_image/pcd_5.png)
![Alt text](result_image/pcd_6.png)
![Alt text](result_image/pcd_7.png)
![Alt text](result_image/pcd_8.png)
![Alt text](result_image/pcd_9.png)
![Alt text](result_image/pcd_10.png)

Most of the rear bumpers, headover lights, car front lights are included inside the images. From different angle of perspective, some of the parts of vehicles cannot be displayed well due to the intensity of light or the opposite side of the vehicle are being blocked by the body itself. Some of the obstacles in front of the vehicle can still be clearly preojected. Although the reviews are subjected to vehicles, one of the images (pcd_7.png) is showing two people crossing the road. 

## Section 2: Create Birds-Eye View (BEV) from Lidar PCL
These are the changes in 'loop_over_dataset.py' and 'objdet_pcl.py' to run this section
![Alt text](code_image/section_2_1.png)
![Alt text](code_image/section_2_2.png)

The result of BEV map:
![Alt text](result_image/bev_1.png)

For the task (ID_S2_EX3), the changes are done in 'objdet_pcl.py' as below:
![Alt text](code_image/section_2_4.png)

The result of just intensity channel without normalizing height:
![Alt text](result_image/bev_2.png)

The result of normalizing height:
![Alt text](result_image/bev_3.png)

Both of the focus parts are same.

## Section 3: Model-based Object Detection in BEV Image
This section is requiring to cloning the Complex-YOLO github repo and understand the configuration from 'parse_test_configs()' in the 'test.py'. Add the configuration to configs file where the model is "fpn_resnet" in 'objdet_detect.py'. 

To run this section, some changes are needed to be made in 'loop_over_dataset.py' and 'objdet_detect.py'.
![Alt text](code_image/section_3_1.png)
![Alt text](code_image/section_3_2.png)

The result of object detection on BEV coordinate and metric coordinate on real world image.
![Alt text](result_image/bounding_box.png)

## Section 4: Performance Evaluation for Object Detection
In this section, the number of true positive, false positive and false negative are found based on the IoU between detected objects and ground-truth label. The provided data are not retrained, but from the Waymo Dataset. This model is using DarkNet model from Complex-YOLO architecture. After that, the precision-recall curve along 100 frames are plot and the precision and recall are calculated. Another graph is shown where the ground-truth label is taken as objects. 

To run this section, some changes are needed to be made in 'loop_over_dataset.py' and 'objdet_eval.py'.
![Alt text](code_image/section_5_1.png)
![Alt text](code_image/section_5_2.png)
![Alt text](code_image/section_5_3.png)
![Alt text](code_image/section_5_4.png)
![Alt text](code_image/section_5_5.png)

After running this section, the precision and recall will be calculated. 
Precision : 0.95066
Recall : 0.94444

The graph plot is shown as below:
![Alt text](result_image/pr_curve.png)

As required, the ground-truth labels are taken as objects to do the same step as above. In the 'loop_over_dataset.py', change the following code,

<code> configs_det.use_labels_as_objects = True </code>

Precision : 1.0
Recall : 1.0

![Alt text](result_image/pr_curve2.png)

## Conclusion
This project is helping in understanding the data struture of lidar sensor, converting to 3D point cloud from the data, transforming the 3D point cloud to BEV map, evaluating the performance of Complex-YOLO model by calculating precision and recall, analysing through the plot graph of Precision-Recall curve. This project also helps in understanding the basic of sensor fusion and usage of model before going to the next topic with combining object detection. 
