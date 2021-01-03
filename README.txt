This branch server for object detection and segmentation using Mask RCNN.

For the first time to run the dockerfile and build the Mask RCNN node, please follow the instructions in 
"docker_img_mrcnn/fromstart_docker_commands_mrcnn.txt".

After creating a docker image, if you would like to re-connect to the same docker image, please follow the instructions in "docker_img_mrcnn/late_building_commands.txt".

After a succesfull run, images will be segmented and objects will be identified in it.

Mask RCNN detects both bounding box and segmentation mask for object instances in images. 
For now, results are published as following:
	For each image to be segmented, an .txt file is created in docker_img_mrcnn/mrcnn/src/mask_rcnn_ros/results. The name of the file is the same with the name of image that is been processed.
	
	The content of the txt file is the following:

	1st obj’s bounding box identifer -> x_offset y_offset height width (seperated by 1 space character)
	1st obj’s class id
	1st obj’s class name
	2nd obj’s bounding box identifer -> x_offset y_offset height width
	2nd obj’s class id
	2nd obj’s class name

	Example: name of text file : 1341847986.561706.txt
	323 38 93 26
	40
	bottle
	230 53 97 33
	40
	bottle
	234 403 72 242
	57
