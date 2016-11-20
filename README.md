# HandTracking

##Build Instructions

1. Make sure to have installed the libraries opencv, boost and eigen on yoour system.
2. Run the provided makefile

##Run Instructions

###Needed files 

 1. For calculating the mean geodesic distances of the training samples, you need to:
 - put "labels.txt" in the "InputData" folder; provided here: http://www.iis.ee.ic.ac.uk/dtang/hand.html 
 - put the images in "InputData/Depth" folder (Warning! The images must be png, **8bit** grayscale);  provided here: https://www.dropbox.com/s/ze1lqvgj46zmdas/Depth.rar?dl=0 
 2. For calculating the Latent Tree you should provide "MeanDistances.txt" file in "InputData" folder  with the mean distances from each node to the others 
 3. For building a latent rergession tree, put "LatentTreeModel" dot file and "labels.txt" in the InputData directory

###Path to directory 

Give the absolute path to the HanTracking folder in the main.cpp file or in tne Params.h file

###Parameters

1. Choose which part to run from the main.cpp file
2. If you want to test with different images, set them in the "labels.txt" and adjust the number of images in "Params.h" file
3. in "Params.h" you can adjust parameters for the training of the LRT

