/*
 * Params.h
 *
 *  Created on: 13 Nov 2015
 *      Author: osboxes
 */

#ifndef PARAMS_H_
#define PARAMS_H_

#define numNodes 16
#define maxLTMDepth 16
#define imageH 240
#define imageW 320
#define tauIntensity 240
#define meanDepth 345.018

//CHANGE FOR TESTS
#define dir "/home/osboxes/workspace/HandTracking/"
//to set accordingly to the images in "labels.txt"
#define numImages 12//331006
//defines how long to apply split nodes; important parameter
#define InfoGainThreshold 4
//defines if images from "labels.txt" are randomly added at each stage or all are added at the beginning
#define randomChoosingImages false
//defines how many random values are generated before the optimal u and v are chosen
#define testNums 64
//defines how big should be the square where u and v are generated(note it gets 2 times smaller for last stages)
#define randOffsetMax 64


#endif /* PARAMS_H_ */
