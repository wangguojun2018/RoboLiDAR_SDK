/**
 *****************************************************************************
 * COPYRIGHT STATEMENT
 * Copyright (c) 2018, Robosense Co.,Ltd. - www.robosense.ai
 * All Rights Reserved.
 *
 * You can not use, copy or spread without official authorization.
 *****************************************************************************
 *
 * Author: Robosense Perception Group
 * Version: 2.0
 * Date: 2017.11
 *
 * DESCRIPTION
 *
 * Robosense classification module, for object recognition.
 *
 */

#ifndef CLASSIFIER_BASE_2_H
#define CLASSIFIER_BASE_2_H
#include <iostream>
#include <fstream>
#include <vector>

/**
 * @brief classify the input sample, the multi-classifier is of 5 classes
 * @param[in] sample: vector of object features
 * @return : vector of object classify probability
 */
std::vector<float> classifierBase1(std::vector<float> &sample);

#endif
