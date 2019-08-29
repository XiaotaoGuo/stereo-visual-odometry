//
// Created by guoxt on 19-8-28.
//

#include "config.h"



int main(int argc, char** argv){
    string dataset_path = argv[1];
    stringstream left_img_path, right_img_path, calib_path;
    left_img_path << dataset_path << "image_0/";
    right_img_path << dataset_path << "image_1/";
    calib_path << dataset_path << "calib.txt";

    readCalib();

}


