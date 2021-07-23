// color map following the SemanticKitti point cloud color
#include "color_table.h"
#include <vector>
#include <algorithm>
#include <iostream>

colorTable::colorTable()
{
    labelList = {
        "unlabeled",
        "outlier",
        "car",
        "bicycle",
        "bus",
        "motorcycle",
        "on-rails",
        "truck",
        "other-vehicle",
        "person",
        "bicyclist",
        "motorcyclist",
        "road",
        "parking",
        "sidewalk",
        "other-ground",
        "building",
        "fence",
        "other-structure",
        "lane-marking",
        "vegetation",
        "trunk",
        "terrain",
        "pole",
        "traffic-sign",
        "other-object",
        "moving-car",
        "moving-bicyclist",
        "moving-person",
        "moving-motorcyclist",
        "moving-on-rails",
        "moving-bus",
        "moving-truck",
        "moving-other-vehicle"};


    labelID = {
        0, 1, 10, 11, 13, 15, 16, 18, 20, 30, 31, 
        32, 40, 44, 48, 49, 50, 51, 52, 60, 70, 71, 72, 80, 81, 99, 
        252, 253, 254, 255, 256, 257, 258, 259};

    // color map format: bgr
    colorList = {
        {0, 0, 0},
        {0, 0, 255},
        {245, 150, 100},
        {245, 230, 100},
        {250, 80, 100},
        {150, 60, 30},
        {255, 0, 0},
        {180, 30, 80},
        {255, 0, 0},
        {30, 30, 255},
        {200, 40, 255},
        {90, 30, 150},
        {255, 0, 255},
        {255, 150, 255},
        {75, 0, 75},
        {75, 0, 175},
        {0, 200, 255},
        {50, 120, 255},
        {0, 150, 255},
        {170, 255, 150},
        {0, 175, 0},
        {0, 60, 135},
        {80, 240, 150},
        {150, 240, 255},
        {0, 0, 255},
        {255, 255, 50},
        {245, 150, 100},
        {200, 40, 255},
        {30, 30, 255},
        {90, 30, 150},
        {255, 0, 0},
        {250, 80, 100},
        {180, 30, 80},
        {255, 0, 0}};
    
}

std::vector<int> colorTable::getColor(int id)
{
    std::vector<int>::iterator it = std::find(labelID.begin(), labelID.end(), id);
    if (it == labelID.end())
        return {-1, -1, -1};
    
    int index = it - labelID.begin();
    return colorList[index];
}

std::string colorTable::getLabels(int id)
{
    std::vector<int>::iterator it = std::find(labelID.begin(), labelID.end(), id);
    int index = it - labelID.begin();
    return labelList[index];   
}
