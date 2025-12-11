#include "nms.h"

float jetnet::box_iou(cv::Rect2f a, cv::Rect2f b)
{
    const cv::Rect2f intersection = a & b;
    const float intersection_area = intersection.area();
    const float union_area = a.area() + b.area() - intersection_area;

    return intersection_area / union_area;
}

void jetnet::nms(std::vector<Detection>& detections, float thresh)
{
    size_t i, j, k;

    // suppress by setting detection probabilities to zero
    for (i = 0; i < detections.size(); ++i) {
        for (j = i+1; j < detections.size(); ++j) {
            if (box_iou(detections[i].bbox, detections[j].bbox) > thresh) {
                // suppress individual classes to keep supporting multi class label detections
                for (k = 0; k < detections[i].probabilities.size(); ++k) {
                    if (detections[i].probabilities[k] < detections[j].probabilities[k])
                        detections[i].probabilities[k] = 0;
                    else
                        detections[j].probabilities[k] = 0;
                }
            }
        }
    }

    // delete detections where all probabilities are zero
    for (i=detections.size()-1; i < detections.size(); --i) {
        if (std::all_of(detections[i].probabilities.begin(), detections[i].probabilities.end(), [](float v) { return v==0; }))
            detections.erase(detections.begin() + i);
    }
}

void jetnet::nms_sort(std::vector<Detection>& detections, float thresh)
{
    size_t i, j, k;
    size_t num_classes = 0;

    if (detections.size() != 0)
        num_classes = detections[0].probabilities.size();

    // suppress by setting detection probabilities to zero
    for (k = 0; k < num_classes; ++k) {
        // sort detections in descending order based on class k's probability
        std::sort(detections.begin(), detections.end(),
                  [=](const Detection& a, const Detection& b) -> bool
        {
            return a.probabilities[k] > b.probabilities[k];
        });
        for (i = 0; i < detections.size(); ++i) {
            if (detections[i].probabilities[k] == 0)
                continue;
            for (j = i+1; j < detections.size(); ++j) {
                if (box_iou(detections[i].bbox, detections[j].bbox) > thresh)
                    detections[j].probabilities[k] = 0;
            }
        }
    }

    // delete detections where all probabilities are zero
    for (i=detections.size()-1; i < detections.size(); --i) {
        if (std::all_of(detections[i].probabilities.begin(), detections[i].probabilities.end(), [](float v) { return v==0; }))
            detections.erase(detections.begin() + i);
    }
}
