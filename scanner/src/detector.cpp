
#include "detector.h"
#include <android/log.h>
#include <cstdlib>

/** \brief Constructor; passes the required file directories, sets the desired detection method and
* initializes some class parameters
*
* \param [in]     assetsDir The directory of the asset files
* \param [in]     dm        Refers to the desired detection algorithm. 0 if it is "Yolo-v3", 1 if it is
*                           "Tiny Yolo", 2 if it is "MobileNet SSD"
* \param [in]     conf		Determines the amount of "confidence" parameter used by detectors to threshold
*							the detected objects based on assurance rate of detection accuracy
*/
Detector::Detector(std::string assetsDir, DetectionMethod dm, float conf, float nms)
{
    this->detectionMethod = dm;

    if (dm == DetectionMethod::YOLO_V3)
    {
        std::string model = assetsDir + "/yolov3.cfg";
        std::string config = assetsDir + "/yolov3.weights";

        this->net = cv::dnn::readNetFromDarknet(model, config);
        this->net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        this->net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }
    else if (dm == DetectionMethod::YOLO_TINY)
    {
        std::string model = assetsDir + "/yolov3-tiny.cfg";
        std::string config = assetsDir + "/yolov3-tiny.weights";

        this->net = cv::dnn::readNetFromDarknet(model, config);
        this->net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        this->net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }
    else if (dm == DetectionMethod::MN_SSD)
    {
        std::string model = assetsDir + "/MobileNetSSD_deploy.prototxt.txt";
        std::string config = assetsDir + "/MobileNetSSD_deploy.caffemodel";

        this->net = cv::dnn::readNetFromCaffe(model, config);
    }

    this->confidence = conf;
    this->nmsThreshold = nms;

    this->assets_dir = assetsDir;
}

/** \brief The main function which detects existing objects within the input image
*
* \param [in]   frame  	    cv::Mat; The camera image in which objects must be detected
* \param [out]  objects     std::vector<Object>; A list of "Object" structure objects each including
* 					        obtained information about each detected object
*
* This function can be called whenever the camera image is available
*/
void Detector::detect(cv::Mat &frame, std::vector<Object> &objects)
{
    cv::Mat blob;

    if (this->detectionMethod == YOLO_V3 || this->detectionMethod == YOLO_TINY)
    {
        cv::dnn::blobFromImage(frame, blob, 1/255.0, cv::Size(416, 416), cv::Scalar(0,0,0), true, false);
        this->net.setInput(blob);
        std::vector<cv::Mat> outs;
        this->net.forward(outs, getOutputsNames(net));
        yolov3PostProcess(frame, outs, objects);
    }
    else if (this->detectionMethod == MN_SSD)
    {
        cv::dnn::blobFromImage(frame, blob, 0.007843, cv::Size(300, 300), cv::Scalar(127.5, 127.5, 127.5), false);
        this->net.setInput(blob);
        cv::Mat prob = this->net.forward();
        ssdPostProcess(frame, prob, objects);
    }
}

/** \brief Extracts the detected objects by yolo algorithms into a list of Object instances, based on detection confidence
*
* \param [in]   frame  	    The camera image in which objects have been detected
* \param [out]  outs        The output of the detector algorithm; A list of matrices containing information
*                           about each detected object
* \param [out]  objects     A list of Object instances each containing the obtained data about a detected
*                           object
*
* This function can be called whenever the camera image is available. It is called if the user chooses the
* YOLO-V3 or YOLO-TINY algorithm to detect objects
*/
void Detector::yolov3PostProcess(cv::Mat& frame, const std::vector<cv::Mat>& outs, std::vector<Object> &objects)
{
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    for (size_t i = 0; i < outs.size(); ++i)
    {
        float* data = (float*)outs[i].data;
        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
        {
            cv::Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
            cv::Point classIdPoint;
            double cnf;

            cv::minMaxLoc(scores, 0, &cnf, 0, &classIdPoint);
            //coco.names: person, bicycle, car, motorbike, aeroplane, bus, train, truck, ...
            if ((float)cnf > this->confidence && (classIdPoint.x == 0 || classIdPoint.x == 2 || classIdPoint.x == 3 || classIdPoint.x == 5 || classIdPoint.x == 7))
            {
                int centerX = (int)(data[0] * frame.cols);
                int centerY = (int)(data[1] * frame.rows);
                int width = (int)(data[2] * frame.cols);
                int height = (int)(data[3] * frame.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;

                confidences.push_back((float)cnf);
                Object obj;
                obj.box = cv::Rect(left, top, width, height);
                obj.picture = frame(obj.box);
                if (classIdPoint.x == 0)
                    obj.type = Object::PERSON;
                else
                    obj.type = Object::CAR;
                objects.push_back(obj);
            }
        }
    }
}

/** \brief Extracts the detected objects by ssd algorithm into a list of Object instances, based on detection confidence
*
* \param [in]   frame  	    The camera image in which objects have been detected
* \param [out]  outs        The output of the detector algorithm; A list of matrices containing information
*                           about each detected object
* \param [out]  objects     A list of Object instances each containing the obtained data about a detected
*                           object
*
* This function can be called whenever the camera image is available. It is called if the user chooses the
* MobileNet-SSD algorithm to detect objects
*/
void Detector::ssdPostProcess(cv::Mat& frame, cv::Mat &outs, std::vector<Object> &objects)
{
    cv::Mat detectionMat(outs.size[2], outs.size[3], CV_32F, outs.ptr<float>());

    for (int i = 0; i < detectionMat.rows; i++)
    {
        int idx = static_cast<int>(detectionMat.at<float>(i, 1));
        float cnf = detectionMat.at<float>(i, 2);

//        CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
//                "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
//                "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
//                "sofa", "train", "tvmonitor"]
        if (cnf > this->confidence && (idx == 15 || idx == 6 || idx == 7))
        {
            int xLeftBottom = static_cast<int>(detectionMat.at<float>(i, 3) * frame.cols);
            int yLeftBottom = static_cast<int>(detectionMat.at<float>(i, 4) * frame.rows);
            int xRightTop = static_cast<int>(detectionMat.at<float>(i, 5) * frame.cols);
            int yRightTop = static_cast<int>(detectionMat.at<float>(i, 6) * frame.rows);

            cv::Rect box((int)xLeftBottom, (int)yLeftBottom,
                            (int)(xRightTop - xLeftBottom),
                            (int)(yRightTop - yLeftBottom));

            Object obj;
            obj.box = box;
            obj.picture = frame(obj.box);
            if (idx == 15)
                obj.type = Object::PERSON;
            else
                obj.type = Object::CAR;
            objects.push_back(obj);

        }
    }
}

/** \brief Gets the names of the yolo output layers
*
* \param [in]   net     The network object for which the output layer names are to be determined
*
* \returns      A list of names corresponding to yolo output layers
*
* This function is called during a call to yolo network forward pass, to get the result of the output
* layers
*/
std::vector<cv::String> Detector::getOutputsNames(const cv::dnn::Net& net)
{
    static std::vector<cv::String> names;
    if (names.empty())
    {
        //Get the indices of the output layers, i.e. the layers with unconnected outputs
        std::vector<int> outLayers = net.getUnconnectedOutLayers();

        //get the names of all the layers in the network
        std::vector<cv::String> layersNames = net.getLayerNames();

        // Get the names of the output layers in names
        names.resize(outLayers.size());
        for (size_t i = 0; i < outLayers.size(); ++i)
            names[i] = layersNames[outLayers[i] - 1];
    }
    return names;
}

/** \brief Draws bounding boxes around the detected objects on the input image
*
* \param [in,out] 	frame  		cv::Mat; The camera image in which detected objects are to be drawn. It
* 								must be exactly the same as the detect() function input
* \param [in]  	    objects 	std::vector<Object>; A list of detected objects that are to be drawn
*
* This function MUST be called after detect() function call with the same input frame for which detect()
* function is called
*/
void Detector::drawDetections(cv::Mat &dst, std::vector<Object> &objects)
{
    for(auto & object : objects)
    {
        if (object.action == Object::REMAIN)
            continue;

        cv::Scalar color;
        if(object.type == Object::PERSON)
            color = cv::Scalar(0,0,255);
        else if (object.type == Object::CAR)
            color = cv::Scalar(42,42,165);

        rectangle(dst, object.box, color, 3, 1);
    }
}