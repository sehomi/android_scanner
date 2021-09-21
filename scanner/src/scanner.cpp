//
// Created by a on 6/7/2021.
//

#include "scanner.h"


/** \brief Constructor; passes the required file directories and initializes some class parameters
*
* \param [in]     assetsDir   The directory of the asset files
* \param [in]     logsDir     The directory in which the synchronized sensor data is saved - if desired
* \param [in]     dm          Refers to the desired detection algorithm. 0 if it is "yolov3", 1 if it is
*                             "tiny yolo", and 2 if it is "MobileNet SSD"
* \param [in]     log_mode    If false, it is the normal functionality. Otherwise, offline data is read from log
* \param [in]     hva_        the camera horizontal view angle
* \param [in]     maxDist     Refers to the maximum distance at which the objects are mapped in the online map
*/
Scanner::Scanner(std::string assetsDir, std::string logsDir, DetectionMethod dm, bool log_mode, float hva_, int maxdist)
{
    hva = hva_;
    max_dist = maxdist;

    if (dm == MN_SSD)
        detector = new Detector(assetsDir, DetectionMethod::MN_SSD, 0.4, 0.4);
    else if (dm == YOLO_V3)
        detector = new Detector(assetsDir, DetectionMethod::YOLO_V3, 0.4, 0.4);
    else if (dm == YOLO_TINY)
        detector = new Detector(assetsDir, DetectionMethod::YOLO_TINY, 0.4, 0.4);

    logger = new Logger(logsDir, log_mode, true, "/storage/emulated/0/LogFolder/log_2021_07_08_20_05_38/");
    sweeper = new SweeperGeometry::Sweeper();
    motionDetector = new MotionDetector(hva_);
}

/** \brief Sets the initial information related to camera as the first image is received
*
* \param [out]  imgSt   An ImageSet instance containing first camera image synchronized with IMU and GPS data
*
* This function sets the focal length (f) and x-axis and y-axis optical center of camera (cx and cy respectively)
* simply assuming that the camera focal point is exactly placed in front of image center. Usually this assumption
* is not true. However the little error is negligible
*/
void Scanner::setInitialInfo(ImageSet &imgSt)
{
    int width = imgSt.image.size().width;
    int height = imgSt.image.size().height;

    // TODO: Set the true cx, cy and f based on camera params instead of this simplification
    f = (float) (0.5 * width * (1.0 / tan((hva/2.0)*PI/180)));
    cx = (float) width/2;
    cy = (float) height/2;

    setReferenceLoc(imgSt.lat, imgSt.lng, false);
}

bool Scanner::scan(ImageSet &imgSt, Mat &detections_img, Mat &movings_img, std::vector<Object> &objects)
{
    if (!logger->readFromLog)
        return false;

    std::vector<Object> moving_objects;

    // TODO: hva must be set automatically from log
    //       This is for mavic mini:
    hva = 66.0;
    if (!initialInfoSet)
    {
        setInitialInfo(imgSt);
        initialInfoSet = true;
    }

    motionDetector->detect(imgSt, movings_img, moving_objects, fovPoses);

    detector->detect(imgSt.image, objects);
    detector->drawDetections(detections_img, objects);

    objects.insert(objects.end(), moving_objects.begin(), moving_objects.end());
    camToMap(objects, imgSt);

    return true;
}

/** \brief The main function which detects motion and objects, and maps them into the online map
*
* \param [out]  objects     std::vector<Objects>; A list containing last location, last picture,
*                           and some other data for each detected object
* \param [out]  detections  cv::Mat; An image with detected objects highlighted within
* \param [out]  movings_img cv::Mat; An image with moving objects highlighted within
* \param [in]   det_mode    Integer; If 1, the function detects moving objects. Otherwise, objects
*                           such as persons, car, etc. are detected
*
* \returns      true if the required data is provided and so the outputs are achieved successfully
*
* This function is called with sensor data received and synchronized previously
*/
bool Scanner::scan(std::vector<Object> &objects, Mat &detections, Mat &movings_img, int det_mode = 0, bool rgba = false)
{
    if (logger->readFromLog)
        return false;

    ImageSet imgSt;

    if (!logger->getImageSet(imgSt))
    {
        __android_log_print(ANDROID_LOG_ERROR, "android_scanner", "Logger did not provide image set for scanner");
        return false;
    }

    if (!initialInfoSet)
    {
        setInitialInfo(imgSt);
        initialInfoSet = true;
    }

    detections = imgSt.image.clone();

    if (det_mode == 1)
    {
        motionDetector->detect(imgSt, movings_img, objects, fovPoses);
    }
    else
        {
        if (rgba) {
            Mat img;
            cvtColor(imgSt.image, img, COLOR_RGBA2BGR);
            cvtColor(detections, detections, COLOR_RGBA2BGR);
            detector->detect(img, objects);
        }
        else
            detector->detect(imgSt.image, objects);

        detector->drawDetections(detections, objects);

        movings_img = cv::Mat::zeros(imgSt.image.rows, imgSt.image.cols, CV_8UC3);
    }

    camToMap(objects, imgSt);

//    associate(object_poses);

    return true;
}

/** \brief Calculates the corresponding map location for a couple of image bounding boxes, given at input
*
* \param [in,out]   objects     A list containing which contains the related bounding box in image for each
*                               object
* \param [in]       is          An ImageSet instance containing the image in which objects are detected along
*                               with corresponding IMU and GPS data
*
* This function is mainly responsible for mapping detected objects (moving or custom) for which a bounding
* box exists within the corresponding image
*/
void Scanner::camToMap(std::vector<Object> &objects, const ImageSet& is)
{
    for (auto & object : objects)
    {
        // TODO: center attribute must be initialized in the detector
        object.center = cv::Point(object.box.x + float(object.box.width)/2, object.box.y + float (object.box.height)/2);
    }

    imageToMap(is.roll, is.pitch, is.azimuth, is.lat, is.lng, is.alt, objects);

    calcDistances(objects);
}

/** \brief Calculates the distance from each of input objects to a reference point
*
* \param [in,out]   objects     A list containing which contains the related bounding box in image for each
*                               object
* \param [in]       is          An ImageSet instance containing the image in which objects are detected along
*                               with corresponding IMU and GPS data
*
* If the user location is available, the distance to user location is calculated. Otherwise, the distance is
* calculated with respect to the drone initial location
*/
void Scanner::calcDistances(std::vector<Object> &objects)
{
    double refX, refY;
    refX = (userLocation.zone != 0 ? userLocation.x : firstLocation.x);
    refY = (userLocation.zone != 0 ? userLocation.y : firstLocation.y);

    for (auto & object : objects)
    {
        object.distance = sqrt(pow((refX - object.location.x),2) + pow((refY - object.location.y),2));
    }
}

/** \brief Generates a rotation matrix from 3 given euler angles
*
* \param [in]  roll     Euler roll angle
* \param [in]  pitch    Euler pitch angle
* \param [in]  azimuth  Euler azimuth angle
* \param [out] output   Output rotation matrix
*/
void Scanner::eulerToRotationMat(double roll, double pitch, double azimuth, Eigen::Matrix3d &output)
{
    Eigen::Quaternion<double> q;

    q = Eigen::AngleAxisd(azimuth, Eigen::Vector3d::UnitZ())
         * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

    Eigen::Matrix3d dcm_body_to_inertia = q.matrix();

    output = dcm_body_to_inertia;
}

/** \brief Converts a given GPS location to a UTM location
*
* \param [in]  lat  GPS latitude
* \param [in]  lng  GPS longitude
* \param [out] x    UTM x coordinate
* \param [out] y    UTM y coordinate
*/
void Scanner::gpsToUtm(double lat, double lng, double &x, double &y)
{
    zone = LatLonToUTMXY(lat, lng, 0, x, y);
    isSouth = (lat < 0);
}

/** \brief Scales the input vector such that ...
*
* \param [in]  v        ...
* \param [in]  output   ...
* \param [out] z        ...
*/
bool Scanner::scaleVector(Eigen::VectorXd v, Eigen::VectorXd &output, double z)
{
    if (v[2] > 0)
    {
        double factor = fabs(z) / fabs(v[2]);
        if ((factor*v).norm() < max_dist)
        {
            output = factor*v;
            return true;
        }
    }
    output = max_dist * v;
    return false;
}

/** \brief Scales the input vector such that ...
*
* \param [in]  objects  ...
* \param [in]  output   ...
*/
void Scanner::calcDirVec(float x, float y, Eigen::VectorXd &z)
{
    Eigen::VectorXd w(3);
//    __android_log_print(ANDROID_LOG_VERBOSE, "android_scanner----2", "f: %s", std::to_string(f).c_str());
    w<<(double) (x-cx),
       (double) (y-cy),
       (double) f;
    z = w*(1/w.norm());
}

void Scanner::associate(const std::vector<Location> &object_pos)
{
    for(auto & object : object_pos)
    {
        bool found = false;
        for (int k = 0; k < objectPoses.size(); k++)
        {
            double dist = sqrt(((objectPoses[k].x-object.x)*(objectPoses[k].x-object.x))+((objectPoses[k].y-object.y)*(objectPoses[k].y-object.y)));
            if (dist < 10)
            {
                objectPoses[k] = object;
                markers[k].action = Marker::REMAIN;
//                markers[k].type = Marker::CAR or Marker::PERSON or ...
                markers[k].pos = object;
                found = true;
            }
        }
        if (!found)
        {
            objectPoses.push_back(object);
            Marker mk;
            mk.action = Marker::DRAW;
//            mk.type = Marker::CAR or Marker::PERSON or ...
            mk.pos = object;
            markers.push_back(mk);
        }
    }
}

/** \brief It maps the calculated camera FOV into the online map and updates the swept area
*
* \param [out]  objects     std::vector<Objects>; A list containing the location for points representing
*                           four camera FOV corners
*
* \returns      true if the required data is provided and so the outputs are achieved successfully
*
* This function is called with camera info prepared previously
*/
bool Scanner::calcFov(std::vector<Object> &objects)
{
    if(logger->readFromLog)
        return false;

    ImuSet imuSt;
    if (!logger->getImuSet(imuSt)) {
        return false;
    }

    std::vector<bool> sps;
    int w, h;
    w = logger->img.image.size().width;
    h = logger->img.image.size().height;

    std::vector<Point2f> points{{0, 0}, {(float)w, 0}, {(float)w, (float)h}, { 0, (float)h }};
    for (auto & point : points)
    {
        Object obj;
        obj.type = Object::FOV;
        obj.center.x = point.x;
        obj.center.y = point.y;
        objects.push_back(obj);
    }

    imageToMap(imuSt.roll, imuSt.pitch, imuSt.azimuth, imuSt.lat, imuSt.lng, imuSt.alt, objects);


    std::vector<Object> swept_area;
//    TODO: Uncomment this after fixing sweeper to match "Object" structure:
//    sweeper->update(poses_gps, swept_area);
//// Note: Add the swept_area to the fov objects vector like below. In the "putIntoArray" function, the objects will be recognized using object.type attribute
    objects.insert(objects.end(), swept_area.begin(), swept_area.end());

    return true;
}

bool Scanner::calcFov(std::vector<Object> &objects, ImageSet &imgSt)
{
    __android_log_print(ANDROID_LOG_VERBOSE, "android_scanner", "ji---1-1");

    if (!logger->readFromLog)
        return false;

    int w = imgSt.image.size().width;
    int h = imgSt.image.size().height;
    std::vector<bool> sps;
    std::vector<Point2f> points{{0, 0}, {(float)w, 0}, {(float)w, (float)h}, { 0, (float)h }};
    for (auto & point : points)
    {
        Object obj;
        obj.type = Object::FOV;
        obj.center.x = point.x;
        obj.center.y = point.y;
        objects.push_back(obj);
    }

    imageToMap(imgSt.roll, imgSt.pitch, imgSt.azimuth, imgSt.lat, imgSt.lng, imgSt.alt, objects);

    fovPoses.clear();
    fovPoses = objects;

    std::vector<Object> swept_area;
//    sweeper->update(poses_gps, swept_area);
    objects.insert(objects.end(), swept_area.begin(), swept_area.end());

    return true;
}

/** \brief converts image points into map points
*
* \param [in]  roll     Camera roll angle at the moment in which image is captured
* \param [in]  pitch    Camera pitch angle at the moment in which image is captured
* \param [in]  azimuth  Camera azimuth angle at the moment in which image is captured
* \param [in]  lat      Camera location latitude at the moment in which image is captured
* \param [in]  lng      Camera location longitude at the moment in which image is captured
* \param [in]  alt      Camera location altitude at the moment in which image is captured
* \param [in,out] objects  A set of Object instances containing each image point coordinates. The calculated
*                       corresponding map location is written on each objects as well
*
* Given a set of points in image coordinates along with camera orientation and location at the moment in
* which image is captures, this function assigns a location on map to each given point. This function can
* be called for image points referring to various objects such as camera FOV corners, swept areas, detected
* objects centers and moving objects centers
*/
void Scanner::imageToMap(double roll, double pitch, double azimuth, double lat, double lng, double alt, std::vector<Object> &objects)
{
    double x, y;
    Eigen::VectorXd pos(3);
    Eigen::Matrix3d camToInertia;

    gpsToUtm(lat, lng, x, y);
    pos << y, x, -alt;

    eulerToRotationMat(roll, pitch, azimuth, camToInertia);

    for (auto & object : objects)
    {
        Eigen::VectorXd w_cam(3), w_(3), v(3), p(3), scaled_v(3);
        calcDirVec(object.center.x, object.center.y, w_);
        w_cam << w_[2], w_[0], w_[1];
        v = camToInertia * w_cam;

        object.show = scaleVector(v, scaled_v, pos[2]);

        object.location.x = (scaled_v[1] + pos[1]);
        object.location.y = scaled_v[0] + pos[0];
        object.location.alt = -(scaled_v[2] + pos[2]);
    }

    utmToGps(objects);
}

/** \brief Converts a given UTM location to a GPS location
*
* \param [out]  lat  GPS latitude
* \param [out]  lng  GPS longitude
* \param [in]   x    UTM x coordinate
* \param [in]   y    UTM y coordinate
*/
void Scanner::utmToGps(std::vector<Object> &objs)
{
    for (auto & obj : objs)
    {
        double lat=0, lon=0;
        UTMXYToLatLon (obj.location.x, obj.location.y, zone, isSouth, lat, lon);
        obj.location.lat = lat*180/PI;
        obj.location.lng = lon*180/PI;
    }
}

/** \brief Is used to set user's location simultaneously or set the drone's initial location
*
* \param [in]     lat       The latitude to set
* \param [in]     lng       The longitude to set
* \param [in]     isUserLoc If true, function sets the input latitude and longitude as user's location.
*                           If false, it sets the input as drone's initial location
*/
void Scanner::setReferenceLoc(double lat, double lng, bool isUserLoc)
{
    if (isUserLoc)
        userLocation.zone = LatLonToUTMXY(lat, lng, 0, userLocation.x, userLocation.y);
    else
        firstLocation.zone = LatLonToUTMXY(lat, lng, 0, firstLocation.x, firstLocation.y);
}
