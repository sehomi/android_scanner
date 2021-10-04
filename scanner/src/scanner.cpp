//
// Created by a on 6/7/2021.
//

#include "scanner.h"
#include "sweeper.h"


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
Scanner::Scanner(std::string assetsDir, std::string logsDir, DetectionMethod dm, int log_mode, float hva_, int maxdist)
{
    hva = hva_;
    max_dist = maxdist;

    if (dm == MN_SSD)
        detector = new Detector(assetsDir, DetectionMethod::MN_SSD, 0.1, 0.4);
    else if (dm == YOLO_V3)
        detector = new Detector(assetsDir, DetectionMethod::YOLO_V3, 0.1, 0.4);
    else if (dm == YOLO_TINY)
        detector = new Detector(assetsDir, DetectionMethod::YOLO_TINY, 0.1, 0.4);

//    std::string logFolder = "/storage/emulated/0/LogFolder/log_2021_07_08_20_05_38/";
//    std::string logFolder = "/storage/emulated/0/LogFolder/log_2021_08_18_18_52_14/";
//    std::string logFolder = "/storage/emulated/0/LogFolder/log_2021_08_18_18_59_38/";
//    std::string logFolder = "/storage/emulated/0/LogFolder/log_2021_08_18_19_10_35/";
    std::string logFolder = "/storage/emulated/0/LogFolder/log_2021_10_04_18_11_02/";

    if (log_mode == 0)
        logger = new Logger(logsDir, true, false, logFolder);
    else if (log_mode == 1)
        logger = new Logger(logsDir, false, true, logFolder);
    else
        logger = new Logger(logsDir, false, false, logFolder);

    sweeper = new SweeperGeometry::Sweeper();
    motionDetector = new MotionDetector(hva_);

    assets_dir = assetsDir;
//    FloodUtils::setdir(assetsDir.c_str());
//    Grid<NasaGridSquare>::cache_limit = 20;
//    grid = new Grid<NasaGridSquare>();

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

bool Scanner::scan(ImageSet &imgSt, Mat &detections_img, Mat &movings_img, std::vector<Object> &objects, double stamp)
{
    if (!logger->readFromLog)
        return false;

    if (lastProcessStamp == -1){
        lastProcessStamp = stamp;
        lastProcessImgSetStamp = imgSt.time;
    }
    else{
        double diff1 = stamp - lastProcessStamp;
        double diff2 = imgSt.time - lastProcessImgSetStamp;

        __android_log_print(ANDROID_LOG_VERBOSE, "android_scanner", "diff %f %f %f %f", (float)diff1, (float)diff2, (float)stamp, (float)imgSt.time);

        if (diff2 < diff1)
            return false;

        lastProcessStamp = stamp;
        lastProcessImgSetStamp = imgSt.time;
    }

    std::vector<Object> moving_objects;

    // TODO: hva must be set automatically from log
    //       This is for mavic mini:
    hva = 66.0;
    if (!initialInfoSet)
    {
        setInitialInfo(imgSt);
        initialInfoSet = true;
    }

    motionDetector->detect(imgSt, movings_img, moving_objects, fovPoses, true);

    detector->detect(imgSt.image, objects);
    detector->drawDetections(detections_img, objects);

    objects.insert(objects.end(), moving_objects.begin(), moving_objects.end());
    camToMap(objects, imgSt);

//    for (auto & obj : objects)
//    {
//        __android_log_print(ANDROID_LOG_VERBOSE, "android_scanner----", "object type: %s", std::to_string(obj.type).c_str());
//        __android_log_print(ANDROID_LOG_VERBOSE, "android_scanner----", "object width: %s", std::to_string(obj.picture.cols).c_str());
//        __android_log_print(ANDROID_LOG_VERBOSE, "android_scanner----", "object height: %s", std::to_string(obj.picture.rows).c_str());
//    }

    associate(objects);
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
bool Scanner::scan(std::vector<Object> &objects, Mat &detections, Mat &movings_img, int det_mode = 0, bool rgba = false, bool isFix = false)
{
    if (logger->readFromLog)
        return false;

    __android_log_print(ANDROID_LOG_VERBOSE, "android_scanner scanner mode ", "%s", std::to_string(det_mode).c_str());

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
        __android_log_print(ANDROID_LOG_VERBOSE, "scan nn1 fov size:", "%s", std::to_string(fovPoses.size()).c_str());
        motionDetector->detect(imgSt, movings_img, objects, fovPoses, isFix);
        __android_log_print(ANDROID_LOG_VERBOSE, "scan ", "nn2");
    }
    else
        {
            __android_log_print(ANDROID_LOG_VERBOSE, "scan ", "nn3");
            if (rgba) {
            Mat img;
            cvtColor(imgSt.image, img, COLOR_RGBA2BGR);
            cvtColor(detections, detections, COLOR_RGBA2BGR);
            detector->detect(img, objects);
        }
        else {
                __android_log_print(ANDROID_LOG_VERBOSE, "scan ", "nn4");
                detector->detect(imgSt.image, objects);
            }

        detector->drawDetections(detections, objects);
            __android_log_print(ANDROID_LOG_VERBOSE, "scan ", "nn5");

        movings_img = cv::Mat::zeros(imgSt.image.rows, imgSt.image.cols, CV_8UC3);
            __android_log_print(ANDROID_LOG_VERBOSE, "scan ", "nn6");

        }

    camToMap(objects, imgSt);

    associate(objects);
    __android_log_print(ANDROID_LOG_VERBOSE, "scan ", "nn7");

    return true;
}

/** \brief Calculates the corresponding map location for a couple of bounding boxes (within image) given at input
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
    std::vector<bool> show_permissions;

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

    for (auto &object : objects) {
        object.distance = sqrt(
                pow((refX - object.location.x), 2) + pow((refY - object.location.y), 2));
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

/** \brief Scales a unit direction vector in such a way that it ends on the ground or on a maximum length
*
* \param [in]  v        The input direction vector that is to be scaled
* \param [in]  output   The output scaled vector
* \param [out] z        The altitude of the camera
*
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

/** \brief Calculates a 3D direction vector that starts at the camera focal point and ends at a given image point
*
* \param [in]  x    The horizontal coordinate of the image point
* \param [in]  y    The vertical coordinate of the image point
* \param [out] z    The output 3D direction vector
*
* This function is called when the camera focal length (f) and x-axis and y-axis optical center (cx and cy
* respectively) are predetermined. The function scales the direction vector to touch the ground if its
* length is less than a maximum length (the parameter max_dist). Otherwise, it scales the direction vector
* into the maximum length
*/
void Scanner::calcDirVec(float x, float y, Eigen::VectorXd &z)
{
    Eigen::VectorXd w(3);

    w<<(double) (x-cx),
       (double) (y-cy),
       (double) f;

    z = w*(1/w.norm());
}

/** \brief Modifies and updates the Object list of the UI online map
*
* \param [in,out]   objects     The list of last detected objects
*
* This function is called when the object detection and mapping procedure is completed. The function decides
* for each object whether it should be added, remained or updated in the UI online map
*/
void Scanner::associate(std::vector<Object> &objects)
{
    for (int k = 0; k < objectPoses.size(); k++)
    {
        if(objectPoses[k].action == Object::UPDATE || objectPoses[k].action == Object::ADD)
            objectPoses[k].action = Object::REMAIN;
    }

    std::vector<Object> newObjs;

    for(auto & object : objects)
    {
        bool found = false;
        for (int k = 0; k < objectPoses.size(); k++)
        {
            if(objectPoses[k].type != object.type) continue;

            double dist = sqrt(((objectPoses[k].location.x-object.location.x)*(objectPoses[k].location.x-object.location.x))+
                    ((objectPoses[k].location.y-object.location.y)*(objectPoses[k].location.y-object.location.y)));
            if (dist < 3)
            {
                objectPoses[k] = object;
                objectPoses[k].action = Object::UPDATE;
                objectPoses[k].lastIdx = k;
                found = true;
            }
        }
        if (!found)
        {
            newObjs.push_back(object);
            newObjs.at(newObjs.size()-1).action = Object::ADD;
        }
    }

    objectPoses.insert(objectPoses.end(), newObjs.begin(), newObjs.end());
    objects = objectPoses;
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
// TODO: fov calculation is not necessary when on the ground or in horizontal fov case
bool Scanner::calcFov(std::vector<Object> &objects)
{
    if(logger->readFromLog)
        return false;

    ImuSet imuSt;
    if (!logger->getImuSet(imuSt) || abs(imuSt.time-lastFovTime)<fovDelay) {
        return false;
    }
    lastFovTime = imuSt.time;

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

    fovPoses.clear();
    fovPoses = objects;

    std::vector<Object> swept_area;
//    TODO: Uncomment this after fixing sweeper to match "Object" structure:
    sweeper->update(objects, swept_area);
//// Note: Add the swept_area to the fov objects vector like below. In the "putIntoArray" function, the objects will be recognized using object.type attribute
    objects.insert(objects.end(), swept_area.begin(), swept_area.end());

    return true;
}

bool Scanner::calcFov(std::vector<Object> &objects, ImageSet &imgSt)
{
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
    sweeper->update(objects, swept_area);
    objects.insert(objects.end(), swept_area.begin(), swept_area.end());

    return true;
}

double Scanner::elev(ImuSet &imuSt)
{
    FloodUtils::setdir(assets_dir.c_str());
    Grid<NasaGridSquare>::cache_limit = 20;
    grid = new Grid<NasaGridSquare>();

    return (double) grid->height((float)imuSt.lng,(float)imuSt.lat);
}

double Scanner::elev()
{
    FloodUtils::setdir(assets_dir.c_str());
    Grid<NasaGridSquare>::cache_limit = 20;
    grid = new Grid<NasaGridSquare>();

    ImuSet imuSt;
    if (!logger->getImuSet(imuSt)) {
        return 0.0;
    }

    return (double) grid->height((float)imuSt.lng,(float)imuSt.lat);
}

bool Scanner::elevDiff(double newLat, double newLon, double &diff)
{
    FloodUtils::setdir(assets_dir.c_str());
    Grid<NasaGridSquare>::cache_limit = 20;
    grid = new Grid<NasaGridSquare>();

    if (!useElev or !initialInfoSet){
        diff = 0;
        return true;
    }

    double newElev = (double) grid->height((float)newLon,(float)newLat);
    double initElev = (double) grid->height((float)firstLocation.lng,(float)firstLocation.lat);

    __android_log_print(ANDROID_LOG_VERBOSE, "imageToMap", " %f %f %f %f %f %f", (float)newLon,(float)newLat, (float)firstLocation.lng,(float)firstLocation.lat, (float)newElev, (float)initElev );

    if (newElev == -32768 || initElev == -32768) {
        diff = 0;
        return false;
    }
    else
    {
        diff = newElev - initElev;
        return true;
    }
}

/** \brief converts image points into map points
*
* \param [in]       roll        Camera roll angle at the moment in which image is captured
* \param [in]       pitch       Camera pitch angle at the moment in which image is captured
* \param [in]       azimuth     Camera azimuth angle at the moment in which image is captured
* \param [in]       lat         Camera location latitude at the moment in which image is captured
* \param [in]       lng         Camera location longitude at the moment in which image is captured
* \param [in]       alt         Camera location altitude at the moment in which image is captured
* \param [in,out]   objects     A set of Object instances containing each image point coordinates. The
*                               calculated corresponding map location is written on each objects as well
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
    double diff = 0;
    bool res = elevDiff(lat,lng,diff);
    pos << y, x, -(alt+diff);

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
    else {
        firstLocation.lat = lat;
        firstLocation.zone = zone;
        firstLocation.zone = LatLonToUTMXY(lat, lng, 0, firstLocation.x, firstLocation.y);
    }
}
