//
// Created by a on 6/7/2021.
//

#include "scanner.h"


Scanner::Scanner(std::string assetsDir, std::string logsDir, DetectionMethod dm, bool log_mode, float hva_, int maxdist)
{
//    __android_log_print(ANDROID_LOG_VERBOSE, "android_scanner", "---------1-1");
    hva = hva_;
    max_dist = maxdist;
//    RAD = PI/180.0;

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

void Scanner::setInitialInfo(ImageSet &imgSt)
{
    int width = imgSt.image.size().width;
    int height = imgSt.image.size().height;
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

//    for (auto & obj : objects)
//    {
//        __android_log_print(ANDROID_LOG_VERBOSE, "android_scanner----", "object type: %s", std::to_string(obj.type).c_str());
//        __android_log_print(ANDROID_LOG_VERBOSE, "android_scanner----", "object width: %s", std::to_string(obj.picture.cols).c_str());
//        __android_log_print(ANDROID_LOG_VERBOSE, "android_scanner----", "object height: %s", std::to_string(obj.picture.rows).c_str());
//    }

//    associate(object_poses);
    return true;
}

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

void Scanner::eulerToRotationMat(double roll, double pitch, double azimuth, Eigen::Matrix3d &output)
{
    Eigen::Quaternion<double> q;

//        q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
//             * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
//             * Eigen::AngleAxisd(azimuth, Eigen::Vector3d::UnitZ());

    q = Eigen::AngleAxisd(azimuth, Eigen::Vector3d::UnitZ())
         * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

    Eigen::Matrix3d dcm_body_to_inertia = q.matrix();

    output = dcm_body_to_inertia;// * dcm_cam_to_body;
}

void Scanner::gpsToUtm(double lat, double lng, double &x, double &y)
{
    zone = LatLonToUTMXY(lat, lng, 0, x, y);
    isSouth = (lat < 0);
}

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

void Scanner::toDirectionVector(std::vector<Rect> &objects, std::vector<Eigen::VectorXd> &output)
{
    std::vector<float> dirVec;
    std::vector<Point2d> center;
    output.clear();

    for(auto & object : objects)
    {
        Eigen::VectorXd w(3);
        calcDirVec(object.x + float(object.width)/2, object.y + float (object.height)/2, w);
        output.push_back(w);
    }
}

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

        // TODO: This method must be applied ... not the above mess
//        v = camToInertia.transpose() * imToCam.transpose() * w_;

//        std::stringstream ss;
//        ss << v;
//        __android_log_print(ANDROID_LOG_VERBOSE, "android_scanner----2", "v: %s", ss.str().c_str());

        object.show = scaleVector(v, scaled_v, pos[2]);

        object.location.x = (scaled_v[1] + pos[1]);
        object.location.y = scaled_v[0] + pos[0];
        object.location.alt = -(scaled_v[2] + pos[2]);

//      TODO : The scanned places must be saved - An overall fov must be generated simultaneously
    }

    utmToGps(objects);
}

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

void Scanner::setReferenceLoc(double lat, double lng, bool isUserLoc)
{
    if (isUserLoc)
        userLocation.zone = LatLonToUTMXY(lat, lng, 0, userLocation.x, userLocation.y);
    else
        firstLocation.zone = LatLonToUTMXY(lat, lng, 0, firstLocation.x, firstLocation.y);
}
