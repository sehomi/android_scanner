//
// Created by a on 6/7/2021.
//

#include "scanner.h"
#include "sweeper.h"
//#include <opencv2/imgproc.hpp>

Scanner::Scanner(std::string assetsDir, std::string logsDir, DetectionMethod dm, int log_mode, float hva_, int maxdist)
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

//    beta = 60.0;                // Assumption: the pitch down angle is fixed - May be changed in a set-function

    if (log_mode == 0)
        logger = new Logger(logsDir, true, false, "/storage/emulated/0/LogFolder/log_2021_07_08_20_05_38/");
    else if (log_mode == 1)
        logger = new Logger(logsDir, false, true, "/storage/emulated/0/LogFolder/log_2021_07_08_20_05_38/");
    else
        logger = new Logger(logsDir, false, false, "/storage/emulated/0/LogFolder/log_2021_07_08_20_05_38/");

    sweeper = new SweeperGeometry::Sweeper();
    motionDetector = new MotionDetector();
//    __android_log_print(ANDROID_LOG_VERBOSE, "android_scanner", "---------1-2");
}

Scanner::Scanner(std::string assetsDir, std::string logsDir, DetectionMethod dm, float f_, float cx_, float cy_, float res_, int maxdist)
{
    f = f_;
    cx = cx_;
    cy = cy_;
    max_dist = maxdist;
    RAD = PI/180.0;

    if (dm == MN_SSD)
        detector = new Detector(assetsDir, DetectionMethod::MN_SSD, 0.4, 0.4);
    else if (dm == YOLO_V3)
        detector = new Detector(assetsDir, DetectionMethod::YOLO_V3, 0.4, 0.4);
    else if (dm == YOLO_TINY)
        detector = new Detector(assetsDir, DetectionMethod::YOLO_TINY, 0.4, 0.4);

//    beta = 60.0;                // Assumption: the pitch down angle is fixed - May be changed in a set-function

    // TODO: log_mode must be input
    bool log_mode = false;
    logger = new Logger(logsDir, log_mode, false, "");
}

//void Scanner::readFromLog(std::string logs_dir)
//{
//    int count = 0;
//    std::string txt_adrs = logs_dir + "log.txt";
//    std::ifstream file;
//    file.open(txt_adrs);
//    std::string line_text;
//
//    while (getline(file, line_text))
//    {
//        ImuSet imuSt;
//        Mat image;
//        count++;
//        std::string img_adrs = logs_dir + "image" + std::to_string(count) + ".jpg";
//        logger->readData(img_adrs, line_text, image, imuSt);
//    }
//    file.close();
//}

void Scanner::setCamInfo(Mat &img)
{
    int width = img.size().width;
    int height = img.size().height;
    f = (float) (0.5 * width * (1.0 / tan((hva/2.0)*PI/180)));
    cx = (float) width/2;
    cy = (float) height/2;
}

bool Scanner::scan(ImageSet &imgSt, Mat &detections_img, std::vector<Location> &object_poses, Mat &movings_img)
{
    if (!logger->readFromLog)
        return false;

    std::vector<cv::Rect> bboxes;

    // TODO: hva must be set automatically from log
    //       This is for mavic mini:
    hva = 66.0;
    if (!camInfoSet)
    {
        setCamInfo(imgSt.image);
        camInfoSet = true;
    }

    motionDetector->detect(imgSt.image, movings_img);

    detector->detect(imgSt.image, bboxes);
    detector->drawDetections(detections_img, bboxes);

    camToMap(bboxes, imgSt, object_poses);

//    for (auto & op : object_poses)
//    {
//        __android_log_print(ANDROID_LOG_VERBOSE, "android_scanner----32", "lat %s", std::to_string(op.lat).c_str());
//        __android_log_print(ANDROID_LOG_VERBOSE, "android_scanner----32", "lng %s", std::to_string(op.lng).c_str());
//        __android_log_print(ANDROID_LOG_VERBOSE, "android_scanner----32", "alt %s", std::to_string(op.alt).c_str());
//        __android_log_print(ANDROID_LOG_VERBOSE, "android_scanner----32", "x %s", std::to_string(op.x).c_str());
//        __android_log_print(ANDROID_LOG_VERBOSE, "android_scanner----32", "y %s", std::to_string(op.y).c_str());
//    }

//    __android_log_print(ANDROID_LOG_VERBOSE, "android_scanner--length of objects:", "%s", std::to_string(object_poses.size()).c_str());
    associate(object_poses);
    return true;
}

bool Scanner::scan(std::vector<Location> &object_poses, Mat &detections, Mat &movings_img, int det_mode = 0, bool rgba = false)
{
    if (logger->readFromLog)
        return false;

    ImageSet imgSt;
//    ImuSet imuSt;
    std::vector<cv::Rect> bboxes;

    if (!logger->getImageSet(imgSt))
    {
        __android_log_print(ANDROID_LOG_VERBOSE, "android_scanner", "---------113");
        return false;
    }

    if (!camInfoSet)
    {
        setCamInfo(imgSt.image);
        camInfoSet = true;
    }

    detections = imgSt.image.clone();

    if (det_mode == 1) {
        motionDetector->detect(imgSt.image, movings_img);
    }
    else {
        if (rgba) {
            Mat img;
            cvtColor(imgSt.image, img, COLOR_RGBA2BGR);
            cvtColor(detections, detections, COLOR_RGBA2BGR);
            detector->detect(img, bboxes);
        } else
            detector->detect(imgSt.image, bboxes);

        detector->drawDetections(detections, bboxes);

        camToMap(bboxes, imgSt, object_poses);

        movings_img = cv::Mat::zeros(imgSt.image.rows, imgSt.image.cols, CV_8UC3);
    }

//    associate(object_poses);

//    ********* Not necessary - Required for Rviz:
//    if self._former_markers is not None:
//    for i in range(len(self._former_markers.markers)):
//    self._former_markers.markers[i].action = Marker.DELETE
//    self.markers_pub.publish(self._former_markers)

//    ********* Drawings - Must be handled in java
//    for i, per in enumerate(pers):
//        cv.rectangle(img, per, (0,0,255), 2)
//        text = "{}".format("person")
//        cv.putText(img, text, (per[0], per[1] - 5), cv.FONT_HERSHEY_SIMPLEX,
//            		0.5, (0,0,255), 2)

//    ********* Markers object quantification - Must be handled alternatively
//    for per in self._per_pos:
//    markers_msg.markers.append(self.marker_for_object(per, "person", len(markers_msg.markers)+1))
//    info_msg.markers.append(self.marker_for_object(per, "person_info", len(info_msg.markers)+1))

//    ********* Rest of python code
//    img_msg = self.bridge.cv2_to_imgmsg(img)
//    self.image_pub.publish(img_msg)
//    self._is_scanning = False

    return true;
}

//Scanner::Scanner(std::string assetsDir) {
//    detector = new Detector(assetsDir, DetectionMethod::MN_SSD, 0.4, 0.4);
//    return;
//}

void Scanner::myFlip(Mat src) {
    flip(src, src, 0);
}

void Scanner::myBlur(Mat src, float sigma) {
    GaussianBlur(src, src, Size(), sigma);
}

void Scanner::camToMap(std::vector<Rect> &objects, const ImageSet& is, std::vector<Location> &object_poses)
{
    std::vector<Point2f> centers;
    std::vector<bool> show_permissions;

    centers.reserve(objects.size());
    for (auto & object : objects)
    {
        centers.push_back(Point(object.x + float(object.width)/2, object.y + float (object.height)/2));
    }

    imageToMap(is.roll, is.pitch, is.azimuth, is.lat, is.lng, is.alt, centers, object_poses, show_permissions);
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
//    int zone;
//    bool northp;
//    GeographicLib::UTMUPS::Forward(lat, lng, zone, northp, x, y);

    zone = LatLonToUTMXY(lat, lng, 0, x, y);
    isSouth = (lat < 0);
}

//Eigen::Quaternion<double> Scanner::eulerToQuat(double roll, double pitch, double azimuth)
//{
//    return Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
//           * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
//           * Eigen::AngleAxisd(azimuth, Eigen::Vector3d::UnitZ());
//}

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

//bool Scanner::scaleVector2(Eigen::VectorXd v, Eigen::VectorXd &output, double z)
//{
//    if (v[2] > 0)
//    {
//        output = (fabs(z)/fabs(v[2]))*v;
//        return true;
//    }
//    else
//    {
//        return false;
//    }
//}

void Scanner::toDirectionVector(std::vector<Rect> &objects, std::vector<Eigen::VectorXd> &output)
{
    std::vector<float> dirVec;
    std::vector<Point2d> center;
    output.clear();

//    for(int k=0; k<objects.size(); k++){
//        Eigen::VectorXf w(3);
//        w<<-((objects[k].x + (float(objects[k].width)/2))-cx),
//                -((objects[k].y + (float (objects[k].height)/2))-cy),
//                f;
//        output.push_back(w*(1/w.norm())); }

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
//    __android_log_print(ANDROID_LOG_VERBOSE, "android_scanner----2", "cx: %s", std::to_string(cx).c_str());
//    __android_log_print(ANDROID_LOG_VERBOSE, "android_scanner----2", "cy: %s", std::to_string(cy).c_str());
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

// TODO: fov calculation is not necessary when on the ground or in horizontal fov case
bool Scanner::calcFov(std::vector<Location> &poses_gps, std::vector<Location> &sweeped_area)
{
//    __android_log_print(ANDROID_LOG_VERBOSE, "android_scanner", "---------1-1");
    if(logger->readFromLog)
        return false;

//    std::vector<std::vector<Eigen::VectorXd>> arrows;
    ImuSet imuSt;
    if (!logger->getImuSet(imuSt)) {
        return false;
    }

    std::vector<bool> sps;
    int w, h;
    w = logger->img.image.size().width;
    h = logger->img.image.size().height;

    std::vector<Point2f> points{{0, 0}, { 0, (float)h }, {(float)w, (float)h}, {(float)w, 0}};

    imageToMap(imuSt.roll, imuSt.pitch, imuSt.azimuth, imuSt.lat, imuSt.lng, imuSt.alt, points, poses_gps, sps);

    sweeper->update(poses_gps, sweeped_area);

    __android_log_print(ANDROID_LOG_VERBOSE, "android_scanner--length of fov_locs:", "%s", std::to_string(poses_gps.size()).c_str());
    return true;
}

bool Scanner::calcFov(std::vector<Location> &poses_gps, std::vector<Location> &sweeped_area, ImuSet &imuSt, ImageSet &imgSt)
{
//    __android_log_print(ANDROID_LOG_VERBOSE, "android_scanner", "---------1-1");
    if (!logger->readFromLog)
        return false;

//    std::vector<std::vector<Eigen::VectorXd>> arrows;
    int w = imgSt.image.size().width;
    int h = imgSt.image.size().height;
    std::vector<bool> sps;
    std::vector<Point2f> points{{0, 0}, { 0, (float)h }, {(float)w, (float)h}, {(float)w, 0}};

    imageToMap(imgSt.roll, imgSt.pitch, imgSt.azimuth, imgSt.lat, imgSt.lng, imgSt.alt, points, poses_gps, sps);

    sweeper->update(poses_gps, sweeped_area);

//    __android_log_print(ANDROID_LOG_VERBOSE, "android_scanner--length of fov_locs:", "%s", std::to_string(poses_gps.size()).c_str());
    return true;
}

void Scanner::imageToMap(double roll, double pitch, double azimuth, double lat, double lng, double alt, std::vector<Point2f> points, std::vector<Location> &poses_gps, std::vector<bool> &show_permissions)
{
    double x, y;
    Eigen::VectorXd pos(3);
    Eigen::Matrix3d camToInertia;
    std::vector<Eigen::VectorXd> poses;//, ws;

    show_permissions.clear();

    gpsToUtm(lat, lng, x, y);
    pos << y, x, -alt;

    eulerToRotationMat(roll, pitch, azimuth, camToInertia);

    for (auto & point : points)
    {
        Eigen::VectorXd w_cam(3), w_(3), v(3), p(3), scaled_v(3);
        calcDirVec(point.x, point.y, w_);
        w_cam << w_[2], w_[0], w_[1];
        v = camToInertia * w_cam;
//        v = camToInertia * imToCam * w_;

        // TODO: This method must be applied ... not the above mess
//        v = camToInertia.transpose() * imToCam.transpose() * w_;

//        std::stringstream ss;
//        ss << v;
//        __android_log_print(ANDROID_LOG_VERBOSE, "android_scanner----2", "v: %s", ss.str().c_str());

        show_permissions.push_back(scaleVector(v, scaled_v, pos[2]));

//        p << v[0] + pos[0], v[1] + pos[1], v[2] + pos[2];
        p << (scaled_v[1] + pos[1]), scaled_v[0] + pos[0], -(scaled_v[2] + pos[2]);
        poses.push_back(p);
//        vector<Eigen::VectorXd> arrow{pos, p};
//        arrows.push_back(arrow);
//      TODO : The scanned places must be saved - An overall fov must be generated simultaneously
    }
    utmToGps(poses, poses_gps);
}

void Scanner::utmToGps(std::vector<Eigen::VectorXd> utms, std::vector<Location> &latlons)
{
    latlons.clear();
    for (auto & utm : utms)
    {
        double lat=0, lon=0;
        UTMXYToLatLon (utm[0], utm[1], zone, isSouth, lat, lon);
        Location loc;
        loc.lat = lat*180/PI;
        loc.lng = lon*180/PI;
        loc.alt = utm[2];
        loc.x = utm[0];
        loc.y = utm[1];
        latlons.push_back(loc);
    }
}
