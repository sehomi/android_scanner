//
// Created by a on 6/7/2021.
//

#include "scanner.h"
//#include <opencv2/imgproc.hpp>

Scanner::Scanner(std::string assetsDir, std::string logsDir, DetectionMethod dm, bool log_mode, float hva_, int maxdist)
{

    hva = hva_;
    max_dist = maxdist;
//    RAD = PI/180.0;

    if (dm == MN_SSD)
        detector = new Detector(assetsDir, DetectionMethod::MN_SSD, 0.4, 0.4);
    else if (dm == YOLO_V3)
        detector = new Detector(assetsDir, DetectionMethod::YOLO_V3, 0.4, 0.4);
    else if (dm == YOLO_TINY)
        detector = new Detector(assetsDir, DetectionMethod::YOLO_TINY, 0.4, 0.4);

    beta = 60.0;                // Assumption: the pitch down angle is fixed - May be changed in a set-function

    logger = new Logger(logsDir, log_mode);

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

    beta = 60.0;                // Assumption: the pitch down angle is fixed - May be changed in a set-function

    // TODO: log_mode must be input
    bool log_mode = false;
    logger = new Logger(logsDir, log_mode);
}

void Scanner::setCamInfo(Mat img)
{
    int width = img.size().width;
    int height = img.size().height;
    f = (0.5 * width * (1.0 / tan((hva/2)*PI/180)));
    cx = width/2;
    cy = height/2;
}

void Scanner::scan()
{
    ImageSet imgSt;
    std::vector<cv::Rect> bboxes;
    std::vector<Eigen::VectorXd> object_pos;

    bool check = logger->getImageSet(imgSt);

    if (!camInfoSet)
    {
        setCamInfo(imgSt.image);
        camInfoSet = true;
    }

    detector->detect(imgSt.image, bboxes);

    camToMap(bboxes, imgSt, object_pos);

    associate(object_pos);

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

void Scanner::camToMap(std::vector<Rect> &objects, const ImageSet& is, std::vector<Eigen::VectorXd> &object_pos)
{
    std::vector<Eigen::VectorXd> objects_ws, objects_vs, scaled_objects_vs;
    Eigen::Matrix3d camToInertia;

    camToInertiaMat(90+beta, 0, 90, is.roll, is.pitch, is.azimuth, camToInertia);

    toDirectionVector(objects, objects_ws);

    for(auto & object : objects_ws)
    {
//        Eigen::VectorXd r = dcm_cam_to_body * object;
//        Eigen::VectorXd u =  dcm_body_to_inertia * r;
        objects_vs.push_back(camToInertia * object);
    }

    double x, y;
    gpsToUtm(is.lat, is.lng, x, y);

    for(int kk=0; kk<objects_vs.size(); kk++)
    {
        Eigen::VectorXd vec(3), pos(3);
        bool isTrue;
        vec<<0, 0, 0;
        isTrue = scaleVector2(objects_vs[kk], vec, -x);
        scaled_objects_vs.push_back(vec);
        if (isTrue)
        {
            pos << scaled_objects_vs[kk][0]+y, scaled_objects_vs[kk][1]-x, scaled_objects_vs[kk][2]+is.alt;
        }
        else
        {
            pos << 0, 0, 0;
        }
        object_pos.push_back(pos);
    }
}

void Scanner::camToInertiaMat(double phi, double theta, double psi, double roll, double pitch, double azimuth, Eigen::Matrix3d &output)
{
    Eigen::AngleAxisd rollAngle(phi, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(theta, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(psi, Eigen::Vector3d::UnitZ());

    Eigen::Quaternion<double> q1 = rollAngle *pitchAngle* yawAngle;

    Eigen::Matrix3d dcm_cam_to_body = q1.matrix();

    Eigen::Quaternion<double> q2 = eulerToQuat(roll, pitch, azimuth);
    Eigen::Matrix3d dcm_body_to_inertia = q2.matrix();

    output = dcm_body_to_inertia * dcm_cam_to_body;
}

void Scanner::gpsToUtm(double lat, double lng, double &x, double &y)
{
//    int zone;
//    bool northp;
//    GeographicLib::UTMUPS::Forward(lat, lng, zone, northp, x, y);

    zone = LatLonToUTMXY(lat, lng, 0, x, y);
    isSouth = (lat < 0);

}

Eigen::Quaternion<double> Scanner::eulerToQuat(double roll, double pitch, double azimuth)
{
    return Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
           * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
           * Eigen::AngleAxisd(azimuth, Eigen::Vector3d::UnitZ());
}

void Scanner::scaleVector1(Eigen::VectorXd v, Eigen::VectorXd &output, double z)
{
    if (v[2] < 0)
    {
        double factor = fabs(z) / fabs(v[2]);
        if ((factor*v).norm() < max_dist)
        {
            output = factor*v;
        }
    }
    else
    {
        output = max_dist * v;
    }
}

bool Scanner::scaleVector2(Eigen::VectorXd v, Eigen::VectorXd &output, double z)
{
    if (v[2] < 0)
    {
        output = (fabs(z)/fabs(v[2]))*v;
        return true;
    }
    else
    {
        return false;
    }
}

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
    w<<-(x-cx),
       -(y-cy),
       f;
    z = w*(1/w.norm());
}

void Scanner::associate(std::vector<Eigen::VectorXd> &object_pos)
{
    for(auto & object : object_pos)
    {
        bool found = false;
        for (int k = 0; k < object_poses.size(); k++)
        {
            double x, y, new_x, new_y, dist;
            x = object_poses[k][0];
            y = object_poses[k][1];
            new_x = object[0];
            new_y = object[1];

            dist = sqrt(((x-new_x)*(x-new_x))+((y-new_y)*(y-new_y)));
            if (dist < 10)
            {
                object_poses[k] = object;
                markers[k].action = Marker::REMAIN;
//                markers[k].type = Marker::CAR or Marker::PERSON or ...
                markers[k].pos = object;
                found = true;
            }
        }
        if (!found)
        {
            object_poses.push_back(object);
            Marker *mk;
            mk->action = Marker::DRAW;
//            mk.type = Marker::CAR or Marker::PERSON or ...
            mk->pos = object;
            markers.push_back(*mk);
        }
    }
}

bool Scanner::calcFov(std::vector<Location> &poses_gps)
{
    __android_log_print(ANDROID_LOG_VERBOSE, "android_scanner", "---------1-1");
    int w, h;
    double x, y;
    Eigen::Quaternion<double> q;
    Eigen::VectorXd pos(3);
    Eigen::Matrix3d camToInertia;
    std::vector<Eigen::VectorXd> vs, scaled_vs, poses;//, ws;

//    std::vector<std::vector<Eigen::VectorXd>> arrows;
    ImuSet imuSt;
    if (!logger->getImuSet(imuSt))
    {
        __android_log_print(ANDROID_LOG_VERBOSE, "android_scanner", "---------logger->imuset");

        return false;
    }
    __android_log_print(ANDROID_LOG_VERBOSE, "android_scanner", "---------1-2");
    // TODO: move this check to getImuSet
    if (logger->img.image.empty())
    {
        __android_log_print(ANDROID_LOG_VERBOSE, "android_scanner", "---------logger->imgempty");
        return false;
    }

    w = logger->img.image.size().width;
    h = logger->img.image.size().height;
    q = eulerToQuat(imuSt.roll, imuSt.pitch, imuSt.azimuth);
    gpsToUtm(imuSt.lat, imuSt.lng, x, y);
    pos << y, -x, imuSt.alt;
    camToInertiaMat(90+beta, 0, 90, imuSt.roll, imuSt.pitch, imuSt.azimuth, camToInertia);
    std::vector<Point> points{{0, 0}, { 0, h }, {w, h}, {w, 0}};
    for (auto & point : points)
    {
        Eigen::VectorXd w_(3), v(3), p(3), scaled_v(3);
        calcDirVec(point.x, point.y, w_);
        v = camToInertia * w_;
        vs.push_back(v);
        scaleVector1(v, scaled_v, pos[2]);
        scaled_vs.push_back(scaled_v);
        p << v[0] + pos[0], v[1] + pos[1], v[2] + pos[2];
        poses.push_back(p);
//        vector<Eigen::VectorXd> arrow{pos, p};
//        arrows.push_back(arrow);
//      TODO : The scanned places must be saved - An overall fov must be generated simultaneously
    }
    utmToGps(poses, poses_gps);
    return true;
}

void Scanner::utmToGps(std::vector<Eigen::VectorXd> utms, std::vector<Location> &latlons)
{
    latlons.clear();
    for (auto & utm : utms)
    {
        double lat, lon;
        UTMXYToLatLon (utm[0], utm[1], zone, isSouth, lat, lon);
        Location loc;
        loc.lat = lat;
        loc.lng = lon;
        loc.alt = utm[2];
        latlons.push_back(loc);
    }
}
