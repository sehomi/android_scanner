#include <jni.h>
#include <string>
#include "android/bitmap.h"
#include <opencv2/opencv.hpp>
#include "scanner.h"
#include "Logger.h"
#include <android/log.h>

// TODO: declare JNI function in a base class like CameraApplication
#include "Eigen/Core"
#include <Eigen/Geometry>

Scanner *sc;

void bitmapToMat(JNIEnv *env, jobject bitmap, Mat& dst, jboolean needUnPremultiplyAlpha)
{
    AndroidBitmapInfo  info;
    void*              pixels = 0;

    try {
        CV_Assert( AndroidBitmap_getInfo(env, bitmap, &info) >= 0 );
        CV_Assert( info.format == ANDROID_BITMAP_FORMAT_RGBA_8888 ||
                   info.format == ANDROID_BITMAP_FORMAT_RGB_565 );
        CV_Assert( AndroidBitmap_lockPixels(env, bitmap, &pixels) >= 0 );
        CV_Assert( pixels );
        dst.create(info.height, info.width, CV_8UC4);
        if( info.format == ANDROID_BITMAP_FORMAT_RGBA_8888 )
        {
            Mat tmp(info.height, info.width, CV_8UC4, pixels);
            if(needUnPremultiplyAlpha) {
                cvtColor(tmp, dst, COLOR_mRGBA2RGBA);
            }
            else{
                tmp.copyTo(dst);
            }
        } else {
            // info.format == ANDROID_BITMAP_FORMAT_RGB_565
            Mat tmp(info.height, info.width, CV_8UC2, pixels);
            cvtColor(tmp, dst, COLOR_BGR5652RGBA);
        }
        AndroidBitmap_unlockPixels(env, bitmap);
        return;
    } catch(const cv::Exception& e) {
        AndroidBitmap_unlockPixels(env, bitmap);
        jclass je = env->FindClass("java/lang/Exception");
        env->ThrowNew(je, e.what());
        return;
    } catch (...) {
        AndroidBitmap_unlockPixels(env, bitmap);
        jclass je = env->FindClass("java/lang/Exception");
        env->ThrowNew(je, "Unknown exception in JNI code {nBitmapToMat}");
        return;
    }
}

void matToBitmap(JNIEnv* env, Mat src, jobject bitmap, jboolean needPremultiplyAlpha)
{
    AndroidBitmapInfo  info;
    void*              pixels = 0;


    try {
        CV_Assert( AndroidBitmap_getInfo(env, bitmap, &info) >= 0 );
        CV_Assert( info.format == ANDROID_BITMAP_FORMAT_RGBA_8888 ||
                   info.format == ANDROID_BITMAP_FORMAT_RGB_565 );
        // TODO: clean this resize mess
        resize(src, src, Size(info.width, info.height));
        CV_Assert( src.dims == 2 && info.height == (uint32_t)src.rows && info.width == (uint32_t)src.cols );
        CV_Assert( src.type() == CV_8UC1 || src.type() == CV_8UC3 || src.type() == CV_8UC4 );
        CV_Assert( AndroidBitmap_lockPixels(env, bitmap, &pixels) >= 0 );
        CV_Assert( pixels );
        if( info.format == ANDROID_BITMAP_FORMAT_RGBA_8888 )
        {
            Mat tmp(info.height, info.width, CV_8UC4, pixels);
            if(src.type() == CV_8UC1)
            {
                cvtColor(src, tmp, COLOR_GRAY2RGBA);
            } else if(src.type() == CV_8UC3){
                cvtColor(src, tmp, COLOR_RGB2RGBA);
            } else if(src.type() == CV_8UC4){
                if(needPremultiplyAlpha) cvtColor(src, tmp, COLOR_RGBA2mRGBA);
                else src.copyTo(tmp);
            }
        } else {
            // info.format == ANDROID_BITMAP_FORMAT_RGB_565
            Mat tmp(info.height, info.width, CV_8UC2, pixels);
            if(src.type() == CV_8UC1)
            {
                cvtColor(src, tmp, COLOR_GRAY2BGR565);
            } else if(src.type() == CV_8UC3){
                cvtColor(src, tmp, COLOR_RGB2BGR565);
            } else if(src.type() == CV_8UC4){
                cvtColor(src, tmp, COLOR_RGBA2BGR565);
            }
        }
        AndroidBitmap_unlockPixels(env, bitmap);
        return;
    } catch(const cv::Exception& e) {
        AndroidBitmap_unlockPixels(env, bitmap);
        jclass je = env->FindClass("java/lang/Exception");
        env->ThrowNew(je, e.what());
        return;
    } catch (...) {
        AndroidBitmap_unlockPixels(env, bitmap);
        jclass je = env->FindClass("java/lang/Exception");
        env->ThrowNew(je, "Unknown exception in JNI code {nMatToBitmap}");
        return;
    }
}

jobjectArray putIntoArray(JNIEnv* env, std::vector<Object> objects)
{
    const int colsNum = 5;
    jclass cls = env->FindClass("[D");
    jdoubleArray iniVal = env->NewDoubleArray(colsNum);
    jobjectArray outer = env->NewObjectArray(objects.size(), cls, iniVal);

    for (int i = 0; i < objects.size(); i++)
    {
        jdoubleArray inner = env->NewDoubleArray(colsNum);

        Object det = objects.at(i);
        int type = det.type;
        double posa[colsNum] = {det.location.lat, det.location.lng, det.location.alt, (double) type, det.distance};

        env->SetDoubleArrayRegion(inner, 0, colsNum, posa);
        env->SetObjectArrayElement(outer, i, inner);
        env->DeleteLocalRef(inner);
    }
    return outer;
}

extern "C" JNIEXPORT jstring JNICALL
Java_com_example_android_1scanner_MainActivity_stringFromJNI(
        JNIEnv* env,
        jobject /* this */) {
    std::string hello = "Hello from C++";
    return env->NewStringUTF(hello.c_str());
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_android_1scanner_MainActivity_createScanner(JNIEnv* env, jobject p_this, jstring assets, jstring logs, jboolean log_mode, jint method, jfloat hva)
{
    jboolean isCopy;
    const char *convertedValue = (env)->GetStringUTFChars(assets, &isCopy);
    std::string assets_str = std::string(convertedValue);

    jboolean isCopyl;
    const char *convertedValuel = (env)->GetStringUTFChars(logs, &isCopyl);
    std::string logs_str = std::string(convertedValuel);

    sc = new Scanner(assets_str, logs_str, (DetectionMethod)method, (bool) log_mode, hva, 300);

    return;
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_android_1scanner_MainActivity_scan(JNIEnv* env, jobject p_this, jobject detections) {

    Mat det, movings;
    std::vector<Object> objects;
    bitmapToMat(env, detections, det, false);
    cvtColor(det, det, COLOR_RGBA2BGR);

    if (!sc->scan(objects, det, movings, 0, true))
        {
        putText(det, "SENSOR DATA NOT PROVIDED", cv::Point(50,200),cv::FONT_HERSHEY_DUPLEX,4,cv::Scalar(0,0,255),3,false);
    }

    cvtColor(det, det, COLOR_BGR2RGB);
    matToBitmap(env, det, detections, false);
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_android_1scanner_MainActivity_setImage(JNIEnv* env, jobject p_this, jobject bitmap, jdouble time)
{
    Mat img;
    bitmapToMat(env, bitmap, img, false);

    sc->logger->setImage(img, time);
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_android_1scanner_MainActivity_setLocation(JNIEnv* env, jobject p_this, jdouble lat, jdouble lng, jdouble alt, jdouble time)
{
    sc->logger->setLocation(lat, lng, alt, time);
}

extern "C" JNIEXPORT jobjectArray JNICALL
Java_com_example_android_1scanner_MainActivity_setOrientation(JNIEnv* env, jobject p_this, jdouble roll, jdouble pitch, jdouble azimuth, jdouble time)
{
    std::vector<Object> fov_objects;

    if (sc->logger->setOrientation(roll, pitch, azimuth, time) && sc->calcFov(fov_objects))
    {
        return putIntoArray(env, fov_objects);
    }
    else
    {
        return  NULL;
    }
}

extern "C" JNIEXPORT jobjectArray JNICALL
Java_com_example_android_1scanner_MainActivity_readLog(JNIEnv* env, jobject p_this, jobject bitmap, jobject processedBitmap, jobject movingsBitmap)
{
    ImageSet imgSt;
    ImuSet imuSt;
    jobjectArray fov_poses_array = NULL;
    std::vector<Object> objects, fov_objects;

    // TODO: get readlog mode from connection activity
    if (!sc->logger->getImageSetFromLogger(imgSt, imuSt))
        return NULL;

    Mat movings;
    Mat dst = imgSt.image.clone();

    sc->scan(imgSt, dst, movings, objects);

    Mat src = imgSt.image.clone();
    cvtColor(src, src, COLOR_BGR2RGB);
    matToBitmap(env, src, bitmap, false);

    cvtColor(dst, dst, COLOR_BGR2RGB);
    matToBitmap(env, dst, processedBitmap, false);

    cvtColor(movings, movings, COLOR_BGR2RGB);
    matToBitmap(env, movings, movingsBitmap, false);

    //// Note: calcFov sets fov for an image after scan while scan (motionDetector) uses it for current
    //// image. But it's not important because of the assumption of fixed camera in motion detection mode
    if (sc->calcFov(fov_objects, imgSt))
    {
        fov_objects.insert(fov_objects.end(), objects.begin(), objects.end());
        return putIntoArray(env, fov_objects);
    }
    else
        return NULL;
}

extern "C"
JNIEXPORT void JNICALL
Java_com_example_android_1scanner_AircraftActivity_createScanner(JNIEnv *env, jobject thiz, jstring assets, jstring logs, jboolean log_mode, jfloat hva, jint method) {
    jboolean isCopy;
    const char *convertedValue = (env)->GetStringUTFChars(assets, &isCopy);
    std::string assets_str = std::string(convertedValue);

    jboolean isCopyl;
    const char *convertedValuel = (env)->GetStringUTFChars(logs, &isCopyl);
    std::string logs_str = std::string(convertedValuel);

    sc = new Scanner(assets_str, logs_str, (DetectionMethod)method, (bool) log_mode, hva, 300);

    return;
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_android_1scanner_AircraftActivity_setImage(JNIEnv* env, jobject p_this, jobject bitmap, jdouble time)
{
    Mat img;
    bitmapToMat(env, bitmap, img, false);
    cvtColor(img, img, COLOR_RGB2BGR);

    sc->logger->setImage(img, time);
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_android_1scanner_AircraftActivity_setLocation(JNIEnv* env, jobject p_this, jdouble lat, jdouble lng, jdouble alt, jdouble time)
{
    sc->logger->setLocation(lat, lng, alt, time);
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_android_1scanner_AircraftActivity_setUserLocation(JNIEnv* env, jobject p_this, jdouble lat, jdouble lng)
{
    sc->setReferenceLoc(lat, lng, true);
}

extern "C" JNIEXPORT jobjectArray JNICALL
Java_com_example_android_1scanner_AircraftActivity_setOrientation(JNIEnv* env, jobject p_this, jdouble roll, jdouble pitch, jdouble azimuth, jdouble time)
{
    std::vector<Location> fov_poses, sweeped_area;
    std::vector<Object> fov_objects;
    jobjectArray fov_poses_array = NULL;

    //// fov_objects includes both fov points and swept area
    if (sc->logger->setOrientation(roll, pitch, azimuth, time) && sc->calcFov(fov_objects))
        {
        fov_poses_array = putIntoArray(env, fov_objects);
        return fov_poses_array;
    }
    else
        return  NULL;
}

extern "C" JNIEXPORT jobjectArray JNICALL
Java_com_example_android_1scanner_AircraftActivity_scan(JNIEnv* env, jobject p_this, jobject detections, jobject movings_img, jint detMode)
{
    std::vector<Object> objects;

    Mat det, movings;
    bitmapToMat(env, detections, det, false);
    cvtColor(det, det, COLOR_RGBA2BGR);

    bitmapToMat(env, movings_img, movings, false);
    cvtColor(movings, movings, COLOR_RGBA2BGR);

    if (!sc->scan(objects, det, movings, (int) detMode, false))
    {
        putText(det, "SENSOR DATA NOT PROVIDED", cv::Point(50,200),cv::FONT_HERSHEY_DUPLEX,4,cv::Scalar(0,0,255),3,false);
        putText(movings, "SENSOR DATA NOT PROVIDED", cv::Point(50,200),cv::FONT_HERSHEY_DUPLEX,4,cv::Scalar(0,0,255),3,false);
        return NULL;
    }

    cvtColor(det, det, COLOR_BGR2RGB);
    matToBitmap(env, det, detections, false);

    cvtColor(movings, movings, COLOR_BGR2RGB);
    matToBitmap(env, movings, movings_img, false);

    return putIntoArray(env, objects);
}
