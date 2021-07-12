#include <jni.h>
#include <string>
#include "android/bitmap.h"
//#include <opencv2/core.hpp>
//#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "scanner.h"
#include "Logger.h"
#include <android/log.h>

// TODO: declare JNI function in a base class like CameraApplication
#include "Eigen/Core"
#include <Eigen/Geometry>

Scanner *sc;
//Logger *lg;

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

jobjectArray convertArray(JNIEnv* env, std::vector<Location> fov_poses)
{
    jclass cls = env->FindClass("[D");
    jdoubleArray iniVal = env->NewDoubleArray(3);
// Create the returnable jobjectArray with an initial value
    jobjectArray outer = env->NewObjectArray(fov_poses.size(),cls, iniVal);

    for (int i = 0; i < fov_poses.size(); i++)
    {
        jdoubleArray inner = env->NewDoubleArray(3);

        Location pos = fov_poses[i];
        double posa[3] = {pos.lat, pos.lng, pos.alt};
        env->SetDoubleArrayRegion(inner, 0, 3, posa);
//             set inner's values
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
Java_com_example_android_1scanner_MainActivity_createScanner(JNIEnv* env, jobject p_this, jstring assets, jstring logs, jboolean log_mode, jint method, jfloat hva) {

    __android_log_print(ANDROID_LOG_VERBOSE, "android_scanner", "---------111");

    jboolean isCopy;
    const char *convertedValue = (env)->GetStringUTFChars(assets, &isCopy);
    std::string assets_str = std::string(convertedValue);

    jboolean isCopyl;
    const char *convertedValuel = (env)->GetStringUTFChars(logs, &isCopyl);
    std::string logs_str = std::string(convertedValuel);

//    sc = new Scanner(assets_str, (DetectionMethod)method, 1.0, 1.0, 1.0, 1.0, 300);
//    __android_log_print(ANDROID_LOG_VERBOSE, "outerrrrrrrr", "oute");
    sc = new Scanner(assets_str, logs_str, (DetectionMethod)method, (bool) log_mode, hva, 300);
//    __android_log_print(ANDROID_LOG_VERBOSE, "outerrrrrrrr", "outer[0][0]");

//    lg = new Logger();
    return;
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_android_1scanner_MainActivity_flip(JNIEnv* env, jobject p_this, jobject bitmapIn, jobject bitmapOut) {
    Mat src;
    bitmapToMat(env, bitmapIn, src, false);
    // NOTE bitmapToMat returns Mat in RGBA format, if needed convert to BGRA using cvtColor

    sc->myFlip(src);

    // NOTE matToBitmap expects Mat in GRAY/RGB(A) format, if needed convert using cvtColor
    matToBitmap(env, src, bitmapOut, false);
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_android_1scanner_MainActivity_blur(JNIEnv* env, jobject p_this, jobject bitmapIn, jobject bitmapOut, jfloat sigma) {
    Mat src;
    bitmapToMat(env, bitmapIn, src, false);
    sc->myBlur(src, sigma);
    matToBitmap(env, src, bitmapOut, false);
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_android_1scanner_MainActivity_detect(JNIEnv* env, jobject p_this, jobject bitmapIn, jobject bitmapOut) {
    Mat src;
    bitmapToMat(env, bitmapIn, src, false);
    cvtColor(src, src, COLOR_RGBA2BGR);

    std::vector<cv::Rect> bboxes;
    Mat dst = src.clone();

    sc->detector->detect(src, bboxes);
    sc->detector->drawDetections(dst, bboxes);

    cvtColor(dst, dst, COLOR_BGR2RGB);
    matToBitmap(env, dst, bitmapOut, false);

//    return bboxes;
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_android_1scanner_MainActivity_scan(JNIEnv* env, jobject p_this) {
//    Mat src;
//    bitmapToMat(env, bitmapIn, src, false);
//    cvtColor(src, src, COLOR_RGBA2BGR);

    sc->scan();



//    std::vector<cv::Rect> bboxes;
//    Mat dst = src.clone();

//    sc->detector->detect(src, bboxes);
//    sc->scan(bboxes)

//    sc->detector->drawDetections(dst, bboxes);

//    cvtColor(dst, dst, COLOR_BGR2RGB);
//    matToBitmap(env, dst, bitmapOut, false);

//    return bboxes;
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
    std::vector<Location> fov_poses;

    if (!sc->logger->setOrientation(roll, pitch, azimuth, time))
        return NULL;

    bool success = sc->calcFov(fov_poses);
    if (success)
    {
        return convertArray(env, fov_poses);
    }
    else
        {
        return  NULL;
    }

}

extern "C" JNIEXPORT jobjectArray JNICALL
Java_com_example_android_1scanner_MainActivity_readLog(JNIEnv* env, jobject p_this, jobject bitmap, jobject processedBitmap)
{
//    sc->readFromLog(logs_str);
    ImageSet imgSt;
    ImuSet imuSt;
    std::vector<Location> fov_locs;
    jobjectArray fov_poses_array = NULL;

//    __android_log_print(ANDROID_LOG_VERBOSE, "outer", "1");
    sc->logger->getImageSetFromLogger(imgSt, imuSt);

//    __android_log_print(ANDROID_LOG_VERBOSE, "outer", "2");
//    sc->scan(imgSt);
    std::vector<Rect> bboxes;
    Mat dst = imgSt.image.clone();
    Mat src = imgSt.image.clone();
// TODO: Must be replaced by scan:
    sc->detector->detect(imgSt.image, bboxes);
    sc->detector->drawDetections(dst, bboxes);

    cvtColor(src, src, COLOR_BGR2RGB);
    matToBitmap(env, src, bitmap, false);

    cvtColor(dst, dst, COLOR_BGR2RGB);
    matToBitmap(env, dst, processedBitmap, false);

    if (sc->calcFov(fov_locs, imuSt, imgSt))
        fov_poses_array = convertArray(env, fov_locs);

    return fov_poses_array;

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

//    sc = new Scanner(assets_str, (DetectionMethod)method, 1.0, 1.0, 1.0, 1.0, 300);
    sc = new Scanner(assets_str, logs_str, (DetectionMethod)method, (bool) log_mode, hva, 300);

    return;
}

extern "C"
JNIEXPORT void JNICALL
Java_com_example_android_1scanner_AircraftActivity_detect(JNIEnv *env, jobject thiz,
                                                          jobject bitmapIn, jobject bitmapOut) {
    Mat src;
    bitmapToMat(env, bitmapIn, src, false);
    cvtColor(src, src, COLOR_RGBA2BGR);

    std::vector<cv::Rect> bboxes;
    Mat dst = src.clone();

    sc->detector->detect(src, bboxes);
    sc->detector->drawDetections(dst, bboxes);

    cvtColor(dst, dst, COLOR_BGR2RGB);
    matToBitmap(env, dst, bitmapOut, false);
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

extern "C" JNIEXPORT jobjectArray JNICALL
Java_com_example_android_1scanner_AircraftActivity_setOrientation(JNIEnv* env, jobject p_this, jdouble roll, jdouble pitch, jdouble azimuth, jdouble time)
{
    std::vector<Location> fov_poses;

    sc->logger->setOrientation(roll, pitch, azimuth, time);

    bool success = sc->calcFov(fov_poses);
    if (success)
    {
        return convertArray(env, fov_poses);
    }
    else
    {
        return  NULL;
    }
//    if (outer == NULL) {
//
//        __android_log_print(ANDROID_LOG_VERBOSE, "outerrrrrrrr", "outer[0][0]");
//        return NULL;
//    }

}
