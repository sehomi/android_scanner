
#include "Logger.h"

Logger::Logger(std::string logs_dir, bool log_mode, bool rfl=false, std::string pre_dir="")
{
    logsDir = logs_dir;
    counter = 0;
    logMode = log_mode;
    readFromLog = rfl;
    prelogged_dir = pre_dir;
    if (rfl)
        iLogFile.open(prelogged_dir + "log.txt");
}

void Logger::enableLogMode()
{
    logMode = true;
}

void Logger::disableLogMode()
{
    logMode = false;
}

void Logger::setImage(Mat &image, double time)
{
    if (readFromLog)
        return;

    img.image = image;
    img.time = time;
//    __android_log_print(ANDROID_LOG_VERBOSE, "android_scanner/setImage", "%f  %f", img.time, time);

    if (logMode)
    {
        ImageSet imgSt;
//        getImageSet(imgSt);
        if (getImageSet(imgSt)) {
//        if (!image.empty()) {
            writeImageSet(imgSt);
//            __android_log_print(ANDROID_LOG_VERBOSE, "android_scanner...", "%s", address.c_str());
        }
    }
}

void Logger::writeImageSet(const ImageSet &imgSet)
{
    counter++;
    std::string imgName = "image" + std::to_string(counter) + ".jpg";
    std::string address = logsDir + imgName;
    __android_log_print(ANDROID_LOG_VERBOSE, "android_scanner...", "%s", address.c_str());
    imwrite(address, imgSet.image);
//    __android_log_print(ANDROID_LOG_VERBOSE, "android_scanner.../write", "%f", imgSet.time);
    logFile.open(logsDir + "log.txt", std::ios::out | std::ios::in | std::ios::app);
    logFile << imgName << ',' << std::to_string(imgSet.time) << ',' << std::to_string(imgSet.lat) << ',' << std::to_string(imgSet.lng) \
            << ',' << std::to_string(imgSet.alt) << ',' << std::to_string(imgSet.roll) << ',' << std::to_string(imgSet.pitch) \
            << ',' << std::to_string(imgSet.azimuth) << std::endl;
    logFile.close();
}

void Logger::readData( std::string imData, Mat &image, ImuSet &imuSt)
{

    std::istringstream iss(imData);
    std::string num;
    int k = 0;
    while (std::getline(iss, num, ',') && k<8)
    {
        switch (k++) {
            case 0: {
                std::string imDir = prelogged_dir + num;
//                __android_log_print(ANDROID_LOG_ERROR, "readData", "%s", imDir.c_str());
                image = imread(imDir);
//                __android_log_print(ANDROID_LOG_ERROR, "readData", "%s", std::to_string(image.empty()).c_str());
                break;
            }
            case 1:
            {
                imuSt.time = std::stod(num);
                break;
            }
            case 2:
            {
                imuSt.lat = std::stod(num);
                break;
            }
            case 3:
            {
                imuSt.lng = std::stod(num);
                break;
            }
            case 4:
            {
                imuSt.alt = std::stod(num);
                break;
            }
            case 5:
            {
                imuSt.roll = std::stod(num);
                break;
            }
            case 6:
            {
                imuSt.pitch = std::stod(num);
                break;
            }
            case 7:
            {
                imuSt.azimuth = std::stod(num);
//                __android_log_print(ANDROID_LOG_ERROR, "readData", "%s", std::to_string(imuSt.azimuth).c_str());
                break;
            }
            default: break;
        }

    }
//    __android_log_print(ANDROID_LOG_ERROR, "readData", "%s", "after inner while");
}

//void Logger::readFromFile(std::string address)
//{
//    std::string adrs = "/storage/emulated/0/LogFolder/log_2021_07_11_19_47_12/";
//    int count = 0;
//    std::string txt_adrs = adrs + "log.txt";
//    std::ifstream file;
//    file.open(txt_adrs);
//    std::string line_text;
//    ImageSet imgSt;
//    while (getline(file, line_text))
//    {
//        count++;
//        std::string img_adrs = adrs + "image" + std::to_string(count) + ".jpg";
//        imgSt.image = imread(img_adrs);
//        std::istringstream is(line_text);
//        std::string num;
//        int k = 1;
//        while (std::getline(is, num, ','))
//        {
//            switch (k++) {
//                case 1: imgSt.time = std::stod(num);
//                case 2: imgSt.lat = std::stod(num);
//                case 3: imgSt.lng = std::stod(num);
//                case 4: imgSt.alt = std::stod(num);
//                case 5: imgSt.roll = std::stod(num);
//                case 6: imgSt.pitch = std::stod(num);
//                case 7: imgSt.azimuth = std::stod(num);
//            }
//        }
//    }
//    file.close();
//}


void Logger::setLocation(double lat, double lng, double alt, double time)
{
    if (readFromLog)
        return;

    Location loc;

    loc.lat = lat;
    loc.lng = lng;
    loc.alt = alt;
    loc.time = time;

    bufferLocation(loc);

    if (locationBuffer.size() == 0)
    {
        refLoc.lat = lat;
        refLoc.lng = lng;
        refLoc.alt = alt;
        refLoc.time = time;
    }
}

bool Logger::setOrientation(double roll, double pitch, double azimuth, double time)
{
    if (readFromLog)
        return false;

    Orientation orn;

    orn.roll = roll*PI/180;
    orn.pitch = pitch*PI/180;
    orn.azimuth = azimuth*PI/180;
    orn.time = time;
//    __android_log_print(ANDROID_LOG_ERROR, "TRACKERS", "%s", "Str--------------------------------------------------------------");

    bufferOrientation(orn);

    return true;
}

void Logger::bufferLocation(Location loc)
{
    if (locationBuffer.size() < locBufLen)
    {
        locationBuffer.push_back(loc);
    }
    else
    {
        locationBuffer.erase(locationBuffer.begin());
        locationBuffer.push_back(loc);
    }
}

void Logger::bufferOrientation(Orientation orn)
{
    if (orientationBuffer.size() < ornBufLen)
    {
        orientationBuffer.push_back(orn);
    }
    else
    {
        orientationBuffer.erase(orientationBuffer.begin());
        orientationBuffer.push_back(orn);
    }
}

bool Logger::getImageSetFromLogger(ImageSet &imgSt, ImuSet &imuSt)
{
    __android_log_print(ANDROID_LOG_ERROR, "getImageSetFromLogger", "%s", "----21");
    if (!readFromLog)
        return false;
    __android_log_print(ANDROID_LOG_ERROR, "getImageSetFromLogger", "%s", "----22");
    Mat image;

    std::string line_text;
    bool retVal = (bool) getline(iLogFile, line_text);
    if (!retVal)
        return false;
    __android_log_print(ANDROID_LOG_ERROR, "getImageSetFromLogger", "%s", "----23");
    readData(line_text, image, imuSt);
    __android_log_print(ANDROID_LOG_ERROR, "getImageSetFromLogger", "%s", "----24");

    imgSt.image = image;
    imgSt.time = imuSt.time;
    imgSt.lat = imuSt.lat;
    imgSt.lng = imuSt.lng;
    imgSt.alt = imuSt.alt;
    imgSt.roll = imuSt.roll;
    imgSt.pitch = imuSt.pitch;
    imgSt.azimuth = imuSt.azimuth;

    return true;
}

bool Logger::getImageSet(ImageSet &imgSet)
{
//    if (readFromLog)
//    {
//
//    }
//    else
//    {
//
//    }

    Location location;
    Orientation orientation;
    double dist = 0, minDist = 1e7;

    if (locationBuffer.size() == 0 || orientationBuffer.size() == 0 || img.image.empty())
    {
        return false;
    }

//    __android_log_print(ANDROID_LOG_VERBOSE, "android_scanner/getImageSet", "%f", img.time);


    for(int k=0; k<locationBuffer.size(); k++)
    {
        dist = fabs(locationBuffer[k].time - img.time);
        if (dist < minDist)
        {
            location = locationBuffer[k];
            minDist = dist;
        }
    }

    dist = 0;
    minDist = 1e7;
    for(int k=0; k<orientationBuffer.size(); k++)
    {
        dist = fabs(orientationBuffer[k].time - img.time);
        if (dist < minDist)
        {
            orientation = orientationBuffer[k];
            minDist = dist;
        }
    }

    imgSet.image = img.image;
    imgSet.lat = location.lat;
    imgSet.lng = location.lng;
    imgSet.alt = location.alt;
    imgSet.roll = orientation.roll;
    imgSet.pitch = orientation.pitch;
    imgSet.azimuth = orientation.azimuth;
    imgSet.time = img.time;

    return true;
}

bool Logger::getImuSet(ImuSet &imuSet)
{
    Location location;
    float dist = 0, minDist = 1e7;

    if (orientationBuffer.size() == 0 || locationBuffer.size() == 0 || img.image.empty())
    {
        return false;
    }

    for(int k=0; k<locationBuffer.size(); k++)
    {
        dist = fabs(locationBuffer[k].time - orientationBuffer.back().time);
        if (dist < minDist)
        {
            location = locationBuffer[k];
            minDist = dist;
        }
    }


    imuSet.roll = orientationBuffer.back().roll;
    imuSet.pitch = orientationBuffer.back().pitch;
    imuSet.azimuth = orientationBuffer.back().azimuth;
    imuSet.lat = location.lat;
    imuSet.lng = location.lng;
    imuSet.alt = location.alt;
    imuSet.time = orientationBuffer.back().time;

    return true;
}