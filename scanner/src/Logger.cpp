
#include "Logger.h"

Logger::Logger(std::string logs_dir)
{
    logsDir = logs_dir;
    counter = 0;
}

void Logger::enableLogMode()
{
    logMode = true;
}

void Logger::disableLogMode()
{
    logMode = false;
}

void Logger::setImage(Mat &image, float time)
{
    img.image = image;
    img.time = time;

    if (logMode)
    {
        ImageSet imgSet;
//        getImageSet(imgSet);
        if (getImageSet(imgSet)) {
//        if (!image.empty()) {
            writeImageSet(imgSet);
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
    logFile.open(logsDir + "log.txt", std::ios::out | std::ios::in | std::ios::app);
    logFile << imgName << ',' << std::to_string(imgSet.time) << ',' << std::to_string(imgSet.lat) << ',' << std::to_string(imgSet.lng) \
            << ',' << std::to_string(imgSet.alt) << ',' << std::to_string(imgSet.roll) << ',' << std::to_string(imgSet.pitch) \
            << ',' << std::to_string(imgSet.azimuth) << std::endl;
    logFile.close();
}

//void Logger::writeImageSet(ImageSet imgSet, std::string imgName)
//{
//    logFile.open(logsDir + "log.txt", std::ios::out | std::ios::in | std::ios::app);
//    logFile << imgName << ',' << std::to_string(imgSet.time) << ',' << std::to_string(imgSet.lat) << ',' << std::to_string(imgSet.lng) \
//            << ',' << std::to_string(imgSet.alt) << ',' << std::to_string(imgSet.roll) << ',' << std::to_string(imgSet.pitch) \
//            << ',' << std::to_string(imgSet.azimuth) << std::endl;
//    logFile.close();
//}

void Logger::setLocation(double lat, double lng, double alt, float time)
{
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

void Logger::setOrientation(double roll, double pitch, double azimuth, float time)
{
    Orientation orn;

    orn.roll = roll*PI/180;
    orn.pitch = pitch*PI/180;
    orn.azimuth = azimuth*PI/180;
    orn.time = time;
//    __android_log_print(ANDROID_LOG_ERROR, "TRACKERS", "%s", "Str--------------------------------------------------------------");

    bufferOrientation(orn);
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

bool Logger::getImageSet(ImageSet &imgSet)
{
    Location location;
    Orientation orientation;
    float dist = 0, minDist = 1e7;

    if (locationBuffer.size() == 0 || orientationBuffer.size() == 0 || img.image.empty())
    {
        return false;
    }

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

    if (orientationBuffer.size() == 0 || locationBuffer.size() == 0)
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