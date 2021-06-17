
#include "Logger.h"

void Logger::setImage(Mat image, float time)
{
    img.image = image;
    img.time = time;

    setImageSet(img);
}

void Logger::setLocation(float lat, float lng, float time)
{
    loc.lat = lat;
    loc.lng = lng;
    loc.time = time;

    bufferLocation(loc);
}

void Logger::setOrientation(float roll, float pitch, float azimuth, float time)
{
    orn.roll = roll;
    orn.pitch = pitch;
    orn.azimuth = azimuth;
    orn.time = time;

    bufferOrientation(orn);
}

void Logger::bufferLocation(location loc)
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

void Logger::bufferOrientation(orientation orn)
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

void Logger::setImageSet(image image)
{
    location location;
    orientation orientation;
    float dist = 0, minDist = 1e7;

    for(int k=0; k<locBufLen; k++)
    {
        dist = fabs(locationBuffer[k].time - image.time);
        if (dist < minDist)
        {
            location = locationBuffer[k];
            minDist = dist;
        }
    }

    dist = 0;
    minDist = 1e7;
    for(int k=0; k<ornBufLen; k++)
    {
        dist = fabs(orientationBuffer[k].time - image.time);
        if (dist < minDist)
        {
            orientation = orientationBuffer[k];
            minDist = dist;
        }
    }

    imgSet.image = image.image;
    imgSet.lat = location.lat;
    imgSet.lng = location.lng;
    imgSet.roll = orientation.roll;
    imgSet.pitch = orientation.pitch;
    imgSet.azimuth = orientation.azimuth;
    imgSet.time = image.time;
}
