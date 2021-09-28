
#include "Logger.h"

/** \brief Constructor; passes the log directoriy and initializes some class parameters
*
* \param [in]   logs_dir    String; The directory to write data, if desired
* \param [in]   logs_mode   Boolean; If true, the received images and their synchronized sensor data
*                           are written to directory specified by logs_dir
* \param [in]   rfl 		Boolean; If true, no synchronization is done; Only the pre-logged data
*                           within the specified (pre_dir) directory is read
* \param [in]   pre_dir     String; The directory in which the pre-logged data exists
*/
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

/** \brief Turns the write-to-log mode on
*
* Whenever this function is called, the mode in which the synchronized ImageSet instances are written
* to log directory is activated
*
* \sa disableLogMode()
*/
void Logger::enableLogMode()
{
    logMode = true;
}

/** \brief Turns the write-to-log mode off
*
* Whenever this function is called, the mode in which the synchronized ImageSet instances are written
* to log directory is deactivated
*
* \sa enableLogMode()
*/
void Logger::disableLogMode()
{
    logMode = false;
}

/** \brief Sets the input image and its time to receive as the last received image
*
* \param [in]   image   cv::Mat; The new camera image
* \param [in]   time    Double; The exact time instant in which the image is received
*
* This function must be called once the camera image is received. If write-to-log mode is on, it
* saves it in the log directory
*/
void Logger::setImage(Mat &image, double time)
{
    if (readFromLog)
        return;

    img.image = image;
    img.time = time;

    if (logMode)
    {
        ImageSet imgSt;
        if (getImageSet(imgSt))
        {
            writeImageSet(imgSt);
        }
    }
}

/** \brief Sets the input ImageSet data into the log directory
*
* \param [in]   imgSet  The desired ImageSet to write
*/
void Logger::writeImageSet(const ImageSet &imgSet)
{
    counter++;
    std::string imgName = "image" + std::to_string(counter) + ".jpg";
    std::string address = logsDir + imgName;
    imwrite(address, imgSet.image);
    logFile.open(logsDir + "log.txt", std::ios::out | std::ios::in | std::ios::app);
    logFile << imgName << ',' << std::to_string(imgSet.time) << ',' << std::to_string(imgSet.lat) << ',' << std::to_string(imgSet.lng) \
            << ',' << std::to_string(imgSet.alt) << ',' << std::to_string(imgSet.roll) << ',' << std::to_string(imgSet.pitch) \
            << ',' << std::to_string(imgSet.azimuth) << std::endl;
    logFile.close();
}

/** \brief Reads a data sequence including image, IMU and GPS data from pre-logged directory
*
* \param [in]   imData  A comma delimited text line containing IMU and GPS data
* \param [out]  image   The output image read from log
* \param [out]  imuSt   The output IMU and GPS data read from file
*
* The pre-logged file is a fixed global member. Thus, with each call, this function reads the next data
* data sequence
*/
void Logger::readData( std::string imData, Mat &image, ImuSet &imuSt)
{
    std::istringstream iss(imData);
    std::string num;
    int k = 0;
    while (std::getline(iss, num, ',') && k<8)
    {
        switch (k++)
        {
            case 0: {
                std::string imDir = prelogged_dir + num;
                image = imread(imDir);
                break;
            }
            case 1: {
                imuSt.time = std::stod(num);
                break;
            }
            case 2: {
                imuSt.lat = std::stod(num);
                break;
            }
            case 3: {
                imuSt.lng = std::stod(num);
                break;
            }
            case 4: {
                imuSt.alt = std::stod(num);
                break;
            }
            case 5: {
                imuSt.roll = std::stod(num);
                break;
            }
            case 6: {
                imuSt.pitch = std::stod(num);
                break;
            }
            case 7: {
                imuSt.azimuth = std::stod(num);
                break;
            }
            default: break;
        }
    }
}

/** \brief Sets the input GPS data and its time to receive as the last received location in a buffer
*
* \param [in]   lat     Latitude in degrees
* \param [in]   lng     Longitude in degrees
* \param [in]   alt     Altitude in meters
* \param [in]   time    The exact time instant in which the location is received
*
* This function must be called once the GPS data is received
*/
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
}

/** \brief Sets the input IMU data and its time to receive as the last received orientation in a buffer
*
* \param [in]   roll    Roll angle in degrees
* \param [in]   pitch   Pitch angle in degrees
* \param [in]   azimuth Azimuth angle in degrees
* \param [in]   time    The exact time instant in which the orientation is received
*
* \returns      true if the read-from-log mode is not active and so the orientation is set successfully
*
* This function must be called once the IMU data is received
*/
bool Logger::setOrientation(double roll, double pitch, double azimuth, double time)
{
    if (readFromLog)
        return false;

    Orientation orn;
    orn.roll = roll*PI/180;
    orn.pitch = pitch*PI/180;
    orn.azimuth = azimuth*PI/180;
    orn.time = time;

    bufferOrientation(orn);

    return true;
}

/** \brief Buffers the input location
*
* \param [in]   loc    The Location instance to be buffered
*
* This function adds the input location to the end of the class location buffer and deletes the oldest
* in case that buffer length is reached the location buffer's maximum allowed length (locBufLen)
*/
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

/** \brief Buffers the input orientation
*
* \param [in]   orn    The Orientation instance to be buffered
*
* This function adds the input orientation to the end of the class orientation buffer and deletes the
* oldest in case that buffer length is reached the orientation buffer's maximum allowed length (ornBufLen)
*/
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

/** \brief Reads and provides a pre-logged data sequence
*
* \param [out]   imgSt  The full new read data sequence as an ImageSet instance
* \param [out]   imuSt  The orientation and location of new read data sequence as an
*                       ImuSet instance
*
* \returns      true if the ImageSet data is read from log and provided successfully
*
* This function is called when the mode in which the pre-logged data is to be read is on. With each
* call, an image and its corresponding IMU and GPS data is read and provided at the output as an
* ImageSet instance along with an ImuSet instance
*/
bool Logger::getImageSetFromLogger(ImageSet &imgSt, ImuSet &imuSt)
{
    if (!readFromLog)
        return false;

    Mat image;

    std::string line_text;
    bool retVal = (bool) getline(iLogFile, line_text);
    if (!retVal)
        return false;

    readData(line_text, image, imuSt);

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

/** \brief Provides a synchronized ImageSet instance corresponding to the last received image
*
* \param [out]  imgSet  The synchronized ImageSet instance
*
* \returns      true if the ImageSet data is synchronized and provided successfully
*
* Whenever this function is called, the last received image and its nearest IMU and GPS data in terms
* of time are provided
*/
bool Logger::getImageSet(ImageSet &imgSet)
{
    Location location;
    Orientation orientation;
    double dist = 0, minDist = 1e7;

    if (locationBuffer.empty() || orientationBuffer.empty() || img.image.empty())
    {
        return false;
    }

    for(auto & k : locationBuffer)
    {
        dist = fabs(k.time - img.time);
        if (dist < minDist)
        {
            location = k;
            minDist = dist;
        }
    }

    dist = 0;
    minDist = 1e7;
    for(auto & k : orientationBuffer)
    {
        dist = fabs(k.time - img.time);
        if (dist < minDist)
        {
            orientation = k;
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

/** \brief Provides a synchronized ImuSet instance corresponding to the last received orientation
*
* \param [out]   imuSet     The synchronized ImuSet instance
*
* \returns      true if the ImageSet data is synchronized and provided successfully
*
* Whenever this function is called, the last received IMU data and its nearest GPS data in terms of
* time are provided as an ImuSet instance
*/
bool Logger::getImuSet(ImuSet &imuSet)
{
    Location location;
    float dist = 0, minDist = 1e7;

    if (orientationBuffer.empty() || locationBuffer.empty() || img.image.empty())
    {
        return false;
    }

    for(auto & k : locationBuffer)
    {
        dist = fabs(k.time - orientationBuffer.back().time);
        if (dist < minDist)
        {
            location = k;
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