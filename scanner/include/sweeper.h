
#ifndef ANDROID_SCANNER_SWEEPER_H
#define ANDROID_SCANNER_SWEEPER_H

#include <iostream>
#include <vector>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/io/wkt/wkt.hpp>

#include <boost/foreach.hpp>

// TODO: included for "Location" structure. all typedefs must be moved to an external base header file.
#include "Logger.h"

namespace SweeperGeometry{

    typedef boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double> > polygon;
    typedef boost::geometry::model::d2::point_xy<double> boost2dPoint;

    class Sweeper {

    public:

        void update(std::vector<Location> &, std::vector<Location> &);

    private:

        static Location p0;
        std::vector<Location> sweeped_area_loc;
        polygon sweeped_area;
        bool isFirstPolygon = true;

        static double dist(double, double, double, double);
        void refineLocations(polygon &);

    };
}

#endif //ANDROID_SCANNER_SWEEPER_H