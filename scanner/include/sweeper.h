
#ifndef ANDROID_SCANNER_SWEEPER_H
#define ANDROID_SCANNER_SWEEPER_H

#include <iostream>
#include <vector>

// TODO: included for "Location" structure. all typedefs must be moved to an external base header file.
#include "Logger.h"

namespace SweeperGeometry{

    struct Point
    {
        double x;
        double y;
    };

    class Sweeper {

    public:

        void update(std::vector<Location> &, std::vector<Location> &);

    private:

        static Location p0;
        std::vector<Location> sweeped_area_loc;
        std::vector<Point> polygon;

        bool onSegment(Point p, Point q, Point r);
        int orientation(Point p, Point q, Point r);
        bool doIntersect(Point p1, Point q1, Point p2, Point q2);
        bool isInside(Point p);
        void swap(Location &p1, Location &p2);
        static double dist(Location p1, Location p2);
        static int compare(const void *vp1, const void *vp2);
        static int orientation(Location p, Location q, Location r);
        void sortToClosedPath(std::vector<Location> &);
        void refineLocations(std::vector<Point> &, std::vector<Location> &);

    };
}

#endif //ANDROID_SCANNER_SWEEPER_H