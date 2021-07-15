
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

        std::vector<Location> sweeped_area_loc;
        std::vector<Point> polygon;

        bool onSegment(Point p, Point q, Point r);
        int orientation(Point p, Point q, Point r);
        bool doIntersect(Point p1, Point q1, Point p2, Point q2);
        bool isInside(Point p);

    };
}

#endif //ANDROID_SCANNER_SWEEPER_H