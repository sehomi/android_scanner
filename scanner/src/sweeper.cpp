

#include <iostream>

//#include <bits/stdc++.h>
#include "sweeper.h"

using namespace std;
using namespace SweeperGeometry;
//using boost::geometry::append;
//using boost::geometry::make;
//using boost::geometry::correct;

void Sweeper::update(std::vector<Location> & fov_loc, std::vector<Location> & output)
{

    polygon new_poly;

    for (int i=0; i<fov_loc.size(); i++)
    {
        Location ver = fov_loc.at(i);
        boost::geometry::append( new_poly, boost::geometry::make<boost2dPoint>(ver.lng, ver.lat) );
    }
    Location last = fov_loc[fov_loc.size()-1];
    boost::geometry::append( new_poly, boost::geometry::make<boost2dPoint>(last.lng, last.lat) );
    boost::geometry::correct( new_poly );

    if (isFirstPolygon)
    {
        sweeped_area = new_poly;
        isFirstPolygon = false;
        output = fov_loc;
        return;
    }

    std::vector<polygon> out_polygon;
    boost::geometry::union_(sweeped_area, new_poly, out_polygon);
    output = std::vector<Location>();

    for(int i=0;i<out_polygon.size();i++){

        if (i!=0) continue;

        polygon temp_poly = out_polygon.at(i);
        sweeped_area = temp_poly;

        for(auto it = boost::begin(boost::geometry::exterior_ring(temp_poly)); it != boost::end(boost::geometry::exterior_ring(temp_poly)); ++it)
        {
            Location temp_loc;
            temp_loc.lng = boost::geometry::get<0>(*it);
            temp_loc.lat = boost::geometry::get<1>(*it);

            output.push_back(temp_loc);
        }
    }
//    for (int i=0; i<fov_loc.size(); i++){
//        Point p;
//        p.x = fov_loc[i].lng;
//        p.y = fov_loc[i].lat;
//
//        if ( !isInside(p) )
//        {
//            sweeped_area_loc.push_back(fov_loc[i]);
//            polygon.push_back(p);
//        }
//    }
//
////    refineLocations(polygon, sweeped_area_loc);
//    output = sweeped_area_loc;
//    sortToClosedPath(output);
}

//
//void Sweeper::swap(Location &p1, Location &p2)
//{
//    Location temp = p1;
//    p1 = p2;
//    p2 = temp;
//}
//
//
//double Sweeper::dist(Location p1, Location p2)
//{
//    return (p1.lng - p2.lng)*(p1.lng - p2.lng) +
//           (p1.lat - p2.lat)*(p1.lat - p2.lat);
//}
//
//int Sweeper::orientation(Location p, Location q, Location r)
//{
//    double val = (q.lat - p.lat) * (r.lng - q.lng) -
//              (q.lng - p.lng) * (r.lat - q.lat);
//
//    if (val == 0) return 0; // colinear
//    return (val > 0)? 1: 2; // clock or counterclock wise
//}
//
//int Sweeper::compare(const void *vp1, const void *vp2)
//{
//    Location *p1 = (Location *)vp1;
//    Location *p2 = (Location *)vp2;
//
//    // Find orientation
//    int o = orientation(p0, *p1, *p2);
//    if (o == 0)
//        return (dist(p0, *p2) >= dist(p0, *p1))? -1 : 1;
//
//    return (o == 2)? -1: 1;
//}
//
//
//void Sweeper::sortToClosedPath(vector<Location> &points)
//{
//
//    int n = points.size();
//
//    // Find the bottommost point
//    double ymin = points[0].lat, min = 0;
//    for (int i = 1; i < n; i++)
//    {
//        double y = points[i].lat;
//
//        // Pick the bottom-most. In case of tie, chose the
//        // left most point
//        if ((y < ymin) || (ymin == y &&
//                           points[i].lng < points[min].lng))
//            ymin = points[i].lat, min = i;
//    }
//
//    // Place the bottom-most point at first position
//    swap(points[0], points[min]);
//
//    // Sort n-1 points with respect to the first point.
//    // A point p1 comes before p2 in sorted output if p2
//    // has larger polar angle (in counterclockwise
//    // direction) than p1
//    p0 = points[0];
//    qsort(&points[1], n-1, sizeof(Location), compare);
//
//}
//
//void Sweeper::refineLocations(std::vector<Point> &points, std::vector<Location> &locs){
//
//    vector<bool> idxs_to_remove(locs.size());
//
//    for(int i=0; i<locs.size()-1; i++){
//        for(int j=i+1; j<locs.size(); j++){
//            if (Sweeper::dist(locs[i], locs[j]) < (double)0.00002 * (double)0.00002)
//            {
//                idxs_to_remove.at(j) = true;
//            }
//        }
//    }
//
//    std::vector<Point> new_points;
//    std::vector<Location> new_locs;
//    for(int i=0; i<locs.size()-1; i++){
//        if (!idxs_to_remove.at(i))
//        {
//            new_points.push_back(points.at(i));
//            new_locs.push_back(locs.at(i));
//        }
//    }
//
//    points = new_points;
//    locs = new_locs;
//}