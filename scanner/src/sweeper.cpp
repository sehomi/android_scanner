

#include <iostream>
//#include <bits/stdc++.h>
#include "sweeper.h"

using namespace std;
using namespace SweeperGeometry;

#define INF 10000

Location Sweeper::p0;

bool Sweeper::onSegment(Point p, Point q, Point r)
{
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) &&
        q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y))
        return true;
    return false;
}


int Sweeper::orientation(Point p, Point q, Point r)
{
    int val = (q.y - p.y) * (r.x - q.x) -
              (q.x - p.x) * (r.y - q.y);

    if (val == 0) return 0; // colinear
    return (val > 0)? 1: 2; // clock or counterclock wise
}


bool Sweeper::doIntersect(Point p1, Point q1, Point p2, Point q2)
{
    // Find the four orientations needed for general and
    // special cases
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // General case
    if (o1 != o2 && o3 != o4)
        return true;

    // Special Cases
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;

    // p1, q1 and p2 are colinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;

    // p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;

    // p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;

    return false;
}

bool Sweeper::isInside(Point p)
{
    int n = polygon.size();

    if (n < 3) return false;

    // Create a point for line segment from p to infinite
    Point extreme = {INF, p.y};

    // Count intersections of the above line with sides of polygon
    int count = 0, i = 0;
    do
    {
        int next = (i+1)%n;

        // Check if the line segment from 'p' to 'extreme' intersects
        // with the line segment from 'polygon[i]' to 'polygon[next]'
        if (doIntersect(polygon[i], polygon[next], p, extreme))
        {
            // If the point 'p' is colinear with line segment 'i-next',
            // then check if it lies on segment. If it lies, return true,
            // otherwise false
            if (orientation(polygon[i], p, polygon[next]) == 0)
                return onSegment(polygon[i], p, polygon[next]);

            count++;
        }
        i = next;
    } while (i != 0);

    // Return true if count is odd, false otherwise
    return count&1; // Same as (count%2 == 1)
}

void Sweeper::update(std::vector<Location> & fov_loc, std::vector<Location> & output)
{

    for (int i=0; i<fov_loc.size(); i++){
        Point p;
        p.x = fov_loc[i].lng;
        p.y = fov_loc[i].lat;

        if ( !isInside(p) )
        {
            sweeped_area_loc.push_back(fov_loc[i]);
            polygon.push_back(p);
        }
    }

    refineLocations(polygon, sweeped_area_loc);
    output = sweeped_area_loc;
    sortToClosedPath(output);
}


void Sweeper::swap(Location &p1, Location &p2)
{
    Location temp = p1;
    p1 = p2;
    p2 = temp;
}


double Sweeper::dist(Location p1, Location p2)
{
    return (p1.lng - p2.lng)*(p1.lng - p2.lng) +
           (p1.lat - p2.lat)*(p1.lat - p2.lat);
}

int Sweeper::orientation(Location p, Location q, Location r)
{
    double val = (q.lat - p.lat) * (r.lng - q.lng) -
              (q.lng - p.lng) * (r.lat - q.lat);

    if (val == 0) return 0; // colinear
    return (val > 0)? 1: 2; // clock or counterclock wise
}

int Sweeper::compare(const void *vp1, const void *vp2)
{
    Location *p1 = (Location *)vp1;
    Location *p2 = (Location *)vp2;

    // Find orientation
    int o = orientation(p0, *p1, *p2);
    if (o == 0)
        return (dist(p0, *p2) >= dist(p0, *p1))? -1 : 1;

    return (o == 2)? -1: 1;
}


void Sweeper::sortToClosedPath(vector<Location> &points)
{

    int n = points.size();

    // Find the bottommost point
    double ymin = points[0].lat, min = 0;
    for (int i = 1; i < n; i++)
    {
        double y = points[i].lat;

        // Pick the bottom-most. In case of tie, chose the
        // left most point
        if ((y < ymin) || (ymin == y &&
                           points[i].lng < points[min].lng))
            ymin = points[i].lat, min = i;
    }

    // Place the bottom-most point at first position
    swap(points[0], points[min]);

    // Sort n-1 points with respect to the first point.
    // A point p1 comes before p2 in sorted output if p2
    // has larger polar angle (in counterclockwise
    // direction) than p1
    p0 = points[0];
    qsort(&points[1], n-1, sizeof(Location), compare);

}

void Sweeper::refineLocations(std::vector<Point> &points, std::vector<Location> &locs){

    vector<bool> idxs_to_remove(locs.size());

    for(int i=0; i<locs.size()-1; i++){
        for(int j=i+1; j<locs.size(); j++){
            if (Sweeper::dist(locs[i], locs[j]) < (double)0.00002 * (double)0.00002)
            {
                idxs_to_remove.at(j) = true;
            }
        }
    }

    std::vector<Point> new_points;
    std::vector<Location> new_locs;
    for(int i=0; i<locs.size()-1; i++){
        if (!idxs_to_remove.at(i))
        {
            new_points.push_back(points.at(i));
            new_locs.push_back(locs.at(i));
        }
    }

    points = new_points;
    locs = new_locs;
}