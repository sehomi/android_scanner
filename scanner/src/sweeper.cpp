

#include <iostream>

#include "sweeper.h"

using namespace std;
using namespace SweeperGeometry;

/** \brief updates the sweeped polygon. merges a new field of view polygon with the sweeped area polygon.
*
* \param [in]     fov_loc     The new field of view of the camera.
*
* \param [out]    output      The updated sweeped area. The result of the merge of new fov with last sweeped area.
*
* This function makes use of Boost library geometrical algorithms to merge polygons.
*/

void Sweeper::update(std::vector<Object> & fov_loc, std::vector<Object> & output) {

    polygon new_poly;

    for (int i = 0; i < fov_loc.size(); i++) {
        Location ver = fov_loc.at(i).location;
        boost::geometry::append(new_poly, boost::geometry::make<boost2dPoint>(ver.lng, ver.lat));
    }
    Location last = fov_loc[fov_loc.size() - 1].location;
    boost::geometry::append(new_poly, boost::geometry::make<boost2dPoint>(last.lng, last.lat));
    boost::geometry::correct(new_poly);

    if (isFirstPolygon) {
        sweeped_area = new_poly;
        isFirstPolygon = false;
        output = fov_loc;
        return;
    }

    std::vector<polygon> out_polygons;
    boost::geometry::union_(sweeped_area, new_poly, out_polygons);
    output = std::vector<Object>();

    for (int i = 0; i < out_polygons.size(); i++) {

        if (i != 0) continue;


        polygon temp_poly = out_polygons.at(i);
        refineLocations(temp_poly);
        sweeped_area = temp_poly;

        for (auto it = boost::begin(boost::geometry::exterior_ring(temp_poly));
             it != boost::end(boost::geometry::exterior_ring(temp_poly)); ++it) {
            Location temp_loc;
            temp_loc.lng = boost::geometry::get<0>(*it);
            temp_loc.lat = boost::geometry::get<1>(*it);

            output.push_back({.type = Object::SWEPT, .location = temp_loc});
        }
    }
}

double Sweeper::dist(double x1, double y1, double x2, double y2)
{
    return (x1 - x2)*(x1 - x2) +
           (y1 - y2)*(y1 - y2);
}


/** \brief Removes close vertices of sweeped polygon for memory efficiency.
*
* \param [in,out]     points     The polygon that needs to be refined.
*
* This function finds distance between polygon vertices and those which are closer than 0.00002 deg
* in terms of lat & lon, will be removed.
*/

void Sweeper::refineLocations(polygon &points){

    polygon refined_points;
    int size = boost::end(boost::geometry::exterior_ring(points)) - boost::begin(boost::geometry::exterior_ring(points));
    vector<bool> idxs_to_remove(size);

    int i = -1;
    for(auto it = boost::begin(boost::geometry::exterior_ring(points)); it != boost::end(boost::geometry::exterior_ring(points)) - 1; ++it)
    {
        i++;
        if (idxs_to_remove.at(i)) continue;

        double x1 = boost::geometry::get<0>(*it);
        double y1 = boost::geometry::get<1>(*it);

        int j = i+1;
        for(auto it1 = it+1; it1 != boost::end(boost::geometry::exterior_ring(points)) - 1; ++it1)
        {
            double x2 = boost::geometry::get<0>(*it1);
            double y2 = boost::geometry::get<1>(*it1);

            if (Sweeper::dist(x1, y1, x2, y2) < (double)0.00002 * (double)0.00002)
            {
                idxs_to_remove.at(j) = true;
            }

            j++;
        }

    }

    i = -1;
    for(auto it = boost::begin(boost::geometry::exterior_ring(points)); it != boost::end(boost::geometry::exterior_ring(points)); ++it)
    {
        i++;
        if (idxs_to_remove.at(i))
            continue;
        else{
            double x = boost::geometry::get<0>(*it);
            double y = boost::geometry::get<1>(*it);
            boost::geometry::append( refined_points, boost::geometry::make<boost2dPoint>(x, y) );
        }
    }

    points = refined_points;
}