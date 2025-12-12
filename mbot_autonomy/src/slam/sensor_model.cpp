#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/grid_utils.hpp>
#include <utils/geometric/point.hpp>
#include <set>
SensorModel::SensorModel(void)
:   sigma_hit_(0.075),
	occupancy_threshold_(10),
	ray_stride_(7),
	max_ray_range_(1000),
    search_range(2),
    offset_quality_weight(3)
{
    initialize_bfs_offsets();
}

void SensorModel::initialize_bfs_offsets()
{
    /// TODO: Initialize the BFS offsets based on the search range 
    // bfs_offsets_.clear();
     
    // for(int dx = -search_range; dx <= search_range; dx++){
    //     for(int dy = -search_range; dy <= search_range; dy++){
    //         if(dx != 0 || dy != 0){
    //             bfs_offsets_.emplace_back(dx, dy);
    //         }
    //     }
    // }
}

double SensorModel::likelihood(const mbot_lcm_msgs::particle_t& sample, 
                               const mbot_lcm_msgs::lidar_t& scan, 
                               const OccupancyGrid& map)
{
    /// TODO: Compute the likelihood of the given particle using the provided laser scan and map. 
    
    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose);
    double scanScore = 0.0;
    double rayScore;

    for(auto& ray : movingScan){
        Point<double> endpoint(ray.origin.x + ray.range * std::cos(ray.theta),
                               ray.origin.y + ray.range * std::sin(ray.theta));
        auto rayEnd = global_position_to_grid_position(endpoint, map);

        if(map.logOdds(rayEnd.x, rayEnd.y) > 0.0){
            scanScore += 1.0;
        }
    }

    // for(auto& ray : movingScan){
    //     scanScore += scoreRay(ray, map);
    // }

    return scanScore;
    // return 0.0; // Placeholder
}

double SensorModel::scoreRay(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    /// TODO: Compute a score for a given ray based on its end point and the map. 
    // Consider the offset from the nearest occupied cell.  

    // Point<float> rayEndOnMap = getRayEndPointOnMap(ray, map);
    // Point<int> gridEnd = global_position_to_grid_position(rayEndOnMap, map);
    // Point<int> nearest_occ = gridBFS(gridEnd, map);
    
    // double dist = std::sqrt(std::pow((gridEnd.x - nearest_occ.x), 2.0) + std::pow((gridEnd.y - nearest_occ.y), 2.0));
    // return NormalPdf(dist) * offset_quality_weight;
    // return 0.0; // Placeholder
}

double SensorModel::NormalPdf(const double& x)
{
    return (1.0/(sqrt(2.0 * M_PI)*sigma_hit_))*exp((-0.5*x*x)/(sigma_hit_*sigma_hit_));
}

Point<int> SensorModel::gridBFS(const Point<int> end_point, const OccupancyGrid& map)
{
    /// TODO: Use Breadth First Search to find the nearest occupied cell to the given end point. 
    // std::vector<Point<int>> open_list;
    // open_list.push_back(end_point);

    // std::set<Point<int>> closed_list;
    // closed_list.insert(end_point);

    // while(!open_list.empty()){
    //     Point<int> current = open_list.front();
    //     open_list.erase(open_list.begin());

    //     if(map.logOdds(current.x, current.y) > occupancy_threshold_){
    //         return current;
    //     }

    //     for(auto& offset : bfs_offsets_){
    //         Point<int> neighbor(current.x + offset.x, current.y + offset.y);

    //         if(map.isCellInGrid(neighbor.x, neighbor.y) && closed_list.find(neighbor) == closed_list.end()){
    //             open_list.push_back(neighbor);
    //             closed_list.insert(neighbor);
    //         }
    //     }
    // }

    // return end_point;
    return Point<int>(0,0); // Placeholder
}

Point<float> SensorModel::getRayEndPointOnMap(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    /// TODO: Calculate the end point of a given ray on the map 
    Point<int> rayStart = global_position_to_grid_cell(ray.origin, map);
    Point<int> rayEnd;
    rayEnd.x = static_cast<int>((ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + rayStart.x);
    rayEnd.y = static_cast<int>((ray.range * std::sin(ray.theta) * map.cellsPerMeter()) + rayStart.y);

    int x0 = rayStart.x;
    int y0 = rayStart.y;
    int x1 = rayEnd.x;
    int y1 = rayEnd.y;

    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    int x = x0;
    int y = y0;

    while(true){
        if(!map.isCellInGrid(x, y)){
            break;
        }

        if(map.logOdds(x, y) > occupancy_threshold_){
            break;
        }

        if(x == x1 && y == y1){
            break;
        }

        int e2 = 2 * err;
        if(e2 >= -dy){
            err = err - dy;
            x = x + sx;
        }
        if(e2 <= dx){
            err = err + dx;
            y = y + sy;
        }
    }
    
    return grid_position_to_global_position(Point<int>(x, y), map);
    // return Point<float>(0.0f, 0.0f); // Placeholder
}
