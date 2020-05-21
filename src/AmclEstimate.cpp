#include <AmclEstimate.h>

AmclEstimate::AmclEstimate(ros::NodeHandle nh)
{
    map_received = false;
    estimate_current_score = 0;
    nomotion_updates = 0;
    node_handle = nh;

    node_handle.param<double>("range_threshold", range_threshold, 0.1);
    node_handle.param<double>("estimate_minimum_score", estimate_minimum_score, 0.75);
    node_handle.param<int>("nomotion_max_retries", nomotion_max_retries, 25);
    node_handle.param<std::string>("map_topic", map_topic, "/map");
    node_handle.param<std::string>("scan_topic", scan_topic, "/scan");
    node_handle.param<std::string>("global_localization_service", global_localization_service, "/global_localization");
    node_handle.param<std::string>("nomotion_update_service", nomotion_update_service, "/request_nomotion_update");
    _localization_client = node_handle.serviceClient<std_srvs::Empty>(global_localization_service);
    _nomotion_update_client = node_handle.serviceClient<std_srvs::Empty>(nomotion_update_service);
    _map_subscriber = node_handle.subscribe(map_topic, 1, &AmclEstimate::map_callback, this);
    _scan_subscriber = node_handle.subscribe(scan_topic, 1, &AmclEstimate::scan_callback, this);

    listener = new tf::TransformListener(node_handle, ros::Duration(30), true);
    while (ros::ok() && !_localization_client.waitForExistence(ros::Duration(0.1)))
    {
        ROS_INFO("Wait for %s", global_localization_service.c_str());
        ros::spinOnce();
    }
    while (ros::ok() && !_nomotion_update_client.waitForExistence(ros::Duration(0.1)))
    {
        ROS_INFO("Wait for %s", nomotion_update_service.c_str());
        ros::spinOnce();
    }

    ros::Rate loop_rate(4);

    while (ros::ok() && !map_received)
    {
        loop_rate.sleep();
        ros::spinOnce();
    }
    call_global_localization();
    while (ros::ok() && estimate_current_score < estimate_minimum_score && nomotion_updates < nomotion_max_retries)
    {
        call_nomotion_update();
        nomotion_updates++;
        ros::spinOnce();
        update_estimate();
        loop_rate.sleep();
    }
    if (nomotion_updates == nomotion_max_retries)
    {
        ROS_ERROR("Could not find estimate with sufficient score after %d updates.", nomotion_updates);
    }
    else
    {
        ROS_INFO("Estimate found after %d updates with score %f", nomotion_updates, estimate_current_score);
    }
}

AmclEstimate::~AmclEstimate()
{
}

bool AmclEstimate::update_estimate()
{
    if (current_scan.header.stamp > last_loc_update)
    {

        try
        {
            listener->waitForTransform(current_map.header.frame_id,
                                       current_scan.header.frame_id,
                                       current_scan.header.stamp,
                                       ros::Duration(1.0));
            listener->lookupTransform(current_map.header.frame_id,
                                      current_scan.header.frame_id,
                                      current_scan.header.stamp,
                                      scan_transform);

            double roll, pitch, yaw;
            tf::Matrix3x3(scan_transform.getRotation()).getRPY(roll, pitch, yaw);

            scan_x = scan_transform.getOrigin()[0];
            scan_y = scan_transform.getOrigin()[1];
            scan_theta = yaw;
            estimate_current_score = calculate_match();
            ROS_INFO("Scan match quality: %f", estimate_current_score);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
        }
    }
    else
    {
        ROS_ERROR("Could not get scan with proper timestamp");
    }
}

void AmclEstimate::map_callback(const nav_msgs::OccupancyGridConstPtr &map)
{
    current_map = *map;
    map_received = true;
}

void AmclEstimate::scan_callback(const sensor_msgs::LaserScanConstPtr &scan)
{
    current_scan = *scan;
}

bool AmclEstimate::call_global_localization()
{
    if (_localization_client.call(localization_req_res))
    {

        last_loc_update == ros::Time::now();
    }
}

bool AmclEstimate::call_nomotion_update()
{
    if (_nomotion_update_client.call(localization_req_res))
    {
        last_loc_update == ros::Time::now();
    }
}

double AmclEstimate::calculate_match()
{
    double match = 0;
    long valid_points = 0;
    long exact_match = 0;
    long range_match = 0;
    double dist;
    for (long i = 0; i < current_scan.ranges.size(); i++)
    {
        if (current_scan.ranges[i] > current_scan.range_min && current_scan.ranges[i] < current_scan.range_max)
        {
            valid_points++;
            double point_x_pos = scan_x + current_scan.ranges[i] * std::cos(scan_theta + current_scan.angle_min + current_scan.angle_increment * i);
            double point_y_pos = scan_y + current_scan.ranges[i] * std::sin(scan_theta + current_scan.angle_min + current_scan.angle_increment * i);
            if (is_point_within_map_range(point_x_pos, point_y_pos))
            {
                if (is_obstacle_at_point(point_x_pos, point_y_pos))
                {

                    exact_match++;
                }
                else if (is_obstacle_at_range(point_x_pos, point_y_pos, range_threshold, &dist))
                {
                    range_match++;
                }
            }
        }
    }
    match = (double)exact_match / (double)valid_points;
    match += (double)range_match / (double)valid_points;
    return match;
}

bool AmclEstimate::is_obstacle_at_point(double x, double y)
{
    long col = ((x - current_map.info.origin.position.x) / current_map.info.resolution) - 1;
    long row = ((y - current_map.info.origin.position.y) / current_map.info.resolution) - 1;
    if (current_map.data[(row * current_map.info.width) + col] > 50)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool AmclEstimate::is_obstacle_at_range(double x, double y, double range, double *dist_found)
{
    double res_sq = current_map.info.resolution * current_map.info.resolution;
    double dist_coefficient = 0;
    int grid_range = range / current_map.info.resolution;
    *dist_found = range;
    double dist = 0;
    bool range_found = false;
    for (int i = -grid_range; i < grid_range; i++)
    {
        for (int j = -grid_range; j < grid_range; j++)
        {
            dist = sqrt((i * i * res_sq) + (j * j * res_sq));

            if (dist < range)
            {
                if (is_point_within_map_range(x + i * current_map.info.resolution, y + j * current_map.info.resolution))
                {
                    if (is_obstacle_at_point(x + i * current_map.info.resolution, y + j * current_map.info.resolution))
                    {

                        if (dist < *dist_found)
                        {
                            *dist_found = dist;
                        }
                        range_found = true;
                    }
                }
            }
        }
    }
    return range_found;
}

bool AmclEstimate::is_point_within_map_range(double x, double y)
{
    if (x > current_map.info.origin.position.x)
    {
        if (x < current_map.info.origin.position.x + current_map.info.resolution * (double)current_map.info.width)
        {
            if (y > current_map.info.origin.position.y)
            {
                if (y < current_map.info.origin.position.y + current_map.info.resolution * (double)current_map.info.height)
                {

                    return true;
                }
            }
        }
    }
    return false;
}