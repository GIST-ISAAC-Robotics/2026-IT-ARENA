static inline float getAngle(const sl_lidar_response_measurement_node_hq_t& node)
    {
        return node.angle_z_q14 * 90.f / 16384.f;
    }
static inline void setAngle(sl_lidar_response_measurement_node_hq_t& node, float v)
    {
        node.angle_z_q14 = sl_u32(v * 16384.f / 90.f);
    }
static inline sl_u32 getDistanceQ2(const sl_lidar_response_measurement_node_hq_t& node)
    {
        return node.dist_mm_q2;
    }
template <class TNode>
    static bool angleLessThan(const TNode& a, const TNode& b)
    {
        return getAngle(a) < getAngle(b);
    }
template < class TNode >
    static sl_result ascendScanData_(TNode * nodebuffer, size_t count)
    {
        float inc_origin_angle = 360.f / count;
        size_t i = 0;

        //Tune head
        for (i = 0; i < count; i++) {
            if (getDistanceQ2(nodebuffer[i]) == 0) {
                continue;
            }
            else {
                while (i != 0) {
                    i--;
                    float expect_angle = getAngle(nodebuffer[i + 1]) - inc_origin_angle;
                    if (expect_angle < 0.0f) expect_angle = 0.0f;
                    setAngle(nodebuffer[i], expect_angle);
                }
                break;
            }
        }

        // all the data is invalid
        if (i == count) return SL_RESULT_OPERATION_FAIL;

        //Tune tail
        for (i = count - 1; i < count; i--) {
            // To avoid array overruns, use the i < count condition
            if (getDistanceQ2(nodebuffer[i]) == 0) {
                continue;
            }
            else {
                while (i != (count - 1)) {
                    i++;
                    float expect_angle = getAngle(nodebuffer[i - 1]) + inc_origin_angle;
                    if (expect_angle > 360.0f) expect_angle -= 360.0f;
                    setAngle(nodebuffer[i], expect_angle);
                }
                break;
            }
        }

        //Fill invalid angle in the scan
        float frontAngle = getAngle(nodebuffer[0]);
        for (i = 1; i < count; i++) {
            if (getDistanceQ2(nodebuffer[i]) == 0) {
                float expect_angle = frontAngle + i * inc_origin_angle;
                if (expect_angle > 360.0f) expect_angle -= 360.0f;
                setAngle(nodebuffer[i], expect_angle);
            }
        }

        // Reorder the scan according to the angle value
        std::sort(nodebuffer, nodebuffer + count, &angleLessThan<TNode>);

        return SL_RESULT_OK;
    }
void publish_scan(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& pub,
                  sl_lidar_response_measurement_node_hq_t *nodes,
                  size_t node_count, rclcpp::Time start,
                  double scan_time, bool inverted,
                  float angle_min, float angle_max,
                  float max_distance,
                  std::string frame_id)
    {
        static int scan_count = 0;
        auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

        scan_msg->header.stamp = start;
        scan_msg->header.frame_id = frame_id;
        scan_count++;

        bool reversed = (angle_max > angle_min);
        if ( reversed ) {
            scan_msg->angle_min =  M_PI - angle_max;
            scan_msg->angle_max =  M_PI - angle_min;
        } else {
            scan_msg->angle_min =  M_PI - angle_min;
            scan_msg->angle_max =  M_PI - angle_max;
        }
        scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / (double)(node_count-1);

        scan_msg->scan_time = scan_time;
        scan_msg->time_increment = scan_time / (double)(node_count-1);
        scan_msg->range_min = 0.05;
        scan_msg->range_max = max_distance;//8.0;

        scan_msg->intensities.resize(node_count);
        scan_msg->ranges.resize(node_count);
        bool reverse_data = (!inverted && reversed) || (inverted && !reversed);
        if (!reverse_data) {
            for (size_t i = 0; i < node_count; i++) {
                float read_value = (float) nodes[i].dist_mm_q2/4.0f/1000;
                if (read_value == 0.0)
                    scan_msg->ranges[i] = std::numeric_limits<float>::infinity();
                else
                    scan_msg->ranges[i] = read_value;
                scan_msg->intensities[i] = (float) (nodes[i].quality >> 2);
            }
        } else {
            for (size_t i = 0; i < node_count; i++) {
                float read_value = (float)nodes[i].dist_mm_q2/4.0f/1000;
                if (read_value == 0.0)
                    scan_msg->ranges[node_count-1-i] = std::numeric_limits<float>::infinity();
                else
                    scan_msg->ranges[node_count-1-i] = read_value;
                scan_msg->intensities[node_count-1-i] = (float) (nodes[i].quality >> 2);
            }
        }

        pub->publish(*scan_msg);
    }
void compensate(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& scan_pub,
  sl_lidar_response_measurement_node_hq_t* nodes, size_t count,
  size_t angle_compensate_multiple, double start, double scan_duration) {
  const rclcpp::Time start_scan_time{start};
  const bool inverted=false;
  const float angle_min=0, angle_max=2*M_PI, max_distance=12;
  const std::string frame_id="laser";

                        //const int angle_compensate_multiple = 1;
                        const int angle_compensate_nodes_count = 360*angle_compensate_multiple;
                        int angle_compensate_offset = 0;
                        auto angle_compensate_nodes = new sl_lidar_response_measurement_node_hq_t[angle_compensate_nodes_count];
                        memset(angle_compensate_nodes, 0, angle_compensate_nodes_count*sizeof(sl_lidar_response_measurement_node_hq_t));

                        size_t i = 0, j = 0;
                        for( ; i < count; i++ ) {
                            if (nodes[i].dist_mm_q2 != 0) {
                                float angle = getAngle(nodes[i]);
                                int angle_value = (int)(angle * angle_compensate_multiple);
                                if ((angle_value - angle_compensate_offset) < 0) angle_compensate_offset = angle_value;
                                for (j = 0; j < angle_compensate_multiple; j++) {
                                    int angle_compensate_nodes_index = angle_value-angle_compensate_offset + j;
                                    if(angle_compensate_nodes_index >= angle_compensate_nodes_count)
                                        angle_compensate_nodes_index = angle_compensate_nodes_count - 1;
                                    angle_compensate_nodes[angle_compensate_nodes_index] = nodes[i];
                                }
                            }
                        }
    
                        publish_scan(scan_pub, angle_compensate_nodes, angle_compensate_nodes_count,
                                start_scan_time, scan_duration, inverted,
                                angle_min, angle_max, max_distance,
                                frame_id);

                        if (angle_compensate_nodes) {
                            delete[] angle_compensate_nodes;
                            angle_compensate_nodes = nullptr;
                        }
                    
}
