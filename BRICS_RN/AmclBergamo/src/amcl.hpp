#include "map/map.h"
#include "pf/pf.h"
#include "sensors/amcl_odom.h"
#include "sensors/amcl_laser.h"

#include <cmath>

#include "nav_msgs/typekit/Types.h"
#include "geometry_msgs/typekit/Types.h"
#include <tf/tf.h>

/*
#include <algorithm>
#include <vector>
#include <map>

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

#include "ros/assert.h"

// roscpp
#include "ros/ros.h"

// Messages that I need
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/GetMap.h"
#include "std_srvs/Empty.h"

// For transform support
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

// Dynamic_reconfigure
#include "dynamic_reconfigure/server.h"
#include "amcl/AMCLConfig.h"

*/

#define NEW_UNIFORM_SAMPLING 1

using namespace amcl;

// Pose hypothesis
typedef struct
{
  // Total weight (weights sum to 1)
  double weight;

  // Mean of pose esimate
  pf_vector_t pf_pose_mean;

  // Covariance of pose estimate
  pf_matrix_t pf_pose_cov;

} amcl_hyp_t;

#if NEW_UNIFORM_SAMPLING
    static std::vector<std::pair<int,int> > free_space_indices;
#endif

class AMCLocalizer {
  public:
	AMCLocalizer();
    ~AMCLocalizer();

  private:
    /*
     * Particle filter
     */
    pf_t *pf_;
    double pf_err_, pf_z_;
    bool pf_init_;
    double d_thresh_, a_thresh_;
    int resample_interval_;
    int resample_count_;
    int max_beams_, min_particles_, max_particles_;

    /*
     * Pose
     */
    geometry_msgs::PoseWithCovarianceStamped last_published_pose;
    double init_pose_[3];
    double init_cov_[3];
    amcl_hyp_t* initial_pose_hyp_;
    pf_vector_t pf_odom_pose_;
  
    bool first_reconfigure_call_;
    //bool sent_first_transform_;
    //bool use_map_topic_;
    //bool first_map_only_;

    /*
     * Odometry
     */
    AMCLOdom* odom_;
    odom_model_t odom_model_type_;
    double alpha1_, alpha2_, alpha3_, alpha4_, alpha5_;
    double alpha_slow_, alpha_fast_;

    /*
     * Laser
     */
    AMCLLaser* laser_;
    laser_model_t laser_model_type_;
    double z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_;
    double laser_likelihood_max_dist_;
    double laser_min_range_;
    double laser_max_range_;

    /*
     * Map
     */
    map_t* map_;
    char* mapdata;
    int sx, sy;
    double resolution;
    bool first_map_received_;

public:

    /*
     * memory management functions
     */
    void initialize();
    void configure();
    void freeMapDependentMemory();


    /*
     * map management functions
     */
    void initMap( const nav_msgs::OccupancyGrid& map_msg );
    //map_t* convertMap( const nav_msgs::OccupancyGrid& map_msg );
    //void handleMapMessage(const nav_msgs::OccupancyGrid& msg);


    /*
     * pose management functions
     */
    double getYaw(tf::Pose& t);
    void applyInitialPose();
    void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    static pf_vector_t uniformPoseGenerator(void* arg);
};
