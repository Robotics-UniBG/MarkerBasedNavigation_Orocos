#include "amcl.hpp"

#include <iostream>


static double normalize(double z) {
	return atan2(sin(z),cos(z));
}

static double angle_diff(double a, double b) {
	double d1, d2;
	a = normalize(a);
	b = normalize(b);
	d1 = a-b;
	d2 = 2*M_PI - fabs(d1);
	if(d1 > 0)
		d2 *= -1.0;
	if(fabs(d1) < fabs(d2))
		return(d1);
	else
		return(d2);
}


AMCLocalizer::AMCLocalizer() {
	map_ = NULL;
	pf_ = NULL;
	resample_count_ = 0;
	odom_ = NULL;
	laser_ = NULL;
	initial_pose_hyp_ = NULL;
	first_map_received_ = false;
	first_reconfigure_call_ = true;

	init_pose_[0] = 0.0;
	init_pose_[1] = 0.0;
	init_pose_[2] = 0.0;
	init_cov_[0] = 0.5 * 0.5;
	init_cov_[1] = 0.5 * 0.5;
	init_cov_[2] = (M_PI/12.0) * (M_PI/12.0);

	/*
	 * ********************* ROS specific ************************************
	 */
	// sent_first_transform_= false;		// serve ???
	// latest_tf_valid_ = false;			// serve ???
	// private_nh_ = "~";                   // serve ???
	// use_map_topic_, false;				// serve ???
	// first_map_only_, false;				// serve ???
	// tmp_tol = 0.1;						// serve ???  transform_tolerance
	/* *********************************************************************** */

	initialize();
}

AMCLocalizer::~AMCLocalizer()
{
	freeMapDependentMemory();
	/*
	 * ********************* ROS specific ************************************
	 */
	//  delete dsrv_;
	//  delete laser_scan_filter_;
	//  delete laser_scan_sub_;
	//  delete tfb_;
	//  delete tf_;
	/* *********************************************************************** */
}

/*
 * ************************************************************************
 * ********************* initialization functions *************************
 * ************************************************************************
 */

void AMCLocalizer::initialize() {
	d_thresh_ = 0.2;			// update_min_d
	a_thresh_ = M_PI/6.0;		// update_min_a
	resample_interval_ = 2; 	// resample_interval

	// laser parameters
	laser_min_range_ = -1.0;
	laser_max_range_ = -1.0;
	max_beams_ = 30;
	z_hit_ = 0.95;
	z_short_ = 0.1;
	z_max_ = 0.05;
	z_rand_ = 0.05;
	sigma_hit_ = 0.2;
	lambda_short_ = 0.1;
	laser_likelihood_max_dist_ = 2.0;
	laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD;  // default
	//  laser_model_type_ = LASER_MODEL_BEAM;

	// odometry parameters
	alpha1_ = 0.2;
	alpha2_ = 0.2;
	alpha3_ = 0.2;
	alpha4_ = 0.2;
	alpha5_ = 0.2;
	odom_model_type_ = ODOM_MODEL_DIFF;	 // default
	//  odom_model_type_ = ODOM_MODEL_OMNI;

	// pf parameters
	min_particles_ = 100;
	max_particles_ = 5000;
	alpha_slow_ = 0.001;		// recovery_alpha_slow
	alpha_fast_ = 0.1;			// recovery_alpha_fast
	pf_err_ = 0.01;
	pf_z_ = 0.99;


	/*
	 * ********************* ROS specific ************************************
	 */
	//  private_nh_.param("odom_frame_id", odom_frame_id_, std::string("odom"));
	//  private_nh_.param("base_frame_id", base_frame_id_, std::string("base_link"));
	//  private_nh_.param("global_frame_id", global_frame_id_, std::string("map"));
	//  double tmp_tol;
	//  transform_tolerance_.fromSec(tmp_tol);
	//  cloud_pub_interval.fromSec(1.0);
	//  tfb_ = new tf::TransformBroadcaster();
	//  tf_ = new tf::TransformListener();
	//  pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 2);
	//  particlecloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particlecloud", 2);
	//  global_loc_srv_ = nh_.advertiseService("global_localization",
	//					 &AmclNode::globalLocalizationCallback,  this);
	//  laser_scan_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, scan_topic_, 100);
	//  laser_scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_,
	//                                                        *tf_, odom_frame_id_,  100);
	//  laser_scan_filter_->registerCallback(boost::bind(&AmclNode::laserReceived,  this, _1));
	//  initial_pose_sub_ = nh_.subscribe("initialpose", 2, &AmclNode::initialPoseReceived, this);
	//  if(use_map_topic_) {
	//    map_sub_ = nh_.subscribe("map", 1, &AmclNode::mapReceived, this);
	//    ROS_INFO("Subscribed to map topic.");
	//  } else {
	//    requestMap();
	//  }
	//  dsrv_ = new dynamic_reconfigure::Server<amcl::AMCLConfig>(ros::NodeHandle("~"));
	//  dynamic_reconfigure::Server<amcl::AMCLConfig>::CallbackType cb = boost::bind(&AmclNode::reconfigureCB, this, _1, _2);
	//  dsrv_->setCallback(cb);
	//  // 15s timer to warn on lack of receipt of laser scans, #5209
	//  laser_check_interval_ = ros::Duration(15.0);
	//  check_laser_timer_ = nh_.createTimer(laser_check_interval_, boost::bind(&AmclNode::checkLaserReceived, this, _1));
	/* *********************************************************************** */
}


//void AmclNode::reconfigureCB(AMCLConfig &config, uint32_t level)
void AMCLocalizer::configure() {
	initialize();

	pf_ = pf_alloc(min_particles_, max_particles_,
			alpha_slow_, alpha_fast_,
			(pf_init_model_fn_t)AMCLocalizer::uniformPoseGenerator,
			(void *)map_);

	pf_->pop_err = pf_err_;
	pf_->pop_z = pf_z_;

	// Initialize the filter
	pf_vector_t pf_init_pose_mean = pf_vector_zero();
	pf_init_pose_mean.v[0] = last_published_pose.pose.pose.position.x;
	pf_init_pose_mean.v[1] = last_published_pose.pose.pose.position.y;
	pf_init_pose_mean.v[2] = tf::getYaw(last_published_pose.pose.pose.orientation);
	pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
	pf_init_pose_cov.m[0][0] = last_published_pose.pose.covariance[6*0+0];
	pf_init_pose_cov.m[1][1] = last_published_pose.pose.covariance[6*1+1];
	pf_init_pose_cov.m[2][2] = last_published_pose.pose.covariance[6*5+5];
	pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
	pf_init_ = false;

	// Instantiate Odometry
	delete odom_;
	odom_ = new AMCLOdom();
	if(odom_model_type_ == ODOM_MODEL_OMNI)
		odom_->SetModelOmni(alpha1_, alpha2_, alpha3_, alpha4_, alpha5_);
	else
		odom_->SetModelDiff(alpha1_, alpha2_, alpha3_, alpha4_);

	// Instantiate Laser
	delete laser_;
	laser_ = new AMCLLaser(max_beams_, map_);
	if(laser_model_type_ == LASER_MODEL_BEAM)
		laser_->SetModelBeam(z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_, 0.0);
	else  {
		printf("Initializing likelihood field model; this can take some time on large maps...");
		laser_->SetModelLikelihoodField(z_hit_, z_rand_, sigma_hit_, laser_likelihood_max_dist_);
		printf("Done initializing likelihood field model.");
	}

	/*
	 * ********************* ROS specific ************************************
	 */
	//  transform_tolerance_.fromSec(config.transform_tolerance);
	//  odom_frame_id_ = config.odom_frame_id;
	//  base_frame_id_ = config.base_frame_id;
	//  global_frame_id_ = config.global_frame_id;
	//  delete laser_scan_filter_;
	//  laser_scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_, *tf_, odom_frame_id_, 100);
	//  laser_scan_filter_->registerCallback(boost::bind(&AmclNode::laserReceived,  this, _1));
	//  initial_pose_sub_ = nh_.subscribe("initialpose", 2, &AmclNode::initialPoseReceived, this);
	/* *********************************************************************** */
}



/*
 * ************************************************************************
 * ******************** memory management functions ***********************
 * ************************************************************************
 */
/*
 * memory management functions
 */
void AMCLocalizer::freeMapDependentMemory() {
	if( map_ != NULL ) {
		map_free( map_ );
		map_ = NULL;
	}
	if( pf_ != NULL ) {
		pf_free( pf_ );
		pf_ = NULL;
	}
	delete odom_;
	odom_ = NULL;
	delete laser_;
	laser_ = NULL;
}


/*
 * ************************************************************************
 * ********************** map management functions ************************
 * ************************************************************************
 */
/**
 * Convert an OccupancyGrid map message into the internal representation.
 */
void AMCLocalizer::initMap( const nav_msgs::OccupancyGrid& map_msg ) {
	/*
	 * Convert an OccupancyGrid map message into the internal representation.
	 */
	std::cout << "1" << std::endl;
	freeMapDependentMemory();
	//  Clear queued laser objects because they hold pointers to the existing map, #5202.
	//  lasers_.clear();
	//  lasers_update_.clear();
	std::cout << "2" << std::endl;
	map_t* map = map_alloc();
	//	ROS_ASSERT(map);
	std::cout << "3" << std::endl;
	map->size_x = map_msg.info.width;
	map->size_y = map_msg.info.height;
	map->scale = map_msg.info.resolution;
	map->origin_x = map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
	map->origin_y = map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;
	// Convert to player format
	std::cout << "4" << std::endl;
	map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
	//ROS_ASSERT(map->cells);
	std::cout << "5" << std::endl;
	for(int i=0;i<map->size_x * map->size_y;i++) {
		if(map_msg.data[i] == 0)
			map->cells[i].occ_state = -1;
		else if(map_msg.data[i] == 100)
			map->cells[i].occ_state = +1;
		else
			map->cells[i].occ_state = 0;
	}
	std::cout << "6" << std::endl;
	first_map_received_ = true;

	/*
	 * Initialize the particle filter
	 */
#if NEW_UNIFORM_SAMPLING
	// Index of free space
	std::cout << "7" << std::endl;
	free_space_indices.resize(0);
	std::cout << "8" << std::endl;
	for(int i = 0; i < map_->size_x; i++)
		for(int j = 0; j < map_->size_y; j++)
			if(map_->cells[MAP_INDEX(map_,i,j)].occ_state == -1)
				free_space_indices.push_back(std::make_pair(i,j));
#endif
	std::cout << "9" << std::endl;
	//	  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);


	pf_ = pf_alloc(min_particles_, max_particles_, alpha_slow_, alpha_fast_,
			(pf_init_model_fn_t)AMCLocalizer::uniformPoseGenerator,
			(void *)map_);
	std::cout << "10" << std::endl;
	pf_->pop_err = pf_err_;
	pf_->pop_z = pf_z_;

	pf_vector_t pf_init_pose_mean = pf_vector_zero();
	pf_init_pose_mean.v[0] = init_pose_[0];
	pf_init_pose_mean.v[1] = init_pose_[1];
	pf_init_pose_mean.v[2] = init_pose_[2];
	pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
	pf_init_pose_cov.m[0][0] = init_cov_[0];
	pf_init_pose_cov.m[1][1] = init_cov_[1];
	pf_init_pose_cov.m[2][2] = init_cov_[2];
	pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
	pf_init_ = false;

	/*
	 * Instantiate the sensor objects: Odometry
	 */
	delete odom_;
	odom_ = new AMCLOdom();
	//  ROS_ASSERT(odom_);
	if(odom_model_type_ == ODOM_MODEL_OMNI)
		odom_->SetModelOmni(alpha1_, alpha2_, alpha3_, alpha4_, alpha5_);
	else
		odom_->SetModelDiff(alpha1_, alpha2_, alpha3_, alpha4_);

	/*
	 * Instantiate the sensor objects: Laser
	 */
	delete laser_;
	laser_ = new AMCLLaser(max_beams_, map_);
	//  ROS_ASSERT(laser_);
	if(laser_model_type_ == LASER_MODEL_BEAM)
		laser_->SetModelBeam(z_hit_, z_short_, z_max_, z_rand_,
				sigma_hit_, lambda_short_, 0.0);
	else {
		laser_->SetModelLikelihoodField(z_hit_, z_rand_, sigma_hit_,
				laser_likelihood_max_dist_);
	}
	// In case the initial pose message arrived before the first map,
	// try to apply the initial pose now that the map has arrived.
	applyInitialPose();
}



double AMCLocalizer::getYaw(tf::Pose& t) {
	double yaw, pitch, roll;
	t.getBasis().getEulerYPR(yaw,pitch,roll);
	return yaw;
}


/**
 * If initial_pose_hyp_ and map_ are both non-null, apply the initial
 * pose to the particle filter state.  initial_pose_hyp_ is deleted
 * and set to NULL after it is used.
 */
void AMCLocalizer::applyInitialPose() {
	//  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);
	if( initial_pose_hyp_ != NULL && map_ != NULL ) {
		pf_init(pf_, initial_pose_hyp_->pf_pose_mean, initial_pose_hyp_->pf_pose_cov);
		pf_init_ = false;

		delete initial_pose_hyp_;
		initial_pose_hyp_ = NULL;
	}
}

/*
 * This method accepts only initial pose estimates in the global frame, #5148.
 *
 * ************** N.B. This method has been rewritten by DB *****************
 */
void AMCLocalizer::initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
	tf::Pose pose_new;
	tf::poseMsgToTF(msg->pose.pose, pose_new);

	// Re-initialize the filter
	pf_vector_t pf_init_pose_mean = pf_vector_zero();
	pf_init_pose_mean.v[0] = pose_new.getOrigin().x();
	pf_init_pose_mean.v[1] = pose_new.getOrigin().y();
	pf_init_pose_mean.v[2] = getYaw(pose_new);
	pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
	// Copy in the covariance, converting from 6-D to 3-D
	for(int i=0; i<2; i++)
		for(int j=0; j<2; j++)
			pf_init_pose_cov.m[i][j] = msg->pose.covariance[6*i+j];

	pf_init_pose_cov.m[2][2] = msg->pose.covariance[6*5+5];

	delete initial_pose_hyp_;
	initial_pose_hyp_ = new amcl_hyp_t();
	initial_pose_hyp_->pf_pose_mean = pf_init_pose_mean;
	initial_pose_hyp_->pf_pose_cov = pf_init_pose_cov;

	applyInitialPose();
}




pf_vector_t AMCLocalizer::uniformPoseGenerator(void* arg) {
	map_t* map = (map_t*)arg;
#if NEW_UNIFORM_SAMPLING
	unsigned int rand_index = drand48() * free_space_indices.size();
	std::pair<int,int> free_point = free_space_indices[rand_index];
	pf_vector_t p;
	p.v[0] = MAP_WXGX(map, free_point.first);
	p.v[1] = MAP_WYGY(map, free_point.second);
	p.v[2] = drand48() * 2 * M_PI - M_PI;
#else
	double min_x, max_x, min_y, max_y;

	min_x = (map->size_x * map->scale)/2.0 - map->origin_x;
	max_x = (map->size_x * map->scale)/2.0 + map->origin_x;
	min_y = (map->size_y * map->scale)/2.0 - map->origin_y;
	max_y = (map->size_y * map->scale)/2.0 + map->origin_y;

	pf_vector_t p;

	ROS_DEBUG("Generating new uniform sample");
	for(;;)
	{
		p.v[0] = min_x + drand48() * (max_x - min_x);
		p.v[1] = min_y + drand48() * (max_y - min_y);
		p.v[2] = drand48() * 2 * M_PI - M_PI;
		// Check that it's a free cell
		int i,j;
		i = MAP_GXWX(map, p.v[0]);
		j = MAP_GYWY(map, p.v[1]);
		if(MAP_VALID(map,i,j) && (map->cells[MAP_INDEX(map,i,j)].occ_state == -1))
			break;
	}
#endif
	return p;
}

