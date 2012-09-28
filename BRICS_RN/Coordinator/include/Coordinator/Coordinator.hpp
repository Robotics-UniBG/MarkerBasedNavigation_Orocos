/*
 * CoordinatorMapAndMarkers.hpp
 *
 *  Created on: Jun 11, 2012
 *      Author: andrea
 */

#ifndef COORDINATORMARKERS_HPP
#define COORDINATORMARKERS_HPP
#include "string"
#include "geometry_msgs/PoseStamped.h"
#include "mbn_common/Events.hpp"

/**
 * String representing a null event.
 */
const std::string NULL_EVENT = "nullevent";

/**
 * String representing a null marker ID.
 */
const int NULLMARKER_ID = -2;

namespace Navigation {
class Coordinator {
private:

	//Coordinator states
	typedef enum {INIT = 0, IDLE = 1, SEARCH = 2, MARKER_BASED_NAV = 3} CoordinatorMarkerState;
	CoordinatorMarkerState coordinatorState;
	void setCoordinatorState(CoordinatorMarkerState state);

public:
	/**
	 * Class constructor.
	 */
	Coordinator();

	/**
	 * Class distructor.
	 */
	~Coordinator();

	/**
	 * Starts the coordinator changing the internal state to INIT.
	 */
	void startCoordinator();

	/**
	 * \param[in] goal the input goal. Set to NULL_EVENT constant if no new goal is available.
	 * \param[in] event the input event. Set to NULL_EVENT constant if no new event is available.
	 * \param[out] outID output goal ID. This parameter is set to NULLMARKER_ID if no goal should be output in this step.
	 * \param[ouy] outCommand output command string. It is set to NULL_EVENT if no output commands should be output in this step.
	 *
	 * Trigger the state machine and eventually updates the internal state according to the inputs. New outputs are
	 * computed every step.
	 */
	void computeStateMachine(std::string goal, std::string event, int& outID, std::string& outCommand);

	/**
	 * \return the string representing the current coordinator state.
	 */
	std::string coordinatorStateToString();

	/**
	 * the ID of the homing marker.
	 */
	std::string homingMarkerID;

	/**
	 * string representing the coordinator state.
	 */
	std::string coordinatorStateString;

	/**
	 * flag, true if in homing phase.
	 */
	bool homing;



};

} /* namespace navigation */
#endif /* COORDINATORMARKERS_HPP_ */
