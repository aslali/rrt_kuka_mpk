// *************************************************************
// RRT Path rrtPlanner
// *************************************************************

#ifndef rrt_PLANNER_H
#define rrt_PLANNER_H




#include <stdlib.h>
#include <queue>
#include <vector>
#include <list>

#include "mpkSimpleSegmentChecker.H"
#include "sblRn.H"





class rrtPlanner {

public:

	/**@doc Takes a pointer {\bf robots} to the robot collection, a set
	of collision test pairs (see
	{@link mpkCollPairSet mpkCollPairSet})
	and a c-space resolution for the
	{@link mpkSimpleSegmentChecker mpkSimpleSegmentChecker}.
	*/
	rrtPlanner(mpkRobotCollection* robots, vector<mpkCollPair> *test_pairs, double epsilon = 0.012);

	virtual ~rrtPlanner() {
		delete point_checker;
	};

	/**@doc This function is used to invoke the planner.  {\bf rho} is
	the initial neighborhood size for sampling around a milestone. */
	bool Query(const mpkConfig& q0, const mpkConfig& q1, list<mpkConfig> &genPath, double rho = 0.15, int max_it = 100000, double dg = 0.385,double id=0.2);

	/**@doc Writes the final path to a file */
	void writePath(const list<mpkConfig>&, char *file_name);

	int iter;



protected:

	//void randConfig(mpkConfig& q);
	bool edgeCollCheck(const mpkConfig& prevState, const mpkConfig& qn, double minDist);

	/* To store the first path found */
	list<mpkConfig> genPath;

	vector<int> sequen;

	/* For Randon configuration generation */
	sblRn randConfig;

	vector<mpkCollPair> *test_pairs_;

	mpkConfigChecker *point_checker;

	double EPSILON;
	mpkRobotCollection *robots_;

};

#endif