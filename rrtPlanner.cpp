// **************************************************************
// RRT Path rrtPlanner implementation.
// **************************************************************

#ifndef CLK_TCK
#define CLK_TCK CLOCKS_PER_SEC
#endif

#include <stdio.h>
#include <time.h>
#ifndef WIN32
#include <algo.h>
#endif

#include "mpk_rand.h"
#include "rrtPlanner.H"



// Class variable initializations

// ------------------------------
// Constructor & Destructor
// ------------------------------
rrtPlanner::
rrtPlanner(mpkRobotCollection* robots, vector<mpkCollPair> *test_pairs, double epsilon)
{
	test_pairs_ = test_pairs;
	robots_ = robots;

	genPath.clear();


	point_checker = new mpkConfigChecker(test_pairs, robots);
	EPSILON = epsilon; // Epsilon is the resolution to which a segment is checked
}


// ----------------------------------------------------------------
// Query: receive the start and goal configurations (qs and qg) and
//        returns the path (genPath) if found. rho = neighborhood to
//        sample, max_it = max number of iterations allowed
// ----------------------------------------------------------------

bool rrtPlanner::Query(const mpkConfig& qs, const mpkConfig& qg, list<mpkConfig> &freePath, double rho, int max_it, double dg,double id)
{
	bool ifail = edgeCollCheck(qs, qg, qs.dist(qg));
	if (ifail == 0){
		freePath.push_back(qs);
		freePath.push_back(qg);
		return true;
	}
	if (point_checker->collision(&qs)){
		cerr << "Initial configuration in collision!" << endl;
		return false;
	}

	if (point_checker->collision(&qg)){
		cerr << "Goal configuration in collision!" << endl;
		return false;
	}

	int iter = 1;
	genPath.push_back(qs);
	sequen.push_back(-1);
	mpkConfig qRand;
	mpkConfig qNew;
	double minDist;
	int indMin;
	mpkConfig prevState;
	double a;
	double valn;
	double dist;
	bool fail1;
	srand(time(NULL));
	//bool fail;

	for (int j = 0; j < qs.size(); j++){
		qRand.push_back(qs[j]);
	}

	while (iter <= max_it){
		//randConfig.UniformSample(&qRand);
		//randConfig(qRand);
		randConfig.LocalBoxSample(qs,1,&qRand);
		double diiist =qs.dist(qRand);
		bool fail = point_checker->collision(&qRand);
		if (fail == 1){
			iter++;
		}

		if (fail == 0){

			list <mpkConfig>::iterator j;
			int jj = 0;

			for (j = genPath.begin(); j != genPath.end(); ++j){
				dist = (*j).dist(qRand);
				if (jj == 0 || dist < minDist){
					minDist = dist;
					indMin = jj;
					prevState = *j;
				}
				jj++;
			}
			if (minDist>id){
				a = id / minDist;
				for (int j = 0; j < qRand.size(); j++){
					valn = (prevState[j])*(1 - a) + (qRand[j]) * a;
					qNew.push_back(valn);
				}
				minDist = (prevState).dist(qNew);
			}
			else
				qNew = qRand;

			fail = edgeCollCheck(prevState, qNew, minDist);
			if (fail == 0){
				genPath.push_back(qNew);
				sequen.push_back(indMin);

				double distg = qNew.dist(qg);
				if (distg < dg){
					fail1 = edgeCollCheck(qg, qNew, distg);
					if (fail1 == 0){
						genPath.push_back(qg);
						int ind = genPath.size()-2;
						sequen.push_back(ind);
						freePath.push_front(qg);
						
						while (true){
							list <mpkConfig>::iterator ith_iter = genPath.begin();
							advance(ith_iter, ind);
							freePath.push_front(*ith_iter);
							ind = sequen[ind];
							if (ind == -1) return true;
						}
					}
				}

			}
			if (fail == 1){
				iter++;
			}

		}
		qNew.clear();
	}
	return false;



	if (iter > max_it){
		cerr << "Sorry, couldn't find a path" << endl;
		return false;
	}

}


bool rrtPlanner::edgeCollCheck(const mpkConfig& prevState, const mpkConfig& qNew, double minDist){
	int m = minDist / EPSILON;
	mpkConfig ql;
	bool fail = false;
	int i = 1;
	while (i < m && !fail){
		double val;
		double t;
		ql.clear();
		t = double(i) / m;
		for (int j = 0; j < qNew.size(); j++){
			val = (prevState[j])*(1-t) + (qNew[j]) * t;
			ql.push_back(val);
		}
		fail = point_checker->collision(&ql);
		i++;
	}
	return fail;
}

/*void rrtPlanner::randConfig(mpkConfig& q){
	srand(time(NULL));
	for (int i = 0; i < q.size(); i++){
		q[i] = ((double)rand()) / ((double)RAND_MAX);
	}
}*/




// --------------------------------------------------------------
// writePath: Sends the path to a file
// --------------------------------------------------------------

void
rrtPlanner::
writePath(const list<mpkConfig>& path, char* name)
{
	list<mpkConfig>::const_iterator it;
	FILE *fp;

	fp = fopen(name, "wt");

	if (fp == NULL) {
		cerr << "Error in write_path when opening file" << endl;
		return;
	}

	for (it = path.begin(); it != path.end(); it++) {
		const mpkConfig& q = (*it);
		int dim = q.size();
		for (int i = 0; i < dim; i++)       fprintf(fp, "%f ", q[i]);
		fprintf(fp, "\n");
	}
	fclose(fp);
}