#ifndef SALPOINTVEC_H
#define SALPOINTVEC_H

#include <vector>
#include "RoverState.h"
#include "SALPoint.h"
#include<ctime>


class SALPointVec{

	public:
		RoverState state;
		std::time_t moment;
		std::vector<SALPoint> points;
		
		SALPointVec(){


		};
		
		SALPointVec(RoverState sts, std::time_t t){
			moment = t;
			state = sts;
		}
		SALPointVec(RoverState sts, std::time_t t, std::vector<SALPoint> pointsIn){
			moment = t;
			state = sts;
			points = pointsIn;
		}
		

		void push_back(SALPoint s){
			points.push_back(s);
		}

		int size(){
			return points.size();
		}
		
		SALPoint at(size_t i){
			return points.at(i);
		}
			
		float atX(size_t i ){
			return points.at(i).x;
		}

		float atY(size_t i ){
			return points.at(i).y;
		}

		float atH(size_t i ){
			return points.at(i).height;
		}
		float atW(size_t i ){
			return points.at(i).width;
		}		

};


#endif
