#ifndef SALPOINT_H
#define SALPOINT_H
#include <math.h> 

class SALPoint{

	public:
		float x;
		float y;
		float width;
		float height;
		int n, id, flag, step;
		
		SALPoint(){
			x=-1;
			y=-1;
			width=0;
			height=0;
			n = 0;
			id = 0;
			step = 0;
			flag = 0;
		}
		SALPoint(float xin, float yin, float win, float hin){
			x=xin;
			y=yin;
			width=win;
			height=hin;
		}
		SALPoint(float xin, float yin, float win, float hin, int nIn, int idIn, int stepIn, int flagIn){
			x=xin;
			y=yin;
			width=win;
			height=hin;
			n = nIn;
			id = idIn;
			step = stepIn;
			flag = flagIn;
		}
		
		float dist(SALPoint s){
			return sqrt((x-s.x)*(x-s.x)+(y-s.y)*(y-s.y));
		}

};
#endif
