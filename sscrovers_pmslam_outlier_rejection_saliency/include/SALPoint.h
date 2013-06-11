#ifndef SALPOINT_H
#define SALPOINT_H


class SALPoint{

	public:
		float x;
		float y;
		float width;
		float height;
		
		SALPoint(){
			x=-1;
			y=-1;
			width=0;
			height=0;
		}
		SALPoint(float xin, float yin, float win, float hin){
			x=xin;
			y=yin;
			width=win;
			height=hin;
		}
		
		float dist(SALPoint s){
			return (x-s.x)*(x-s.x)+(y-s.y)*(y-s.y);
		}

};
#endif
