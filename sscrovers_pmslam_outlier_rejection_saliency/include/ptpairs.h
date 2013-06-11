#ifndef PTPAIRS_H
#define PTPAIRS_H

#include <vector>
#include "ros/ros.h"

struct ptpair{
	int i;
	int j;
	float d;
	int frame;
};


class ptpairs{

	public:
		std::vector <ptpair> pairs;
		
		ptpairs(){
		};

		void push_back(ptpair u){
			pairs.push_back(u);
		}
 
		ptpairs(int ii){
			for (int i=0;i<ii;i++){
				ptpair p;
				p.i = i;
				p.j = i;
				p.d = 0;
				pairs.push_back(p);
			}
		};

		void add (int jIn, float dIn){
			int i = pairs.size();
			bool test = true;
			for (int w=0; w<i; w++){
				if (pairs.at(w).j == jIn){
					if(pairs.at(w).d > dIn){
						ptpair p;
						p.i = i;
						p.j = jIn;
						p.d = dIn;
						pairs.push_back(p);
						pairs.at(w).d = 0;
						pairs.at(w).j = w;
					}else{
						ptpair p;
						p.i = i;
						p.j = w;
						p.d = 0;
						pairs.push_back(p);
					}
					test = false;
					break;
				}
			}
			if (test){
				ptpair p;
				p.i = i;
				p.j = jIn;
				p.d = dIn;
				pairs.push_back(p);
			}
		}

		void add (int jIn, float dIn, int frameIn){
			int i = pairs.size();
			bool test = true;
			for (int w=0; w<i; w++){
				if (pairs.at(w).j == jIn  && pairs.at(w).frame == frameIn){
					if(pairs.at(w).d > dIn){

						ptpair p;
						p.i = i;
						p.j = jIn;
						p.d = dIn;
						p.frame = frameIn;
						pairs.push_back(p);
						pairs[w].d = 0;
						pairs[w].j = w;
						pairs[w].frame = 0;
					}else{

						ptpair p;
						p.i = i;
						p.j = i;
						p.d = 0;
						p.frame = 0;
						pairs.push_back(p);
					}
					test = false;
					break;
				}
			}
			if (test){
				ptpair p;
				p.i = i;
				p.j = jIn;
				p.d = dIn;
				p.frame = frameIn;
				pairs.push_back(p);
			}
		}	
		
		int size(){
			return pairs.size();
		}

		int atJ(int i){
			return pairs.at(i).j;
		}
		ptpair at(int i){
			return pairs.at(i);
		}
		float atD(int i){
			return pairs.at(i).d;
		}
		int atFrame(int i){
			return pairs.at(i).frame;
		}
		
		std::vector <int> nToNMinusOne(){
			std::vector<int> ret;
			for (int i=0; i<size();i++){
				ret.push_back(pairs.at(i).j);
			}
			return ret;
		}



		std::vector<ptpair> bubblesort(std::vector<ptpair> w){ 
			ptpair temp;
			bool finished = false;
			 while (!finished) 
			{ 
				finished = true;
				for (int i = 0; i < w.size()-1; i++) { 
					if (w[i].j > w[i+1].j) { 
						temp = w[i];
						 w[i] = w[i+1];
						 w[i+1] = temp;
						 finished=false; 
					} 
				} 
			} 
			return w; 
		}

		std::vector <int> NMinusOneToN(){
			std::vector<int> ret;
			std::vector<ptpair> pp = bubblesort(pairs);
			for (int i=0; i<size();i++){
				ret.push_back(pp.at(i).i);
			}
			return ret;
		}

};

#endif
