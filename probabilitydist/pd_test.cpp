
#include "probabilitydist.h"

int main(int argc, char *argv[])
{
	ProbabilityDist pd(2,2,6,6);


	pd.SampleUpdate(2,2,true);
	for (int i=0;i<6;i++) {
		for(int j=0;j<6;j++) {
			if(!(i ==2 && j ==2)){
				pd.SampleUpdate(i,j, false);
			}
		}
	}

	pd.MotionUpdate(3,3);

	pd.SampleUpdate(3,3,true);
	for (int i=0;i<6;i++) {
		for(int j=0;j<6;j++) {
			if(!(i ==3 && j ==3)){
				pd.SampleUpdate(i,j, false);
			}
		}
	}

	pd.MotionUpdate(4,4);

	pd.SampleUpdate(4,4,true);
	for (int i=0;i<6;i++) {
		for(int j=0;j<6;j++) {
			if(!(i ==4 && j ==4)){
				pd.SampleUpdate(i,j, false);
			}
		}
	}

	pd.Normalize();
	pd.OutputDist();

	pd.EstimatePosition();
}
