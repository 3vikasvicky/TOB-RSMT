#include "OARST.h"


//generate timing constrain
void OARSteinerTree::GenTimingConstrain(char* filename,Database& database,float timing_constrain_mul,elmoreDelay* elmoreModel)
{	 

	FILE *fp;
	cout<<"Write file = "<<filename<<endl;
	fp = fopen(filename,"w");
	
	int minX=0,minY=0,maxX=0,maxY=0,topLayer=_UNSET,buttomLayer=_UNSET; //coordination boundary
	int numPin=0;
	int numObstacle=0;


	if(fp==NULL) //Check file is available or not
	{	cout<<"Can not write file!\n";
		exit(0);
	}
	else
	{	
		fprintf(fp,"Net Example\n");
		//fprintf(fp,"UnitCap: %lf\n",elmoreModel->get_Unit_C()/1000);
		fprintf(fp,"UnitCap: 0.000118\n");		
		//fprintf(fp,"UnitRes: %lf\n",elmoreModel->get_Unit_R()/1000);
		fprintf(fp,"UnitRes: 0.000076\n");
		fprintf(fp,"Number Sinks %d\n",database.getPinNum()-1);

		//wirte driving node first
		RGNode* driver = this->getOARG()->getDrivingNode();
		fprintf(fp,"Source 0\n");
		fprintf(fp,"  Location (%d, %d)\n",driver->getX(),driver->getY());
		fprintf(fp,"  Resistance %lf\n",elmoreModel->get_Rd()/1000);
		fprintf(fp,"  Arr %lf\n",elmoreModel->get_Driver_Arrival_Time()/1000);

		int count = 0;
		for(int i=0;i<this->getOARG()->getNumNode();i++)
		{	
			RGNode* N = this->getOARG()->getNode(i);

			if(N->getType()==_PIN && N!=getOARG()->getDrivingNode())
			{	count ++ ;
				fprintf(fp,"Sink %d\n",count);
				fprintf(fp,"  Location (%d,%d)\n",N->getX(),N->getY());
				fprintf(fp,"  Capacitance %lf\n",N->getLoading()/1000);				
				fprintf(fp,"  Req %lf\n",(N->getDelay()/1000)*timing_constrain_mul);
				fprintf(fp,"  Polarity 0\n");
			}
		}

		printf("# of Sink = %d\n",count);

		//wirte obstacles
		count = 0;
		fprintf(fp,"%d\n",database.getObstacleNum());
		for(int i=0;i<database.getObstacleNum();i++)
		{	Obstacle* O = database.GetObstacle(i);
			fprintf(fp,"%d %d %d %d\n",O->getX1(),O->getY1(),O->getX2(),O->getY2());
			count ++ ;
		}

		printf("# of Obstacle = %d\n",count);
	}
}