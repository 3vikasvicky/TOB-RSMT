#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include "database.h"
#include "elmoreDelay.h"


using namespace std;


//[fscanf] : Read formatted data from stream
//			 Using this function call would be a great help to coding!!!
void readfile(char* filename,Database& database,elmoreDelay* elmoreModel)
{
	FILE *fp;
	fp = fopen(filename,"r");
	
	int minX=0,minY=0,maxX=0,maxY=0,topLayer=3,buttomLayer=3; //coordination boundary
	int numPin=0;
	int numObstacle=0;

	double unit_c = 0;        // new unit_c
	double unit_r = 0;		  // new unit_r
	double Rd = 0;            // new Rd
	int x=0,y=0,layer=3;	  //default layer = 1
	double req_time=0;
	int sinkNum = 0;
	double sink_loading=1;    //default 1 Ff
	double driver_arrival_time=0;    //default 0 ps
	char Line[80];


	if(fp==NULL) //Check file is available or not
	{	cout<<"Can not open file!\n";
		exit(0);
	}
	else
	{
		//=================================================================
		fgets(Line,80,fp); //skip net name			
		fscanf(fp,"UnitCap: %lf\n",&unit_c);
		fscanf(fp,"UnitRes: %lf\n",&unit_r);
		fscanf(fp,"Number Sinks %d\n",&sinkNum); //read # of sinks
		unit_c = unit_c*1000;
		unit_r = unit_r*1000;		
		elmoreModel->set_Unit_C(unit_c);
		elmoreModel->set_Unit_R(unit_r);

		//printf("%s",Line);	
		//printf("UnitCap: %lf (Ff)\n",unit_c);
		//printf("UnitRes: %lf (ohms)\n",unit_r);		
		printf("# of Sink = %d (not include driver)\n",sinkNum);
		numPin = sinkNum + 1; //sink + driver = all the pins

		//=================================================================
		fgets(Line,80,fp);									//skip Source 0
		fscanf(fp,"  Location (%d, %d)\n",&x,&y);			//Read (X,Y)
		fscanf(fp,"  Resistance %lf\n",&Rd);				//Read Resistance
		fscanf(fp,"  Arr %lf\n",&driver_arrival_time);      //Read Arrrival Time of Driver
		driver_arrival_time = driver_arrival_time * 1000;   //Arrival time for the signal at source pin (unit: 10^3 ps) 
		Rd = Rd * 1000; //Driver resistance at the source pin (unit: 10^3 Ohm) 

		//Use Our Own Setting
		elmoreModel->set_Driver_Arrival_Time(driver_arrival_time); // Homer: evansin marked it originally
		elmoreModel->set_Rd(Rd);  // Homer: evansin marked it originally

		//printf("Rd = %lf (ps)\n",Rd);		
		//printf("%s",Line);
		
		//=================================================================

		//database.AddPin(X(x),Y(y),Layer(layer),req_time,sink_loading); //Add driver into database
		database.AddPin(X(x),Y(y),Layer(layer),req_time,elmoreModel->get_Loading()); //Add driver into database
		database.setDriver(database.GetPin(0));

		printf("Driver %d %d %d %f\n",database.getDriver()->getX(),database.getDriver()->getY(),database.getDriver()->getLayer(),database.getDriver()->getRequiredTime());

		//Compared with minX ....... in order to record the max and min x y
		if(x<minX) { minX=x; }
		if(x>maxX) { maxX=x; }
		if(y<minY) { minY=y; }
		if(y>maxY) { maxY=y; }

		for(int i=0;i<sinkNum;i++)
		{	
			if(fp==NULL) {
				cout<<"Specified pin number dosen't correspond with the actual pin number!\n";
				exit(0);
			}

			//=================================================================
			fgets(Line,80,fp);                                     //skip Sink X
			fscanf(fp,"  Location (%d, %d)\n",&x,&y);              //Read (X,Y)
			fscanf(fp,"  Capacitance %lf\n",&sink_loading);        //Read Cap
			fscanf(fp,"  Req %lf\n",&req_time);                    //Read Req Time

			sink_loading = sink_loading * 1000; //Load capacitance at a sink pin (unit: 10^3 fF) 
			//sink_loading = 0.5; //Load capacitance at a sink pin (unit: 10^3 fF) 
			req_time = req_time * 1000; //Required arrival time for the signal at a sink pin (unit: 10^3 ps) 
			//printf("%s",Line);
			//printf("  Location (%d, %d) Cloading = %lf (Ff)\n",x,y,sink_loading);		
			//printf("  Req %lf (ps)\n",req_time); 
			fgets(Line,80,fp);                                     //skip Polarity 0
			//=================================================================			
			
			//database.AddPin(X(x),Y(y),Layer(layer),req_time,sink_loading); //Add Pin into database
			database.AddPin(X(x),Y(y),Layer(layer),req_time,elmoreModel->get_Loading()); //Add Pin into database


			//Compared with minX ....... in order to record the max and min x y
			if(x<minX) { minX=x; }
			if(x>maxX) { maxX=x; }
			if(y<minY) { minY=y; }
			if(y>maxY) { maxY=y; }
		} 

		//=================================================================
		fscanf(fp,"%d\n",&numObstacle);
		printf("%d\n",numObstacle);

		int x1=0,y1=0,x2=0,y2=0;
		for(int i=0;i<numObstacle;i++)
		{	
			if(fp==NULL) {
				cout<<"Specified obstacle number dosen't correspond with the actual obstacle number!\n";
				exit(0); 
			}
		
			fscanf(fp,"%d %d %d %d\n",&x1,&y1,&x2,&y2); //Read Obstacle information
			//printf("%d %d %d %d\n",x1,y1,x2,y2);
			database.AddObstacle(X1(x1),Y1(y1),X2(x2),Y2(y2),Layer(layer)); //Add Obstacle into database

			//Compared with minX ....... in order to record the max and min x y
			if(x1<minX) { minX=x1; }
			if(x2>maxX) { maxX=x2; }
			if(y1<minY) { minY=y1; }
			if(y2>maxY) { maxY=y2; }
		}
	}


	cout<<"\n--[Read Input File]\n";
	cout<<"# of Obstacles   = "<<numObstacle<<endl;
	cout<<"# of Pins        = "<<numPin<<endl;
	cout<<"(MinX,MinY) = ("<<minX<<","<<minY<<")\n";
	cout<<"(MaxX,MaxY) = ("<<maxX<<","<<maxY<<")\n";
	cout<<"Buttom Layer Num = "<<buttomLayer<<"\n";
	cout<<"Top    Layer Num = "<<topLayer<<"\n";

	database.setObstacleNum(numObstacle);
	database.setPinNum(numPin);
	database.setMinX(minX);
	database.setMinY(minY);
	database.setMaxX(maxX);
	database.setMaxY(maxY);
	database.setButtomLayerNum(buttomLayer);
	database.setTopLayerNum(topLayer);

	//compute length of horizontal section and vertical section
	int numUnitX=(maxX-minX)/10;
	int numUnitY=(maxY-minY)/10;
	
	int secXLength=10;
	int secYLength=10;

	while(numUnitX>100) //too many section
	{	numUnitX=numUnitX/10;		//reduce it
		secXLength=secXLength*10;
	}
	while(numUnitY>100) //too many section
	{	numUnitY=numUnitY/10;		//reduce it
		secYLength=secYLength*10;
	}		

	database.setSecXLength(min(secXLength,secYLength));
	database.setSecYLength(min(secXLength,secYLength));

	cout<<"W of Grid = "<<min(secXLength,secYLength)<<" (just for displaying usage)"<<endl;
	cout<<"L of Grid = "<<min(secXLength,secYLength)<<" (just for displaying usage)"<<endl;


	elmoreModel->show_setting();
}

#ifdef BUF_DRIVEN
void readBufFile(char* filename,Database& database,elmoreDelay* elmoreModel)
{
  FILE *fp;
  double UB(0);
  fp = fopen(filename,"r");
  if(fp==NULL) //Check file is available or not
  {	
    cout<<"Can not open file!\n";
    exit(0);
  }
  else
  {
    //=================================================================
    fscanf(fp,"Capacitance Upper Bound: %lf\n",&UB);
    printf("Capacitance Upper Bound is %lf\n",UB);
    database.setCapUpperBound(UB);
    elmoreModel->setCapUpperBound(UB);
    return;
  }
}
#endif

void set_driver_arr_0(Database& database,elmoreDelay* elmoreModel)
{
	for(size_t i=0;i<database.GetPinSize();i++)
	{	
		Pin* P = database.GetPin(i);
		
		if(P != database.getDriver()) //bypass driving node
		{	P->setRequiredTime( P->getRequiredTime() - elmoreModel->get_Driver_Arrival_Time());
		}
	}

	elmoreModel->set_Driver_Arrival_Time(0);
}
