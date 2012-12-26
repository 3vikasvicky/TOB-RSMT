#include "OARST.h"

#ifdef XP_MODE
#include <windows.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif

#include <string.h>
#include <iostream>
#include <fstream>
#include <map>
#include <string>
#include <sstream>

//----------------Global Record--------------
Database database;	//a class to hold needful data for porcessing
OASpanningGraph* OASG; //a class to build spanning graph
OARSteinerTree* OARST; //a class to route obstacle avoiding rectilinear steiner tree

bool secondTime;
string argv1_path;
string argv4_slew;
double BEST_WNS = -1*(_INFINITE);//By yilin

RunTime* RunTime = database.getRunTimeSet();
elmoreDelay* elmoreModel = new elmoreDelay();
double Initial_WD = _UNSET;
double WD_When_Min_WNS = _UNSET;
double Min_WL = _INFINITE;
double Initial_WNS = _UNSET;
double Min_WNS = -1*(_INFINITE);
int    Min_WNS_In_Times = _UNSET;
double WL_When_Min_WNS = _UNSET;
double Initial_WL = _UNSET;
//-------------------------------------------

//----------------Display-Related--------------
int indexRoutingNet=0; //index for routing net by net
bool visible_layer[11]={0,0,0,1,0,0,0,0,0,0,0};
int menu,layer_menu;
float zoomX=1;
float zoomY=1;
float transferX=0;
float transferY=0;
bool fullscreen = 0; //set fullscreen or not
int Xpos = 0;    // record screen locates at where
int Ypos = 0;	 // record screen locates at where
int Xsize = 0;	 // width of screen
int Ysize = 0;   // height of screen

bool visible_SpanningEdge=true;
bool visible_Routed_SpanningEdge=true;
bool visible_H_V_Edge=true;
bool visible_Grids=true;
bool visible_Sink_Long_Radius=false;
bool visible_Sink_Bad_Delay=false;
bool visible_Critical_Path=false;
bool visible_Critical_Sink=false;
bool visible_Leaf_in_OASG=false;
bool visible_Nodes_In_Critical_Path=false;
bool visible_Positive_Slack_In_OARG=false;
//-------------------------------------------

//----------------Debug-Related--------------
SGNode* Source=NULL;	//for DEBUG : show Source on the canvas
SGNode* Target=NULL;  //for DEBUG : show Target on the canvas
vector<SGNode*> vMultiSource;  //for DEBUG : show Myulti-Sources on the canvas
//-------------------------------------------


#ifdef XP_MODE
void display(void)
{
	glClear(GL_COLOR_BUFFER_BIT); // clear the buffer background color
	
	glClearColor(1.0,1.0,1.0,1.0); 
	glPushMatrix();
    
	
	//transfer coordination
    glTranslatef(transferX,transferY,0);
	//zoom in or zoom out
    glScalef(zoomX,zoomY,1);

		
	//Drawing measure lines	
	glLineWidth(1);
/*
	float d=50;
	glColor3f(0.1, 0.1, 0.5);
	glBegin(GL_LINES);	
		glVertex3f(-d,0,0.0f);
		glVertex3f(d,0,0.0f);	
	glEnd();	
	glBegin(GL_LINES);	
		glVertex3f(0,-d,0.0f);
		glVertex3f(0,d,0.0f);	
	glEnd();
*/
	glLineWidth(1);
	float RoutedEdge[3]={1.0,0.0,1.0};
	float RGB[11][3]={{0.0,0.0,0.0}
					,{1.0,0.5,0.5}
					,{0.8,0.1,0.7}
					,{0.7,0.7,0.0}
					,{0.9,0.6,0.7}
					,{0.6,0.9,0.9}
					,{1.0,0.0,0.0}
					,{0.5,0.0,1.0}
					,{0.7,0.2,0.0}
					,{0.0,0.7,0.2}
					,{0.2,0.7,0.7}};

	//scan from buttom layer to top layer
	for(int numLayer=database.getButtomLayerNum();numLayer<=database.getTopLayerNum();numLayer++)
	{

		glLineWidth(1);

		if(visible_layer[numLayer]==true)
		{

			//Drawing Bounding Box
			//glColor3f(RGB[numLayer][0]-0.7,RGB[numLayer][1]-0.5,RGB[numLayer][2]-0.5);
			//glColor3f(1.0,1.0,1.0);
			//glBegin(GL_QUADS);
			glColor3f(0.0,0.0,0.0);
			glBegin(GL_LINE_LOOP);
			
				glVertex3f(database.getMinX(),database.getMinY(),0.0f);
				glVertex3f(database.getMaxX(),database.getMinY(),0.0f);
				glVertex3f(database.getMaxX(),database.getMaxY(),0.0f);
				glVertex3f(database.getMinX(),database.getMaxY(),0.0f);
			glEnd();


			//Drawing Obstacles
		    //glColor3f(RGB[numLayer][0]-0.3,RGB[numLayer][1]-0.3,RGB[numLayer][2]-0.3);	
			glColor3f(0.7,0.7,0.7);
			for(size_t i=0;i<database.GetObstacleSize();i++)
			{	
				Obstacle* O = database.GetObstacle(i);
				if(O->getLayer()==numLayer)
				{
					glBegin(GL_QUADS);	
						glVertex3f(O->getX1(),O->getY1(),0.0f);
						glVertex3f(O->getX2(),O->getY1(),0.0f);
						glVertex3f(O->getX2(),O->getY2(),0.0f);
						glVertex3f(O->getX1(),O->getY2(),0.0f);
					glEnd();		
				}
			}

			//Drawing Grids
			/*if(visible_Grids)
			{
				glColor3f(RGB[numLayer][0]-0.6,RGB[numLayer][1]-0.4,RGB[numLayer][2]-0.4);	
				int tempX=database.getMinX();
				int tempY=database.getMinY();
				while(tempX < database.getMaxX())
				{
					glBegin(GL_LINES);	
						glVertex3f(tempX,database.getMinY(),0.0f);
						glVertex3f(tempX,database.getMaxY(),0.0f);
					glEnd();
					tempX=tempX+database.getSecXLength();
				}
				while(tempY < database.getMaxY())
				{
					glBegin(GL_LINES);	
						glVertex3f(database.getMinX(),tempY,0.0f);
						glVertex3f(database.getMaxX(),tempY,0.0f);
					glEnd();
					tempY=tempY+database.getSecYLength();
				}
			}*/


			//Drawing SG Edges
			if(visible_SpanningEdge)
			{
				for(int i=0;i<OARST->getOASG()->getNumEdge();i++)
				{
					SGEdge* E = OARST->getOASG()->getEdge(i);
					SGNode* N = E->getFromNode();
					SGNode* ToNode=E->getToNode();	

					if(N->getLayer()==numLayer && ToNode->getLayer()==numLayer)
					{		
							//glLineWidth(1);
							glLineWidth(2);
							//glColor3f(0.7f,0.7f,0.5f);	
							glColor3f(0.0f,0.0f,0.0f);	
							//glColor3f(0.4,0.4,0.4);
							glBegin(GL_LINES);	
								glVertex3f(N->getX(),N->getY(),0.0f);
								glVertex3f(ToNode->getX(),ToNode->getY(),0.0f);
							glEnd();
					}
				}
			}
			if(visible_Routed_SpanningEdge)
			{
				for(int i=0;i<OARST->getOASG()->getNumEdge();i++)
				{
					SGEdge* E = OARST->getOASG()->getEdge(i);
					SGNode* N = E->getFromNode();
					SGNode* ToNode=E->getToNode();	

					if(E->checkRouted()==true && N->getLayer()==numLayer && ToNode->getLayer()==numLayer)
					{		
							glLineWidth(2);
							//glColor3f(RoutedEdge[0],RoutedEdge[1],RoutedEdge[2]);
							//glColor3f(0.0f,0.0f,0.0f);	
							//glColor3f(0.2,0.2,0.2);
							//glColor3f(0.4,0.4,0.4);
							glColor3f(0.0f,0.0f,0.0f);	
							glBegin(GL_LINES);	
								glVertex3f(N->getX(),N->getY(),0.0f);
								glVertex3f(ToNode->getX(),ToNode->getY(),0.0f);
							glEnd();
					}
				}
			}
	
			//Drawing RG Edges
			if(OARST->getOARG() != NULL && visible_H_V_Edge==true)
			{
				for(size_t i=0;i<OARST->getOARG()->getNumEdge();i++)
				{
					RGNode* FromNode = OARST->getOARG()->getEdge(i)->getFromNode();				
					RGNode* ToNode = OARST->getOARG()->getEdge(i)->getToNode();

					glLineWidth(2);
					//glColor3f(0,0.5,1.0);
					//glColor3f(0.0f,0.0f,0.0f);	
					//glColor3f(0.2,0.2,0.2);
					glColor3f(0.4,0.4,0.4);
					glBegin(GL_LINES);	
						glVertex3f(FromNode->getX(),FromNode->getY(),0.0f);
						glVertex3f(ToNode->getX(),ToNode->getY(),0.0f);
					glEnd();				
				}
			}

			//Drawing SG Edges (in critical path)
			if(visible_Critical_Path)
			{
				for(size_t i=0;i<OASG->getNumEdge();i++)
				{	
					SGEdge* E=OASG->getEdge(i);
					
					SGNode* N=E->getFromNode();
					SGNode* ToNode=E->getToNode();	

					if(N->getLayer()==numLayer && ToNode->getLayer()==numLayer)
					{
						if(E->checkInCriticalPath())
						{	//glLineWidth(2);
							glLineWidth(5);
							//glColor3f(1.0,0.0,0.0);
							//glColor3f(0.2,0.2,0.2);
							glColor3f(0.4,0.4,0.4);
							glBegin(GL_LINES);	
								glVertex3f(N->getX(),N->getY(),0.0f);
								glVertex3f(ToNode->getX(),ToNode->getY(),0.0f);
							glEnd();
						}
						//display branch edge connect to critical path
						else if(E->checkRouted() && (N->checkInCriticalPath() && !ToNode->checkInCriticalPath() || !N->checkInCriticalPath() && ToNode->checkInCriticalPath()))						
						{	glLineWidth(6);
							glColor3f(1.0,1.0,1.0);
							glBegin(GL_LINES);	
								glVertex3f(N->getX(),N->getY(),0.0f);
								glVertex3f(ToNode->getX(),ToNode->getY(),0.0f);
							glEnd();
						}
					}
				}	
			}

			//Drawing Pins
			//glColor3f(1.0f,1.0f,0.0f);
			glColor3f(0.0f,0.0f,0.0f);	
			for(size_t i=0;i<database.GetPinSize();i++)
			{	
				Pin *Pref = database.GetPin(i);
				Pin const P = *Pref;
				if(P.getLayer()==numLayer)
				{
					glPointSize(5);
					//glPointSize(8);
					//glPointSize(10);
					glBegin(GL_POINTS);
						glVertex3f(P.getX(),P.getY(),0.0f);
					glEnd();
				}
			}	


			if(visible_Leaf_in_OASG)
			{
				for(size_t i=0;i<OARST->getOASG()->getNumNode();i++)
				{	
					glColor3f(0.0f,1.0f,1.0f);
					SGNode* P = OARST->getOASG()->getNode(i);
					if(P->getLayer()==numLayer && P->checkIsLeaf())
					{
						glPointSize(8);
						glBegin(GL_POINTS);
							glVertex3f(P->getX(),P->getY(),0.0f);
						glEnd();
					}
				}
			}

			//Drawing critical sink compute by c = 1
			if(visible_Critical_Sink)
			{
				for(size_t i=0;i<OARST->getOASG()->getNumNode();i++)
				{	
					glColor3f(1.0f,1.0f,1.0f);
					SGNode* P = OARST->getOASG()->getNode(i);
					if(P->getLayer()==numLayer && P->getType() == _PIN && P->checkCriticalSink())
					{
						glPointSize(10);
						glBegin(GL_POINTS);
							glVertex3f(P->getX(),P->getY(),0.0f);
						glEnd();
					}
				}
			}

			//Drawing Pins with bad delay
			//plot the pin its delay is bigger than 0.5 x worst sink delay
			if(visible_Sink_Bad_Delay)
			{
				glColor3f(1.0f,0.9f,0.9f);
				if(OARST->getOARG()!=NULL && OARST->getOARG()->getWorstDelaySink() != NULL)
				{
					double threshold = OARST->getOARG()->getWorstDelaySink()->getDelay() * 0.5;
					for(size_t i=0;i<OARST->getOARG()->getNumNode();i++)
					{	
						RGNode* P = OARST->getOARG()->getNode(i);

						if(P->getLayer()==numLayer && P->getType() == _PIN && P->getDelay() > threshold)
						{
							glPointSize(5);
							glBegin(GL_POINTS);
								glVertex3f(P->getX(),P->getY(),0.0f);
							glEnd();
						}
					}
				}
			}

			//Drawing Pins with bad delay
			//plot the pin its delay is bigger than 0.7 x worst sink delay
			if(visible_Sink_Bad_Delay)
			{
				glColor3f(1.0f,0.6f,0.8f);
				if(OARST->getOARG()!=NULL && OARST->getOARG()->getWorstDelaySink() != NULL)
				{
					double threshold = OARST->getOARG()->getWorstDelaySink()->getDelay() * 0.7;
					for(size_t i=0;i<OARST->getOARG()->getNumNode();i++)
					{	
						RGNode* P = OARST->getOARG()->getNode(i);

						if(P->getLayer()==numLayer && P->getType() == _PIN && P->getDelay() > threshold)
						{
							glPointSize(5);
							glBegin(GL_POINTS);
								glVertex3f(P->getX(),P->getY(),0.0f);
							glEnd();
						}
					}
				}
			}


			//Drawing Pins with bad delay
			//plot the pin its delay is bigger than 0.8 x worst sink delay
			if(visible_Sink_Bad_Delay)
			{
				glColor3f(0.8f,0.3f,0.6f);
				if(OARST->getOARG()!=NULL && OARST->getOARG()->getWorstDelaySink() != NULL)
				{
					double threshold = OARST->getOARG()->getWorstDelaySink()->getDelay() * 0.8;
					for(size_t i=0;i<OARST->getOARG()->getNumNode();i++)
					{	
						RGNode* P = OARST->getOARG()->getNode(i);

						if(P->getLayer()==numLayer && P->getType() == _PIN && P->getDelay() > threshold)
						{
							glPointSize(5);
							glBegin(GL_POINTS);
								glVertex3f(P->getX(),P->getY(),0.0f);
							glEnd();
						}
					}
				}
			}

			//Drawing Pins with bad delay
			//plot the pin its delay is bigger than 0.9 x worst sink delay
			if(visible_Sink_Bad_Delay)
			{
				glColor3f(0.8f,0.0f,0.5f);
				if(OARST->getOARG()!=NULL && OARST->getOARG()->getWorstDelaySink() != NULL)
				{
					double threshold = OARST->getOARG()->getWorstDelaySink()->getDelay() * 0.9;
					for(size_t i=0;i<OARST->getOARG()->getNumNode();i++)
					{	
						RGNode* P = OARST->getOARG()->getNode(i);

						if(P->getLayer()==numLayer && P->getType() == _PIN && P->getDelay() > threshold)
						{
							glPointSize(5);
							glBegin(GL_POINTS);
								glVertex3f(P->getX(),P->getY(),0.0f);
							glEnd();
						}
					}
				}
			}

			//Drawing Pins with bad delay
			//plot the pin its delay is bigger than 0.95 x worst sink delay
			if(visible_Sink_Bad_Delay)
			{				
				glColor3f(1.0f,0.0f,0.0f);
				if(OARST->getOARG()!=NULL && OARST->getOARG()->getWorstDelaySink() != NULL)
				{
					double threshold = OARST->getOARG()->getWorstDelaySink()->getDelay() * 0.95;
					for(size_t i=0;i<OARST->getOARG()->getNumNode();i++)
					{	
						RGNode* P = OARST->getOARG()->getNode(i);

						if(P->getLayer()==numLayer && P->getType() == _PIN && P->getDelay() > threshold)
						{
							glPointSize(6);
							glBegin(GL_POINTS);
								glVertex3f(P->getX(),P->getY(),0.0f);
							glEnd();
						}
					}
				}
			}

			//Drawing Pins with long radius
			//plot the pin its radius is longer than [CriticalSinkRatio] x worst sink radius
			if(visible_Sink_Long_Radius)
			{

				double max_radius_in_OASG = 0;
				for(int i=0;i<OARST->getOASG()->getNumNode();i++)
				{	SGNode* P = OARST->getOASG()->getNode(i);
					if(max_radius_in_OASG < P->getDistFromDriving() && P->getType() == _PIN)
					{	max_radius_in_OASG = P->getDistFromDriving();
					}
				}


				glColor3f(0.0f,1.0f,1.0f);
				double threshold = max_radius_in_OASG * 0.8;//OARST->maxRadiusInOASG * 0.8;//OARST->CriticalSinkRatio;
				for(size_t i=0;i<OARST->getOASG()->getNumNode();i++)
				{	
					SGNode* P = OARST->getOASG()->getNode(i);

					if(P->getLayer()==numLayer && P->getType() == _PIN && P->getDistFromDriving() > threshold)
					{
						glPointSize(5);
						glBegin(GL_POINTS);
							glVertex3f(P->getX(),P->getY(),0.0f);
						glEnd();
					}
				}

			}

			//Drawing nodes in critical path
			if(visible_Nodes_In_Critical_Path)
			{
				glColor3f(0.0f,1.0f,0.5f);
				//glColor3f(0.0f,0.0f,0.0f);	

				for(size_t i=0;i<OARST->getOASG()->getNumNode();i++)
				{	
					SGNode* P = OARST->getOASG()->getNode(i);

					if(P->getLayer()==numLayer && P->checkInCriticalPath())
					{
						glPointSize(5);
						glBegin(GL_POINTS);
							glVertex3f(P->getX(),P->getY(),0.0f);
						glEnd();
					}
				}
			}

	/*		//Draw WNS Sink in OASG
			SGNode* WorstSlackSink_OASG = OARST->getOASG()->getWorstSlackSink();
			if(WorstSlackSink_OASG != NULL)
			{
				glPointSize(10);
				glColor3f(1.0f,0.0f,0.5f);
				glBegin(GL_POINTS);
				glVertex3f(WorstSlackSink_OASG->getX(),WorstSlackSink_OASG->getY(),0.0f);
				glEnd();
			}


			//Draw WNS Sink in OARG
			if(OARST->getOARG()!=NULL && OARST->getOARG()->getWorstSlackSink()!=NULL)
			{
				RGNode* WorstSlackSink_OARG = OARST->getOARG()->getWorstSlackSink();
				if(WorstSlackSink_OARG != NULL)
				{
					glPointSize(12);
					glColor3f(1.0f,0.0f,1.0f);
					glBegin(GL_POINTS);
					glVertex3f(WorstSlackSink_OARG->getX(),WorstSlackSink_OARG->getY(),0.0f);
					glEnd();
				}
			}
*/
			//Draw Positive slack sinks in OARG
			if(visible_Positive_Slack_In_OARG)
			{
				for(size_t i=0;i<OARST->getOASG()->getNumNode();i++)
				{	
					RGNode* P = OARST->getOARG()->getNode(i);
					double slack = P->getRequiredTime() - elmoreModel->get_Driver_Arrival_Time() - P->getDelay();
					glColor3f(1.0f,1.0f,1.0f);
					if(P->getLayer()==numLayer && slack > 0)
					{
						glPointSize(5);
						glBegin(GL_POINTS);
							glVertex3f(P->getX(),P->getY(),0.0f);
						glEnd();
					}
				}
			}


			//Draw driving node
			if(OARST->getDrivingNode()!=NULL)
			{
				glPointSize(12);
				//glPointSize(15);
				//glColor3f(0.2f,1.0f,1.5f);
				glColor3f(0.0f,0.0f,0.0f);	
				glBegin(GL_POINTS);
				glVertex3f(OARST->getDrivingNode()->getPoint()->getX(),OARST->getDrivingNode()->getPoint()->getY(),0.0f);
				glEnd();
			}


			glPointSize(5);
			//Drawing Processing Two-Pin Net's Multi-Source
			glColor3f(0.7f,0.9f,0.5f);
			for(int i=0;i<vMultiSource.size();i++)
			{				
				SGNode* n=vMultiSource[i];
				glBegin(GL_POINTS);
				glVertex3f(n->getPoint()->getX(),n->getPoint()->getY(),0.0f);
				glEnd();
			}

			glPointSize(7);
			//Drawing Processing Two-Pin Net's Source and Target
			if(Source != NULL)
			{
				glColor3f(1.0f,0.5f,0.0f);
				glBegin(GL_POINTS);
				glVertex3f(Source->getPoint()->getX(),Source->getPoint()->getY(),0.0f);
				glEnd();
			}
			if(Target != NULL)
			{
				glColor3f(0.5f,1.0f,0.0f);
				glBegin(GL_POINTS);
				glVertex3f(Target->getPoint()->getX(),Target->getPoint()->getY(),0.0f);
				glEnd();
			}


		}
	}
	glPopMatrix();
	glutSwapBuffers(); // include the flush effect
    glFlush();

}

// the reshape callback function:
void reshape(int w, int h)
{
	// set the viewport with exact window size:
	// glViewport(0, 0, w, h);
	glViewport(0, 0, (GLsizei) w, (GLsizei) h); // with explicit casting
	gluPerspective( 45.0, (GLfloat)w/(GLfloat)h, 0.1, -600.0 );
	glMatrixMode(GL_PROJECTION);

	// replace the current matric with the indentity matrix:
	glLoadIdentity();

	if (w <= h)
		glOrtho(-50, 50, -50 * (GLfloat) h / (GLfloat) w,
            50 * (GLfloat) h / (GLfloat) w, -1.0, 1.0);
	else
		glOrtho(-50 * (GLfloat) w / (GLfloat) h,
            50 * (GLfloat) w / (GLfloat) h, -50, 50, -1.0, 1.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}



//the keyboard callback function:
void keyboard(unsigned char key, int x, int y )
{

	float sutX=(database.getMaxX()-database.getMinX())/(20/zoomX);	
	float sutY=(database.getMaxY()-database.getMinY())/(20/zoomY);		

	switch( key )
	{
		case 'q':
			cout<<"\n--'q' : enable/disable fullscrean\n";
			fullscreen = !fullscreen;
			if (fullscreen) {
				Xpos = glutGet((GLenum)GLUT_WINDOW_X);    // Save parameters
				Ypos = glutGet((GLenum)GLUT_WINDOW_Y);
				Xsize = glutGet((GLenum)GLUT_WINDOW_WIDTH);
				Ysize = glutGet((GLenum)GLUT_WINDOW_HEIGHT);
				glutFullScreen();                // Go to full screen
			} else {
				glutReshapeWindow(Xsize, Ysize);        // Restore us
				glutPositionWindow(Xpos,Ypos);
			}
			break;
		case 'd':
			cout<<"\n--'d' : print all nodes' information (nodes of OASG)\n";
			for(int i=0;i<OASG->getNumNode();i++)
			{	SGNode* n= OASG->getNode(i);
				PrintNode(n);
			}
			break;
		case 'f':
			cout<<"\n--'f' : print all nodes' information (nodes of OARG)\n";
			if(OARST->getOARG()==NULL)
			{	cout<<"rectilinearize not yet\n";
			}
			else
			{
				for(int i=0;i<OARST->getOARG()->getNumNode();i++)
				{	RGNode* n= OARST->getOARG()->getNode(i);
					PrintRGNode(n);

					if(n->getDistFromDriving()==_INFINITE)
					{	cout<<"Error occured! DistFromDriving of node in OARG cannot be INFINITE\n";
						exit(0);
					}
				}
				cout<<"# of nodes in OARG = "<<OARST->getOARG()->getNumNode()<<"\n";
			}
			break;
		case 'm':
			cout<<"--'m' : print all nodes' information containing timing (nodes of OARG)\n";
			if(OARST->getOARG()==NULL)
			{	cout<<"rectilinearize not yet\n";
			}
			else
			{
				for(int i=0;i<OARST->getOARG()->getNumNode();i++)
				{	RGNode* n= OARST->getOARG()->getNode(i);
					PrintRGNode_Delay(n);

					if(n->getDistFromDriving()==_INFINITE)
					{	cout<<"Error occured! DistFromDriving of node in OARG cannot be INFINITE\n";
						exit(0);
					}
				}
				cout<<"# of nodes in OARG = "<<OARST->getOARG()->getNumNode()<<"\n";
			}
			break;			
		case 'l':
			cout<<"\n--'l' : print all leaf nodes' information (nodes of OARG)\n";
			if(OARST->getOARG()==NULL)
			{	cout<<"rectilinearize not yet\n";
			}
			else
			{
				int numLeaf = 0;
				for(int i=0;i<OARST->getOARG()->getNumNode();i++)
				{						
					RGNode* n= OARST->getOARG()->getNode(i);
					if(n->checkIsLeaf())
					{	PrintRGNode(n);
						numLeaf++;

						if(n->getDistFromDriving()==_INFINITE)
						{	cout<<"Error occured! DistFromDriving of node in OARG cannot be INFINITE\n";
							exit(0);
						}
					}

				}
				cout<<"# of leaf nodes in OARG = "<<numLeaf<<"\n";
			}
			break;
		case 'c':
			if(OARST->getOARG()==NULL)
			{	cout<<"rectilinearize not yet\n";
			}
			else
			{
				cout<<"--'c' : check nodes' connection (nodes of OARG)\n";
				OARST->getOARG()->Check_Conn();
			}
			break;
		case 'v':
			if(OARST->getOARG()==NULL)
			{	cout<<"rectilinearize not yet\n";
			}
			else
			{
				cout<<"--'v' : check do edges have duplication ? (edge of OARG)\n";
				OARST->getOARG()->Check_Edge();
			}
			break;
		case 'r':
			cout<<"\n--'r' : incrementally route 2-pin net (spanning edge)\n";
			if(indexRoutingNet < OARST->getNumTwoPinNet())
			{	
				TwoPinNet* net=OARST->getTwoPinNet(indexRoutingNet);
				Source = net->getSource();
				Target = net->getTarget();

				vMultiSource.clear();
				OARST->computeMultiSources(Source,Target,vMultiSource);

				OARST->routeTwoPinNet(indexRoutingNet,Source,Target,vMultiSource);
				indexRoutingNet++;

				cout<<"Route #"<<indexRoutingNet<<" two-pin-net"<<"\n";
				OARST->getOASG()->Check_Conn();
			}
			break;
		case 'e':
			cout<<"\n--'e' : initialize previos work of rectilinearizing (init OARG)\n";
			if(OARST->getOARG()==NULL)
			{	OARST->initOARG();
			}

			break;
		case 'w':
			cout<<"\n--'w' : incrementally rectilinearize spanning edge\n";
			if(OARST->getOARG()==NULL)
			{	OARST->initOARG();
			}			
			if(OARST->getOARG()->getNumUnPropRGNode()!=0)
			{	OARST->getOARG()->TurnRectilinear_Process_One_Node();
			}
			break;
		case 't':
			cout<<"\n--'t' : print all horizontal and vertical edges' information (OARG)\n";
			if(OARST->getOARG() != NULL)
			{
				for(int i=0;i<OARST->getOARG()->getNumEdge();i++)
				{	RGEdge* E = OARST->getOARG()->getEdge(i);
					PrintRGEdge(E);
					cout<<"========================\n";
				}
				cout<<"# of horizontal and vertical edges (OARG) = "<<OARST->getOARG()->getNumEdge()<<"\n";
			}
			else
			{	cout<<"horizontal and vertical edges don't exist\n";
			}
			break;
		case 'i':
			cout<<"\n--'i' : print informations about this steiner tree\n";
			cout<<"driving node:\n";
			PrintNode(OARST->getDrivingNode());
			break;
		case 'g':
			//cout<<"--'g' : hide/display grids\n";
			visible_Grids=!visible_Grids;
			break;
		case 's':
			//cout<<"\n--'s' : hide/display spanning edges\n";
			visible_SpanningEdge=!visible_SpanningEdge;
			break;
		case 'a':
			//cout<<"\n--'a' : hide/display routed spanning edges\n";
			visible_Routed_SpanningEdge=!visible_Routed_SpanningEdge;
			break;
		case 'b':		
			//cout<<"--'b' : hide/display H/V edges\n";			
			visible_H_V_Edge=!visible_H_V_Edge;
			break;
		case '~':
			//cout<<"--'~' : hide/display sink with long radius\n";
			visible_Sink_Long_Radius=!visible_Sink_Long_Radius;
			break;
		case '!':
			//cout<<"--'!' : hide/display sink with bad delay\n";
			visible_Sink_Bad_Delay=!visible_Sink_Bad_Delay;
			break;
		case '@':
			//cout<<"--'@' : hide/display critical paths\n";
			visible_Critical_Path=!visible_Critical_Path;
			break;
		case '#':
			//cout<<"--'#' : hide/display nodes in critical path\n";
			visible_Nodes_In_Critical_Path=!visible_Nodes_In_Critical_Path;
			break;
		case '$':
			//cout<<"--'$' : hide/display critical sinks\n";
			visible_Critical_Sink=!visible_Critical_Sink;
			break;
		case '%':
			//cout<<"--'%' : hide/display leaf node in OASG\n";
			visible_Leaf_in_OASG=!visible_Leaf_in_OASG;
			break;
		case '^':
			//cout<<"--'^' : hide/display positive slack sink in OARG\n";
			visible_Positive_Slack_In_OARG=!visible_Positive_Slack_In_OARG;
			break;
		case 27:
			cout<<"\n--'ESC' : end this program\n";
			exit (0);
			break;

		
		case '8':   //display window control
			transferY=transferY+sutY;	
			break;
		case '2':   //display window control
			transferY=transferY-sutY;
			break;
		case '4':   //display window control
			transferX=transferX-sutX;
			break;
		case '6':   //display window control
			transferX=transferX+sutX;
			break;
		case 'z':   //display window control
			zoomX=zoomX*10/9;
			zoomY=zoomY*10/9;
			break;
		case 'x':   //display window control
			zoomX=zoomX*9/10;
			zoomY=zoomY*9/10;
			break;
		
		case 'h':
			cout<<"\n--[HELP]\n";
			cout<<"--'q' : enable/disable fullscrean\n";
			cout<<"--'d' : print all nodes' information (nodes of OASG)\n";
			cout<<"--'f' : print all nodes' information (nodes of OARG)\n";
			cout<<"--'m' : print all nodes' information containing timing (nodes of OARG)\n";
			cout<<"--'l' : print all leaf nodes' information (nodes of OARG)\n";
			cout<<"--'c' : check nodes' connection (nodes of OARG)\n";
			cout<<"--'v' : check do edges have duplication ? (edge of OARG)\n";
			cout<<"--'r' : incrementally route 2-pin net (spanning edge)\n";
			cout<<"--'e' : initialize previos work of rectilinearizing (init OARG)\n";
			cout<<"--'w' : incrementally rectilinearize spanning edge\n";
			cout<<"--'t' : print all horizontal and vertical edges' information (OARG)\n";
			cout<<"--'i' : print informations about this steiner tree\n";
			cout<<"--'g' : hide/display grids\n";
			cout<<"--'s' : hide/display spanning edges\n";
			cout<<"--'a' : hide/display routed spanning edges\n";
			cout<<"--'b' : hide/display H/V edges\n";
			cout<<"--'~' : hide/display sink with long radius\n";
			cout<<"--'!' : hide/display sink with bad delay\n";
			cout<<"--'@' : hide/display critical paths\n";
			cout<<"--'#' : hide/display nodes in critical path\n";
			cout<<"--'$' : hide/display critical sinks\n";
			cout<<"--'%' : hide/display leaf node in OASG\n";
			cout<<"--'^' : hide/display positive slack sink in OARG\n";
			cout<<"--'z' : zoom in  (magnification = 10/9)\n";
			cout<<"--'x' : zoom out (magnification = 9/10)\n";
			cout<<"--'ESC' : end this program\n";
			break;
		default:
			break;
	}

	display();
}

//menu for setting layer visible
void LayerFunc(int data)
{

	int num= glutGet(GLUT_MENU_NUM_ITEMS);
	glutRemoveMenuItem(num);

	switch(data)
	{
		case 1:
			visible_layer[1]=!visible_layer[1];
			break;
		case 2:
			visible_layer[2]=!visible_layer[2];
			break;
		case 3:
			visible_layer[3]=!visible_layer[3];
			break;
		case 4:
			visible_layer[4]=!visible_layer[4];
			break;
		case 5:
			visible_layer[5]=!visible_layer[5];
			break;
		case 6:
			visible_layer[6]=!visible_layer[6];
			break;
		case 7:
			visible_layer[7]=!visible_layer[7];
			break;
		case 8:
			visible_layer[8]=!visible_layer[8];
			break;
		case 9:
			visible_layer[9]=!visible_layer[9];
			break;
		case 10:
			visible_layer[10]=!visible_layer[10];
			break;
		default:
			break;
	}

	char s[100]="Now visible layers are ";
	for(int i=0;i<10;i++)
	{	
		if(visible_layer[i]==1)
		{	strcat(s,",");
			switch(i)
			{	case 1: strcat(s,"1");
						 break;
				case 2: strcat(s,"2");
						 break;
				case 3: strcat(s,"3");
						 break;
				case 4: strcat(s,"4");
						 break;
				case 5: strcat(s,"5");
						 break;
				case 6: strcat(s,"6");
						 break;
				case 7: strcat(s,"7");
						 break;
				case 8: strcat(s,"8");
						 break;
				case 9: strcat(s,"9");
						 break;
				case 10: strcat(s,"10");
						 break;

			}
		}
	}
	glutSetMenu(layer_menu);
	glutAddMenuEntry(s, 12);


	display();
}

//
void MenuFunc(int data)
{	
	switch(data)
	{
	}
}


void initGUI(int argc, char **argv)
{
	try
	{
		glutInit(&argc, argv); // initializes GLUT and processes any command line arguments
							   // Note: glutInit() should be called before any other GLUT routine
		glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB); // chooses double buffered and RGBA mode
		glutInitWindowPosition(100, 100); // specifies the upper-left corner of window
		glutInitWindowSize(600,600); // specifies the size, in pixels, of window
		glutCreateWindow("Display"); // creates a window of name given by input string
		glClearColor (0, 0, 0, 0); // set clear window color black
		glShadeModel (GL_FLAT);
		glutDisplayFunc(display); // registers display callback function, which is display()
		glutReshapeFunc(reshape); // registers reshape callback function, which is reshape()
		glutKeyboardFunc(keyboard); 


		menu = glutCreateMenu(MenuFunc);
		layer_menu = glutCreateMenu(LayerFunc);

		glutSetMenu(menu);
		glutAddMenuEntry("Run Program",100);
		glutAddSubMenu("Enable Visible Layer", layer_menu);
		glutAttachMenu(GLUT_RIGHT_BUTTON); 


		glutSetMenu(layer_menu);
		glutAddMenuEntry("Layer 1", 1);
		glutAddMenuEntry("Layer 2", 2);
		glutAddMenuEntry("Layer 3", 3);
		glutAddMenuEntry("Layer 4", 4);
		glutAddMenuEntry("Layer 5", 5);
		glutAddMenuEntry("Layer 6", 6);
		glutAddMenuEntry("Layer 7", 7);
		glutAddMenuEntry("Layer 8", 8);
		glutAddMenuEntry("Layer 9", 9);
		glutAddMenuEntry("Layer 10", 10);
		glutAddMenuEntry("=======================", 11);
		glutAddMenuEntry("Now visible layers are ", 12);


		glClearColor(1.0,1.0,1.0,1.0); 
		glutMainLoop(); // shows the window and loops to accepts and processes input events

	}
	catch(int errorcode)
	{	cout<<"catch exception,error code:"<<errorcode<<endl;
	}
}
#endif

void PrintRunTimeInfo()
{	
#ifdef DEBUG_ANALYSIS_RUN_TIME
	cout<<"Run-time of fileIO = "<< float(RunTime->read_file_end   -   RunTime->read_file_start)   /   CLK_TCK<<endl;
		
	cout<<"\nRun-time of building spanning graph = "<< float(RunTime->SG_build_end   -   RunTime->SG_build_start)   /   CLK_TCK<<endl;
/*	cout<<"		Run-time of part 1 = "<< RunTime->SG_build_part1<<endl;
	cout<<"		Run-time of part 2 = "<< RunTime->SG_build_part2<<endl;
	cout<<"		Run-time of part 3 = "<< RunTime->SG_build_part3<<endl;
	cout<<"		Run-time of part 4 = "<< RunTime->SG_build_part4<<endl;
	cout<<"		Run-time of meeting process entry = "<< RunTime->SG_meet_process_entry<<endl;
	cout<<"		Run-time of not meeting process entry = "<< RunTime->SG_not_meet_process_entry<<endl;
	cout<<"		Run-time of check ASet = "<< RunTime->SG_check_ASet<<endl;
	cout<<"		Run-time of check intersection (sweep line) = "<< RunTime->SG_check_intersection <<endl;
*/		
#ifdef PRIORITY_BASED
	cout<<"\nRun-time of assigning slack for each node = "<<float(RunTime->slack_assignment_end - RunTime->slack_assignment_start) / CLK_TCK<<endl;
	cout<<"\nSlackBeforePrim Max="<<database.getTempMaxSlack()<<", Min="<<database.getTempMinSlack()<<endl;
#endif
	cout<<"\nRun-time of doing prim's algorithm = "<< float(RunTime->prim_end   -   RunTime->prim_start)   /   CLK_TCK<<endl;
	cout<<"\nTimes of update in prim's algorithm = "<<RunTime->times_of_prim<<endl;
	cout<<"		Run-time of consider obstacle (by r-tree) = "<< RunTime->prim_consider_obstacle <<endl;

	cout<<"\nRun-time of routing spanning graph = "<< float(RunTime->SG_routing_end   -   RunTime->SG_routing_start)   /   CLK_TCK<<endl;
	cout<<"		more detail \n";
	cout<<"		Run-time of generating multi-sources (by r-tree) = "<< RunTime->SG_routing_gen_multi_sources<<endl;
	cout<<"		Run-time of maze routing = "<< RunTime->SG_routing_maze_route<<endl;
	cout<<"		Run-time of back tracing = "<< RunTime->SG_routing_back_trace<<endl;
	cout<<"		Run-time of restoring variable = "<< RunTime->SG_routing_restore_variable<<endl;
	cout<<"         TimesOfMaze = "<< RunTime->times_of_maze<<endl;

	cout<<"\nRun-time of rectilinearizing spanning edges = "<< float(RunTime->RG_rectilinear_end   -   RunTime->RG_rectilinear_start)   /   CLK_TCK<<endl;

	cout<<"\nRun-time of post processing = "<< float(RunTime->post_process_end   -   RunTime->post_process_start)   /   CLK_TCK<<endl;

	cout<<"\nRun-time of trunk algo = "<< float(RunTime->trunk_end   -   RunTime->trunk_start)   /   CLK_TCK<<endl;

	cout<<"\nRun-time of program = "<< float(RunTime->program_end   -   RunTime->program_start)   /   CLK_TCK<<endl;
	
	cout<<"\nOthers of program = "<< float(RunTime->others_end   -   RunTime->others_start)   /   CLK_TCK<<endl;
#endif
}

void CheckCorrection()
{
#ifdef DEBUG_CHECK_CORRECTION
	//DEBUG
	//cout<<"\n--[Check nodes]\n";
	OARST->getOARG()->Check_Node();
	//cout<<"\n--[Check all pins in tree in OARG]\n";
	OARST->getOARG()->Check_All_Pin_In_Tree(database);	
	//cout<<"\n--[Check all the node in tree in OARG]\n";
	OARST->getOARG()->Check_Node_All_In_Tree();
	//cout<<"\n--[Check edges]\n";
	//OARST->getOARG()->Check_Edge(); //for post processing
	//cout<<"\n--[Check connection]\n";
	OARST->getOARG()->Check_Conn();
	//cout<<"\n--[Check node type]\n";
	OARST->getOARG()->Check_Node_Type();
	//cout<<"\n--[Check duplicated nodes]\n";
	OARST->getOARG()->Check_Duplicate_Node();
	//cout<<"\n--[Correct overlap edge in OARG if any]\n";
	//OARST->getOARG()->OverlapCorrect();
#endif
#ifndef DEBUG_CHECK_CORRECTION
	cout<<"CHECK CORRECTION is disable!\n\n";
#endif
}

void ResultOfRouteSG()
{
	cout<<"\n--[Routing Result :]\n";	
	cout<<"Total SGEdge Length = "<<OARST->getOASG()->getRoutedSGEdgeLength()<<"\n";
	if(OARST->getOASG()->getLongestRadius()==0)
		OARST->getOASG()->computeRadius();
	cout<<"Longest Radius = "<<OARST->getOASG()->getLongestRadius()<<"\n";
}

double ResultOfRectiSG()
{
	cout<<"\n--[Rectilinearizing Result :]\n";
	cout<<"# of RGEdge = "<<OARST->getOARG()->vRGEdge.size()<<"\n";
	double total_WL = OARST->getOARG()->getRoutedRGEdgeLength();
	cout<<"Total RGEdge Length = "<<total_WL<<"\n";
	if(OARST->getOARG()->getLongestRadius()==0)
		OARST->getOARG()->computeRadius();
	cout<<"Longest Radius = "<<OARST->getOARG()->getLongestRadius()<<"\n";

	return total_WL;
}

void openfile(string filename)
{
#ifdef DEBUG_ANALYSIS_RUN_TIME
	RunTime->read_file_start = clock();
#endif
	cout<<"Open file = "<<filename<<endl;
	readfile(filename,database,elmoreModel);	
	
#ifdef DEBUG_ANALYSIS_RUN_TIME
	RunTime->read_file_end = clock();
#endif

}

#ifdef BUF_DRIVEN
void openBufFile(char filename[])
{
  readBufFile(filename,database,elmoreModel);
}
#endif

void Init_Data()
{
	cout<<"\n--[SortPin]";
	cout<<"\n--[SortObstacle]\n";
	database.SortPin_ByX();	
	database.SortObstacle_ByX1();

#ifdef DEBUG_ANALYSIS_RUN_TIME
	RunTime->SG_build_start = clock();
#endif
	
	cout<<"\n--[Init OASpanningGraph]\n";
	OASG =new OASpanningGraph(database,elmoreModel);

#ifdef DEBUG_ANALYSIS_RUN_TIME
	RunTime->SG_build_end = clock();
#endif

	cout<<"\n--[Init Parameters of OARSteinerTree]\n";
	OARST =new OARSteinerTree(OASG,elmoreModel);
	OARST->setRunTimeSet(RunTime);
	elmoreModel->setOASG(OARST->getOASG());

	cout<<"\n--[Designate Driving Node of OARSteinerTree]\n";
	OARST->designateDrivingNode(database);

#ifdef PRIORITY_BASED

#ifdef DEBUG_ANALYSIS_RUN_TIME
	RunTime->slack_assignment_start=clock();
#endif

	cout<<"\n==[computeSlackForEachNode]\n";
	OARST->computeSlackForEachNode(database);

#ifdef DEBUG_ANALYSIS_RUN_TIME
	RunTime->slack_assignment_end=clock();
#endif

#endif


#ifdef DEBUG_ANALYSIS_RUN_TIME
	RunTime->prim_start = clock();
#endif

	cout<<"\n--[Prim]\n";
#ifdef TEST_HOMER
	cout<<"Just do doRefinedPrim()\n";
	OARST->doRefinedPrim(database);
#else
	//determine which prim's algorithms (pins mainly or whole layout mainly)
	if(OARST->getOASG()->getNumNode() > database.getPinNum()*5)
	{	cout<<"# of SGNode > '# of Pin * 5' : doPrim()\n";
		OARST->doRefinedPrim(database);	
	}
	else
	{	cout<<"# of SGNode <= '# of Pin * 5' : doRefinedPrim()\n";
		OARST->doRefinedPrim(database);	
	}
#endif
	

#ifdef DEBUG_ANALYSIS_RUN_TIME
	RunTime->prim_end = clock();
#endif

}


void Grow_Trunk()
{	
#ifdef DEBUG_ANALYSIS_RUN_TIME
	RunTime->trunk_start = clock();
#endif
	cout<<"\n--[Compute Critical Path]\n";
	OARST->GrowCriticalPath(database,elmoreModel,elmoreModel->get_Driver_Arrival_Time());

#ifdef DEBUG_ANALYSIS_RUN_TIME
	RunTime->trunk_end = clock();
#endif
}

void Route_OASG()
{
#ifdef DEBUG_ANALYSIS_RUN_TIME
	RunTime->SG_routing_start = clock();
#endif
	cout<<"\n--[RouteSpanningEdge]\n";
	OARST->RouteSpanningGraph();

	cout<<"\n--[Compute each sink's distance from driver in OASG]\n";
	OARST->getOASG()->computeDistFromDriver();

#ifdef DEBUG_ANALYSIS_RUN_TIME
	RunTime->SG_routing_end = clock();	
#endif
}

void Build_OARG()
{
#ifdef DEBUG_ANALYSIS_RUN_TIME
	RunTime->RG_rectilinear_start = clock();
#endif
	
	cout<<"\n--[Init OARectilinearGraph]\n";
	OARST->initOARG();
	elmoreModel->setOARG(OARST->getOARG());

	cout<<"\n--[Rectilinearize spanning edge]\n";
	OARST->getOARG()->TurnRectilinear_From_Driver();

	//if(database.getObstacleNum()==0)
	//{		
		cout<<"\n--[Remove U shape in OARG]\n";
		OARST->getOARG()->setReduceUOK(true);
		OARST->getOARG()->UShapeRemove();
	//}
	//else
	//{
	//	cout<<"\n--[Remove U shape in OARG]\n";
	//	OARST->getOARG()->setReduceUOK(true);
	//	OARST->getOARG()->UShapeRemove();
	//}

	cout<<"\n--[Compute each sink's distance from driver in OARG]\n";
	OARST->getOARG()->computeDistFromDriver();	
	
#ifdef DEBUG_ANALYSIS_RUN_TIME
	RunTime->RG_rectilinear_end = clock();
#endif
}

void Timing_Analysis()
{
	cout<<"\n--[Apply Elmore delay model to compute sink delay (OARG)]\n";
	elmoreModel->Compute_OARG_Delay(database);
}

//==============================================
//parameter related with post processing
double Old_WNS = _UNSET;
double New_WNS = _UNSET;
int Redirect_Times = 0;
bool DoRedirect = true;
int fail_times = 0;
int fail_upper_bound = 10;
//==============================================


int Post_Processing_Part1()
{
		if(Initial_WL == _UNSET)
		{	Initial_WL = OARST->getOARG()->getRoutedRGEdgeLength();
			Initial_WD = elmoreModel->getWorseDelaySinkInOARG()->getDelay();
		}

		RGNode* Old_WNS_Sink = OARST->getOARG()->getWorstSlackSink();
		if(Old_WNS_Sink == NULL)
		{	Min_WNS = 0; //all sinks meet timing constrain
			Min_WNS_In_Times = Redirect_Times;
			WL_When_Min_WNS = OARST->getOARG()->getRoutedRGEdgeLength();
			WD_When_Min_WNS = elmoreModel->getWorseDelaySinkInOARG()->getDelay();
			if(Initial_WNS == _UNSET)
			{	Initial_WNS = 0;
			}
			return _STOP;
		}


		Old_WNS = Old_WNS_Sink->getRequiredTime() - elmoreModel->get_Driver_Arrival_Time() - Old_WNS_Sink->getDelay();
		if(Initial_WNS == _UNSET)
		{	Initial_WNS = Old_WNS;
		}

		if(Old_WNS > Min_WNS)
		{	Min_WNS = Old_WNS;
			Min_WNS_In_Times = Redirect_Times;
			WL_When_Min_WNS = OARST->getOARG()->getRoutedRGEdgeLength();
			WD_When_Min_WNS = elmoreModel->getWorseDelaySinkInOARG()->getDelay();
		}

		return _CONTINUE;
}

int Post_Processing_Part2()
{
		RGNode* New_WNS_Sink = OARST->getOARG()->getWorstSlackSink();

		if(New_WNS_Sink == NULL)
		{	Min_WNS = 0; //all sinks meet timing constrain
			Min_WNS_In_Times = Redirect_Times;
			WL_When_Min_WNS = OARST->getOARG()->getRoutedRGEdgeLength();
			WD_When_Min_WNS = elmoreModel->getWorseDelaySinkInOARG()->getDelay();
			
		}

		New_WNS = New_WNS_Sink->getRequiredTime() - elmoreModel->get_Driver_Arrival_Time() - New_WNS_Sink->getDelay();		
		if(New_WNS > 0)
		{	return _STOP; //all sinks meet timing constrain
		}

		cout<<"\n============ Redirect Result ============\n";
		cout<<"Old WNS = "<<Old_WNS<<", New WNS¡@= "<<New_WNS<<endl;
		cout<<"WNS become "<<(double)(New_WNS/Old_WNS)*100<<"%\n";
		if(New_WNS <= Old_WNS)
		{	fail_times ++ ;
			if(fail_times > fail_upper_bound)
			{	cout<<"=> fail_times > fail_upper_bound ("<<fail_upper_bound<<") so STOP\n";
				DoRedirect = false;
			}
		}
		if(New_WNS < Min_WNS*1.5) //limit the number of times of redirecting
		{	cout<<"=> enough redirect\n";
			DoRedirect = false;
		}

		if(New_WNS > Min_WNS)
		{	Min_WNS = New_WNS;
			Min_WNS_In_Times = Redirect_Times;
			WL_When_Min_WNS = OARST->getOARG()->getRoutedRGEdgeLength();
			WD_When_Min_WNS = elmoreModel->getWorseDelaySinkInOARG()->getDelay();
		}

		return _CONTINUE;
}

void Post_Processing()
{	
#ifdef DEBUG_ANALYSIS_RUN_TIME
	RunTime->post_process_start = clock();
#endif

	cout<<"\n============ WNS and TNS ============";
	OARST->getOARG()->Check_Timing_Violation(elmoreModel->get_Driver_Arrival_Time());

	if(OARST->getOARG()->getWorstSlackSink()==NULL)
	{	return ;
	}

	cout<<"\n--[Reduce Negative Slack by Redirect Mechanism]\n";
	
	int times = 0;
	while(DoRedirect && (Old_WNS==_UNSET || Old_WNS!=New_WNS) && times<22)
	{
		cout<<"\n--[Part-1]\n";
		if(Post_Processing_Part1()==_STOP)
		{	cout<<"all sinks meet timing constrain!\n\n";
			break;
		}
		cout<<"\n--[Part-1 end]\n";

		Redirect_Times++;
		if(OARST->ReduceNegativeSlack() == false)
		{	cout<<"detect NULL RG Node, and redirect (WNS reduction) is terminated\n";
			cout<<"use previous best result as final answer\n"; //notice that the final layout of WNS reduction is wrong
			                                                    //because we didn't restore the GUI layout back to best one
			                                                    //we only use variables to store the best result 
			                                                    //such as WL_When_Min_WNS, Min_WNS and WD_When_Min_WNS.
			cout<<"=================================\n";
			break;
		}
		

		Timing_Analysis();

		OARST->getOARG()->Check_Timing_Violation(elmoreModel->get_Driver_Arrival_Time());

		cout<<"# "<<times<<endl;
		cout<<"\n--[Part-2]\n";
		if(Post_Processing_Part2()==_STOP)
		{	cout<<"all sinks meet timing constrain!\n\n";
			break;
		}
		cout<<"\n--[Part-2 end]\n";

		times++;
	}

	double DDD(0);
	double NN(0);
	for(int l=0;l<OARST->getOARG()->getNumNode();l++)
	{
	  RGNode* T = OARST->getOARG()->getNode(l);
	  if(T->getType()==_PIN&&T!=OARST->getOARG()->getDrivingNode())
	  {
	    DDD+=T->getDelay();
	    NN++;
	  }
	}
	DDD/=NN;

	/*
	cout<<"we did total "<<Redirect_Times<<" rounds of redirect\n";
	cout<<"Init WL = "<<Initial_WL<<endl;
	cout<<"Init WNS = "<<Initial_WNS<<endl;		
	printf("Init WD = %lf (ps)\n",Initial_WD);
	*/

	cout<<"=================================\n";
	cout<<"we use the result of # "<<Min_WNS_In_Times<<" round"<<endl;
	cout<<"WL when Min WNS = "<<WL_When_Min_WNS<<endl;
	cout<<"Min WNS = "<<Min_WNS<<endl;
	printf("WD When Min WNS= %lf (ps)\n",WD_When_Min_WNS);
	printf("TD When Min WNS= %lf (ps)\n",DDD);

#ifdef DEBUG_ANALYSIS_RUN_TIME
	RunTime->post_process_end = clock();
#endif

}

void Slack_Analysis()
{	
	OARST->getOARG()->Check_Timing_Violation(elmoreModel->get_Driver_Arrival_Time());
}



void Display_WL_Result()
{
		ResultOfRouteSG();
		double total_WL = ResultOfRectiSG();
		if(Min_WL > total_WL)
		{	Min_WL = total_WL;
		}
}

void Write_Table(char* filename,char* net_name)
{
	FILE *fp;
	cout<<"write table file = "<<filename<<endl;	
	fp = fopen(filename,"a+"); //append
	
	if(fp==NULL) //Check file is available or not
	{	cout<<"Can not write table!\n";
		exit(0);
	}

	fprintf(fp,"\nNet %s : # of Pins = %d, # of Obstacle = %d\n",net_name,database.getPinNum(),database.getObstacleNum());
	
		
	if(Min_WNS == -1*(_INFINITE))
	{
		fprintf(fp,"== Not Do Redirecting ==\n");
		fprintf(fp,"WD = %lf (ps)\n",elmoreModel->getWorseDelaySinkInOARG()->getDelay());
		fprintf(fp,"WL = %lf (um)\n",Min_WL);
		if(OARST->getOARG()->getWorstSlackSink()==NULL)
		{	fprintf(fp,"WNS = 0 (ps)\n");	
		}
		else
		{	double slack = OARST->getOARG()->getWorstSlackSink()->getRequiredTime() - OARST->getOARG()->getWorstSlackSink()->getDelay() - elmoreModel->get_Driver_Arrival_Time();
			fprintf(fp,"WNS = %lf (ps)\n",slack);
		}
	}
	else
	{
		fprintf(fp,"== Do Redirecting ==\n");
		fprintf(fp,"Init WD = %lf (ps)\n",Initial_WD);		
		fprintf(fp,"Init WL = %lf (um)\n",Initial_WL);
		fprintf(fp,"Init WNS = %lf (ps)\n",Initial_WNS);
		fprintf(fp,"In # %d times\n",Min_WNS_In_Times);
		fprintf(fp,"WD When Min WNS= %lf (ps)\n",WD_When_Min_WNS);
		fprintf(fp,"WL When Min WNS= %lf (um)\n",WL_When_Min_WNS);
		fprintf(fp,"Min WNS = %lf (ps)\n",Min_WNS);
	}
	fprintf(fp,"Run-time of post process = %lf (s)\n",float(RunTime->post_process_end - RunTime->post_process_start)/CLK_TCK);
	fprintf(fp,"Run-time of program = %lf (s)\n",float(RunTime->program_end - RunTime->program_start)/CLK_TCK);
	fprintf(fp,"Others of program (check correction) = %lf (s)\n",float(RunTime->others_end - RunTime->others_start)/CLK_TCK);
}

#ifdef XP_MODE
int main(int argc, char **argv)
{
	cout<<"\n-------------Read file-----------\n";
	//char readfile[80] = "benchOARST_NoObstacle_Timing/RC12_T.net";
	char readfile[80] = "benchOARST_Timing/RC05_T.net";
	//char readfile[80] = "sample.net";
	//char readfile[80] = "benchOARST_Modified/IND2.net";	
	openfile(readfile);

#ifdef DEBUG_ANALYSIS_RUN_TIME
	RunTime->program_start = clock();
#endif

	cout<<"\n-------------init data-----------\n";
	Init_Data();

	cout<<"\n-------------Grow trunk-----------\n";
	Grow_Trunk();

	cout<<"\n-------------Route OASG-----------\n";
	Route_OASG();

	cout<<"\n-------------Build OARG-----------\n";
	Build_OARG();

	cout<<"\n-------------Timing Analysis-----------\n";
	Timing_Analysis();

	cout<<"\n-------------Slack Analysis-----------\n";
	Slack_Analysis();

	cout<<"\n-------------Post Processing : Refine Timging-----------\n";
	Post_Processing();

#ifdef DEBUG_ANALYSIS_RUN_TIME
	RunTime->program_end = clock();
#endif

	cout<<"\n-------------Check Correction-----------\n";
	CheckCorrection();

	cout<<"\n-------------Display WL Result-----------\n";
	Display_WL_Result();

	cout<<"\n-------------Display Run-Time Result-----------\n";
	PrintRunTimeInfo();
	initGUI(argc,argv);

	return 0;
}

#endif


void Write_FBI(){
  ofstream outfile("FBI.out");
  //outfile<<"Just a test"<<endl;
  outfile<<"wire_res_per_unit 0.076"<<endl;
  outfile<<"wire_cap_per_unit 0.118"<<endl<<endl;

  int IDD=0;
  OARST->getDrivingNode()->getRefRGNode()->setIndex(IDD);
  ++IDD;
  outfile<<"driver "<<OARST->getDrivingNode()->getX()<<" "<<OARST->getDrivingNode()->getY()<<" 440"<<endl; // x, y, driver resistance

  outfile<<"number_of_sinks "<<(database.GetPinSize()-1)<<endl;

  for(int i=0;i<OARST->getOARG()->getNumNode();++i){
    RGNode* p(0);
    p=OARST->getOARG()->getNode(i);
    assert(p);
    if(p->getType()==_PIN&&p!=OARST->getOARG()->getDrivingNode()){
      assert(p);
      assert(p->getSGNode());
      p->setIndex(IDD);
      ++IDD;
      outfile<<"sink "<<p->getIndex()<<" "<<p->getX()<<" "<<p->getY()<<" "<<p->getLoading()<<" "<<p->getRequiredTime()<<endl;
    }
  }
  outfile<<endl;

  outfile<<"number_of_candidate_nodes "<<(OARST->getOARG()->getNumNode()-database.GetPinSize())<<endl;
  for(int i=0;i<OARST->getOARG()->getNumNode();++i){
    RGNode* p(OARST->getOARG()->getNode(i));
    assert(p);
    if(p->getType()!=_PIN){
      p->setIndex(IDD);
      ++IDD;
      outfile<<"candidate "<<p->getIndex()<<" "<<p->getX()<<" "<<p->getY()<<endl;
    }
  }
  outfile<<endl;

  // edges
  for(size_t i=0;i<OARST->getOARG()->getNumEdge();++i){
    RGEdge* e(OARST->getOARG()->getEdge(i));
    if(e->getFromNode()->getDistFromDriving()<e->getToNode()->getDistFromDriving()){
      if(e->isUsed()==false){
	outfile<<"edge "<<e->getFromNode()->getIndex()<<" "<<e->getToNode()->getIndex()<<endl;
	e->setUsed();
	e->getDualEdge()->setUsed();
	assert(e->getFromNode()->getDistFromDriving()<e->getToNode()->getDistFromDriving());
      }
    }
  }

  outfile.close();
}

void Write_OARSMT(){
	string file_name = "../benchmarks/" + argv1_path  + "/" + argv1_path + ".branch";
	const char * c = file_name.c_str();
  ofstream outfile(c);
  ofstream outfile2("OARSMT_gpl.out");
  //outfile<<"Just a test"<<endl;
  int IDD=0;
  OARST->getDrivingNode()->getRefRGNode()->setIndex(IDD);
  ++IDD;
  //outfile<<"driver "<<OARST->getDrivingNode()->getX()<<" "<<OARST->getDrivingNode()->getY()<<" 440"<<endl; // x, y, driver resistance

  //outfile<<"number_of_sinks "<<(database.GetPinSize()-1)<<endl;

  for(int i=0;i<OARST->getOARG()->getNumNode();++i){
    RGNode* p(0);
    p=OARST->getOARG()->getNode(i);
    assert(p);
    if(p->getType()==_PIN&&p!=OARST->getOARG()->getDrivingNode()){
      assert(p);
      assert(p->getSGNode());
      p->setIndex(IDD);
      ++IDD;
      //outfile<<"sink "<<p->getIndex()<<" "<<p->getX()<<" "<<p->getY()<<" "<<p->getLoading()<<" "<<p->getRequiredTime()<<endl;
    }
  }
  //outfile<<endl;

  //outfile<<"number_of_candidate_nodes "<<(OARST->getOARG()->getNumNode()-database.GetPinSize())<<endl;
  for(int i=0;i<OARST->getOARG()->getNumNode();++i){
    RGNode* p(OARST->getOARG()->getNode(i));
    assert(p);
    if(p->getType()!=_PIN){
      p->setIndex(IDD);
      ++IDD;
      //outfile<<"candidate "<<p->getIndex()<<" "<<p->getX()<<" "<<p->getY()<<endl;
    }
  }
  //outfile<<endl;
  //all nodes
  for(int i=0;i<OARST->getOARG()->getNumNode();++i){
    RGNode* p(OARST->getOARG()->getNode(i));
    assert(p);
    //outfile<<"candidate "<<p->getIndex()<<" "<<p->getX()<<" "<<p->getY()<<endl;
  }
  //outfile<<endl;
  // edges
  IDD = 0;
  std::map<int,int> m;//Sort the index in 0,1,2,3... sequence
  for(size_t i=0;i<OARST->getOARG()->getNumEdge();++i){
      RGEdge* e(OARST->getOARG()->getEdge(i));
      if(e->getFromNode()->getDistFromDriving()>e->getToNode()->getDistFromDriving()){//From bottom to up direction
          //if(e->isUsed()==false){//already used by last loop, don't know what's the useage here
          RGNode* p = e->getFromNode();
          RGNode* p2 = e->getToNode();
          //outfile<<e->getToNode()->getIndex()<<" "<< p2->getX()<<" "<<p2->getY()<<" "<<e->getFromNode()->getIndex()<<endl;
          //e->setUsed();
          //e->getDualEdge()->setUsed();
          assert(e->getFromNode()->getDistFromDriving()>e->getToNode()->getDistFromDriving());
          assert(m.find(e->getFromNode()->getIndex()) == m.end());
          m[e->getFromNode()->getIndex()] = IDD;
          IDD++;
          //}
      }
  }
  m[OARST->getOARG()->getDrivingNode()->getIndex()] = IDD;

  outfile<< "DegreeNumber : "<< OARST->getOARG()->getNumEdge()/2<< endl;//Because dual edges(2 directions)
  for(size_t i=0;i<OARST->getOARG()->getNumEdge();++i){
      RGEdge* e(OARST->getOARG()->getEdge(i));
      if(e->getFromNode()->getDistFromDriving()>e->getToNode()->getDistFromDriving()){
          if(e->isUsed()==false){
              RGNode* p = e->getFromNode();
              RGNode* p2 = e->getToNode();
              //Use to_node to from_node which is bottom to up. This manner enables the first node of the branch to be unique
              outfile<< m[p->getIndex()]<<" "<< p->getX()<<" "<<p->getY()<<" "<<m[p2->getIndex()]<<endl;
              outfile2<< p->getX()<<" "<<p->getY()<<" "<<p2->getX()<<" "<<p2->getY()<<endl;
              e->setUsed();
              e->getDualEdge()->setUsed();
              assert(e->getFromNode()->getDistFromDriving()>e->getToNode()->getDistFromDriving());
          }
      }
  }
  //Root to root itself, because the buffering tool needs this format...
  outfile<< m[OARST->getOARG()->getDrivingNode()->getIndex()]<<" "<< OARST->getOARG()->getDrivingNode()->getX()<<
      " "<<OARST->getOARG()->getDrivingNode()->getY()<<" "<<m[OARST->getOARG()->getDrivingNode()->getIndex()]<<endl;
  outfile2<< OARST->getOARG()->getDrivingNode()->getX()<<" "<<OARST->getOARG()->getDrivingNode()->getY()<<" "<<
      OARST->getOARG()->getDrivingNode()->getX()<<" "<<OARST->getOARG()->getDrivingNode()->getY()<<endl;

  outfile.close();
  outfile2.close();
}


void Write_Slack(){
  string delayfilename = "../benchmarks/" + argv1_path + "/timing.txt";
  string sinkfilename = "./benchOARST_Timing/" + argv1_path + ".net";
  string slackfilename = "../benchmarks/" + argv1_path + "/slack.txt";
  ifstream delayfile(delayfilename.c_str());
  ifstream sinkfile(sinkfilename.c_str());
  ofstream slackfile(slackfilename.c_str(), ios::app);
  double WNS = 0;
  string WNS_position;
  slackfile<<"Start\n";
  while (sinkfile.good()){
	  string sinkline;
	  getline (sinkfile,sinkline);
	  if(sinkline.find("Sink")!=-1 && sinkline.find("Sinks")==-1){
		  string line1, line2, line3;
		  getline (sinkfile,line1);
		  getline (sinkfile,line2);
		  getline (sinkfile,line3);
		  string position = line1.substr(line1.find("(")+1, line1.find(")")-line1.find("(")-1);
		  slackfile<<"Location:"<<position<<endl;
		  position = position.substr(0, position.find(","))+", "+position.substr(position.find(",")+1);
		  delayfile.clear();
		  delayfile.seekg(0, ios::beg);
		  while(delayfile.good()){
			  string delayline;
			  getline(delayfile, delayline);
			  if(delayline.find(position)!=-1){
				  double delay = atof(delayline.substr(delayline.find(" is ")+4).c_str());
				  double requiretime = atof(line3.substr(line3.find("Req")+4).c_str());
				  double slack = requiretime - delay;
				  ostringstream slack_s;
				  slack_s << slack;
				  slackfile<<"Slack:"+slack_s.str()+"\n";
				  if (WNS > slack){
					  WNS = slack;
					  WNS_position = position;
				  }
			  }
		  }
	  }
  }
  ostringstream WNS_s;
  WNS_s<<WNS;
  slackfile<<"WNS_Position:"<<WNS_position<<"\nWNS:"<<WNS_s.str()<<"\n";
  sinkfile.close();
  slackfile.close();
  delayfile.close();
  
  if (WNS > BEST_WNS){
    BEST_WNS = WNS;
	string copy = "cp ../benchmarks/" + argv1_path + "/" + argv1_path + ".branch ../benchmarks/" + argv1_path + "/" + argv1_path + ".branch.best";
	const char * copy_c = copy.c_str();
	system(copy_c);
  }
}


#ifdef LINUX_MODE
int main(int argc, char **argv)
{
    cout<<"\nargv[1] = "<<argv[1]<<endl;
	string argv1(argv[1]);
	string argv4(argv[4]);

	argv1_path = argv[1];	//RC2_T
	argv4_slew = argv[4];		//80000
    cout<<"argv[2] = "<<argv[2]<<endl;	
	
	string deleteLog = "rm ../benchmarks/" + argv1_path + "/slack.txt";
	const char * deleteLog_c = deleteLog.c_str();
	system(deleteLog_c);

#ifdef BUF_DRIVEN
    if(argc>3)
        cout<<"argv[3] = "<<argv[3]<<endl;
#endif

    cout<<"\n-------------Read file-----------\n";
	string netfilename = "benchOARST_Timing/" + argv1_path + ".net";
  openfile(netfilename);

#ifdef BUF_DRIVEN
  openBufFile(argv[3]);
#endif

#ifdef DEBUG_ANALYSIS_RUN_TIME
  RunTime->program_start = clock();
#endif

  cout<<"\n-------------init data-----------\n";
  Init_Data();

  cout<<"\n-------------Grow trunk-----------\n";
  Grow_Trunk();

  cout<<"\n-------------Route OASG-----------\n";
  Route_OASG();

  cout<<"\n-------------Build OARG-----------\n";
  Build_OARG();

  cout<<"\n-------------Timing Analysis-----------\n";
  Timing_Analysis();

  cout<<"\n-------------Slack Analysis-----------\n";
  Slack_Analysis();

  cout<<"\n-------------Post Processing : Refine Timging-----------\n";
  Post_Processing();

  cout<<"\n-------------Write input file for bobrsmt-----------\n";
  //Write_FBI(); These two cannot get together because the edge will be set used
  Write_OARSMT();

  cout<<"\n-------------Run Buffering-----------\n";
  //string command = "../buffering_tool/roo ../benchmarks/" + " " + argv1_path + "  " + argv4_slew;
  string command = "../buffering_tool/roo ../benchmarks/ " + argv1_path + "  " + argv4_slew;
  const char * command_c = command.c_str();
  system(command_c);

  cout<<"\n-------------Save Slack Information-----------\n";
  Write_Slack();

  for(int i=0; i<5; i++){  

      secondTime=true;

      elmoreDelay* elmoreModel = new elmoreDelay();
      Initial_WD = _UNSET;
      WD_When_Min_WNS = _UNSET;
      Min_WL = _INFINITE;
      Initial_WNS = _UNSET;
      Min_WNS = -1*(_INFINITE);
      Min_WNS_In_Times = _UNSET;
      WL_When_Min_WNS = _UNSET;
      Initial_WL = _UNSET;

      Old_WNS = _UNSET;
      New_WNS = _UNSET;
      Redirect_Times = 0;
      DoRedirect = true;
      fail_times = 0;
      fail_upper_bound = 10;

      cout<<"\n-------------Second init data-----------\n";
      Init_Data();

      cout<<"\n-------------Grow trunk-----------\n";
      Grow_Trunk();

      cout<<"\n-------------Route OASG-----------\n";
      Route_OASG();

      cout<<"\n-------------Build OARG-----------\n";
      Build_OARG();

      cout<<"\n-------------Timing Analysis-----------\n";
      Timing_Analysis();

      cout<<"\n-------------Slack Analysis-----------\n";
      Slack_Analysis();

      cout<<"\n-------------Post Processing : Refine Timging-----------\n";
      Post_Processing();

      cout<<"\n-------------Write input file for bobrsmt-----------\n";
      Write_OARSMT();

      cout<<"\n-------------Run Buffering-----------\n";
      command = "../buffering_tool/roo ../benchmarks/ " + argv1_path + "  " + argv4_slew;
      command_c = command.c_str();
      system(command_c);

      cout<<"\n-------------Save Slack Information-----------\n";
      Write_Slack();
  }
  string copy = "cp ../benchmarks/" + argv1_path + "/" + argv1_path + ".branch.best ../benchmarks/" + argv1_path + "/" + argv1_path + ".branch";
  const char * copy_c = copy.c_str();
  system(copy_c);

#ifdef DEBUG_ANALYSIS_RUN_TIME
  RunTime->program_end = clock();
#endif
  //cout<<"\n--[Write Timing Constrain of each sink]\n";
  //cout<<"delay * 1\n";
  //OARST->GenTimingConstrain(argv[2],database,1,elmoreModel);

#ifdef DEBUG_ANALYSIS_RUN_TIME
  RunTime->others_start = clock();
#endif

  cout<<"\n-------------Check Correction-----------\n";
  CheckCorrection();

  cout<<"\n-------------Display WL Result-----------\n";
  Display_WL_Result();

#ifdef DEBUG_ANALYSIS_RUN_TIME
  RunTime->others_end = clock();
#endif

  cout<<"\n-------------Display Run-Time Result-----------\n";
  PrintRunTimeInfo();

  cout<<"\n-------------Write Result to Table-----------\n";
  Write_Table(argv[2],argv[1]);

  //cout<<"\n-------------Write input file for FBI-----------\n";
  //Write_FBI();

  //cout<<"\n-------------Write input file for bobrsmt-----------\n";
  //Write_OARSMT();

  return 0;
}
#endif
