#ifndef ELMOREDELAY_H //Avoid duplicate definition
#define ELMOREDELAY_H

#include "OARST.h"


//-------------------------------------------------------------------------------------------------------
//refer to OARG's information (rectilinear graph), then calculate the delay of each pin
//-------------------------------------------------------------------------------------------------------
//method to compute delay: trace from each node to root (notice that every node, not only leaf), 
//						   and calculate downstream capacitance by incrementally adding
//
//	                      for each pin, applying elmore delay model with the formula (rL)(cL)/2
//									   (L means the distance from this leaf to driving)
//									   then trace to root, and plus ths delay occured by sub-tree
//-------------------------------------------------------------------------------------------------------

class elmoreDelay
{
public:
	elmoreDelay() : OASG(NULL), OARG(NULL), Worse_Delay_Sink_In_OASG(NULL), Worse_Delay_Sink_In_OARG(NULL)
	{		
		unit_r   = 0.076; //(ohms/um)
		unit_c   = 0.118; //(Ff/um)
		Rd       = 440;  //(ohms) //1440 is too big
		//Cloading = 500;   //(Ff)
		Cloading = 1;   //(Ff)
		Driver_Arrival_Time = 0; //(ps)


		cout<<"\n--[Init Parameters of Elmore Delay Model]\n";
		cout<<"default: unit_r   = "<<unit_r<<" (ohms/um)\n";
		cout<<"default: unit_c   = "<<unit_c<<" (Ff/um)\n";
		cout<<"default: Rd       = "<<Rd<<"  (ohms)\n";
		cout<<"default: Cloading = "<<Cloading<<"  (Ff)\n";
		cout<<"default: Driver's arrival time = "<<Driver_Arrival_Time<<"  (ps)\n";
	}
	void show_setting()
	{
		cout<<"\n--[Display Parameters of Elmore Delay Model]\n";
		cout<<"unit_r   = "<<unit_r<<" (ohms/um)\n";
		cout<<"unit_c   = "<<unit_c<<" (Ff/um)\n";
		cout<<"Rd       = "<<Rd<<"  (ohms)\n";
		cout<<"Driver's arrival time = "<<Driver_Arrival_Time<<"  (ps)\n";
	}
	double get_Loading() { return Cloading; }
	void set_Rd(double val) { Rd=val; }
	void set_Unit_R(double val) { unit_r=val; }
	void set_Unit_C(double val) { unit_c=val; }
	double get_Rd() { return Rd; }
	double get_Unit_R() { return unit_r; }
	double get_Unit_C() { return unit_c; }
	void set_Driver_Arrival_Time(double val) { Driver_Arrival_Time=val; }
	double get_Driver_Arrival_Time() { return Driver_Arrival_Time; }

	OARectilinearGraph* getOARG() { return OARG; }
	OASpanningGraph* getOASG() { return OASG; }
    void setOARG(OARectilinearGraph* OARG) { this->OARG = OARG; }
	void setOASG(OASpanningGraph* OASG) { this->OASG = OASG; }
	

	void Restore_Delay_Info_OASG();
	void Restore_Delay_Info_OARG();
	//=========================================

	double Compute_OARG_Delay(Database &data); //return average sink delay
	void Compute_Sink_Delay(RGNode* Sink);
	void Compute_OARG_DownstreamCap();

	//=========================================

	double Compute_OASG_Delay(Database &data); //return average sink delay
	void Compute_Sink_Delay(SGNode* Sink);
	void Compute_OASG_DownstreamCap();
#ifdef PRIORITY_BASED
	void computeHalfRC();
	double getHalfRC();
	double Compute_OASG_Priority(Database &data); //return nullified priority

	void setWorstPrioritySinkInOASG(SGNode* N) { Worst_Priority_Sink_In_OASG=N; }
	SGNode* getWorstPrioritySinkInOASG() { return Worst_Priority_Sink_In_OASG; }
	void setWorstPrioritySinkInOARG(RGNode* N) { Worst_Priority_Sink_In_OARG=N; }
	RGNode* getWorstPrioritySinkInOARG() { return Worst_Priority_Sink_In_OARG; }
	void setBestPrioritySinkInOASG(SGNode* N) { Best_Priority_Sink_In_OASG=N; }
	SGNode* getBestPrioritySinkInOASG() { return Best_Priority_Sink_In_OASG; }
	void setBestPrioritySinkInOARG(RGNode* N) { Best_Priority_Sink_In_OARG=N; }
	RGNode* getBestPrioritySinkInOARG() { return Best_Priority_Sink_In_OARG; }
	void setOrgWorstPriority(double p) { orgWorstPriority = p ; }
	double getOrgWorstPriority() { return orgWorstPriority ; }
	void setOrgBestPriority(double p) { orgBestPriority = p ; }
	double getOrgBestPriority() { return orgBestPriority; }
#endif

#ifdef BUF_DRIVEN
	void setCapUpperBound(double ub) { capUpperBound=ub; }
	double getCapUpperBound() { return capUpperBound; }
	void Compute_OASG_DownstreamCap_BUF();
#endif


	//=========================================
	void setWorseDelaySinkInOASG(SGNode* N) { Worse_Delay_Sink_In_OASG=N; }
	SGNode* getWorseDelaySinkInOASG() { return Worse_Delay_Sink_In_OASG; }
	void setWorseDelaySinkInOARG(RGNode* N) { Worse_Delay_Sink_In_OARG=N; }
	RGNode* getWorseDelaySinkInOARG() { return Worse_Delay_Sink_In_OARG; }

private:
	OARectilinearGraph* OARG;
	OASpanningGraph* OASG;

	SGNode* Worse_Delay_Sink_In_OASG;
	RGNode* Worse_Delay_Sink_In_OARG;

	//delay-related parameter 
	double unit_r;
	double unit_c;
	double Rd;
	double Driver_Arrival_Time;
	double Cloading;
#ifdef PRIORITY_BASED
	double half_RC;
	SGNode* Worst_Priority_Sink_In_OASG;
	RGNode* Worst_Priority_Sink_In_OARG;
	SGNode* Best_Priority_Sink_In_OASG;
	RGNode* Best_Priority_Sink_In_OARG;
	double orgWorstPriority ;
	double orgBestPriority ;
#endif

#ifdef BUF_DRIVEN
	double capUpperBound;
#endif
	friend void readfile(char* filename,Database& database,elmoreDelay* elmoreModel);
};






#endif
