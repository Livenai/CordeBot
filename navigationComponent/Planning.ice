//******************************************************************
// 
//  Generated by RoboCompDSL
//  
//  File name: Planning.ice
//  Source: Planning.idsl
//
//******************************************************************
#ifndef ROBOCOMPPLANNING_ICE
#define ROBOCOMPPLANNING_ICE
module RoboCompPlanning
{
	exception ServerException{string what;};
	sequence <string> StringVector;
	dictionary <string,sstring> StringDictionary;
	struct Action
	{
		string name;
		StringVector symbols;
	};
	sequence <Action> ActionSequence;
	struct Plan
	{
		ActionSequence actions;
		float cost;
	};
	interface PlanReceiver
	{
		void setPlan (Plan p);
	};
	interface Planning
	{
		bool getNextAction (string Problem, out Plan solution) throws ServerException;
		bool getSolution (string Domain, string Problem, out Plan solution) throws ServerException;
	};
	interface PeleaAgent
	{
		void stateChanged (StringDictionary state);
	};
};

#endif
