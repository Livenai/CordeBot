//******************************************************************
// 
//  Generated by RoboCompDSL
//  
//  File name: AGMExecutive.ice
//  Source: AGMExecutive.idsl
//
//******************************************************************
#ifndef ROBOCOMPAGMEXECUTIVE_ICE
#define ROBOCOMPAGMEXECUTIVE_ICE
#include <AGMWorldModel.ice>#include <Planning.ice>
module RoboCompAGMExecutive
{
	exception Locked{ };
	exception OldModel{ };
	exception InvalidChange{ };
	interface AGMExecutive
	{
		void activate ();
		void addSelfEdge (int nodeid, string edgeType, StringDictionary attributes);
		void broadcastModel ();
		void broadcastPlan ();
		void deactivate ();
		void delSelfEdge (int nodeid, string edgeType);
		void edgeUpdate (RoboCompAGMWorldModel::Edge e);
		void edgesUpdate (RoboCompAGMWorldModel::EdgeSequence es);
		void getData (out RoboCompAGMWorldModel::World world, out string target, out RoboCompPlanning::Plan plan);
		RoboCompAGMWorldModel::Edge getEdge (int srcIdentifier, int dstIdentifier, string label);
		RoboCompAGMWorldModel::World getModel ();
		RoboCompAGMWorldModel::Node getNode (int identifier);
		void setMission (string path);
		void structuralChangeProposal (RoboCompAGMWorldModel::World w, string sender, string log) throws Locked,OldModel,InvalidChange;
		void symbolUpdate (RoboCompAGMWorldModel::Node n);
		void symbolsUpdate (RoboCompAGMWorldModel::NodeSequence ns);
	};
};

#endif
