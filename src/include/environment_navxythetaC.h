/*
  This is the environment for running motion primitive in continuous space.
  This environment is used for 2D planning.(xytheta)
  It is adapted from environment_xythetalat in sbpl. Hash entry structure
  is modified into double value version as well as some function interfaces.

  Double-precision x,y,theta will be mapped into a cell. Then the hash key
  will be calcultated by the integer value and push into the hash table.

  Version: 1.0
  Date: Dec/18/2017
  Author: Wei Du
*/

// The function CONTXY2DISC & ContTheta2DiscNew transform the contiuous value
// of pose to discretized vale and store them.



#ifndef __ENVIRONMENT_NAVXYTHETAC_H_
#define __ENVIRONMENT_NAVXYTHETAC_H_

#include <cstdio>
#include <vector>
#include <sstream>
#include <ctime>
#include <fstream>
#include <iostream>

#include "dubins.h"
#include <sbpl/discrete_space_information/environment.h>

#include <sbpl/utils/utils.h>

// Define to test against in client code. Signals that Set2DBlockSize and
// Set2DBucketSize are available in EnvironmentNAVXYTHETALATTICE
#define SBPL_CUSTOM_2D_OPTIONS 1

// eight-connected grid
#define NAVXYTHETALAT_DXYWIDTH 8

#define ENVNAVXYTHETAC_DEFAULTOBSTHRESH 254   //see explanation of the value below
//maximum number of states for storing them into lookup (as opposed to hash)
// #define SBPL_XYTHETAC_MAXSTATESFORLOOKUP 100000000
#define SBPL_XYTHETAC_MAXSTATESFORLOOKUP 1  // now using hashtable



// this could be changed
// theta increases as we go counterclockwise
// number of theta values - should be power of 2
// ###############################################
// ###############################################
#define NAVXYTHETAC_THETADIRS 16
// ###############################################
// ###############################################
// it was setup in config.h file to decide whether get into DEBUG mode

#define ANGLERESOLUTION M_PI/16

//number of actions per x,y,theta state
#define NAVXYTHETAC_DEFAULT_ACTIONWIDTH 5
#define NAVXYTHETAC_COSTMULT_MTOMM 1000

class CMDPSTATE;
//class MDPConfig;
class SBPL2DGridSearch;

struct EnvNAVXYTHETACAction_t{
    unsigned char aind; // index of the action (unique for given starttheta)
    char starttheta;
    char dX;            // one cell equals 1
    char dY;
    char endtheta;
    unsigned int cost;

    // int X, int Y
    std::vector<sbpl_2Dcell_t> intersectingcellsV;

    // start at 0,0, starttheta and end at endcell in continuous
    // domain with half-bin less to account for 0,0 start
    std::vector<sbpl_xy_theta_pt_t> intermptV;

    //start at 0,0, starttheta and end at endcell in discrete domain
    std::vector<sbpl_xy_theta_cell_t> interm3DcellsV;

    int motprimID;
    double turning_radius;
};


struct EnvNAVXYTHETACHashEntry_t{
    int stateID;
    int parentStateID;
    double valid_children_ratio;
    double X;
    double Y;
    double Theta;
    int iteration;
};

struct SBPL_xytheta_mprimitive{
    int motprimID;
    unsigned char starttheta_c;
    int additionalactioncostmult;
    sbpl_xy_theta_cell_t endcell;
    double turning_radius;
    //intermptV start at 0,0,starttehta and end at endcell in continuous
    //domain with half-bin less to account for 0,0 start
    std::vector<sbpl_xy_theta_pt_t> intermptV;
};

//variables that dynamically change (e.g., array of states, ..)
struct EnvironmentNAVXYTHETAC_t{
    int startstateid;
    int goalstateid;

    bool bInitialized;
    // any additional variables
};


//configuration parameters
struct EnvNAVXYTHETACConfig_t{
    int EnvWidth_c;
    int EnvHeight_c;
    int NumThetaDirs;
    int StartXi;
    int StartYi;
    double StartX_c;
    double StartY_c;
    double StartTheta;
    double EndX_c;
    double EndY_c;
    double EndTheta;
    unsigned char** Grid2D;

    std::vector<double> ThetaDirs;
    double StartTheta_rad;
    double EndTheta_rad;
    double min_turning_radius_m;
    double min_stepsize;

    // tolerance of goal
    double tol_x;
    double tol_y;
    double tol_theta;

    // the value at which and above which cells are obstacles in the maps sent from outside
    // the default is defined above
    unsigned char obsthresh;

    /* the value at which and above which until obsthresh (not including it)
       cells have the nearest obstacle at distance smaller than or equal to
       the inner circle of the robot. In other words, the robot is definitely
       colliding with the obstacle, independently of its orientation
    */
    unsigned char cost_inscribed_thresh;

    /* the value at which and above until cost_inscribed_thresh (not including it) cells
       may have a nearest obstacle within the distance that is in between
       the robot inner circle and the robot outer circle
       any cost below this value means that the robot will NOT collide with any
       obstacle, indenpendently of its orientation.
    */

    int cost_possibly_circumscribed_thresh;

    double nominalvel_mpersecs;

    double timetoturn45degsinplace_secs;

    double cellsize_m;

    double R_times;

    int dXY[NAVXYTHETALAT_DXYWIDTH][2];

    // array of actions, ActionsV[i][j] -jth action for sourcetheta = i
    EnvNAVXYTHETACAction_t** ActionsV;

    //PredActionsV[i] -vector of pointers to the actions that result in a state with theta = i
    std::vector<EnvNAVXYTHETACAction_t*>* PredActionsV;

    int actionwidth; // number of motion primitives

    std::vector<SBPL_xytheta_mprimitive> mprimV;

    std::vector<sbpl_2Dpt_t> FootprintPolygon;
};

class EnvNAVXYTHETAC_InitParms{
public:
    unsigned int numThetas;
    const unsigned char* mapdata;
    double startx;
    double starty;
    double starttheta;
    double goalx;
    double goaly;
    double goaltheta;
    double goaltol_x;
    double goaltol_y;
    double goaltol_theta;
};



class EnvironmentNAVXYTHETAC : public DiscreteSpaceInformation{
public:

     EnvironmentNAVXYTHETAC();
    ~EnvironmentNAVXYTHETAC();
//------------------- part1 initialize ---------------------------------
    /*
      Initialization of environment from file. Using .cfg file.
      It also takes the primeter of the robot with respect to some
      reference point centered at x=0,y=0 and orientation = 0.
      The perimeter is defined in meters as a sequence of vertices of
      a polygon defining the perimeter.
      Motion primitives file defines the motion primitives available to the robot
    */
    // just to make sure environment is in right condition //check
    virtual bool InitializeEnv(const char* sEnvFile, const std::vector<sbpl_2Dpt_t>& perimeterptsV,
                               const char* sMotPrimFile);

    //its a pure virtual function in from class
    bool InitializeEnv(const char* sEnvFile);

    // read in map distance transform data
    //virtual void ReadinTransform(FILE* vfile);
  
    // read in voronoi data
    //virtual void ReadinVoronoi(FILE* vfile);

    //initialization of MDP data structure // check
    virtual bool InitializeMDPCfg(MDPConfig *MDPCfg);

    virtual void PrintState(int stateID, bool bVerbose, FILE* fOut);

    // sets start in meters/radians
    virtual int SetStart(double x, double y, double theta); // check set function
    //sets goal in meters/radians
    virtual int SetGoal(double x, double y, double theta);

    //Sets goal tolerance   // not in use yet
    virtual void SetGoalTolerance(double tol_x, double tol_y, double tol_theta);

    virtual bool IsWithinGoalRegion(double x, double y, double theta);

    virtual int ContTheta2DiscNew(double theta) const;

    virtual int ContTheta2DiscFromSet(double theta) const;

    virtual int normalizeDiscAngle(int theta) const;

    virtual bool IsWithinMapCell(int X, int Y);

    /*
     * returns false if robot intersects obstacles or lies outside of
     * the map. Note this is pretty expensive operation since it computes the
     * footprint of the robot based on its x,y,theta
     */
    virtual bool IsValidConfiguration(double X, double Y, double Theta);


//----------------- part2 run --------------------------------------------

    // heuristic estimate from state FromStateID to state ToStateID
    virtual int GetFromToHeuristic(int FromStateID, int ToStateID);

    // returns state coordinates of state with ID=stateID
    virtual void GetCoordFromState(int stateID, double& x, double& y, double& theta) const;

    // returns stateID for a state with coords x,y,theta
    virtual int GetStateFromCoord(double x, double y, double theta);

    // heuristic estimate from start state to state with stateID
    virtual int GetStartHeuristic(int stateID);

    // heuristic estimate from state with stateID to goal state
    virtual int GetGoalHeuristic(int stateID);              // Dijkstra Heuristic
    
    virtual int GetPenaltyGoalHeuristic(int stateID);       // Dijkstra Heuristic + Penalty heuristic


    /* depending on the search used, it may call GetSuccs function or GetPreds function
       (forward search/backward search). At least one of these functions should be
       implemented.
       pure functions from parent class
    */
    virtual void GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV);
    // virtual void GetSuccs_t(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV);

    virtual void GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV);

    virtual void GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV,
                          std::vector<EnvNAVXYTHETACAction_t*>* actionindV = NULL);

    // virtual void GetSuccs_t(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV,
                        //   std::vector<EnvNAVXYTHETACAction_t*>* actionindV = NULL);

    virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state);


    /*
      by default the heuristics are up-to-date, but in some cases, the
      heuristics are computed only when really needed. For example,
      xytheta environment uses 2D gridsearch as heuristics, and then
      the number of times heuristics are re-computed which is an expensive operation.
      if bGoalHeuristics == true, then it updates goal heuristics, otherwise it updates start heuristics.
    */
    virtual void EnsureHeuristicsUpdated(bool bGoalHeuristics);     // check

    /*
      converts a path given by stateIDs into a sequence of coordinates. Note that since motion primitives
      are short actions represented as a sequence of points, the path returned by this function contains
      much more points than the number of points in the input path. The returned coordinates are in
      meters, meters, radians
    */
    virtual void ConvertStateIDPathintoXYThetaPath(std::vector<int>* stateIDPath,               // check
                                                   std::vector<sbpl_xy_theta_pt_t>* xythetaPath);

    // returns the number of states (hashentries) created        // check
    virtual int SizeofCreatedEnv();

    // prints environment config file  //check
    virtual void PrintEnv_Config(FILE* fOut);

    // prints time statistics     //check
    virtual void PrintTimeStat(FILE* fOut);

    virtual double DiscTheta2ContNew(int theta) const;

    virtual double DiscTheta2ContFromSet(int theta) const;

    virtual void UpdateParent(int childStateID, int parentStateID);

    virtual void SetupR(double r);


    EnvNAVXYTHETACConfig_t EnvNAVXYTHETACCfg;

protected:


    virtual int GetActionCost(double SourceX, double SourceY, double SourceTheta, EnvNAVXYTHETACAction_t* action);

    // hash talbe of size x_size*y_size. Maps from coords ot stateId
    int HashTableSize;


    EnvironmentNAVXYTHETAC_t EnvNAVXYTHETAC;
    std::vector<sbpl_xy_theta_cell_t> affectedsuccstatesV;  // arrays of states whose outgoing actions cross cell 0,0
    std::vector<sbpl_xy_theta_cell_t> affectedpredstatesV;  // arrays of states whose outgoing actions cross cell 0,0
    int iteration;
    int blocksize; // 2D block size
    int bucketsize; // 2D bucket size

    bool bUseNonUniformAngles;

    //2D search for heuristic computations
    bool bNeedtoRecomputeStartHeuristics; // set whenever grid2Dsearchfromstart needs to be re-executed
    bool bNeedtoRecomputeGoalHeuristics; // set whenever grid2Dsearchfromgoal needs to be re-executed
    SBPL2DGridSearch* grid2Dsearchfromstart; // computes h-values that estimates distances from start x,y to all  cells
    SBPL2DGridSearch* grid2Dsearchfromgoal;  // computes h-valuse that estimates distances to goal x,y from all cells

    // continuous edition of hashtable
    std::vector<EnvNAVXYTHETACHashEntry_t*>* Coord2StateIDHashTable;
    // stores stateID of states in the same cell;
    std::vector<EnvNAVXYTHETACHashEntry_t*>* NNtable;

    // vector that maps fromo stateID to coords
    std::vector<EnvNAVXYTHETACHashEntry_t*> StateID2CoordTable;

	std::ofstream output;

    /*
      This is a lookup table. Different from _hash method.
      Initialized in the Envinitialization function(with maxsize, which is
      calculated from parameters given by map) and modified in
      CreateNewHashEntry_lookup function
    */
    EnvNAVXYTHETACHashEntry_t** Coord2StateIDHashTable_lookup;

    virtual unsigned int GETHASHBIN(unsigned int X, unsigned int Y, unsigned int Theta);

//    virtual EnvNAVXYTHETACHashEntry_t* GetHashEntry_hash(int X, int Y, int Theta);
//    virtual EnvNAVXYTHETACHashEntry_t* CreateNewHashEntry_hash(int X, int Y, int Theta);
//    virtual EnvNAVXYTHETACHashEntry_t* GetHashEntry_lookup(int X, int Y, int Theta);
//    virtual EnvNAVXYTHETACHashEntry_t* CreateNewHashEntry_lookup(int X, int Y, int Theta);

    virtual EnvNAVXYTHETACHashEntry_t* GetHashEntry_hash(double X, double Y, double Theta);
    virtual EnvNAVXYTHETACHashEntry_t* CreateNewHashEntry_hash(double X, double Y, double Theta);
    virtual EnvNAVXYTHETACHashEntry_t* GetHashEntry_lookup(double X, double Y, double Theta);
    virtual EnvNAVXYTHETACHashEntry_t* CreateNewHashEntry_lookup(double X, double Y, double Theta);


    // pointers to functions
//    EnvNAVXYTHETACHashEntry_t* (EnvironmentNAVXYTHETAC::*GetHashEntry)(int X, int Y, int Theta);
//    EnvNAVXYTHETACHashEntry_t* (EnvironmentNAVXYTHETAC::*CreateNewHashEntry)(int X, int Y, int Theta);
    EnvNAVXYTHETACHashEntry_t* (EnvironmentNAVXYTHETAC::*GetHashEntry)(double X, double Y, double Theta);
    EnvNAVXYTHETACHashEntry_t* (EnvironmentNAVXYTHETAC::*CreateNewHashEntry)(double X, double Y, double Theta);


    // check
    virtual void InitializeEnvironment();
    virtual bool InitGeneral(std::vector<SBPL_xytheta_mprimitive>* motionprimitiveV);
    virtual void InitializeEnvConfig(std::vector<SBPL_xytheta_mprimitive>* motionprimitiveV);

    virtual void ComputeHeuristicValues();

    // check
    virtual void PrintHashTableHist(FILE* fOut);

    virtual void SetConfiguration(int width, int height,
    // check                      /* if mapdata is NULL the grid is initialized to all freespace */
                                  const unsigned char* mapdata,
                                  int startx, int starty, int starttheta,
                                  int goalx, int goaly, int goaltheta,
                                  double cellsize_m, double nominalvel_mpersecs, double timetoturn45degsinplace_secs,
                                  const std::vector<sbpl_2Dpt_t> & roobt_perimeterV);
    //check
    virtual void ReadConfiguration(FILE* fCfg);
    virtual bool ReadMotionPrimitives(FILE* fMotPrims);
    virtual bool ReadinMotionPrimitive(SBPL_xytheta_mprimitive* pMotPrim, FILE* fIn);
    virtual bool ReadinCell(sbpl_xy_theta_cell_t* cell, FILE *fIn);
    virtual bool ReadinPose(sbpl_xy_theta_pt_t* pose, FILE* fIn);

    virtual void DeprecatedPrecomputeActions();
    virtual void ComputeReplanningData();
    virtual void ComputeReplanningDataforAction(EnvNAVXYTHETACAction_t* action);
    virtual void PrecomputeActionswithCompleteMotionPrimitive(std::vector<SBPL_xytheta_mprimitive>* motionprimitiveV);
    virtual void RemoveSourceFootprint(sbpl_xy_theta_pt_t sourcepose, std::vector<sbpl_2Dcell_t>* footprint);
    virtual void RemoveSourceFootprint(sbpl_xy_theta_pt_t sourcepose, std::vector<sbpl_2Dcell_t>* footprint,
                                       const std::vector<sbpl_2Dpt_t>& FootprintPolygon);



    int GoTo(int parentID, int sampleStateID);

    int GoTo(int parentID, int sampleStateID, std::vector<sbpl_xy_theta_pt_t> &subpath, double& length);
    virtual bool IsValidCell(int X, int Y);

    //virtual int GetTransformValue(int X, int Y);

    //virtual int GetVoronoiValue(int X, int Y);

    virtual double EuclideanDistance_m(double X1, double Y1, double X2, double Y2);

    // pure virtual function from environment.h
    virtual void SetAllPreds(CMDPSTATE* state){/*not defined */ state = NULL;}
};

#endif

