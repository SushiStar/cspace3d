/*
  This is the continuous version of an environment for navigation in x,y,theta
  2D map.
  This environment is adapted from environment_navxyTHETA LAT.

  Since motion of a robot are not gauranteed to end in the center of a cell.
  Some
  Transformations are made to map a continuous space state into a cell so as to
  get stateID for planning.

  Version: 1.0
  Date: Dec/18/2017
  Author: Wei Du
*/

#include <cmath>
#include <cstring>
#include <ctime>

#include "include/environment_navxythetaC.h"

#include <sbpl/utils/2Dgridsearch.h>
#include <sbpl/utils/key.h>
#include <sbpl/utils/mdp.h>
#include <sbpl/utils/mdpconfig.h>

#include <iostream>

#if TIME_DEBUG
static clock_t time3_addallout = 0;
static clock_t time_gethash = 0;
static clock_t time_createhash = 0;
static clock_t time_getsuccs = 0;
#endif

static long int checks = 0;

#define PENALTY 1

#define XYTHETA2INDEX(X, Y, THETA)                \
    (THETA + X * EnvNAVXYTHETACCfg.NumThetaDirs + \
     Y * EnvNAVXYTHETACCfg.EnvWidth_c * EnvNAVXYTHETACCfg.NumThetaDirs)

#if !defined(MAX)
#define MAX(A, B) ((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define MIN(A, B) ((A) < (B) ? (A) : (B))
#endif

// hash function
static unsigned int inthash(unsigned int key) {
    key += (key << 12);
    key ^= (key >> 22);
    key += (key << 4);
    key ^= (key >> 9);
    key += (key << 10);
    key ^= (key >> 2);
    key += (key << 7);
    key ^= (key >> 12);
    return key;
}

EnvironmentNAVXYTHETAC::EnvironmentNAVXYTHETAC() {
    EnvNAVXYTHETACCfg.obsthresh = ENVNAVXYTHETAC_DEFAULTOBSTHRESH;
    // the value that pretty much makes it disabled
    EnvNAVXYTHETACCfg.cost_inscribed_thresh = EnvNAVXYTHETACCfg.obsthresh;
    // the value that pretty much makes it disables
    EnvNAVXYTHETACCfg.cost_possibly_circumscribed_thresh = -1;

    grid2Dsearchfromstart = NULL;
    grid2Dsearchfromgoal = NULL;
    iteration = 0;
    bucketsize = 0;  // fixed bucket size
    blocksize = 1;
    bUseNonUniformAngles = false;

    EnvNAVXYTHETAC.bInitialized = false;

    EnvNAVXYTHETACCfg.actionwidth = NAVXYTHETAC_DEFAULT_ACTIONWIDTH;

    EnvNAVXYTHETACCfg.NumThetaDirs = NAVXYTHETAC_THETADIRS;

    // no memory allocated in cfg yet
    EnvNAVXYTHETACCfg.Grid2D = NULL;
    EnvNAVXYTHETACCfg.ActionsV = NULL;
    EnvNAVXYTHETACCfg.PredActionsV = NULL;

    output.open("debug_state.txt");
}

EnvironmentNAVXYTHETAC::~EnvironmentNAVXYTHETAC() {
    SBPL_PRINTF("destorying XYTHETAC\n");
    if (grid2Dsearchfromstart != NULL) {
        delete grid2Dsearchfromstart;
    }
    grid2Dsearchfromstart = NULL;

    if (grid2Dsearchfromgoal != NULL) {
        delete grid2Dsearchfromgoal;
    }
    grid2Dsearchfromgoal = NULL;

    if (EnvNAVXYTHETACCfg.Grid2D != NULL) {
        for (int x = 0; x < EnvNAVXYTHETACCfg.EnvWidth_c; ++x) {
            delete[] EnvNAVXYTHETACCfg.Grid2D[x];
        }
        delete[] EnvNAVXYTHETACCfg.Grid2D;
        EnvNAVXYTHETACCfg.Grid2D = NULL;
    }

    // delete actions
    if (EnvNAVXYTHETACCfg.ActionsV != NULL) {
        for (int tind = 0; tind < EnvNAVXYTHETACCfg.NumThetaDirs; ++tind) {
            delete[] EnvNAVXYTHETACCfg.ActionsV[tind];
        }
        delete[] EnvNAVXYTHETACCfg.ActionsV;
        EnvNAVXYTHETACCfg.ActionsV = NULL;
    }
    if (EnvNAVXYTHETACCfg.PredActionsV != NULL) {
        delete[] EnvNAVXYTHETACCfg.PredActionsV;
        EnvNAVXYTHETACCfg.PredActionsV = NULL;
    }

    output.close();
}

bool EnvironmentNAVXYTHETAC::InitializeEnv(
    const char* sEnvFile, const std::vector<sbpl_2Dpt_t>& perimeterptsV,
    const char* sMotPrimFile) {
    EnvNAVXYTHETACCfg.FootprintPolygon = perimeterptsV;

    SBPL_INFO("InitializeEnv start: sEnvFile=%s sMotPrimFile=%s\n", sEnvFile,
              sMotPrimFile);
    fflush(stdout);

    FILE* fCfg = fopen(sEnvFile, "r");
    if (fCfg == NULL) {
        std::stringstream ss;
        ss << "ERROR: unable to open " << sEnvFile;
        throw SBPL_Exception(ss.str());
    }

    ReadConfiguration(fCfg);
    fclose(fCfg);

    if (sMotPrimFile != NULL) {
        FILE* fMotPrim = fopen(sMotPrimFile, "r");
        if (fMotPrim == NULL) {
            std::stringstream ss;
            ss << "ERROR: unable to open " << sMotPrimFile;
            throw SBPL_Exception(ss.str());
        }
        if (ReadMotionPrimitives(fMotPrim) == false) {
            throw SBPL_Exception(
                "ERROR: failed to read in motion primitive file");
        }

        EnvNAVXYTHETACCfg.StartTheta =
            ContTheta2DiscNew(EnvNAVXYTHETACCfg.StartTheta_rad);
        if (EnvNAVXYTHETACCfg.StartTheta < 0 ||
            EnvNAVXYTHETACCfg.StartTheta >= EnvNAVXYTHETACCfg.NumThetaDirs) {
            throw new SBPL_Exception(
                "ERROR: illegal start coordinates for theta");
        }
        EnvNAVXYTHETACCfg.EndTheta =
            ContTheta2DiscNew(EnvNAVXYTHETACCfg.EndTheta_rad);
        if (EnvNAVXYTHETACCfg.EndTheta < 0 ||
            EnvNAVXYTHETACCfg.EndTheta >= EnvNAVXYTHETACCfg.NumThetaDirs) {
            throw new SBPL_Exception(
                "ERROR: illegal goal coordinates for theta");
        }

        InitGeneral(&EnvNAVXYTHETACCfg.mprimV);
        fclose(fMotPrim);
    } else {
        (NULL);
    }

    SBPL_PRINTF("size of env: %d by %d\n", EnvNAVXYTHETACCfg.EnvWidth_c,
                EnvNAVXYTHETACCfg.EnvHeight_c);

    return true;
}

// currently not get into
bool EnvironmentNAVXYTHETAC::InitializeEnv(const char* sEnvFile) {
    FILE* fCfg = fopen(sEnvFile, "r");
    if (fCfg == NULL) {
        SBPL_ERROR("ERROR: unable to open %s\n", sEnvFile);
        throw SBPL_Exception();
    }
    ReadConfiguration(fCfg);
    fclose(fCfg);

    InitGeneral(NULL);

    return true;
}

// examples of hash function: map state coordinates onto a hash table
unsigned int EnvironmentNAVXYTHETAC::GETHASHBIN(unsigned int X1,
                                                unsigned int X2,
                                                unsigned int Theta) {
    unsigned int a;
    a = inthash(inthash(X1) + (inthash(X2) << 1) + (inthash(Theta) << 2)) &
        (HashTableSize - 1);
    return a;
}

// unsigned int EnvironmentNAVXYTHETAC::GETHASHBIN(
// unsigned int X,
// unsigned int Y,
// unsigned int Theta){

// unsigned int a;
// a = (X + Y*EnvNAVXYTHETACCfg.EnvWidth_c)* NAVXYTHETAC_THETADIRS + Theta;

// a = X + Y*EnvNAVXYTHETACCfg.EnvWidth_c;
// return a;
//}

bool EnvironmentNAVXYTHETAC::InitializeMDPCfg(MDPConfig* MDPCfg) {
    // initialize MDPCfg with the start and goal ids
    MDPCfg->goalstateid = EnvNAVXYTHETAC.goalstateid;
    MDPCfg->startstateid = EnvNAVXYTHETAC.startstateid;
    return true;
}

void EnvironmentNAVXYTHETAC::InitializeEnvironment() {
    EnvNAVXYTHETACHashEntry_t* HashEntry;

    int maxsize = EnvNAVXYTHETACCfg.EnvWidth_c * EnvNAVXYTHETACCfg.EnvHeight_c;

#if PENALTY
    NNtable = new std::vector<EnvNAVXYTHETACHashEntry_t*>[maxsize];
#else
    NNtable = nullptr;
#endif

    if (maxsize <= SBPL_XYTHETAC_MAXSTATESFORLOOKUP) {  // not using hash table
        SBPL_PRINTF("environment stores states in lookup table\n");

        Coord2StateIDHashTable_lookup = new EnvNAVXYTHETACHashEntry_t*[maxsize];
        for (int i = 0; i < maxsize; ++i) {
            Coord2StateIDHashTable_lookup[i] = NULL;
        }
        GetHashEntry = &EnvironmentNAVXYTHETAC::GetHashEntry_lookup;
        CreateNewHashEntry = &EnvironmentNAVXYTHETAC::CreateNewHashEntry_lookup;

        HashTableSize = 0;
        Coord2StateIDHashTable = NULL;
    } else {  // not using lookup
        SBPL_PRINTF("environment stores states in hash table");

        // initialize the map from coord to stateID
        HashTableSize = 16 * 1024 * 1024;  // should be power of 2

        // HashTableSize = maxsize;

        Coord2StateIDHashTable =
            new std::vector<EnvNAVXYTHETACHashEntry_t*>[HashTableSize];
        GetHashEntry = &EnvironmentNAVXYTHETAC::GetHashEntry_hash;
        CreateNewHashEntry = &EnvironmentNAVXYTHETAC::CreateNewHashEntry_hash;

        Coord2StateIDHashTable_lookup = NULL;
    }

    // initialize the map from StateID to Coord
    StateID2CoordTable.clear();

    // create start state
    if (NULL == (HashEntry = (this->*GetHashEntry)(
                     EnvNAVXYTHETACCfg.StartX_c, EnvNAVXYTHETACCfg.StartY_c,
                     EnvNAVXYTHETACCfg.StartTheta))) {
        // have to create a new entry
        HashEntry = (this->*CreateNewHashEntry)(EnvNAVXYTHETACCfg.StartX_c,
                                                EnvNAVXYTHETACCfg.StartY_c,
                                                EnvNAVXYTHETACCfg.StartTheta);
    }
    EnvNAVXYTHETAC.startstateid = HashEntry->stateID;

    // create goal state
    if ((HashEntry = (this->*GetHashEntry)(
             EnvNAVXYTHETACCfg.EndX_c, EnvNAVXYTHETACCfg.EndY_c,
             EnvNAVXYTHETACCfg.EndTheta)) == NULL) {
        // have to create a new entry
        HashEntry = (this->*CreateNewHashEntry)(EnvNAVXYTHETACCfg.EndX_c,
                                                EnvNAVXYTHETACCfg.EndY_c,
                                                EnvNAVXYTHETACCfg.EndTheta);
    }
    EnvNAVXYTHETAC.goalstateid = HashEntry->stateID;

    // initialized
    EnvNAVXYTHETAC.bInitialized = true;

}  // InitializeEnvironment

bool EnvironmentNAVXYTHETAC::InitGeneral(
    std::vector<SBPL_xytheta_mprimitive>* motionprimitiveV) {
    // Initialize other parameters of the environment
    InitializeEnvConfig(motionprimitiveV);

    // initialize Environment
    InitializeEnvironment();

    // pre-compute heuristics
    ComputeHeuristicValues();

    return true;
}

void EnvironmentNAVXYTHETAC::InitializeEnvConfig(
    std::vector<SBPL_xytheta_mprimitive>* motionprimitiveV) {
    // additional to configuration file initialization of EnvNAVXYTHETACCfg if
    // necessary
    // dXY dirs
    EnvNAVXYTHETACCfg.dXY[0][0] = -1;
    EnvNAVXYTHETACCfg.dXY[0][1] = -1;
    EnvNAVXYTHETACCfg.dXY[1][0] = -1;
    EnvNAVXYTHETACCfg.dXY[1][1] = 0;
    EnvNAVXYTHETACCfg.dXY[2][0] = -1;
    EnvNAVXYTHETACCfg.dXY[2][1] = 1;
    EnvNAVXYTHETACCfg.dXY[3][0] = 0;
    EnvNAVXYTHETACCfg.dXY[3][1] = -1;
    EnvNAVXYTHETACCfg.dXY[4][0] = 0;
    EnvNAVXYTHETACCfg.dXY[4][1] = 1;
    EnvNAVXYTHETACCfg.dXY[5][0] = 1;
    EnvNAVXYTHETACCfg.dXY[5][1] = -1;
    EnvNAVXYTHETACCfg.dXY[6][0] = 1;
    EnvNAVXYTHETACCfg.dXY[6][1] = 0;
    EnvNAVXYTHETACCfg.dXY[7][0] = 1;
    EnvNAVXYTHETACCfg.dXY[7][1] = 1;

    sbpl_xy_theta_pt_t temppose;
    temppose.x = 0.0;
    temppose.y = 0.0;
    temppose.theta = 0.0;

    std::vector<sbpl_2Dcell_t> footprint;
    get_2d_footprint_cells(EnvNAVXYTHETACCfg.FootprintPolygon, &footprint,
                           temppose, EnvNAVXYTHETACCfg.cellsize_m);
    SBPL_PRINTF("number of cells in footprint of the robot = %d\n",
                (unsigned int)footprint.size());

    for (std::vector<sbpl_2Dcell_t>::iterator it = footprint.begin();
         it != footprint.end(); ++it) {
        SBPL_PRINTF("Footprint cell at (%d, %d)\n", it->x, it->y);
    }

#if DEBUG
    SBPL_FPRINTF(fDeb, "footprint cells (size=%d):\n", (int)footprint.size());
    for (int i = 0; i < (int)footprint.size(); ++i) {
        SBPL_FPRINTF(
            fDeb, "%d %d (cont: %.3f %.3f)\n", footprint.at(i).x,
            footprint.at(i).y,
            DISCXY2CONT(footprint.at(i).x, EnvNAVXYTHETACCfg.cellsize_m),
            DISCXY2CONT(footprint.at(i).y, EnvNAVXYTHETACCfg.cellsize_m));
    }
#endif

    if (motionprimitiveV == NULL) {
        DeprecatedPrecomputeActions();
    } else {
        PrecomputeActionswithCompleteMotionPrimitive(motionprimitiveV);
    }
}  // InitializeEnvConfig

void EnvironmentNAVXYTHETAC::DeprecatedPrecomputeActions() {
    SBPL_PRINTF(
        "Use of DeprecatedPrecomputeActions() is deprecated and probably "
        "doesn't work!\n");
    ;

    // construct list of actions
    SBPL_PRINTF(
        "Pre-computing action data using the motion primitives for a 3D "
        "kinematic planning...\n");
    EnvNAVXYTHETACCfg.ActionsV =
        new EnvNAVXYTHETACAction_t*[EnvNAVXYTHETACCfg.NumThetaDirs];
    EnvNAVXYTHETACCfg.PredActionsV = new std::vector<
        EnvNAVXYTHETACAction_t*>[EnvNAVXYTHETACCfg.NumThetaDirs];
    std::vector<sbpl_2Dcell_t> footprint;
    // iterate over source angles
    for (int tind = 0; tind < EnvNAVXYTHETACCfg.NumThetaDirs; ++tind) {
        SBPL_PRINTF("processing angle %d\n", tind);
        EnvNAVXYTHETACCfg.ActionsV[tind] =
            new EnvNAVXYTHETACAction_t[EnvNAVXYTHETACCfg.actionwidth];

        // compute sourcepose
        sbpl_xy_theta_pt_t sourcepose;
        sourcepose.x = DISCXY2CONT(0, EnvNAVXYTHETACCfg.cellsize_m);
        sourcepose.y = DISCXY2CONT(0, EnvNAVXYTHETACCfg.cellsize_m);
        sourcepose.theta = DiscTheta2ContNew(tind);

        // the construction sasumes that the robot first turns and then goes
        // along this new theta
        int aind = 0;
        for (; aind < 3; ++aind) {
            EnvNAVXYTHETACCfg.ActionsV[tind][aind].aind = aind;
            EnvNAVXYTHETACCfg.ActionsV[tind][aind].starttheta = tind;
            // -1,0,1
            EnvNAVXYTHETACCfg.ActionsV[tind][aind].endtheta =
                (tind + aind - 1) % EnvNAVXYTHETACCfg.NumThetaDirs;
            double angle = DiscTheta2ContNew(
                EnvNAVXYTHETACCfg.ActionsV[tind][aind].endtheta);
            EnvNAVXYTHETACCfg.ActionsV[tind][aind].dX =
                (int)(cos(angle) + 0.5 * (cos(angle) > 0 ? 1 : -1));
            EnvNAVXYTHETACCfg.ActionsV[tind][aind].dY =
                (int)(sin(angle) + 0.5 * (sin(angle) > 0 ? 1 : -1));
            EnvNAVXYTHETACCfg.ActionsV[tind][aind].cost = (int)(ceil(
                NAVXYTHETAC_COSTMULT_MTOMM * EnvNAVXYTHETACCfg.cellsize_m /
                EnvNAVXYTHETACCfg.nominalvel_mpersecs *
                sqrt((double)(EnvNAVXYTHETACCfg.ActionsV[tind][aind].dX *
                                  EnvNAVXYTHETACCfg.ActionsV[tind][aind].dX +
                              EnvNAVXYTHETACCfg.ActionsV[tind][aind].dY *
                                  EnvNAVXYTHETACCfg.ActionsV[tind][aind].dY))));

            // compute intersecting cells
            sbpl_xy_theta_pt_t pose;
            pose.x = DISCXY2CONT(EnvNAVXYTHETACCfg.ActionsV[tind][aind].dX,
                                 EnvNAVXYTHETACCfg.cellsize_m);
            pose.y = DISCXY2CONT(EnvNAVXYTHETACCfg.ActionsV[tind][aind].dY,
                                 EnvNAVXYTHETACCfg.cellsize_m);
            pose.theta = angle;
            EnvNAVXYTHETACCfg.ActionsV[tind][aind].intermptV.clear();
            EnvNAVXYTHETACCfg.ActionsV[tind][aind].intersectingcellsV.clear();
            get_2d_footprint_cells(
                EnvNAVXYTHETACCfg.FootprintPolygon,
                &EnvNAVXYTHETACCfg.ActionsV[tind][aind].intersectingcellsV,
                pose, EnvNAVXYTHETACCfg.cellsize_m);
            RemoveSourceFootprint(
                sourcepose,
                &EnvNAVXYTHETACCfg.ActionsV[tind][aind].intersectingcellsV);

#if DEBUG
            SBPL_PRINTF(
                "action tind=%d aind=%d: endtheta=%d (%f) dX=%d dY=%d "
                "cost=%d\n",
                tind, aind, EnvNAVXYTHETACCfg.ActionsV[tind][aind].endtheta,
                angle, EnvNAVXYTHETACCfg.ActionsV[tind][aind].dX,
                EnvNAVXYTHETACCfg.ActionsV[tind][aind].dY,
                EnvNAVXYTHETACCfg.ActionsV[tind][aind].cost);
#endif
            // add to the list of backward actions
            int targettheta = EnvNAVXYTHETACCfg.ActionsV[tind][aind].endtheta;
            if (targettheta < 0) {
                targettheta = targettheta + EnvNAVXYTHETACCfg.NumThetaDirs;
            }
            EnvNAVXYTHETACCfg.PredActionsV[targettheta].push_back(
                &(EnvNAVXYTHETACCfg.ActionsV[tind][aind]));
        }

        // decrease and increase angle without movement
        aind = 3;
        EnvNAVXYTHETACCfg.ActionsV[tind][aind].aind = aind;
        EnvNAVXYTHETACCfg.ActionsV[tind][aind].starttheta = tind;
        EnvNAVXYTHETACCfg.ActionsV[tind][aind].endtheta = tind - 1;
        if (EnvNAVXYTHETACCfg.ActionsV[tind][aind].endtheta < 0) {
            EnvNAVXYTHETACCfg.ActionsV[tind][aind].endtheta +=
                EnvNAVXYTHETACCfg.NumThetaDirs;
        }
        EnvNAVXYTHETACCfg.ActionsV[tind][aind].dX = 0;
        EnvNAVXYTHETACCfg.ActionsV[tind][aind].dY = 0;
        EnvNAVXYTHETACCfg.ActionsV[tind][aind].cost =
            (int)(NAVXYTHETAC_COSTMULT_MTOMM *
                  EnvNAVXYTHETACCfg.timetoturn45degsinplace_secs);

        // compute intersecting cells
        sbpl_xy_theta_pt_t pose;
        pose.x = DISCXY2CONT(EnvNAVXYTHETACCfg.ActionsV[tind][aind].dX,
                             EnvNAVXYTHETACCfg.cellsize_m);
        pose.y = DISCXY2CONT(EnvNAVXYTHETACCfg.ActionsV[tind][aind].dY,
                             EnvNAVXYTHETACCfg.cellsize_m);
        pose.theta =
            DiscTheta2ContNew(EnvNAVXYTHETACCfg.ActionsV[tind][aind].endtheta);
        EnvNAVXYTHETACCfg.ActionsV[tind][aind].intermptV.clear();
        EnvNAVXYTHETACCfg.ActionsV[tind][aind].intersectingcellsV.clear();
        get_2d_footprint_cells(
            EnvNAVXYTHETACCfg.FootprintPolygon,
            &EnvNAVXYTHETACCfg.ActionsV[tind][aind].intersectingcellsV, pose,
            EnvNAVXYTHETACCfg.cellsize_m);
        RemoveSourceFootprint(
            sourcepose,
            &EnvNAVXYTHETACCfg.ActionsV[tind][aind].intersectingcellsV);

#if DEBUG
        SBPL_PRINTF(
            "action tind=%d aind=%d: endtheta=%d (%f) dX=%d dY=%d cost=%d\n",
            tind, aind, EnvNAVXYTHETACCfg.ActionsV[tind][aind].endtheta,
            DiscTheta2ContNew(EnvNAVXYTHETACCfg.ActionsV[tind][aind].endtheta),
            EnvNAVXYTHETACCfg.ActionsV[tind][aind].dX,
            EnvNAVXYTHETACCfg.ActionsV[tind][aind].dY,
            EnvNAVXYTHETACCfg.ActionsV[tind][aind].cost);
#endif

        // add to the list of backward actions
        int targettheta = EnvNAVXYTHETACCfg.ActionsV[tind][aind].endtheta;
        if (targettheta < 0) {
            targettheta = targettheta + EnvNAVXYTHETACCfg.NumThetaDirs;
        }
        EnvNAVXYTHETACCfg.PredActionsV[targettheta].push_back(
            &(EnvNAVXYTHETACCfg.ActionsV[tind][aind]));

        aind = 4;
        EnvNAVXYTHETACCfg.ActionsV[tind][aind].aind = aind;
        EnvNAVXYTHETACCfg.ActionsV[tind][aind].starttheta = tind;
        EnvNAVXYTHETACCfg.ActionsV[tind][aind].endtheta =
            (tind + 1) % EnvNAVXYTHETACCfg.NumThetaDirs;
        EnvNAVXYTHETACCfg.ActionsV[tind][aind].dX = 0;
        EnvNAVXYTHETACCfg.ActionsV[tind][aind].dY = 0;

        EnvNAVXYTHETACCfg.ActionsV[tind][aind].cost =
            (int)(NAVXYTHETAC_COSTMULT_MTOMM *
                  EnvNAVXYTHETACCfg.timetoturn45degsinplace_secs);

        // compute intersecting cells
        pose.x = DISCXY2CONT(EnvNAVXYTHETACCfg.ActionsV[tind][aind].dX,
                             EnvNAVXYTHETACCfg.cellsize_m);
        pose.y = DISCXY2CONT(EnvNAVXYTHETACCfg.ActionsV[tind][aind].dY,
                             EnvNAVXYTHETACCfg.cellsize_m);
        pose.theta =
            DiscTheta2ContNew(EnvNAVXYTHETACCfg.ActionsV[tind][aind].endtheta);
        EnvNAVXYTHETACCfg.ActionsV[tind][aind].intermptV.clear();
        EnvNAVXYTHETACCfg.ActionsV[tind][aind].intersectingcellsV.clear();
        get_2d_footprint_cells(
            EnvNAVXYTHETACCfg.FootprintPolygon,
            &EnvNAVXYTHETACCfg.ActionsV[tind][aind].intersectingcellsV, pose,
            EnvNAVXYTHETACCfg.cellsize_m);
        RemoveSourceFootprint(
            sourcepose,
            &EnvNAVXYTHETACCfg.ActionsV[tind][aind].intersectingcellsV);

#if DEBUG
        SBPL_PRINTF(
            "action tind=%d aind=%d: endtheta=%d (%f) dX=%d dY=%d cost=%d\n",
            tind, aind, EnvNAVXYTHETACCfg.ActionsV[tind][aind].endtheta,
            DiscTheta2ContNew(EnvNAVXYTHETACCfg.ActionsV[tind][aind].endtheta),
            EnvNAVXYTHETACCfg.ActionsV[tind][aind].dX,
            EnvNAVXYTHETACCfg.ActionsV[tind][aind].dY,
            EnvNAVXYTHETACCfg.ActionsV[tind][aind].cost);
#endif

        // add to the list of backward actions
        targettheta = EnvNAVXYTHETACCfg.ActionsV[tind][aind].endtheta;
        if (targettheta < 0)
            targettheta = targettheta + EnvNAVXYTHETACCfg.NumThetaDirs;
        EnvNAVXYTHETACCfg.PredActionsV[targettheta].push_back(
            &(EnvNAVXYTHETACCfg.ActionsV[tind][aind]));
    }

    // now compute replanning data
    ComputeReplanningData();

    SBPL_PRINTF("done pre-computing action data\n");

}  // DeprecatedPrecomputeActions

// removes a set of cells that correspond to the specified footprint at the
// sourcepose adds points to it (does not clear it beforehand)
void EnvironmentNAVXYTHETAC::RemoveSourceFootprint(
    sbpl_xy_theta_pt_t sourcepose, std::vector<sbpl_2Dcell_t>* footprint,
    const std::vector<sbpl_2Dpt_t>& FootprintPolygon) {
    std::vector<sbpl_2Dcell_t> sourcefootprint;

    // compute source footprint
    get_2d_footprint_cells(FootprintPolygon, &sourcefootprint, sourcepose,
                           EnvNAVXYTHETACCfg.cellsize_m);

    // now remove the source cells from the footprint
    for (int sind = 0; sind < (int)sourcefootprint.size(); sind++) {
        for (int find = 0; find < (int)footprint->size(); find++) {
            if (sourcefootprint.at(sind).x == footprint->at(find).x &&
                sourcefootprint.at(sind).y == footprint->at(find).y) {
                footprint->erase(footprint->begin() + find);
                break;
            }
        }  // over footprint
    }      // over source
}

// removes a set of cells that correspond to the footprint of the base at the
// sourcepose adds points to it (does not clear it beforehand)
void EnvironmentNAVXYTHETAC::RemoveSourceFootprint(
    sbpl_xy_theta_pt_t sourcepose, std::vector<sbpl_2Dcell_t>* footprint) {
    RemoveSourceFootprint(sourcepose, footprint,
                          EnvNAVXYTHETACCfg.FootprintPolygon);
}

// here motionprimitivevector contains actions for all angles
void EnvironmentNAVXYTHETAC::PrecomputeActionswithCompleteMotionPrimitive(
    std::vector<SBPL_xytheta_mprimitive>* motionprimitiveV) {
    SBPL_PRINTF(
        "Pre-computing action data using motion primitives for every "
        "angle...\n");
    EnvNAVXYTHETACCfg.ActionsV =
        new EnvNAVXYTHETACAction_t*[EnvNAVXYTHETACCfg.NumThetaDirs];
    EnvNAVXYTHETACCfg.PredActionsV = new std::vector<
        EnvNAVXYTHETACAction_t*>[EnvNAVXYTHETACCfg.NumThetaDirs];
    std::vector<sbpl_2Dcell_t> footprint;

    if (motionprimitiveV->size() % EnvNAVXYTHETACCfg.NumThetaDirs != 0) {
        throw SBPL_Exception(
            "ERROR: motionprimitives should be uniform across actions");
    }

    EnvNAVXYTHETACCfg.actionwidth =
        ((int)motionprimitiveV->size()) / EnvNAVXYTHETACCfg.NumThetaDirs;

    // iterate over source angles
    int maxnumofactions = 0;
    for (int tind = 0; tind < EnvNAVXYTHETACCfg.NumThetaDirs; tind++) {
        SBPL_PRINTF("pre-computing for angle %d out of %d angles\n", tind,
                    EnvNAVXYTHETACCfg.NumThetaDirs);

        EnvNAVXYTHETACCfg.ActionsV[tind] =
            new EnvNAVXYTHETACAction_t[EnvNAVXYTHETACCfg.actionwidth];

        // compute sourcepose
        sbpl_xy_theta_pt_t sourcepose;
        sourcepose.x = DISCXY2CONT(0, EnvNAVXYTHETACCfg.cellsize_m);
        sourcepose.y = DISCXY2CONT(0, EnvNAVXYTHETACCfg.cellsize_m);
        sourcepose.theta = DiscTheta2ContNew(tind);

        // iterate over motion primitives
        int numofactions = 0;
        int aind = -1;
        for (int mind = 0; mind < (int)motionprimitiveV->size(); mind++) {
            // find a motion primitive for this angle
            if (motionprimitiveV->at(mind).starttheta_c != tind) {
                continue;
            }

            aind++;
            numofactions++;

            // action index
            EnvNAVXYTHETACCfg.ActionsV[tind][aind].aind = aind;

            // start angle
            EnvNAVXYTHETACCfg.ActionsV[tind][aind].starttheta = tind;

            // compute dislocation
            EnvNAVXYTHETACCfg.ActionsV[tind][aind].endtheta =
                motionprimitiveV->at(mind).endcell.theta;
            EnvNAVXYTHETACCfg.ActionsV[tind][aind].dX =
                motionprimitiveV->at(mind).endcell.x;
            EnvNAVXYTHETACCfg.ActionsV[tind][aind].dY =
                motionprimitiveV->at(mind).endcell.y;

            // compute and store interm points as well as intersecting cells
            EnvNAVXYTHETACCfg.ActionsV[tind][aind].intersectingcellsV.clear();
            EnvNAVXYTHETACCfg.ActionsV[tind][aind].intermptV.clear();
            EnvNAVXYTHETACCfg.ActionsV[tind][aind].interm3DcellsV.clear();

            sbpl_xy_theta_cell_t previnterm3Dcell;
            previnterm3Dcell.x = 0;
            previnterm3Dcell.y = 0;

            // Compute all the intersected cells for this action (intermptV and
            // interm3DcellsV)
            for (int pind = 0;
                 pind < (int)motionprimitiveV->at(mind).intermptV.size();
                 pind++) {
                sbpl_xy_theta_pt_t intermpt =
                    motionprimitiveV->at(mind).intermptV[pind];
                EnvNAVXYTHETACCfg.ActionsV[tind][aind].intermptV.push_back(
                    intermpt);

                // also compute the intermediate discrete cells if not there
                // already
                sbpl_xy_theta_pt_t pose;
                pose.x = intermpt.x + sourcepose.x;
                pose.y = intermpt.y + sourcepose.y;
                pose.theta = intermpt.theta;

                sbpl_xy_theta_cell_t intermediate2dCell;
                intermediate2dCell.x =
                    CONTXY2DISC(pose.x, EnvNAVXYTHETACCfg.cellsize_m);
                intermediate2dCell.y =
                    CONTXY2DISC(pose.y, EnvNAVXYTHETACCfg.cellsize_m);

                // add unique cells to the list
                if (EnvNAVXYTHETACCfg.ActionsV[tind][aind]
                            .interm3DcellsV.size() == 0 ||
                    intermediate2dCell.x != previnterm3Dcell.x ||
                    intermediate2dCell.y != previnterm3Dcell.y) {
                    EnvNAVXYTHETACCfg.ActionsV[tind][aind]
                        .interm3DcellsV.push_back(intermediate2dCell);
                }

                previnterm3Dcell = intermediate2dCell;
            }

            // compute linear and angular time
            double linear_distance = 0;
            for (unsigned int i = 1;
                 i < EnvNAVXYTHETACCfg.ActionsV[tind][aind].intermptV.size();
                 i++) {
                double x0 =
                    EnvNAVXYTHETACCfg.ActionsV[tind][aind].intermptV[i - 1].x;
                double y0 =
                    EnvNAVXYTHETACCfg.ActionsV[tind][aind].intermptV[i - 1].y;
                double x1 =
                    EnvNAVXYTHETACCfg.ActionsV[tind][aind].intermptV[i].x;
                double y1 =
                    EnvNAVXYTHETACCfg.ActionsV[tind][aind].intermptV[i].y;
                double dx = x1 - x0;
                double dy = y1 - y0;
                linear_distance += sqrt(dx * dx + dy * dy);
            }
            double linear_time =
                linear_distance / EnvNAVXYTHETACCfg.nominalvel_mpersecs;
            double angular_distance;
            angular_distance = fabs(computeMinUnsignedAngleDiff(
                DiscTheta2ContNew(
                    EnvNAVXYTHETACCfg.ActionsV[tind][aind].endtheta),
                DiscTheta2ContNew(
                    EnvNAVXYTHETACCfg.ActionsV[tind][aind].starttheta)));

            double angular_time =
                angular_distance /
                ((PI_CONST / 4.0) /
                 EnvNAVXYTHETACCfg.timetoturn45degsinplace_secs);
            // make the cost the max of the two times
            EnvNAVXYTHETACCfg.ActionsV[tind][aind].cost =
                (int)(ceil(NAVXYTHETAC_COSTMULT_MTOMM *
                           std::max(linear_time, angular_time)));
            // use any additional cost multiplier
            EnvNAVXYTHETACCfg.ActionsV[tind][aind].cost *=
                motionprimitiveV->at(mind).additionalactioncostmult;

            // now compute the intersecting cells for this motion (including
            // ignoring the source footprint)
            get_2d_motion_cells(
                EnvNAVXYTHETACCfg.FootprintPolygon,
                motionprimitiveV->at(mind).intermptV,
                &EnvNAVXYTHETACCfg.ActionsV[tind][aind].intersectingcellsV,
                EnvNAVXYTHETACCfg.cellsize_m);

#if DEBUG
            SBPL_FPRINTF(
                fDeb,
                "action tind=%2d aind=%2d: dX=%3d dY=%3d endtheta=%3d (%6.2f "
                "degs -> %6.2f degs) "
                "cost=%4d (mprimID %3d: %3d %3d %3d) numofintermcells = %d "
                "numofintercells=%d\n",
                tind, aind, EnvNAVXYTHETACCfg.ActionsV[tind][aind].dX,
                EnvNAVXYTHETACCfg.ActionsV[tind][aind].dY,
                EnvNAVXYTHETACCfg.ActionsV[tind][aind].endtheta,
                EnvNAVXYTHETACCfg.ActionsV[tind][aind].intermptV[0].theta *
                    180 / PI_CONST,
                EnvNAVXYTHETACCfg.ActionsV[tind][aind]
                        .intermptV[EnvNAVXYTHETACCfg.ActionsV[tind][aind]
                                       .intermptV.size() -
                                   1]
                        .theta *
                    180 / PI_CONST,
                EnvNAVXYTHETACCfg.ActionsV[tind][aind].cost,
                motionprimitiveV->at(mind).motprimID,
                motionprimitiveV->at(mind).endcell.x,
                motionprimitiveV->at(mind).endcell.y,
                motionprimitiveV->at(mind).endcell.theta,
                (int)EnvNAVXYTHETACCfg.ActionsV[tind][aind]
                    .interm3DcellsV.size(),
                (int)EnvNAVXYTHETACCfg.ActionsV[tind][aind]
                    .intersectingcellsV.size());
#endif

            // add to the list of backward actions
            int targettheta = EnvNAVXYTHETACCfg.ActionsV[tind][aind].endtheta;
            if (targettheta < 0) {
                targettheta = targettheta + EnvNAVXYTHETACCfg.NumThetaDirs;
            }
            EnvNAVXYTHETACCfg.PredActionsV[targettheta].push_back(
                &(EnvNAVXYTHETACCfg.ActionsV[tind][aind]));
        }

        if (maxnumofactions < numofactions) {
            maxnumofactions = numofactions;
        }
    }

    // at this point we don't allow nonuniform number of actions
    if (motionprimitiveV->size() !=
        (size_t)(EnvNAVXYTHETACCfg.NumThetaDirs * maxnumofactions)) {
        std::stringstream ss;
        ss << "ERROR: nonuniform number of actions is not supported"
           << " (maxnumofactions=" << maxnumofactions
           << " while motprims=" << motionprimitiveV->size()
           << " thetas=" << EnvNAVXYTHETACCfg.NumThetaDirs;
        throw SBPL_Exception(ss.str());
    }

    // now compute replanning data
    ComputeReplanningData();

    SBPL_PRINTF("done pre-computing action data based on motion primitives\n");

}  // PrecomputeActionswithCompleteMotionPrimitive

void EnvironmentNAVXYTHETAC::ComputeHeuristicValues() {
    SBPL_PRINTF("Precomputing heuristics...\n");

    // allocated 2D grid searches
    grid2Dsearchfromstart = new SBPL2DGridSearch(
        EnvNAVXYTHETACCfg.EnvWidth_c, EnvNAVXYTHETACCfg.EnvHeight_c,
        (float)EnvNAVXYTHETACCfg.cellsize_m, blocksize, bucketsize);
    grid2Dsearchfromgoal = new SBPL2DGridSearch(
        EnvNAVXYTHETACCfg.EnvWidth_c, EnvNAVXYTHETACCfg.EnvHeight_c,
        (float)EnvNAVXYTHETACCfg.cellsize_m, blocksize, bucketsize);

    // set OPEN type to sliding buckets
    grid2Dsearchfromstart->setOPENdatastructure(
        SBPL_2DGRIDSEARCH_OPENTYPE_SLIDINGBUCKETS);
    grid2Dsearchfromgoal->setOPENdatastructure(
        SBPL_2DGRIDSEARCH_OPENTYPE_SLIDINGBUCKETS);

    SBPL_PRINTF("done\n");
}

void EnvironmentNAVXYTHETAC::SetConfiguration(
    int width, int height, const unsigned char* mapdata, int startx, int starty,
    int starttheta, int goalx, int goaly, int goaltheta, double cellsize_m,
    double nominalvel_mpersecs, double timetoturn45degsinplace_secs,
    const std::vector<sbpl_2Dpt_t>& robot_perimeterV) {
    EnvNAVXYTHETACCfg.EnvWidth_c = width;
    EnvNAVXYTHETACCfg.EnvHeight_c = height;

    EnvNAVXYTHETACCfg.StartX_c = startx;
    EnvNAVXYTHETACCfg.StartY_c = starty;
    EnvNAVXYTHETACCfg.StartTheta = starttheta;

    if (EnvNAVXYTHETACCfg.StartX_c < 0 ||
        EnvNAVXYTHETACCfg.StartX_c >= EnvNAVXYTHETACCfg.EnvWidth_c) {
        throw SBPL_Exception("ERROR: illegal start coordinates");
    }
    if (EnvNAVXYTHETACCfg.StartY_c < 0 ||
        EnvNAVXYTHETACCfg.StartY_c >= EnvNAVXYTHETACCfg.EnvHeight_c) {
        throw SBPL_Exception("ERROR: illegal start coordinates");
    }
    if (EnvNAVXYTHETACCfg.StartTheta < 0 ||
        EnvNAVXYTHETACCfg.StartTheta >= EnvNAVXYTHETACCfg.NumThetaDirs) {
        throw SBPL_Exception("ERROR: illegal start coordinates for theta");
    }

    EnvNAVXYTHETACCfg.EndX_c = goalx;
    EnvNAVXYTHETACCfg.EndY_c = goaly;
    EnvNAVXYTHETACCfg.EndTheta = goaltheta;

    if (EnvNAVXYTHETACCfg.EndX_c < 0 ||
        EnvNAVXYTHETACCfg.EndX_c >= EnvNAVXYTHETACCfg.EnvWidth_c) {
        throw SBPL_Exception("ERROR: illegal goal coordinates");
    }
    if (EnvNAVXYTHETACCfg.EndY_c < 0 ||
        EnvNAVXYTHETACCfg.EndY_c >= EnvNAVXYTHETACCfg.EnvHeight_c) {
        throw SBPL_Exception("ERROR: illegal goal coordinates");
    }
    if (EnvNAVXYTHETACCfg.EndTheta < 0 ||
        EnvNAVXYTHETACCfg.EndTheta >= EnvNAVXYTHETACCfg.NumThetaDirs) {
        throw SBPL_Exception("ERROR: illegal goal coordinates for theta");
    }

    EnvNAVXYTHETACCfg.FootprintPolygon = robot_perimeterV;

    EnvNAVXYTHETACCfg.nominalvel_mpersecs = nominalvel_mpersecs;
    EnvNAVXYTHETACCfg.cellsize_m = cellsize_m;
    EnvNAVXYTHETACCfg.timetoturn45degsinplace_secs =
        timetoturn45degsinplace_secs;

    // unallocate the 2D environment
    if (EnvNAVXYTHETACCfg.Grid2D != NULL) {
        for (int x = 0; x < EnvNAVXYTHETACCfg.EnvWidth_c; ++x) {
            delete[] EnvNAVXYTHETACCfg.Grid2D[x];
        }
        delete[] EnvNAVXYTHETACCfg.Grid2D;
        EnvNAVXYTHETACCfg.Grid2D = NULL;
    }

    // allocate the 2D environment
    EnvNAVXYTHETACCfg.Grid2D = new unsigned char*[EnvNAVXYTHETACCfg.EnvWidth_c];
    for (int x = 0; x < EnvNAVXYTHETACCfg.EnvWidth_c; ++x) {
        EnvNAVXYTHETACCfg.Grid2D[x] =
            new unsigned char[EnvNAVXYTHETACCfg.EnvHeight_c];
    }

    // environment
    if (0 == mapdata) {
        for (int y = 0; y < EnvNAVXYTHETACCfg.EnvHeight_c; ++y) {
            for (int x = 0; x < EnvNAVXYTHETACCfg.EnvWidth_c; ++x) {
                EnvNAVXYTHETACCfg.Grid2D[x][y] = 0;
            }
        }
    } else {
        for (int y = 0; y < EnvNAVXYTHETACCfg.EnvHeight_c; ++y) {
            for (int x = 0; x < EnvNAVXYTHETACCfg.EnvWidth_c; ++x) {
                EnvNAVXYTHETACCfg.Grid2D[x][y] = mapdata[x + y * width];
            }
        }
    }
}  // SetConfiguration

void EnvironmentNAVXYTHETAC::ReadConfiguration(FILE* fCfg) {
    // read inthe configuration of environment and initialize
    // EnvNAVXYTHETACCfg structure

    char sTemp[1024], sTemp1[1024];
    int dTemp;
    int x, y;

    // discretization(cells)
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception(
            "ERROR: ran out of env file early (discretization)");
    }
    strcpy(sTemp1, "discretization(cells):");
    if (strcmp(sTemp1, sTemp) != 0) {
        std::stringstream ss;
        ss << "ERROR: configuration file has incorrect format (discretization)"
           << " Expected " << sTemp1 << " got " << sTemp;
        throw SBPL_Exception(ss.str());
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception(
            "ERROR: ran out of env file early (discretization)");
    }
    EnvNAVXYTHETACCfg.EnvWidth_c = atoi(sTemp);
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception(
            "ERROR: ran out of env file early (discretization)");
    }
    EnvNAVXYTHETACCfg.EnvHeight_c = atoi(sTemp);

    // Scan for optional NumThetaDirs parameter. Check for following obsthresh.
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    strcpy(sTemp1, "NumThetaDirs:");
    if (strcmp(sTemp1, sTemp) != 0) {
        // optional NumThetaDirs not available; default is NAVXYTHETAC_THETADIRS
        // (16)
        strcpy(sTemp1, "obsthresh:");
        if (strcmp(sTemp1, sTemp) != 0) {
            std::stringstream ss;
            ss << "ERROR: configuration file has incorrect format"
               << " Expected " << sTemp1 << " got " << sTemp;
            throw SBPL_Exception(ss.str());
        } else {
            EnvNAVXYTHETACCfg.NumThetaDirs = NAVXYTHETAC_THETADIRS;
        }
    } else {
        if (fscanf(fCfg, "%s", sTemp) != 1) {
            throw SBPL_Exception(
                "ERROR: ran out of env file early (NumThetaDirs)");
        }
        EnvNAVXYTHETACCfg.NumThetaDirs = atoi(sTemp);

        // obsthresh:
        if (fscanf(fCfg, "%s", sTemp) != 1) {
            throw SBPL_Exception(
                "ERROR: ran out of env file early (obsthresh)");
        }
        strcpy(sTemp1, "obsthresh:");
        if (strcmp(sTemp1, sTemp) != 0) {
            std::stringstream ss;
            ss << "ERROR: configuration file has incorrect format"
               << " Expected " << sTemp1 << " got " << sTemp
               << " see existing examples of env files for the right format of "
                  "heading";
            throw SBPL_Exception(ss.str());
        }
    }

    // obsthresh
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    EnvNAVXYTHETACCfg.obsthresh = atoi(sTemp);
    SBPL_PRINTF("obsthresh = %d\n", EnvNAVXYTHETACCfg.obsthresh);

    // cost_inscribed_thresh:
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    strcpy(sTemp1, "cost_inscribed_thresh:");
    if (strcmp(sTemp1, sTemp) != 0) {
        std::stringstream ss;
        ss << "ERROR: configuration file has incorrect format"
           << " Expected " << sTemp1 << " got " << sTemp
           << " see existing examples of env files for the right format of "
              "heading";
        throw SBPL_Exception(ss.str());
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    EnvNAVXYTHETACCfg.cost_inscribed_thresh = atoi(sTemp);
    SBPL_PRINTF("cost_inscribed_thresh = %d\n",
                EnvNAVXYTHETACCfg.cost_inscribed_thresh);

    // cost_possibly_circumscribed_thresh:
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    strcpy(sTemp1, "cost_possibly_circumscribed_thresh:");
    if (strcmp(sTemp1, sTemp) != 0) {
        std::stringstream ss;
        ss << "ERROR: configuration file has incorrect format"
           << " Expected " << sTemp1 << " got " << sTemp
           << " see existing examples of env files for the right format of "
              "heading";
        throw SBPL_Exception(ss.str());
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    EnvNAVXYTHETACCfg.cost_possibly_circumscribed_thresh = atoi(sTemp);
    SBPL_PRINTF("cost_possibly_circumscribed_thresh = %d\n",
                EnvNAVXYTHETACCfg.cost_possibly_circumscribed_thresh);

    // cellsize
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    strcpy(sTemp1, "cellsize(meters):");
    if (strcmp(sTemp1, sTemp) != 0) {
        std::stringstream ss;
        ss << "ERROR: configuration file has incorrect format"
           << " Expected " << sTemp1 << " got " << sTemp;
        throw SBPL_Exception(ss.str());
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    EnvNAVXYTHETACCfg.cellsize_m = atof(sTemp);

    // speeds
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    strcpy(sTemp1, "nominalvel(mpersecs):");
    if (strcmp(sTemp1, sTemp) != 0) {
        std::stringstream ss;
        ss << "ERROR: configuration file has incorrect format"
           << " Expected " << sTemp1 << " got " << sTemp;
        throw SBPL_Exception(ss.str());
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    EnvNAVXYTHETACCfg.nominalvel_mpersecs = atof(sTemp);
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    strcpy(sTemp1, "timetoturn45degsinplace(secs):");
    if (strcmp(sTemp1, sTemp) != 0) {
        std::stringstream ss;
        ss << "ERROR: configuration file has incorrect format"
           << " Expected " << sTemp1 << " got " << sTemp;
        throw SBPL_Exception(ss.str());
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    EnvNAVXYTHETACCfg.timetoturn45degsinplace_secs = atof(sTemp);

    // //////////////
    // start(meters,rads):
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }

    // EnvNAVXYTHETACCfg.StartX_c = CONTXY2DISC(atof(sTemp),
    // EnvNAVXYTHETACCfg.cellsize_m);
    EnvNAVXYTHETACCfg.StartX_c = atof(sTemp);
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }

    // EnvNAVXYTHETACCfg.StartY_c = CONTXY2DISC(atof(sTemp),
    // EnvNAVXYTHETACCfg.cellsize_m);
    EnvNAVXYTHETACCfg.StartY_c = atof(sTemp);
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }

    EnvNAVXYTHETACCfg.StartTheta_rad = atof(sTemp);

    if (EnvNAVXYTHETACCfg.StartX_c < 0 ||
        EnvNAVXYTHETACCfg.StartX_c >= EnvNAVXYTHETACCfg.EnvWidth_c) {
        throw SBPL_Exception("ERROR: illegal start coordinates");
    }
    if (EnvNAVXYTHETACCfg.StartY_c < 0 ||
        EnvNAVXYTHETACCfg.StartY_c >= EnvNAVXYTHETACCfg.EnvHeight_c) {
        throw SBPL_Exception("ERROR: illegal start coordinates");
    }

    ////////////////////
    // end(meters,rads):
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }

    EnvNAVXYTHETACCfg.EndX_c = atof(sTemp);
    // EnvNAVXYTHETACCfg.EndX_c = CONTXY2DISC(atof(sTemp),
    // EnvNAVXYTHETACCfg.cellsize_m);
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }

    EnvNAVXYTHETACCfg.EndY_c = atof(sTemp);
    // EnvNAVXYTHETACCfg.EndY_c = CONTXY2DISC(atof(sTemp),
    // EnvNAVXYTHETACCfg.cellsize_m);
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }

    EnvNAVXYTHETACCfg.EndTheta_rad = atof(sTemp);

    if (EnvNAVXYTHETACCfg.EndX_c < 0 ||
        EnvNAVXYTHETACCfg.EndX_c >= EnvNAVXYTHETACCfg.EnvWidth_c) {
        throw SBPL_Exception("ERROR: illegal end coordinates");
    }
    if (EnvNAVXYTHETACCfg.EndY_c < 0 ||
        EnvNAVXYTHETACCfg.EndY_c >= EnvNAVXYTHETACCfg.EnvHeight_c) {
        throw SBPL_Exception("ERROR: illegal end coordinates");
    }

    // unallocate the 2d environment
    if (EnvNAVXYTHETACCfg.Grid2D != NULL) {
        for (x = 0; x < EnvNAVXYTHETACCfg.EnvWidth_c; x++) {
            delete[] EnvNAVXYTHETACCfg.Grid2D[x];
        }
        delete[] EnvNAVXYTHETACCfg.Grid2D;
        EnvNAVXYTHETACCfg.Grid2D = NULL;
    }

    // allocate the 2D environment
    EnvNAVXYTHETACCfg.Grid2D = new unsigned char*[EnvNAVXYTHETACCfg.EnvWidth_c];
    for (x = 0; x < EnvNAVXYTHETACCfg.EnvWidth_c; x++) {
        EnvNAVXYTHETACCfg.Grid2D[x] =
            new unsigned char[EnvNAVXYTHETACCfg.EnvHeight_c];
    }

    // environment:
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    for (y = 0; y < EnvNAVXYTHETACCfg.EnvHeight_c; y++) {
        for (x = 0; x < EnvNAVXYTHETACCfg.EnvWidth_c; x++) {
            if (fscanf(fCfg, "%d", &dTemp) != 1) {
                throw SBPL_Exception("ERROR: incorrect format of config file");
            }
            EnvNAVXYTHETACCfg.Grid2D[x][y] = dTemp;
        }
    }
}  // ReadConfiguration

// returns the stateid if success, and -1 otherwise
int EnvironmentNAVXYTHETAC::SetStart(double x_m, double y_m, double theta_rad) {
    // convert continuous value to discretized
    int x = CONTXY2DISC(x_m, EnvNAVXYTHETACCfg.cellsize_m);
    int y = CONTXY2DISC(y_m, EnvNAVXYTHETACCfg.cellsize_m);
    int theta = ContTheta2DiscNew(theta_rad);

    EnvNAVXYTHETACCfg.StartXi = x;
    EnvNAVXYTHETACCfg.StartYi = y;

    if (!IsWithinMapCell(x, y)) {
        SBPL_ERROR(
            "ERROR: trying to set a start cell %d %d that is outside of map\n",
            x, y);
        return -1;
    }

    SBPL_PRINTF("env: setting start to %.3f %.3f %.3f (%d %d %d)\n", x_m, y_m,
                theta_rad, x, y, theta);

    if (!IsValidConfiguration(x_m, y_m, theta_rad)) {
        SBPL_PRINTF("WARNING: start configuration %d %d %d is invalid\n", x, y,
                    theta);
    }

    EnvNAVXYTHETACHashEntry_t* OutHashEntry;
    if ((OutHashEntry = (this->*GetHashEntry)(x_m, y_m, theta_rad)) == NULL) {
        OutHashEntry = (this->*CreateNewHashEntry)(x_m, y_m, theta_rad);
        OutHashEntry->parentStateID = OutHashEntry->stateID;
        OutHashEntry->valid_children_ratio = 1;
    }

    // need to recompute start heuristics?
    if (EnvNAVXYTHETAC.startstateid != OutHashEntry->stateID) {
        bNeedtoRecomputeStartHeuristics = true;
        // because termination condition can be not all states TODO - make it
        // dependent on term. condition
        bNeedtoRecomputeGoalHeuristics = true;
    }

    // set start
    EnvNAVXYTHETAC.startstateid = OutHashEntry->stateID;
    EnvNAVXYTHETACCfg.StartX_c = x_m;
    EnvNAVXYTHETACCfg.StartY_c = y_m;
    EnvNAVXYTHETACCfg.StartTheta = theta_rad;

    return EnvNAVXYTHETAC.startstateid;
}  // SetStart

int EnvironmentNAVXYTHETAC::SetGoal(double x_m, double y_m, double theta_rad) {
    int x = CONTXY2DISC(x_m, EnvNAVXYTHETACCfg.cellsize_m);
    int y = CONTXY2DISC(y_m, EnvNAVXYTHETACCfg.cellsize_m);
    int theta = ContTheta2DiscNew(theta_rad);

    SBPL_PRINTF("env: setting goal to %.3f %.3f %.3f (%d %d %d)\n", x_m, y_m,
                theta_rad, x, y, theta);

    if (!IsWithinMapCell(x, y)) {
        SBPL_ERROR(
            "ERROR: tring to set goal cell %d %d that is outside the map\n", x,
            y);
        return -1;
    }

    if (!IsValidConfiguration(x_m, y_m, theta_rad)) {
        SBPL_PRINTF("WARNING: goal configuration is invalid\n");
    }

    EnvNAVXYTHETACHashEntry_t* OutHashEntry;
    if ((OutHashEntry = (this->*GetHashEntry)(x_m, y_m, theta_rad)) == NULL) {
        // HAVE TO CREATET A NEW ENTRY
        OutHashEntry = (this->*CreateNewHashEntry)(x_m, y_m, theta_rad);
        OutHashEntry->parentStateID = OutHashEntry->stateID;
        OutHashEntry->valid_children_ratio = 1;
    }

    // need to recompute start heuristics?
    if (EnvNAVXYTHETAC.goalstateid != OutHashEntry->stateID) {
        // because termination condition may not plan all the way to the new
        // goal
        bNeedtoRecomputeStartHeuristics = true;

        // because goal heuristicsf change
        bNeedtoRecomputeGoalHeuristics = true;

        EnvNAVXYTHETAC.goalstateid = OutHashEntry->stateID;

        EnvNAVXYTHETACCfg.EndX_c = x_m;
        EnvNAVXYTHETACCfg.EndY_c = y_m;
        EnvNAVXYTHETACCfg.EndTheta = theta_rad;

        return EnvNAVXYTHETAC.goalstateid;
    }
    return 0;
}  // SetGoal

int EnvironmentNAVXYTHETAC::ContTheta2DiscNew(double theta) const {
    if (bUseNonUniformAngles) {
        return ContTheta2DiscFromSet(theta);
    } else {
        return ContTheta2Disc(theta, EnvNAVXYTHETACCfg.NumThetaDirs);
    }
}

int EnvironmentNAVXYTHETAC::ContTheta2DiscFromSet(double theta) const {
    theta = normalizeAngle(theta);
    // ThetaDirs should contain extra angle (2PI) for overlap
    if (EnvNAVXYTHETACCfg.NumThetaDirs >=
        (int)EnvNAVXYTHETACCfg.ThetaDirs.size()) {
        throw SBPL_Exception(
            "ERROR: list of bin angles are not properly set to use function "
            "ContTheta2DiscFromSet");
    }

    int lower_bound_ind = -1;
    int upper_bound_ind = -1;

    for (int i = 1; i < (int)EnvNAVXYTHETACCfg.ThetaDirs.size(); ++i) {
        if ((EnvNAVXYTHETACCfg.ThetaDirs[i]) >= theta) {
            lower_bound_ind = i - 1;
            upper_bound_ind = i;
            break;
        }
    }

    // Critical error if could not find bin location from given angle
    if (lower_bound_ind == -1) {
        std::stringstream ss;
        ss << "ERROR: unable to find bin index for angle " << theta;
        throw SBPL_Exception(ss.str());
    }

    // Get closest angle of two
    double angle_low = EnvNAVXYTHETACCfg.ThetaDirs[lower_bound_ind];
    double angle_up = EnvNAVXYTHETACCfg.ThetaDirs[upper_bound_ind];
    double diff_low = fabs(theta - angle_low);
    double diff_up = fabs(theta - angle_up);

    if (diff_low < diff_up) {
        return lower_bound_ind;
    } else {
        // Wrap upper bound index around when it reaches last index (assumed to
        // be 2PI)
        if (upper_bound_ind == EnvNAVXYTHETACCfg.NumThetaDirs) {
            upper_bound_ind = 0;
        }
        return upper_bound_ind;
    }
}  // ContTheta2DiscFromSet

bool EnvironmentNAVXYTHETAC::IsWithinMapCell(int X, int Y) {
    return (X >= 0 && X < EnvNAVXYTHETACCfg.EnvWidth_c && Y >= 0 &&
            Y < EnvNAVXYTHETACCfg.EnvHeight_c);
}

// now continuous pose are passed to this function directly
bool EnvironmentNAVXYTHETAC::IsValidConfiguration(double X, double Y,
                                                  double Theta) {
    std::vector<sbpl_2Dcell_t> footprint;
    sbpl_xy_theta_pt_t pose;

    // Compute continuous pose
    pose.x = X;
    pose.y = Y;
    pose.theta = Theta;

    // compute footprint cells
    get_2d_footprint_cells(EnvNAVXYTHETACCfg.FootprintPolygon, &footprint, pose,
                           EnvNAVXYTHETACCfg.cellsize_m);

    // iterate over all footprint cells
    for (int find = 0; find < (int)footprint.size(); find++) {
        int x = footprint.at(find).x;
        int y = footprint.at(find).y;

        if (x < 0 || x >= EnvNAVXYTHETACCfg.EnvWidth_c || y < 0 ||
            y >= EnvNAVXYTHETACCfg.EnvHeight_c ||
            EnvNAVXYTHETACCfg.Grid2D[x][y] >= EnvNAVXYTHETACCfg.obsthresh) {
            return false;
        }
    }

    return true;
}  // IsValidConfiguration

void EnvironmentNAVXYTHETAC::SetAllActionsandAllOutcomes(CMDPSTATE* state) {
    int cost;
#if DEBUG
    if (state->StateID >= (int)StateID2CoordTable.size()) {
        throw SBPL_Exception("ERROR in Env... function: stateID illegal");
    }

    if ((int)state->Actions.size() != 0) {
        throw SBPL_Exception(
            "ERROR in Env_setAllActionsandAllOutcomes: actions already exist "
            "for the state");
    }
#endif

    // goal state should be absorbing
    if (state->StateID == EnvNAVXYTHETAC.goalstateid) {
        return;
    }

    // get X, Y for the state
    EnvNAVXYTHETACHashEntry_t* HashEntry = StateID2CoordTable[state->StateID];

    // iterate through actions
    for (int aind = 0; aind < EnvNAVXYTHETACCfg.actionwidth; ++aind) {
        EnvNAVXYTHETACAction_t* nav3daction =
            &EnvNAVXYTHETACCfg.ActionsV[(unsigned int)HashEntry->Theta][aind];
        double newX =
            HashEntry->X + nav3daction->dX * EnvNAVXYTHETACCfg.cellsize_m;
        double newY =
            HashEntry->Y + nav3daction->dY * EnvNAVXYTHETACCfg.cellsize_m;
        double newTheta = (double)normalizeDiscAngle(nav3daction->endtheta);

        int coverX = CONTXY2DISC(newX, EnvNAVXYTHETACCfg.cellsize_m);
        int coverY = CONTXY2DISC(newY, EnvNAVXYTHETACCfg.cellsize_m);

        // skip the invalid cells
        if (!IsValidCell(coverX, coverY)) {
            continue;
        }

        // get cost
        cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta,
                             nav3daction);
        if (cost >= INFINITECOST) {
            continue;
        }

        // add the action
        CMDPACTION* action = state->AddAction(aind);

#if TIME_DEBUG
        clock_t currenttime = clock();
#endif
        EnvNAVXYTHETACHashEntry_t* OutHashEntry;
        if ((OutHashEntry = (this->*GetHashEntry)(newX, newY, newTheta)) ==
            NULL) {
            // have to create a new entry
            OutHashEntry = (this->*CreateNewHashEntry)(newX, newY, newTheta);
        }
        action->AddOutcome(OutHashEntry->stateID, cost, 1.0);

#if TIME_DEBUG
        time3_addallout += clock() - currenttime;
#endif
    }

}  // SetAllActionsandAllOutcomes

int EnvironmentNAVXYTHETAC::GetFromToHeuristic(int FromStateID, int ToStateID) {
#if USE_HEUR == 0
    return 0;
#endif

#if DEBUG
    if (FromStateID >= (int)StateID2CoordTable.size() ||
        ToStateID >= (int)StateID2CoordTable.size()) {
        SBPL_ERROR("ERROR in EnvNAVXYTHETAC... function: stateID illegal\n");
        throw SBPL_Exception();
    }
#endif

    // get X, Y for the state
    EnvNAVXYTHETACHashEntry_t* FromHashEntry = StateID2CoordTable[FromStateID];
    EnvNAVXYTHETACHashEntry_t* ToHashEntry = StateID2CoordTable[ToStateID];

    // TODO - check if one fo the gridsearches already computed and then use it.

    return (int)(NAVXYTHETAC_COSTMULT_MTOMM *
                 EuclideanDistance_m(FromHashEntry->X, FromHashEntry->Y,
                                     ToHashEntry->X, ToHashEntry->Y) /
                 EnvNAVXYTHETACCfg.nominalvel_mpersecs);

}  // GetFromToHeuristic

void EnvironmentNAVXYTHETAC::GetCoordFromState(int stateID, double& x,
                                               double& y, double& theta) const {
    EnvNAVXYTHETACHashEntry_t* HashEntry = StateID2CoordTable[stateID];
    x = HashEntry->X;
    y = HashEntry->Y;
    theta = HashEntry->Theta;
}

int EnvironmentNAVXYTHETAC::GetStateFromCoord(double x, double y,
                                              double theta) {
    EnvNAVXYTHETACHashEntry_t* OutHashEntry;
    if ((OutHashEntry = (this->*GetHashEntry)(x, y, theta)) == NULL) {
        // have to create a new entry
        OutHashEntry = (this->*CreateNewHashEntry)(x, y, theta);
    }
    return OutHashEntry->stateID;
}

int EnvironmentNAVXYTHETAC::GetStartHeuristic(int stateID) {
#if USE_HEUR == 0
    return 0;
#endif

#if DEBUG
    if (stateID >= (int)StateID2CoordTable.size()) {
        throw SBPL_Exception(
            "ERROR in EnvNAVXYTHETAC... function: stateID illegal");
    }
#endif

    EnvNAVXYTHETACHashEntry_t* HashEntry = StateID2CoordTable[stateID];
    int h2D = grid2Dsearchfromstart->getlowerboundoncostfromstart_inmm(
        HashEntry->X, HashEntry->Y);
    int hEuclid = (int)(NAVXYTHETAC_COSTMULT_MTOMM *
                        EuclideanDistance_m(EnvNAVXYTHETACCfg.StartX_c,
                                            EnvNAVXYTHETACCfg.StartY_c,
                                            HashEntry->X, HashEntry->Y));

    // define this function if it is used in the planner (heuristic backward
    // search would use it)
    return (int)(((double)__max(h2D, hEuclid)) /
                 EnvNAVXYTHETACCfg.nominalvel_mpersecs);
}

// goal heuristic returned in mm
int EnvironmentNAVXYTHETAC::GetPenaltyGoalHeuristic(int stateID) {
#if DEBUG
    if (stateID >= (int)StateID2CoordTable.size()) {
        throw SBPL_Exception(
            "ERROR in EnvNAVXYTHETAC... function: stateID illegal");
    }
#endif
    EnvNAVXYTHETACHashEntry_t* HashEntry = StateID2CoordTable[stateID];
    int hashentryx = CONTXY2DISC(HashEntry->X, EnvNAVXYTHETACCfg.cellsize_m);
    int hashentryy = CONTXY2DISC(HashEntry->Y, EnvNAVXYTHETACCfg.cellsize_m);

    int h2D = grid2Dsearchfromgoal->getlowerboundoncostfromstart_inmm(
        hashentryx, hashentryy);
    int h = (int)((double)h2D / EnvNAVXYTHETACCfg.nominalvel_mpersecs);

    return h;
}

int EnvironmentNAVXYTHETAC::GetGoalHeuristic(int stateID) {
#if DEBUG
    if (stateID >= (int)StateID2CoordTable.size()) {
        throw SBPL_Exception(
            "ERROR in EnvNAVXYTHETAC... function: stateID illegal");
    }
#endif
    EnvNAVXYTHETACHashEntry_t* HashEntry = StateID2CoordTable[stateID];
    auto parent = StateID2CoordTable[HashEntry->parentStateID];
    int hashentryx = CONTXY2DISC(HashEntry->X, EnvNAVXYTHETACCfg.cellsize_m);
    int hashentryy = CONTXY2DISC(HashEntry->Y, EnvNAVXYTHETACCfg.cellsize_m);
    double dH = 1.00;

#if PENALTY

    double position = parent->valid_children_ratio;
    double amp = EnvNAVXYTHETACCfg.min_stepsize;
    double T = 2.0;
    double stepsize = EnvNAVXYTHETACCfg.cellsize_m * amp * position;
    double radius =
        stepsize *
        EnvNAVXYTHETACCfg
            .R_times;  // largest Euclidean distance + Angular distance
    int range = (int)(radius * 0.5 / EnvNAVXYTHETACCfg.cellsize_m + 0.5);

    std::vector<int> binid;
    std::vector<int> neighbors;
    for (int i = -range; i <= range; ++i) {
        for (int j = -range; j <= range; ++j) {
            if (!IsValidCell((hashentryx + i), (hashentryy + j))) {
                continue;
            }
            neighbors.push_back((hashentryx + i) +
                                (hashentryy + j) *
                                    EnvNAVXYTHETACCfg.EnvWidth_c);
        }
    }

    double Edist = 100;
    double closest = 100;
    double Sdist = closest;
    std::vector<EnvNAVXYTHETACHashEntry_t*>* neighborcell;
    EnvNAVXYTHETACHashEntry_t* tempEntry;
    for (int i = 0; i < (int)neighbors.size(); ++i) {
        neighborcell = &NNtable[neighbors[i]];
        for (int ind = 0; ind < (int)neighborcell->size(); ++ind) {
            tempEntry = neighborcell->at(ind);

            if (tempEntry->parentStateID !=
                    HashEntry->parentStateID &&  // not siblings/ itself
                tempEntry->stateID != HashEntry->parentStateID) {  // not parent

                Edist = sqrt((tempEntry->X - HashEntry->X) *
                                 (tempEntry->X - HashEntry->X) +
                             (tempEntry->Y - HashEntry->Y) *
                                 (tempEntry->Y - HashEntry->Y));

                Sdist = stepsize / 2.0 *
                            (1.0 - cos(tempEntry->Theta - HashEntry->Theta)) +
                        Edist;
                closest = closest < Sdist ? closest : Sdist;
            }
        }
    }

    closest = closest < radius ? closest : radius;
    dH = T * (1.0 - closest / radius) + 1.0;

#endif

    int h2D = grid2Dsearchfromgoal->getlowerboundoncostfromstart_inmm(
        hashentryx, hashentryy);
    int h = (int)((double)h2D / EnvNAVXYTHETACCfg.nominalvel_mpersecs * dH);

    return h;

}  // getgoalheuristic

double EnvironmentNAVXYTHETAC::EuclideanDistance_m(double X1, double Y1,
                                                   double X2, double Y2) {
    double sqdist = ((X1 - X2) * (X1 - X2) + (Y1 - Y2) * (Y1 - Y2));
    return sqrt(sqdist);
}

// when cell (0,0) changes its status it also does the same for the 3D states
// whose incoming actions are potentially affected when cell (0,0) changes its
// status
void EnvironmentNAVXYTHETAC::ComputeReplanningData() {
    // iterate over all actions
    // orientations
    for (int tind = 0; tind < EnvNAVXYTHETACCfg.NumThetaDirs; tind++) {
        // actions
        for (int aind = 0; aind < EnvNAVXYTHETACCfg.actionwidth; aind++) {
            // compute replanning data for this action
            ComputeReplanningDataforAction(
                &EnvNAVXYTHETACCfg.ActionsV[tind][aind]);
        }
    }
}

void EnvironmentNAVXYTHETAC::ComputeReplanningDataforAction(
    EnvNAVXYTHETACAction_t* action) {
    int j;

    // iterate over all the cells involved in the action
    sbpl_xy_theta_cell_t startcell3d, endcell3d;
    for (int i = 0; i < (int)action->intersectingcellsV.size(); i++) {
        // compute the translated affected search Pose - what state has an
        // outgoing action whose intersecting cell is at 0,0
        startcell3d.theta = action->starttheta;
        startcell3d.x = -action->intersectingcellsV.at(i).x;
        startcell3d.y = -action->intersectingcellsV.at(i).y;

        // compute the translated affected search Pose - what state has an
        // incoming action whose intersecting cell is at 0,0
        if (bUseNonUniformAngles) {
            endcell3d.theta = normalizeDiscAngle(action->endtheta);
        } else {
            endcell3d.theta = NORMALIZEDISCTHETA(
                action->endtheta, EnvNAVXYTHETACCfg.NumThetaDirs);
        }
        endcell3d.x = startcell3d.x + action->dX;
        endcell3d.y = startcell3d.y + action->dY;

        // store the cells if not already there
        for (j = 0; j < (int)affectedsuccstatesV.size(); j++) {
            if (affectedsuccstatesV.at(j) == endcell3d) {
                break;
            }
        }
        if (j == (int)affectedsuccstatesV.size()) {
            affectedsuccstatesV.push_back(endcell3d);
        }

        for (j = 0; j < (int)affectedpredstatesV.size(); j++) {
            if (affectedpredstatesV.at(j) == startcell3d) {
                break;
            }
        }
        if (j == (int)affectedpredstatesV.size()) {
            affectedpredstatesV.push_back(startcell3d);
        }
    }  // over intersecting cells

    // add the centers since with h2d we are using these in cost computations
    // ---intersecting cell = origin
    // compute the translated affected search Pose - what state has an outgoing
    // action whose intersecting cell is at 0,0
    startcell3d.theta = action->starttheta;
    startcell3d.x = -0;
    startcell3d.y = -0;

    // compute the translated affected search Pose - what state has an incoming
    // action whose intersecting cell is at 0,0
    if (bUseNonUniformAngles) {
        endcell3d.theta = normalizeDiscAngle(action->endtheta);
    } else {
        endcell3d.theta = NORMALIZEDISCTHETA(action->endtheta,
                                             EnvNAVXYTHETACCfg.NumThetaDirs);
    }
    endcell3d.x = startcell3d.x + action->dX;
    endcell3d.y = startcell3d.y + action->dY;

    // store the cells if not already there
    for (j = 0; j < (int)affectedsuccstatesV.size(); j++) {
        if (affectedsuccstatesV.at(j) == endcell3d) {
            break;
        }
    }
    if (j == (int)affectedsuccstatesV.size()) {
        affectedsuccstatesV.push_back(endcell3d);
    }

    for (j = 0; j < (int)affectedpredstatesV.size(); j++) {
        if (affectedpredstatesV.at(j) == startcell3d) {
            break;
        }
    }
    if (j == (int)affectedpredstatesV.size()) {
        affectedpredstatesV.push_back(startcell3d);
    }

    //---intersecting cell = outcome state
    // compute the translated affected search Pose - what state has an outgoing
    // action whose intersecting cell is at 0,0
    startcell3d.theta = action->starttheta;
    startcell3d.x = -action->dX;
    startcell3d.y = -action->dY;

    // compute the translated affected search Pose - what state has an incoming
    // action whose intersecting cell is at 0,0
    if (bUseNonUniformAngles) {
        endcell3d.theta = normalizeDiscAngle(action->endtheta);
    } else {
        endcell3d.theta = NORMALIZEDISCTHETA(action->endtheta,
                                             EnvNAVXYTHETACCfg.NumThetaDirs);
    }
    endcell3d.x = startcell3d.x + action->dX;
    endcell3d.y = startcell3d.y + action->dY;

    for (j = 0; j < (int)affectedsuccstatesV.size(); j++) {
        if (affectedsuccstatesV.at(j) == endcell3d) {
            break;
        }
    }
    if (j == (int)affectedsuccstatesV.size()) {
        affectedsuccstatesV.push_back(endcell3d);
    }

    for (j = 0; j < (int)affectedpredstatesV.size(); j++) {
        if (affectedpredstatesV.at(j) == startcell3d) {
            break;
        }
    }
    if (j == (int)affectedpredstatesV.size()) {
        affectedpredstatesV.push_back(startcell3d);
    }
}  // computereplanningdataforaction

bool EnvironmentNAVXYTHETAC::ReadMotionPrimitives(FILE* fMotPrims) {
    char sTemp[1024], sExpected[1024];
    float fTemp;
    int dTemp;
    int totalNumofActions = 0;

    SBPL_INFO("Reading in motion primitives...");
    fflush(stdout);

    // read in the resolution
    strcpy(sExpected, "resolution_m:");
    if (fscanf(fMotPrims, "%s", sTemp) == 0) {
        return false;
    }
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        fflush(stdout);
        return false;
    }
    if (fscanf(fMotPrims, "%f", &fTemp) == 0) {
        return false;
    }
    if (fabs(fTemp - EnvNAVXYTHETACCfg.cellsize_m) > ERR_EPS) {
        SBPL_ERROR(
            "ERROR: invalid resolution %f (instead of %f) in the dynamics "
            "file\n",
            fTemp, EnvNAVXYTHETACCfg.cellsize_m);
        fflush(stdout);
        return false;
    }
    SBPL_INFO("resolution_m: %f\n", fTemp);

    // read in stepsize: times of resolution
    strcpy(sExpected, "min_stepsize:");
    if (fscanf(fMotPrims, "%s", sTemp) == 0) {
        return false;
    }
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        fflush(stdout);
        return false;
    }
    if (fscanf(fMotPrims, "%f", &fTemp) == 0) {
        return false;
    }
    SBPL_INFO("min_stepsize: %f\n", fTemp);
    EnvNAVXYTHETACCfg.min_stepsize = fTemp;

    // read in turning radius
    if (fscanf(fMotPrims, "%s", sTemp) == 0) {
        return false;
    }
    // SBPL_INFO("sTemp: %s\n", sTemp);
    if (strncmp(sTemp, "min_turning_radius_m:", 21) == 0) {
        bUseNonUniformAngles = true;
    }
    SBPL_INFO("bUseNonUniformAngles = %d", bUseNonUniformAngles);

    if (bUseNonUniformAngles) {
        float min_turn_rad;
        strcpy(sExpected, "min_turning_radius_m:");
        if (strcmp(sTemp, sExpected) != 0) {
            SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
            fflush(stdout);
            return false;
        }
        if (fscanf(fMotPrims, "%f", &min_turn_rad) == 0) {
            return false;
        }
        SBPL_PRINTF("min_turn_rad: %f\n", min_turn_rad);
        fflush(stdout);
        if (fscanf(fMotPrims, "%s", sTemp) == 0) {
            return false;
        }
    }

    // read in the angular resolution
    strcpy(sExpected, "numberofangles:");
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fMotPrims, "%d", &dTemp) == 0) {
        return false;
    }
    if (dTemp != EnvNAVXYTHETACCfg.NumThetaDirs) {
        SBPL_ERROR(
            "ERROR: invalid angular resolution %d angles (instead of %d "
            "angles) in the motion primitives file\n",
            dTemp, EnvNAVXYTHETACCfg.NumThetaDirs);
        return false;
    }
    SBPL_PRINTF("numberofangles: %d\n", dTemp);
    EnvNAVXYTHETACCfg.NumThetaDirs = dTemp;

    if (bUseNonUniformAngles) {
        // read in angles
        EnvNAVXYTHETACCfg.ThetaDirs.clear();
        for (int i = 0; i < EnvNAVXYTHETACCfg.NumThetaDirs; i++) {
            std::ostringstream string_angle_index;
            string_angle_index << i;
            std::string angle_string = "angle:" + string_angle_index.str();

            float angle;
            strcpy(sExpected, angle_string.c_str());
            if (fscanf(fMotPrims, "%s", sTemp) == 0) {
                return false;
            }
            if (strcmp(sTemp, sExpected) != 0) {
                SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
                return false;
            }
            if (fscanf(fMotPrims, "%f", &angle) == 0) {
                return false;
            }
            SBPL_PRINTF("%s %f\n", angle_string.c_str(), angle);
            EnvNAVXYTHETACCfg.ThetaDirs.push_back(angle);
        }
        EnvNAVXYTHETACCfg.ThetaDirs.push_back(
            2.0 * M_PI);  // Add 2 PI at end for overlap
    }

    // read in the total number of actions
    strcpy(sExpected, "totalnumberofprimitives:");
    if (fscanf(fMotPrims, "%s", sTemp) == 0) {
        return false;
    }
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fMotPrims, "%d", &totalNumofActions) == 0) {
        return false;
    }
    SBPL_PRINTF("totalnumberofprimitives: %d\n", totalNumofActions);

    // Read in motion primitive for each action
    for (int i = 0; i < totalNumofActions; i++) {
        SBPL_xytheta_mprimitive motprim;

        if (!EnvironmentNAVXYTHETAC::ReadinMotionPrimitive(&motprim,
                                                           fMotPrims)) {
#if DEBUG
            printf("wrong number: %d\n", i);
#endif
            return false;
        }

        EnvNAVXYTHETACCfg.mprimV.push_back(motprim);
    }
    SBPL_PRINTF("done");
    SBPL_FFLUSH(stdout);
    return true;
}  // ReadMotionPrimitives

bool EnvironmentNAVXYTHETAC::ReadinMotionPrimitive(
    SBPL_xytheta_mprimitive* pMotPrim, FILE* fIn) {
    char sTemp[1024];
    int dTemp;
    char sExpected[1024];
    int numofIntermPoses;
    float fTemp;

    // read in actionID
    strcpy(sExpected, "primID:");
    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        fflush(stdout);
        return false;
    }
    if (fscanf(fIn, "%d", &pMotPrim->motprimID) != 1) {
        return false;
    }

    // read in start angle
    strcpy(sExpected, "startangle_c:");
    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fIn, "%d", &dTemp) == 0) {
        SBPL_ERROR("ERROR reading startangle\n");
        return false;
    }
    pMotPrim->starttheta_c = dTemp;

    // read endpose into endcell
    // actually it's already a discretized value
    // which set a cell as a unit
    strcpy(sExpected, "endpose_c:");
    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }

    if (ReadinCell(&pMotPrim->endcell, fIn) == false) {
        SBPL_ERROR("ERROR: failed to read in endsearchpose\n");
        return false;
    }

    // read in action cost
    strcpy(sExpected, "additionalactioncostmult:");
    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fIn, "%d", &dTemp) != 1) {
        return false;
    }
    pMotPrim->additionalactioncostmult = dTemp;

    if (bUseNonUniformAngles) {
        // read in action turning radius
        strcpy(sExpected, "turning_radius:");
        if (fscanf(fIn, "%s", sTemp) == 0) {
            return false;
        }
        if (strcmp(sTemp, sExpected) != 0) {
            SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
            return false;
        }
        if (fscanf(fIn, "%f", &fTemp) != 1) {
            return false;
        }
        pMotPrim->turning_radius = fTemp;
    }

    // read in intermediate poses
    strcpy(sExpected, "intermediateposes:");
    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fIn, "%d", &numofIntermPoses) != 1) {
        return false;
    }
    // all intermposes should be with respect to 0,0 as starting pose since it
    // will be added later and should be done after the action is rotated by
    // initial orientation

    for (int i = 0; i < numofIntermPoses; i++) {
        sbpl_xy_theta_pt_t intermpose;
        if (ReadinPose(&intermpose, fIn) == false) {
            SBPL_ERROR("ERROR: failed to read in intermediate poses\n");
            return false;
        }
        pMotPrim->intermptV.push_back(intermpose);
    }

    // Check that the last pose of the motion matches (within lattice
    // resolution) the designated end pose of the primitive
    sbpl_xy_theta_pt_t sourcepose;
    sourcepose.x = DISCXY2CONT(0, EnvNAVXYTHETACCfg.cellsize_m);
    sourcepose.y = DISCXY2CONT(0, EnvNAVXYTHETACCfg.cellsize_m);
    sourcepose.theta = DiscTheta2ContNew(pMotPrim->starttheta_c);
    double mp_endx_m =
        sourcepose.x + pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].x;
    double mp_endy_m =
        sourcepose.y + pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].y;
    double mp_endtheta_rad =
        pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].theta;

    int endtheta_c;
    int endx_c = CONTXY2DISC(mp_endx_m, EnvNAVXYTHETACCfg.cellsize_m);
    int endy_c = CONTXY2DISC(mp_endy_m, EnvNAVXYTHETACCfg.cellsize_m);
    endtheta_c = ContTheta2DiscNew(mp_endtheta_rad);
    if (endx_c != pMotPrim->endcell.x || endy_c != pMotPrim->endcell.y ||
        endtheta_c != pMotPrim->endcell.theta) {
        SBPL_ERROR(
            "ERROR: incorrect primitive %d with startangle=%d "
            "last interm point %f %f %f does not match end pose %d %d %d\n",
            pMotPrim->motprimID, pMotPrim->starttheta_c,

            pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].x,
            pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].y,
            pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].theta,

            pMotPrim->endcell.x, pMotPrim->endcell.y, pMotPrim->endcell.theta);

        SBPL_FFLUSH(stdout);
        return false;
    }

    return true;
}  // ReadinMotionPrimitive

bool EnvironmentNAVXYTHETAC::ReadinCell(sbpl_xy_theta_cell_t* cell, FILE* fIn) {
    char sTemp[60];

    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    cell->x = atoi(sTemp);

    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    cell->y = atoi(sTemp);

    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    cell->theta = atoi(sTemp);

    // normalize the angle
    cell->theta = normalizeDiscAngle(cell->theta);
    return true;
}  // ReadinCell

bool EnvironmentNAVXYTHETAC::ReadinPose(sbpl_xy_theta_pt_t* pose, FILE* fIn) {
    char sTemp[60];

    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    pose->x = atof(sTemp);

    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    pose->y = atof(sTemp);

    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    pose->theta = atof(sTemp);

    // normalize the angle
    pose->theta = normalizeAngle(pose->theta);
    return true;
}  // ReadinPose

bool EnvironmentNAVXYTHETAC::IsValidCell(int X, int Y) {
    return (X >= 0 && X < EnvNAVXYTHETACCfg.EnvWidth_c && Y >= 0 &&
            Y < EnvNAVXYTHETACCfg.EnvHeight_c &&
            EnvNAVXYTHETACCfg.Grid2D[X][Y] < EnvNAVXYTHETACCfg.obsthresh);
}

int EnvironmentNAVXYTHETAC::normalizeDiscAngle(int theta) const {
    if (bUseNonUniformAngles) {
        if (theta < 0) {
            theta += EnvNAVXYTHETACCfg.NumThetaDirs;
        }
        if (theta >= EnvNAVXYTHETACCfg.NumThetaDirs) {
            theta -= EnvNAVXYTHETACCfg.NumThetaDirs;
        }
    } else {
        theta = NORMALIZEDISCTHETA(theta, EnvNAVXYTHETACCfg.NumThetaDirs);
    }
    return theta;
}  // normalizeDiscAngle

EnvNAVXYTHETACHashEntry_t* EnvironmentNAVXYTHETAC::GetHashEntry_hash(
    double X, double Y, double Theta) {
    // returns a hash bin
    unsigned int coverX = CONTXY2DISC(X, EnvNAVXYTHETACCfg.cellsize_m);
    unsigned int coverY = CONTXY2DISC(Y, EnvNAVXYTHETACCfg.cellsize_m);
    unsigned int coverTheta = ContTheta2DiscNew(Theta);
    int binid = GETHASHBIN(coverX, coverY, coverTheta);

#if DEBUG
    if ((int)Coord2StateIDHashTable[binid].size() > 5) {
        SBPL_FPRINTF(
            fDeb, "WARNING: Hash table has a bin %d (X=%d Y=%d) of size %d\n",
            binid, coverX, coverY, (int)Coord2StateIDHashTable[binid].size());
        PrintHashTableHist(fDeb);
    }
#endif

    std::vector<EnvNAVXYTHETACHashEntry_t*>* binV =
        &Coord2StateIDHashTable[binid];

    // iterate
    for (int ind = 0; ind < (int)binV->size(); ++ind) {
        EnvNAVXYTHETACHashEntry_t* hashentry = binV->at(ind);

        // check only identical nodes
        if (fabs(hashentry->X - X) < 0.00001 &&
            fabs(hashentry->Y - Y) < 0.00001 &&
            fabs(hashentry->Theta - Theta) < 0.00001) {
            return hashentry;
        }
    }

    return NULL;
}
// GetHashEntry_hash

EnvNAVXYTHETACHashEntry_t* EnvironmentNAVXYTHETAC::CreateNewHashEntry_hash(
    double X, double Y, double Theta) {
    int i;

    // build a new hash entry
    EnvNAVXYTHETACHashEntry_t* HashEntry = new EnvNAVXYTHETACHashEntry_t;

    // set hash entry with real value
    // produce HASHBIN value with integer value
    HashEntry->X = X;
    HashEntry->Y = Y;
    HashEntry->Theta = Theta;
    HashEntry->iteration = 0;

    // by CONTXY2DISC macro
    unsigned int coverX = CONTXY2DISC(X, EnvNAVXYTHETACCfg.cellsize_m);
    unsigned int coverY = CONTXY2DISC(Y, EnvNAVXYTHETACCfg.cellsize_m);
    unsigned int coverTheta = ContTheta2DiscNew(Theta);
    // int coverTheta = normalizeDiscAngle( ContTheta2DiscNew(Theta) );
    i = GETHASHBIN(coverX, coverY, coverTheta);

    // create a new id, which is the last element's index in StateID2CoordTable
    HashEntry->stateID = StateID2CoordTable.size();
    StateID2CoordTable.push_back(HashEntry);

    Coord2StateIDHashTable[i].push_back(HashEntry);
#if PENALTY
    NNtable[coverX + coverY * EnvNAVXYTHETACCfg.EnvWidth_c].push_back(
        HashEntry);
#endif

    // insert into and initialize the mappings
    int* entry = new int[NUMOFINDICES_STATEID2IND];
    StateID2IndexMapping.push_back(entry);

    for (i = 0; i < NUMOFINDICES_STATEID2IND; ++i) {
        StateID2IndexMapping[HashEntry->stateID][i] = -1;
    }
    if (HashEntry->stateID != (int)StateID2IndexMapping.size() - 1) {
        throw SBPL_Exception(
            "ERROR in Env... function: last state has incorrect stateID");
    }

    return HashEntry;

}  // CreateNewHashEntry_hash

EnvNAVXYTHETACHashEntry_t* EnvironmentNAVXYTHETAC::GetHashEntry_lookup(
    double X, double Y, double Theta) {
    if (X < 0 || X >= EnvNAVXYTHETACCfg.EnvWidth_c || Y < 0 ||
        Y >= EnvNAVXYTHETACCfg.EnvHeight_c || Theta < 0 ||
        Theta >= EnvNAVXYTHETACCfg.NumThetaDirs) {
        return NULL;
    }
    /*
     *
     *   int coverX = CONTXY2DISC(X,EnvNAVXYTHETACCfg.cellsize_m);
     *   int coverY = CONTXY2DISC(Y,EnvNAVXYTHETACCfg.cellsize_m);
     *   int coverTheta = ContTheta2DiscNew(Theta);
     */

    //    if (fabs(hashentry->X - X) <= 0.00001 &&
    // 	   fabs(hashentry->Y - Y) <= 0.00001 &&
    // 	   fabs(hashentry->Theta - Theta) <= 0.00001){
    // 	   return hashentry;
    //    }

    // int index = XYTHETA2INDEX(coverX,coverY,(int)Theta);
    int index = (int)XYTHETA2INDEX(X, Y, Theta);

    return Coord2StateIDHashTable_lookup[index];
}

EnvNAVXYTHETACHashEntry_t* EnvironmentNAVXYTHETAC::CreateNewHashEntry_lookup(
    double X, double Y, double Theta) {
    int i;
#if TIME_DEBUG
    clock_t currenttime = clock();
#endif

    EnvNAVXYTHETACHashEntry_t* HashEntry = new EnvNAVXYTHETACHashEntry_t;

    // modified
    int coverX = CONTXY2DISC(X, EnvNAVXYTHETACCfg.cellsize_m);
    int coverY = CONTXY2DISC(Y, EnvNAVXYTHETACCfg.cellsize_m);
    int coverTheta = ContTheta2DiscNew(Theta);

    HashEntry->X = X;
    HashEntry->Y = Y;
    HashEntry->Theta = Theta;
    HashEntry->iteration = 0;
    HashEntry->stateID = StateID2CoordTable.size();

    // insert into the tables
    StateID2CoordTable.push_back(HashEntry);

    int index = XYTHETA2INDEX(X, Y, Theta);

    // #if DEBUG
    // 	if(Coord2StateIDHashTable_lookup[index] != NULL){
    // 		throw SBPL_Exception("ERROR: createing hash entry for non-NULL
    // hashentry");
    // 	}
    // #endif

    Coord2StateIDHashTable_lookup[index] = HashEntry;

    // insert into and initialzie the mappings
    int* entry = new int[NUMOFINDICES_STATEID2IND];
    StateID2IndexMapping.push_back(entry);

    for (i = 0; i < NUMOFINDICES_STATEID2IND; i++) {
        StateID2IndexMapping[HashEntry->stateID][i] = -1;
    }

    if (HashEntry->stateID != (int)StateID2IndexMapping.size() - 1) {
        throw SBPL_Exception(
            "ERROR in Env... function: last state has incorrect stateID");
    }

#if TIME_DEBUG
    time_createhash += clock() - currenttime;
#endif

    return HashEntry;
}

int EnvironmentNAVXYTHETAC::GetActionCost(double SourceX, double SourceY,
                                          double SourceTheta,
                                          EnvNAVXYTHETACAction_t* action) {
    double cellsize = EnvNAVXYTHETACCfg.cellsize_m;

    sbpl_2Dcell_t cell;
    sbpl_xy_theta_cell_t interm3Dcell;
    int i;

    int intSourceX = CONTXY2DISC(SourceX, cellsize);
    int intSourceY = CONTXY2DISC(SourceY, cellsize);

    int intendX = CONTXY2DISC(SourceX + action->dX * cellsize, cellsize);
    int intendY = CONTXY2DISC(SourceY + action->dY * cellsize, cellsize);

    // collision check-go over bounding box (minput and maxpt) to test validity
    // and skip
    // testing boundaries below.
    // farthest pts go first
    if (!IsValidCell(intSourceX, intSourceY)) {
        return INFINITECOST;
    }
    if (!IsValidCell(intendX, intendY)) {
        return INFINITECOST;
    }
    if (EnvNAVXYTHETACCfg.Grid2D[intendX][intendY] >=
        EnvNAVXYTHETACCfg.cost_inscribed_thresh) {
        return INFINITECOST;
    }

    // need to iterate over discretized center cells and compute cost based on
    // them
    unsigned char maxcellcost = 0;
    for (i = 0; i < (int)action->interm3DcellsV.size(); ++i) {
        interm3Dcell = action->interm3DcellsV.at(i);
        interm3Dcell.x =
            CONTXY2DISC((interm3Dcell.x * cellsize) + SourceX, cellsize);

        interm3Dcell.y =
            CONTXY2DISC((interm3Dcell.y * cellsize) + SourceY, cellsize);

        // out of map
        if (interm3Dcell.x < 0 ||
            interm3Dcell.x >= EnvNAVXYTHETACCfg.EnvWidth_c ||
            interm3Dcell.y < 0 ||
            interm3Dcell.y >= EnvNAVXYTHETACCfg.EnvHeight_c) {
            return INFINITECOST;
        }

        maxcellcost =
            __max(maxcellcost,
                  EnvNAVXYTHETACCfg.Grid2D[interm3Dcell.x][interm3Dcell.y]);

        // check that the robot is NOT in the cell at which there is no valid
        // orientation
        if (EnvNAVXYTHETACCfg.FootprintPolygon.size() > 1 &&
            (int)maxcellcost >=
                EnvNAVXYTHETACCfg.cost_possibly_circumscribed_thresh) {
            checks++;

            for (i = 0; i < (int)action->intersectingcellsV.size(); ++i) {
                // get the cell in the map
                cell = action->intersectingcellsV.at(i);
                cell.x = CONTXY2DISC(cell.x * cellsize + SourceX, cellsize);
                cell.y = CONTXY2DISC(cell.y * cellsize + SourceY, cellsize);

                // check validity
                if (!IsValidCell(cell.x, cell.y)) {
                    return INFINITECOST;
                }
            }
        }

        // to ensure consistency of h2D:
        maxcellcost = __max(maxcellcost,
                            EnvNAVXYTHETACCfg.Grid2D[intSourceX][intSourceY]);
        int currentmaxcost =
            (int)__max(maxcellcost, EnvNAVXYTHETACCfg.Grid2D[intendX][intendY]);

        // use cell cost as multiplicative factor
        return action->cost * (currentmaxcost + 1);

    }  // for
    return 0;
}  // GetActionCost

void EnvironmentNAVXYTHETAC::GetSuccs(
    int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV,
    std::vector<EnvNAVXYTHETACAction_t*>* actionV) {
    int aind;

#if TIME_DEBUG
    clock_t currenttime = clock();
#endif

    SuccIDV->clear();
    CostV->clear();
    SuccIDV->reserve(EnvNAVXYTHETACCfg.actionwidth);
    CostV->reserve(EnvNAVXYTHETACCfg.actionwidth);
    if (actionV != NULL) {
        actionV->clear();
        actionV->reserve(EnvNAVXYTHETACCfg.actionwidth);
    }

    if (SourceStateID == EnvNAVXYTHETAC.goalstateid) return;

    // predcessor state
    EnvNAVXYTHETACHashEntry_t* HashEntry = StateID2CoordTable[SourceStateID];

    // iterate through actions
    for (aind = 0; aind < EnvNAVXYTHETACCfg.actionwidth; aind++) {
        EnvNAVXYTHETACAction_t* nav3daction =
            &EnvNAVXYTHETACCfg.ActionsV[ContTheta2DiscNew(HashEntry->Theta)]
                                       [aind];

        double newX =
            HashEntry->X + (nav3daction->dX * EnvNAVXYTHETACCfg.cellsize_m);
        double newY =
            HashEntry->Y + (nav3daction->dY * EnvNAVXYTHETACCfg.cellsize_m);
        double newTheta = DiscTheta2ContNew(nav3daction->endtheta);

        // skip the invalid cells
        if (!IsValidCell(CONTXY2DISC(newX, EnvNAVXYTHETACCfg.cellsize_m),
                         CONTXY2DISC(newY, EnvNAVXYTHETACCfg.cellsize_m))) {
            continue;
        }

        // get cost
        int cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta,
                                 nav3daction);
        if (cost >= INFINITECOST) {
            continue;
        }

        EnvNAVXYTHETACHashEntry_t* OutHashEntry;

        if (IsWithinGoalRegion(newX, newY, newTheta)) {
            OutHashEntry = StateID2CoordTable[EnvNAVXYTHETAC.goalstateid];
        } else {
            if ((OutHashEntry = (this->*GetHashEntry)(newX, newY, newTheta)) ==
                NULL) {
                OutHashEntry =
                    (this->*CreateNewHashEntry)(newX, newY, newTheta);

                // this operation will be done in planner
                OutHashEntry->parentStateID = SourceStateID;
            }
        }

        SuccIDV->push_back(OutHashEntry->stateID);
        CostV->push_back(cost);
        if (actionV != NULL) {
            actionV->push_back(nav3daction);
        }

    }  // for

    HashEntry->valid_children_ratio =
        (double)(SuccIDV->size()) / EnvNAVXYTHETACCfg.actionwidth;
#if TIME_DEBUG
    time_getsuccs += clock() - currenttime;
#endif

}  // GetSuccs

void EnvironmentNAVXYTHETAC::GetSuccs(int SourceStateID,
                                      std::vector<int>* SuccIDV,
                                      std::vector<int>* CostV) {
    GetSuccs(SourceStateID, SuccIDV, CostV, NULL);
}

void EnvironmentNAVXYTHETAC::GetPreds(int TargetStateID,
                                      std::vector<int>* PredIDV,
                                      std::vector<int>* CostV) {
    // TODO - to support tolerance, need
    // a) generate preds for goal state based on all possible goal state
    // variable setting,
    // b) change goal check condition in get hashentry
    // c) change getpredsofchangedcells and getsuccsofchangedcessl functions

}  // GetPreds

void EnvironmentNAVXYTHETAC::PrintState(int stateID, bool bVerbose,
                                        FILE* fOut) {
#if DEBUG
    if (stateID >= (int)StateID2CoordTable.size()) {
        SBPL_ERROR(
            "ERROR in EnvNAVXYTHETAC... function: stateID illegal (2)\n");
        throw SBPL_Exception();
    }
#endif

    if (fOut == NULL) {
        fOut = stdout;
    }

    EnvNAVXYTHETACHashEntry_t* HashEntry = StateID2CoordTable[stateID];

    if (stateID == EnvNAVXYTHETAC.goalstateid && bVerbose) {
        SBPL_FPRINTF(fOut, "the state is a goal state\n");
    }

    if (bVerbose) {
        SBPL_FPRINTF(fOut, "X=%f Y=%f Theta=%f\n", HashEntry->X, HashEntry->Y,
                     HashEntry->Theta);
        output << HashEntry->X << " " << HashEntry->Y << " " << HashEntry->Theta
               << std::endl;
    } else {
        SBPL_FPRINTF(fOut, "%.3f %.3f %.3f\n",
                     DISCXY2CONT(HashEntry->X, EnvNAVXYTHETACCfg.cellsize_m),
                     DISCXY2CONT(HashEntry->Y, EnvNAVXYTHETACCfg.cellsize_m),
                     DiscTheta2ContNew(HashEntry->Theta));
    }

}  // PrintState

void EnvironmentNAVXYTHETAC::EnsureHeuristicsUpdated(bool bGoalHeuristics) {
    if (bNeedtoRecomputeStartHeuristics && !bGoalHeuristics) {
        grid2Dsearchfromstart->search(
            EnvNAVXYTHETACCfg.Grid2D, EnvNAVXYTHETACCfg.cost_inscribed_thresh,
            CONTXY2DISC(EnvNAVXYTHETACCfg.StartX_c,
                        EnvNAVXYTHETACCfg.cellsize_m),
            CONTXY2DISC(EnvNAVXYTHETACCfg.StartY_c,
                        EnvNAVXYTHETACCfg.cellsize_m),
            CONTXY2DISC(EnvNAVXYTHETACCfg.EndX_c, EnvNAVXYTHETACCfg.cellsize_m),
            CONTXY2DISC(EnvNAVXYTHETACCfg.EndY_c, EnvNAVXYTHETACCfg.cellsize_m),
            SBPL_2DGRIDSEARCH_TERM_CONDITION_TWOTIMESOPTPATH);
        bNeedtoRecomputeStartHeuristics = false;
        SBPL_PRINTF(
            "2dsolcost_infullunits=%d\n",
            (int)(grid2Dsearchfromstart->getlowerboundoncostfromstart_inmm(
                      EnvNAVXYTHETACCfg.StartX_c, EnvNAVXYTHETACCfg.StartY_c) /
                  EnvNAVXYTHETACCfg.nominalvel_mpersecs));
    }

    // forwardsearch
    if (bNeedtoRecomputeGoalHeuristics && bGoalHeuristics) {
        grid2Dsearchfromgoal->search(
            EnvNAVXYTHETACCfg.Grid2D, EnvNAVXYTHETACCfg.cost_inscribed_thresh,
            CONTXY2DISC(EnvNAVXYTHETACCfg.EndX_c, EnvNAVXYTHETACCfg.cellsize_m),
            CONTXY2DISC(EnvNAVXYTHETACCfg.EndY_c, EnvNAVXYTHETACCfg.cellsize_m),
            CONTXY2DISC(EnvNAVXYTHETACCfg.StartX_c,
                        EnvNAVXYTHETACCfg.cellsize_m),
            CONTXY2DISC(EnvNAVXYTHETACCfg.StartY_c,
                        EnvNAVXYTHETACCfg.cellsize_m),
            SBPL_2DGRIDSEARCH_TERM_CONDITION_TWOTIMESOPTPATH);
        bNeedtoRecomputeGoalHeuristics = false;
        SBPL_PRINTF(
            "2dsolcost_infullunits=%d\n",
            (int)(grid2Dsearchfromgoal->getlowerboundoncostfromstart_inmm(
                      EnvNAVXYTHETACCfg.StartX_c, EnvNAVXYTHETACCfg.StartY_c) /
                  EnvNAVXYTHETACCfg.nominalvel_mpersecs));
    }
}

void EnvironmentNAVXYTHETAC::ConvertStateIDPathintoXYThetaPath(
	std::vector<int>* stateIDPath,
	std::vector<sbpl_xy_theta_pt_t>* xythetaPath){

    int totalcost = 0;
    xythetaPath->clear();
    stateIDPath->clear();
    auto goalentry = StateID2CoordTable[EnvNAVXYTHETAC.goalstateid];
    if (-10 == goalentry->parentStateID ) {
        std::cout << "goal is not reached.\n";
        return;
    }
    
    EnvNAVXYTHETACHashEntry_t* tempEntry = StateID2CoordTable[goalentry->parentStateID];

    // then goal is expanded and do backtrack
    // stateID path get
    while (tempEntry->stateID != EnvNAVXYTHETAC.startstateid) {
        stateIDPath->push_back(tempEntry->stateID); 
        tempEntry = StateID2CoordTable[tempEntry->parentStateID];
    }

    // do short cutting while backtracking
    double angular_v = (PI_CONST / 4.0) / EnvNAVXYTHETACCfg.timetoturn45degsinplace_secs;

    for (int i = stateIDPath->size() - 1; i > 0; ) {

        assert(i>=1);
        int sourceID = stateIDPath->at(i);
        auto sourceEntry = StateID2CoordTable[sourceID];
        int targetID = stateIDPath->at(i-1);
        bool found = false;

        /*
         *do short cutting now
         *from the other state, moving forward, save short cutting after each iteration
         *if obstacle occurred, stop, push_back the current solution.
         */

        std::vector<sbpl_xy_theta_pt_t> temp_path;
        // the basic solution: move from source to the neighbor state
        double temp_cost = -0.01;
        GoTo(sourceID, targetID, temp_path, temp_cost);
        if(temp_cost > 0.0) found = true;
        
        //if (i >=2){
            //assert(temp_path.size() > 0);
        //}

        // using the dubins algorithm
        DubinsPath currentPath;
        double start[3];
        double end[3];
        double rho = 0.3284 * EnvNAVXYTHETACCfg.min_stepsize;
        start[0] = sourceEntry->X;
        start[1] = sourceEntry->Y;
        start[2] = sourceEntry->Theta;

        int counter = 2;
        for (; ; ++counter) {
            if((int)i-counter <= 0 ){
                break;
            }
            targetID = stateIDPath->at((int)i-counter);
            auto targetEntry = StateID2CoordTable[targetID];
            end[0] = targetEntry->X;
            end[1] = targetEntry->Y;
            end[2] = targetEntry->Theta;

            int dubin_flag = dubins_init( start, end, rho, &currentPath);

            if (0 == dubin_flag) {  // no error
                // get the path
                std::vector<sbpl_xy_theta_pt_t> sub_temp_path;
                std::vector<double> pts;
                dubins_path_sample_many(&currentPath, 0.001,pts, false);
                for(int count = 0; count < pts.size(); count+=3){
                    sbpl_xy_theta_pt_t ppt(pts.at(count), pts.at(count+1), pts.at(count+2));
                    sub_temp_path.push_back(ppt);
                }

                // collision checking on the subpath
                bool collision = false;
                for(auto pt : sub_temp_path) {
                    if (!IsValidCell( CONTXY2DISC(pt.x, EnvNAVXYTHETACCfg.cellsize_m),
                                CONTXY2DISC(pt.y, EnvNAVXYTHETACCfg.cellsize_m))) {
                        collision = true;
                        break;
                    }
                }
                if(!collision) {     // not in collision, save the path and continue the short cutting loop;
                    temp_path.clear();
                    temp_path = std::move(sub_temp_path);
                    temp_cost = dubins_path_length(&currentPath);
                } else {            // in collision, quit the loop 
                    break;
                }
            } else {
                break;
            } 
        }
        //save the short cutting path
        xythetaPath->insert(xythetaPath->end(), temp_path.begin(), temp_path.end());
        totalcost+= (int)(temp_cost/EnvNAVXYTHETACCfg.nominalvel_mpersecs * NAVXYTHETAC_COSTMULT_MTOMM);

        // compensate --i in loop;
        // compensate the current state which cannot be reached by short cutting
        i = i-counter+1;

        // to reach the goal

        if (!found && targetID != EnvNAVXYTHETAC.goalstateid) {
            double targetx, targety, targettheta;
            double sourcex, sourcey, sourcetheta; 

            GetCoordFromState(sourceID, sourcex, sourcey, sourcetheta);
            GetCoordFromState(targetID, targetx, targety, targettheta);
            std::cout<<sourceID << "  "<< targetID << std::endl;
            std::cout<<sourcex<<" "<< sourcey<<" "<< sourcetheta << " "
                << targetx << " "<< targety<<" "<< targettheta << std::endl;
            auto temp = StateID2CoordTable[targetID];
            std::cout << "parent of target: " << temp->parentStateID << std::endl;


            throw SBPL_Exception("ERROR: successor not found for transition");
        }
        
    }
    std::cout << "post_process solution cost: " << totalcost << std::endl;
    //std::cout << "Number of expansions: " << sample_times << std::endl;

}       // ConvertStateIDPathintoXYThetaPath



/*
 *void EnvironmentNAVXYTHETAC::ConvertStateIDPathintoXYThetaPath(
 *    std::vector<int>* stateIDPath,
 *    std::vector<sbpl_xy_theta_pt_t>* xythetaPath) {
 *    std::vector<EnvNAVXYTHETACAction_t*> actionV;
 *    std::vector<int> CostV;
 *    std::vector<int> SuccIDV;
 *    double targetx_c, targety_c, targettheta_c;
 *    double sourcex_c, sourcey_c, sourcetheta_c;
 *
 *    SBPL_PRINTF("checks=%ld\n", checks);
 *
 *    xythetaPath->clear();
 *#if DEBUG
 *    SBPL_FPRINTF(fDeb, "converting stateid path into coordinates:\n");
 *#endif
 *
 *    for (int pind = 0; pind < (int)(stateIDPath->size()) - 1; pind++) {
 *        int sourceID = stateIDPath->at(pind);
 *        int targetID = stateIDPath->at(pind + 1);
 *#if DEBUG
 *        GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c);
 *#endif
 *
 *        // get succssors and pick the target via the cheapest action
 *        SuccIDV.clear();
 *        CostV.clear();
 *        actionV.clear();
 *        GetSuccs(sourceID, &SuccIDV, &CostV, &actionV);
 *
 *        int bestcost = INFINITECOST;
 *        int bestsind = -1;
 *
 *#if DEBUG
 *        GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c);
 *        GetCoordFromState(targetID, targetx_c, targety_c, targettheta_c);
 *        SBPL_FPRINTF(fDeb, "looking for %f %f %f -> %f %f %f (numofsuccs=%d)\n",
 *                     sourcex_c, sourcey_c, sourcetheta_c, targetx_c, targety_c,
 *                     targettheta_c, (int)SuccIDV.size());
 *#endif
 *        for (int sind = 0; sind < (int)SuccIDV.size(); ++sind) {
 *#if DEBUG
 *            double x_c, y_c, theta_c;
 *            GetCoordFromState(SuccIDV[sind], x_c, y_c, theta_c);
 *            SBPL_FPRINTF(fDeb, "succ: %.3f %.3f %.3f\n", x_c, y_c, theta_c);
 *#endif
 *            if (SuccIDV[sind] == targetID && CostV[sind] <= bestcost) {
 *                bestcost = CostV[sind];
 *                bestsind = sind;
 *            }
 *        }
 *
 *        if (bestsind == -1) {
 *            SBPL_ERROR("ERROR: successor not found for transition");
 *            GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c);
 *            GetCoordFromState(targetID, targetx_c, targety_c, targettheta_c);
 *            printf("%d %d\n", sourceID, targetID);
 *            SBPL_PRINTF("%f %f %f -> %f %f %f\n", sourcex_c, sourcey_c,
 *                        sourcetheta_c, targetx_c, targety_c, targettheta_c);
 *            throw SBPL_Exception("ERROR: successor not found for transition");
 *        }
 *
 *        // now push in the actual path
 *        GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c);
 *
 *        // TODO - when there are no motion primitives we should still print
 *        // source state
 *        for (int ipind = 0;
 *             ipind < ((int)actionV[bestsind]->intermptV.size()) - 1; ++ipind) {
 *            // translate appropriately
 *            sbpl_xy_theta_pt_t intermpt = actionV[bestsind]->intermptV[ipind];
 *            intermpt.x += sourcex_c;
 *            intermpt.y += sourcey_c;
 *
 *#if DEBUG
 *            int nx = CONTXY2DISC(intermpt.x, EnvNAVXYTHETACCfg.cellsize_m);
 *            int ny = CONTXY2DISC(intermpt.y, EnvNAVXYTHETACCfg.cellsize_m);
 *            int ntheta;
 *            ntheta = ContTheta2DiscNew(intermpt.theta);
 *
 *            SBPL_FPRINTF(fDeb, "%.3f %.3f %.3f (%d %d %d cost=%d)", intermpt.x,
 *                         intermpt.y, intermpt.theta, nx, ny, ntheta,
 *                         EnvNAVXYTHETACCfg.Grid2D[nx][ny]);
 *
 *            if (ipind == 0) {
 *                SBPL_FPRINTF(fDeb, "first (heur=%d)\n",
 *                             GetStartHeuristic(sourceID));
 *            } else {
 *                SBPL_FPRINTF(fDeb, "\n");
 *            }
 *#endif
 *            // store
 *            xythetaPath->push_back(intermpt);
 *        }
 *    }
 *}  // ConvertStateIDPathintoXYThetaPath
 */

int EnvironmentNAVXYTHETAC::SizeofCreatedEnv() {
    return (int)StateID2CoordTable.size();
}

void EnvironmentNAVXYTHETAC::PrintEnv_Config(FILE* fOut) {
    // implement this if the planner needs to print out
    // EnvNAVXYTHETAC.configuration
    throw SBPL_Exception(
        "ERROR in EnvNAVXYTHETAC... function: PrintEnv_Config is undefined");
}

void EnvironmentNAVXYTHETAC::PrintTimeStat(FILE* fOut) {
#if TIME_DEBUG
    SBPL_FPRINTF(fOut,
                 "time3_addallout = %f secs, time_gethash = %f secs, "
                 "time_createhash = %f secs, "
                 "time_getsuccs = %f\n",
                 time3_addallout / (double)CLOCKS_PER_SEC,
                 time_gethash / (double)CLOCKS_PER_SEC,
                 time_createhash / (double)CLOCKS_PER_SEC,
                 time_getsuccs / (double)CLOCKS_PER_SEC);
#endif
}

void EnvironmentNAVXYTHETAC::PrintHashTableHist(FILE* fOut) {
    int s0 = 0, s1 = 0, s50 = 0, s100 = 0, s200 = 0, s300 = 0, slarge = 0;

    for (int j = 0; j < HashTableSize; ++j) {
        if ((int)Coord2StateIDHashTable[j].size() == 0)
            s0++;
        else if ((int)Coord2StateIDHashTable[j].size() < 5)
            s1++;
        else if ((int)Coord2StateIDHashTable[j].size() < 25)
            s50++;
        else if ((int)Coord2StateIDHashTable[j].size() < 50)
            s100++;
        else if ((int)Coord2StateIDHashTable[j].size() < 100)
            s200++;
        else if ((int)Coord2StateIDHashTable[j].size() < 400)
            s300++;
        else
            slarge++;
    }
    SBPL_FPRINTF(fOut,
                 "hash tabel histogram: 0:%d, <5:%d, <25:%d, <50:%d, <100:%d, "
                 "<400:%d, >400:%d\n",
                 s0, s1, s50, s100, s200, s300, slarge);
}

double EnvironmentNAVXYTHETAC::DiscTheta2ContNew(int theta) const {
    if (bUseNonUniformAngles) {
        return DiscTheta2ContFromSet(theta);
    } else {
        return DiscTheta2Cont(theta, EnvNAVXYTHETACCfg.NumThetaDirs);
    }
}

double EnvironmentNAVXYTHETAC::DiscTheta2ContFromSet(int theta) const {
    theta = normalizeDiscAngle(theta);

    // ThetaDirs should contain extara angle (2PI) for overlap
    if (EnvNAVXYTHETACCfg.NumThetaDirs >=
        (int)EnvNAVXYTHETACCfg.ThetaDirs.size()) {
        throw SBPL_Exception(
            "ERROR: list of bin angles are not properly set to use function "
            "DiscTheta2ConfFromSet");
    }

    if (theta > EnvNAVXYTHETACCfg.NumThetaDirs || theta < 0) {
        std::stringstream ss;
        ss << "ERROR: discrete value theta " << theta << " out of range ";
        throw SBPL_Exception(ss.str());
    }
    return EnvNAVXYTHETACCfg.ThetaDirs[theta];
}

void EnvironmentNAVXYTHETAC::SetGoalTolerance(double tol_x, double tol_y,
                                              double tol_theta) {
    EnvNAVXYTHETACCfg.tol_x = tol_x;
    EnvNAVXYTHETACCfg.tol_y = tol_y;
    EnvNAVXYTHETACCfg.tol_theta = tol_theta;
}

// meter meter rad
bool EnvironmentNAVXYTHETAC::IsWithinGoalRegion(double x, double y,
                                                double theta) {
    if (sqrt((x - EnvNAVXYTHETACCfg.EndX_c) * (x - EnvNAVXYTHETACCfg.EndX_c) +
             (y - EnvNAVXYTHETACCfg.EndY_c) * (y - EnvNAVXYTHETACCfg.EndY_c)) <
            EnvNAVXYTHETACCfg.tol_x &&
        fabs(theta - EnvNAVXYTHETACCfg.EndTheta) <
            EnvNAVXYTHETACCfg.tol_theta) {
        return true;
    }

    return false;
}

void EnvironmentNAVXYTHETAC::UpdateParent(int childStateID, int parentStateID) {
    if (childStateID < StateID2CoordTable.size() &&
        parentStateID < StateID2CoordTable.size()) {
        auto child = StateID2CoordTable[childStateID];
        child->parentStateID = parentStateID;
    }
}

int EnvironmentNAVXYTHETAC::GoTo(int parentID, int sampleStateID) {
    std::vector<sbpl_xy_theta_pt_t> subpath;
    double length;

    return GoTo(parentID, sampleStateID, subpath, length);
}

// build edges from current state to smapleState
int EnvironmentNAVXYTHETAC::GoTo(int parentID, int sampleStateID, std::vector<sbpl_xy_theta_pt_t> &subpath, double& length){

    subpath.clear();
    auto parentEntry = StateID2CoordTable.at(parentID);
    auto sampleEntry = StateID2CoordTable.at(sampleStateID);
   
    // use dubins path to get the extension
    double angular_v = (PI_CONST / 4.0) / EnvNAVXYTHETACCfg.timetoturn45degsinplace_secs;
    double rho = 0.3284 * EnvNAVXYTHETACCfg.min_stepsize;
    DubinsPath currentPath;

    double parent[3], sample[3];
    parent[0] = parentEntry->X; parent[1] = parentEntry->Y; parent[2] = parentEntry->Theta;
    sample[0] = sampleEntry->X; sample[1] = sampleEntry->Y; sample[2] = sampleEntry->Theta;
    int flag = dubins_init(parent, sample, rho, &currentPath);

    std::vector<double> pts;
    bool collision = false;
    if(!flag) {
        // get the path
        length = dubins_path_sample_many(&currentPath, 0.001, pts, true);

        for(int count = 0; count < pts.size(); count+=3){
            sbpl_xy_theta_pt_t ppt(pts.at(count), pts.at(count+1), pts.at(count+2));

            // do collision checking
            if (!IsValidCell( CONTXY2DISC(ppt.x, EnvNAVXYTHETACCfg.cellsize_m),
                        CONTXY2DISC(ppt.y, EnvNAVXYTHETACCfg.cellsize_m))) {
                collision = true;
                subpath.clear();
                break;
            }
            subpath.push_back(ppt);
        }
    }


    EnvNAVXYTHETACHashEntry_t* HashEntry; 
    if ( !collision ) {

        auto endpt = subpath.back();
        if (IsWithinGoalRegion(endpt.x, endpt.y, endpt.theta)) {

			HashEntry = StateID2CoordTable[EnvNAVXYTHETAC.goalstateid];
            HashEntry->parentStateID = parentID;

        } else {

            if (nullptr == (HashEntry =(this->*GetHashEntry)(endpt.x, endpt.y, endpt.theta))) {
                HashEntry = (this->*CreateNewHashEntry)(endpt.x, endpt.y, endpt.theta);
            }
            HashEntry->parentStateID = parentID;

            //treeNodes.nodes.push_back(HashEntry);
            //int size = treeNodes.kdtree_get_point_count();
            //kdtree->addPoints((size_t)size-1, (size_t)size-1);

            output << HashEntry->X << " "
                << HashEntry->Y << " "
                << HashEntry->Theta << std::endl;
        }
        return HashEntry->stateID;
    }
    return -1;
}



void EnvironmentNAVXYTHETAC::SetupR(double r) { EnvNAVXYTHETACCfg.R_times = r; }
