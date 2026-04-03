#pragma once

#include "chainmaker.h"
#include <string>

// Compile result from ChainWorld → mjSpec → mjModel
struct CompileResult {
    mjSpec*  spec  = nullptr;
    mjModel* model = nullptr;
    mjData*  data  = nullptr;
    std::string error;

    // body_id → (chain_id, position_in_chain)
    std::vector<std::pair<int,int>> body_chain_info;
};

// Compile the entire ChainWorld to a MuJoCo physics model.
CompileResult CompileWorld(const ChainWorld& world);

// Compute ball joint cone angle limit in degrees (from progui reference).
double ComputeConeLimit(double half_size, double gap);

// Contact filter callback (installed before simulation starts).
int ChainmakerContactFilter(const mjModel* m, mjData* d, int geom1, int geom2);

// Global body→chain info table (populated by CompileWorld, read by contact filter).
extern std::vector<std::pair<int,int>> g_body_chain_info;
