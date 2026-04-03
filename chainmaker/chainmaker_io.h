#pragma once

#include "chainmaker.h"
#include <string>

bool SaveWorldToJSON(const ChainWorld& world, const char* filename);
bool LoadWorldFromJSON(ChainWorld& world, const char* filename);
