#pragma once

#include "chainmaker.h"

// Populate the mjvScene with all visual geoms for the build stage.
// Called every frame (resets scn.ngeom = 0 first).
void PopulateBuildScene(AppState& app);

// Individual geom helpers
void AddFloorGeom(mjvScene& scn, const ChainWorld& world);
void AddBlockGeom(mjvScene& scn, const ChainWorld& world,
                  const IVec3& pos, const GridCell& cell);
void AddGhostGeom(mjvScene& scn, const ChainWorld& world, const Chain& chain);
void AddHeadMarker(mjvScene& scn, const ChainWorld& world, const IVec3& head);
void AddDirectionArrow(mjvScene& scn, const ChainWorld& world,
                       const IVec3& head, SpawnFace face);
void AddJunctionHighlight(mjvScene& scn, const ChainWorld& world,
                          const IVec3& pos);

// Compute the blended RGBA color for a grid cell
void ComputeBlockColor(const ChainWorld& world, const GridCell& cell, float rgba[4]);

// Build a 3×3 rotation matrix (row-major, 9 doubles) that orients an arrow
// (default pointing along +Z) to point along the given face direction.
void BuildArrowRotation(SpawnFace face, mjtNum mat[9]);

// Render status-bar and file-IO overlays
void RenderOverlays(AppState& app, mjrRect viewport);
