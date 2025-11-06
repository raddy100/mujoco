#include "progui_globals.h"
#include <fstream>
#include <sstream>

// Definitions of globals
mjModel* m = nullptr;
mjData* d = nullptr;
mjSpec* spec = nullptr;
mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;

mjuiState uistate;
mjUI ui;

bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx =0.0;
double lasty =0.0;

bool g_boxesCollide = false;
bool g_physicsEnabled = false;
mjtNum g_savedGravity[3] = {0,0,0};

mjsBody* g_lastBody = nullptr;
std::vector<ChainEntry> g_chain;
std::unordered_map<int, size_t> g_bodyIdToIndex;

mjsGeom* g_lastMarker = nullptr;
mjsGeom* g_probeRect = nullptr;
std::string g_probeName;

// gap control
double g_gapRatio =0.05; // default5% of box width
char g_gapRatioLabel[64] = "gap:0.05";

// IO filename buffer
char g_ioFilename[256] = "chain_save.txt";
int g_fileIoMode =0;

// pre-physics state
bool g_hasSavedPrePhysicsState = false;
std::vector<SavedBodyPos> g_savedPrePhysicsChain;

// Softer defaults: increase constraint time constant and reduce joint damping
const double kJointDamping =0.15; // was1.0
const double kGeomMargin =0.001;
const double kSolref[2] = {0.15,0.7}; // was {0.03,1.0}
const double kSolimp[5] = {0.9,0.95,0.001,0.5,2.0};
const double kBoxHalf =0.05;

int g_uiWidth =220;

int g_spawnFace = FACE_POSX;

// Global button label definitions
const char* kBtnSpawnCube = "Spawn Cube (C)";
const char* kBtnStartPhysics= "Start Physics (P)";
const char* kBtnResetChain = "Reset Chain (R)";
const char* kBtnIncreaseGap = "Increase Gap (+)";
const char* kBtnDecreaseGap = "Decrease Gap (-)";
const char* kBtnSaveChain = "Save Chain (Ctrl+S)";
const char* kBtnLoadChain = "Load Chain (Ctrl+L)";
const char* kBtnSaveToFile = "Save To File";
const char* kBtnLoadFromFile= "Load From File";
const char* kBtnPrintDirections = "Print Directions";

// Direction tracking
std::vector<std::string> g_directionHistory;

// Helpers to map faces to axis/sign
static inline void FaceToAxisSignLocal(int face, int &axis, int &sign)
{
 switch (face)
 {
 case FACE_POSX: axis =0; sign = +1; break;
 case FACE_NEGX: axis =0; sign = -1; break;
 case FACE_POSY: axis =1; sign = +1; break;
 case FACE_NEGY: axis =1; sign = -1; break;
 case FACE_POSZ: axis =2; sign = +1; break;
 case FACE_NEGZ: axis =2; sign = -1; break;
 default: axis =0; sign = +1; break;
 }
}

// Expose helper implementations declared in header
void RecordTurnInput(int oldFace, int newFace)
{
 if (oldFace == newFace) return; // no change

 int oldAxis, oldSign; FaceToAxisSignLocal(oldFace, oldAxis, oldSign);
 int newAxis, newSign; FaceToAxisSignLocal(newFace, newAxis, newSign);

 // If turning into Z, it's up/down based on the new sign
 if (newAxis ==2)
 {
 g_directionHistory.push_back(newSign >0 ? "up" : "down");
 return;
 }

 // If both in XY, classify left/right
 if ((oldAxis !=2) && (newAxis !=2))
 {
 std::string dir;
 if (oldAxis ==0 && newAxis ==1)
 {
 // X -> Y
 int prod = oldSign * newSign;
 dir = (prod >0) ? "left" : "right";
 }
 else if (oldAxis ==1 && newAxis ==0)
 {
 // Y -> X
 int prod = oldSign * newSign;
 dir = (prod >0) ? "right" : "left";
 }
 else
 {
 // same axis (including180 turn) or unsupported, ignore
 return;
 }
 g_directionHistory.push_back(dir);
 return;
 }

 // Leaving Z to XY: ambiguous left/right — record as Turnfront / TurnBack
 if (oldAxis ==2 && (newAxis ==0 || newAxis ==1))
 {
 // Use the sign of the new face to disambiguate: positive => front, negative => back
 g_directionHistory.push_back(newSign >0 ? "Turnfront" : "TurnBack");
 return;
 }

 // Leaving XY to Z handled above; other cases ignored
}

bool g_suppressDirRecord = false;

void RecordForwardInput()
{
 if (g_suppressDirRecord) return;
 g_directionHistory.push_back("forward");
}

void ClearDirectionHistory()
{
 g_directionHistory.clear();
}
