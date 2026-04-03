#include "chainmaker_io.h"

#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <cstring>

using json = nlohmann::json;

bool SaveWorldToJSON(const ChainWorld& world, const char* filename) {
    if (!filename || filename[0] == '\0') {
        std::cerr << "SaveWorldToJSON: empty filename\n";
        return false;
    }

    json j;
    j["version"]   = 1;
    j["bead_size"] = world.bead_size;
    j["gap_ratio"] = world.gap_ratio;

    json chains_arr = json::array();
    for (const auto& chain : world.chains) {
        json cj;
        cj["id"]             = chain.id;
        cj["name"]           = chain.name;
        cj["color"]          = {chain.color[0], chain.color[1],
                                 chain.color[2], chain.color[3]};
        cj["head_direction"] = static_cast<int>(chain.head_direction);

        json blocks_arr = json::array();
        for (const auto& pos : chain.blocks) {
            json bj;
            bj["pos"] = {pos.x, pos.y, pos.z};

            auto it = world.grid.find(pos);
            if (it != world.grid.end()) {
                bj["is_turn"]        = it->second.is_turn;
                bj["turn_entry_axis"] = it->second.turn_entry_axis;
                bj["turn_exit_axis"]  = it->second.turn_exit_axis;
            } else {
                bj["is_turn"]        = false;
                bj["turn_entry_axis"] = -1;
                bj["turn_exit_axis"]  = -1;
            }
            blocks_arr.push_back(bj);
        }
        cj["blocks"] = blocks_arr;
        chains_arr.push_back(cj);
    }
    j["chains"] = chains_arr;

    std::ofstream ofs(filename);
    if (!ofs) {
        std::cerr << "SaveWorldToJSON: cannot open '" << filename << "' for writing\n";
        return false;
    }
    ofs << j.dump(2);
    std::cout << "Saved " << world.grid.size() << " cells to " << filename << "\n";
    return true;
}

bool LoadWorldFromJSON(ChainWorld& world, const char* filename) {
    if (!filename || filename[0] == '\0') {
        std::cerr << "LoadWorldFromJSON: empty filename\n";
        return false;
    }

    std::ifstream ifs(filename);
    if (!ifs) {
        std::cerr << "LoadWorldFromJSON: cannot open '" << filename << "'\n";
        return false;
    }

    json j;
    try {
        ifs >> j;
    } catch (const json::parse_error& e) {
        std::cerr << "LoadWorldFromJSON: parse error: " << e.what() << "\n";
        return false;
    }

    world.grid.clear();
    world.chains.clear();
    world.active_chain_id = -1;

    world.bead_size = j.value("bead_size", 0.05);
    world.gap_ratio = j.value("gap_ratio", 0.05);

    for (const auto& cj : j["chains"]) {
        Chain chain;
        chain.id   = cj["id"].get<int>();
        chain.name = cj["name"].get<std::string>();
        auto col   = cj["color"];
        for (int i = 0; i < 4; i++) chain.color[i] = col[i].get<float>();
        chain.head_direction = static_cast<SpawnFace>(cj["head_direction"].get<int>());

        for (const auto& bj : cj["blocks"]) {
            auto p = bj["pos"];
            IVec3 pos{p[0].get<int>(), p[1].get<int>(), p[2].get<int>()};
            chain.blocks.push_back(pos);

            GridCell& cell = world.grid[pos];
            cell.pos = pos;

            // Determine which axis this chain passes through
            int block_idx = (int)chain.blocks.size() - 1;
            int axis = -1;
            if (block_idx > 0) {
                IVec3 prev = chain.blocks[block_idx - 1];
                IVec3 diff = pos - prev;
                for (int ax = 0; ax < 3; ax++) {
                    if ((&diff.x)[ax] != 0) { axis = ax; break; }
                }
            } else {
                int sign;
                FaceToAxisSign(chain.head_direction, axis, sign);
            }
            if (axis >= 0) cell.chain_on_axis[axis] = chain.id;

            if (bj.value("is_turn", false)) {
                cell.is_turn         = true;
                cell.turn_entry_axis = bj["turn_entry_axis"].get<int>();
                cell.turn_exit_axis  = bj["turn_exit_axis"].get<int>();
            }
        }
        world.chains.push_back(chain);
    }

    if (!world.chains.empty()) world.active_chain_id = 0;

    std::cout << "Loaded " << world.grid.size() << " cells from " << filename << "\n";
    return true;
}
