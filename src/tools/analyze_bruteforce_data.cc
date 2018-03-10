
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <map>
#include <posers/survive_reproject.h>
#include <posers/sba_solver.h>
#include <cmath>

int main(int argc, char** argv) {
    std::ifstream fin(argv[1]);  //ifstream to read from
    std::string line;
    std::getline(fin, line); // header row - throw away

    std::map<size_t, double> error[2];
    std::map<size_t, size_t> cnt[2];
    std::map<size_t, double> error_map;

    double best = INFINITY, worst = 0;
    while (std::getline(fin, line))
    {
        std::istringstream iss(line);
        size_t idx;
        double err;
        std::string comma;
        iss >> idx >> comma >> err;
        error_map[idx] = err;
        for(size_t i = 0;i < sizeof(survive_calibration_config) + 1;i++) {
            error[(bool)(idx & (1 << i))][i] += err;
            cnt[(bool)(idx & (1 << i))][i]++;
        }

        best = std::min(best, err);
        worst = std::max(worst, err);
    }

    size_t recommended = 0;
    for(size_t i = 0;i < sizeof(survive_calibration_config) + 1;i++) {
        std::cerr << "For bit " << i<< ":" << std::endl;
        auto onAvg = error[1][i] / cnt[1][i];
        auto offAvg = error[0][i] / cnt[0][i];
        std::cerr << "On:\t" << onAvg << std::endl;
        std::cerr << "Off:\t" << offAvg << std::endl;

        if(onAvg < offAvg)
            recommended = recommended | (1 << i);

    }
    auto calib = survive_calibration_config_create_from_idx(recommended);
    recommended = survive_calibration_config_index(&calib);
    std::cerr << "Recommended: " << recommended << std::endl;
    std::cerr << calib << std::endl;
    std::cerr << " which had " << error_map[recommended] << std::endl;

    std::cerr << "Best was " << best << ", worst was " << worst << std::endl;
    return 0;
}