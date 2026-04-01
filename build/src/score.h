#ifndef PPCA_SCORE_H
#define PPCA_SCORE_H

#include <cmath>
#include "config.h"

class Score {
private:
    int time_max;
    int use_shared_info_num = 0;

    double share_punish() {
        if (use_shared_info_num == 0) {
            return 0.0;
        } else {
            return 5.0 + 0.05 * (use_shared_info_num - 1);
        }
    }

public:
    Score(int _time_max) {
        time_max = _time_max;
    }

    ~Score() = default;

    void use_share() {
        ++use_shared_info_num;
    }

    double score_done(double time) {
        return std::max(0.0, 70.0 + 30.0 * (1 - std::sqrt(time / time_max)) - share_punish());
    }

    double score_no_done(double n, double robot_num) {
        return std::max(0.0, 70.0 * (n / robot_num) * (n / robot_num) - share_punish());
    }
};

#endif //PPCA_SCORE_H
