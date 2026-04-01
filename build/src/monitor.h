#ifndef PPCA_MONITOR_H
#define PPCA_MONITOR_H

#include <vector>
#include <string>
#include "score.h"

struct Warning {
    bool speeding = false;
    std::vector<int> collision;

    bool empty() const {
        return !speeding && collision.empty();
    }

    void reset() {
        speeding = false;
        collision.clear();
    }
};

class Simulator;

class Interface {
    friend class Simulator;

private:
    std::vector<Warning> warnings;
    std::vector<Vec> pos_cur_list;
    std::vector<Vec> v_cur_list;
    std::vector<double> r_list;
    bool done = false;

    void init_set(int robot_num) {
        warnings.clear();
        for (int i = 0; i < robot_num; ++i) {
            warnings.push_back(Warning());
            pos_cur_list.push_back(Vec());
            v_cur_list.push_back(Vec());
        }
    }

    void warning_reset() {
        for (auto &warning: warnings) {
            warning.reset();
        }
    }

    void warning_set_collision(int id1, int id2) {
        warnings[id1].collision.push_back(id2);
        warnings[id2].collision.push_back(id1);
    }

    void warning_set_speeding(int id) {
        warnings[id].speeding = true;
    }

    bool no_warning() const {
        for (auto &warning: warnings) {
            if (warning.speeding || !warning.collision.empty()) {
                return false;
            }
        }
        return true;
    }

    void set_r(double _r) {
        r_list.push_back(_r);
    }

    void set_done() {
        done = true;
    }

    void set_pos_cur(int id, const Vec &pos_cur) {
        pos_cur_list[id] = pos_cur;
    }

    void set_v_cur(int id, const Vec &v_cur) {
        v_cur_list[id] = v_cur;
    }

public:
    Interface() = default;

    bool get_speeding(int id) const {
        return warnings[id].speeding;
    }

    std::vector<int> get_collision(int id) const {
        return warnings[id].collision;
    }

    bool get_warning() const {
        return !no_warning();
    }

    Vec get_pos_cur(int id) const {
        return pos_cur_list[id];
    }

    Vec get_v_cur(int id) const {
        return v_cur_list[id];
    }

    double get_r(int id) const {
        return r_list[id];
    }

    bool get_done() const {
        return done;
    }

    int get_robot_number() const {
        return r_list.size();
    }
};


class Monitor {
private:
    Interface *interface;
    Score *score;
    int test_id;
    long long share_info = 0;

public:
    Monitor(Interface *_interface, Score *_score, int _test_id) {
        interface = _interface;
        score = _score;
        test_id = _test_id;
    }

    bool get_speeding(int id) const {
        return interface->get_speeding(id);
    }

    std::vector<int> get_collision(int id) const {
        return interface->get_collision(id);
    }

    bool get_warning() const {
        return interface->get_warning();
    }

    Vec get_pos_cur(int id) const {
        return interface->get_pos_cur(id);
    }

    Vec get_v_cur(int id) const {
        return interface->get_v_cur(id);
    }

    double get_r(int id) const {
        return interface->get_r(id);
    }

    bool get_done() const {
        return interface->get_done();
    }

    int get_robot_number() const {
        return interface->get_robot_number();
    }

    int get_test_id() const {
        return test_id;
    }

    void write_share_info(long long info) {
        score->use_share();
        share_info = info;
    }

    long long use_share_info() const {
        score->use_share();
        return share_info;
    }
};

#endif //PPCA_MONITOR_H
