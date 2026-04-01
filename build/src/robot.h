#ifndef PPCA_ROBOT_H
#define PPCA_ROBOT_H

#include "config.h"
#include "math.h"
#include "monitor.h"
#include "src.hpp"

class Robot {
private:
    Vec pos_tar;
    double v_max;
    Monitor *monitor;
    Controller controller;

public:
    Vec pos_cur;
    Vec v_cur;
    double r;
    int id;

    Robot(const Vec &_pos_init, const Vec &_pos_tar, double _r, double _v_max, int _id, Monitor *_monitor) :
            controller(_pos_tar, _v_max, _r, _id, _monitor) {
        pos_cur = _pos_init;
        pos_tar = _pos_tar;
        v_cur = Vec();
        r = _r;
        v_max = _v_max;
        id = _id;
        monitor = _monitor;
    }

    void set_position() {
        pos_cur += v_cur * TIME_INTERVAL;
    }

    bool speeding() const {
        return v_cur.norm_sqr() >= v_max * v_max + EPSILON;
    }

    bool done() const {
        return (pos_cur - pos_tar).norm_sqr() <= EPSILON * EPSILON;
    }

    void set_velocity() {
        controller.set_v_cur(v_cur);
        controller.set_pos_cur(pos_cur);
        v_cur = controller.get_v_next();
    }
};

#endif //PPCA_ROBOT_H
