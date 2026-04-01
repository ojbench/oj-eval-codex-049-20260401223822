#include <vector>
#include <iostream>
#include "config.h"
#include "robot.h"
#include "monitor.h"


class Simulator {
private:
    std::vector<Robot *> robots;
    Interface *interface;
    Monitor *monitor;
    int round = 0;

public:
    Simulator(Interface *_interface, Monitor *_monitor) {
        interface = _interface;
        monitor = _monitor;
    }

    ~Simulator() {
        for (auto &robot: robots) {
            delete robot;
        }
    }

    void init_set() {
        int robot_num;
        std::cin >> robot_num;
        std::cout << robot_num << '\n';
        for (int i = 0; i < robot_num; ++i) {
            Vec pos_init, pos_tar;
            double r, v_max;
            std::cin >> pos_init >> pos_tar >> r >> v_max;
            std::cout << pos_init << pos_tar << r << '\n' << v_max << '\n';
            robots.push_back(new Robot(pos_init, pos_tar, r, v_max, i, monitor));
        }
        interface->init_set(robot_num);
        for (auto &robot: robots) {
            interface->set_r(robot->r);
        }
        for (std::size_t i = 0; i < robots.size(); ++i) {
            robots[i]->set_position();
            interface->set_pos_cur(i, robots[i]->pos_cur);
            interface->set_v_cur(i, robots[i]->v_cur);
            std::cout << robots[i]->pos_cur;
        }
    }

    void next_step() {
        for (auto robot: robots) {
            robot->set_velocity();
        }
        interface->warning_reset();
        // check velocity is no more than v_max
        for (std::size_t i = 0; i < robots.size(); ++i) {
            if (robots[i]->speeding()) {
                interface->warning_set_speeding(i);
            }
        }
        // collision detection
        for (std::size_t i = 0; i < robots.size(); ++i) {
            for (std::size_t j = i; j < robots.size(); ++j) {
                Vec delta_pos = robots[i]->pos_cur - robots[j]->pos_cur;
                Vec delta_v = robots[i]->v_cur - robots[j]->v_cur;
                double project = delta_pos.dot(delta_v);
                if (project >= 0) {
                    continue;
                }
                project /= -delta_v.norm();
                double min_dis_sqr;
                double delta_r = robots[i]->r + robots[j]->r;
                if (project < delta_v.norm() * TIME_INTERVAL) {
                    min_dis_sqr = delta_pos.norm_sqr() - project * project;
                } else {
                    min_dis_sqr = (delta_pos + delta_v * TIME_INTERVAL).norm_sqr();
                }
                if (min_dis_sqr <= delta_r * delta_r - EPSILON) {
                    interface->warning_set_collision(i, j);
                }
            }
        }
        // set next position and output
        if (interface->no_warning()) {
            for (std::size_t i = 0; i < robots.size(); ++i) {
                robots[i]->set_position();
                interface->set_pos_cur(i, robots[i]->pos_cur);
                interface->set_v_cur(i, robots[i]->v_cur);
            }
            bool flag = true;
            for (auto robot: robots) {
                if (!robot->done()) {
                    flag = false;
                    break;
                }
            }
            if (flag) {
                interface->set_done();
            }
        } else {
            std::cerr << "[round " << round << "]\n";
            for (std::size_t i = 0; i < robots.size(); ++i) {
                if (interface->get_speeding(i)) {
                    std::cerr << "[Speeding] Robot " << std::to_string(i) << ".\n";
                }
                auto list = interface->get_collision(i);
                if (!list.empty()) {
                    std::cerr << "[Collision] Robot " << std::to_string(i) << " and";
                    for (int id: list) {
                        std::cerr << ' ' << std::to_string(id);
                    }
                    std::cerr << '\n';
                }
            }
        }
        for (std::size_t i = 0; i < robots.size(); ++i) {
            std::cout << robots[i]->pos_cur;
        }
        ++round;
    }

    int get_done_num() const {
        int sum = 0;
        for (auto &robot: robots) {
            if (robot->done()) {
                ++sum;
            }
        }
        return sum;
    }

    int get_robot_num() const {
        return robots.size();
    }

};