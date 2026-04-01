#ifndef PPCA_SRC_HPP
#define PPCA_SRC_HPP
#include "math.h"

// Forward declarations from monitor.h (included before this in the judge build)
class Monitor;

class Controller {

public:
    Controller(const Vec &_pos_tar, double _v_max, double _r, int _id, Monitor *_monitor) {
        pos_tar = _pos_tar;
        v_max = _v_max;
        r = _r;
        id = _id;
        monitor = _monitor;
    }

    void set_pos_cur(const Vec &_pos_cur) {
        pos_cur = _pos_cur;
    }

    void set_v_cur(const Vec &_v_cur) {
        v_cur = _v_cur;
    }

private:
    int id;
    Vec pos_tar;
    Vec pos_cur;
    Vec v_cur;
    double v_max, r;
    Monitor *monitor;

    // Compute preferred velocity towards target, capped to not overshoot and not exceed v_max
    Vec preferred_velocity() const {
        Vec to_tar = pos_tar - pos_cur;
        double dist = to_tar.norm();
        if (dist <= 1e-3) return Vec(0, 0);
        double max_step_speed = dist / TIME_INTERVAL; // to stop exactly at target next step
        double speed = v_max;
        if (max_step_speed < speed) speed = max_step_speed;
        if (speed <= 1e-6) return Vec(0, 0);
        return to_tar.normalize() * speed;
    }

    // Predict if using velocity v_self would cause collision with robot j (using its current external state)
    bool will_collide_with(int j, const Vec &v_self) const {
        // Read other robot state
        Vec pos_j = monitor->get_pos_cur(j);
        Vec v_j = monitor->get_v_cur(j); // last step velocity as our prediction
        double r_j = monitor->get_r(j);

        Vec delta_pos = pos_cur - pos_j;
        Vec delta_v = v_self - v_j;

        double delta_v_norm = delta_v.norm();
        double project = delta_pos.dot(delta_v);
        if (project >= 0) {
            // Moving (relatively) away; no closer than now
            // But still check distance at end in case of near-parallel small approach
            double end_dis_sqr = (delta_pos + delta_v * TIME_INTERVAL).norm_sqr();
            double delta_r = r + r_j;
            return end_dis_sqr <= delta_r * delta_r - EPSILON;
        }
        project /= -delta_v_norm; // time to closest approach along relative motion
        double min_dis_sqr;
        double delta_r = r + r_j;
        if (project < delta_v_norm * TIME_INTERVAL) {
            // Closest approach happens within interval
            min_dis_sqr = delta_pos.norm_sqr() - project * project;
        } else {
            // Closest within interval is at the end
            min_dis_sqr = (delta_pos + delta_v * TIME_INTERVAL).norm_sqr();
        }
        return min_dis_sqr <= delta_r * delta_r - EPSILON;
    }

    // Check if velocity v_self is safe against all others under our prediction model
    bool is_safe_velocity(const Vec &v_self) const {
        int n = monitor->get_robot_number();
        for (int j = 0; j < n; ++j) {
            if (j == id) continue;
            if (will_collide_with(j, v_self)) return false;
        }
        // Speeding is checked by judge, but ensure we don't exceed bound here
        if (v_self.norm_sqr() > v_max * v_max + 1e-9) return false;
        return true;
    }

    // Generate candidate velocities by scaling and slight steering
    void generate_candidates(const Vec &v_pref, Vec out_list[], int &out_cnt) const {
        out_cnt = 0;
        // Speed scales (prioritized high to low)
        const double scales[] = {1.0, 0.8, 0.6, 0.4, 0.2, 0.0};
        // Angles to try (radians), small detours first
        const double angs[] = {0.0, 0.35, -0.35, 0.7, -0.7, 1.2, -1.2};

        // If we have right-of-way (small id), be more assertive: try straighter angles first already done
        // For larger ids, we can bias to smaller speeds by ordering scales as above

        for (double ang : angs) {
            Vec dir = v_pref.norm() > 1e-9 ? v_pref.rotate(ang).normalize() : Vec(0, 0);
            for (double s : scales) {
                Vec cand = dir * (v_pref.norm() * s);
                // Clamp just in case
                double sp = cand.norm();
                if (sp > v_max) cand = cand * (v_max / sp);
                out_list[out_cnt++] = cand;
            }
        }
        // Ensure zero velocity exists as last resort
        out_list[out_cnt++] = Vec(0, 0);
    }

public:

    Vec get_v_next() {
        // If very close to target, stop to avoid oscillation
        if ((pos_cur - pos_tar).norm() <= 1e-3) {
            return Vec(0, 0);
        }

        // Baseline preferred velocity towards the target
        Vec v_pref = preferred_velocity();

        // Simple yielding policy: higher IDs yield more when warnings exist near them
        // If last round had a warning involving us, reduce aggressiveness
        bool had_issue = false;
        if (monitor->get_speeding(id)) had_issue = true;
        if (!monitor->get_collision(id).empty()) had_issue = true;

        // Build candidate list
        Vec candidates[64];
        int cnt = 0;
        generate_candidates(v_pref, candidates, cnt);

        // If we are a larger id, bias to slower first when had issues
        if (had_issue && id > 0) {
            // Reorder: move slower candidates earlier (simple stable selection sort by speed asc)
            for (int i = 0; i < cnt; ++i) {
                int best = i;
                double best_sp = candidates[best].norm();
                for (int j = i + 1; j < cnt; ++j) {
                    double sp = candidates[j].norm();
                    if (sp < best_sp) { best = j; best_sp = sp; }
                }
                if (best != i) {
                    Vec tmp = candidates[i];
                    candidates[i] = candidates[best];
                    candidates[best] = tmp;
                }
            }
        }

        // Try candidates and pick the first safe one
        for (int i = 0; i < cnt; ++i) {
            Vec v_try = candidates[i];
            if (is_safe_velocity(v_try)) {
                return v_try;
            }
        }

        // As a fallback, creep very slowly toward target (tiny speed)
        Vec tiny = (pos_tar - pos_cur).normalize() * std::min(v_max, 0.05);
        return tiny;
    }
};


#endif //PPCA_SRC_HPP

