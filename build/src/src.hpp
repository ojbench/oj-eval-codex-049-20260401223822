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

    // Build a perpendicular unit vector to dir
    static Vec perp_unit(const Vec &dir) {
        Vec p(-dir.y, dir.x);
        double n = p.norm();
        return (n > 1e-9) ? (p / n) : Vec(0, 0);
    }

    // Rotate a unit direction by angle using std::cos/sin
    static Vec rotate_dir(const Vec &dir_unit, double ang) {
        double c = std::cos(ang);
        double s = std::sin(ang);
        Vec p = perp_unit(dir_unit);
        Vec v = dir_unit * c + p * s;
        double n = v.norm();
        return (n > 1e-9) ? (v / n) : dir_unit;
    }

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
        const double scales[] = {1.0, 0.9, 0.75, 0.6, 0.45, 0.3, 0.15, 0.0};
        // Angles to try (radians), small detours first
        const double angs[] = {0.0, 0.25, -0.25, 0.5, -0.5, 0.9, -0.9, 1.2, -1.2, 1.57, -1.57};

        // If we have right-of-way (small id), be more assertive: try straighter angles first already done
        // For larger ids, we can bias to smaller speeds by ordering scales as above

        Vec base_dir = v_pref.norm() > 1e-9 ? v_pref.normalize() : Vec(1, 0);
        for (double ang : angs) {
            Vec dir = rotate_dir(base_dir, ang);
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

        // We'll build candidate list below after checking detour necessity

        // Pre-check: detect potential head-on corridor and prepend perpendicular detour candidates
        bool need_detour = false;
        Vec dir = v_pref.norm() > 1e-9 ? v_pref.normalize() : Vec(1, 0);
        int n = monitor->get_robot_number();
        bool yield_to_lower = false;
        for (int j = 0; j < n; ++j) {
            if (j == id) continue;
            Vec pj = monitor->get_pos_cur(j);
            Vec vj = monitor->get_v_cur(j);
            double rj = monitor->get_r(j);
            Vec rel = pj - pos_cur;
            double ahead = rel.dot(dir);
            double lateral = std::abs(rel.cross(dir));
            double corridor = r + rj + 0.5; // margin
            if (ahead > 0 && lateral < corridor) {
                // approaching corridor
                double closing = (dir.dot(vj) < 0) ? 1.0 : 0.0;
                if (closing > 0.5) { need_detour = true; }
                if (j < id) { yield_to_lower = true; }
                if (need_detour && yield_to_lower) break;
            }
        }
        // Simple right-of-way: yield to any lower-id robot in our corridor to avoid deadlock
        if (yield_to_lower) {
            return Vec(0, 0);
        }

        // Build candidate list
        Vec candidates[96];
        int cnt = 0;
        if (need_detour) {
            // Try perpendicular motions first, pick side by id parity for symmetry breaking
            Vec perp = Vec(-dir.y, dir.x);
            double sign = (id % 2 == 0) ? 1.0 : -1.0;
            double base = std::min(v_max, std::max(0.2, v_pref.norm()));
            Vec det1 = perp * (base * sign);
            Vec det2 = perp * (base * -sign * 0.6) + dir * (base * 0.3);
            // Clamp to v_max
            auto clamp = [&](Vec v){ double sp = v.norm(); return sp > v_max ? v * (v_max / sp) : v; };
            det1 = clamp(det1);
            det2 = clamp(det2);
            candidates[cnt++] = det1;
            candidates[cnt++] = det2;
        }

        // Standard candidates
        Vec tmp_list[64]; int tmp_cnt = 0;
        generate_candidates(v_pref, tmp_list, tmp_cnt);
        for (int i = 0; i < tmp_cnt && cnt < 96; ++i) candidates[cnt++] = tmp_list[i];

        // Optional reorder: if we had issues and have larger id, prefer slower candidates
        if (had_issue && id > 0) {
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
        Vec tiny = (pos_tar - pos_cur).normalize() * std::min(v_max, 0.2);
        // Add slight perpendicular to break symmetry
        Vec perp = Vec(-dir.y, dir.x) * (0.1 * ((id % 2 == 0) ? 1 : -1));
        tiny += perp;
        double sp = tiny.norm();
        if (sp > v_max) tiny = tiny * (v_max / sp);
        return tiny;
    }
};


#endif //PPCA_SRC_HPP
