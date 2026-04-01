#include "simulator.h"
#include "score.h"

int main() {
    freopen(FILE_NAME, "w", stdout);
    int test_id, round_max;
    std::cin >> test_id >> round_max;
    std::cout << round_max << '\n';
    Score score = Score(round_max);
    Interface interface = Interface();
    Monitor monitor = Monitor(&interface, &score, test_id);
    Simulator simulator = Simulator(&interface, &monitor);
    simulator.init_set();
    for (int t = 0; t < round_max; ++t) {
        simulator.next_step();
        if (interface.get_done()) {
            std::cerr << "\nEnded at round " << t << ".\nYour score: " << score.score_done(t) << " / 100";
            return 0;
        }
    }
    std::cerr << "\nDidn't end with " << simulator.get_done_num() << " robots done.\nYour score: "
              << score.score_no_done(simulator.get_done_num(), simulator.get_robot_num()) << " / 100";
    return 0;
}
