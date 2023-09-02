
#include <cstdint>
#include <algorithm>

template<typename T>
T sign(T x) {
    if (x < 0) return -1;
    if (x > 0) return 1;
    return 0;
}

class SimulatedMotor {
    std::int64_t target_pos = 0;
    std::int64_t curr_pos = 0;
    std::int64_t speed = 55;

public:
    constexpr SimulatedMotor() {}

    bool is_moving() const { return this->curr_pos != this->target_pos; }
    std::int64_t pos() const { return this->curr_pos; }

    void go_to_pos(std::int64_t pos) {
        this->target_pos = pos;
    }

    void simulate_tick() {
        const std::int64_t diff = this->target_pos - this->curr_pos;
        const std::int64_t step = std::min(std::abs(diff), speed);
        this->curr_pos += sign(diff) * step;
    }

    void stop() {
        this->target_pos = this->curr_pos;
    }
}; //struct SimulatedMotor


class SimulatedPiston {
    bool curr_extended = true;
    int ticks_until_change = 0;

public:
    constexpr SimulatedPiston() {}

    bool is_moving() const { return this->ticks_until_change != 0;  }
    bool is_extended() const { return !this->ticks_until_change && this->curr_extended; }
    bool is_retracted() const { return !this->ticks_until_change && !this->curr_extended; }

    void extend() {
        if (!this->curr_extended) {
            this->ticks_until_change = 3;
        }
    }

    void retract() {
        if (this->curr_extended) {
            this->ticks_until_change = 3;
        }
    }

    void simulate_tick() {
        if (this->ticks_until_change > 0) {
            this->ticks_until_change--;
            if (this->ticks_until_change == 0) {
                this->curr_extended = !this->curr_extended;
            }
        }
    }

}; //struct SimulatedPiston

