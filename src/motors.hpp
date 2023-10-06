
#include <cstdint>
#include <algorithm>
#include <vector>
#include <algorithm>
#include <concepts>

template<typename T>
constexpr T sign(T x) {
    if (x < 0) return -1;
    if (x > 0) return 1;
    return 0;
}

template<typename Derived>
class SimulatedThing {
    friend Derived; //allows call of private constructor / destructor
    static inline std::vector<Derived*> instances = {};

    SimulatedThing() {
        static_assert(std::derived_from<Derived, SimulatedThing<Derived>>); //see crtp
        instances.push_back((Derived*)this);
    }

    ~SimulatedThing() {
        //expensive operation, not intended to be done until program is finished anyway
        auto pos = std::find(instances.begin(), instances.end(), (Derived*)this);
        instances.erase(pos);
    }

public:
    static void simulate_tick_for_all_instances() {
        for (Derived* const inst : instances) {
            inst->simulate_tick();
        }
    }
}; //class SimulatedThing

class SimulatedMotor: public SimulatedThing<SimulatedMotor> {
    std::int64_t target_pos = 0;
    std::int64_t curr_pos = 0;
    std::int64_t speed = 17;

public:
    SimulatedMotor() {}

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


class SimulatedPiston: public SimulatedThing<SimulatedPiston> {
    bool curr_extended = true;
    int ticks_until_change = 0;

public:
    SimulatedPiston() {}

    bool is_moving() const { return this->ticks_until_change != 0;  }
    bool is_extended() const { return !this->is_moving() && this->curr_extended; }
    bool is_retracted() const { return !this->is_moving() && !this->curr_extended; }

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


void simulate_all_parts() {
    SimulatedThing<SimulatedMotor>::simulate_tick_for_all_instances();
    SimulatedThing<SimulatedPiston>::simulate_tick_for_all_instances();
}

