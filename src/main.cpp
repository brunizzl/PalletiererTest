
#include <algorithm>
#include <cassert>
#include <array>
#include <chrono>
#include <thread>
#include <iostream>

#include "coro_support.hpp"
#include "motors.hpp"
#include "timer.hpp"
#include "settings.hpp"


enum class Error {
    InvalidGripperPos,
    EmergencyStop,
    BoxCatchedOnConveyor,
    //To be continued...

    COUNT
};

constinit Settings<Error> settings = {};

struct Position { std::int64_t x, y, z; };

//only correct in x and y axes.
constexpr auto update_x_y_positions(std::int64_t x1, std::int64_t x2, std::int64_t y1, std::int64_t y2) {
    return std::array{
        Position{ x1, y1, -1000 },
        Position{ x2, y1, -1000 },
        Position{ x1, y2, -1000 },
        Position{ x2, y2, -1000 }
    };
}

struct GripperPositionParameters {
    std::array<Position, 4> x_y_positions = update_x_y_positions(250, 150, 300, 200);
    Position wait_pos = Position{ 100, 100, 100 };
    Position box_pickup_pos = Position{ 100, 100, 200 };
    std::int64_t box_height = 30;
    std::int64_t floor_pos = 300;
    std::int64_t boxes_per_palette = 48;
};

constinit auto positions = GripperPositionParameters{};
constinit std::int64_t nr_boxes = 0;

Position next_stack_box_pos() {
    Position pos = positions.x_y_positions[nr_boxes % 4];
    pos.z = positions.floor_pos + (nr_boxes / 4) * positions.box_height;
    return pos;
}

struct Inlet {
    enum class State {
        Undefined,
        NoBox,
        MoveBox,
        BoxReady,
        COUNT
    };
    static inline constinit State state = State::Undefined;
    static constexpr std::size_t coroutines_stack_size = 512;
    static constexpr auto name = "Inlet";

    static SideEffectCoroutine<Inlet> run() {
        assert(state == State::Undefined);
        while (true) {
            WAIT_WHILE(!settings.is_active());
            state = State::MoveBox;
            for (auto i = 0; i < 10; i++) {
                YIELD;
            }
            state = State::BoxReady;
            WAIT_WHILE(state == State::BoxReady);
        }
    }
}; //struct Inlet

struct Mag {
    enum class State {
        Undefined,
        Ready,
        Reloading,
        Empty,
        COUNT
    };
    static inline constinit State state = State::Undefined;
    static constexpr std::size_t coroutines_stack_size = 512;
    static constexpr auto name = "Magazine";

    static SideEffectCoroutine<Mag> run() {
        assert(state == State::Undefined);
        while (true) {
            state = State::Ready;
            WAIT_WHILE(nr_boxes < positions.boxes_per_palette);
            state = State::Reloading;
            nr_boxes = 0;
            //TODO: simulate magazine (better then waiting for some time)
            for (auto i = 0; i < 5; i++) {
                YIELD;
            }
        }
    }

}; //struct Mag


struct Arm {
    enum class State {
        Undefined,
        Homeing,
        InHomePos,
        ToWaitPos,
        Waiting,
        TakeBox,
        TransportBox,
        ReleaseBox,
        COUNT
    };
    static inline constinit State state = State::Undefined;
    static constexpr std::size_t coroutines_stack_size = 512;
    static constexpr auto name = "Arm";

    //global variables
    static inline SimulatedMotor x_axis = {};
    static inline SimulatedMotor y_axis = {};
    static inline SimulatedMotor z_axis = {};
    static inline SimulatedPiston gripper = {};

    //moves first vertical to initial_z, then to x and y, then to z
    static SideEffectCoroutine<Arm> go_to(std::int64_t const initial_z, Position const pos) {
        z_axis.go_to_pos(initial_z);
        WAIT_WHILE(z_axis.is_moving());

        x_axis.go_to_pos(pos.x);
        y_axis.go_to_pos(pos.y);
        WAIT_WHILE(x_axis.is_moving() || y_axis.is_moving());

        z_axis.go_to_pos(pos.z);
        WAIT_WHILE(z_axis.is_moving());
    }

    static SideEffectCoroutine<Arm> box_stacking_cycle() {
        assert(
            state != State::Undefined && 
            state != State::Homeing && 
            gripper.is_extended());

        state = State::ToWaitPos;
        EXEC(go_to(100, positions.wait_pos));

        state = State::Waiting;
        do {
            if (!settings.is_active()) {
                co_return;
            }
            YIELD;
        } while (Inlet::state != Inlet::State::BoxReady || Mag::state != Mag::State::Ready);

        state = State::TakeBox;
        assert(gripper.is_extended());
        EXEC(go_to(100, positions.box_pickup_pos));
        gripper.retract();
        WAIT_WHILE(!gripper.is_retracted());
        Inlet::state = Inlet::State::NoBox;

        state = State::TransportBox;
        EXEC(go_to(100, next_stack_box_pos()));

        state = State::ReleaseBox;
        gripper.extend();
        WAIT_WHILE(!gripper.is_extended());
        nr_boxes++;

        state = State::ToWaitPos;
        EXEC(go_to(100, positions.wait_pos));

        state = State::Waiting;
    }
    
    static SideEffectCoroutine<Arm> homeing() {
        assert(state == State::Homeing);
        //this is obv. not how homeing works in practice
        //TODO: include sensors
        EXEC(go_to(0, Position{ 0, 0, 0 }));
        state = State::InHomePos;
    }

    static SideEffectCoroutine<Arm> run() {
        while (true) {
            assert(state == State::Undefined);

            WAIT_WHILE(settings.has_error());
            state = State::Homeing;
            EXEC_WHILE(!settings.has_error(), homeing());

            while (!settings.has_error()) { //box transport cycle
                while (!settings.is_active()) {
                    //here manual Arm operation management could be called (and allowed)
                    YIELD;
                }
                while (settings.is_active()) {
                    EXEC_WHILE(!settings.has_error(), box_stacking_cycle());
                }
            }
            //if an error eccurs, the program jumps here
            x_axis.stop();
            y_axis.stop();
            z_axis.stop();
            state = State::Undefined;
        }
    } //run
}; //struct Arm

void debug_print(std::chrono::nanoseconds const sleep_time) {
    char const* gripper = "??";
    if (Arm::gripper.is_moving()) gripper = "move";
    if (Arm::gripper.is_extended()) gripper = "open";
    if (Arm::gripper.is_retracted()) gripper = "clse";

    auto motor_state = [](auto const& motor) {
        return motor.is_moving() ? " move" : "still";
    };
    std::cout << "[gripper: " << gripper
        << ", x: " << motor_state(Arm::x_axis)
        << ", y: " << motor_state(Arm::y_axis)
        << ", z: " << motor_state(Arm::z_axis)
        << "] ";
    std::cout << "box nr: " << nr_boxes;

    using namespace std::chrono_literals;
    std::chrono::duration<double, std::milli> const as_millis = sleep_time;
    if (as_millis > 0ms) {
        std::cout << " (" << as_millis.count() << "ms left)\n";
    }
    else {
        std::cout << " TOOK " << -as_millis.count() << "ms TOO LONG!\n";
    }
}


int main() {
    settings.set_active();

    auto arm_update = Arm::run();
    auto mag_update = Mag::run();
    auto inl_update = Inlet::run();

    using namespace std::chrono_literals;
    auto timer = Tick(10ms);
    while (true) {
        arm_update();
        mag_update();
        inl_update(); 
        simulate_all_parts();

        auto const sleep_time = timer.wait_till_end_of_tick();
        debug_print(sleep_time);
    }
}
