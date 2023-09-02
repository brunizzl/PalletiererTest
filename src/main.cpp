
#include <print>
#include <algorithm>
#include <cassert>
#include <array>

#include "coro_support.hpp"
#include "motors.hpp"

struct Position { std::int64_t x, y, z; };

constinit bool active = false;
constinit bool error = false;

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
    std::int64_t boxes_per_palette = 12;
};

constinit auto positions = GripperPositionParameters{};
constinit std::int64_t nr_boxes = 0;

Position next_stack_box_pos() {
    Position pos = positions.x_y_positions[nr_boxes % 4];
    pos.z = positions.floor_pos + (nr_boxes / 4) * positions.box_height;
    return pos;
}


namespace inlet {
    enum class State {
        Undefined,
        NoBox,
        MoveBox,
        BoxReady,
        COUNT
    };
    constinit State state = State::Undefined;

} //namespace inlet

namespace mag {
    enum class State {
        Undefined,
        Ready,
        Reloading,
        Empty,
        COUNT
    };
    constinit State state = State::Undefined;

    SideEffectCoroutine run() {
        assert(state == State::Undefined);
        while (true) {
            state = State::Ready;
            WAIT_WHILE(nr_boxes < positions.boxes_per_palette);
            state = State::Reloading;
            nr_boxes = 0;
            //TODO: simulate magazine (better then waiting for some time)
            for (auto i = 0; i < 5; i++) {
                WAIT;
            }
        }
    }

} //namespace mag


namespace arm {
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
    constinit State state = State::Undefined;

    //global variables
    constinit SimulatedMotor x_axis = {};
    constinit SimulatedMotor y_axis = {};
    constinit SimulatedMotor z_axis = {};
    constinit SimulatedPiston gripper = {};

    void update_simulated_parts() {
        x_axis.simulate_tick();
        y_axis.simulate_tick();
        z_axis.simulate_tick();
        gripper.simulate_tick();
    }

    //moves first vertical to initial_z, then to x and y, then to z
    SideEffectCoroutine go_to(std::int64_t const initial_z, Position const pos) {
        z_axis.go_to_pos(initial_z);
        WAIT_WHILE(z_axis.is_moving());

        x_axis.go_to_pos(pos.x);
        y_axis.go_to_pos(pos.y);
        WAIT_WHILE(x_axis.is_moving() || y_axis.is_moving());

        z_axis.go_to_pos(pos.z);
        WAIT_WHILE(z_axis.is_moving());
    }

    SideEffectCoroutine box_stacking_cycle() {
        assert(
            state != State::Undefined && 
            state != State::Homeing && 
            gripper.is_extended());

        state = State::ToWaitPos;
        EXEC(go_to(100, positions.wait_pos));

        state = State::Waiting;
        do {
            if (!active) {
                co_return;
            }
            WAIT;
        } while (inlet::state != inlet::State::BoxReady || mag::state != mag::State::Ready);

        state = State::TakeBox;
        assert(gripper.is_extended());
        EXEC(go_to(100, positions.box_pickup_pos));
        gripper.retract();
        WAIT_WHILE(!gripper.is_retracted());
        inlet::state = inlet::State::NoBox;

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
    
    SideEffectCoroutine homeing() {
        assert(state == State::Homeing);
        //this is obv. not how homeing works in practice
        //TODO: include sensors
        EXEC(go_to(0, Position{ 0, 0, 0 }));
        state = State::InHomePos;
    }

    SideEffectCoroutine run() {
        while (true) {
            assert(state == State::Undefined);

            WAIT_WHILE(error);
            state = State::Homeing;
            EXEC_WHILE(!error, homeing());

            while (!error) { //box transport cycle
                while (!active) {
                    //here manual arm operation management could be called (and allowed)
                    WAIT;
                }
                while (active) {
                    EXEC_WHILE(!error, box_stacking_cycle());
                }
            }
            //if an error eccurs, the program jumps here
            x_axis.stop();
            y_axis.stop();
            z_axis.stop();
            state = State::Undefined;
        }
    } //run


} //namespace arm

void debug_print() {
    char const* gripper = "??";
    if (arm::gripper.is_moving()) gripper = "move";
    if (arm::gripper.is_extended()) gripper = "open";
    if (arm::gripper.is_retracted()) gripper = "clse";

    std::println("gripper: {}, x: {:5}, y: {:5}, z: {:5}", 
        gripper, arm::x_axis.pos(), arm::y_axis.pos(), arm::z_axis.pos());
}


int main() {
    auto arm_update = arm::run();
    auto mag_update = mag::run();
    while (true) {
        arm::update_simulated_parts();
        arm_update();
        mag_update();
        debug_print();
    }
}
