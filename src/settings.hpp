#pragma once

#include <array>

template<typename Error>
class Settings {
    bool active = false;
    std::size_t nr_errors = 0;
    std::array<bool, (std::size_t)Error::COUNT> curr_errors = {};

    static constexpr std::size_t to_id(Error err) {
        std::size_t const err_id = static_cast<std::size_t>(err);
        assert(err_id < (size_t)(Error::COUNT));
        return err_id;
    }

public:
    bool is_active() const { return this->active; }
    bool has_error() const { return this->nr_errors; }
    std::size_t curr_error_count() const { return this->nr_errors; }
    bool error_is_set(Error const err) const { return this->curr_errors[to_id(err)]; }

    constexpr Settings() {}

    void set_error(Error const err) {
        auto const err_id = to_id(err);
        this->active = false;
        this->nr_errors += 1 - this->curr_errors[err_id]; //only add if this error was previously unreported
        this->curr_errors[err_id] = true;
    }

    void reset_error(Error const err) {
        auto const err_id = to_id(err);
        this->nr_errors -= this->curr_errors[err_id]; //only subtract if this error was previously reported
    }

    void set_active() {
        if (!this->has_error()) {
            this->active = true;
        }
    }

    void reset_active() { this->active = false; }
}; //class Settings


