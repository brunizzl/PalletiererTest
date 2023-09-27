#pragma once

#include <chrono>


class Tick {
	//different standard libraries use different types as result of ...::now()
	decltype(std::chrono::high_resolution_clock::now()) start;
	std::chrono::nanoseconds period; //length of one tick

public:
	Tick(std::chrono::nanoseconds period) :
		start(std::chrono::high_resolution_clock::now()),
		period(period)
	{}

	//waits for the time remaining between now and start of tick + period
	//returns the required waittime
	std::chrono::nanoseconds wait_till_end_of_tick() {
		auto const now = std::chrono::high_resolution_clock::now();
		auto const curr_duration = now - this->start;

		if (curr_duration < this->period) {
			this->start += this->period;
			std::this_thread::sleep_until(this->start);
		}
		else {
			this->start = now;
		}

		return this->period - curr_duration;
	}
}; //Tick
