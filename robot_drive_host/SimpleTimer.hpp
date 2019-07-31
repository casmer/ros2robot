#pragma once

#include <chrono>
#include <atomic>


template <typename Clock = std::chrono::high_resolution_clock>
class SimpleTimer
{
	typename Clock::time_point start_point;
public:
	SimpleTimer() :
		start_point(Clock::now())
{}

	template <typename Rep = typename Clock::duration::rep, typename Units = typename Clock::duration>
	Rep elapsed_time() const
	{
		std::atomic_thread_fence(std::memory_order_relaxed);
		auto counted_time = std::chrono::duration_cast<Units>(Clock::now() - start_point).count();
		std::atomic_thread_fence(std::memory_order_relaxed);
		return static_cast<Rep>(counted_time);
	}

	void reset()
	{
		start_point = Clock::now();
	}
};

using PreciseSimpleTimer = SimpleTimer<>;
using SystemSimpleTimer = SimpleTimer<std::chrono::system_clock>;
using MonotonicSimpleTimer = SimpleTimer<std::chrono::steady_clock>;
