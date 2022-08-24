#ifndef SPOT_MICRO_BENCHMARK_HPP_
#define SPOT_MICRO_BENCHMARK_HPP_

/*=============================================
 * Creator     : thib
 * Created on  : 24/08/22
 * Path        : /micro_benchmark.hpp
 * Description : A series of utilities and structs to perform time-related benchmarks.
 *=============================================
 */

#include <chrono>
#include <memory>
#include <string>
#include <vector>

/// @brief Some utility classes for time-related benchmarks.
namespace micro_benchmarks {

	/// @brief The used clock for benchmarks.
	using my_clock_t = std::chrono::system_clock;
	/// @brief The used duration type for the iteration time benchmarks.
	using duration_t = my_clock_t::duration;
	/// @brief The time point type used in lap chronos.
	using timepoint_t = my_clock_t::time_point;
	/// @brief Milliseconds with a double-precision floating point representation (easier to read)
	using coarse_duration_t = std::chrono::duration<double, std::milli>;
	/// @brief Like the duration_t type, but forced to be in double-precision floating point.
	using fine_duration_t = std::chrono::duration<double, duration_t::period>;

	class TimingsLogger {

	public: /* Constructors and destructors */
		explicit TimingsLogger();

		explicit TimingsLogger(unsigned int pre_allocated_laps);

		TimingsLogger(const TimingsLogger &) = delete;

		TimingsLogger(TimingsLogger &&) = delete;

		~TimingsLogger() = default;

	public: /* Public functions : */
		/// @brief Pre-allocate a new set of laps. Can be lower or higher than the number allocated before.
		void preallocate_laps(unsigned int number_laps);

		/// @brief Starts the chronometer for this lap.
		void start_lap();

		/// @brief Sets the lap time for lap 'n'.
		void set_lap_time(unsigned int lap_nb, duration_t lap_length);

		/// @brief Stops the chrono for this lap.
		void stop_lap();

		/// @brief Compute info about the timings.
		void compute_timing_stats();

		/// @brief Print some useful information to the screen, after the last call to fast_iterative_sliced_optimal_transfer().
		void print_timings(const std::string &banner_message, const std::string &message_prefix) const;

		/// @brief Resets the iteration times computed for the last run of fast_iterative_sliced_optimal_transfer().
		void reset_timings(unsigned int number_laps);

	protected:
		std::vector<duration_t> iteration_times; ///< The iteration times, updated each time fast_iterative_sliced_optimal_transfer() is called.
		unsigned int last_lap; ///< The last lap index (whenever using the {start|stop}_lap() functions)
		timepoint_t last_start; ///< The last start time point of the {start|stop}_lap() functions

		coarse_duration_t mean, min, max, variance, std_dev;
		duration_t quartile_1, median, quartile_3;
		duration_t percentile_90, percentile_95, percentile_99;
	};

	/// @brief RAII-style lap timer.
	/// @details Since the iterations can be parallelized, we need independent lap timers for each thread.
	class LapTimer {
	public:
		/// @brief Initializes the timer and its members, fetching the current clock time.
		LapTimer(std::shared_ptr<TimingsLogger> &logger, unsigned int lap_number);

		~LapTimer();

	protected:
		timepoint_t start;
		const std::shared_ptr<TimingsLogger> &logger;
		const unsigned int lap_number;
	};

	template<typename rep, typename period>
	std::chrono::duration<double, std::milli> to_coarse_duration(const std::chrono::duration<rep, period> &other_duration) {
		return std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(other_duration);
	}

} // namespace micro_benchmarks

#endif //SPOT_MICRO_BENCHMARK_HPP_
