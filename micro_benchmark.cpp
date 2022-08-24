//
// Created by thib on 24/08/22.
//

#include "./micro_benchmark.hpp"

#include <fmt/core.h>
#include <fmt/chrono.h>

#include <iostream>
#include <numeric>

namespace micro_benchmarks {

	constexpr duration_t no_time = duration_t(0);
	constexpr coarse_duration_t no_time_coarse = coarse_duration_t(0);

	// If nothing's given, preallocate 1000 spots.
	TimingsLogger::TimingsLogger() : TimingsLogger(1000) {}

	TimingsLogger::TimingsLogger(unsigned int number_laps) :
			mean(no_time_coarse), min(no_time_coarse), max(no_time_coarse),
			variance(no_time_coarse), std_dev(no_time_coarse),
			quartile_1(no_time), quartile_3(no_time), median(no_time),
			percentile_90(no_time), percentile_95(no_time), percentile_99(no_time) {
		this->last_lap = 0;
		this->last_start = my_clock_t::now();
		this->preallocate_laps(number_laps);
	}

	void TimingsLogger::preallocate_laps(unsigned int number_laps) {
		this->iteration_times = std::vector<duration_t>(number_laps, duration_t(0));
	}

	void TimingsLogger::start_lap() {
		this->last_start = my_clock_t::now();
	}

	void TimingsLogger::set_lap_time(unsigned int lap_nb, duration_t lap_length) {
		this->iteration_times[lap_nb] = lap_length;
	}

	void TimingsLogger::stop_lap() {
		timepoint_t end = my_clock_t::now();
		this->iteration_times[this->last_lap] = end - this->last_start;
		this->last_start = end;
	}

	void TimingsLogger::compute_timing_stats() {
		std::cout << "Sorting data ..." << '\n';
		// Sort data :
		std::vector<duration_t> sorted_data(this->iteration_times.cbegin(), this->iteration_times.cend());
		std::sort(sorted_data.begin(), sorted_data.end());

		std::cout << "Converting to fine duration ..." << '\n';
		// Automagically converts and performs the duration_cast<>() for us !
		std::vector<fine_duration_t> fine_iteration_times(this->iteration_times.begin(), this->iteration_times.end());

		// Compute the mean value :
		std::cout << "Accumulating and computing mean/min/max ..." << '\n';
		double sum = std::accumulate(this->iteration_times.begin(), this->iteration_times.end(), 0.0, [](double &until_now, duration_t &duration) {
			return until_now + static_cast<double>(duration.count());
		});
		double raw_mean = sum / static_cast<double>(this->iteration_times.size());

		// Compute the sum of ((data - mean)^2) over the whole dataset :
		std::cout << "Computing variance and standard deviation ..." << '\n';
		double std_dev_sum = std::accumulate(fine_iteration_times.begin(), fine_iteration_times.end(), 0.0, [=](double &until_now, fine_duration_t &fine_duration) {
			double diff = fine_duration.count() - raw_mean;
			return diff * diff;
		});
		double raw_variance = std_dev_sum / static_cast<double>(this->iteration_times.size());

		std::cout << "Computing percentiles (25, 50, 75, 90, 95, 99) ...." << '\n';
		auto compute_ordinal_percentile = [&sorted_data](double percentage) {
			return static_cast<unsigned int>(std::ceil(percentage/100.0*static_cast<double>(sorted_data.size())));
		};
		this->quartile_1 = duration_t(sorted_data[compute_ordinal_percentile(25.0)]);
		this->median = duration_t(sorted_data[compute_ordinal_percentile(50.0)]);
		this->quartile_3 = duration_t(sorted_data[compute_ordinal_percentile(75.0)]);
		this->percentile_90 = duration_t(sorted_data[compute_ordinal_percentile(90.0)]);
		this->percentile_95 = duration_t(sorted_data[compute_ordinal_percentile(95.0)]);
		this->percentile_99 = duration_t(sorted_data[compute_ordinal_percentile(99.0)]);

		std::cout << "Done computing statistics.\n";

		// Cast data from 'double' --> chrono duration of duration_t in 'double' --> chrono of coarse_duration_t :
		this->mean = to_coarse_duration(fine_duration_t(raw_mean));
		this->min = to_coarse_duration(sorted_data[0]);
		this->max = to_coarse_duration(sorted_data[this->iteration_times.size()]);
		this->variance = to_coarse_duration(fine_duration_t(raw_variance));
		this->std_dev = to_coarse_duration(fine_duration_t(std::sqrt(raw_variance)));
	}

	void TimingsLogger::print_timings(const std::string &banner_message = "", const std::string &message_prefix = "") const {
		std::string prefix = message_prefix;
		if (not message_prefix.empty()) { prefix = message_prefix + " "; }

		if (not banner_message.empty()) { std::cout << prefix << "--- " << banner_message << " ---" << '\n'; }
		std::cout << prefix << "Time statistics for the current run :" << '\n';
		std::cout << prefix << fmt::format("- Mean time  : {}", this->mean) << '\n';
		std::cout << prefix << fmt::format("- Variance   : {}", this->variance) << '\n';
		std::cout << prefix << fmt::format("- Std-dev    : {}", this->std_dev) << '\n';
		std::cout << prefix << fmt::format("- Min/Max    : [ {} , {} ]", this->min, this->max) << '\n';
		std::cout << prefix << "Significant values of the series :\n";
		std::cout << prefix << fmt::format("- Quartile 1 : {}", this->quartile_1) << '\n';
		std::cout << prefix << fmt::format("- Median     : {}", this->median) << '\n';
		std::cout << prefix << fmt::format("- Quartile 3 : {}", this->quartile_3) << '\n';
		std::cout << prefix << fmt::format("- 90th perc. : {}", this->percentile_90) << '\n';
		std::cout << prefix << fmt::format("- 95th perc. : {}", this->percentile_95) << '\n';
		std::cout << prefix << fmt::format("- 99th perc. : {}", this->percentile_99) << '\n';

		if (not banner_message.empty()) { std::cout << prefix << "--- " << banner_message << " ---" << '\n'; }
	}

	void TimingsLogger::reset_timings(unsigned int number_laps) {
		this->iteration_times = std::vector<duration_t>(number_laps, duration_t(0));
		this->last_start = my_clock_t::now();
		this->last_lap = 0;
	}

	LapTimer::LapTimer(std::shared_ptr<TimingsLogger> &timer, unsigned int lap_nb) :
			logger(timer), lap_number(lap_nb) {
		this->start = my_clock_t::now();
	}

	LapTimer::~LapTimer() {
		this->logger->set_lap_time(this->lap_number, my_clock_t::now() - this->start);
	}

}
