//
// Created by thib on 24/08/22.
//

#include "./micro_benchmark.hpp"
#include "fmt_bridge.hpp"

#include <iostream>
#include <numeric>

namespace micro_benchmarks {

	constexpr duration_t no_time = duration_t(0);
	constexpr coarse_duration_t no_time_coarse = coarse_duration_t(0);

	TimeSeriesStatistics::TimeSeriesStatistics() :
		mean(no_time_coarse), min(no_time_coarse), max(no_time_coarse),
		variance(no_time_coarse), std_dev(no_time_coarse),
		quartile_1(no_time), median(no_time), quartile_3(no_time),
		percentile_90(no_time), percentile_95(no_time), percentile_99(no_time)
	{}

	// If nothing's given, preallocate 1000 spots.
	TimingsLogger::TimingsLogger() : TimingsLogger(1000) {}

	TimingsLogger::TimingsLogger(unsigned int number_laps) : stats() {
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
		this->last_lap++;
	}

	void TimingsLogger::compute_timing_stats() {
		if (not this->stats) {
			this->stats = std::make_shared<TimeSeriesStatistics>();
		}
		auto nblaps = this->iteration_times.size();
		std::cout << fmt::format("Computing statistics over {} laps ...", nblaps) << '\n';

		// Sort data :
		std::vector<duration_t> sorted_data(this->iteration_times.cbegin(), this->iteration_times.cend());
		std::sort(sorted_data.begin(), sorted_data.end());

		// Automagically converts and performs the duration_cast<>() for us !
		std::vector<fine_duration_t> fine_iteration_times(this->iteration_times.begin(), this->iteration_times.end());

		// Compute the mean value :
		double sum = std::accumulate(this->iteration_times.begin(), this->iteration_times.end(), 0.0, [](double &until_now, duration_t &duration) {
			return until_now + static_cast<double>(duration.count());
		});
		double raw_mean = sum / static_cast<double>(nblaps);

		// Compute the sum of ((data - mean)^2) over the whole dataset :
		double std_dev_sum = std::accumulate(fine_iteration_times.begin(), fine_iteration_times.end(), 0.0, [=](double &until_now, fine_duration_t &fine_duration) {
			double diff = fine_duration.count() - raw_mean;
			return diff * diff;
		});
		double raw_variance = std_dev_sum / static_cast<double>(nblaps);

		auto compute_percentile = [nblaps,&sorted_data](double percentage) -> duration_t {
			auto ordinal = static_cast<unsigned int>(std::ceil(percentage/100.0*static_cast<double>(nblaps)));
			return sorted_data[ordinal];
		};
		this->stats->quartile_1 = duration_t(compute_percentile(25.0));
		this->stats->median = duration_t(compute_percentile(50.0));
		this->stats->quartile_3 = duration_t(compute_percentile(75.0));
		this->stats->percentile_90 = duration_t(compute_percentile(90.0));
		this->stats->percentile_95 = duration_t(compute_percentile(95.0));
		this->stats->percentile_99 = duration_t(compute_percentile(99.0));

		// Cast data from 'double' --> chrono duration of fine_duration_t in 'double' --> chrono of coarse_duration_t :
		this->stats->total_running_time = to_coarse_duration(fine_duration_t(sum));
		this->stats->mean = to_coarse_duration(fine_duration_t(raw_mean));
		this->stats->min = to_coarse_duration(sorted_data[0]);
		this->stats->max = to_coarse_duration(sorted_data[this->iteration_times.size()-1]);
		this->stats->variance = to_coarse_duration(fine_duration_t(raw_variance));
		this->stats->std_dev = to_coarse_duration(fine_duration_t(std::sqrt(raw_variance)));
	}

	void TimingsLogger::print_timings(const std::string &banner_message = "", const std::string &message_prefix = "") const {
		std::string prefix = message_prefix;
		if (not message_prefix.empty()) { prefix = message_prefix + " "; }

		auto to_seconds = [](coarse_duration_t const& dur) -> std::chrono::duration<double> {
			return std::chrono::duration_cast<std::chrono::duration<double>>(dur);
		};

		if (not banner_message.empty()) { std::cout << prefix << "--- " << banner_message << " ---" << '\n'; }
		std::cout << prefix << "Time statistics for the current run :" << '\n';
		std::cout << prefix << fmt::format("- Total time : {: >24.8}", to_seconds(this->stats->total_running_time)) << '\n';
		std::cout << prefix << fmt::format("- Mean time  : {: >24.8}", this->stats->mean) << '\n';
		std::cout << prefix << fmt::format("- Std-dev    : {: >24.8}", this->stats->std_dev) << '\n';
		std::cout << prefix << fmt::format("- Min        : {: >24.8}", this->stats->min) << '\n';
		std::cout << prefix << fmt::format("- Max        : {: >24.8}", this->stats->max) << '\n';
		std::cout << prefix << "Significant values of the series :\n";
		std::cout << prefix << fmt::format("- Quartile 1 : {: >24.8}", to_coarse_duration(this->stats->quartile_1)) << '\n';
		std::cout << prefix << fmt::format("- Median     : {: >24.8}", to_coarse_duration(this->stats->median)) << '\n';
		std::cout << prefix << fmt::format("- Quartile 3 : {: >24.8}", to_coarse_duration(this->stats->quartile_3)) << '\n';
		std::cout << prefix << fmt::format("- 90th perc. : {: >24.8}", to_coarse_duration(this->stats->percentile_90)) << '\n';
		std::cout << prefix << fmt::format("- 95th perc. : {: >24.8}", to_coarse_duration(this->stats->percentile_95)) << '\n';
		std::cout << prefix << fmt::format("- 99th perc. : {: >24.8}", to_coarse_duration(this->stats->percentile_99)) << '\n';

		if (not banner_message.empty()) { std::cout << prefix << "--- " << banner_message << " ---" << '\n'; }
	}

	void TimingsLogger::reset_timings(unsigned int number_laps) {
		this->iteration_times = std::vector<duration_t>(number_laps, duration_t(0));
		this->last_start = my_clock_t::now();
		this->last_lap = 0;
		this->stats.reset();
	}

	LapTimer::LapTimer(std::shared_ptr<TimingsLogger> &timer, unsigned int lap_nb) :
			logger(timer), lap_number(lap_nb) {
		this->start = my_clock_t::now();
	}

	LapTimer::~LapTimer() {
		this->logger->set_lap_time(this->lap_number, my_clock_t::now() - this->start);
	}

}
