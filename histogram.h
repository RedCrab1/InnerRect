#pragma once

#include <stddef.h>
#include <vector>

template <typename T>
class Histogram
{
public:
	Histogram(const T lower_bound, const T upper_bound, const size_t histogram_bins)
	{
		lower_bound_ = lower_bound;
		upper_bound_ = upper_bound;
		range_inverse_ = 1.0/(upper_bound_-lower_bound_);
		histogram_bins_ = histogram_bins;
		data_.resize(histogram_bins);
		for (size_t i=0; i<data_.size(); ++i)
			data_[i] = 0.;
		raw_data_.resize(histogram_bins);
	}

	void addData(const T val, const double weight=1.0)
	{
		const size_t bin = std::max((size_t)0, std::min(histogram_bins_-1, (size_t)((val - lower_bound_) * range_inverse_ * (T)histogram_bins_)));
		data_[bin] += weight;
		raw_data_[bin].push_back(std::pair<T, double>(val, weight));
	}

	size_t getMaxBin()
	{
		double max_val = 0.;
		size_t max_bin = 0;
		for (size_t i=0; i<data_.size(); ++i)
		{
			if (max_val < data_[i])
			{
				max_val = data_[i];
				max_bin = i;
			}
		}

		return max_bin;
	}

	T getMaxBinPreciseVal()
	{
		if (raw_data_.size() == 0 || raw_data_.size() != data_.size())
			return 0.;

		const size_t max_bin = getMaxBin();
		T sum = 0.;
		T weight_sum = 0.;
		RawData& data = raw_data_[max_bin];
		for (size_t i=0; i<data.size(); ++i)
		{
			sum += data[i].first*data[i].second;
			weight_sum += data[i].second;
		}
		if (weight_sum==0)
			weight_sum = 1;
		return sum/weight_sum;
	}

protected:
	typedef std::vector< std::pair< T, double> > RawData;

	std::vector<double> data_;	// stores the histogram
	std::vector<RawData> raw_data_;	// stores all entered data pairs (data, weight) for each histogram bin
	T lower_bound_;		// lowest possible value
	T upper_bound_;		// highest possible value
	T range_inverse_;	// = 1.0/(upper_bound_-lower_bound_)
	size_t histogram_bins_;		// number of histogram bins
};
