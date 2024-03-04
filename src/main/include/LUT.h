#pragma once

#include <vector>

class LUT {
public:

	LUT(std::vector<double> x, std::vector<double> y) : x_vals(x), y_vals(y) {}

	double lookup(double x) {
		unsigned int i;
		for (i = 1; i < x_vals.size()-1; ++i) {
			if (x < x_vals[i]) break;
		}
		double slope = (y_vals[i] - y_vals[i - 1]) / (x_vals[i] - x_vals[i - 1]);
		return y_vals[i - 1] + (x - x_vals[i - 1]) * slope;
	}
private:
	std::vector<double> x_vals;
	std::vector<double> y_vals;
};