#pragma once

class MovingAvg {
public:
	MovingAvg(unsigned int size, double init_avg=0.0 ) {
		this->size = size;
		this->idx = 0;
		this->sum = init_avg * this->size;
		this->arr = new double[size];
		for (unsigned int i = 0; i < size; ++i ) {
			this->arr[i] = init_avg;
		}
	}

	~MovingAvg() {
		delete[] arr;
	}

	double add(double v) {
		sum -= arr[idx];
		sum += v;
		arr[idx] = v;
		++idx;
		if (idx == size) {
			idx = 0;
		}

		return sum / size;
	}

	double average(void) {
		return sum / size;
	}

private:
	unsigned int size;
	unsigned int idx;
	double* arr;
	double sum;
};