#pragma once

#include <exception>

template<class T>
class History
{
	int size_;
	T *data_;
	int pos_;
	bool full_;

public:
	History(int max_size = 10)
	{
		if (max_size < 1) {
			max_size = 1;
		}

		size_ = max_size;
		data_ = new T[size_];
		pos_ = 0;
		full_ = false;
	}

	~History()
	{
		delete []data_;
	}

	size_t size() const { return size_; }
	bool full() const { return full_; }

	T& prev(int idx = 0) const
	{
		idx %= size_;

		if (pos_ - idx < 0) {
			return data_[size_ + pos_ - idx];
		}
		else {
			return data_[pos_ - idx];
		}
	}

	void operator++()
	{
		pos_++;
		if (pos_ == size_) {
			full_ = true;
		}
		pos_ %= size_;
	}

	History &operator++(int)
	{
		++(*this);
		return *this;
	}
};
