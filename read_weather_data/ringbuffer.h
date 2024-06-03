#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <array>

template <class T, int max_elem>
class Ringbuffer
{
  public:
    using Type = T;

    bool pop(T &val) noexcept
    {
        if (getFillLevel() == 0)
        {
            return false;
        }

        val = data_[read_pos_];
        read_pos_ = (read_pos_ + 1) % size_;
        fill_level--;
        return true;
    }

    void push(const T &val) noexcept
    {
        data_[write_pos_] = val;
        write_pos_ = (write_pos_ + 1) % size_;

        if (full())
        {
            // move read position forward to avoid overwriting 
            read_pos_ = (read_pos_ + 1) % size_;
        }
        else
        {
            fill_level++;
        }
    }

    void reset() noexcept
    {
        read_pos_ = 0;
        write_pos_ = 0;
        fill_level = 0;
        data_.fill(T{});
    }

    int getMaxSize() const noexcept
    {
        return size_;
    }

    int getFillLevel() const noexcept
    {
        return fill_level;
    }

    int empty() const noexcept
    {
        return fill_level == 0;
    }

    int full() const noexcept
    {
        return fill_level == size_;
    }

    int capacity() const noexcept
    {
        return size_;
    }

  private:
    const int size_{max_elem};
    
    std::array<Type, max_elem> data_;
    int read_pos_{0};
    int write_pos_{0};
    int fill_level{0};
};

#endif // RING_BUFFER_H
