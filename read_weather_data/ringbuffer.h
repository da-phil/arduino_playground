#ifndef RING_BUFFER_H
#define RING_BUFFER_H

namespace utils
{

template <class T>
class Ringbuffer
{
  public:
    using Type = T;

    explicit Ringbuffer(unsigned int max_elem)
        : size_{max_elem}, data_{nullptr}, read_pos_{0}, write_pos_{0}, fill_level_{0}
    {
        data_ = new Type[size_]();
    }

    ~Ringbuffer()
    {
        delete[] data_;
    }

    bool pop(T &val) noexcept
    {
        if (getFillLevel() == 0)
        {
            return false;
        }

        val = data_[read_pos_];
        read_pos_ = (read_pos_ + 1) % size_;
        fill_level_--;
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
            fill_level_++;
        }
    }

    void push(T &&val) noexcept
    {
        push(std::move(val));
    }

    void reset() noexcept
    {
        read_pos_ = 0;
        write_pos_ = 0;
        fill_level_ = 0;
    }

    int getMaxSize() const noexcept
    {
        return size_;
    }

    int getFillLevel() const noexcept
    {
        return fill_level_;
    }

    int empty() const noexcept
    {
        return fill_level_ == 0;
    }

    int full() const noexcept
    {
        return fill_level_ == static_cast<int>(size_);
    }

    int capacity() const noexcept
    {
        return size_;
    }

  private:
    const unsigned int size_;

    Type *data_;
    int read_pos_;
    int write_pos_;
    int fill_level_;
};

} // namespace utils

#endif // RING_BUFFER_H
