#include <atomic>
#include <cstddef>

template <typename T, size_t N>
class LockFreeQueue {
private:
    // 缓存行对齐避免伪共享
    struct alignas(64) Head {
        std::atomic<size_t> value{0};  // 读位置
    } head;

    struct alignas(64) Tail {
        std::atomic<size_t> value{0};  // 写位置
    } tail;

    T buffer[N];  // 数据缓冲区

public:
    LockFreeQueue() = default;

bool try_push(T&& item) {
    size_t current_tail = tail.value.load(std::memory_order_relaxed);
    size_t next_tail = (current_tail + 1) % N;

    if (next_tail == head.value.load(std::memory_order_acquire)) {
        return false;
    }

    buffer[current_tail].~T();
    new (&buffer[current_tail]) T(std::move(item)); // 移动构造

    while (!tail.value.compare_exchange_weak(
            current_tail, next_tail,
            std::memory_order_release,
            std::memory_order_relaxed)) {
        next_tail = (current_tail + 1) % N;
        if (next_tail == head.value.load(std::memory_order_acquire)) {
            return false;
        }
    }

    return true;
}

    bool try_pop(T& item) {
    size_t current_head = head.value.load(std::memory_order_relaxed);

    if (current_head == tail.value.load(std::memory_order_acquire)) {
        return false; // 队列空
    }

    // 使用移动语义（避免拷贝）
    item = std::move(buffer[current_head]);

    // 显式调用析构函数
    buffer[current_head].~T();

    // CAS 更新 head
    size_t next_head = (current_head + 1) % N;
    while (!head.value.compare_exchange_weak(
            current_head, next_head,
            std::memory_order_release,
            std::memory_order_relaxed)) {
        if (current_head == tail.value.load(std::memory_order_acquire)) {
            return false;
        }
        next_head = (current_head + 1) % N;
    }

    return true;
}

   bool is_empty() const {
    return head.value.load(std::memory_order_acquire) == 
           tail.value.load(std::memory_order_acquire);
}
};