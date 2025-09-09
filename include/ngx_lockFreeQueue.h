#include <atomic>
#include <cstddef>

template <typename T, size_t N>
class LockFreeQueue {
private:
    // 缓存行对齐避免伪共享
    struct alignas(64) Head {
        std::atomic<size_t> value{0};  // 读位置
        //统一初始化语法，比传统的value()或value=0更简洁
    } head;

    struct alignas(64) Tail {
        std::atomic<size_t> value{0};  // 写位置
        //未实质解决ABA问题
    } tail;
    /*
        伪共享是多线程程序中的一种性能问题，指 多个线程访问不同的数据，但这些数据恰好位于同一个缓存行中 ，
        导致不必要的缓存一致性开销。
        CPU不能单独加载单个字节，必须以缓存行为单位加载。
        缓存一致协议（如MESI）以缓存行为单位工作
        一个缓存行被修改，其他CPU核心的副本都会失效
        伪共享的执行过程：
            1. 线程A（CPU核心1）修改 counter1
            → CPU1的L1缓存获得该缓存行的独占权
            → 其他CPU的缓存行副本被标记为无效

            2. 线程B（CPU核心2）修改 counter2  
            → CPU2需要获得缓存行的独占权
            → 从CPU1的缓存或内存重新加载缓存行
            → CPU1的缓存行被标记为无效

            3. 线程A再次访问 counter1
            → CPU1的缓存行已失效
            → 必须重新从CPU2或内存加载
   
            4. 循环往复...
        伪共享的本质是不相关的数据共享缓存行，导致不必要的缓存一致性开销，利用缓存行对齐各自独占缓存行，无相互干扰。
    */

    T buffer[N];  // 环形数据缓冲区

public:
    LockFreeQueue() = default;
    //告诉编译器为 LockFreeQueue 类生成默认的构造函数
bool try_push(T&& item) {
    size_t current_tail = tail.value.load(std::memory_order_relaxed);
    //简单的原子性操作，无顺序保证
    size_t next_tail = (current_tail + 1) % N;

    // 检查队列是否已满
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
    /*
        compare_exchange_weak操作：
            比较 ：检查 tail.value 是否等于 current_tail
            交换 ：如果相等，将 tail.value 设为 next_tail
            返回 ：成功返回 true ，失败返回 false 并更新 current_tail
            执行流程如下：
                初始状态：tail = 5

                线程A：想要将 tail 从 5 更新到 6
                线程B：同时想要将 tail 从 5 更新到 6

                执行过程：
                1. 线程A：CAS(5 → 6) 成功
                2. 线程B：CAS(5 → 6) 失败，因为 tail 已经是 6
                3. 线程B：重新读取 tail = 6，计算 next_tail = 7
                4. 线程B：CAS(6 → 7) 重试
            循环重试逻辑：
                成功 ：CAS 成功，跳出循环
                失败 ：其他线程修改了 tail，需要重试
            成功时，std::用memory_order_release内存序将next_tail写入tail.value，
            保证数据写入操作在 tail 指针更新之前对其他线程可见
            失败时，std::memory_order_relaxed内存序，更新current_tail为next_tail，
            只保证原子性 ：current_tail更新 tail.value 的当前值是原子的
            CAS失败意味着：没有写入操作，不需要其他线程同步，只是读取了当前值用于重试，性能考虑。
    */

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

    // 获取队列当前大小
    size_t size() const {
        size_t current_head = head.value.load(std::memory_order_acquire);
        size_t current_tail = tail.value.load(std::memory_order_acquire);
        
        if (current_tail >= current_head) {
            return current_tail - current_head;
        } else {
            return N - current_head + current_tail;
        }
    }
    
    // 获取队列容量
    size_t capacity() const {
        return N - 1;  // 实际可用容量比N小1，因为需要区分满和空
    }
};
/*
    内存序：
    编译器优化而产生的指令乱序，cpu指令流水线也会产生指令乱序，总体来讲，编译器优化层面会产生的指令乱序，
    cpu层面也会的指令流水线优化产生乱序指令。当然这些乱序指令都是为了同一个目的，优化执行效率。

    happens-before:按照程序的代码序执行，happens-before 是一个偏序关系，它定义了两个事件之间的可见性和顺序关系。
    而不是严格的执行顺序。happens-before不等于实际的执行时间顺序。
    Happens-before = 可见性保证 + 顺序约束。
    单线程内 ：基本遵循程序顺序（除非编译器优化），多线程间 ：通过同步操作（如release-acquire）建立。
    synchronized-with:不同线程间，对于同一个原子操作，需要同步关系，store（）操作一定要先于 load（），
    也就是说 对于一个原子变量x，先写x，然后读x是一个同步的操作，读x并不会读取之前的值，而是当前写x的值。
    Synchronized-with = 线程间的同步关系
    建立条件 ：Release-Acquire配对 + 同一原子变量 + 读取到写入值
    作用 ：建立happens-before关系，保证内存可见性

    6种memory_order 主要分成3类，relaxed(松弛的内存序)，sequential_consistency（内存一致序）,
    acquire-release(获取-释放一致性)

    relaxed内存序：
    没有顺序一致性的要求，也就是说同一个线程的原子操作还是按照happens-before关系，
    但不同线程间的执行关系是任意的，不同线程的原子操作之间没有顺序关系。只保证原子操作的原子性，不提供任何同步或顺序约束。
    exp:
        // 线程1
        int data = 42;                                    // A - 普通内存操作   
        flag.store(true, std::memory_order_relaxed);  // B - relaxed原子操作

        // 线程2  
        if (flag.load(std::memory_order_relaxed)) {   // C - relaxed原子操作
            int value = data;  // D - 普通内存操作，不保证能看到data=42！
        } 
    操作A（ data = 42 ）和操作B（ flag.store ）之间没有同步关系
    操作C（ flag.load ）和操作D（ int value = data ）之间也没有同步关系
    即使线程2看到了 flag == true ，也不保证能看到 data = 42 的修改
    由于编译器和CPU的优化，可能的执行顺序为：
        线程1: flag.store(true) 先于 data = 42 被其他线程看到
        线程2: 看到 flag == true，但还没看到 data = 42

    sequential consistency(内存一致性)：
    这个是以牺牲优化效率，来保证指令的顺序一致执行，相当于不打开编译器优化指令，
    按照正常的指令序执行(happens-before)，多线程各原子操作也会Synchronized-with
    Sequential Consistency 是 C++ 内存模型中最强的内存序，对应 std::memory_order_seq_cst ，也是原子操作的默认内存序。
    全局同一顺序：
        所有线程看到的所有原子操作都有一个统一的全局顺序
        这个顺序在所有线程中都是一致的
        就像所有操作都在一个单线程中按某种顺序执行
    程序顺序保证：
        每个线程内的操作顺序与程序中的顺序一致
        不允许重排序违反程序中的先后关系
    最强同步保证：
        提供 acquire 和 release 语义
        建立强 happens-before 关系
        保证完全的内存可见性
    exp:
        std::atomic<bool> flag1{false};
        std::atomic<bool> flag2{false};
        int data = 0;

        // 线程1
        data = 42;
        flag1.store(true);  // 默认 seq_cst(C++原子操作默认内存序)

        // 线程2  
        if (flag1.load()) {  // 默认 seq_cst
            flag2.store(true);
        }

        // 线程3
        if (flag2.load()) {  // 默认 seq_cst
            assert(data == 42);  // 保证成功！
        }
    
    acquire-release 获取-释放一致性
    Acquire-Release 是 C++ 内存模型中的一对配合使用的内存序，
    提供了比 relaxed 更强但比 sequential consistency 更轻量的同步保证。
    Release 语义 ( std::memory_order_release )
        释放操作 ：写入操作完成时，确保之前的所有内存操作对其他线程可见
        同步点 ：创建一个"发布点"，之前的操作不能重排到这个点之后
    Acquire 语义 ( std::memory_order_acquire )
        获取操作 ：读取操作完成时，确保之后的所有内存操作不会重排到读取之前
        同步点 ：创建一个"获取点"，之后的操作不能重排到这个点之前
    Synchronized-with 关系（同步机制）
        当满足以下条件时建立同步关系：
            线程A执行 release 操作写入值
            线程B执行 acquire 操作读取到该值
            操作的是同一个原子变量
        std::atomic<bool> ready{false};
        std::atomic<int> data{0};

        // 线程1 (Producer)
        data = 42;                                    // A
        ready.store(true, std::memory_order_release); // B (release)

        // 线程2 (Consumer)
        if (ready.load(std::memory_order_acquire)) {  // C (acquire)
            assert(data == 42);  // D - 保证成功！
        }
        Release 操作之前的所有内存写入对后续的 Acquire 操作可见
        防止编译器和 CPU 将 Release 之前的操作重排到 Release 之后
        Acquire 操作之后的所有内存读取能看到 Release 操作之前的写入
        防止编译器和 CPU 将 Acquire 之后的操作重排到 Acquire 之前
    配对使用 ：Release 和 Acquire 必须配对才能建立同步
    单向同步 ：只保证 Release 之前对 Acquire 之后可见
    局部顺序 ：不保证全局操作顺序，只保证相关操作的顺序
    实用平衡 ：在性能和正确性之间的最佳平衡点
    比 sequential consistency 开销更小，只在必要的同步点施加约束，允许更多的编译器和 CPU 优化。
    实质就是release写操作需发生在acquire之前，保证acquire读取到的值是release写入的值，同步关系。

    compare_exchange_weak 的本质：编译器内建函数 + 硬件指令（CAS）
    完整函数签名：
        tail.value.compare_exchange_weak(
            current_tail,                  // 参数1：期望值引用
            next_tail,                     // 参数2：新值
            std::memory_order_release,     // 参数3：成功时内存序
            std::memory_order_relaxed      // 参数4：失败时内存序
        )
        应用层：
            tail.value.compare_exchange_weak(current_tail, next_tail, ...)
        编译器层：
            // 编译器将其转换为内建函数
            __sync_bool_compare_and_swap(&tail.value, current_tail, next_tail)
        硬件层：
            ; x86-64 汇编指令
            lock cmpxchg [memory], register
    // 1. 编译时：转换为硬件指令
    // 2. 运行时：CPU 直接执行
    // 3. 硬件保证：原子性和内存序
    // 4. 无内核参与：纯用户态操作

    compare_exchange_weak与compare_exchange_strong的区别：
        核心差异在于虚假失败（Spurious Failure）
        compare_exchange_weak允许虚假失败，失败原因可能为值不匹配或者硬件原因，需要循环重试
        compare_exchange_strong不允许虚假失败，失败的原因仅为值不匹配
        虚假失败是指在硬件层面竞争导致失败，真实失败是期望值与真实值不一致
        compare_exchange_strongs失败的唯一原因是期望着和真实值不一致，硬件会自定重试直到确定结果。
    CAS和自旋锁的区别：
        两者都会导致CPU空转。
        CAS的锁定范围是单个原子变量，自旋锁是整个临界区。
        CAS是非阻塞的，自旋锁会阻塞其他线程。前者是局部竞争，后者是全局竞争。
    CAS操作的核心优势在于它通过硬件级别的原子指令实现了非阻塞的同步，
    从而在高并发场景下能提供更高的吞吐量和可伸缩性，同时避免了死锁、优先级反转等传统锁带来的问题。

    传统锁（互斥锁）： 当一个线程获取锁后，其他所有竞争该锁的线程都会被挂起（进入阻塞状态）。
    线程的上下文切换（Context Switching）是一项开销巨大的操作，需要从用户态切换到内核态，
    切换过程会消耗大量的CPU周期。
    CAS操作： 它不会导致线程挂起。当一个线程执行CAS失败后，它不会被阻塞，而是可以立即进行重试或执行其他逻辑。
    这极大地减少了线程上下文切换的开销，使得CPU时间能够更多地花在业务逻辑上，而不是线程管理上。
    在竞争非常激烈的场景下，频繁的重试可能会浪费CPU（自旋），但在低至中度竞争的场景下，性能提升是巨大的。
    优先级反转： 这是一个在实时系统中常见的问题。假设一个低优先级的线程持有了一个锁，
    然后一个高优先级的线程来尝试获取这个锁，它会被阻塞。此时如果中优先级的线程开始运行，
    它会抢占低优先级线程的CPU时间，导致低优先级线程无法继续执行从而无法释放锁，最终导致高优先级线程无限期等待。

    ABA问题：
        ABA问题是无锁编程中的经典问题，指在CAS（Compare-And-Swap）操作过程中， 
        共享变量的值从A变为B，然后又变回A ，导致CAS操作误认为值没有改变而成功执行，但实际上中间状态已经发生了变化。
        // 共享变量
        std::atomic<int> shared_value{100};  // 初始值A = 100

        // 线程1的操作
        void thread1() {
        int expected = shared_value.load();  // expected = 100 (A)
    
        // 模拟一些耗时操作
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
        // 尝试CAS操作：将100改为200
        bool success = shared_value.compare_exchange_weak(expected, 200);
        // 可能成功，但中间状态已经改变！
        }

        // 线程2的操作
        void thread2() {
            shared_value.store(50);   // A → B (100 → 50)
            shared_value.store(100);  // B → A (50 → 100)
            // 值又变回了100，但状态已经不同
        }
    时间轴：
        t0: shared_value = 100 (A)
        t1: 线程1读取 expected = 100
        t2: 线程2修改 shared_value = 50 (B)
        t3: 线程2修改 shared_value = 100 (A)
        t4: 线程1执行CAS(100, 200) → 成功！
        但线程1不知道中间发生了 100→50→100 的变化
    //无锁栈的ABA问题示例：
        struct Node {
        int data;
        Node* next;
    };

    class LockFreeStack {
        std::atomic<Node*> head;
        
    public:
        void push(int value) {
            Node* new_node = new Node{value, nullptr};
            Node* old_head = head.load();
            do {
                new_node->next = old_head;
            } while (!head.compare_exchange_weak(old_head, new_node));
        }               
    
    bool pop(int& result) {
        Node* old_head = head.load();
        do {
            if (old_head == nullptr) return false;
            result = old_head->data;
            // 危险：如果old_head被释放后重新分配到相同地址
            // 就会发生ABA问题！
        } while (!head.compare_exchange_weak(old_head, old_head->next));
        
        delete old_head;  // 可能导致use-after-free
        return true;
        }
    };
    初始状态：Stack = [A] → [B] → [C] → nullptr

    线程1执行pop()：
    1. old_head = A
    2. 准备将head从A改为B
    3. 被调度器暂停...

    线程2执行操作：
    1. pop() A  → Stack = [B] → [C] → nullptr
    2. pop() B  → Stack = [C] → nullptr  
    3. push() A → Stack = [A] → [C] → nullptr
       (A节点被重新分配到相同内存地址)

    线程1恢复执行：
    4. CAS(A, B) 成功！因为head确实等于A
    5. 但现在A->next指向C，不是原来的B
    6. 结果：Stack = [B] → ??? (B已被释放，悬空指针！)
    可通过版本号的方式解决ABA问题：
        每个节点添加一个版本号字段，每次pop时检查版本号是否匹配，不匹配则重试。
*/