#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>   //信号相关头文件 
#include <errno.h>    //errno
#include <unistd.h>
#include <sys/wait.h>
#include <time.h>

#include "ngx_func.h"
#include "ngx_macro.h"
#include "ngx_c_conf.h"

#include "ngx_lockfree_threadPool.h"

#include "ngx_mysql_connection.h"

#include "ngx_global.h"



//函数声明
static void ngx_start_worker_processes();
static void ngx_reap_children();
static void ngx_signal_handler(int signo);
// 新封装的函数声明
static void ngx_init_signal_mask();
static void ngx_set_master_process_title();
static void ngx_register_signal_handlers();
static void ngx_init_shared_memory_queues(NetworkToMasterQueue** net_queue,
                                          MasterToMirorProcessQueue** mirrorICP_queue,
                                          MirorProcessToMasterQueue** mirrorICPed_queue,
                                          MasterToResProcessQueue** result_queue,
                                          ResProcessToMasterQueue** resulted_queue,
                                          MasterToPersistProcessQueue** persist_queue,
                                          AsymmProcessToMaterQueue** ToMasterQueue,
                                          MasterToNetworkQueue** ToNetworkQueue);
static int ngx_monitor_queue_load(NetworkToMasterQueue* net_queue,
                                  MasterToMirorProcessQueue* mirrorICP_queue,
                                  MirorProcessToMasterQueue* mirrorICPed_queue,
                                  MasterToResProcessQueue* result_queue,
                                  ResProcessToMasterQueue* resulted_queue,
                                  MasterToPersistProcessQueue* persist_queue,
                                  AsymmProcessToMaterQueue* ToMasterQueue,
                                  MasterToNetworkQueue* ToNetworkQueue,
                                  time_t current_time,
                                  uint64_t transfer_count);
static void ngx_master_main_loop(NetworkToMasterQueue* net_queue,
                                 MasterToMirorProcessQueue* mirrorICP_queue,
                                 MirorProcessToMasterQueue* mirrorICPed_queue,
                                 MasterToResProcessQueue* result_queue,
                                 ResProcessToMasterQueue* resulted_queue,
                                 MasterToPersistProcessQueue* persist_queue,
                                 AsymmProcessToMaterQueue* ToMasterQueue,
                                 MasterToNetworkQueue* ToNetworkQueue);
static bool ngx_process_data_transfer(NetworkToMasterQueue* net_queue,
                                        MasterToMirorProcessQueue* mirrorICP_queue,
                                        MirorProcessToMasterQueue* mirrorICPed_queue,
                                        MasterToResProcessQueue* result_queue,
                                        ResProcessToMasterQueue* resulted_queue,
                                        MasterToPersistProcessQueue* persist_queue,
                                        AsymmProcessToMaterQueue* ToMasterQueue,
                                        MasterToNetworkQueue* ToNetworkQueue,
                                        int load_balance_mode,
                                        uint64_t* transfer_count,
                                        size_t queue_high_threshold);
//网络模块：点云接收
static int ngx_spawn_process(int procNum,const char *pprocName);
static void ngx_network_process_cycle(int inum,const char *pprocName);
static void ngx_network_process_init(int inum);
//镜像/ICP配准模块：点云镜像
static int ngx_mirror_process(int procNum,const char* pprocName);
static void ngx_mirror_process_cycle(int inum,const char* pprocName);
static void ngx_mirror_process_init(int inum);
//不对称度计算模块：计算两点云的不对称度
static int ngx_result_process(int procNum,const char* pprocName);
static void ngx_result_process_cycle(int inum,const char* pprocName);
static void ngx_result_process_init(int inum);
//持久化模块：将点云持久化到文件系统和数据库
static int ngx_persist_process(int procNum,const char* pprocName);
static void ngx_persist_process_cycle(int inum,const char* pprocName);
static void ngx_persist_process_init(int inum);

//std::unique_ptr<draco::PointCloud> decompressPointCloud(char *pPkgBody,uint32_t iBodyLength);
//void saveAsPCD(const draco::PointCloud& draco_cloud, const std::string& filename);

//变量声明
static u_char  master_process[] = "master process";

// 子进程信息结构体
typedef struct {
    pid_t       pid;     // 进程ID
    int         status;  // 进程状态
    time_t      spawn_time; // 创建时间
    int         respawn; // 是否需要重启标志
    const char* name;    // 进程名称
    int         (*spawn_func)(int, const char*); // 创建函数
    int         proc_num; // 进程编号
} ngx_process_t;

// 子进程数组
static ngx_process_t ngx_processes[] = {
    { -1, 0, 0, 1, "network process", ngx_spawn_process, 1 },
    { -1, 0, 0, 1, "mirror process", ngx_mirror_process, 2 },
    { -1, 0, 0, 1, "result process", ngx_result_process, 3 },
    { -1, 0, 0, 1, "persist process", ngx_persist_process, 4 },
    { 0, 0, 0, 0, NULL, NULL, 0 } // 结束标记
};

// 实际定义全局变量
NetworkToMasterQueue* g_net_to_master_queue = nullptr;
MasterToMirorProcessQueue* g_master_to_miror_process_queue = nullptr;
MirorProcessToMasterQueue* g_miror_process_to_master_queue = nullptr;
MasterToResProcessQueue* g_master_to_res_process_queue = nullptr;
ResProcessToMasterQueue* g_res_process_to_master_queue = nullptr;
MasterToPersistProcessQueue* g_master_to_per_process_queue = nullptr;
//返回给客户端的共享内存
AsymmProcessToMaterQueue* g_asymm_process_to_master_queue = nullptr;
MasterToNetworkQueue* g_master_to_network_queue = nullptr;

//描述：创建worker子进程
// 初始化信号屏蔽
static void ngx_init_signal_mask()
{
    sigset_t set; 
    sigemptyset(&set);            //清空信号集

    sigaddset(&set, SIGCHLD);     //子进程状态改变
    sigaddset(&set, SIGALRM);     //定时器超时
    sigaddset(&set, SIGIO);       //异步I/O
    sigaddset(&set, SIGINT);      //终端中断符
    sigaddset(&set, SIGHUP);      //连接断开
    sigaddset(&set, SIGUSR1);     //用户定义信号
    sigaddset(&set, SIGUSR2);     //用户定义信号
    sigaddset(&set, SIGWINCH);    //终端窗口大小改变
    sigaddset(&set, SIGTERM);     //终止
    sigaddset(&set, SIGQUIT);     //终端退出符
    //可添加其他需要阻塞的信号
    
    if (sigprocmask(SIG_BLOCK, &set, NULL) == -1) //第一个参数用了SIG_BLOCK表明设置 进程 新的信号屏蔽字 为 "当前信号屏蔽字 和 第二个参数指向的信号集的并集
    {        
        ngx_log_error_core(NGX_LOG_ALERT,errno,"ngx_init_signal_mask()中sigprocmask()失败!");
    }
    /*
        sigprocmask()原型：
            int sigprocmask(int how, const sigset_t *set, sigset_t *oldset);
            how：SIG_BLOCK、SIG_UNBLOCK、SIG_SETMASK
                    SIG_BLOCK：将set指向的信号集添加到当前信号屏蔽字中
                    SIG_UNBLOCK：将set指向的信号集从当前信号屏蔽字中移除
                    SIG_SETMASK：将当前信号屏蔽字设置为set指向的信号集（完全替换当前信号集）
            set：指向信号集的指针
            oldset：指向旧信号集的指针
    */
}

// 设置进程标题
static void ngx_set_master_process_title()
{
    size_t size;
    int    i;
    size = sizeof(master_process); 
    size += g_argvneedmem;          //argv参数长度加进来    
    if(size < 1000) //长度小于这个，我才设置标题
    {
        char title[1000] = {0};
        strcpy(title,(const char *)master_process); //"master process"
        strcat(title," ");  //跟一个空格分开一些，清晰    //"master process "
        for (i = 0; i < g_os_argc; i++)         //"master process ./nginx"
        {
            strcat(title,g_os_argv[i]);
        }//end for
        ngx_setproctitle(title); //设置标题
        ngx_log_error_core(NGX_LOG_NOTICE,0,"%s %P 【master进程】启动并开始运行......!",title,ngx_pid); //设置标题时顺便记录下来进程名，进程id等信息到日志
    }
}

// 注册信号处理器
//在主进程中，信号处理过程应该后移至子进程创建之后，子进程就不会继承这个过程，避免子进程产生不必要的信号响应
static void ngx_register_signal_handlers()
{
    sigset_t set;
    sigemptyset(&set);
    // 解除信号屏蔽，使主进程能够接收和处理信号
    if (sigprocmask(SIG_SETMASK, &set, NULL) == -1) {
        ngx_log_error_core(NGX_LOG_ALERT, errno, "sigprocmask() failed to unblock signals");
        exit(1);
    }
    struct sigaction sa;

    // 设置信号处理函数
    memset(&sa, 0, sizeof(struct sigaction));
    sa.sa_handler = ngx_signal_handler;
    sigemptyset(&sa.sa_mask);

    //使用 sigaction 系统调用为多个关键信号设置相同的处理方式，注册信号
    //注册 = 告诉系统"这些信号我要自己处理"
    //统一处理 = 所有信号都交给同一个函数处理
    if (sigaction(SIGCHLD, &sa, NULL) == -1 ||//子进程状态改变（退出、停止、继续）
        sigaction(SIGTERM, &sa, NULL) == -1 ||//终止信号（优雅关闭）
        sigaction(SIGQUIT, &sa, NULL) == -1 ||//退出信号（强制关闭），通常由Ctrl+\触发
        sigaction(SIGHUP, &sa, NULL) == -1 ||//挂起信号（通常用于重新加载配置）
        sigaction(SIGINT, &sa, NULL) == -1) {//中断信号（通常由Ctrl+C触发）

        ngx_log_error_core(NGX_LOG_ALERT, errno, "sigaction() failed");
        exit(1);
    }
}
/*
    sigset_t set 变量： 临时容器 ，用于构造信号集。进程信号屏蔽字： 内核维护的状态 ，影响信号传递。
    实际的工作流程：
        构造阶段 ：在局部变量 set 中构造信号集。
        传递阶段 ：通过 sigprocmask()/sigemptyset() 将 set 内容复制到内核。
        传递阶段 ：通过 sigprocmask() 将 set 内容复制到内核。
        销毁阶段 ：局部变量 set 被销毁，但内核状态保持。
    局部变量set只是数据传递的信使，用完即销毁。内核信号屏蔽字才是真正的执行者，持续生效。多次的sigprocmask()
    多个信号的屏蔽字是覆盖，并非累加。
    信号分为标准信号和实时信号：
        标准信号（1-31）：标准信号是不排队的，意味着内核不会为进程记住它收到了多少次某个标准信号，
        它只会记录该信号至少发生了一次。这个“记录”是通过设置进程PCB（进程控制块）中的一个信号位图（Signal Bitmap）的对应位来实现的。
            屏蔽期间给进程发送两个相同信号的场景：
                1.信号被屏蔽（Blocked）
                    假设屏蔽了信号 SIGUSR1（10号信号）。
                    进程使用sigprocmask或pthread_sigmask将SIGUSR1添加到它的信号掩码（Signal Mask）中。
                2.在屏蔽期间，收到两次（或多次）SIGUSR1
                    第一个SIGUSR1到来。内核检查目标进程的信号掩码，发现该信号被阻塞。
                    内核于是将该信号标记为挂起（Pending）。具体做法是在进程的挂起信号位图中将SIGUSR1对应的比特位设置为1。
                    第二个SIGUSR11很快到来。内核再次检查，信号依然被阻塞。
                    内核看到挂起信号位图中SIGUSR1的位已经是1了，于是它什么也不做。这个第二个信号就被丢弃（Discarded） 或说丢失（Lost） 了。
                    结果： 无论在此期间你发送了多少次SIGUSR1，挂起信号集里该信号的标志位始终是1，代表“有一个或多个该信号在等待处理”。
                3.解除信号屏蔽（Unblock）
                    进程从它的信号掩码中移除了SIGUSR1。
                    内核立即检查挂起信号集，发现SIGUSR1的位被置1。
                    因为该信号现在解除了阻塞，并且其处理方式不是忽略（如果是默认或自定义处理程序），内核就会安排一次信号递送（Delivery）。
                    由于挂起位只被设置了一次，所以内核只会安排一次处理程序的调用。
                    用户的信号处理函数（handler）只会被调用一次，尽管之前发送了两次信号。
        实时信号（34-64）具体范围可能因系统和架构略有不同，编程时应使用宏而不是写死数字：实时信号是需要排队的，同种信号的每个实例都会被记录下来，
        并按顺序递送。FIFO（先进先出），顺序保证先发送的信号先被处理。可以携带一个整数型（int）变量和一个指针（void*），传递更多信息。
        主要有应用程序sigqueue()函数产生，用于进程间的通信。
        实时信号的优势：可靠性、有序性、富信息。
            int sigqueue(pid_t pid, int sig, const union sigval value);
                pid: 目标进程ID
                sig: 要发送的实时信号编号（如 SIGRTMIN+5）
                value: 一个联合体，可以传递一个int或一个void *。
            union sigval {
                int   sival_int;    // 传递整型值
                void *sival_ptr;    // 传递指针（需要接收进程能访问该内存）
            };
        设置实时信号的处理函数时，必须使用sigaction()函数，而不能用singnal()
            为了能接收到附加数据，必须使用sigaction结构体中的新标志SA_SIGINFO。
            需要提供一个三参数的信号处理函数。
        处理函数原型不同:
            标准信号处理函数：void handler(int signo);（注：标准信号处理函数也用实时信号处理函数的原型）
            实时信号处理函数：void handler(int signo, siginfo_t *info, void *context);
        关键参数是siginfo_t *info，它是一个包含信号详细信息的结构体，其中就有发送方通过sigqueue()传递过来的数据：
            info->si_value: 发送过来的union sigval值。
            info->si_pid: 发送者的进程ID。
            info->si_code: 信号的来源代码（如SI_QUEUE表示来自sigqueue）。
        实时信号是一个比标准信号强大得多的工具。常见的应用场景包括：自定义的高精度定时器、需要传递状态的事件通知、
        以及任何需要保证事件不丢失且顺序正确的场景。

        linux实时信号如果在进程屏蔽期间发来两个相同的信号，是不是会执行两次信号处理函数
            标准信号：不支持排队，对于同一个标准信号，在信号被屏蔽期间，无论它被发送了多少次，
            内核只会为进程记录一次。该信号的状态被设置为“挂起”，但不会计数。
            当进程解除对该信号的屏蔽后，内核只会递送一次该信号，因此对应的信号处理函数也只会执行一次。
            实时信号：支持排队，这是实时信号与标准信号的主要区别之一。如果同一个实时信号在屏蔽期间被发送了多次，
            内核会为每一次发送都创建一个独立的记录，并将它们按顺序放入队列。
            当进程解除对该信号的屏蔽后，内核会按照队列顺序，依次递送每一个挂起的信号。因此，如果发送了两次，信号处理函数就会执行两次。



*/
/*
    SIGTERM -终止信号（优雅关闭）
        含义：请求程序正常终止的信号
        用途：允许程序执行清理工作后优雅退出。
        触发方式 ： kill <pid> 或 systemctl stop 等命令。
        特点：可以捕获和处理，程序可以选择如何响应。
    SIGQUIT -退出信号（强制关闭）
        含义：请求程序退出并生成核心转储文件。
        用途：用于强制终止程序，通常在调试时使用。
        触发方式 ： Ctrl+\ 或 kill -QUIT <pid>
        特点：比SIGTERM更强制，但仍可以捕获和处理。
    SIGHUP -挂起信号（通常用于重新加载配置）
        含义：请求程序重新加载配置文件。
        用途：用于在不终止程序的情况下重新加载配置。
        触发方式 ： 发送 SIGHUP 信号到进程 PID。kill -HUP <pid> 或终端断开连接。
        应用场景：配置热重载，日志轮转等。
    SIGINT -中断信号（通常由Ctrl+C触发）优雅的中断
        含义：请求程序中断当前执行。
        用途：用于中断正在运行的程序。
        触发方式 ： 按下 Ctrl+C 组合键。
    SIGKILL -终止信号（强制关闭）
        含义：请求程序立即终止，不生成核心转储文件。
        用途：用于强制终止程序，通常在调试时使用。
        触发方式 ： 发送 SIGKILL 信号到进程 PID。kill -9 <pid>
        与前面所述信号不同的是：无法捕获和处理 ：与前面提到的其他信号不同，SIGKILL 信号无法被程序捕获、阻塞或忽略。
        立即终止 ：操作系统会立即强制终止目标进程，不给程序任何清理机会。
        最强制的信号 ：这是最暴力的进程终止方式，无优雅退出，程序无法执行任何清理工作，可能导致数据丢失和内存泄漏。
    SIGSTOP -停止信号（强制暂停进程）
        含义：请求程序暂停执行。
        用途：用于调试和分析。
        触发方式 ： 发送 SIGSTOP 信号到进程 PID。kill -STOP <pid>
        特点：SIGSTOP信号无法被捕获和处理，也不能被阻塞。
        暂停进程的含义：
            暂时停止执行，但保持其状态以备后续恢复
            停止CPU执行：内核立即剥夺该进程的CPU时间片，不再调度它运行。该进程的代码会冻结在它被中断的那一条机器指令处。
            保持进程状态：这是最关键的一点。进程的整个运行状态都被原封不动地保留在内存中：
                内存数据：堆(heap)、栈(stack)、全局变量等所有数据都保持不变。
                执行上下文：程序计数器(PC，即下一条要执行的指令地址)、寄存器中的值、打开的文件描述符、网络连接等全部被冻结保存。
                内核数据结构：进程控制块(PCB/Task Struct)依然存在，只是其状态被标记为 TASK_STOPPED。
            释放部分资源：虽然状态被保留，但进程占用的CPU会被立即释放，供其他就绪的进程使用。它占用的内存不会被释放。
    被暂停的进程可以通过接收 SIGCONT (Continue) 信号来恢复运行。
        触发方式 ： 发送 SIGCONT 信号到进程 PID。kill -CONT <pid>
        内核收到 SIGCONT 后，会将进程状态从 TASK_STOPPED 改为 TASK_RUNNING。
        进程会被重新放入操作系统的运行队列，等待CPU调度。
        当它再次被调度到CPU上时，内核会精确地恢复其之前保存的所有上下文（寄存器、程序计数器等）。
        进程会从当初被暂停的那条指令之后的一条指令开始，毫不知情地继续执行，就像什么都没有发生过一样。

    默认的kill指令为SIGTERM，即kill (-15) <PID>

    struct sigaction {
        void (*sa_handler)(int);                        // 信号处理函数指针
        void (*sa_sigaction)(int, siginfo_t *, void *); // 扩展信号处理函数指针
        sigset_t sa_mask;                               // 信号屏蔽集
        int sa_flags;                                   // 信号处理标志
        void (*sa_restorer)(void);                      // 已废弃，用于兼容性
    };
    若采用扩展信号处理函数，应将sa_flags设置为SA_SIGINFO，告诉内核使用三参数的信号处理函数
*/

// 初始化共享内存队列
static void ngx_init_shared_memory_queues(NetworkToMasterQueue** net_queue,
                                          MasterToMirorProcessQueue** mirrorICP_queue,
                                          MirorProcessToMasterQueue** mirrorICPed_queue,
                                          MasterToResProcessQueue** result_queue,
                                          ResProcessToMasterQueue** resulted_queue,
                                          MasterToPersistProcessQueue** persist_queue,
                                          AsymmProcessToMaterQueue** ToMasterQueue,
                                          MasterToNetworkQueue** ToNetworkQueue)
{
    // 创建网络到master的共享内存
    *net_queue = open_shm_queue<NetworkToMasterQueue>(NETWORK_TO_MASTER_SHM);
    // 创建master到镜像/ICP处理的共享内存
    *mirrorICP_queue = open_shm_queue<MasterToMirorProcessQueue>(MASTER_TO_MIRROR_PROCESS_SHM);
    // 创建镜像/ICP处理到master的共享内存
    *mirrorICPed_queue = open_shm_queue<MirorProcessToMasterQueue>(MIRROR_PROCESS_TO_MASTER_SHM);
    //创建master到结果处理的共享内存
    *result_queue = open_shm_queue<MasterToResProcessQueue>(MASTER_TO_RESULT_PROCESS_SHM);
    //创建不对称度计算进程到master进程的共享内存
    *resulted_queue = open_shm_queue<ResProcessToMasterQueue>(RESULT_PROCESS_TO_MASTER_SHM);
    //创建master进程到持久化进程的共享内存
    *persist_queue = open_shm_queue<MasterToPersistProcessQueue>(MASTER_TO_PERSIST_PROCESS_SHM);

    //创建返回结果的共享内存
    *ToMasterQueue = open_shm_queue<AsymmProcessToMaterQueue>(RETURN_TO_MASTER_SHM);
    *ToNetworkQueue = open_shm_queue<MasterToNetworkQueue>(RETURN_TO_NETWORK_SHM);
}

void ngx_master_process_cycle()
{   
    // 初始化信号屏蔽
    ngx_init_signal_mask();
    /*
        信号屏蔽字（Signal Mask）：
        信号屏蔽字是一个信号集合，用于指示当前进程在执行过程中哪些信号是被屏蔽的，即暂时不处理的信号。
        当一个信号被添加到信号屏蔽字中时，该信号就会被阻塞，直到从信号屏蔽字中移除。
        信号屏蔽字的作用是保护进程在执行关键代码时不被中断，确保关键代码的执行完整性。
    */

    // 设置进程标题
    ngx_set_master_process_title();
    
    ngx_start_worker_processes();  //子进程创建应该迁移到共享内存创建之前

    //创建子进程后，父进程的执行流程会走到到这里，子进程不会走进来
    // 注册信号处理器
    ngx_register_signal_handlers();
    
    // 初始化共享内存队列
    NetworkToMasterQueue* net_queue;
    MasterToMirorProcessQueue* mirrorICP_queue;
    MirorProcessToMasterQueue* mirrorICPed_queue;
    MasterToResProcessQueue* result_queue;
    ResProcessToMasterQueue* resulted_queue;
    MasterToPersistProcessQueue* persist_queue;
    AsymmProcessToMaterQueue* ToMasterQueue;
    MasterToNetworkQueue* ToNetworkQueue;
    
    ngx_init_shared_memory_queues(&net_queue, &mirrorICP_queue, &mirrorICPed_queue, 
                                  &result_queue, &resulted_queue, &persist_queue,
                                  &ToMasterQueue, &ToNetworkQueue);

    // 开始主循环
    ngx_master_main_loop(net_queue, mirrorICP_queue, mirrorICPed_queue,
                         result_queue, resulted_queue, persist_queue,
                         ToMasterQueue, ToNetworkQueue);
     return;
}

// 队列负载监控和负载均衡模式调整
static int ngx_monitor_queue_load(NetworkToMasterQueue* net_queue,
                                  MasterToMirorProcessQueue* mirrorICP_queue,
                                  MirorProcessToMasterQueue* mirrorICPed_queue,
                                  MasterToResProcessQueue* result_queue,
                                  ResProcessToMasterQueue* resulted_queue,
                                  MasterToPersistProcessQueue* persist_queue,
                                  AsymmProcessToMaterQueue* ToMasterQueue,
                                  MasterToNetworkQueue* ToNetworkQueue,
                                  time_t current_time,
                                  uint64_t transfer_count)
{
    static time_t last_queue_monitor = 0;
    static int load_balance_mode = 0; // 0=正常模式, 1=高负载模式, 2=低负载模式
    const time_t QUEUE_MONITOR_INTERVAL = 2; // 每2秒监控一次队列状态
    const size_t QUEUE_HIGH_THRESHOLD = 24; // 队列高负载阈值(75%容量)
    const size_t QUEUE_LOW_THRESHOLD = 8;   // 队列低负载阈值(25%容量)
    const uint64_t LOG_INTERVAL = 1000; // 每1000次传输记录一次日志
    
    if (current_time - last_queue_monitor >= QUEUE_MONITOR_INTERVAL) {
        size_t net_size = net_queue->size();
        size_t mirror_size = mirrorICP_queue->size();
        size_t mirrored_size = mirrorICPed_queue->size();
        size_t result_size = result_queue->size();
        size_t resulted_size = resulted_queue->size();
        size_t persist_size = persist_queue->size();
        size_t tomaster_size = ToMasterQueue->size();
        size_t tonetwork_size = ToNetworkQueue->size();
        
        // 计算总体负载情况
        size_t total_load = net_size + mirror_size + mirrored_size + result_size + 
                           resulted_size + persist_size + tomaster_size + tonetwork_size;
        size_t avg_load = total_load / 8;
        
        // 动态调整负载均衡模式
        int old_mode = load_balance_mode;
        if (avg_load >= QUEUE_HIGH_THRESHOLD) {
            load_balance_mode = 1; // 高负载模式
        } else if (avg_load <= QUEUE_LOW_THRESHOLD) {
            load_balance_mode = 2; // 低负载模式
        } else {
            load_balance_mode = 0; // 正常模式
        }
        
        // 记录负载状态变化
        if (old_mode != load_balance_mode) {
            const char* mode_names[] = {"正常", "高负载", "低负载"};
            // ngx_log_stderr(0, "负载均衡模式切换: %s -> %s (平均队列长度: %zu)", 
            //               mode_names[old_mode], mode_names[load_balance_mode], avg_load);
        }
        
        // 详细队列状态日志（仅在调试时启用）
        if (transfer_count % (LOG_INTERVAL * 5) == 0) {
            // ngx_log_stderr(0, "队列状态监控 - net:%zu, mirror:%zu->%zu, result:%zu->%zu, persist:%zu, return:%zu->%zu",
            //               net_size, mirror_size, mirrored_size, result_size, resulted_size, 
            //               persist_size, tomaster_size, tonetwork_size);
        }
        
        last_queue_monitor = current_time;//这是一个静态变量，所以每次调用都要更新

    }
    
    return load_balance_mode;
}

// 主循环函数
static void ngx_master_main_loop(NetworkToMasterQueue* net_queue,
                                 MasterToMirorProcessQueue* mirrorICP_queue,
                                 MirorProcessToMasterQueue* mirrorICPed_queue,
                                 MasterToResProcessQueue* result_queue,
                                 ResProcessToMasterQueue* resulted_queue,
                                 MasterToPersistProcessQueue* persist_queue,
                                 AsymmProcessToMaterQueue* ToMasterQueue,
                                 MasterToNetworkQueue* ToNetworkQueue)
{
    // 性能优化变量
    static time_t last_process_check = 0;
    static uint64_t transfer_count = 0;
    const time_t PROCESS_CHECK_INTERVAL = 5; // 每5秒检查一次进程状态
    const uint64_t LOG_INTERVAL = 1000; // 每1000次传输记录一次日志
    const size_t QUEUE_HIGH_THRESHOLD = 24; // 队列高负载阈值(75%容量)
    
    for ( ;; ) 
    {
        bool has_activity = false;
        time_t current_time = time(NULL);
        
        // 队列负载监控和负载均衡模式调整
        int load_balance_mode = ngx_monitor_queue_load(net_queue, mirrorICP_queue, mirrorICPed_queue,
                                                       result_queue, resulted_queue, persist_queue,
                                                       ToMasterQueue, ToNetworkQueue,
                                                       current_time, transfer_count);
        
        // 优化：减少进程检查频率，避免每次循环都检查
        if (current_time - last_process_check >= PROCESS_CHECK_INTERVAL) {
            // 收割已退出的子进程
            ngx_reap_children();
            
            // 检查并重启需要重启的进程
            for (ngx_process_t *proc = ngx_processes; proc->name != NULL; proc++) {
                if (proc->pid == -1 && proc->respawn) {
                    ngx_log_error_core(NGX_LOG_NOTICE, 0, "重启 %s 进程...", proc->name);
                    proc->pid = proc->spawn_func(proc->proc_num, proc->name);
                    proc->spawn_time = current_time;
                }
            }
            last_process_check = current_time;
        }

        // 基于队列长度的智能数据转发逻辑
        has_activity = ngx_process_data_transfer(net_queue, mirrorICP_queue, mirrorICPed_queue,
                                                 result_queue, resulted_queue, persist_queue,
                                                 ToMasterQueue, ToNetworkQueue,
                                                 load_balance_mode, &transfer_count, QUEUE_HIGH_THRESHOLD);

        // 优化的日志记录：减少频繁输出
        if (transfer_count > 0 && transfer_count % LOG_INTERVAL == 0) {
            // ngx_log_stderr(0, "数据转发统计: 已处理 %lu 次传输 (负载模式: %d)", transfer_count, load_balance_mode);
        }

        // 基于负载均衡模式的动态休眠策略
        int sleep_time_active, sleep_time_idle;
        switch (load_balance_mode) {
            case 1: // 高负载模式：减少休眠时间，提高响应速度
                sleep_time_active = 500;   // 有活动时极短休眠
                sleep_time_idle = 2000;    // 无活动时短休眠
                break;
            case 2: // 低负载模式：增加休眠时间，节约CPU
                sleep_time_active = 2000;  // 有活动时适中休眠
                sleep_time_idle = 10000;   // 无活动时长休眠
                break;
            default: // 正常模式
                sleep_time_active = 1000;  // 有活动时短暂休眠
                sleep_time_idle = 5000;    // 无活动时稍长休眠
                break;
        }
        
        if (has_activity) {
            usleep(sleep_time_active);
        } else {
            usleep(sleep_time_idle);
        }
    }// end for(;;)
     return;
}

// 收割子进程
static void ngx_reap_children() {
    int status;
    pid_t pid;

    while ((pid = waitpid(-1, &status, WNOHANG)) > 0) {
        for (ngx_process_t *proc = ngx_processes; proc->name != NULL; proc++) {
            if (proc->pid == pid) {
                ngx_log_error_core(NGX_LOG_NOTICE, 0, "%s 进程 %P 退出, 状态: %d", 
                                  proc->name, pid, status);
                
                // 标记进程为已退出
                proc->pid = -1;
                
                // 如果不是正常退出，设置需要重启
                if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
                    proc->respawn = 1;
                }
                //WIFEXITED(status) 检查进程是否通过 exit() 或从 main() 返回而正常退出
                //WEXITSTATUS(status) 获取进程的退出码
                //在Linux中退出码为0表示成功，非零表示错误

                break;
            }
        }
    }

    if (pid == -1 && errno != ECHILD) {
        ngx_log_error_core(NGX_LOG_ALERT, errno, "waitpid() failed");
    }
}
/*
    waitpid()函数：等待子进程结束
    函数原型：pid_t waitpid(pid_t pid, int *status, int options);
    参数：
        pid：要等待的子进程ID，-1表示等待任意子进程
            pid>0：等待指定ID的子进程
            pid=-1：等待任意子进程
            pid<0：等待进程组ID为pid的任意子进程
        status：子进程的退出状态，通过指针返回
            指向int的指针，用于存储子进程的退出状态
            可以为NULL，不关心子进程的退出状态
        options：选项，WNOHANG表示非阻塞等待，WUNTRACED表示等待停止的子进程
            0：阻塞等待，直到子进程退出
            WNOHANG：非阻塞等待，立即返回，无论子进程是否退出
            WUNTRACED：等待停止的子进程，也会返回
        status参数实际是表示子进程因收到什么信号而退出的
        WIFEXITED(status)    // 正常退出？
            非0：正常退出
            0：异常退出
        WEXITSTATUS(status)  // 获取退出码
            正常退出：返回子进程的退出码
            异常退出：返回0
        WIFSIGNALED(status)  // 被信号终止？
            非0：被信号终止
            0：正常退出
        WTERMSIG(status)     // 获取终止信号
            被信号终止：返回终止信号的编号
            正常退出：返回0
        WIFSTOPPED(status)   // 被停止？
            非0：被停止
            0：正常退出
        WSTOPSIG(status)     // 获取停止信号
            被停止：返回停止信号的编号
            正常退出：返回0

    返回值：
        成功：子进程ID
        失败：-1

    子进程退出时，内核会发送给父进程通知，进程状态：子进程的退出信息（退出码、终止信号等），由内核保存
    僵尸进程产生 ：子进程退出后，内核保留其进程控制块(PCB)和退出状态
    waitpid清理 ：父进程调用 waitpid 后，内核释放子进程的PCB，僵尸进程彻底消失
    进程号回收 ：子进程的PID被标记为可重用，系统可以分配给新进程
    调用waitpid后的变化：
        pid_t pid = waitpid(-1, &status, WNOHANG);
            // 此时发生：
            // 1. 内核将退出状态复制到status变量
            // 2. 立即释放子进程的PCB
            // 3. 从进程表中移除该进程条目
            // 4. 进程ID标记为可重用
            // 5. 僵尸进程彻底消失
    waitpid:从内核获取已退出的子进程状态信息
    子进程退出时，内核持久保存退出状态，SIGCHLD信号只是通知，不携带状态信息。waitpid从内核获取保存的状态，清理僵尸进程
    需要注意的是，状态获取是原子操作。无法对同一个子进程调用waitpid，因为在第一次调用的时候，内核已经将所有资源都释放

    僵尸进程一直不清理会发生什么：
        直接后果：资源泄露（有限的资源被永久占用）
            占用进程PID：
                操作系统中的进程号是有限的资源（例如，在Linux中，可以通过 /proc/sys/kernel/pid_max 查看最大值，通常是32768）。
                每个僵尸进程都会继续占用一个PID号。如果父进程持续产生子进程且从不回收，最终系统的可用PID号会被耗尽。
                当PID号被耗尽时，系统将无法创建任何新的进程。这意味着你无法执行新的命令（ls, bash等），无法启动新的服务，甚至可能导致系统部分功能瘫痪。
            占用内核进程表槽位：
                操作系统内核维护着一张进程表，其中每个活跃的或僵尸进程都占据一个条目。
                这张表的大小也是有限的（通常与最大PID数相关）。僵尸进程会占据这些宝贵的槽位。
                如果进程表被僵尸进程填满，系统和用户同样将无法创建任何新的进程。
        重要澄清：僵尸进程并不消耗其他资源
            内存：僵尸进程的用户空间内存（代码、数据段、堆栈等） 已经在子进程调用 exit() 终止时被操作系统回收和释放了。
            CPU：僵尸进程不是一个可执行的实体，它只是一个残留的数据结构（PCB）。它不会被调度，因此完全不消耗CPU资源。
*/

// 信号处理函数
static void ngx_signal_handler(int signo) {
    switch (signo) {
    case SIGCHLD:
        // 子进程状态变化，在主循环中处理
        break;
        
    case SIGTERM:
    case SIGQUIT:
    case SIGINT:
        // 退出信号，终止所有子进程
        ngx_log_error_core(NGX_LOG_NOTICE, 0, "收到终止信号 %d，开始关闭所有子进程...", signo);
        
        for (ngx_process_t *proc = ngx_processes; proc->name != NULL; proc++) {
            if (proc->pid > 0) {
                kill(proc->pid, SIGTERM);
            }
        }
        
        // 等待所有子进程退出
        int live_children;
        do {
            live_children = 0;
            for (ngx_process_t *proc = ngx_processes; proc->name != NULL; proc++) {
                if (proc->pid > 0) {
                    live_children = 1;
                    break;
                }
            }
            
            if (live_children) {
                sleep(1);
                ngx_reap_children();
            }
        } while (live_children);
        /*
            为什么需要后续等待子进程退出的步骤：
            信号是异步的：
                kill(proc->pid, SIGTERM) 只是发送信号，不会等待进程实际退出
                子进程收到SIGTERM后需要时间来清理资源、保存数据、关闭连接等
                发送信号后立即返回，此时子进程可能仍在运行
            避免僵尸进程：
                如果父进程不等待子进程退出就直接exit，会产生僵尸进程
                僵尸进程会占用系统资源（进程表项），直到系统重启
            确保优雅退出：
                等待机制给子进程充分时间完成：
                    处理完当前请求
                    保存重要数据到数据库
                    关闭网络连接
                    释放共享内存
                防止系统进程表溢出
                确保所有资源正确释放
        */
        
        ngx_log_error_core(NGX_LOG_NOTICE, 0, "所有子进程已关闭，主进程退出");
        exit(0);
        
    case SIGHUP:
        // 重新加载配置
        ngx_log_error_core(NGX_LOG_NOTICE, 0, "收到重新加载信号");
        // TODO: 实现配置重载逻辑
        break;
        
    default:
        break;
    }
}

// 数据转发处理函数
static bool ngx_process_data_transfer(NetworkToMasterQueue* net_queue,
                                        MasterToMirorProcessQueue* mirrorICP_queue,
                                        MirorProcessToMasterQueue* mirrorICPed_queue,
                                        MasterToResProcessQueue* result_queue,
                                        ResProcessToMasterQueue* resulted_queue,
                                        MasterToPersistProcessQueue* persist_queue,
                                        AsymmProcessToMaterQueue* ToMasterQueue,
                                        MasterToNetworkQueue* ToNetworkQueue,
                                        int load_balance_mode,
                                        uint64_t* transfer_count,
                                        size_t queue_high_threshold)
{
    bool has_activity = false;
    
    // 根据负载均衡模式调整处理策略
    int batch_size = 1;
    int max_retries = 10;
    int base_delay = 100;
    
    switch (load_balance_mode) {
        case 1: // 高负载模式：批量处理，减少延迟
            batch_size = 3;
            max_retries = 5;
            base_delay = 50;
            break;
        case 2: // 低负载模式：节能处理，增加延迟
            batch_size = 1;
            max_retries = 15;
            base_delay = 200;
            break;
        default: // 正常模式
            batch_size = 1;
            max_retries = 10;
            base_delay = 100;
            break;
    }
    
    // 第一阶段：网络 -> 镜像/ICP处理（支持批量处理）
    for (int i = 0; i < batch_size; i++) {
        PointCloud cloud;
        if (net_queue->try_pop(cloud)) {
            // 检查目标队列负载，如果过载则跳过
            if (mirrorICP_queue->size() >= queue_high_threshold) {
                // 队列过载，暂时跳过这个数据项
                net_queue->try_push(std::move(cloud)); // 放回队列
                break;
            }
            
            // 使用动态退避策略等待队列可用，指数级退避
            int retry_count = 0;
            while (!mirrorICP_queue->try_push(std::move(cloud)) && retry_count < max_retries) {
                usleep(base_delay * (1 << (retry_count / 3))); // 更平缓的指数退避
                retry_count++;
            }
            if (retry_count < max_retries) {
                has_activity = true;
                (*transfer_count)++;
            }
        } else {
            break; // 没有更多数据
        }
        /*
            动态指数退避策略：
                延迟时间 = base_delay × 2^(retry_count / 3) 每三次重试才翻倍
                传统的指数退避：增长过快 ：延迟时间快速增长到不可接受的程度
                             资源浪费 ：长时间的等待可能导致资源闲置
                             响应性差 ：在系统恢复后需要很长时间才能重新尝试
        */
    }
    
    // 第二阶段：镜像/ICP处理 -> 结果处理（支持批量处理）
    for (int i = 0; i < batch_size; i++) {
        MirrorICPPointCloud ICPPointCloud;
        if (mirrorICPed_queue->try_pop(ICPPointCloud)) {
            // 检查目标队列负载
            if (result_queue->size() >= queue_high_threshold) {
                mirrorICPed_queue->try_push(std::move(ICPPointCloud)); // 放回队列
                break;
            }
            
            int retry_count = 0;
            while (!result_queue->try_push(std::move(ICPPointCloud)) && retry_count < max_retries) {
                usleep(base_delay * (1 << (retry_count / 3)));
                retry_count++;
            }
            if (retry_count < max_retries) {
                has_activity = true;
                (*transfer_count)++;
            }
        } else {
            break;
        }
    }
    
    // 第三阶段：结果处理 -> 持久化（支持批量处理）
    for (int i = 0; i < batch_size; i++) {
        ResPointCloud res;
        if (resulted_queue->try_pop(res)) {
            // 检查目标队列负载
            if (persist_queue->size() >= queue_high_threshold) {
                resulted_queue->try_push(std::move(res)); // 放回队列
                break;
            }
            
            int retry_count = 0;
            while (!persist_queue->try_push(std::move(res)) && retry_count < max_retries) {
                usleep(base_delay * (1 << (retry_count / 3)));
                retry_count++;
            }
            if (retry_count < max_retries) {
                has_activity = true;
                (*transfer_count)++;
            }
        } else {
            break;
        }
    }
    
    // 第四阶段：不对称度计算结果 -> 网络（支持批量处理）
    for (int i = 0; i < batch_size; i++) {
        ResToNetwork ans;
        if (ToMasterQueue->try_pop(ans)) {
            // 检查目标队列负载
            if (ToNetworkQueue->size() >= queue_high_threshold) {
                ToMasterQueue->try_push(std::move(ans)); // 放回队列
                break;
            }
            
            int retry_count = 0;
            while (!ToNetworkQueue->try_push(std::move(ans)) && retry_count < max_retries) {
                usleep(base_delay * (1 << (retry_count / 3)));
                retry_count++;
            }
            if (retry_count < max_retries) {
                has_activity = true;
                (*transfer_count)++;
            }
        } else {
            break;
        }
    }
    
    return has_activity;
}

//threadnums:要创建的子进程数量
static void ngx_start_worker_processes()
{
    for (ngx_process_t *proc = ngx_processes; proc->name != NULL; proc++) {
        proc->pid = proc->spawn_func(proc->proc_num, proc->name);
        proc->spawn_time = time(NULL);//创建时间
    }
    return;
}

//描述：产生一个子进程
//inum：进程编号【0开始】
//pprocName：子进程名字"worker process"
static int ngx_spawn_process(int procNum,const char *pprocName)
{
    pid_t  pid;

    pid = fork(); //fork()系统调用产生子进程
    switch (pid)  //pid判断父子进程，分支处理
    {  
    case -1: //产生子进程失败
        ngx_log_error_core(NGX_LOG_ALERT,errno,"ngx_spawn_process()fork()产生子进程num=%d,procname=\"%s\"失败!",procNum,pprocName);
        return -1;

    case 0:  //子进程分支
        ngx_parent = ngx_pid;              //因为是子进程了，所有原来的pid变成了父pid
        ngx_pid = getpid();                //重新获取pid,即本子进程的pid
        ngx_network_process_cycle(procNum,pprocName);    //我希望所有worker子进程，在这个函数里不断循环着不出来，也就是说，子进程流程不往下边走;
        break;

    default: //这个应该是父进程分支，直接break;，流程往switch之后走            
        break;
    }//end switch

    //父进程分支会走到这里，子进程流程不往下边走-------------------------
    //若有需要，以后再扩展增加其他代码......
    return pid;
}

static void ngx_network_process_cycle(int inum,const char *pprocName) 
{
    //设置一下变量
    ngx_process = NGX_PROCESS_WORKER;  //设置进程的类型，是worker进程

    g_master_to_network_queue = open_shm_queue<MasterToNetworkQueue>(RETURN_TO_NETWORK_SHM);//这段共享内存的初始化一定要提前到线程初始化之前
    ngx_network_process_init(inum);
    Connection conn;//mysql_init()并不是线程安全，在程序初始化的时候调用一次，之后线程安全
    ngx_setproctitle(pprocName); //设置标题   
    ngx_log_error_core(NGX_LOG_NOTICE,0,"%s %P 【network进程】启动并开始运行......!",pprocName,ngx_pid); //设置标题时顺便记录下来进程名，进程id等信息到日志
    g_net_to_master_queue = open_shm_queue<NetworkToMasterQueue>(NETWORK_TO_MASTER_SHM);
    for(;;)
    {
        ngx_process_events_and_timers(); //处理网络事件和定时器事件
        //这个函数实际是调用epoll_wait()函数，等待事件发生，有事件发生才会返回，没有事件发生会阻塞等待
        //循环阻塞，每轮循环都在epoll_wait(-1) 处阻塞
        //事件驱动，有事件发生才会返回，没有事件发生会阻塞等待
        //高效节能，没有事件时，进程休眠，不消耗CPU资源
        //批量处理，一次处理多个事件，而不是一个一个处理
        //循环继续，处理完事件后回到循环开始处，继续等待事件发生
    } //end for(;;)

    //如果从这个循环跳出来
    g_threadpool.StopAll();      //考虑在这里停止线程池
    g_socket.Shutdown_subproc(); //socket需要释放的东西考虑释放
    return;
}

//描述：子进程创建时调用本函数进行一些初始化工作
static void ngx_network_process_init(int inum)
{
    sigset_t  set;      //信号集

    sigemptyset(&set);  //清空信号集
    if (sigprocmask(SIG_SETMASK, &set, NULL) == -1)  //原来是屏蔽那10个信号【防止fork()期间收到信号导致混乱】，现在不再屏蔽任何信号【接收任何信号】
    {
        ngx_log_error_core(NGX_LOG_ALERT,errno,"ngx_network_process_init()中sigprocmask()失败!");
    }
    if(g_socket.Initialize() == false)
    {
        ngx_log_error_core(NGX_LOG_ALERT,errno,"ngx_network_process_init()中g_socket.Initialize()失败!");
    }

    //线程池代码，率先创建，至少要比和socket相关的内容优先
    CConfig *p_config = CConfig::GetInstance();
    int tmpthreadnums = p_config->GetIntDefault("ProcMsgRecvWorkThreadCount",5); //处理接收到的消息的线程池中线程数量
    if(g_threadpool.Create(tmpthreadnums) == false)  //创建线程池中线程
    {
        //内存没释放，但是简单粗暴退出；
        exit(-2);
    }
    sleep(1); //再休息1秒；

    if(g_socket.Initialize_subproc() == false) //初始化子进程需要具备的一些多线程能力相关的信息
    {
        //内存没释放，但是简单粗暴退出；
        exit(-2);
    }
    
    g_socket.ngx_epoll_init();           //初始化epoll相关内容，同时 往监听socket上增加监听事件，从而开始让监听端口履行其职责

    return;
}

static int ngx_mirror_process(int procNum,const char* pprocName){
    pid_t pid;

    pid=fork();
    switch(pid)
    {
        case -1:
            ngx_log_error_core(NGX_LOG_ALERT,errno,"ngx_mirror_process()fork()产生子进程num=%d,procname=\"%s\"失败!",procNum,pprocName);  
            return -1;
        case 0:
            ngx_parent=ngx_pid;
            ngx_pid=getpid();
            ngx_mirror_process_cycle(procNum,pprocName);
            break;
        default:
            break;
    }
    return pid;
}

static void ngx_mirror_process_cycle(int inum,const char* pprocName){
    ngx_process = NGX_PROCESS_WORKER;

    ngx_mirror_process_init(inum);
    ngx_setproctitle(pprocName);
    ngx_log_error_core(NGX_LOG_NOTICE,0,"%s %P 【mirrorwork进程】启动并开始运行......!",pprocName,ngx_pid);
    g_master_to_miror_process_queue = open_shm_queue<MasterToMirorProcessQueue>(MASTER_TO_MIRROR_PROCESS_SHM);
    g_miror_process_to_master_queue = open_shm_queue<MirorProcessToMasterQueue>(MIRROR_PROCESS_TO_MASTER_SHM);

    static MirrorICPProcessingPool processing(2,*g_master_to_miror_process_queue,*g_miror_process_to_master_queue);
    for(;;){
        sleep(1000);
    }
    return;
}

static void ngx_mirror_process_init(int inum){
    sigset_t  set;      //信号集

    sigemptyset(&set);  //清空信号集
    if (sigprocmask(SIG_SETMASK, &set, NULL) == -1)
    {
        ngx_log_error_core(NGX_LOG_ALERT,errno,"ngx_mirror_process_init()中sigprocmask()失败!");
    }
}

static int ngx_result_process(int procNum,const char* pprocName){
    pid_t pid;
    pid = fork();
    switch(pid){
        case -1:
             ngx_log_error_core(NGX_LOG_ALERT,errno,"ngx_result_process()fork()产生子进程num=%d,procname=\"%s\"失败!",procNum,pprocName);
             return -1;
        case 0:
             ngx_parent = ngx_pid;
             ngx_pid = getpid();
             ngx_result_process_cycle(procNum,pprocName);
             break;
        default:
             break;
    }
    return pid;
}

static void ngx_result_process_cycle(int inum,const char* pprocName){
    ngx_process = NGX_PROCESS_WORKER;

    ngx_result_process_init(inum);
    ngx_setproctitle(pprocName);
    ngx_log_error_core(NGX_LOG_NOTICE,0,"%s %P 【resultwork进程】启动并开始运行......!",pprocName,ngx_pid);
    g_master_to_res_process_queue = open_shm_queue<MasterToResProcessQueue>(MASTER_TO_RESULT_PROCESS_SHM);
    g_res_process_to_master_queue = open_shm_queue<ResProcessToMasterQueue>(RESULT_PROCESS_TO_MASTER_SHM);
    g_asymm_process_to_master_queue = open_shm_queue<AsymmProcessToMaterQueue>(RETURN_TO_MASTER_SHM);
    static ResultProcessingPool processing(2,*g_master_to_res_process_queue,*g_res_process_to_master_queue);
    for(;;){
        sleep(1000);
    }
}

static void ngx_result_process_init(int inum){
    sigset_t  set;      //信号集

    sigemptyset(&set);  //清空信号集
    if (sigprocmask(SIG_SETMASK, &set, NULL) == -1)
    {
        ngx_log_error_core(NGX_LOG_ALERT,errno,"ngx_mirror_process_init()中sigprocmask()失败!");
    }
}

static int ngx_persist_process(int procNum,const char* pprocName){
    pid_t pid;
    pid = fork();
    switch(pid){
        case -1:
            ngx_log_error_core(NGX_LOG_ALERT,errno,"ngx_persist_process()fork()产生子进程num=%d,procname=\"%s\"失败!",procNum,pprocName);
            return -1;
        case 0:
            ngx_parent = ngx_pid;
            ngx_pid = getpid();
            ngx_persist_process_cycle(procNum,pprocName);
            break;
        default:
            break;
    }
    return pid;
}

static void ngx_persist_process_cycle(int inum,const char* pprocName){
    ngx_process = NGX_PROCESS_WORKER;

    ngx_persist_process_init(inum);
    ngx_setproctitle(pprocName);
    ngx_log_error_core(NGX_LOG_NOTICE,0,"%s %P 【persistwork进程】启动并开始运行......!",pprocName,ngx_pid);
    g_master_to_per_process_queue = open_shm_queue<MasterToPersistProcessQueue>(MASTER_TO_PERSIST_PROCESS_SHM);
    Connection conn;//mysql_init()并不是线程安全，在程序初始化的时候调用一次，之后线程安全
    static PersistProcessingPool processing(2,*g_master_to_per_process_queue);
    for(;;){
        sleep(1000);
    }
    return;
}

static void ngx_persist_process_init(int inum){
    sigset_t  set;      //信号集

    sigemptyset(&set);  //清空信号集
    if (sigprocmask(SIG_SETMASK, &set, NULL) == -1)
    {
        ngx_log_error_core(NGX_LOG_ALERT,errno,"ngx_persist_process_init()中sigprocmask()失败!");
    }
}