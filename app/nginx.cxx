#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h> 
#include <errno.h>
#include <arpa/inet.h>
#include <sys/time.h>          //gettimeofday

#include "ngx_macro.h"         //各种宏定义
#include "ngx_func.h"          //各种函数声明
#include "ngx_c_conf.h"        //和配置文件处理相关的类,名字带c_表示和类有关
#include "ngx_c_socket.h"      //和socket通讯相关
#include "ngx_c_memory.h"      //和内存分配释放等相关
#include "ngx_c_threadpool.h"  //和多线程有关
#include "ngx_c_crc32.h"       //和crc32校验算法有关
#include "ngx_c_slogic.h"      //和socket通讯相关

//本文件用的函数声明
static void freeresource();

//和设置标题有关的全局量
size_t  g_argvneedmem=0;        //保存下这些argv参数所需要的内存大小
size_t  g_envneedmem=0;         //环境变量所占内存大小
int     g_os_argc;              //参数个数
char    **g_os_argv;            //原始命令行参数数组,在main中会被赋值
char    *gp_envmem=NULL;        //指向自己分配的env环境变量的内存，在ngx_init_setproctitle()函数中会被分配内存
int     g_daemonized=0;         //守护进程标记，标记是否启用了守护进程模式，0：未启用，1：启用了

//CSocekt      g_socket;        //socket全局对象
CLogicSocket   g_socket;        //socket全局对象  
CThreadPool    g_threadpool;    //线程池全局对象

//和进程本身有关的全局量
pid_t   ngx_pid;                //当前进程的pid
pid_t   ngx_parent;             //父进程的pid
int     ngx_process;            //进程类型，比如master,worker进程等
int     g_stopEvent;            //标志程序退出,0不退出1，退出

sig_atomic_t  ngx_reap;         //标记子进程状态变化[一般是子进程发来SIGCHLD信号表示退出],sig_atomic_t:系统定义的类型：访问或改变这些变量需要在计算机的一条指令内完成
                                //一般等价于int【通常情况下，int类型的变量通常是原子访问的，也可以认为 sig_atomic_t就是int类型的数据】                                   

//程序主入口函数----------------------------------
int main(int argc, char *const *argv)
{     

    int exitcode = 0;           //退出代码，先给0表示正常退出
    int i;                      //临时用
    
    g_stopEvent = 0;            //标记程序是否退出，0不退出          

    ngx_pid    = getpid();      //取得进程pid
    ngx_parent = getppid();     //取得父进程的id 
    //统计argv所占的内存
    g_argvneedmem = 0;
    for(i = 0; i < argc; i++)  //argv =  ./nginx -a -b -c asdfas
    {
        g_argvneedmem += strlen(argv[i]) + 1; //+1是给\0留空间。
    } 
    //统计环境变量所占的内存。注意判断方法是environ[i]是否为空作为环境变量结束标记
    for(i = 0; environ[i]; i++) 
    {
        g_envneedmem += strlen(environ[i]) + 1; //+1是因为末尾有\0,是占实际内存位置的，要算进来
    } //end for

    g_os_argc = argc;           //保存参数个数
    g_os_argv = (char **) argv; //保存参数指针

    //全局量有必要初始化的
    ngx_log.fd = -1;                  //-1：表示日志文件尚未打开；因为后边ngx_log_stderr要用所以这里先给-1
    ngx_process = NGX_PROCESS_MASTER; //先标记本进程是master进程
    ngx_reap = 0;                     //标记子进程没有发生变化
   
    CConfig *p_config = CConfig::GetInstance(); //单例类
    if(p_config->Load("nginx.conf") == false) //把配置文件内容载入到内存            
    {   
        ngx_log_init();    //初始化日志
        ngx_log_stderr(0,"配置文件[%s]载入失败，退出!","nginx.conf");
        //exit(1);终止进程，在main中出现和return效果一样 ,exit(0)表示程序正常, exit(1)/exit(-1)表示程序异常退出，exit(2)表示表示系统找不到指定的文件
        exitcode = 2; //标记找不到文件
        goto lblexit;
    }
    //(2.1)内存单例类可以在这里初始化，返回值不用保存
    CMemory::GetInstance();	
    //(2.2)crc32校验算法单例类可以在这里初始化，返回值不用保存
    CCRC32::GetInstance();
        
    //(3)一些必须事先准备好的资源，先初始化
    ngx_log_init();             //日志初始化(创建/打开日志文件)，这个需要配置项，所以必须放配置文件载入的后边；     
        
    //(4)一些初始化函数，准备放这里        
    /*if(ngx_init_signals() != 0) //信号初始化
    {
        exitcode = 1;
        goto lblexit;
    }*/     

    ngx_init_setproctitle();    //把环境变量搬家

    //------------------------------------
    //(6)创建守护进程
    if(p_config->GetIntDefault("Daemon",0) == 1) //读配置文件，拿到配置文件中是否按守护进程方式启动的选项
    {
        //1：按守护进程方式运行
        int cdaemonresult = ngx_daemon();
        if(cdaemonresult == -1) //fork()失败
        {
            exitcode = 1;    //标记失败
            goto lblexit;
        }
        if(cdaemonresult == 1)
        {
            //这是原始的父进程
            freeresource();   //只有进程退出了才goto到 lblexit，用于提醒用户进程退出了
                              //而现在这个情况属于正常fork()守护进程后的正常退出，不应该跑到lblexit()去执行，因为那里有一条打印语句标记整个进程的退出，这里不该限制该条打印语句；
            exitcode = 0;
            return exitcode;  //整个进程直接在这里退出
        }
        g_daemonized = 1;    //守护进程标记，标记是否启用了守护进程模式，0：未启用，1：启用了
    }

    ngx_master_process_cycle(); //不管父进程还是子进程，正常工作期间都在这个函数里循环；
        
lblexit:
    //(5)该释放的资源要释放掉
    ngx_log_stderr(0,"程序退出，再见了!");
    freeresource();  //一系列的main返回前的释放动作函数
    return exitcode;
}

void freeresource()
{
    //(1)对于因为设置可执行程序标题导致的环境变量分配的内存，应该释放
    if(gp_envmem)
    {
        delete []gp_envmem;
        gp_envmem = NULL;
    }

    //(2)关闭日志文件
    if(ngx_log.fd != STDERR_FILENO && ngx_log.fd != -1)  
    {        
        close(ngx_log.fd); //不用判断结果了
        ngx_log.fd = -1; //标记下，防止被再次close吧        
    }
}
/*
    Master-Worker多进程模型相较于单进程多线程的优势
    1.故障隔离与系统稳定性
    进程级别的隔离：每个处理模块（网络、镜像ICP、不对称度计算、持久化）运行在独立的进程中，一个进程崩溃不会影响其他模块的运行。
    内存保护 ：进程间内存完全隔离，避免了线程间的内存污染和野指针问题
    资源泄漏控制 ：单个进程异常退出时，操作系统会自动回收其所有资源，而线程泄漏可能影响整个进程
    2.资源管理与扩展性
    按需资源分配 ：每个进程只加载必需的库和资源（如网络进程不需要PCL库）
    独立扩展 ：可以根据负载独立调整各进程的线程池大小
    水平扩展 ：可以启动多个相同类型的进程实例来处理更大负载
    3.开发与维护优势
    模块化设计 ：每个进程职责单一，代码结构清晰
    独立调试 ：可以单独调试和测试各个处理模块
    相较于单进程多线程的具体优势：
    避免线程同步的复杂性：
        无需复杂的锁机制来保护共享数据
        避免死锁、竞态条件等并发问题
        减少线程间的相互等待和阻塞
    更好的CPU利用率：
        并行处理，操作系统可以将不同进程调度到不同CPU核心
        避免了单进程内线程竞争CPU资源的问题
        更好地利用多核处理器的并行能力
    内存使用优化：
        每个进程只加载必需的动态库和数据结构
        避免了单进程中所有线程共享大量不必要的内存
        更好的内存局部性和缓存效率
    计算密集型与I/O密集型同时存在：
        I/O密集型任务 （网络进程、持久化进程）：主要等待网络传输、磁盘读写、数据库操作
        计算密集型任务 （镜像ICP进程、不对称度计算进程）：主要消耗CPU进行复杂数学运算
        多进程架构：让两类任务在不同的进程中并行进程，互补干扰
        计算任务不会阻塞I/O任务
        在计算密集型和I/O密集型并存的场景下，多进程模型通过牺牲一定的创建开销和内存占用，
        换来了至关重要的隔离性和真正的并行能力。这使得计算任务和I/O任务能够互不干扰地同时进行，
        最大限度地压榨多核CPU和系统资源的性能。
*/

/*
    在 Linux（和大多数类 Unix 系统）中，系统调用和很多库函数在执行失败时通常不会直接抛出一个异常。
    相反，它们会通过返回值（通常是 -1 或 NULL）来表明“发生了错误”，而具体的错误原因则被存储在一个名为 errno 的全局整型变量中。

    什么是errno:
        一个全局变量：errno 是一个在进程空间中全局可访问的整型变量（extern int errno;）。
        存储错误代码：当系统调用或库函数失败时，它们会设置 errno 为一个特定的正整数，这个数字对应一个具体的错误类型。
        成功时不触碰：非常重要的一点是，只有函数发生错误时才会设置 errno。函数成功执行不会将其重置为 0。因此，
        不能通过检查 errno 是否为 0 来判断是否出错，而必须首先检查函数的返回值是否表明失败。
    
    errno的使用:
        检查返回值：在调用系统调用或库函数后，首先检查其返回值是否为 -1 或 NULL。
        检查errno：如果返回值表明失败，再检查 errno 的值，以确定具体的错误类型。
        所包含的头文件#include <errno.h>

    errno的重要特性与线程安全：
        errno 的传统定义是 extern int errno;，这暗示它是一个真正的全局变量。但在现代操作系统中，
        这会导致多线程问题：如果多个线程同时发生错误并修改 errno，值会被相互覆盖，造成混乱。

        为了解决这个问题，现代 Linux 的 errno 实现采用了线程局部存储（TLS, Thread-Local Storage）：
        每个线程都有自己的 errno：每个线程都独立地拥有和管理自己的 errno 副本。
        在多线程环境下，可以安全的使用errno，不必担心竞争条件。
*/
