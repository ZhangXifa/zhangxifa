
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>     //errno
#include <sys/stat.h>
#include <fcntl.h>


#include "ngx_func.h"
#include "ngx_macro.h"
#include "ngx_c_conf.h"

int ngx_daemon()
{
    //(1)创建守护进程的第一步，fork()一个子进程出来
    switch (fork())
    {
    case -1:
        //创建子进程失败
        ngx_log_error_core(NGX_LOG_EMERG,errno, "ngx_daemon()中fork()失败!");
        return -1;
    case 0:
        //子进程，走到这里直接break;
        break;
    default:
        //父进程以往 直接退出exit(0);现在希望回到主流程去释放一些资源
        return 1;  //父进程直接返回1；
    } //end switch

    ngx_parent = ngx_pid;     //ngx_pid是原来父进程的id，因为这里是子进程，所以子进程的ngx_parent设置为原来父进程的pid
    ngx_pid = getpid();       //当前子进程的id要重新取得
    
    //(2)脱离终端，终端关闭，将跟此子进程无关
    if (setsid() == -1)  
    {
        ngx_log_error_core(NGX_LOG_EMERG, errno,"ngx_daemon()中setsid()失败!");
        return -1;
    }
    /*
        会话层：setsid() 会创建一个新的会话(session)，并让当前进程成为新会话的领头进程(session leader)。    

        此时进程会脱离原有的控制终端，因为一个会话只能有一个控制终端，而新创建的会话尚未关联任何终端。
        - 终端关闭不会影响进程运行
        - 进程不会接收终端发送的信号（如SIGINT、SIGHUP）
        - 避免进程在终端退出时被意外终止
        脱离终端的三个层面：
        1. 会话层：setsid()创建新会话，进程成为新会话的领头进程。脱离原有的控制终端。不再受终端关闭的影响。
        2. 进程组层：进程组ID与进程ID相同，进程成为新进程组的组长进程。
        3. 控制终端层：进程与控制终端断开关联，不会接收终端发送的信号。（Ctrl+C (SIGINT)、Ctrl+Z (SIGTSTP)）

    */
    /*
        Linux 采用了 写时拷贝（Copy-On-Write, COW） 技术。这意味着在 fork() 的瞬间，父子进程的物理内存是共享的，
        内核只会将它们的页表（内存映射表）标记为只读。只有当任一进程尝试修改某一块内存时，内核才会真正地为修改进程复制那块内存页。
        这极大地提高了 fork() 的效率。

        fork()的核心思想是创建一个与父进程几乎完全相同的副本，父进程还是会有一些东西是子进程不会继承的，
        不会继承的：
            进程ID（PID）：
                这是最明显的区别。子进程会获得一个全新的、唯一的进程ID。
                通过 getpid() 获取。
            父进程ID (PPID)：
                子进程的父进程ID (getppid()) 被设置为调用fork()的进程的PID。
            资源使用统计：
                子进程的 CPU 时间计数器（如 times() 返回的值会被重置为零。
            挂起的信号：
                子进程不会继承父进程的未决信号（Pending Signals）。任何在父进程中已经产生但尚未被处理的信号，都不会出现在子进程中。
            文件锁 (File Locks)：
                子进程不会继承父进程持有的任何文件锁（例如通过 fcntl() 设置的锁）。
                父进程锁住一个文件后 fork()，子进程需要自己重新获取锁。
            内存锁 (Memory Locks)：
                通过 mlock() 或 mlockall() 锁定的内存区域不会被子进程继承。
        会被共享和继承的资源：
            打开的文件描述符：
                这是最重要也是最需要理解的一点。子进程会获得父进程所有打开的文件描述符的副本。
                “副本”意味着它们指向内核中同一个打开的文件表项（Open File Description）。这导致了以下关键行为：
                    共享文件偏移量（File Offset）：如果父进程打开了一个文件，然后 fork()，子进程从同一个位置开始读写文件。
                    如果父进程读取了100字节，文件偏移量前进，子进程会从第100字节开始读写。
                    共享文件状态标志：使用 fcntl(fd, F_GETFL) 获取的标志是相同的。
            信号处理方式：
                子进程会继承父进程为每个信号设置的处理函数（Handler）、忽略（IGNORE）或默认（DEFAULT）设置。
            内存空间（写时拷贝）
            进程组ID (PGID) 和会话ID (SID)。
            进程继承父进程的文件模式创建屏蔽字（umask）。
            环境变量。
            工作目录。    
    */

    //(3)设置为0，不要让它来限制文件权限，以免引起混乱
    umask(0); 

    //(4)打开黑洞设备，以读写方式打开
    int fd = open("/dev/null", O_RDWR);
    /*
        O_RDONLY    // 只读模式
        O_WRONLY    // 只写模式
        O_RDWR      // 读写模式（代码中使用的）
    */
    if (fd == -1) 
    {
        ngx_log_error_core(NGX_LOG_EMERG,errno,"ngx_daemon()中open(\"/dev/null\")失败!");      
        return -1;
    }
    if (dup2(fd, STDIN_FILENO) == -1)//标准输入重定向到/dev/null
    {
        ngx_log_error_core(NGX_LOG_EMERG,errno,"ngx_daemon()中dup2(STDIN)失败!");        
        return -1;
    }
    if (dup2(fd, STDOUT_FILENO) == -1)//标准输出重定向到/dev/null
    {
        ngx_log_error_core(NGX_LOG_EMERG,errno,"ngx_daemon()中dup2(STDOUT)失败!");
        return -1;
    }
    if (fd > STDERR_FILENO)  //fd应该是3，这个应该成立
     {
        if (close(fd) == -1) 
        {
            ngx_log_error_core(NGX_LOG_EMERG,errno, "ngx_daemon()中close(fd)失败!");
            return -1;
        }
    }
    return 0; //子进程返回0
}
/*
    标准输入（stdin）、标准输出（stdout）和标准错误（stderr）是三个核心的I/O（输入/输出）流，
    它们为程序与外部环境（如终端、文件或其他程序）的交互提供了统一的接口。
    标准输入（STDIN_FILENO）：程序读取输入数据的默认来源（通常是键盘输入或管道/重定向的输入）。
    标准输出（STDOUT_FILENO）：程序正常输出数据的默认目的地（通常是终端或管道/重定向的输出）。
    标准错误（STDERR_FILENO）：程序错误输出数据的默认目的地（通常是终端或管道/重定向的输出）。
    Linux系统下，0、1、2分别代表标准输入、标准输出、标准错误。自己申请的文件描述符是从3开始的。

    综上，创建守护进程的过程为：
        1. fork()一个子进程出来
        2. 子进程调用setsid()创建新会话
        3. 子进程调用umask(0)设置文件权限掩码为0
        4. 子进程调用open(\"/dev/null\")打开黑洞设备
        5. 子进程调用dup2()将标准输入、标准输出、标准错误重定向到黑洞设备
        6. 子进程调用close()关闭黑洞设备
        7. 子进程返回0
    
    对于setsid()函数的详细解释：
        setsid() 的“脱离终端”指的是创建一个全新的会话（Session），
        并让调用进程成为该会话的首进程（Session Leader）和新的进程组组长（Process Group Leader），
        最关键的是，这个新会话将不再与任何控制终端（Controlling Terminal）关联。

        UNIX/Linux进程管理的三个层次：
            进程组 (Process Group)：一个或多个进程的集合。每个进程组有一个唯一的进程组 ID (PGID)。
            通常，一个 Shell 管道命令（如 ls | grep foo | sort）中的所有进程都属于同一个进程组。
            进程组并不等于父进程+子进程，进程组是一个独立的单位，进程组内的进程可以通信，但是进程组外的进程不能通信。

            会话 (Session)：一个或多个进程组的集合。每个会话有一个唯一的会话 ID (SID)。
            一个会话最多只能有一个控制终端。

            控制终端 (Controlling Terminal)：一个终端设备（如 /dev/tty1, 伪终端 pts/0 等），
            用于接收用户的输入并向进程组发送信号（如 Ctrl+C 发送 SIGINT，Ctrl+Z 发送 SIGTSTP）。
        一个典型的登录环境：
            通过终端（或 SSH）登录系统。
            系统启动一个 Shell（如 bash）。
            这个 Shell 进程自己就是一个会话的首进程，并且占据了该终端作为其控制终端。
            在 Shell 中运行的每一个命令（前台或后台）都会成为该会话的一部分，并继承这个控制终端。

            用XShell连接远程的服务器，服务器会产生一个shell（bash），这个看的见的黑框框就是控制终端，
            生成的shell就是会话。
        Shell（例如 Bash）是一个特殊的程序，它充当了用户和操作系统内核（Linux 的核心）之间的翻译官和中间人。
        Shell是运行在控制终端中的“领导者进程”，它代表用户接管并管理这个终端。控制终端则是Shell的工作环境和活动舞台。
*/

