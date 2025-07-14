
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>   //信号相关头文件 
#include <errno.h>    //errno
#include <unistd.h>

#include "ngx_func.h"
#include "ngx_macro.h"
#include "ngx_c_conf.h"

#include "ngx_lockfree_threadPool.h"

#include "ngx_mysql_connection.h"
//函数声明
static void ngx_start_worker_processes();
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

/*std::unique_ptr<draco::PointCloud> decompressPointCloud(char *pPkgBody,uint32_t iBodyLength){
    draco::DecoderBuffer buffer;
    buffer.Init(pPkgBody,iBodyLength);
    draco::Decoder decoder;
        auto statusor = decoder.DecodePointCloudFromBuffer(&buffer);
        if (!statusor.ok()) {
            ngx_log_stderr(0,"解压失败.");
            return nullptr;
        }

        ngx_log_stderr(0,"解压成功.");
        return std::move(statusor).value();
}
void saveAsPCD(const draco::PointCloud& draco_cloud, const std::string& filename) {
    // 创建PCL点云对象
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl_cloud.width = draco_cloud.num_points();
    pcl_cloud.height = 1;
    pcl_cloud.is_dense = false;
    pcl_cloud.points.resize(draco_cloud.num_points());

    // 获取位置属性
    const draco::PointAttribute* pos_attr = draco_cloud.GetNamedAttribute(draco::GeometryAttribute::POSITION);
    if (!pos_attr) {
        ngx_log_stderr(0,"无法获取属性");
        return;
    }

    // 转换数据到PCL格式
    for (uint32_t i = 0; i < draco_cloud.num_points(); ++i) {
        draco::PointIndex point_index(i);  // 创建PointIndex对象
        float pos[3];
        pos_attr->GetMappedValue(point_index, pos);
        pcl_cloud.points[i].x = pos[0];
        pcl_cloud.points[i].y = pos[1];
        pcl_cloud.points[i].z = pos[2];
    };
    // 保存为PCD文件
    if (pcl::io::savePCDFileBinary(filename, pcl_cloud) == -1) {
        ngx_log_stderr(0,"保存失败.");
        return;
    }
}*/

//描述：创建worker子进程
void ngx_master_process_cycle()
{    
    sigset_t set;        //信号集

    sigemptyset(&set);   //清空信号集

    //下列这些信号在执行本函数期间不希望收到【考虑到官方nginx中有这些信号，老师就都搬过来了】（保护不希望由信号中断的代码临界区）
    //建议fork()子进程时学习这种写法，防止信号的干扰；
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
    //.........可以根据开发的实际需要往其中添加其他要屏蔽的信号......
    
    //设置，此时无法接受的信号；阻塞期间，你发过来的上述信号，多个会被合并为一个，暂存着，等你放开信号屏蔽后才能收到这些信号。。。
    //sigprocmask()在第三章第五节详细讲解过
    if (sigprocmask(SIG_BLOCK, &set, NULL) == -1) //第一个参数用了SIG_BLOCK表明设置 进程 新的信号屏蔽字 为 “当前信号屏蔽字 和 第二个参数指向的信号集的并集
    {        
        ngx_log_error_core(NGX_LOG_ALERT,errno,"ngx_master_process_cycle()中sigprocmask()失败!");
    }
    //即便sigprocmask失败，程序流程 也继续往下走

    //首先我设置主进程标题---------begin
    size_t size;
    int    i;
    size = sizeof(master_process);  //注意我这里用的是sizeof，所以字符串末尾的\0是被计算进来了的
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
    //首先我设置主进程标题---------end
    
    // 创建网络到master的共享内存
    NetworkToMasterQueue* net_queue = open_shm_queue<NetworkToMasterQueue>(NETWORK_TO_MASTER_SHM);
    // 创建master到镜像/ICP处理的共享内存
    MasterToMirorProcessQueue* mirrorICP_queue = open_shm_queue<MasterToMirorProcessQueue>(MASTER_TO_MIRROR_PROCESS_SHM);
    // 创建镜像/ICP处理到master的共享内存
    MirorProcessToMasterQueue* mirrorICPed_queue = open_shm_queue<MirorProcessToMasterQueue>(MIRROR_PROCESS_TO_MASTER_SHM);
    //创建master到结果处理的共享内存
    MasterToResProcessQueue* result_queue = open_shm_queue<MasterToResProcessQueue>(MASTER_TO_RESULT_PROCESS_SHM);
    //创建不对称度计算进程到master进程的共享内存
    ResProcessToMasterQueue* resulted_queue = open_shm_queue<ResProcessToMasterQueue>(RESULT_PROCESS_TO_MASTER_SHM);
    //创建master进程到持久化进程的共享内存
    MasterToPersistProcessQueue* persist_queue = open_shm_queue<MasterToPersistProcessQueue>(MASTER_TO_PERSIST_PROCESS_SHM);

    //创建返回结果的共享内存
    AsymmProcessToMaterQueue* ToMasterQueue = open_shm_queue<AsymmProcessToMaterQueue>(RETURN_TO_MASTER_SHM);
    MasterToNetworkQueue* ToNetworkQueue = open_shm_queue<MasterToNetworkQueue>(RETURN_TO_NETWORK_SHM);

    ngx_start_worker_processes();  //这里要创建worker子进程

    //创建子进程后，父进程的执行流程会返回到这里，子进程不会走进来    
    sigemptyset(&set); //信号屏蔽字为空，表示不屏蔽任何信号
    
    for ( ;; ) 
    {

    
        //sigsuspend(&set); //阻塞在这里，等待一个信号，此时进程是挂起的，不占用cpu时间，只有收到信号才会被唤醒（返回）；
        // 数据转发核心逻辑
        PointCloud cloud;
        if (net_queue->try_pop(cloud)) {
            // 从网络队列读取并写入处理队列
            while (!mirrorICP_queue->try_push(std::move(cloud))) {
                usleep(1000); // 队列满时等待
            }
            ngx_log_stderr(0, "数据转移成功");
        } else {
            usleep(1000); // 队列空时等待
        }
        MirrorICPPointCloud ICPPointCloud;
        if(mirrorICPed_queue->try_pop(ICPPointCloud)){
            while(!result_queue->try_push(std::move(ICPPointCloud))){
                usleep(1000);
            }
            ngx_log_stderr(0, "第二次数据转移成功");
        }else{
             usleep(1000);
        }
        ResPointCloud res;
        if(resulted_queue->try_pop(res)){
            while(!persist_queue->try_push(std::move(res))){
                usleep(1000);
            }
            ngx_log_stderr(0, "第三次数据转移成功");
        }else{
             usleep(1000);
        }
        ResToNetwork ans;
        if(ToMasterQueue->try_pop(ans)){
            while(!ToNetworkQueue->try_push(std::move(ans))){
                usleep(1000);
            }
            ngx_log_stderr(0, "第四次数据转移成功");
        }else{
             usleep(1000);
        }

    }// end for(;;)
     return;
}

//描述：根据给定的参数创建指定数量的子进程，因为以后可能要扩展功能，增加参数，所以单独写成一个函数
//threadnums:要创建的子进程数量
static void ngx_start_worker_processes()
{
    ngx_spawn_process(1,"network process");
    ngx_mirror_process(2,"mirror process");
    ngx_result_process(3,"result process");
    ngx_persist_process(4,"persult process");
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

    //重新为子进程设置进程名，不要与父进程重复------
    g_master_to_network_queue = open_shm_queue<MasterToNetworkQueue>(RETURN_TO_NETWORK_SHM);//这段共享内存的初始化一定要提前到线程初始化之前
    ngx_network_process_init(inum);
    Connection conn;//mysql_init()并不是线程安全，在程序初始化的时候调用一次，之后线程安全
    ngx_setproctitle(pprocName); //设置标题   
    ngx_log_error_core(NGX_LOG_NOTICE,0,"%s %P 【network进程】启动并开始运行......!",pprocName,ngx_pid); //设置标题时顺便记录下来进程名，进程id等信息到日志
    g_net_to_master_queue = open_shm_queue<NetworkToMasterQueue>(NETWORK_TO_MASTER_SHM);
    for(;;)
    {

      

        ngx_process_events_and_timers(); //处理网络事件和定时器事件

     

    } //end for(;;)

    //如果从这个循环跳出来
    g_threadpool.StopAll();      //考虑在这里停止线程池；
    g_socket.Shutdown_subproc(); //socket需要释放的东西考虑释放；
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
        sleep(1);
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
        sleep(1);
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
        sleep(1);
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