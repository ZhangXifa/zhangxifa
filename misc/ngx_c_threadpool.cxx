#include <stdarg.h>
#include <unistd.h>  //usleep

#include "ngx_global.h"
#include "ngx_func.h"
#include "ngx_c_threadpool.h"
#include "ngx_c_memory.h"
#include "ngx_macro.h"

//静态成员初始化
pthread_mutex_t CThreadPool::m_pthreadMutex = PTHREAD_MUTEX_INITIALIZER;  //#define PTHREAD_MUTEX_INITIALIZER ((pthread_mutex_t) -1)
pthread_cond_t CThreadPool::m_pthreadCond = PTHREAD_COND_INITIALIZER;     //#define PTHREAD_COND_INITIALIZER ((pthread_cond_t) -1)
/*
    互斥锁和条件变量的静态初始化和动态初始化：
    静态初始化：
        互斥锁：pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
        条件变量：pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
        编译时初始化：在程序加载时就完成初始化
        静态成员变量：作为类的静态成员，全局唯一
        简单高效，无函数调用的开销
    动态初始化：
        互斥锁：pthread_mutex_init(&mutex, NULL);
        条件变量：pthread_cond_init(&cond, NULL);
        动态初始化：在程序运行时调用函数进行初始化
        实例化成员变量：每个对象实例都有自己的互斥锁和条件变量
        需要错误检查：函数可能失败，需要检查返回值
        灵活性高，支持动态创建和销毁
*/
bool CThreadPool::m_shutdown = false;    //刚开始标记整个线程池的线程是不退出的      

//构造函数
CThreadPool::CThreadPool()
{
    m_iRunningThreadNum = 0;  //正在运行的线程，开始给个0【注意这种写法：原子的对象给0也可以直接赋值，当整型变量来用】
    m_iLastEmgTime = 0;       //上次报告线程不够用了的时间；
    //m_iPrintInfoTime = 0;    //上次打印参考信息的时间；
    m_iRecvMsgQueueCount = 0; //收消息队列
}

//析构函数
CThreadPool::~CThreadPool()
{    
    //资源释放在StopAll()里统一进行，就不在这里进行了

    //接收消息队列中内容释放
    clearMsgRecvQueue();
}

//各种清理函数-------------------------
//清理接收消息队列，注意这个函数的写法。
void CThreadPool::clearMsgRecvQueue()
{
	char * sTmpMempoint;
	CMemory *p_memory = CMemory::GetInstance();

	//尾声阶段，需要互斥？该退的都退出了，该停止的都停止了，应该不需要退出了
	while(!m_MsgRecvQueue.empty())
	{
		sTmpMempoint = m_MsgRecvQueue.front();		
		m_MsgRecvQueue.pop_front(); 
		p_memory->FreeMemory(sTmpMempoint);
	}	
}

//创建线程池中的线程，要手工调用，不在构造函数里调用了
//返回值：所有线程都创建成功则返回true，出现错误则返回false
bool CThreadPool::Create(int threadNum)
{    
    ThreadItem *pNew;
    int err;

    m_iThreadNum = threadNum; //保存要创建的线程数量
    
    for(int i = 0; i < m_iThreadNum; ++i)
    {
        m_threadVector.push_back(pNew = new ThreadItem(this));             //创建 一个新线程对象 并入到容器中         
        err = pthread_create(&pNew->_Handle, NULL, ThreadFunc, pNew);      //创建线程，错误不返回到errno，一般返回错误码
        if(err != 0)
        {
            //创建线程有错
            ngx_log_stderr(err,"CThreadPool::Create()创建线程%d失败，返回的错误码为%d!",i,err);
            return false;
        }      
    } //end for
    /*
        pthread_create()函数原型：
        int pthread_create(pthread_t *thread, const pthread_attr_t *attr,
                        void *(*start_routine) (void *), void *arg);
        第一个参数：thread，类型pthread_t*，输出型参数，用来返回创建的线程id
        第二个参数：attr，类型const pthread_attr_t*，输入型参数，用来设置线程的属性，一般用默认值，所以给NULL即可
        第三个参数：start_routine，类型void* (*)(void*)，输入型参数，线程要执行的函数，这里是静态成员函数ThreadFunc()
        第四个参数：arg，类型void*，输入型参数，线程要执行的函数的参数，这里是线程对象的指针
        需要注意的是pthread_create函数只接受四个参数，如果线程函数有多个参数，需要封装成结构体传递
        返回值：成功返回0，失败返回错误码，错误码直接返回，不设置errno
        错误码：
            EAGAIN：系统线程数已达上限
            EINVAL：attr参数无效
            EPERM：调用进程没有足够的权限来创建线程
            ESRCH：没有找到与pthread_t匹配的线程
        需要注意的是pthread_create()函数的线程函数只能接受一个参数，若有多个参数需要传递，则需要封装成结构体传递
        还需要注意的是传递给线程函数的参数必须在堆上申请内存，不能使用栈上内存，栈上内存args可能弹出栈销毁，但是线程
        函数可能还在执行，这时，会发生段错误、数据损坏。未定义的问题。这是由于pthread_create()立即返回，但是新线程
        可能没有立即执行，pthread_create()只是注册，告诉操作系统创建一个线程，并不立即执行。线程的调度是异步的，新
        线程何时开始由操作系统决定。故有可能发生的情况是线程创建函数以及所有局部变量已经弹出栈销毁，创建的某一线程开始
        执行，程序错误。
    */

    //必须保证每个线程都启动并运行到pthread_cond_wait()，本函数才返回，只有这样，这几个线程才能进行后续的正常工作 
    std::vector<ThreadItem*>::iterator iter;
lblfor:
    for(iter = m_threadVector.begin(); iter != m_threadVector.end(); iter++)
    {
        if( (*iter)->ifrunning == false) //这个条件保证所有线程完全启动起来，以保证整个线程池中的线程正常工作；
        {
            //这说明有没有启动完全的线程
            usleep(100 * 1000);  //单位是微妙,又因为1毫秒=1000微妙，所以 100 *1000 = 100毫秒
            goto lblfor;
        }
    }
    return true;
}

//线程入口函数，当用pthread_create()创建线程后，这个ThreadFunc()函数都会被立即执行；
void* CThreadPool::ThreadFunc(void* threadData)
{
    //这个是静态成员函数，是不存在this指针的
    ThreadItem *pThread = static_cast<ThreadItem*>(threadData);
    CThreadPool *pThreadPoolObj = pThread->_pThis;
    
    CMemory *p_memory = CMemory::GetInstance();	    
    int err;

    pthread_t tid = pthread_self(); //获取线程自身id
    while(true)
    {
        //线程用pthread_mutex_lock()函数去锁定指定的mutex变量，若该mutex已经被另外一个线程锁定了，该调用将会阻塞线程直到mutex被解锁。  
        err = pthread_mutex_lock(&m_pthreadMutex);  
        if(err != 0) ngx_log_stderr(err,"CThreadPool::ThreadFunc()中pthread_mutex_lock()失败，返回的错误码为%d!",err);//有问题，要及时报告
        

        while ( (pThreadPoolObj->m_MsgRecvQueue.size() == 0) && m_shutdown == false)
        {
            if(pThread->ifrunning == false)            
                pThread->ifrunning = true; //标记为true了才允许调用StopAll()
            pthread_cond_wait(&m_pthreadCond, &m_pthreadMutex); //整个服务器程序刚初始化的时候，所有线程必然是卡在这里等待的；
        }
        /*
            pthread_cond_wait()函数原型：
            int pthread_cond_wait(pthread_cond_t *cond, pthread_mutex_t *mutex);
            第一个参数：cond，类型pthread_cond_t*，输入型参数，用来指定条件变量
            用于线程间的条件同步，当线程调用pthread_cond_wait()函数时，会自动释放mutex锁，
            并阻塞线程，等待其他线程调用pthread_cond_signal()或pthread_cond_broadcast()函数唤醒该线程。
            静态初始化： PTHREAD_COND_INITIALIZER
            第二个参数：mutex，类型pthread_mutex_t*，输入型参数，用来指定互斥量
            必须在调用前以被当前线程锁定，与条件变量配合使用。
            返回值：成功返回0，失败返回错误码
        */
        //先判断线程退出这个条件
        if(m_shutdown)
        {   
            pthread_mutex_unlock(&m_pthreadMutex); //解锁互斥量
            break;                     
        }

        //走到这里，可以取得消息进行处理了【消息队列中必然有消息】,注意，目前还是互斥着呢
        char *jobbuf = pThreadPoolObj->m_MsgRecvQueue.front();     //返回第一个元素但不检查元素存在与否
        pThreadPoolObj->m_MsgRecvQueue.pop_front();                //移除第一个元素但不返回	
        --pThreadPoolObj->m_iRecvMsgQueueCount;                    //收消息队列数字-1
               
        //可以解锁互斥量了
        err = pthread_mutex_unlock(&m_pthreadMutex); 
        if(err != 0)  ngx_log_stderr(err,"CThreadPool::ThreadFunc()中pthread_mutex_unlock()失败，返回的错误码为%d!",err);//有问题，要及时报告
        
        //能走到这里的，就是有消息可以处理，开始处理
        ++pThreadPoolObj->m_iRunningThreadNum;    //原子+1【记录正在干活的线程数量增加1】，这比互斥量要快很多

        g_socket.threadRecvProcFunc(jobbuf);     //处理消息队列中来的消息


        p_memory->FreeMemory(jobbuf);              //释放消息内存 
        --pThreadPoolObj->m_iRunningThreadNum;     //原子-1【记录正在干活的线程数量减少1】

    } //end while(true)

    //能走出来表示整个程序要结束啊，怎么判断所有线程都结束？
    return (void*)0;
}

//停止所有线程【等待结束线程池中所有线程，该函数返回后，应该是所有线程池中线程都结束了】
void CThreadPool::StopAll() 
{
    //(1)已经调用过，就不要重复调用了
    if(m_shutdown == true)
    {
        return;
    }
    m_shutdown = true;

    //(2)唤醒等待该条件【卡在pthread_cond_wait()的】的所有线程，一定要在改变条件状态以后再给线程发信号
    int err = pthread_cond_broadcast(&m_pthreadCond);
    //广播唤醒所有等待线程的条件变量函数。
    //惊群效应：所有线程同时被唤醒，可能造成系统负载峰值。
    //锁竞争：多个线程同时竞争互斥锁。
    //适用场景：更适合状态改变，而并非任务分发。
    if(err != 0)
    {
        //这肯定是有问题，要打印紧急日志
        ngx_log_stderr(err,"CThreadPool::StopAll()中pthread_cond_broadcast()失败，返回的错误码为%d!",err);
        return;
    }

    //(3)等等线程，让线程真返回    
    std::vector<ThreadItem*>::iterator iter;
	for(iter = m_threadVector.begin(); iter != m_threadVector.end(); iter++)
    {
        pthread_join((*iter)->_Handle, NULL); //等待一个线程终止
        //阻塞特性：如果目标线程尚未终止，pthread_join()会一直阻塞等待。
        //立即返回：如果目标线程已经终止，pthread_join()会立即返回。
        //一次调用：每个线程只能被join一次，重复调用会导致未定义行为。
        //注意事项：不要在目标线程上调用pthread_join()，否则会导致死锁。
        //分离线程：已经分离的线程不能被join。
        //未join线程：如果线程没有被join，那么在主线程退出时，这些线程的资源不会被释放。僵尸线程。
        /*
            线程的终止和未终止的定义：
            终止：
                线程函数执行完毕并返回（return语句）
                调用pthread_exit()主动退出
                线程函数执行到末尾自然结束。
                被其他线程通过pthread_cancel()取消
                进程收到致命信号（SIGKILL、SIGEGV）
                进程异常退出导致所有线程终止
            未终止：
                线程正在CPU上执行
                线程在等待I/O操作完成
                线程被阻塞在同步原语上（mutex、cond）
                线程处于就绪状态等待CPU调用
        */
        /*
            pthread_detach()函数：
            功能：将线程设置为分离状态
            原型：int pthread_detach(pthread_t thread);
            参数：thread，类型pthread_t，输入型参数，用来指定线程ID
            返回值：成功返回0，失败返回错误码
            分离线程：将指定线程标记为分离状态（detached state）
            自动回收：线程终止时系统自动回收其资源，无需手动join。一旦线程被分离，就不能再被join。
        */
    }

    //流程走到这里，那么所有的线程池中的线程肯定都返回了；
    pthread_mutex_destroy(&m_pthreadMutex);
    pthread_cond_destroy(&m_pthreadCond);
    //静态分配的互斥锁和条件变量不是必须要用destroy销毁，动态分配的互斥锁和条件变量需要用destroy销毁。
    //无论哪种分配方式，最好都应该调用destroy销毁。

    //(4)释放一下new出来的ThreadItem【线程池中的线程】    
	for(iter = m_threadVector.begin(); iter != m_threadVector.end(); iter++)
	{
		if(*iter)
			delete *iter;
	}
	m_threadVector.clear();

    ngx_log_stderr(0,"CThreadPool::StopAll()成功返回，线程池中线程全部正常结束!");
    return;    
}

//--------------------------------------------------------------------------------------
//收到一个完整消息后，入消息队列，并触发线程池中线程来处理该消息
void CThreadPool::inMsgRecvQueueAndSignal(char *buf)
{
    //互斥
    int err = pthread_mutex_lock(&m_pthreadMutex);     
    if(err != 0)
    {
        ngx_log_stderr(err,"CThreadPool::inMsgRecvQueueAndSignal()pthread_mutex_lock()失败，返回的错误码为%d!",err);
    }
        
    m_MsgRecvQueue.push_back(buf);	         //入消息队列
    ++m_iRecvMsgQueueCount;                  //收消息队列数字+1，个人认为用变量更方便一点，比 m_MsgRecvQueue.size()高效

    //取消互斥
    err = pthread_mutex_unlock(&m_pthreadMutex);   
    if(err != 0)
    {
        ngx_log_stderr(err,"CThreadPool::inMsgRecvQueueAndSignal()pthread_mutex_unlock()失败，返回的错误码为%d!",err);
    }

    //可以激发一个线程来干活了
    Call();                                  
    return;
}

//来任务了，调一个线程池中的线程下来干活
void CThreadPool::Call()
{
    int err = pthread_cond_signal(&m_pthreadCond); 
    //唤醒单个线程：唤醒等待在指定条件变量上的单个线程
    //非阻塞操作：
    //如果没有线程在等待条件变量，调用pthread_cond_signal()不会阻塞当前线程，也不会返回错误。
    //如果有多个线程在等待条件变量，pthread_cond_signal()会唤醒其中一个线程，具体唤醒哪个线程是不确定的。

    if(err != 0 )
    {
        ngx_log_stderr(err,"CThreadPool::Call()中pthread_cond_signal()失败，返回的错误码为%d!",err);
    }
    
    if(m_iThreadNum == m_iRunningThreadNum) //线程池中线程总量，跟当前正在干活的线程数量一样，说明所有线程都忙碌起来，线程不够用了
    {        
     
        time_t currtime = time(NULL);
        if(currtime - m_iLastEmgTime > 10) //最少间隔10秒钟才报一次线程池中线程不够用的问题；
        {
            m_iLastEmgTime = currtime;  //更新时间
            ngx_log_stderr(0,"CThreadPool::Call()中发现线程池中当前空闲线程数量为0，要考虑扩容线程池了!");
        }
    } //end if 

    return;
}

