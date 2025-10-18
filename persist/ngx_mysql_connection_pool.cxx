#include "ngx_mysql_connection_pool.h"
#include "ngx_func.h"

// 线程安全的懒汉单例函数接口
MySQLConnectionPool* MySQLConnectionPool::getConnectionPool()
{
	static MySQLConnectionPool pool; // lock和unlock
	return &pool;
}

// 从配置文件中加载配置项
bool MySQLConnectionPool::loadConfigFile()
{
	FILE *pf = fopen("mysql.ini", "r");
	if (pf == nullptr)
	{
		ngx_log_stderr(0, "mysql.ini file is not exist!");
		return false;
	}

	while (!feof(pf))
	{
		char line[1024] = { 0 };
		fgets(line, 1024, pf);
		std::string str = line;
		int idx = str.find('=', 0);
		if (idx == -1) // 无效的配置项
		{
			continue;
		}

		int endidx = str.find('\n', idx);
		std::string key = str.substr(0, idx);
		std::string value = str.substr(idx + 1, endidx - idx - 1);

		if (key == "ip")
		{
			_ip = value;
		}
		else if (key == "port")
		{
			_port = atoi(value.c_str());
		}
		else if (key == "username")
		{
			_username = value;
		}
		else if (key == "password")
		{
			_password = value;
		}
		else if (key == "dbname")
		{
			_dbname = value;
		}
		else if (key == "initSize")
		{
			_initSize = atoi(value.c_str());
		}
		else if (key == "maxSize")
		{
			_maxSize = atoi(value.c_str());
		}
		else if (key == "maxIdleTime")
		{
			_maxIdleTime = atoi(value.c_str());
		}
		else if (key == "connectionTimeOut")
		{
			_connectionTimeout = atoi(value.c_str());
		}
	}
	return true;
}

// 连接池的构造
MySQLConnectionPool::MySQLConnectionPool()
{
	_isRuning = true;
	// 加载配置项了
	if (!loadConfigFile())
	{
		return;
	}

	// 创建初始数量的连接
	for (int i = 0; i < _initSize; ++i)
	{
		Connection *p = new Connection();
		p->connect(_ip, _port, _username, _password, _dbname);
		p->refreshAliveTime(); // 刷新一下开始空闲的起始时间
		_connectionQue.push(p);
		_connectionCnt++;
	}

	// 启动一个新的线程，作为连接的生产者 linux thread => pthread_create
	std::thread produce(std::bind(&MySQLConnectionPool::produceConnectionTask, this));
	//C++11的thread函数底层还是调用pthread_create，只是封装了一下，方便使用
	//此函数的返回值是一个线程对象，要得到线程的id需要调用get_id()函数
	//std::thread::id tid = t.get_id();
	produce.detach();
	/*
		函数原型：
			// C++11 std::thread构造函数
			template<class Function, class... Args>
			explicit thread(Function&& f, Args&&... args);
		这里也可以不使用bind：
			1.使用lambda表达式
				std::thread produce([this]() {
    				this->produceConnectionTask();});
			2.使用函数指针
				std::thread produce(&MySQLConnectionPool::produceConnectionTask, this);
			3.使用std::ref()
				std::thread produce(std::mem_fn(&MySQLConnectionPool::produceConnectionTask), this);
		
	*/

	/*
		有参数线程的处理方式：
		直接传参：
			//对于普通函数
			void threadFunc(int param1, std::string param2) { ... }
			std::thread t(threadFunc, 42, "hello");
			//对于成员函数
			std::thread t(&ClassName::memberFunc, &obj, param1, param2);
		使用std::bind()
			std::thread t(std::bind(&ClassName::memberFunc, this, param1, param2));
		使用lambda表达式
			std::thread t([this, param1, param2]() {
    			this->memberFunc(param1, param2);
			});
	*/
	// 启动一个新的定时线程，扫描超过maxIdleTime时间的空闲连接，进行对于的连接回收
	std::thread scanner(std::bind(&MySQLConnectionPool::scannerConnectionTask, this));
	scanner.detach();
	/*
		使用detach和不使用detach的区别：
		使用detach：
			线程分离 ：线程创建后立即与主线程分离，独立运行
			无需保存句柄 ：不需要保存 std::thread 对象，节省内存
			不能join ：分离后的线程无法再调用 join()
			资源自动回收 ：线程结束时系统自动回收资源
			异步创建，同步销毁 ：通过条件变量等机制实现精确的生命周期控制
		不使用detach：
			线程关联 ：线程对象与主线程保持关联
			必须保存句柄 ：需要保存 std::thread 对象用于后续管理
			必须join或detach ：析构前必须调用 join() 或 detach() ，否则程序终止
			主动等待 ：通过 join() 主动等待线程完成
			资源手动管理 ：需要显式管理线程生命周期
		detach适用于：
			后台服务线程 ：如连接池维护、定时清理
			事件驱动系统 ：需要长期运行的监听线程
			资源受限环境 ：需要最小化内存占用
		不使用detach()适用于：
			任务处理线程池 ：需要等待所有任务完成
			批量计算 ：需要确保所有计算线程完成后再继续
			简单同步需求 ：直接使用join()等待即可
		detach对主线程的影响：
			使用和不使用detach在创还能和运行时都不会阻塞主线程，但在细狗时，因为不使用detach的话要用join等待
			线程结束，故会阻塞，使用detach不会阻塞。
	*/
}

MySQLConnectionPool::~MySQLConnectionPool() {

	_isRuning = false;
	cv.notify_all();  // 唤醒所有等待的线程
	std::unique_lock<std::mutex> lock(_queueMutex);
	cv.wait(lock, [this] { return _connectionQue.empty(); });  // 等待所有线程结束	
}

// 运行在独立的线程中，专门负责生产新连接
void MySQLConnectionPool::produceConnectionTask()
{
	while (_isRuning)
	{
		std::unique_lock<std::mutex> lock(_queueMutex);
		cv.wait(lock, [this] { return _connectionQue.empty() || !_isRuning; }); // 队列不空，此处生产线程进入等待状态
		//std::condition_variable 在Linux/Unix系统上的底层确实是基于 pthread_cond_wait 实现的。
		/*
			wait成员函数的函数原型：
				基础版本：void wait(std::unique_lock<std::mutex>& lock);
				带谓词版本：void wait(std::unique_lock<std::mutex>& lock, Predicate pred);
					接受一个锁引用和一个谓词函数，内部会循环检查谓词条件，只有当条件满足时才返回。	
		*/
		if (!_isRuning)
		{
			break;
		}

		// 连接数量没有到达上限，继续创建新的连接
		if (_connectionCnt < _maxSize)
		{
			Connection *p = new Connection();
			p->connect(_ip, _port, _username, _password, _dbname);
			p->refreshAliveTime(); // 刷新一下开始空闲的起始时间
			_connectionQue.push(p);
			_connectionCnt++;
		}
		// 通知消费者线程，可以消费连接了
		cv.notify_all();
	}
}
//只有当连接容器中的连接消耗殆尽时，才会创建新的连接，不然在条件变量处阻塞，等待连接被消费


// 给外部提供接口，从连接池中获取一个可用的空闲连接
std::shared_ptr<Connection> MySQLConnectionPool::getConnection()
{
	std::unique_lock<std::mutex> lock(_queueMutex);
	while (_connectionQue.empty() && _isRuning)
	{
		// sleep
		if (std::cv_status::timeout == cv.wait_for(
			lock, std::chrono::milliseconds(_connectionTimeout)))//填入自己设置的时间即可
		{
			if (_connectionQue.empty()|| !_isRuning)
			{
				ngx_log_stderr(0, "获取空闲连接超时了...获取连接失败!");
				
				return nullptr;
			}
		}
		/*
			开始等待 ：线程调用 cv.wait_for()
			释放锁 ：自动释放 lock 持有的互斥锁
			等待状态 ：线程进入等待，直到：
				1.其他线程调用 cv.notify_one() 或 cv.notify_all() 唤醒
				2.超时时间到达
				3.线程被中断
			重新获取锁 ：返回前自动重新获取互斥锁
			返回状态 ：根据唤醒原因返回相应的 cv_status
		*/

	}
	if (!_isRuning && _connectionQue.empty())
	{
		return nullptr;
	}

	std::shared_ptr<Connection> sp(_connectionQue.front(),
		[&](Connection *pcon) {
		std::unique_lock<std::mutex> lock(_queueMutex);
		if (_isRuning) {
			pcon->refreshAliveTime(); // 刷新一下开始空闲的起始时间
			_connectionQue.push(pcon);
		}
		else {
			delete pcon; // 调用~Connection()释放连接
		}
	});
	_connectionQue.pop();
	cv.notify_all();  // 消费完连接以后，通知生产者线程检查一下，如果队列为空了，赶紧生产连接
	return sp;
}
/*
	MySQLConnectionPool::getConnection()这个函数在不同的线程中调用，如果连接池为空的话，
	会阻塞在wait_for()这个地方，等待超时或者notify_all()，当某个线程执行完毕这个函数，成功取出一个连接后，
	再notify_all()，通知MySQLConnectionPool::produceConnectionTask()线程检查条件变量，创建线程池连接

	1. notify_all() 被调用
		↓
	2. 所有等待线程被唤醒
   		↓
	3. 每个线程重新获取锁
   		↓
	4. 每个线程检查自己的谓词条件
   		↓
	5. 条件满足 → 继续执行
   	   条件不满足 → 重新进入等待
	notify_all() 是 无差别广播，谓词函数 决定线程是否真正继续执行，同一个条件变量可以协调 多种类型的线程
	wait()和wait_for()的区别：
		wait()：线程会一直阻塞，直到被唤醒,底层调用pthread_cond_wait(&cond, &mutex);  // 无超时参数
		wait_for()：线程会阻塞指定的时间，超时后会自动唤醒,
		底层调用pthread_cond_timedwait(&cond, &mutex, &timeout);  // 有超时参数

*/


// 扫描超过maxIdleTime时间的空闲连接，进行对于的连接回收
void MySQLConnectionPool::scannerConnectionTask()
{
	while (_isRuning)
	{
		// 通过sleep模拟定时效果
		std::this_thread::sleep_for(std::chrono::seconds(_maxIdleTime));//定时触发
		// 扫描整个队列，释放多余的连接
		std::unique_lock<std::mutex> lock(_queueMutex);
		while (_connectionCnt > _initSize)
		{
			Connection *p = _connectionQue.front();
			if (p->getAliveeTime() >= (_maxIdleTime * 1000))
			{
				_connectionQue.pop();
				_connectionCnt--;
				delete p; // 调用~Connection()释放连接
			}
			else
			{
				break; // 队头的连接没有超过_maxIdleTime，其它连接肯定没有
			}
		}
	}
	while (!_connectionQue.empty()) {//程序退出清理
		std::unique_lock<std::mutex> lock(_queueMutex);
		Connection *p = _connectionQue.front();
		_connectionQue.pop();
		delete p; // 调用~Connection()释放连接
	}
	cv.notify_all();
}
/*
	std::mutex的用法：
		#include <mutex>
		std::mutex my_mutex; // 创建一个互斥量
		用lock()和unlock()手动加锁和解锁

		{
			// 加锁
			my_mutex.lock();
			// 临界区代码
			// ...
			// 解锁
			my_mutex.unlock();
		}

		不推荐直接使用 lock() 和 unlock()！因为如果在临界区代码中发生异常、或提前返回，可能导致 unlock() 没有被调用，
		从而造成死锁（Deadlock）——锁永远无法被释放，其他线程永远等待。

		推荐用法使用 std::lock_guard（RAII机制）
		{
			std::lock_guard<std::mutex> guard(my_mutex);
			// 临界区代码
			// ...
		}
		// 出作用域时，lock_guard 自动析构，释放锁

		更灵活的用法：std::unique_lock
		{
			std::unique_lock<std::mutex> ulock(my_mutex);
			// 临界区代码
			// ...
		}
		// 出作用域时，unique_lock 自动析构，释放锁

		

*/
