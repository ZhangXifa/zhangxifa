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
	produce.detach();

	// 启动一个新的定时线程，扫描超过maxIdleTime时间的空闲连接，进行对于的连接回收
	std::thread scanner(std::bind(&MySQLConnectionPool::scannerConnectionTask, this));
	scanner.detach();
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


// 给外部提供接口，从连接池中获取一个可用的空闲连接
std::shared_ptr<Connection> MySQLConnectionPool::getConnection()
{
	std::unique_lock<std::mutex> lock(_queueMutex);
	while (_connectionQue.empty() && _isRuning)
	{
		// sleep
		if (std::cv_status::timeout == cv.wait_for(
			lock, std::chrono::milliseconds(_connectionTimeout)))
		{
			if (_connectionQue.empty()|| !_isRuning)
			{
				ngx_log_stderr(0, "获取空闲连接超时了...获取连接失败!");
				
				return nullptr;
			}
		}

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


// 扫描超过maxIdleTime时间的空闲连接，进行对于的连接回收
void MySQLConnectionPool::scannerConnectionTask()
{
	while (_isRuning)
	{
		// 通过sleep模拟定时效果
		std::this_thread::sleep_for(std::chrono::seconds(_maxIdleTime));
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
	while (!_connectionQue.empty()) {
		std::unique_lock<std::mutex> lock(_queueMutex);
		Connection *p = _connectionQue.front();
		_connectionQue.pop();
		delete p; // 调用~Connection()释放连接
	}
	cv.notify_all();
}
