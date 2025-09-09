#ifndef __NGX_C_CRC32_H__
#define __NGX_C_CRC32_H__

#include <stddef.h>  //NULL

class CCRC32
{
private:
	CCRC32();
public:
	~CCRC32();
private:
	static CCRC32 *m_instance;

public:	
	static CCRC32* GetInstance() 
	{
		if(m_instance == NULL)
		{
			//锁
			if(m_instance == NULL)
			{				
				m_instance = new CCRC32();
				static CGarhuishou cl; 
			}
			//放锁
		}
		return m_instance;
	}	
	class CGarhuishou 
	{
	public:				
		~CGarhuishou()
		{
			if (CCRC32::m_instance)
			{						
				delete CCRC32::m_instance;
				CCRC32::m_instance = NULL;				
			}
		}
	};
	//-------
public:

	void  Init_CRC32_Table();
    unsigned int Reflect(unsigned int ref, char ch); // Reflects CRC bits in the lookup table
    
    int   Get_CRC(unsigned char* buffer, unsigned int dwSize);
    
public:
    unsigned int crc32_table[256]; // Lookup table arrays
};

#endif
/*
	单例类和全局变量的区别:
	1. 全局变量在程序启动时就被创建，而单例类在第一次被使用时才被创建。
	2. 全局变量在程序结束时才被销毁，而单例类在程序结束时会自动调用析构函数销毁。
	3. 全局变量可以在任何地方被访问，而单例类只能在类内部被访问。
	4. 全局变量可以被多个线程同时访问，而单例类只能被一个线程访问。
	5. 全局变量可以被直接访问，而单例类需要通过实例化对象来访问。
*/

