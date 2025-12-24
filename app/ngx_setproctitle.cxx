#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>  //env
#include <string.h>

#include "ngx_global.h"

//设置可执行程序标题相关函数：分配内存，并且把环境变量拷贝到新内存中来
void ngx_init_setproctitle()
{   
    gp_envmem = new char[g_envneedmem]; 
    memset(gp_envmem,0,g_envneedmem);  //内存要清空防止出现问题

    char *ptmp = gp_envmem;
    //把原来的内存内容搬到新地方来
    for (int i = 0; environ[i]; i++) 
    {
        size_t size = strlen(environ[i])+1 ; //不要拉下+1，否则内存全乱套了，因为strlen是不包括字符串末尾的\0的
        strcpy(ptmp,environ[i]);      //把原环境变量内容拷贝到新地方[新内存]
        environ[i] = ptmp;            //然后还要让新环境变量指向这段新内存
        ptmp += size;
    }
    return;
}

//设置可执行程序标题
void ngx_setproctitle(const char *title)
{  
    size_t ititlelen = strlen(title); 
   
    size_t esy = g_argvneedmem + g_envneedmem; //argv和environ内存总和
    if( esy <= ititlelen)
    {
        return;
    }

    //空间够保存标题的，够长，存得下，继续走下来    

    g_os_argv[1] = NULL;  

    char *ptmp = g_os_argv[0]; //让ptmp指向g_os_argv所指向的内存
    strcpy(ptmp,title);
    ptmp += ititlelen; //跳过标题

    size_t cha = esy - ititlelen;  //内存总和减去标题字符串长度(不含字符串末尾的\0)，剩余的大小，就是要memset的；
    memset(ptmp,0,cha);
    return;
}