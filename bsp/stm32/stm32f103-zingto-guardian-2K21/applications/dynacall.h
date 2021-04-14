#ifndef __DYNACALL_H__
#define __DYNACALL_H__
#include <rtthread.h>

#if defined(_MSC_VER)
#pragma section("DynaCallTab", read)
#endif

typedef int (*dynamic_func)(void*);

/* dynamic call table */
struct default_dynacall
{
    const char* name;       /* the name of dynamic call */
    dynamic_func func;      /* the function address of dynamic call */
};


#define LAUNCHER_DYNACALL_EXPORT_CMD(name, cmd)                      \
    const char __default_##cmd##_name[] SECTION(".rodata.name") = #cmd;    \
    RT_USED const struct default_dynacall __default_##cmd SECTION("DynaCallTab")= \
        {                                \
            __default_##cmd##_name,     \
            (dynamic_func)&name         \
        };
        
#define LAUNCHER_DYNACALL_EXPORT_CMD_FAST(name) LAUNCHER_DYNACALL_EXPORT_CMD(name, name)

dynamic_func support_dynacall_lookup(const char* name);

#endif
