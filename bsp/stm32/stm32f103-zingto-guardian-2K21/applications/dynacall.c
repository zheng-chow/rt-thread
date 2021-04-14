#include <rthw.h>
#include <rtthread.h>
#include <string.h>

#include "dynacall.h"

struct default_dynacall *_default_dynacall_table_begin = NULL, *_default_dynacall_table_end = NULL;

int default_dynacall_init(void){
    extern const int DynaCallTab$$Base;
    extern const int DynaCallTab$$Limit;
    _default_dynacall_table_begin = (struct default_dynacall *)&DynaCallTab$$Base;
    _default_dynacall_table_end = (struct default_dynacall *)&DynaCallTab$$Limit;
    
    return 0;
}
INIT_ENV_EXPORT(default_dynacall_init);

dynamic_func support_dynacall_lookup(const char* name)
{
    struct default_dynacall* index;
    //struct finsh_syscall_item* item;

    for (index = _default_dynacall_table_begin; index < _default_dynacall_table_end; index++)
    {
        if (strcmp(index->name, name) == 0)
            return index->func;
    }
    return RT_NULL;
}

static int list_support_dynacall(int argc, char **argv)
{
    rt_kprintf("--Dynamic Function List:\n");
    {
        struct default_dynacall *index;
        for (index = _default_dynacall_table_begin; \
                index < _default_dynacall_table_end; \
                index++)
        {
            /* skip the internal command */
            if (strncmp((char *)index->name, "__", 2) == 0) continue;
            rt_kprintf("%s\n", index->name);
        }
    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(list_support_dynacall, list_dynacall, list all support dynamic call in system)

static int exec_support_dynacall(int argc, char **argv)
{
    const char* call_name = RT_NULL;
    const void* call_param = RT_NULL;
    
    if ( argc == 2)  {
        call_name = argv[1];
    }
    else if (argc == 3) {
        call_name = argv[1];
        call_param = argv[2];
    }
    else {
        rt_kprintf("run_dynacall DYNACALL PARAMETER\n");
        return -RT_EIO;
    }
    
    dynamic_func dynacall = support_dynacall_lookup(call_name);
    if (dynacall == RT_NULL) {
        rt_kprintf("--Can not found %s\n", call_name);
        return -RT_EEMPTY;
    }
    
    if (call_param) return dynacall((void *)call_param);
    else return dynacall(0);
}
MSH_CMD_EXPORT_ALIAS(exec_support_dynacall, exec_dynacall, execute a support dynamic call in system)

//static int DefaultTest(void* parameter)
//{
//    const char *input = (char*)parameter;
//    
//    if (input)
//        rt_kprintf("this is test, parameter: %s\n", input);
//    else
//        rt_kprintf("this is test, no parameter.\n");
//    
//    return 0;
//}
//LAUNCHER_DYNACALL_EXPORT_CMD_FAST(DefaultTest);
