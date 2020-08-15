#ifndef __PALTFORM_COMPATIBLE_H__
#define __PALTFORM_COMPATIBLE_H__
#include <rtthread.h>

typedef rt_bool_t bool;
#define true	RT_TRUE
#define false RT_FALSE
#ifndef NULL
#define NULL RT_NULL
#endif
#define udp_malloc(x)				rt_malloc(x)
#define udp_free(x)					rt_free(x)
#define udp_memset(a,b,c)		rt_memset(a,b,c)
#define udp_memcpy(a,b,c)		rt_memcpy(a,b,c)
#define udp_snprintf				rt_snprintf
#ifndef assert
#define assert(x) RT_ASSERT(x)
#endif

typedef rt_list_t									udp_list_t;
#define udp_list_init(n)					rt_list_init(n)
#define udp_list_entry(a,b,c)			rt_list_entry(a,b,c)
#define udp_list_isempty(n)				rt_list_isempty(n)
#define udp_list_remove(n)				rt_list_remove(n)
#define udp_list_pushback(l,n)		rt_list_insert_before(l,n)
#define udp_list_getfirst(l)			(l)->next


typedef rt_mutex_t					udp_mutex_t;
#define udp_mutex_create(x) rt_mutex_create(x, RT_IPC_FLAG_FIFO)
#define udp_mutex_delete(x)	rt_mutex_delete(x) 
#define udp_mutex_lock(x)		rt_mutex_take(x, RT_WAITING_FOREVER)
#define udp_mutex_unlock(x)	rt_mutex_release(x)

typedef rt_sem_t						udp_sem_t;
#define udp_sem_create(x)		rt_sem_create(x, 0, RT_IPC_FLAG_FIFO)
#define udp_sem_delete(x)		rt_sem_delete(x)
#define udp_sem_get(x,t)		(RT_EOK == rt_sem_take(x, t))
#define udp_sem_set(x)			rt_sem_release(x)



typedef rt_mp_t									udp_mp_t;
#define	udp_mp_create(a,b,c)		rt_mp_create(a,b,c)
#define	udp_mp_delete(x)				rt_mp_delete(x)
#define	udp_mp_malloc(x, t)				rt_mp_alloc(x, t)
#define	udp_mp_free(x)					rt_mp_free(x)


typedef rt_thread_t												udp_thread_handle_t;
//#define udp_thread_create(name, entry, parameter)				rt_thread_create(name, entry, parameter, 2048, 16,  RT_TICK_PER_SECOND/20)
#define udp_thread_create_r(name, entry, parameter)				rt_thread_create(name, entry, parameter, 2048, 16,  RT_TICK_PER_SECOND/20)
#define udp_thread_create_t(name, entry, parameter)				rt_thread_create(name, entry, parameter, 2048, 11,  RT_TICK_PER_SECOND/20)
#define udp_thread_delete(x)									rt_thread_delete(x)
#define udp_thread_start(x)										rt_thread_startup(x)
#define udp_thread_stop(x)										__nop()//do{if (((x)->stat & RT_THREAD_STAT_MASK) == RT_THREAD_CLOSE) break; rt_thread_delay(RT_TICK_PER_SECOND/10);}while(1)//__nop()//rt_thread_detach(x)


#endif

