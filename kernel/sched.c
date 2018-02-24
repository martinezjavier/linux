/*
 *  kernel/sched.c
 *
 *  Kernel scheduler and related syscalls
 *
 *  Copyright (C) 1991-2002  Linus Torvalds
 *
 *  1996-12-23  Modified by Dave Grothe to fix bugs in semaphores and
 *		make semaphores SMP safe
 *  1998-11-19	Implemented schedule_timeout() and related stuff
 *		by Andrea Arcangeli
 *  2002-01-04	New ultra-scalable O(1) scheduler by Ingo Molnar:
 *		hybrid priority-list and round-robin design with
 *		an array-switch method of distributing timeslices
 *		and per-CPU runqueues.  Cleanups and useful suggestions
 *		by Davide Libenzi, preemptible kernel bits by Robert Love.
 *  2003-09-03	Interactivity tuning by Con Kolivas.
 *  2004-04-02	Scheduler domains code by Nick Piggin
 *  2005-12-02  Fair Share Scheduling
		por Javier Martinez
		  y Roberto Rodriguez Alcala


 */

#include <linux/mm.h>
#include <linux/module.h>
#include <linux/nmi.h>
#include <linux/init.h>
#include <asm/uaccess.h>
#include <linux/highmem.h>
#include <linux/smp_lock.h>
#include <asm/mmu_context.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/kernel_stat.h>
#include <linux/security.h>
#include <linux/notifier.h>
#include <linux/profile.h>
#include <linux/suspend.h>
#include <linux/blkdev.h>
#include <linux/delay.h>
#include <linux/smp.h>
#include <linux/threads.h>
#include <linux/timer.h>
#include <linux/rcupdate.h>
#include <linux/cpu.h>
#include <linux/cpuset.h>
#include <linux/percpu.h>
#include <linux/kthread.h>
#include <linux/seq_file.h>
#include <linux/syscalls.h>
#include <linux/times.h>

#include <linux/acct.h>
#include <linux/kprobes.h>
#include <asm/tlb.h>
#include <asm/unistd.h>

/*
 * Some helpers for converting nanosecond timing to jiffy resolution
 */
#define NS_TO_JIFFIES(TIME)	((TIME) / (1000000000 / HZ))
#define JIFFIES_TO_NS(TIME)	((TIME) * (1000000000 / HZ))

/*
 * These are the 'tuning knobs' of the scheduler:
 *
 * Minimum timeslice is 5 msecs (or 1 jiffy, whichever is larger),
 * default timeslice is 100 msecs, maximum timeslice is 800 msecs.
 * Timeslices get refilled after they expire.
 */
#define TIMESLICE		(100 * HZ / 1000)
#define MIN_TIMESLICE		max(5 * HZ / 1000, 1)
#define DEF_TIMESLICE		(100 * HZ / 1000)
#define ON_RUNQUEUE_WEIGHT	 30
#define CHILD_PENALTY		 95
#define PARENT_PENALTY		100
#define EXIT_WEIGHT		  3
#define PRIO_BONUS_RATIO	 25
#define MAX_SLEEP_AVG		(DEF_TIMESLICE * MAX_BONUS)
#define STARVATION_LIMIT	(MAX_SLEEP_AVG)
#define NS_MAX_SLEEP_AVG	(JIFFIES_TO_NS(MAX_SLEEP_AVG))

#define CURRENT_BONUS(p) \
	(NS_TO_JIFFIES((p)->sleep_avg) * MAX_BONUS / \
		MAX_SLEEP_AVG)

#define GRANULARITY	(10 * HZ / 1000 ? : 1)

#define SCALE(v1,v1_max,v2_max) \
	(v1) * (v2_max) / (v1_max)

/*
 * Definicion de un elemento de la lista de grupos
 * del calendarizador.
 */
struct grupo_struct
{
	/* ID del Grupo */
	gid_t gid;
	/* Nodo de la lista en ejecucuion */
	struct list_head lista;
	/* Nodo de la lista de todos los grupos */
	struct list_head lista_completa; 
	/* Puntero a la lista completa de usuarios del grupo */
	struct usuario_struct *usuarios_lista_completa;
	/* Puntero a la lista de usuarios del grupo en ejecucion */
	struct usuario_struct *usuarios_lista;
	/* Cantidad de procesos que utilizan el grupo */
	atomic_t ref;
	/* Cantidad de usuarios que se encuentran ejecutando procesos */
	atomic_t usuarios;

	/* Informacion para la estadistica */
#ifdef CONFIG_SCHEDSTATS
	struct sched_info sched_info;
#endif
};

struct usuario_struct
{
	/* ID del Usuario */
	uid_t uid;
	/* Nodo de la lista en ejecucuion */
	struct list_head lista;
	/* Nodo de la lista de todos los usuarios */
	struct list_head lista_completa;
	/* Puntero a la lista de procesos del usuario en ejecucion */
	struct task_struct *procesos_lista;
	/* Cantidad de procesos del usuario en ejecucion */
	atomic_t procesos;
	/* Cantidad de procesos que utilizan el usuario */
	atomic_t ref;
	
	/* Informacion para la estadistica */
#ifdef CONFIG_SCHEDSTATS
	struct sched_info sched_info;
#endif
};

/* Grupo y Usuario 0 (root) utilizado para los hilos del kernel 
   Estos se declaran estaticamente debido a que se utilizan desde
   el principio del arranque */
struct grupo_struct grupo_root;
struct usuario_struct usuario_root;

typedef struct runqueue runqueue_t;
/*
 * This is the main, per-CPU runqueue data structure.
 *
 * Locking rule: those places that want to lock multiple runqueues
 * (such as the load balancing or the thread migration code), lock
 * acquire operations must be ordered by ascending &runqueue.
 */
struct runqueue {
	spinlock_t lock;

	/* Lista enlazada de grupos disponibles para la ejecucion */
	struct grupo_struct *grupos_lista;

	/*
	 * Procesos en ejecucion y cantidad de cambios efectuados.
	 */
	unsigned long nr_running;
	unsigned long long nr_switches;

	/*
	 * This is part of a global counter where only the total sum
	 * over all CPUs matters. A task can increase this counter on
	 * one CPU and if it got migrated afterwards it may decrease
	 * it on another CPU. Always updated under the runqueue lock:
	 */
	unsigned long nr_uninterruptible;

	unsigned long expired_timestamp;
	unsigned long long timestamp_last_tick;
	task_t *curr, *idle;
	struct mm_struct *prev_mm;
	int best_expired_prio;
	atomic_t nr_iowait;
	
	/* Contador de grupos activos */
	atomic_t grupos;

#ifdef CONFIG_SCHEDSTATS
	/* latency stats */
	struct sched_info rq_sched_info;

	/* sys_sched_yield() stats */
	unsigned long yld_exp_empty;
	unsigned long yld_act_empty;
	unsigned long yld_both_empty;
	unsigned long yld_cnt;

	/* schedule() stats */
	unsigned long sched_switch;
	unsigned long sched_cnt;
	unsigned long sched_goidle;

	/* try_to_wake_up() stats */
	unsigned long ttwu_cnt;
	unsigned long ttwu_local;

	/* Bandera de estadisticas en /proc */
	unsigned short stat;
#endif
};

static DEFINE_PER_CPU(struct runqueue, runqueues);

/*
 * The domain tree (rq->sd) is protected by RCU's quiescent state transition.
 * See detach_destroy_domains: synchronize_sched for details.
 *
 * The domain tree of any CPU may only be accessed from within
 * preempt-disabled sections.
 */
#define for_each_domain(cpu, domain) \
for (domain = rcu_dereference(cpu_rq(cpu)->sd); domain; domain = domain->parent)

#define cpu_rq(cpu)		(&per_cpu(runqueues, (cpu)))
#define this_rq()		(&__get_cpu_var(runqueues))
#define task_rq(p)		cpu_rq(task_cpu(p))
#define cpu_curr(cpu)		(cpu_rq(cpu)->curr)

#ifndef prepare_arch_switch
# define prepare_arch_switch(next)	do { } while (0)
#endif
#ifndef finish_arch_switch
# define finish_arch_switch(prev)	do { } while (0)
#endif

#ifndef __ARCH_WANT_UNLOCKED_CTXSW
static inline int task_running(runqueue_t *rq, task_t *p)
{
	return rq->curr == p;
}

static inline void prepare_lock_switch(runqueue_t *rq, task_t *next)
{
}

static inline void finish_lock_switch(runqueue_t *rq, task_t *prev)
{
#ifdef CONFIG_DEBUG_SPINLOCK
	/* this is a valid case when another task releases the spinlock */
	rq->lock.owner = current;
#endif
	spin_unlock_irq(&rq->lock);
}

#else /* __ARCH_WANT_UNLOCKED_CTXSW */
static inline int task_running(runqueue_t *rq, task_t *p)
{
	return rq->curr == p;
}

static inline void prepare_lock_switch(runqueue_t *rq, task_t *next)
{
#ifdef __ARCH_WANT_INTERRUPTS_ON_CTXSW
	spin_unlock_irq(&rq->lock);
#else
	spin_unlock(&rq->lock);
#endif
}

static inline void finish_lock_switch(runqueue_t *rq, task_t *prev)
{

#ifndef __ARCH_WANT_INTERRUPTS_ON_CTXSW
	local_irq_enable();
#endif
}
#endif /* __ARCH_WANT_UNLOCKED_CTXSW */

/* Funcion para decrementar la cantidad de referencias 
   a un grupo en memoria y borrarlo si esta queda en CERO */
static void delete_grupo(struct grupo_struct *grupo)
{
	/* Si no quedan mas referencias eliminar el grupo */
	if(atomic_dec_and_test(&grupo->ref) && grupo != &grupo_root)
	{
		list_del_init(&grupo->lista_completa);
		list_del_init(&grupo->lista);
		kfree(grupo);
	}
}

/* Funcion para decrementar la cantidad de referencias 
   a un usuario en memoria y borrarlo si esta queda en CERO */
static void delete_usuario(struct usuario_struct *usuario)
{
	/* Si no quedan mas referencias eliminar el usuario manteniendo la consistencia de la lista completa */
	if(atomic_dec_and_test(&usuario->ref) && usuario != &usuario_root)
	{
		if(current->grupo->usuarios_lista_completa == usuario)
		 current->grupo->usuarios_lista_completa = list_entry(usuario->lista_completa.next, struct usuario_struct, lista_completa);
		if(current->grupo->usuarios_lista_completa == usuario)
			current->grupo->usuarios_lista_completa = NULL;
		list_del_init(&usuario->lista_completa);
		list_del_init(&usuario->lista);
		kfree(usuario);
	}
}

/*
 * task_rq_lock - lock the runqueue a given task resides on and disable
 * interrupts.  Note the ordering: we can safely lookup the task_rq without
 * explicitly disabling preemption.
 */
static inline runqueue_t *task_rq_lock(task_t *p, unsigned long *flags)
	__acquires(rq->lock)
{
	struct runqueue *rq;

	local_irq_save(*flags);
	rq = this_rq();
	spin_lock(&rq->lock);
	return rq;
}

static inline void task_rq_unlock(runqueue_t *rq, unsigned long *flags)
	__releases(rq->lock)
{
	spin_unlock_irqrestore(&rq->lock, *flags);
}

#ifdef CONFIG_SCHEDSTATS

/*
 * Imprimir un informe acerca de los procesos del sistema segun las listas del calendarizador
 *
 */
static int show_schedstat(struct seq_file *seq, void *v)
{
	
	runqueue_t *rq = this_rq();
	
	rq->stat = 1;

	struct task_struct *tsk;

	seq_printf(seq, "ESTADISTICAS | Fair share Scheduler\n");
	if (rq->stat == 1){
		for_each_process(tsk)
		{
			seq_printf(seq,"%s\tPID: %d\tUID: %d\tGID: %d\tEst: %s\tE=%lu\tD=%lu\n",tsk->comm,tsk->pid,tsk->usuario->uid,tsk->grupo->gid,tsk->state ? "DURM" : "EJEC",((tsk->sched_info.cpu_time)*1000)/(tsk->sched_info.cpu_time+tsk->sched_info.run_delay),((tsk->sched_info.run_delay)*1000)/(tsk->sched_info.cpu_time+tsk->sched_info.run_delay));
		}
	}else{
		seq_printf(seq,"No esta habilitada la opcion de estadisticas\n");
	}

	return 0;
}


static void cerar_stat(void)
{
	
	runqueue_t *rq = this_rq();
	
	rq->stat = 1;

	struct task_struct *tsk;
	preempt_disable();
	for_each_process(tsk)
	{
		tsk->sched_info.run_delay = 1;
		tsk->sched_info.cpu_time = 0;
		tsk->usuario->sched_info.run_delay = 1;
		tsk->usuario->sched_info.cpu_time = 0;
		
		tsk->grupo->sched_info.run_delay = 1;
		tsk->grupo->sched_info.cpu_time = 0;
	}
	preempt_enable();
}
/*
* Apertura del archivo de estadisticas,
*/
static int schedstat_open(struct inode *inode, struct file *file)
{
	unsigned int size = PAGE_SIZE*4;
	char *buf = kmalloc(size, GFP_KERNEL);
	struct seq_file *m;
	int res;

	if (!buf)
		return -ENOMEM;
	res = single_open(file, show_schedstat, NULL);
	if (!res) {
		m = file->private_data;
		m->buf = buf;
		m->size = size;
	} else
		kfree(buf);
	return res;
}

struct file_operations proc_schedstat_operations = {
	.open    = schedstat_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
};


static int show_runstat(struct seq_file *seq, void *v)
{

	runqueue_t *rq = this_rq();

	struct task_struct *tsk;
	struct grupo_struct *grp;
	struct usuario_struct *usr;

	struct list_head *task_head;
	struct list_head *usuario_head;
	struct list_head *grupo_head;

	int i,j,k;

	seq_printf(seq, "ESTADISTICAS | Fair share Scheduler | En ejecucion\n");

	if(rq->stat == 1){
		if(likely(rq->grupos_lista)){
			preempt_disable();

			grupo_head = &rq->grupos_lista->lista;
			for(i=0;i<atomic_read(&rq->grupos);i++){
				grp = list_entry(grupo_head,struct grupo_struct, lista);
				seq_printf(seq,"Grupo   %5d E=%lu D=%lu\n", grp->gid,((grp->sched_info.cpu_time)*10000)/(grp->sched_info.cpu_time+grp->sched_info.run_delay), ((grp->sched_info.run_delay)*10000)/(grp->sched_info.cpu_time+grp->sched_info.run_delay));
				
				usuario_head = &grp->usuarios_lista->lista;
				for(j=0;j<atomic_read(&grp->usuarios);j++){
					usr = list_entry(usuario_head,struct usuario_struct, lista);
					seq_printf(seq,"Usuario %5d E=%lu D=%lu\n", usr->uid,((usr->sched_info.cpu_time)*10000)/(usr->sched_info.cpu_time+usr->sched_info.run_delay), ((usr->sched_info.run_delay)*10000)/(usr->sched_info.cpu_time+usr->sched_info.run_delay));
				
					task_head = &usr->procesos_lista->run_list;
					for(k=0;k<atomic_read(&usr->procesos);k++){
						tsk = list_entry(task_head,struct task_struct, run_list);	
						seq_printf(seq,"%s\tPID: %5d\tUID: %d\tGID: %d\tEstado: %s\tE=%lu\tD=%lu \n",tsk->comm,tsk->pid,tsk->usuario->uid,tsk->grupo->gid,tsk->state ? "BLOQ" : "EJEC", ((tsk->sched_info.cpu_time)*10000)/(tsk->sched_info.cpu_time+tsk->sched_info.run_delay), ((tsk->sched_info.run_delay)*10000)/(tsk->sched_info.cpu_time+tsk->sched_info.run_delay));	
						task_head = task_head->next;
					}
					usuario_head = usuario_head->next;	
				}	
				grupo_head = grupo_head->next;
			}
			preempt_enable();
		}else
			seq_printf(seq,"No hay procesos en ejecucion\n");
	}else{
		seq_printf(seq,"No esta habilitada la opcion de estadisticas\n");	
	}

	return 0;
}

/*
* Apertura del archivo de estadisticas,
*/
static int runstat_open(struct inode *inode, struct file *file)
{
	unsigned int size = PAGE_SIZE*4;
	char *buf = kmalloc(size, GFP_KERNEL);
	struct seq_file *m;
	int res;

	if (!buf)
		return -ENOMEM;
	res = single_open(file, show_runstat, NULL);
	if (!res) {
		m = file->private_data;
		m->buf = buf;
		m->size = size;
	} else
		kfree(buf);
	return res;
}

struct file_operations proc_runstat_operations = {
	.open    = runstat_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
};

# define schedstat_inc(rq, field)	do { (rq)->field++; } while (0)
# define schedstat_add(rq, field, amt)	do { (rq)->field += (amt); } while (0)
#else /* !CONFIG_SCHEDSTATS */
# define schedstat_inc(rq, field)	do { } while (0)
# define schedstat_add(rq, field, amt)	do { } while (0)
#endif

/*
 * rq_lock - lock a given runqueue and disable interrupts.
 */
static inline runqueue_t *this_rq_lock(void)
	__acquires(rq->lock)
{
	runqueue_t *rq;

	local_irq_disable();
	rq = this_rq();
	spin_lock(&rq->lock);

	return rq;
}

#ifdef CONFIG_SCHEDSTATS
/*
 * Called when a process is dequeued from the active arr and given
 * the cpu.  We should note that with the exception of interactive
 * tasks, the expired queue will become the active queue after the active
 * queue is empty, without explicitly dequeuing and requeuing tasks in the
 * expired queue.  (Interactive tasks may be requeued directly to the
 * active queue, thus delaying tasks in the expired queue from running;
 * see scheduler_tick()).
 *
 * This function is only called from sched_info_arrive(), rather than
 * dequeue_task(). Even though a task may be queued and dequeued multiple
 * times as it is shuffled about, we're really interested in knowing how
 * long it was from the *first* time it was queued to the time that it
 * finally hit a cpu.
 */
static inline void sched_info_dequeued(task_t *t)
{
	t->sched_info.last_queued = jiffies;
	t->usuario->sched_info.last_queued = jiffies;
	t->grupo->sched_info.last_queued = jiffies;
}

/*
 * Called when a task finally hits the cpu.  We can now calculate how
 * long it was waiting to run.  We also note when it began so that we
 * can keep stats on how long its timeslice is.
 */
static inline void sched_info_arrive(task_t *t)
{
	unsigned long now = jiffies, diff = 0;
	struct runqueue *rq = task_rq(t);

	if (t->sched_info.last_queued)
		diff = now - t->sched_info.last_queued;
	
	sched_info_dequeued(t);
	
	t->sched_info.run_delay += diff;
	t->usuario->sched_info.run_delay += diff; 
	t->grupo->sched_info.run_delay += diff;
	
	t->sched_info.last_arrival = now;
	t->usuario->sched_info.last_arrival = now;
	t->grupo->sched_info.last_arrival = now;
	
	t->sched_info.pcnt++;
	t->usuario->sched_info.pcnt++;
	t->grupo->sched_info.pcnt++;

	if (!rq)
		return;
}

/*
 * Called when a process is 
 into either the active or expired
 * arr.  The time is noted and later used to determine how long we
 * had to wait for us to reach the cpu.  Since the expired queue will
 * become the active queue after active queue is empty, without dequeuing
 * and requeuing any tasks, we are interested in queuing to either. It
 * is unusual but not impossible for tasks to be dequeued and immediately
 * requeued in the same or another arr: this can happen in sched_yield(),
 * set_user_nice(), and even load_balance() as it moves tasks from runqueue
 * to runqueue.
 *
 * This function is only called from enqueue_task(), but also only updates
 * the timestamp if it is already not set.  It's assumed that
 * sched_info_dequeued() will clear that stamp when appropriate.
 */
static inline void sched_info_queued(task_t *t)
{
	if (!t->sched_info.last_queued){
		t->sched_info.last_queued = jiffies;
		t->usuario->sched_info.last_queued = jiffies;
		t->grupo->sched_info.last_queued = jiffies;
	}
}

/*
 * Called when a process ceases being the active-running process, either
 * voluntarily or involuntarily.  Now we can calculate how long we ran.
 */
static inline void sched_info_depart(task_t *t)
{
	unsigned long diff = jiffies - t->sched_info.last_arrival;

	t->sched_info.cpu_time += diff;
	t->usuario->sched_info.cpu_time += diff;
	t->grupo->sched_info.cpu_time += diff;
}

/*
 * Called when tasks are switched involuntarily due, typically, to expiring
 * their time slice.  (This may also be called when switching to or from
 * the idle task.)  We are only called when prev != next.
 */
static inline void sched_info_switch(task_t *prev, task_t *next)
{
	struct runqueue *rq = task_rq(prev);

	/*
	 * prev now departs the cpu.  It's not interesting to record
	 * stats about how efficient we were at scheduling the idle
	 * process, however.
	 */
	if (prev != rq->idle)
		sched_info_depart(prev);
	
	if (next != rq->idle && prev != next)
		sched_info_arrive(next);
}
#else
#define sched_info_queued(t)		do { } while (0)
#define sched_info_switch(t, next)	do { } while (0)
#endif /* CONFIG_SCHEDSTATS */

/*
 * Crea, inicializa y mantiene los usuarios y la cantidad de
 * referencias a estos(para mejorar el uso de memoria)
 * nuid = UID del grupo solicitado
 * Retorno: Puntero al grupo solicitado.
 */
static struct usuario_struct *init_usuario(int nuid)
{
	/* SI es root retornar la estructura estatica */
	if(nuid == 0 && current->gid == 0)
	{
		atomic_inc(&usuario_root.ref);
		return &usuario_root;
	}
	/* Si la lista completa del grupo esta vacia proceder directamente a crear
	   sino buscar antes a ver si ya exite el usuario */
	if(current->grupo->usuarios_lista_completa != NULL)
	{
		/* Ver si ya existe la estructura */
		struct list_head *tmp;
		struct usuario_struct *grp;
		if(current->grupo->usuarios_lista_completa->uid == nuid)
		{
			atomic_inc(&current->grupo->usuarios_lista_completa->ref);
			return current->grupo->usuarios_lista_completa;
		}

		list_for_each(tmp,&current->grupo->usuarios_lista_completa->lista_completa)
		{
			grp = list_entry(tmp,struct usuario_struct, lista_completa);
			if(grp->uid == nuid){
					atomic_inc(&grp->ref);
					return grp;
			}
		}
	}
	/* Si llegamos hasta aca es porque la estructura de este usuario no existe */
	struct usuario_struct *nueva = kmalloc(sizeof(struct usuario_struct),GFP_KERNEL);
	if(nueva)
	{
		INIT_LIST_HEAD(&nueva->lista);
		nueva->procesos_lista = NULL;
		nueva->uid = nuid;
		atomic_set( &nueva->procesos, 0);
		INIT_LIST_HEAD(&nueva->lista_completa);
#ifdef CONFIG_SCHEDSTATS
		memset(&nueva->sched_info, 0, sizeof(struct sched_info));
		nueva->sched_info.run_delay = 1;
#endif
		if(current->grupo->usuarios_lista_completa != NULL)
			list_add_tail(&nueva->lista_completa,&current->grupo->usuarios_lista_completa->lista_completa);
		else
			current->grupo->usuarios_lista_completa = nueva;
	}
	else
		panic("No se pudo obtener memoria para crear un usuario.");
	
	atomic_inc(&nueva->ref);
	return nueva;
}

EXPORT_SYMBOL(init_usuario);

/*
 * Crea, inicializa y mantiene los grupos y la cantidad de
 * referencias a estos(para mejorar el uso de memoria)
 * ngid = GID del grupo solicitado
 */
static struct grupo_struct *init_grupo(int ngid)
{
	/* Si se solicita root retornar la direccion de la estructura estatica */
	if(ngid == 0){
		atomic_inc(&grupo_root.ref);
		return &grupo_root;
	}

	struct list_head *tmp;
	struct grupo_struct *grp;

	/* Ver si ya existe la estructura */
	list_for_each(tmp,&grupo_root.lista_completa)
	{
		grp = list_entry(tmp,struct grupo_struct, lista_completa);
		if(grp->gid == ngid){
			atomic_inc(&grp->ref);
			return grp;
		}
	}

	/* Si llegamos hasta aca es porque la estructura de este grupo no existe */
	struct grupo_struct *nueva = kmalloc(sizeof(struct grupo_struct),GFP_KERNEL);
	if(nueva)
	{
		nueva->gid = ngid;
		
		INIT_LIST_HEAD(&nueva->lista);
		nueva->usuarios_lista = NULL;
		atomic_set(&nueva->usuarios, 0);
	
		INIT_LIST_HEAD(&nueva->lista_completa);
		list_add_tail(&nueva->lista_completa,&grupo_root.lista_completa);
#ifdef CONFIG_SCHEDSTATS
		memset(&nueva->sched_info, 0, sizeof(struct sched_info));
		nueva->sched_info.run_delay = 1;
#endif
		nueva->usuarios_lista_completa = NULL;
		atomic_set(&nueva->ref, 1);
	
	}else
		panic("No se pudo obtener memoria para crear un grupo.");
	return nueva;
}

EXPORT_SYMBOL(init_grupo);


/*
 *  Eliminar de un proceso de la lista de ejecucion.
 *
 *  La eliminacion de un proceso de la lista de ejecucin, se hace en tres partes
 *  Primero se borra el proceso de la lista de ejecucion
 *  Segundo se verifica si es el ultimo proceso del usuario y de ser asi se elimina el usuario
 *  Por ultimo se verifica si es el ultimo usuario del grupo y de ser asi se elimina el grupo
 */
static void dequeue_task(struct task_struct *p)
{
	runqueue_t *rq = this_rq();

	struct grupo_struct *grupo_actual = p->grupo;
	struct usuario_struct *user_actual = p->usuario;

	/* Si el proceso actual es la cabeza de la lista del usuario, correr la lista en uno */
	if(user_actual->procesos_lista == p)
		user_actual->procesos_lista = list_entry(p->run_list.next, struct task_struct, run_list);

	list_del_init(&p->run_list);

	/* Verificar que queden procesos en la lista del usuario */
	if(atomic_dec_and_test(&user_actual->procesos))
	{
		if(grupo_actual->usuarios_lista == user_actual)
			grupo_actual->usuarios_lista = list_entry(user_actual->lista.next,struct usuario_struct,lista);
		
		list_del_init(&user_actual->lista);
		user_actual->procesos_lista = NULL;
		
		if(atomic_dec_and_test(&grupo_actual->usuarios)){
			if(rq->grupos_lista == grupo_actual)
				rq->grupos_lista = list_entry(grupo_actual->lista.next,struct grupo_struct,lista);
			
			list_del_init(&grupo_actual->lista);
			grupo_actual->usuarios_lista = NULL;
			atomic_dec(&rq->grupos);
			/* En caso de que no queden mas procesos apuntar la lista de grupos a null
			   para avisar al calendarizador que debe seleccionar  a "IDLE" */
			if(rq->grupos_lista == grupo_actual)
				rq->grupos_lista = NULL;
		}
	}
}

/*
 * Insercion de un proceso a la lista de ejecucion
 *
 * Para encolar un proceso, primero se verifica si su grupo ya esta en la lista de grupos
 * y de no ser asi se inserta, luego se ejectua la misma verificacion para el usuario, 
 * por ultimo insertamos el proceso a la lista de ejecucion 
 */

static void enqueue_task(struct task_struct *p)
{

	struct grupo_struct *grupo_head, *grupo_actual = p->grupo;
	struct usuario_struct *user_actual = p->usuario;

	runqueue_t *rq = this_rq();

	/* Si no habia ningun proceso inicializar la lista completa */
	if(rq->grupos_lista == NULL)
	{
		rq->grupos_lista = grupo_actual;
		rq->grupos_lista->usuarios_lista = user_actual;
		rq->grupos_lista->usuarios_lista->procesos_lista = p;
		atomic_set(&rq->grupos,1);
		atomic_set(&rq->grupos_lista->usuarios_lista->procesos,1);
		atomic_set(&rq->grupos_lista->usuarios,1);
		return;
	}

	grupo_head = rq->grupos_lista;

	/* Si la lista que queres meter esta vacia y no sos la cabeza a no ser que sea el unico grupo en ejecucion */
	if(list_empty(&grupo_actual->lista) && rq->grupos_lista != grupo_actual){
		list_add_tail(&grupo_actual->lista,&grupo_head->lista);
		atomic_inc(&rq->grupos);
	}
	
	if(unlikely(grupo_actual->usuarios_lista == NULL))
	{
		grupo_actual->usuarios_lista = user_actual;
		atomic_set(&grupo_actual->usuarios,1);
	}
	else if(list_empty(&user_actual->lista) && grupo_actual->usuarios_lista != user_actual)
	{	/* Si la lista que queres meter esta vacia y no sos la cabeza a no ser que sea el unico usuario en ejecucion */
		list_add_tail(&user_actual->lista, &grupo_actual->usuarios_lista->lista);
		atomic_inc(&grupo_actual->usuarios);
	}
	
	if(unlikely(user_actual->procesos_lista == NULL))
	{
		user_actual->procesos_lista = p;
	}
	else
		list_add_tail(&p->run_list,&user_actual->procesos_lista->run_list);
	
	atomic_inc(&user_actual->procesos);
}

/* Inserta el proceso a las estructuras del calendarizador
 * e incrementa la variable de procesos en ejecucion. */
static inline void __activate_task(task_t *p, runqueue_t *rq)
{
	enqueue_task(p);
	rq->nr_running++;
}
/*
 * activate_task ---
 * Actualiza la marca de tiempo del proceso al tiempo actual
 * y coloca la tarea dentro de la estructura del calendarizador
 */
static void activate_task(task_t *p, runqueue_t *rq)
{
	p->timestamp = sched_clock();
	__activate_task(p, rq);
}

/*
 * deactivate_task ---
 * Decrementa la cantidad de procesos en ejecucion y quita al proceso
 * solicitado de las estructuras del calendarizador.
 */
static void deactivate_task(struct task_struct *p, runqueue_t *rq)
{
	rq->nr_running--;
	dequeue_task(p);
}

/*
 * resched_task - mark a task 'to be rescheduled now'.
 */
static inline void resched_task(task_t *p)
{
	set_tsk_need_resched(p);
}

/**
 * task_curr - is this task currently executing on a CPU?
 * @p: the task in question.
 */
inline int task_curr(const task_t *p)
{
	return cpu_curr(task_cpu(p)) == p;
}


/*
 * wake_idle() will wake a task on an idle cpu if task->cpu is
 * not idle and an idle cpu is available.  The span of cpus to
 * search starts with cpus closest then further out as needed,
 * so we always favor a closer, idle cpu.
 *
 * Returns the CPU we should wake onto.
 */
#if defined(ARCH_HAS_SCHED_WAKE_IDLE)
static int wake_idle(int cpu, task_t *p)
{
	cpumask_t tmp;
	struct sched_domain *sd;
	int i;

	if (idle_cpu(cpu))
		return cpu;

	for_each_domain(cpu, sd) {
		if (sd->flags & SD_WAKE_IDLE) {
			cpus_and(tmp, sd->span, p->cpus_allowed);
			for_each_cpu_mask(i, tmp) {
				if (idle_cpu(i))
					return i;
			}
		}
		else
			break;
	}
	return cpu;
}
#else
static inline int wake_idle(int cpu, task_t *p)
{
	return cpu;
}
#endif

/***
 * try_to_wake_up - wake up a thread
 * @p: the to-be-woken-up thread
 * @state: the mask of task states that can be woken
 * @sync: do a synchronous wakeup?
 *
 * Put it on the run-queue if it's not already there. The "current"
 * thread is always on the run-queue (except when the actual
 * re-schedule is in progress), and as such you're allowed to do
 * the simpler "current->state = TASK_RUNNING" to mark yourself
 * runnable without the overhead of this.
 *
 * returns failure only if the task is already active.
 */
static int try_to_wake_up(task_t *p, unsigned int state, int sync)
{
	int cpu, this_cpu, success = 0;
	unsigned long flags;
	long old_state;
	runqueue_t *rq;

	rq = task_rq_lock(p, &flags);
	old_state = p->state;

	/* Si la tarea ya estaba, tranquilo */
	if (!(old_state & state))
		goto out;

	cpu = task_cpu(p);
	this_cpu = smp_processor_id();

	if (old_state == TASK_UNINTERRUPTIBLE) 
	{
		rq->nr_uninterruptible--;
		p->activated = -1;
	}
	
	p->state = TASK_RUNNING;
	if(list_empty(&(p->run_list)) && p->usuario->procesos_lista != p)
	{
		/* Adicionar a la tarea una fraccion mas de tiempo mientras que
		   posea menos de dos fracciones, esto se hace para mejorar
		   la justicia con procesos que duermen */
		if(unlikely(p->time_slice >= (2 * DEF_TIMESLICE)))
			p->time_slice = (2 * DEF_TIMESLICE);
		else
			p->time_slice += DEF_TIMESLICE;
		if (old_state & TASK_NONINTERACTIVE)
			__activate_task(p, rq);
		else
			activate_task(p, rq);
	}
	/*
	 * Sync wakeups (i.e. those types of wakeups where the waker
	 * has indicated that it will leave the CPU in short order)
	 * don't trigger a preemption, if the woken up task will run on
	 * this cpu. (in this case the 'I will reschedule' promise of
	 * the waker guarantees that the freshly woken up task is going
	 * to be considered on this CPU.)
	 */
	if (!sync || cpu != this_cpu) {
		resched_task(rq->curr);
	}
	success = 1;
	
out:
	task_rq_unlock(rq, &flags);

	return success;
}

int fastcall wake_up_process(task_t *p)
{
	return try_to_wake_up(p, TASK_STOPPED | TASK_TRACED |
				 TASK_INTERRUPTIBLE | TASK_UNINTERRUPTIBLE, 0);
}

EXPORT_SYMBOL(wake_up_process);

int fastcall wake_up_state(task_t *p, unsigned int state)
{
	return try_to_wake_up(p, state, 0);
}


/*
 * Realiza las inicializaciones referentes al scheduler en un proceso (current)
 */
void fastcall sched_fork(task_t *p, int clone_flags)
{
	int cpu = get_cpu();
	set_task_cpu(p, cpu);

	/* Inicializacion del nodo para la lista de procesos del usuario */
	INIT_LIST_HEAD(&p->run_list);
	p->static_prio = 0;

	/* Asignacion del primer timeslice */
	p->first_time_slice = 1;
	p->time_slice = DEF_TIMESLICE;
#ifdef CONFIG_SCHEDSTATS
	memset(&p->sched_info, 0, sizeof(p->sched_info));
	p->sched_info.run_delay = 1;
#endif
#ifdef CONFIG_PREEMPT
	/* Want to start with kernel preemption disabled. */
	p->thread_info->preempt_count = 1;
#endif
	/*
	 * Asignacion de grupo y usuario correspondiente al proceso
	 */
	p->grupo = init_grupo(p->gid);
	p->usuario = init_usuario(p->uid);
	/* Marcar en ejecucion */
	p->state = TASK_RUNNING;

	local_irq_disable();
	/* Timestamp es long long y requiere dos instrucciones para la asignacion */
	p->timestamp = sched_clock();
	local_irq_enable();

	put_cpu();
}

/*
 * wake_up_new_task - wake up a newly created task for the first time.
 *
 * This function will do some initial scheduler statistics housekeeping
 * that must be done for every newly created context, then puts the task
 * on the runqueue and wakes it.
 */
void fastcall wake_up_new_task(task_t *p, unsigned long clone_flags)
{
	unsigned long flags;
	runqueue_t *rq;

	rq = task_rq_lock(p, &flags);
	BUG_ON(p->state != TASK_RUNNING);

	p->sleep_avg = 0;

	__activate_task(p, rq);

	task_rq_unlock(rq, &flags);
}

/*
 * Limpieza de referencias y posible memoria utilizada
 * por un proceso que acaba de terminar.
 * Basicamente decrementar las referencias en el grupo
 * y usuario, y liberar memoria si corresponde.
 */
void fastcall sched_exit(task_t *p)
{
#ifdef CONFIG_SCHEDSTAT
	p->usuario->sched_info.run_delay -= p->sched_info.run_delay;
	p->usuario->sched_info.cpu_time -= p->sched_info.cpu_time;
	p->grupo->sched_info.run_delay -= p->sched_info.run_delay;
	p->grupo->sched_info.cpu_time -= p->sched_info.cpu_time;
	if(p->grupo->sched_info.run_delay <= 0) p->grupo->sched_info.run_delay =1;
	if(p->usuario->sched_info.run_delay <= 0) p->usuario->sched_info.run_delay =1;
#endif
	delete_usuario(p->usuario);
	delete_grupo(p->grupo);
	
}

/**
 * prepare_task_switch - prepare to switch tasks
 * @rq: the runqueue preparing to switch
 * @next: the task we are going to switch to.
 *
 * This is called with the rq lock held and interrupts off. It must
 * be paired with a subsequent finish_task_switch after the context
 * switch.
 *
 * prepare_task_switch sets up locking and calls architecture specific
 * hooks.
 */
static inline void prepare_task_switch(runqueue_t *rq, task_t *next)
{
	prepare_lock_switch(rq, next);
	prepare_arch_switch(next);
}

/**
 * finish_task_switch - clean up after a task-switch
 * @rq: runqueue associated with task-switch
 * @prev: the thread we just switched away from.
 *
 * finish_task_switch must be called after the context switch, paired
 * with a prepare_task_switch call before the context switch.
 * finish_task_switch will reconcile locking set up by prepare_task_switch,
 * and do any other architecture-specific cleanup actions.
 *
 * Note that we may have delayed dropping an mm in context_switch(). If
 * so, we finish that here outside of the runqueue lock.  (Doing it
 * with the lock held can cause deadlocks; see schedule() for
 * details.)
 */
static inline void finish_task_switch(runqueue_t *rq, task_t *prev)
	__releases(rq->lock)
{
	struct mm_struct *mm = rq->prev_mm;
	unsigned long prev_task_flags;

	rq->prev_mm = NULL;

	/*
	 * A task struct has one reference for the use as "current".
	 * If a task dies, then it sets EXIT_ZOMBIE in tsk->exit_state and
	 * calls schedule one last time. The schedule call will never return,
	 * and the scheduled task must drop that reference.
	 * The test for EXIT_ZOMBIE must occur while the runqueue locks are
	 * still held, otherwise prev could be scheduled on another cpu, die
	 * there before we look at prev->state, and then the reference would
	 * be dropped twice.
	 *		Manfred Spraul <manfred@colorfullife.com>
	 */
	prev_task_flags = prev->flags;
	finish_arch_switch(prev);
	finish_lock_switch(rq, prev);
	if (mm)
		mmdrop(mm);
	if (unlikely(prev_task_flags & PF_DEAD))
		put_task_struct(prev);
}

/**
 * schedule_tail - first thing a freshly forked thread must call.
 * @prev: the thread we just switched away from.
 */
asmlinkage void schedule_tail(task_t *prev)
	__releases(rq->lock)
{
	runqueue_t *rq = this_rq();
	finish_task_switch(rq, prev);
#ifdef __ARCH_WANT_UNLOCKED_CTXSW
	/* In this case, finish_task_switch does not reenable preemption */
	preempt_enable();
#endif
	if (current->set_child_tid)
		put_user(current->pid, current->set_child_tid);
}

/*
 * context_switch - switch to the new MM and the new
 * thread's register state.
 */
static inline
task_t * context_switch(runqueue_t *rq, task_t *prev, task_t *next)
{
	struct mm_struct *mm = next->mm;
	struct mm_struct *oldmm = prev->active_mm;

	if (unlikely(!mm)) {
		next->active_mm = oldmm;
		atomic_inc(&oldmm->mm_count);
		enter_lazy_tlb(oldmm, next);
	} else
		switch_mm(oldmm, mm, next);

	if (unlikely(!prev->mm)) {
		prev->active_mm = NULL;
		WARN_ON(rq->prev_mm);
		rq->prev_mm = oldmm;
	}

	/* Here we just switch the register state and the stack. */
	switch_to(prev, next, prev);

	return prev;
}

/*
 * nr_running, nr_uninterruptible and nr_context_switches:
 *
 * externally visible scheduler statistics: current number of runnable
 * threads, current number of uninterruptible-sleeping threads, total
 * number of context switches performed since bootup.
 */
unsigned long nr_running(void)
{
	unsigned long i, sum = 0;

	for_each_online_cpu(i)
		sum += cpu_rq(i)->nr_running;

	return sum;
}

unsigned long nr_uninterruptible(void)
{
	unsigned long i, sum = 0;

	for_each_cpu(i)
		sum += cpu_rq(i)->nr_uninterruptible;

	/*
	 * Since we read the counters lockless, it might be slightly
	 * inaccurate. Do not allow it to go below zero though:
	 */
	if (unlikely((long)sum < 0))
		sum = 0;

	return sum;
}

unsigned long long nr_context_switches(void)
{
	unsigned long long i, sum = 0;

	for_each_cpu(i)
		sum += cpu_rq(i)->nr_switches;

	return sum;
}

unsigned long nr_iowait(void)
{
	unsigned long i, sum = 0;

	for_each_cpu(i)
		sum += atomic_read(&cpu_rq(i)->nr_iowait);

	return sum;
}

/*
 * on UP we do not need to balance between CPUs:
 */
static inline void rebalance_tick(int cpu, runqueue_t *rq, enum idle_type idle)
{
}
static inline void idle_balance(int cpu, runqueue_t *rq)
{
}

static inline int wake_priority_sleeper(runqueue_t *rq)
{
	return 0;
}

DEFINE_PER_CPU(struct kernel_stat, kstat);

EXPORT_PER_CPU_SYMBOL(kstat);

/*
 * This is called on clock ticks and on context switches.
 * Bank in p->sched_time the ns elapsed since the last tick or switch.
 */
static inline void update_cpu_clock(task_t *p, runqueue_t *rq,
				    unsigned long long now)
{
	unsigned long long last = max(p->timestamp, rq->timestamp_last_tick);
	p->sched_time += now - last;
}

/*
 * Return current->sched_time plus any more ns on the sched_clock
 * that have not yet been banked.
 */
unsigned long long current_sched_time(const task_t *tsk)
{
	unsigned long long ns;
	unsigned long flags;
	local_irq_save(flags);
	ns = max(tsk->timestamp, task_rq(tsk)->timestamp_last_tick);
	ns = tsk->sched_time + (sched_clock() - ns);
	local_irq_restore(flags);
	return ns;
}

/*
 *
 * To guarantee that this does not starve expired tasks we ignore the
 * interactivity of a task if the first expired task had to wait more
 * than a 'reasonable' amount of time. This deadline timeout is
 * increasing number of running tasks. We also ignore the interactivity
 * if a better static_prio task has expired:
 */
#define EXPIRED_STARVING(rq) \
	((STARVATION_LIMIT && ((rq)->expired_timestamp && \
		(jiffies - (rq)->expired_timestamp >= \
			STARVATION_LIMIT * ((rq)->nr_running) + 1))) || \
			((rq)->curr->static_prio > (rq)->best_expired_prio))

/*
 * Account user cpu time to a process.
 * @p: the process that the cpu time gets accounted to
 * @hardirq_offset: the offset to subtract from hardirq_count()
 * @cputime: the cpu time spent in user space since the last update
 */
void account_user_time(struct task_struct *p, cputime_t cputime)
{
	struct cpu_usage_stat *cpustat = &kstat_this_cpu.cpustat;
	cputime64_t tmp;

	p->utime = cputime_add(p->utime, cputime);

	/* Add user time to cpustat. */
	tmp = cputime_to_cputime64(cputime);
	/*if (TASK_NICE(p) > 0) Ya no existe mas el tema de nice.
		cpustat->nice = cputime64_add(cpustat->nice, tmp);
	else*/
		cpustat->user = cputime64_add(cpustat->user, tmp);
}

/*
 * Account system cpu time to a process.
 * @p: the process that the cpu time gets accounted to
 * @hardirq_offset: the offset to subtract from hardirq_count()
 * @cputime: the cpu time spent in kernel space since the last update
 */
void account_system_time(struct task_struct *p, int hardirq_offset,
			 cputime_t cputime)
{
	struct cpu_usage_stat *cpustat = &kstat_this_cpu.cpustat;
	runqueue_t *rq = this_rq();
	cputime64_t tmp;

	p->stime = cputime_add(p->stime, cputime);

	/* Add system time to cpustat. */
	tmp = cputime_to_cputime64(cputime);
	if (hardirq_count() - hardirq_offset)
		cpustat->irq = cputime64_add(cpustat->irq, tmp);
	else if (softirq_count())
		cpustat->softirq = cputime64_add(cpustat->softirq, tmp);
	else if (p != rq->idle)
		cpustat->system = cputime64_add(cpustat->system, tmp);
	else if (atomic_read(&rq->nr_iowait) > 0)
		cpustat->iowait = cputime64_add(cpustat->iowait, tmp);
	else
		cpustat->idle = cputime64_add(cpustat->idle, tmp);
	/* Account for system time used */
	acct_update_integrals(p);
	/* Update rss highwater mark */
	update_mem_hiwater(p);
}

/*
 * Account for involuntary wait time.
 * @p: the process from which the cpu time has been stolen
 * @steal: the cpu time spent in involuntary wait
 */
void account_steal_time(struct task_struct *p, cputime_t steal)
{
	struct cpu_usage_stat *cpustat = &kstat_this_cpu.cpustat;
	cputime64_t tmp = cputime_to_cputime64(steal);
	runqueue_t *rq = this_rq();

	if (p == rq->idle) {
		p->stime = cputime_add(p->stime, steal);
		if (atomic_read(&rq->nr_iowait) > 0)
			cpustat->iowait = cputime64_add(cpustat->iowait, tmp);
		else
			cpustat->idle = cputime64_add(cpustat->idle, tmp);
	} else
		cpustat->steal = cputime64_add(cpustat->steal, tmp);
}

/*
 * This function gets called by the timer code, with HZ frequency.
 * We call it with interrupts disabled.
 *
 */
void scheduler_tick(void)
{
	runqueue_t *rq = this_rq();
	task_t *p = current;
	unsigned long long now = sched_clock();

	update_cpu_clock(p, rq, now);

	rq->timestamp_last_tick = now;

	if (p == rq->idle) return;

	/*
	 * The task was running during this tick - update the
	 * time slice counter. Note: we do not update a thread's
	 * priority until it either goes to sleep or uses up its
	 * timeslice. This makes it possible for interactive tasks
	 * to use up their timeslices at their highest priority levels.
	 */
	/* Si se le acaba el tiempo */
	if (!--p->time_slice) {
		set_tsk_need_resched(p);
		p->time_slice = DEF_TIMESLICE;
 	}
}


static inline void wake_sleeping_dependent(int this_cpu, runqueue_t *this_rq)
{
}

static inline int dependent_sleeper(int this_cpu, runqueue_t *this_rq)
{
	return 0;
}

#if defined(CONFIG_PREEMPT) && defined(CONFIG_DEBUG_PREEMPT)

void fastcall add_preempt_count(int val)
{
	/*
	 * Underflow?
	 */
	BUG_ON((preempt_count() < 0));
	preempt_count() += val;
	/*
	 * Spinlock count overflowing soon?
	 */
	BUG_ON((preempt_count() & PREEMPT_MASK) >= PREEMPT_MASK-10);
}
EXPORT_SYMBOL(add_preempt_count);

void fastcall sub_preempt_count(int val)
{
	/*
	 * Underflow?
	 */
	BUG_ON(val > preempt_count());
	/*
	 * Is the spinlock portion underflowing?
	 */
	BUG_ON((val < PREEMPT_MASK) && !(preempt_count() & PREEMPT_MASK));
	preempt_count() -= val;
}
EXPORT_SYMBOL(sub_preempt_count);

#endif

/*
 * schedule() is the main scheduler function.
 */
asmlinkage void __sched schedule(void)
{

	long *switch_count;
	task_t *prev, *next;
	runqueue_t *rq;
	unsigned long long now;
	int cpu;
	struct grupo_struct *grupo_actual;
	struct usuario_struct *user_actual;

	/*
	 * Test if we are atomic.  Since do_exit() needs to call into
	 * schedule() atomically, we ignore that path for now.
	 * Otherwise, whine if we are scheduling when we should not be.
	 */
	if (likely(!current->exit_state)) {
		if (unlikely(in_atomic())) {
			printk(KERN_ERR "scheduling while atomic: "
				"%s/0x%08x/%d\n",
				current->comm, preempt_count(), current->pid);
			dump_stack();
		}
	}
	profile_hit(SCHED_PROFILING, __builtin_return_address(0));
	
need_resched:
	preempt_disable();
	prev = current;
	release_kernel_lock(prev);
need_resched_nonpreemptible:
	rq = this_rq();
	spin_lock_irq(&rq->lock);

	/*	
	 * The idle thread is not allowed to schedule!
	 * Remove this check after it has been exercised a bit.
	 */
	if (unlikely(prev == rq->idle) && prev->state != TASK_RUNNING) {
		printk(KERN_ERR "bad: scheduling from the idle thread!\n");
		dump_stack();
	}
	
	schedstat_inc(rq, sched_cnt);
	now = sched_clock();

	if (unlikely(prev->flags & PF_DEAD))
		prev->state = EXIT_DEAD;

	switch_count = &prev->nivcsw;
	if (prev->state && !(preempt_count() & PREEMPT_ACTIVE)) {
		switch_count = &prev->nvcsw;
		if (unlikely((prev->state & TASK_INTERRUPTIBLE) &&
				unlikely(signal_pending(prev))))
			prev->state = TASK_RUNNING;
		else {
			if (prev->state == TASK_UNINTERRUPTIBLE)
				rq->nr_uninterruptible++;
			deactivate_task(prev, rq);
		}
	}
	/* Si no existe una lista de grupos disponibles para ejecutar
           calendarizar a idle*/
	cpu = smp_processor_id();
	if (rq->grupos_lista == NULL) 
	{
		next = rq->idle;
		rq->expired_timestamp = 0;
	}
	else
	{
		/* Obtener un grupo y luego mover la punta de la lista al siguente elemento */
		grupo_actual = rq->grupos_lista;
		rq->grupos_lista = list_entry(rq->grupos_lista->lista.next, struct grupo_struct, lista);
		/* Obtener un usuario del grupo y luego mover la punta de la lista al siguente elemento */
		user_actual = grupo_actual->usuarios_lista;
		grupo_actual->usuarios_lista = list_entry(grupo_actual->usuarios_lista->lista.next, struct usuario_struct, lista);
		/* Obtener un proceso del usuario y luego mover la punta de la lista al siguente elemento */
		next = user_actual->procesos_lista;
		user_actual->procesos_lista =  list_entry(user_actual->procesos_lista->run_list.next, struct task_struct, run_list);
	}
	
	/* Iniciar el proceso de calendarizacion y coleccion de 
	   datos para el proceso, usuario y grupo seleccionado */
	if (next == rq->idle)
		schedstat_inc(rq, sched_goidle);
	prefetch(next);
	prefetch_stack(next);

	clear_tsk_need_resched(prev);
	rcu_qsctr_inc(task_cpu(prev));

	update_cpu_clock(prev, rq, now);

	prev->timestamp = prev->last_ran = now;

	sched_info_switch(prev, next);
	if (likely(prev != next)) {
		next->timestamp = now;
		rq->nr_switches++;
		rq->curr = next;
		++*switch_count;

		prepare_task_switch(rq, next);
		prev = context_switch(rq, prev, next);
		barrier();
		/*
		 * this_rq must be evaluated again because prev may have moved
		 * CPUs since it called schedule(), thus the 'rq' on its stack
		 * frame will be invalid.
		 */
		finish_task_switch(this_rq(), prev);

	} else
		spin_unlock_irq(&rq->lock);

	prev = current;
	if (unlikely(reacquire_kernel_lock(prev) < 0))
		goto need_resched_nonpreemptible;
	preempt_enable_no_resched();
	if (unlikely(test_thread_flag(TIF_NEED_RESCHED)))
		goto need_resched;
}

EXPORT_SYMBOL(schedule);

#ifdef CONFIG_PREEMPT
/*
 * this is is the entry point to schedule() from in-kernel preemption
 * off of preempt_enable.  Kernel preemptions off return from interrupt
 * occur there and call schedule directly.
 */
asmlinkage void __sched preempt_schedule(void)
{
	struct thread_info *ti = current_thread_info();
#ifdef CONFIG_PREEMPT_BKL
	struct task_struct *task = current;
	int saved_lock_depth;
#endif
	/*
	 * If there is a non-zero preempt_count or interrupts are disabled,
	 * we do not want to preempt the current task.  Just return..
	 */
	if (unlikely(ti->preempt_count || irqs_disabled()))
		return;

need_resched:
	add_preempt_count(PREEMPT_ACTIVE);
	/*
	 * We keep the big kernel semaphore locked, but we
	 * clear ->lock_depth so that schedule() doesnt
	 * auto-release the semaphore:
	 */
#ifdef CONFIG_PREEMPT_BKL
	saved_lock_depth = task->lock_depth;
	task->lock_depth = -1;
#endif
	schedule();
#ifdef CONFIG_PREEMPT_BKL
	task->lock_depth = saved_lock_depth;
#endif
	sub_preempt_count(PREEMPT_ACTIVE);

	/* we could miss a preemption opportunity between schedule and now */
	barrier();
	if (unlikely(test_thread_flag(TIF_NEED_RESCHED)))
		goto need_resched;
}

EXPORT_SYMBOL(preempt_schedule);

/*
 * this is is the entry point to schedule() from kernel preemption
 * off of irq context.
 * Note, that this is called and return with irqs disabled. This will
 * protect us against recursive calling from irq.
 */
asmlinkage void __sched preempt_schedule_irq(void)
{
	struct thread_info *ti = current_thread_info();
#ifdef CONFIG_PREEMPT_BKL
	struct task_struct *task = current;
	int saved_lock_depth;
#endif
	/* Catch callers which need to be fixed*/
	BUG_ON(ti->preempt_count || !irqs_disabled());

need_resched:
	add_preempt_count(PREEMPT_ACTIVE);
	/*
	 * We keep the big kernel semaphore locked, but we
	 * clear ->lock_depth so that schedule() doesnt
	 * auto-release the semaphore:
	 */
#ifdef CONFIG_PREEMPT_BKL
	saved_lock_depth = task->lock_depth;
	task->lock_depth = -1;
#endif
	local_irq_enable();
	schedule();
	local_irq_disable();
#ifdef CONFIG_PREEMPT_BKL
	task->lock_depth = saved_lock_depth;
#endif
	sub_preempt_count(PREEMPT_ACTIVE);

	/* we could miss a preemption opportunity between schedule and now */
	barrier();
	if (unlikely(test_thread_flag(TIF_NEED_RESCHED)))
		goto need_resched;
}

#endif /* CONFIG_PREEMPT */

int default_wake_function(wait_queue_t *curr, unsigned mode, int sync,
			  void *key)
{
	task_t *p = curr->private;
	return try_to_wake_up(p, mode, sync);
}

EXPORT_SYMBOL(default_wake_function);

/*
 * The core wakeup function.  Non-exclusive wakeups (nr_exclusive == 0) just
 * wake everything up.  If it's an exclusive wakeup (nr_exclusive == small +ve
 * number) then we wake all the non-exclusive tasks and one exclusive task.
 *
 * There are circumstances in which we can try to wake a task which has already
 * started to run but is not in state TASK_RUNNING.  try_to_wake_up() returns
 * zero in this (rare) case, and we handle it by continuing to scan the queue.
 */
static void __wake_up_common(wait_queue_head_t *q, unsigned int mode,
			     int nr_exclusive, int sync, void *key)
{
	struct list_head *tmp, *next;

	list_for_each_safe(tmp, next, &q->task_list) {
		wait_queue_t *curr;
		unsigned flags;
		curr = list_entry(tmp, wait_queue_t, task_list);
		flags = curr->flags;
		if (curr->func(curr, mode, sync, key) &&
		    (flags & WQ_FLAG_EXCLUSIVE) &&
		    !--nr_exclusive)
			break;
	}
}

/**
 * __wake_up - wake up threads blocked on a waitqueue.
 * @q: the waitqueue
 * @mode: which threads
 * @nr_exclusive: how many wake-one or wake-many threads to wake up
 * @key: is directly passed to the wakeup function
 */
void fastcall __wake_up(wait_queue_head_t *q, unsigned int mode,
			int nr_exclusive, void *key)
{
	unsigned long flags;

	spin_lock_irqsave(&q->lock, flags);
	__wake_up_common(q, mode, nr_exclusive, 0, key);
	spin_unlock_irqrestore(&q->lock, flags);
}

EXPORT_SYMBOL(__wake_up);

/*
 * Same as __wake_up but called with the spinlock in wait_queue_head_t held.
 */
void fastcall __wake_up_locked(wait_queue_head_t *q, unsigned int mode)
{
	__wake_up_common(q, mode, 1, 0, NULL);
}

/**
 * __wake_up_sync - wake up threads blocked on a waitqueue.
 * @q: the waitqueue
 * @mode: which threads
 * @nr_exclusive: how many wake-one or wake-many threads to wake up
 *
 * The sync wakeup differs that the waker knows that it will schedule
 * away soon, so while the target thread will be woken up, it will not
 * be migrated to another CPU - ie. the two threads are 'synchronized'
 * with each other. This can prevent needless bouncing between CPUs.
 *
 * On UP it can prevent extra preemption.
 */
void fastcall __wake_up_sync(wait_queue_head_t *q, unsigned int mode, int nr_exclusive)
{
	unsigned long flags;
	int sync = 1;

	if (unlikely(!q))
		return;

	if (unlikely(!nr_exclusive))
		sync = 0;

	spin_lock_irqsave(&q->lock, flags);
	__wake_up_common(q, mode, nr_exclusive, sync, NULL);
	spin_unlock_irqrestore(&q->lock, flags);
}
EXPORT_SYMBOL_GPL(__wake_up_sync);	/* For internal use only */

void fastcall complete(struct completion *x)
{
	unsigned long flags;

	spin_lock_irqsave(&x->wait.lock, flags);
	x->done++;
	__wake_up_common(&x->wait, TASK_UNINTERRUPTIBLE | TASK_INTERRUPTIBLE,
			 1, 0, NULL);
	spin_unlock_irqrestore(&x->wait.lock, flags);
}
EXPORT_SYMBOL(complete);

void fastcall complete_all(struct completion *x)
{
	unsigned long flags;

	spin_lock_irqsave(&x->wait.lock, flags);
	x->done += UINT_MAX/2;
	__wake_up_common(&x->wait, TASK_UNINTERRUPTIBLE | TASK_INTERRUPTIBLE,
			 0, 0, NULL);
	spin_unlock_irqrestore(&x->wait.lock, flags);
}
EXPORT_SYMBOL(complete_all);

void fastcall __sched wait_for_completion(struct completion *x)
{
	might_sleep();
	spin_lock_irq(&x->wait.lock);
	if (!x->done) {
		DECLARE_WAITQUEUE(wait, current);

		wait.flags |= WQ_FLAG_EXCLUSIVE;
		__add_wait_queue_tail(&x->wait, &wait);
		do {
			__set_current_state(TASK_UNINTERRUPTIBLE);
			spin_unlock_irq(&x->wait.lock);
			schedule();
			spin_lock_irq(&x->wait.lock);
		} while (!x->done);
		__remove_wait_queue(&x->wait, &wait);
	}
	x->done--;
	spin_unlock_irq(&x->wait.lock);
}
EXPORT_SYMBOL(wait_for_completion);

unsigned long fastcall __sched
wait_for_completion_timeout(struct completion *x, unsigned long timeout)
{
	might_sleep();

	spin_lock_irq(&x->wait.lock);
	if (!x->done) {
		DECLARE_WAITQUEUE(wait, current);

		wait.flags |= WQ_FLAG_EXCLUSIVE;
		__add_wait_queue_tail(&x->wait, &wait);
		do {
			__set_current_state(TASK_UNINTERRUPTIBLE);
			spin_unlock_irq(&x->wait.lock);
			timeout = schedule_timeout(timeout);
			spin_lock_irq(&x->wait.lock);
			if (!timeout) {
				__remove_wait_queue(&x->wait, &wait);
				goto out;
			}
		} while (!x->done);
		__remove_wait_queue(&x->wait, &wait);
	}
	x->done--;
out:
	spin_unlock_irq(&x->wait.lock);
	return timeout;
}
EXPORT_SYMBOL(wait_for_completion_timeout);

int fastcall __sched wait_for_completion_interruptible(struct completion *x)
{
	int ret = 0;

	might_sleep();

	spin_lock_irq(&x->wait.lock);
	if (!x->done) {
		DECLARE_WAITQUEUE(wait, current);

		wait.flags |= WQ_FLAG_EXCLUSIVE;
		__add_wait_queue_tail(&x->wait, &wait);
		do {
			if (signal_pending(current)) {
				ret = -ERESTARTSYS;
				__remove_wait_queue(&x->wait, &wait);
				goto out;
			}
			__set_current_state(TASK_INTERRUPTIBLE);
			spin_unlock_irq(&x->wait.lock);
			schedule();
			spin_lock_irq(&x->wait.lock);
		} while (!x->done);
		__remove_wait_queue(&x->wait, &wait);
	}
	x->done--;
out:
	spin_unlock_irq(&x->wait.lock);

	return ret;
}
EXPORT_SYMBOL(wait_for_completion_interruptible);

unsigned long fastcall __sched
wait_for_completion_interruptible_timeout(struct completion *x, unsigned long timeout)
{
	might_sleep();

	spin_lock_irq(&x->wait.lock);
	if (!x->done) {
		DECLARE_WAITQUEUE(wait, current);

		wait.flags |= WQ_FLAG_EXCLUSIVE;
		__add_wait_queue_tail(&x->wait, &wait);
		do {
			if (signal_pending(current)) {
				timeout = -ERESTARTSYS;
				__remove_wait_queue(&x->wait, &wait);
				goto out;
			}
			__set_current_state(TASK_INTERRUPTIBLE);
			spin_unlock_irq(&x->wait.lock);
			timeout = schedule_timeout(timeout);
			spin_lock_irq(&x->wait.lock);
			if (!timeout) {
				__remove_wait_queue(&x->wait, &wait);
				goto out;
			}
		} while (!x->done);
		__remove_wait_queue(&x->wait, &wait);
	}
	x->done--;
out:
	spin_unlock_irq(&x->wait.lock);
	return timeout;
}
EXPORT_SYMBOL(wait_for_completion_interruptible_timeout);


#define	SLEEP_ON_VAR					\
	unsigned long flags;				\
	wait_queue_t wait;				\
	init_waitqueue_entry(&wait, current);

#define SLEEP_ON_HEAD					\
	spin_lock_irqsave(&q->lock,flags);		\
	__add_wait_queue(q, &wait);			\
	spin_unlock(&q->lock);

#define	SLEEP_ON_TAIL					\
	spin_lock_irq(&q->lock);			\
	__remove_wait_queue(q, &wait);			\
	spin_unlock_irqrestore(&q->lock, flags);

void fastcall __sched interruptible_sleep_on(wait_queue_head_t *q)
{
	SLEEP_ON_VAR

	current->state = TASK_INTERRUPTIBLE;

	SLEEP_ON_HEAD
	schedule();
	SLEEP_ON_TAIL
}

EXPORT_SYMBOL(interruptible_sleep_on);

long fastcall __sched
interruptible_sleep_on_timeout(wait_queue_head_t *q, long timeout)
{
	SLEEP_ON_VAR

	current->state = TASK_INTERRUPTIBLE;

	SLEEP_ON_HEAD
	timeout = schedule_timeout(timeout);
	SLEEP_ON_TAIL

	return timeout;
}

EXPORT_SYMBOL(interruptible_sleep_on_timeout);

void fastcall __sched sleep_on(wait_queue_head_t *q)
{
	SLEEP_ON_VAR

	current->state = TASK_UNINTERRUPTIBLE;

	SLEEP_ON_HEAD
	schedule();
	SLEEP_ON_TAIL
}

EXPORT_SYMBOL(sleep_on);

long fastcall __sched sleep_on_timeout(wait_queue_head_t *q, long timeout)
{
	SLEEP_ON_VAR

	current->state = TASK_UNINTERRUPTIBLE;

	SLEEP_ON_HEAD
	timeout = schedule_timeout(timeout);
	SLEEP_ON_TAIL

	return timeout;
}

EXPORT_SYMBOL(sleep_on_timeout);

void set_user_nice(task_t *p, long nice)
{
}

EXPORT_SYMBOL(set_user_nice);


#ifdef CONFIG_SCHEDSTATS
/* Tratativas de llamadas al sistema para configurar las estadisticas */

asmlinkage long sys_sched_set_stat(int parametro)
{
	runqueue_t *rq = this_rq();
	
	int ret = -EINVAL;
	switch (parametro){
	case 0:	/* cera las estadisticas */
		if(rq->stat == 1){
			cerar_stat();
			ret = 0;
		}else
			ret = -1;
		break;
	case 1:	/* habilita las estadisticas */
		if(rq->stat == 0){
			rq->stat = 1;
			ret = 0;
		}else
			ret = -1;
		break;
	case 2: /* deshabilita las estadisticas */
		if(rq->stat == 1){
			rq->stat = 0;
			ret = 0;
		}else
			ret = -1;
		break;
	}
	
	return ret;
	
}

asmlinkage long sys_sched_get_stat(void)
{
	runqueue_t *rq = this_rq();

	return rq->stat;
	
}
#else
asmlinkage long sys_sched_set_stat(int parametro){ return 0; }
asmlinkage long sys_sched_get_stat(void){ return 0; }
#endif

/* NO SE APLICA A FAIR SHARE
 * can_nice - check if a task can reduce its nice value
 * @p: task
 * @nice: nice value
 */
int can_nice(const task_t *p, const int nice)
{
	return 0;
}

#ifdef __ARCH_WANT_SYS_NICE

/* NO SE APLICA A FAIR SHARE
 * sys_nice - change the priority of the current process.
 * @increment: priority increment
 *
 */
asmlinkage long sys_nice(int increment)
{
	return 0;
}

#endif

/**  NO SE APLICA A FAIR SHARE
 * task_prio - return the priority value of a given task.
 * @p: the task in question.
 *
 */
int task_prio(const task_t *p)
{
	return 0;
}

/**  NO SE APLICA A FAIR SHARE
 * task_nice - return the nice value of a given task.
 * @p: the task in question.
 */
int task_nice(const task_t *p)
{
	return 0;
}
EXPORT_SYMBOL_GPL(task_nice);

/**
 * idle_cpu - is a given cpu idle currently?
 * @cpu: the processor in question.
 */
int idle_cpu(int cpu)
{
	return cpu_curr(cpu) == cpu_rq(cpu)->idle;
}

EXPORT_SYMBOL_GPL(idle_cpu);

/**
 * idle_task - return the idle task for a given cpu.
 * @cpu: the processor in question.
 */
task_t *idle_task(int cpu)
{
	return cpu_rq(cpu)->idle;
}

/**
 * find_process_by_pid - find a process with a matching PID value.
 * @pid: the pid in question.
 */
static inline task_t *find_process_by_pid(pid_t pid)
{
	return pid ? find_task_by_pid(pid) : current;
}


/**  NO SE APLICA A FAIR SHARE
 * sched_setscheduler - change the scheduling policy and/or RT priority of
 * a thread.
 * @p: the task in question.
 * @policy: new policy.
 * @param: structure containing the new RT priority.
 */
int sched_setscheduler(struct task_struct *p, int policy,
		       struct sched_param *param)
{
	return 0;
}
EXPORT_SYMBOL_GPL(sched_setscheduler);

/**  NO SE APLICA A FAIR SHARE
 * sys_sched_setscheduler - set/change the scheduler policy and RT priority
 * @pid: the pid in question.
 * @policy: new policy.
 * @param: structure containing the new RT priority.
 */
asmlinkage long sys_sched_setscheduler(pid_t pid, int policy,
				       struct sched_param __user *param)
{
	return 0;
}

/**  NO SE APLICA A FAIR SHARE
 * sys_sched_setparam - set/change the RT priority of a thread
 * @pid: the pid in question.
 * @param: structure containing the new RT priority.
 */
asmlinkage long sys_sched_setparam(pid_t pid, struct sched_param __user *param)
{
	return 0;
}

/**  NO SE APLICA A FAIR SHARE
 * sys_sched_getscheduler - get the policy (scheduling class) of a thread
 * @pid: the pid in question.
 */
asmlinkage long sys_sched_getscheduler(pid_t pid)
{
	return SCHED_NORMAL;
}

/**
 * sys_sched_getscheduler - get the RT priority of a thread
 * @pid: the pid in question.
 * @param: structure containing the RT priority.
 */
asmlinkage long sys_sched_getparam(pid_t pid, struct sched_param __user *param)
{
	struct sched_param lp;
	int retval = -EINVAL;
	task_t *p;

	if (!param || pid < 0)
		goto out_nounlock;

	read_lock(&tasklist_lock);
	p = find_process_by_pid(pid);
	retval = -ESRCH;
	if (!p)
		goto out_unlock;

	retval = security_task_getscheduler(p);
	if (retval)
		goto out_unlock;

	lp.sched_priority = p->rt_priority;
	read_unlock(&tasklist_lock);

	/*
	 * This one might sleep, we cannot do it with a spinlock held ...
	 */
	retval = copy_to_user(param, &lp, sizeof(*param)) ? -EFAULT : 0;

out_nounlock:
	return retval;

out_unlock:
	read_unlock(&tasklist_lock);
	return retval;
}

long sched_setaffinity(pid_t pid, cpumask_t new_mask)
{
	task_t *p;
	int retval;
	cpumask_t cpus_allowed;

	lock_cpu_hotplug();
	read_lock(&tasklist_lock);

	p = find_process_by_pid(pid);
	if (!p) {
		read_unlock(&tasklist_lock);
		unlock_cpu_hotplug();
		return -ESRCH;
	}

	/*
	 * It is not safe to call set_cpus_allowed with the
	 * tasklist_lock held.  We will bump the task_struct's
	 * usage count and then drop tasklist_lock.
	 */
	get_task_struct(p);
	read_unlock(&tasklist_lock);

	retval = -EPERM;
	if ((current->euid != p->euid) && (current->euid != p->uid) &&
			!capable(CAP_SYS_NICE))
		goto out_unlock;

	cpus_allowed = cpuset_cpus_allowed(p);
	cpus_and(new_mask, new_mask, cpus_allowed);
	retval = set_cpus_allowed(p, new_mask);

out_unlock:
	put_task_struct(p);
	unlock_cpu_hotplug();
	return retval;
}

static int get_user_cpu_mask(unsigned long __user *user_mask_ptr, unsigned len,
			     cpumask_t *new_mask)
{
	if (len < sizeof(cpumask_t)) {
		memset(new_mask, 0, sizeof(cpumask_t));
	} else if (len > sizeof(cpumask_t)) {
		len = sizeof(cpumask_t);
	}
	return copy_from_user(new_mask, user_mask_ptr, len) ? -EFAULT : 0;
}

/**
 * sys_sched_setaffinity - set the cpu affinity of a process
 * @pid: pid of the process
 * @len: length in bytes of the bitmask pointed to by user_mask_ptr
 * @user_mask_ptr: user-space pointer to the new cpu mask
 */
asmlinkage long sys_sched_setaffinity(pid_t pid, unsigned int len,
				      unsigned long __user *user_mask_ptr)
{
	cpumask_t new_mask;
	int retval;

	retval = get_user_cpu_mask(user_mask_ptr, len, &new_mask);
	if (retval)
		return retval;

	return sched_setaffinity(pid, new_mask);
}

/*
 * Represents all cpu's present in the system
 * In systems capable of hotplug, this map could dynamically grow
 * as new cpu's are detected in the system via any platform specific
 * method, such as ACPI for e.g.
 */

cpumask_t cpu_present_map;
EXPORT_SYMBOL(cpu_present_map);

cpumask_t cpu_online_map = CPU_MASK_ALL;
EXPORT_SYMBOL_GPL(cpu_online_map);
cpumask_t cpu_possible_map = CPU_MASK_ALL;

long sched_getaffinity(pid_t pid, cpumask_t *mask)
{
	int retval;
	task_t *p;

	lock_cpu_hotplug();
	read_lock(&tasklist_lock);

	retval = -ESRCH;
	p = find_process_by_pid(pid);
	if (!p)
		goto out_unlock;

	retval = 0;
	cpus_and(*mask, p->cpus_allowed, cpu_possible_map);

out_unlock:
	read_unlock(&tasklist_lock);
	unlock_cpu_hotplug();
	if (retval)
		return retval;

	return 0;
}

/**
 * sys_sched_getaffinity - get the cpu affinity of a process
 * @pid: pid of the process
 * @len: length in bytes of the bitmask pointed to by user_mask_ptr
 * @user_mask_ptr: user-space pointer to hold the current cpu mask
 */
asmlinkage long sys_sched_getaffinity(pid_t pid, unsigned int len,
				      unsigned long __user *user_mask_ptr)
{
	int ret;
	cpumask_t mask;

	if (len < sizeof(cpumask_t))
		return -EINVAL;

	ret = sched_getaffinity(pid, &mask);
	if (ret < 0)
		return ret;

	if (copy_to_user(user_mask_ptr, &mask, sizeof(cpumask_t)))
		return -EFAULT;

	return sizeof(cpumask_t);
}

/**
 * sys_sched_yield - yield the current processor to other threads.
 *
 * this function yields the current CPU by moving the calling thread
 * CPU then this function will return.
 */
asmlinkage long sys_sched_yield(void)
{
	runqueue_t *rq = this_rq_lock();
	__release(rq->lock);
	_raw_spin_unlock(&rq->lock);
	preempt_enable_no_resched();

	schedule();

	return 0;
}

static inline void __cond_resched(void)
{
	/*
	 * The BKS might be reacquired before we have dropped
	 * PREEMPT_ACTIVE, which could trigger a second
	 * cond_resched() call.
	 */
	if (unlikely(preempt_count()))
		return;
	do {
		add_preempt_count(PREEMPT_ACTIVE);
		schedule();
		sub_preempt_count(PREEMPT_ACTIVE);
	} while (need_resched());
}

int __sched cond_resched(void)
{
	if (need_resched()) {
		__cond_resched();
		return 1;
	}
	return 0;
}

EXPORT_SYMBOL(cond_resched);

/*
 * cond_resched_lock() - if a reschedule is pending, drop the given lock,
 * call schedule, and on return reacquire the lock.
 *
 * This works OK both with and without CONFIG_PREEMPT.  We do strange low-level
 * operations here to prevent schedule() from being called twice (once via
 * spin_unlock(), once by hand).
 */
int cond_resched_lock(spinlock_t *lock)
{
	int ret = 0;

	if (need_lockbreak(lock)) {
		spin_unlock(lock);
		cpu_relax();
		ret = 1;
		spin_lock(lock);
	}
	if (need_resched()) {
		_raw_spin_unlock(lock);
		preempt_enable_no_resched();
		__cond_resched();
		ret = 1;
		spin_lock(lock);
	}
	return ret;
}

EXPORT_SYMBOL(cond_resched_lock);

int __sched cond_resched_softirq(void)
{
	BUG_ON(!in_softirq());

	if (need_resched()) {
		__local_bh_enable();
		__cond_resched();
		local_bh_disable();
		return 1;
	}
	return 0;
}

EXPORT_SYMBOL(cond_resched_softirq);


/**
 * yield - yield the current processor to other threads.
 *
 * this is a shortcut for kernel-space yielding - it marks the
 * thread runnable and calls sys_sched_yield().
 */
void __sched yield(void)
{
	set_current_state(TASK_RUNNING);
	sys_sched_yield();
}

EXPORT_SYMBOL(yield);

/*
 * This task is about to go to sleep on IO.  Increment rq->nr_iowait so
 * that process accounting knows that this is a task in IO wait state.
 *
 * But don't do that if it is a deliberate, throttling IO wait (this task
 * has set its backing_dev_info: the queue against which it should throttle)
 */
void __sched io_schedule(void)
{
	struct runqueue *rq = &per_cpu(runqueues, raw_smp_processor_id());

	atomic_inc(&rq->nr_iowait);
	schedule();
	atomic_dec(&rq->nr_iowait);
}

EXPORT_SYMBOL(io_schedule);

long __sched io_schedule_timeout(long timeout)
{
	struct runqueue *rq = &per_cpu(runqueues, raw_smp_processor_id());
	long ret;

	atomic_inc(&rq->nr_iowait);
	ret = schedule_timeout(timeout);
	atomic_dec(&rq->nr_iowait);
	return ret;
}

/**
 * sys_sched_get_priority_max - return maximum RT priority.
 * @policy: scheduling class.
 *
 * this syscall returns the maximum rt_priority that can be used
 * by a given scheduling class.
 */
asmlinkage long sys_sched_get_priority_max(int policy)
{
	return 0;
}

/**
 * sys_sched_get_priority_min - return minimum RT priority.
 * @policy: scheduling class.
 *
 * this syscall returns the minimum rt_priority that can be used
 * by a given scheduling class.
 */
asmlinkage long sys_sched_get_priority_min(int policy)
{
	int ret = -EINVAL;

	switch (policy) {
	case SCHED_FIFO:
	case SCHED_RR:
		ret = 1;
		break;
	case SCHED_NORMAL:
		ret = 0;
	}
	return ret;
}

/**
 * sys_sched_rr_get_interval - return the default timeslice of a process.
 * @pid: pid of the process.
 * @interval: userspace pointer to the timeslice value.
 *
 * this syscall writes the default timeslice value of a given process
 * into the user-space timespec buffer. A value of '0' means infinity.
 */
asmlinkage
long sys_sched_rr_get_interval(pid_t pid, struct timespec __user *interval)
{
	int retval = -EINVAL;
	struct timespec t;
	task_t *p;

	if (pid < 0)
		goto out_nounlock;

	retval = -ESRCH;
	read_lock(&tasklist_lock);
	p = find_process_by_pid(pid);
	if (!p)
		goto out_unlock;

	retval = security_task_getscheduler(p);
	if (retval)
		goto out_unlock;

	jiffies_to_timespec(p->policy & SCHED_FIFO ?
				0 : DEF_TIMESLICE, &t);
	read_unlock(&tasklist_lock);
	retval = copy_to_user(interval, &t, sizeof(t)) ? -EFAULT : 0;
out_nounlock:
	return retval;
out_unlock:
	read_unlock(&tasklist_lock);
	return retval;
}

static inline struct task_struct *eldest_child(struct task_struct *p)
{
	if (list_empty(&p->children)) return NULL;
	return list_entry(p->children.next,struct task_struct,sibling);
}

static inline struct task_struct *older_sibling(struct task_struct *p)
{
	if (p->sibling.prev==&p->parent->children) return NULL;
	return list_entry(p->sibling.prev,struct task_struct,sibling);
}

static inline struct task_struct *younger_sibling(struct task_struct *p)
{
	if (p->sibling.next==&p->parent->children) return NULL;
	return list_entry(p->sibling.next,struct task_struct,sibling);
}

static void show_task(task_t *p)
{
	task_t *relative;
	unsigned state;
	unsigned long free = 0;
	static const char *stat_nam[] = { "R", "S", "D", "T", "t", "Z", "X" };

	printk("%-13.13s ", p->comm);
	state = p->state ? __ffs(p->state) + 1 : 0;
	if (state < ARRAY_SIZE(stat_nam))
		printk(stat_nam[state]);
	else
		printk("?");
#if (BITS_PER_LONG == 32)
	if (state == TASK_RUNNING)
		printk(" running ");
	else
		printk(" %08lX ", thread_saved_pc(p));
#else
	if (state == TASK_RUNNING)
		printk("  running task   ");
	else
		printk(" %016lx ", thread_saved_pc(p));
#endif
#ifdef CONFIG_DEBUG_STACK_USAGE
	{
		unsigned long *n = (unsigned long *) (p->thread_info+1);
		while (!*n)
			n++;
		free = (unsigned long) n - (unsigned long)(p->thread_info+1);
	}
#endif
	printk("%5lu %5d %6d ", free, p->pid, p->parent->pid);
	if ((relative = eldest_child(p)))
		printk("%5d ", relative->pid);
	else
		printk("      ");
	if ((relative = younger_sibling(p)))
		printk("%7d", relative->pid);
	else
		printk("       ");
	if ((relative = older_sibling(p)))
		printk(" %5d", relative->pid);
	else
		printk("      ");
	if (!p->mm)
		printk(" (L-TLB)\n");
	else
		printk(" (NOTLB)\n");

	if (state != TASK_RUNNING)
		show_stack(p, NULL);
}

void show_state(void)
{
	task_t *g, *p;

#if (BITS_PER_LONG == 32)
	printk("\n"
	       "                                               sibling\n");
	printk("  task             PC      pid father child younger older\n");
#else
	printk("\n"
	       "                                                       sibling\n");
	printk("  task                 PC          pid father child younger older\n");
#endif
	read_lock(&tasklist_lock);
	do_each_thread(g, p) {
		/*
		 * reset the NMI-timeout, listing all files on a slow
		 * console might take alot of time:
		 */
		touch_nmi_watchdog();
		show_task(p);
	} while_each_thread(g, p);

	read_unlock(&tasklist_lock);
}

/**
 * init_idle - set up an idle thread for a given CPU
 * @idle: task in question
 * @cpu: cpu the idle task belongs to
 *
 * NOTE: this function does not set the idle thread's NEED_RESCHED
 * flag, to make booting more robust.
 */
void __devinit init_idle(task_t *idle, int cpu)
{
	runqueue_t *rq = cpu_rq(cpu);
	unsigned long flags;

	idle->sleep_avg = 0;
	idle->state = TASK_RUNNING;
	idle->cpus_allowed = cpumask_of_cpu(cpu);
	set_task_cpu(idle, cpu);
	idle->grupo = init_grupo(0);
	idle->usuario = init_usuario(0);
	spin_lock_irqsave(&rq->lock, flags);
	rq->curr = rq->idle = idle;
	spin_unlock_irqrestore(&rq->lock, flags);

	/* Set the preempt count _outside_ the spinlocks! */
#if defined(CONFIG_PREEMPT) && !defined(CONFIG_PREEMPT_BKL)
	idle->thread_info->preempt_count = (idle->lock_depth >= 0);
#else
	idle->thread_info->preempt_count = 0;
#endif
}

/*
 * In a system that switches off the HZ timer nohz_cpu_mask
 * indicates which cpus entered this state. This is used
 * in the rcu update to wait only for active cpus. For system
 * which do not switch off the HZ timer nohz_cpu_mask should
 * always be CPU_MASK_NONE.
 */
cpumask_t nohz_cpu_mask = CPU_MASK_NONE;

void __init sched_init_smp(void)
{
}

int in_sched_functions(unsigned long addr)
{
	/* Linker adds these: start and end of __sched functions */
	extern char __sched_text_start[], __sched_text_end[];
	return in_lock_functions(addr) ||
		(addr >= (unsigned long)__sched_text_start
		&& addr < (unsigned long)__sched_text_end);
}

/* Elimina la estructura grupo, solamente si nadie mas hace referencia a ella */


/* Rutina para mover un proceso de un usuario a otro */
void switch_uid_sched(uid_t nuid)
{
	unsigned long flags;
	runqueue_t *rq;
	/* Se deshabilita la expropiacion debido a que las estructuras quedan
           inconsistentes por un momento */
	rq = task_rq_lock(current,&flags);
	dequeue_task(current);
	delete_usuario(current->usuario);
	current->usuario = init_usuario(nuid);
	enqueue_task(current);
	task_rq_unlock(rq, &flags);
	
}

EXPORT_SYMBOL(switch_uid_sched);

/* Rutina para mover un proceso de un grupo a otro, y de un usuario de un grupo a otro
 * Lo segundo se hace debido a que un usuario de un grupo debe ser distinto a un usuario de
 * otro grupo con el mismo nombre.
 */
void switch_gid_sched(void)
{
	unsigned long flags;
	runqueue_t *rq;
	/* Se deshabilita la expropiacion debido a que las estructuras quedan
           inconsistentes por un momento */
	rq = task_rq_lock(current,&flags);
	dequeue_task(current);
	delete_usuario(current->usuario);
	delete_grupo(current->grupo);
	/* El proceso esta totalmente desconectado, volver a insertarlo
	   en la nueva ubicacion */
	current->grupo = (struct grupo_struct *)init_grupo(current->gid);
	current->usuario = init_usuario(current->uid);
	enqueue_task(current);
	task_rq_unlock(rq, &flags);
}

EXPORT_SYMBOL(switch_gid_sched);

/*
* Inicializacion del Scheduler de Linux
* Esta funcion inicializa los valores de la cola de ejecucion, el grupo y usuario incial, y por ultimo la tarea "idle"
*/
void __init sched_init(void)
{
	runqueue_t *rq;
	rq = cpu_rq(0);
	spin_lock_init(&rq->lock);
	rq->nr_running = 0;
	rq->best_expired_prio = 1;
	atomic_set(&rq->nr_iowait, 0);
	atomic_set(&rq->grupos, 0);

	rq->grupos_lista = NULL;

	/* Inicializacion del Grupo 0 */
	grupo_root.gid = 0;
	grupo_root.usuarios_lista = NULL;
	INIT_LIST_HEAD(&grupo_root.lista);
	INIT_LIST_HEAD(&grupo_root.lista_completa);
	atomic_set( &grupo_root.usuarios, 0);
#ifdef CONFIG_SCHEDSTATS
	memset(&grupo_root.sched_info, 0, sizeof(grupo_root.sched_info));
#endif

	/* Inicializacion del Usuario 0 */
	usuario_root.uid = 0;
	usuario_root.procesos_lista = NULL;
	INIT_LIST_HEAD(&usuario_root.lista);
	INIT_LIST_HEAD(&usuario_root.lista_completa);
	atomic_set(&usuario_root.procesos, 0);
#ifdef CONFIG_SCHEDSTATS
	memset(&usuario_root.sched_info, 0, sizeof(usuario_root.sched_info));
#endif
	/* Inicializacion del Proceso */
	init_task.grupo = &grupo_root;
	atomic_set(&grupo_root.ref,1);
	atomic_set(&usuario_root.ref,1);
	init_task.usuario = &usuario_root;
	grupo_root.usuarios_lista_completa = &usuario_root;
	
	/*
	 * The boot idle thread does lazy MMU switching as well:
	 */
	atomic_inc(&init_mm.mm_count);
	enter_lazy_tlb(&init_mm, current);

	/*
	 * Make us the idle thread. Technically, schedule() should not be
	 * called from this thread, however somewhere below it might be,
	 * but because we are the idle thread, we just pick up running again
	 * when this runqueue becomes "idle".
	 */
	init_idle(current, smp_processor_id());
}

#ifdef CONFIG_DEBUG_SPINLOCK_SLEEP
void __might_sleep(char *file, int line)
{
#if defined(in_atomic)
	static unsigned long prev_jiffy;	/* ratelimiting */

	if ((in_atomic() || irqs_disabled()) &&
	    system_state == SYSTEM_RUNNING && !oops_in_progress) {
		if (time_before(jiffies, prev_jiffy + HZ) && prev_jiffy)
			return;
		prev_jiffy = jiffies;
		printk(KERN_ERR "Debug: sleeping function called from invalid"
				" context at %s:%d\n", file, line);
		printk("in_atomic():%d, irqs_disabled():%d\n",
			in_atomic(), irqs_disabled());
		dump_stack();
	}
#endif
}
EXPORT_SYMBOL(__might_sleep);
#endif

#ifdef CONFIG_MAGIC_SYSRQ
void normalize_rt_tasks(void)
{
}
#endif /* CONFIG_MAGIC_SYSRQ */
