#ifndef __ASM_ARM_IRQ_H
#define __ASM_ARM_IRQ_H

#define NR_IRQS_LEGACY	16

#ifndef CONFIG_SPARSE_IRQ
#include <mach/irqs.h>
#else
#if !defined(CONFIG_IPIPE) || defined(CONFIG_IRQ_DOMAIN)
#define NR_IRQS NR_IRQS_LEGACY
#else
#define NR_IRQS 512
#endif
#endif


#ifndef irq_canonicalize
#define irq_canonicalize(i)	(i)
#endif

/*
 * Use this value to indicate lack of interrupt
 * capability
 */
#ifndef NO_IRQ
#define NO_IRQ	((unsigned int)(-1))
#endif

#ifndef __ASSEMBLY__
struct irqaction;
struct pt_regs;
extern void migrate_irqs(void);

extern void asm_do_IRQ(unsigned int, struct pt_regs *);
void handle_IRQ(unsigned int, struct pt_regs *);
void init_IRQ(void);

#ifdef CONFIG_MULTI_IRQ_HANDLER
extern void (*handle_arch_irq)(struct pt_regs *);
extern void set_handle_irq(void (*handle_irq)(struct pt_regs *));
#endif

#ifdef CONFIG_SMP
extern void arch_trigger_all_cpu_backtrace(bool);
#define arch_trigger_all_cpu_backtrace(x) arch_trigger_all_cpu_backtrace(x)
#endif

static inline int nr_legacy_irqs(void)
{
	return NR_IRQS_LEGACY;
}

#endif

#endif
