#ifndef __ASM_ARM_PLATFORMS_OMAP5_H
#define __ASM_ASM_PLATFORMS_OMAP5_H

#define REALTIME_COUNTER_BASE                   0x48243200
#define INCREMENTER_NUMERATOR_OFFSET            0x10
#define INCREMENTER_DENUMERATOR_RELOAD_OFFSET   0x14
#define NUMERATOR_DENUMERATOR_MASK              0xfffff000
#define PRM_FRAC_INCREMENTER_DENUMERATOR_RELOAD 0x00010000

#define OMAP5_L4_WKUP                           0x4AE00000
#define OMAP5_PRM_BASE                          (OMAP5_L4_WKUP + 0x6000)
#define OMAP5_CKGEN_PRM_BASE                    (OMAP5_PRM_BASE + 0x100)
#define OMAP5_CM_CLKSEL_SYS                     0x10
#define SYS_CLKSEL_MASK                         0xfffffff8

#define OMAP5_PRCM_MPU_BASE                     0x48243000
#define OMAP5_WKUPGEN_BASE                      0x48281000
#define OMAP5_SRAM_PA                           0x40300000

#define OMAP_AUX_CORE_BOOT_0_OFFSET             0x800
#define OMAP_AUX_CORE_BOOT_1_OFFSET             0x804

#ifdef CONFIG_CROSSBAR_INTC

#define DT_MATCH_CROSSBAR                       \
    DT_MATCH_COMPATIBLE("ti,irq-crossbar")

#define CTRL_CORE_MPU_IRQ_BASE                  0x4A002A48
#define CTRL_CORE_BASE                          0x4A002000

#define CTRL_CORE_MPU_START_OFFSET              0xA48
#define CTRL_CORE_MPU_END_OFFSET                0xB76

#define BAD_IRQ_LINE -1

#define CROSSBAR_MAX_LINE_NUM                   159
#define CROSSBAR_NUM_OF_LINES                   160
#endif /* CONFIG_CROSSBAR_INTC */

#endif /* __ASM_ARM_PLATFORMS_OMAP5_H */

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
