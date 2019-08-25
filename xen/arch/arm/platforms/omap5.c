/*
 * xen/arch/arm/platforms/omap5.c
 *
 * OMAP5 specific settings
 *
 * Chen Baozi <baozich@gmail.com>
 * Copyright (c) 2013
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <asm/platform.h>
#include <asm/platforms/omap5.h>
#include <xen/mm.h>
#include <xen/vmap.h>
#include <asm/io.h>
#include <asm/domain_build.h>
#include <xen/irq.h>
#include <xen/device_tree.h>

#ifdef CONFIG_CROSSBAR_INTC
static int current_offset = 0;
/* Starting from the fourth line (available are 4,7-130,133-138,141-159) */
static int mpu_irq = 4;
/* Crossbar control registers offsets from the CTRL_CORE_MPU_IRQ_BASE */
static int crossbar_offsets[CROSSBAR_NUM_OF_LINES] = {0};
/* Map to these addresses first CROSSBAR control register and its page start */
static void * base_ctrl;
static void * base_ctrl_page;

/*
 * Initialise crossbar and related structures. Remember that dom0 should reinit
 * the crossbar because not all working interrupts are mapped.
 * Also, this function is called after the first call of crossbar_translate
 * for xen console initialization.
 */
static int omap5_crossbar_init(struct domain* d);

/* IO handlers that write to/read from crossbar registers and control access */
static int crossbar_mmio_read(struct vcpu *v, mmio_info_t *info,
                           register_t *r, void *priv);
static int crossbar_mmio_write(struct vcpu *v, mmio_info_t *info,
                            register_t r, void *priv);

static const struct mmio_handler_ops crossbar_mmio_handler = {
    .read  = crossbar_mmio_read,
    .write = crossbar_mmio_write,
};
#endif /* CONFIG_CROSSBAR_INTC */

void omap5_init_secondary(void);
asm (
        ".text                              \n\t"
        "omap5_init_secondary:              \n\t"
        "        ldr   r12, =0x102          \n\t" /* API_HYP_ENTRY */
        "        adr   r0, omap5_hyp        \n\t"
        "        smc   #0                   \n\t"
        "omap5_hyp:                         \n\t"
        "        b     init_secondary       \n\t"
        );

static uint16_t num_den[8][2] = {
    {         0,          0 },  /* not used */
    {  26 *  64,  26 *  125 },  /* 12.0 Mhz */
    {   2 * 768,   2 * 1625 },  /* 13.0 Mhz */
    {         0,          0 },  /* not used */
    { 130 *   8, 130 *   25 },  /* 19.2 Mhz */
    {   2 * 384,   2 * 1625 },  /* 26.0 Mhz */
    {   3 * 256,   3 * 1125 },  /* 27.0 Mhz */
    { 130 *   4, 130 *   25 },  /* 38.4 Mhz */
};

/*
 * The realtime counter also called master counter, is a free-running
 * counter, which is related to real time. It produces the count used
 * by the CPU local timer peripherals in the MPU cluster. The timer counts
 * at a rate of 6.144 MHz. Because the device operates on different clocks
 * in different power modes, the master counter shifts operation between
 * clocks, adjusting the increment per clock in hardware accordingly to
 * maintain a constant count rate.
 */
static int omap5_init_time(void)
{
    void __iomem *ckgen_prm_base;
    void __iomem *rt_ct_base;
    unsigned int sys_clksel;
    unsigned int num, den, frac1, frac2;

    ckgen_prm_base = ioremap_nocache(OMAP5_CKGEN_PRM_BASE, 0x20);
    if ( !ckgen_prm_base )
    {
        dprintk(XENLOG_ERR, "%s: PRM_BASE ioremap failed\n", __func__);
        return -ENOMEM;
    }

    sys_clksel = readl(ckgen_prm_base + OMAP5_CM_CLKSEL_SYS) &
        ~SYS_CLKSEL_MASK;

    iounmap(ckgen_prm_base);

    rt_ct_base = ioremap_nocache(REALTIME_COUNTER_BASE, 0x20);
    if ( !rt_ct_base )
    {
        dprintk(XENLOG_ERR, "%s: REALTIME_COUNTER_BASE ioremap failed\n", __func__);
        return -ENOMEM;
    }

    frac1 = readl(rt_ct_base + INCREMENTER_NUMERATOR_OFFSET);
    num = frac1 & ~NUMERATOR_DENUMERATOR_MASK;
    if ( num_den[sys_clksel][0] != num )
    {
        frac1 &= NUMERATOR_DENUMERATOR_MASK;
        frac1 |= num_den[sys_clksel][0];
    }

    frac2 = readl(rt_ct_base + INCREMENTER_DENUMERATOR_RELOAD_OFFSET);
    den = frac2 & ~NUMERATOR_DENUMERATOR_MASK;
    if ( num_den[sys_clksel][1] != num )
    {
        frac2 &= NUMERATOR_DENUMERATOR_MASK;
        frac2 |= num_den[sys_clksel][1];
    }

    writel(frac1, rt_ct_base + INCREMENTER_NUMERATOR_OFFSET);
    writel(frac2 | PRM_FRAC_INCREMENTER_DENUMERATOR_RELOAD,
           rt_ct_base + INCREMENTER_DENUMERATOR_RELOAD_OFFSET);

    iounmap(rt_ct_base);

    return 0;
}

/* Additional mappings for dom0 (not in the DTS) */
static int omap5_specific_mapping(struct domain *d)
{
    /* Map the PRM module */
    map_mmio_regions(d, gaddr_to_gfn(OMAP5_PRM_BASE), 2,
                     maddr_to_mfn(OMAP5_PRM_BASE));

    /* Map the PRM_MPU */
    map_mmio_regions(d, gaddr_to_gfn(OMAP5_PRCM_MPU_BASE), 1,
                     maddr_to_mfn(OMAP5_PRCM_MPU_BASE));

    /* Map the Wakeup Gen */
    map_mmio_regions(d, gaddr_to_gfn(OMAP5_WKUPGEN_BASE), 1,
                     maddr_to_mfn(OMAP5_WKUPGEN_BASE));

    /* Map the on-chip SRAM */
    map_mmio_regions(d, gaddr_to_gfn(OMAP5_SRAM_PA), 32,
                     maddr_to_mfn(OMAP5_SRAM_PA));

#ifdef CONFIG_CROSSBAR_INTC
    /* FIXME: it shouldn't be here, but it needs a domain to be passed in */
    omap5_crossbar_init(d);
#endif /* CONFIG_CROSSBAR_INTC */
    return 0;
}

static int __init omap5_smp_init(void)
{
    void __iomem *wugen_base;

    wugen_base = ioremap_nocache(OMAP5_WKUPGEN_BASE, PAGE_SIZE);
    if ( !wugen_base )
    {
        dprintk(XENLOG_ERR, "Unable to map omap5 MMIO\n");
        return -EFAULT;
    }

    printk("Set AuxCoreBoot1 to %"PRIpaddr" (%p)\n",
           __pa(omap5_init_secondary), omap5_init_secondary);
    writel(__pa(omap5_init_secondary),
           wugen_base + OMAP_AUX_CORE_BOOT_1_OFFSET);

    printk("Set AuxCoreBoot0 to 0x20\n");
    writel(0x20, wugen_base + OMAP_AUX_CORE_BOOT_0_OFFSET);
    iounmap(wugen_base);

    return 0;
}

#ifdef CONFIG_CROSSBAR_INTC
static int crossbar_translate(const u32 *intspec, unsigned int intsize,
                  unsigned int *out_hwirq,
                  unsigned int *out_type)
{
    int installed_irq;
    int crossbar_irq_id = intspec[1];
    dprintk(XENLOG_INFO, "In %s\n", __func__);

    if ( intsize < 3 )
        return -EINVAL;

    if ( out_type )
        *out_type = intspec[2] & IRQ_TYPE_SENSE_MASK;

    /* For SPIs, we need to add 16 more to get the GIC irq ID number */
    if ( intspec[0] )
    {
        *out_hwirq = intspec[1] + 16;
        return 0;
    }

    /*
     * We need to remap those addresses here because we need to use them
     * in the very begining for the console interrupt. omap5_crossbar_init is
     * called later - we can't do it there.
     */
    base_ctrl = ioremap(CTRL_CORE_MPU_IRQ_BASE, 300);
    base_ctrl_page = ioremap(CTRL_CORE_BASE, 0x1000);
    while ( mpu_irq <= CROSSBAR_MAX_LINE_NUM )
    {
        if ( mpu_irq == 4 ||
           ( mpu_irq >= 7 && mpu_irq <= 130 ) ||
           ( mpu_irq >= 133 && mpu_irq <= 138 ) ||
           ( mpu_irq >= 141 && mpu_irq <= 159 ))
        {
            break;
        }
        else
        {
            mpu_irq++;
        }
    }
    if ( mpu_irq >= CROSSBAR_MAX_LINE_NUM )
        /* Map all interrupts that don't fit to one crossbar line */
        mpu_irq = CROSSBAR_MAX_LINE_NUM;

    installed_irq = mpu_irq;
    current_offset = crossbar_offsets[mpu_irq];
    writew(crossbar_irq_id, base_ctrl+current_offset);
    dprintk(XENLOG_INFO, "crossbar_id = %d, mapped to irq = %d, \
            current_offset = %d\n", crossbar_irq_id, installed_irq, current_offset);
    mpu_irq++;
    /* Get the interrupt number and add 32 to skip over local IRQs */
    *out_hwirq = installed_irq + 32;
    return 0;
}

static bool crossbar_register_accessible(mmio_info_t *info)
{
    int i;
    u32 current_offset;

    if ( info->gpa-CTRL_CORE_BASE >= CTRL_CORE_MPU_START_OFFSET && 
         info->gpa-CTRL_CORE_BASE <= CTRL_CORE_MPU_END_OFFSET )
    {
        current_offset = (u32)(info->gpa-CTRL_CORE_BASE-\
                               CTRL_CORE_MPU_START_OFFSET);
        dprintk(XENLOG_G_INFO, "Current crossbar offset = %u\n", current_offset);
        for (i = 0; i < CROSSBAR_NUM_OF_LINES; i++)
        {
            if ( crossbar_offsets[i] == current_offset )
               break;
        }
        /*
         * If the corresponging interrupt hasn't been found or
         * if dom0 tries to write into xen console's crossbar control register
         * we don't allow the access
         */
        if (i == CROSSBAR_NUM_OF_LINES || i == 4)
        {
            dprintk(XENLOG_G_ERR, "Access to the crossbar register \
                    MPU_IRQ_%u, GIC ID = %u was forbidden\n", i, i+32);
            return false;
        }

        dprintk(XENLOG_G_INFO, "Accessing the crossbar register \
                MPU_IRQ_%u, GIC ID = %u\n", i, i+32);
        return true;
    } else
    {
        dprintk(XENLOG_G_INFO, "Not a crossbar register\n");
        return false;
    }
}
static int crossbar_mmio_read(struct vcpu *v, mmio_info_t *info,
                              register_t *r, void *priv)
{
    u16 * ptr = (u16*)((u32)info->gpa - CTRL_CORE_BASE + base_ctrl_page);
    dprintk(XENLOG_G_INFO, "Reading from the unmapped region, r=%u, paddr=%x\n",
            *r, (u32)info->gpa);
    if ( info->gpa-CTRL_CORE_BASE >= CTRL_CORE_MPU_START_OFFSET && 
         info->gpa-CTRL_CORE_BASE <= CTRL_CORE_MPU_END_OFFSET )
    {
        if ( crossbar_register_accessible(info) )
        {
            dprintk(XENLOG_G_INFO, "Reading from the crossbar register\n");
        } else
        {
            dprintk(XENLOG_G_INFO, "Read from the crossbar register was \
                    forbidden\n");
            return 0;
        }
    }
    *r = readw(ptr);
    return 1;
}

static int crossbar_mmio_write(struct vcpu *v, mmio_info_t *info,
                               register_t r, void *priv)
{
    dprintk(XENLOG_G_INFO, "Writing into unmapped region, r=%u, paddr=%x\n",
            r, (u32)info->gpa);
    if ( info->gpa-CTRL_CORE_BASE >= CTRL_CORE_MPU_START_OFFSET && 
         info->gpa-CTRL_CORE_BASE <= CTRL_CORE_MPU_END_OFFSET )
    {
        if ( crossbar_register_accessible(info) )
        {
            dprintk(XENLOG_G_INFO, "Writing into the crossbar register\n");
        } else
        {
            dprintk(XENLOG_G_INFO, "Write to the crossbar register was \
                    forbidden\n");
            return 0;
        }
    }
    writew((u16)r, (u16*)(u32)(info->gpa - CTRL_CORE_BASE + base_ctrl_page));
    return 1;
}

static int omap5_crossbar_init(struct domain *d)
{
    int i, res, mpu_irq_n;
    struct irq_desc *desc;
    int crb_offset = 0;
    /* Unmap the page with crossbar to control it */
    unmap_mmio_regions(d, gaddr_to_gfn(CTRL_CORE_BASE), 1,
                       maddr_to_mfn(CTRL_CORE_BASE));

    /* Check changes made on the crossbar's page */
    register_mmio_handler(d, &crossbar_mmio_handler,
                          CTRL_CORE_BASE,
                          0x1000,
                          NULL);

    /* Map all irqs so virq=irq */
    for( i = NR_LOCAL_IRQS; i < vgic_num_irqs(d); i++ )
    {
        /*
         * TODO: Exclude the SPIs SMMU uses which should not be routed to
         * the hardware domain.
         */
        desc = irq_to_desc(i);
        if ( desc->action != NULL )
            continue;

        /* XXX: Shall we use a proper devname? */
        res = map_irq_to_domain(d, i, true, "CROSSBAR");
        if ( res )
            return res;
    }
    base_ctrl = ioremap(CTRL_CORE_MPU_IRQ_BASE, 300);
    base_ctrl_page = ioremap(CTRL_CORE_BASE, 0x1000);
    mpu_irq_n = 4; //starting address
    while ( mpu_irq_n < CROSSBAR_NUM_OF_LINES )
    {
        /*
         * We can use only available crossbar lines.
         * Generally, we should use ti,irqs-skip property from device tree.
         * But it is not implemented yet.
         */
        if ( mpu_irq_n == 4 ||
           ( mpu_irq_n >= 7 && mpu_irq_n <= 130 ) ||
           ( mpu_irq_n >= 133 && mpu_irq_n <= 159 ))
        {
            crossbar_offsets[mpu_irq_n] = crb_offset;
            /*
             * Since available IRQ lines are placed linearly in memory with
             * every next register being two bytes aligned.
             */
            crb_offset += 2;
            mpu_irq_n++;
        }
        else
        {
            crossbar_offsets[mpu_irq_n] = BAD_IRQ_LINE;
            mpu_irq_n++;
        }
    }
    return 0;
}

static const char * const crossbar_dt_compat[] __initconst =
{
    "arm,cortex-a15-gic",
    "ti,irq-crossbar",
    "ti,omap5-wugen-mpu",
    "ti,omap4-wugen-mpu",
    NULL
};

bool crossbar_irq_is_routable(const struct dt_raw_irq * rirq)
{
    dprintk(XENLOG_DEBUG, "In crossbar_irq_is_routable\n");
    if ( true )
    {
        int i;
        /* ARRAY_SIZE doesn't work for platform->irq_compatible */
        for (i = 0; i < 4; i++)
        {
            if ( dt_device_is_compatible(rirq->controller,
                                         crossbar_dt_compat[i]) )
                return true;
        }
    }
    return false;
}
#endif /* CONFIG_CROSSBAR_INTC */

static const char * const omap5_dt_compat[] __initconst =
{
    "ti,omap5",
    NULL
};

static const char * const dra7_dt_compat[] __initconst =
{
    "ti,dra7",
    NULL
};


PLATFORM_START(omap5, "TI OMAP5")
    .compatible = omap5_dt_compat,
    .init_time = omap5_init_time,
    .specific_mapping = omap5_specific_mapping,
    .smp_init = omap5_smp_init,
    .cpu_up = cpu_up_send_sgi,
PLATFORM_END

PLATFORM_START(dra7, "TI DRA7")
    .compatible = dra7_dt_compat,
    .specific_mapping = omap5_specific_mapping,
    .init_time = omap5_init_time,
    .cpu_up = cpu_up_send_sgi,
    .smp_init = omap5_smp_init,
    /* Crossbar is presented only in some SoCs, e.g. OMAP AM5728 */
#ifdef CONFIG_CROSSBAR_INTC
    .irq_is_routable = crossbar_irq_is_routable,
    .irq_translate = crossbar_translate,
    .irq_compatible = crossbar_dt_compat,
#endif /* CONFIG_CROSSBAR_INTC */
PLATFORM_END

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
