/*
 * domain.h: HVM per domain definitions
 *
 * Copyright (c) 2004, Intel Corporation.
 * Copyright (c) 2005, International Business Machines Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59 Temple
 * Place - Suite 330, Boston, MA 02111-1307 USA.
 */

#ifndef __ASM_X86_HVM_DOMAIN_H__
#define __ASM_X86_HVM_DOMAIN_H__

#include <xen/iommu.h>
#include <asm/hvm/irq.h>
#include <asm/hvm/vpt.h>
#include <asm/hvm/vlapic.h>
#include <asm/hvm/vioapic.h>
#include <asm/hvm/io.h>
#include <xen/hvm/iommu.h>
#include <asm/hvm/viridian.h>
#include <asm/hvm/vmx/vmcs.h>
#include <asm/hvm/svm/vmcb.h>
#include <public/grant_table.h>
#include <public/hvm/params.h>
#include <public/hvm/save.h>

struct hvm_ioreq_page {
    struct page_info *page;
    void *va;
};

struct hvm_ioreq_vcpu {
    struct list_head list_entry;
    struct vcpu      *vcpu;
    evtchn_port_t    ioreq_evtchn;
};

struct hvm_ioreq_server {
    struct domain          *domain;

    /* Lock to serialize toolstack modifications */
    spinlock_t             lock;

    /* Domain id of emulating domain */
    domid_t                domid;
    struct hvm_ioreq_page  ioreq;
    struct list_head       ioreq_vcpu_list;
    struct hvm_ioreq_page  bufioreq;

    /* Lock to serialize access to buffered ioreq ring */
    spinlock_t             bufioreq_lock;
    evtchn_port_t          bufioreq_evtchn;
};

struct hvm_domain {
    spinlock_t              ioreq_server_lock;
    struct hvm_ioreq_server *ioreq_server;

    struct pl_time         pl_time;

    struct hvm_io_handler *io_handler;

    /* Lock protects access to irq, vpic and vioapic. */
    spinlock_t             irq_lock;
    struct hvm_irq         irq;
    struct hvm_hw_vpic     vpic[2]; /* 0=master; 1=slave */
    struct hvm_vioapic    *vioapic;
    struct hvm_hw_stdvga   stdvga;

    /* VCPU which is current target for 8259 interrupts. */
    struct vcpu           *i8259_target;

    /* emulated irq to pirq */
    struct radix_tree_root emuirq_pirq;

    uint64_t              *params;

    /* Memory ranges with pinned cache attributes. */
    struct list_head       pinned_cacheattr_ranges;

    /* VRAM dirty support. */
    struct sh_dirty_vram *dirty_vram;

    /* If one of vcpus of this domain is in no_fill_mode or
     * mtrr/pat between vcpus is not the same, set is_in_uc_mode
     */
    spinlock_t             uc_lock;
    bool_t                 is_in_uc_mode;

    /* Pass-through */
    struct hvm_iommu       hvm_iommu;

    /* hypervisor intercepted msix table */
    struct list_head       msixtbl_list;
    spinlock_t             msixtbl_list_lock;

    struct viridian_domain viridian;

    bool_t                 hap_enabled;
    bool_t                 mem_sharing_enabled;
    bool_t                 qemu_mapcache_invalidate;
    bool_t                 is_s3_suspended;

    /*
     * TSC value that VCPUs use to calculate their tsc_offset value.
     * Used during initialization and save/restore.
     */
    uint64_t sync_tsc;

    union {
        struct vmx_domain vmx;
        struct svm_domain svm;
    };
};

#define hap_enabled(d)  ((d)->arch.hvm_domain.hap_enabled)

#endif /* __ASM_X86_HVM_DOMAIN_H__ */

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
