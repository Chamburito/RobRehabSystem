/*******************************/
/* Memory management functions */
/*******************************/

#ifndef _KVMEM_H
#define _KVMEM_H
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 4, 0)
   #include <linux/slab.h>
#else
   #include <linux/malloc.h>
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 5, 0)
#else
#include <linux/wrapper.h>
#endif
#include <linux/pagemap.h>
#include <asm/pgtable.h>
#include <asm/io.h>
#include "kernel_compat.h"

#ifndef INIT_MM_EXPORTED 
 #define INIT_MM_EXPORTED (((LINUX_VERSION_CODE>=KERNEL_VERSION(2,2,14))&&\
(LINUX_VERSION_CODE<KERNEL_VERSION(2,3,0)))||\
(LINUX_VERSION_CODE>=KERNEL_VERSION(2,3,40)))
#endif

#if INIT_MM_EXPORTED /* true since 2.2.14 and 2.3.40 */
/* Use new implementation by David Miller. This is taken from bttv.c from
linux-2.3.47pre6 and edited by me (Tomasz Motylewski) for brevity
It looks like we have 5 types of addresses: physical|phys|pa, bus, kva,
uvirt, and kvirt. WOW !
*/
/* [DaveM] I've recoded most of this so that:
 * 1) It's easier to tell what is happening
 * 2) It's more portable, especially for translating things
 *    out of vmalloc mapped areas in the kernel.
 * 3) Less unnecessary translations happen.
 *
 * The code used to assume that the kernel vmalloc mappings
 * existed in the page tables of every process, this is simply
 * not guarenteed.  We now use pgd_offset_k which is the
 * defined way to get at the kernel page tables.
 */

/* Given PGD from the address space's page table, return the kernel
 * virtual mapping of the physical memory mapped at ADR.
 */
static inline unsigned long uvirt_to_kva(pgd_t *pgd, unsigned long adr)
{
    unsigned long ret = 0UL;
    pmd_t *pmd;
    pte_t pte;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 11)
    pud_t *pud;
#endif
    
    if (!pgd_none(*pgd)) 
    {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 11)
        pud = pud_offset(pgd, adr);
        pmd = pmd_offset(pud, adr);
#else
        pmd = pmd_offset(pgd, adr);
#endif
        if (!pmd_none(*pmd)) 
        {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 5, 0) || defined(pte_offset_kernel) && !defined(pte_offset)
            pte = *(pte_offset_kernel(pmd, adr));
#else
            pte = *(pte_offset(pmd, adr));
#endif
            if(pte_present(pte))
#if (LINUX_VERSION_CODE>=KERNEL_VERSION(2,3,0))
                ret  = (unsigned long) page_address(pte_page(pte));
            ret |= (adr & (PAGE_SIZE - 1));
/*          ret = ( page_address( pte_page(pte)) 
                    | (adr&(PAGE_SIZE-1))); */
#else
            ret =  pte_page(pte) | (adr&(PAGE_SIZE-1));
#endif
        }
    }
    return ret;
}

static inline unsigned long uvirt_to_phys(unsigned long adr)
{
    return __pa(uvirt_to_kva(pgd_offset(current->mm, adr),  adr) );
}

static inline unsigned long uvirt_to_bus(unsigned long adr) 
{
    return virt_to_bus((void *) uvirt_to_kva(
                pgd_offset(current->mm, adr), adr) );
}

static inline unsigned long kvirt_to_bus(unsigned long adr)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 5, 0)
    adr = VMALLOC_VMADDR(adr);
#endif
    return virt_to_bus((void *) uvirt_to_kva(pgd_offset_k(adr), adr) );
}

/* Here we want the physical address of the memory.
 * This is used when initializing the contents of the
 * area and marking the pages as reserved.
 */
static inline unsigned long kvirt_to_pa(unsigned long adr)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 5, 0)
    adr = VMALLOC_VMADDR(adr);
#endif
    return __pa(uvirt_to_kva(pgd_offset_k(adr), adr));
}
#define kvirt_to_phys  kvirt_to_pa



#else  /* INIT_MM_EXPORTED */

/* convert virtual memory address to physical address */
/* (virt_to_phys only works for kmalloced kernel memory) */

static inline unsigned long uvirt_to_phys(unsigned long adr)
{
    pgd_t *pgd;
    pmd_t *pmd;
    pte_t *ptep, pte;
  
/* this just happens to work for kvirt, but see above remarks of DaveM */
    pgd = pgd_offset(current->mm, adr);
    if (pgd_none(*pgd))
        return 0;
    pmd = pmd_offset(pgd, adr);
    if (pmd_none(*pmd))
        return 0;
    ptep = pte_offset(pmd, adr/*&(~PGDIR_MASK)*/);
    pte = *ptep;
    if(pte_present(pte))
        return virt_to_phys((void *)(pte_page(pte)|(adr&(PAGE_SIZE-1))));
    return 0;
}

static inline unsigned long uvirt_to_bus(unsigned long adr) 
{
    return virt_to_bus(phys_to_virt(uvirt_to_phys(adr)));
}

/* kvirt_to_phys pauperum */
static inline unsigned long kvirt_to_phys(unsigned long adr) 
{
    return uvirt_to_phys(VMALLOC_VMADDR(adr));
}

static inline unsigned long kvirt_to_bus(unsigned long adr) 
{
    return uvirt_to_bus(VMALLOC_VMADDR(adr));
}

#endif /* INIT_MM_EXPORTED */

extern void * rvmalloc(unsigned long size);
extern void rvfree(void * mem, unsigned long size);
extern int rvmmap(void *mem, unsigned memsize, struct vm_area_struct *vma);
#endif

