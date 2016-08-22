/*
 * pci4e.h -- definitions etc.
 * Original file phob.h by rubini@linux.it (Alessandro Rubini) was modified
 * by replacing all occurences of PHOB and phob with PCI4E and pci4e respectively.  
 * Not much else has been modified in file.
 * The pci4e driver simply uses the open, ioctl, and release methods.
 * The ioctl interface only uses PCI4E_IOCREAD and PCI4E_IOCWRITE.
 */

#ifndef _PCI4E_H_
#define _PCI4E_H_

#include <linux/types.h>  /* we need __u32 etc */

#define PCI4E_VERSION "1.0"
#define PCI4E_MSG "pci4e: "

/* messaging stuff */
#undef PDEBUG             /* undef it, just in case */
#undef DEBUG_CODE
#ifdef PHOB_DEBUG
#  ifdef __KERNEL__
     /* This one if debugging is on, and kernel space */
#    define PDEBUG(fmt, args...) printk( KERN_DEBUG PHOB_MSG fmt, ## args)
#    define DEBUG_CODE(code) code
#  else
     /* This one for user space */
#    define PDEBUG(fmt, args...) fprintf(stderr, fmt, ## args)
#  endif
#else
#  define PDEBUG(fmt, args...) /* not debugging: nothing */
#  define DEBUG_CODE(code)
#endif

#undef PDEBUGG
#define PDEBUGG(fmt, args...) /* nothing: it's a placeholder */

/* The minor number is split into two bitfields */

#define PCI4E_TYPE(inode)  ((MINOR((inode)->i_rdev)) & 0x0f)
#define PCI4E_DEV(inode)  ((MINOR((inode)->i_rdev)) >> 4)
#define PCI4E_NR_DEVS 16

enum {
    PCI4E_TYPE_PCI     = 0,
    PCI4E_TYPE_REGION0 = 1,
    PCI4E_TYPE_REGION1 = 2,
    PCI4E_TYPE_REGION2 = 3,
    PCI4E_TYPE_REGION3 = 4,
    PCI4E_TYPE_REGION4 = 5,
    PCI4E_TYPE_REGION5 = 6,
    PCI4E_TYPE_IRQ     = 7,
    PCI4E_TYPE_MEM     = 8,
    PCI4E_TYPE_MAX /* max and above are invalid */
};
#define PCI4E_TYPE_REGION_MIN PCI4E_TYPE_REGION0
#define PCI4E_TYPE_REGION_MAX PCI4E_TYPE_REGION5

/* internal description of one PCI region */
struct pci4e_region {
    __u32 maskbits;    /* io, mem, etc */
    __u32 address;
    __u32 len;
    void *remap; /* if memory */
};

/* internal instruction of the IRQ machine */
struct pci4e_insn {
    __u8 opcode;/* messaging stuff */
#undef PDEBUG             /* undef it, just in case */
#undef DEBUG_CODE
#ifdef PHOB_DEBUG
#  ifdef __KERNEL__
     /* This one if debugging is on, and kernel space */
#    define PDEBUG(fmt, args...) printk( KERN_DEBUG PHOB_MSG fmt, ## args)
#    define DEBUG_CODE(code) code
#  else
     /* This one for user space */
#    define PDEBUG(fmt, args...) fprintf(stderr, fmt, ## args)
#  endif
#else
#  define PDEBUG(fmt, args...) /* not debugging: nothing */
#  define DEBUG_CODE(code)
#endif

#undef PDEBUGG
#define PDEBUGG(fmt, args...) /* nothing: it's a placeholder */




    __u8 regnum;
    __u16 arg16;
    __u32 arg32; /* may be a char pointer */
};

enum opcodes { /* NOTE: must be same order as opcodes[] in pci4e_irq.y */
    OPCODE_NONE = 0,
    OPCODE_READ8,
    OPCODE_READ16,
    OPCODE_READ32,
    OPCODE_WRITE8,
    OPCODE_WRITE16,
    OPCODE_WRITE32,/* messaging stuff */
#undef PDEBUG             /* undef it, just in case */
#undef DEBUG_CODE
#ifdef PHOB_DEBUG
#  ifdef __KERNEL__
     /* This one if debugging is on, and kernel space */
#    define PDEBUG(fmt, args...) printk( KERN_DEBUG PHOB_MSG fmt, ## args)
#    define DEBUG_CODE(code) code
#  else
     /* This one for user space */
#    define PDEBUG(fmt, args...) fprintf(stderr, fmt, ## args)
#  endif
#else
#  define PDEBUG(fmt, args...) /* not debugging: nothing */
#  define DEBUG_CODE(code)
#endif

#undef PDEBUGG
#define PDEBUGG(fmt, args...) /* nothing: it's a placeholder */




    OPCODE_MEMPAGE,
    OPCODE_OR,
    OPCODE_AND,
    OPCODE_XOR,
    OPCODE_ADD,
    OPCODE_SUB,
    OPCODE_LOAD,
    OPCODE_STORE,
    OPCODE_PRINTK,
    OPCODE_PRINTF,
    OPCODE_JZ,
    OPCODE_JNZ,
    OPCODE_JMP,
    OPCODE_RET,
    OPCODE_MAX /* This *must* be last (used by interpreter.c) */
};

/* structure used to pass irq handler data to kernel space */
struct pci4e_irqdata {
    int insnlen, stringlen;
    struct pci4e_insn *insn;
    char *string;
};
    
/* state-machine for interrupt management */
struct pci4e_machine {
    unsigned long ip;
    struct pci4e_struct *device;
    __u32 accumulator;
    __u32 regs[15];
};


/* overall device structure */
struct pci4e_struct {
    unsigned short vendor_id;
    unsigned short device_id;
    unsigned short device_class;
    unsigned short filler;
    int irq;
    int active_irq;

    struct pci_dev *pcidev;
    struct pci4e_region region[6];
    unsigned long memory;
    struct pci4e_irqdata irqdata;
    struct pci4e_machine machine;
    void (*handler)(struct pci4e_struct *dev);
};

extern struct pci4e_struct *pci4e_devices[PCI4E_NR_DEVS];

/* private data remembers the type as well */
struct pci4e_file {
    struct pci4e_struct *dev;
    int type;
};


/* this is used by ioctl() commands */
struct pci4e_io_struct {
    unsigned long offset; /* within the region */
    unsigned long value;   /* read or written */
    unsigned short len;    /* 1, 2, 4 */
    unsigned char region;  /* 'p', '0', ... 'm' */
    unsigned char filler;
};

/*
 * ioctl() commands
 */
#define PCI4E_IOC_MAGIC  'p' /* Use 'p' as magic number */

#define PCI4E_IOCUNUSE        _IO(PCI4E_IOC_MAGIC,  0)
#define PCI4E_IOCREAD       _IOWR(PCI4E_IOC_MAGIC,  1, struct pci4e_io_struct)
#define PCI4E_IOCWRITE       _IOW(PCI4E_IOC_MAGIC,  2, struct pci4e_io_struct)
#define PCI4E_IOCGADDRESS    _IOR(PCI4E_IOC_MAGIC,  3, unsigned long)
#define PCI4E_IOCIRQPROC     _IOW(PCI4E_IOC_MAGIC,  4, struct pci4e_irqdata)
#define PCI4E_IOCENABLEIRQ    _IO(PCI4E_IOC_MAGIC,  5)

#define PCI4E_IOC_MAXNR                            5

#ifdef __KERNEL__

extern void pci4e_interpreter(struct pci4e_struct *dev);

#endif /* __KERNEL__ */

#endif /* _PCI4E_H_ */
