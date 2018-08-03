/*********************************************************************************************
    Copyright (c) 2018, Harman Connected Services
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.

    3. Neither the name of Harman Connected Services nor the names of its
       contributors may be used to endorse or promote products derived from
       this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS LISTED "AS IS" AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS LISTED BE LIABLE FOR
    ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************/

/*********************************************************************************************
    Copyright (c) 2012, Intel Corporation
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.

    3. Neither the name of the Intel Corporation nor the names of its
       contributors may be used to endorse or promote products derived from
       this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS LISTED "AS IS" AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS LISTED BE LIABLE FOR
    ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************/

#include <stdio.h>
#include <pci/pci.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include "lib/igb.h"
#include "kmod/e1000_regs.h"

#define IGB_BIND_NAMESZ (24)
#define I210_NO_QUEUES (4)
#define FULL_NAME_SIZE (10)

#define VERSION ("1.0")

/* Per Queue Transmit Drop Packets Count */
#define E1000_TQDPC(_n)	((0x0E030 + ((_n) * 0x40)))
/* Per Queue Good Received Packets Count */
#define E1000_PQGPRC(_n)	(0x010010 + (0x100 * (_n)))
/* Per Queue Multicast Packets Received Count */
#define E1000_PQMPRC(_n)	(0x010038 + (0x100 * (_n)))

typedef struct
{
    const char *description;
    const char *mnemonic;
    const unsigned base_address;
    const unsigned perqoffset;      /* addr = base_address + perqoffset * q_index */
} i210_register;

const i210_register regs_table[] =
{
    /* Stats and errors */
    {"[Received Missed packets Count]", "MPC", E1000_MPC, 0},
    {"[Total Received Packets Count]", "TPR", E1000_TPR, 0},
    {"[Good Packets Received Count]", "GPRC", E1000_GPRC,  0},
    {"[Broadcast Packets Received Count]", "BPRC", E1000_BPRC, 0},
    {"[Multicast Packets Received Count]", "MPRC", E1000_MPRC, 0},
    {"[Code Violation Packet Count]", "SCVPC", E1000_SCVPC, 0},
    {"[Rx Error Count]", "RXERRC", E1000_RXERRC, 0},
    {"[Symbol Error Count]", "SYMERRS", E1000_SYMERRS, 0},
    {"[CRC Error Count]", "CRCERRS", E1000_CRCERRS, 0},
    {"[Length Error Count]", "LENERRS", E1000_LENERRS, 0},
    /* Tx queues */
    {"[Transmit Control Register]", "TCTL", E1000_TCTL, 0},
    {"[Per Queue Transmit Descriptor Control]", "TXDCTL", E1000_TXDCTL(0), 0x100},
    {"[Per Queue Good Packets Transmitted Counters]", "PQGPTC", E1000_PQGPTC(0), 0x100},
    {"[Per Queue Transmit Drop Packets Counters]", "TQDPC", E1000_TQDPC(0), 0x40},
    /* Rx queues */
    {"[Receive Control Register]", "RCTL", E1000_RCTL, 0},
    {"[Per Queue Received Descriptor Control]", "RXDCTL", E1000_RXDCTL(0), 0x100},
    {"[Per Queue Good Packets Received Counters]", "PQGPRC", E1000_PQGPRC(0), 0x100},
    {"[Per Queue Received Drop Packets Counters]", "RQDPC", E1000_RQDPC(0), 0x100},
    {"[Per Queue Multicast Received Packets Counters]", "PQMPRC", E1000_PQMPRC(0), 0x100},
};

/* based on linux_hal_i210.cpp */
static int pci_connect(device_t *igb_dev)
{
    char devpath[IGB_BIND_NAMESZ];
    struct pci_access *pacc;
    struct pci_dev *dev;
    int err;

    memset(igb_dev, 0, sizeof(device_t));
    pacc = pci_alloc();
    pci_init(pacc);
    pci_scan_bus(pacc);

    for (dev = pacc->devices; dev; dev = dev->next) {
        pci_fill_info(dev, PCI_FILL_IDENT | PCI_FILL_BASES | PCI_FILL_CLASS);
        igb_dev->pci_vendor_id = dev->vendor_id;
        igb_dev->pci_device_id = dev->device_id;
        igb_dev->domain = dev->domain;
        igb_dev->bus = dev->bus;
        igb_dev->dev = dev->dev;
        igb_dev->func = dev->func;
        snprintf(devpath, IGB_BIND_NAMESZ, "%04x:%02x:%02x.%d",
                 dev->domain, dev->bus, dev->dev, dev->func);
        err = igb_probe(igb_dev);
        if (err) {
            continue;
        }
        fprintf(stdout, "\nAttaching to %s\n", devpath);
        err = igb_attach(devpath, igb_dev);
        if (err) {
            fprintf(stderr, "ERROR! attach failed! (%s)\n", strerror(err));
            continue;
        }
        goto out;
    }
    pci_cleanup(pacc);
    return ENXIO;
out:
    pci_cleanup(pacc);
    return 0;
}

static int read_reg(device_t *igb_dev, unsigned reg, unsigned *val)
{
    igb_lock(igb_dev);
    igb_readreg(igb_dev, reg, val);
    igb_unlock(igb_dev);
    return 0;
}

static int print_label(const char * line)
{
    fprintf(stdout,"\n---------------------------------------------------------------------------\n");
    fprintf(stdout, "%s", line);
    fprintf(stdout,"\n---------------------------------------------------------------------------\n");
    return 0;
}

static int read_and_print(const char *name, unsigned addr, device_t *igb_dev)
{
    struct timespec now;
    unsigned reg_value = 0;
    read_reg(igb_dev, addr, &reg_value);
    if (clock_gettime(CLOCK_MONOTONIC, &now) < 0)
    {
        fprintf(stderr, "ERROR! clock_gettime: %s(%d)\n", strerror(errno), errno);
        return errno;
    }
    fprintf(stdout, "%s\t(0x%06x)\t0x%08x\t%08u\tts: %ld,%ld\n", name, addr, reg_value, reg_value,
            now.tv_sec, now.tv_nsec);
    return 0;
}

static int read_registers_set(device_t *igb_dev)
{
    int regs_count = sizeof(regs_table) / sizeof(i210_register);
    char full_name[FULL_NAME_SIZE] = "";
    int r_index, q_index;
    const i210_register *i210reg;
    for (r_index = 0; r_index < regs_count; r_index++)
    {
        i210reg = &regs_table[r_index];
        print_label(i210reg->description);
        if (i210reg->perqoffset > 0)
        {
            for (q_index = 0; q_index < I210_NO_QUEUES; q_index++)
            {
                snprintf(full_name, FULL_NAME_SIZE, "%s%c", i210reg->mnemonic, (char)q_index + 0x30);
                read_and_print(full_name, i210reg->base_address + i210reg->perqoffset * q_index, igb_dev);
            }
        }
        else {
            read_and_print(i210reg->mnemonic, i210reg->base_address, igb_dev);
        }
    }
    return 0;
}

int main()
{
    unsigned addr;
    device_t _dev;
    device_t *igb_dev = &_dev;
    if( pci_connect(igb_dev) != 0 ) {
        fprintf(stderr, "ERROR! connect failed: %s(%d)- are you running as root?\n", strerror(errno), errno);
        return errno;
    }
    if( igb_init(igb_dev) != 0 ) {
        fprintf(stderr, "ERROR! init failed: %s(%d) - is the driver loaded?\n", strerror(errno), errno);
        return errno;
    }

    read_registers_set(igb_dev);
    print_label("Qav specific registers ...");
    /* Qav specific (raw example) */
    addr = E1000_I210_TQAVCTRL;
    print_label("[Transmit Qav Control Register]");
    read_and_print("TQAVCTRL", addr, igb_dev);

    print_label("[Transmit Qav]");
    addr = E1000_I210_TQAVCC(0);
    read_and_print("TQAVCC0", addr, igb_dev);
    addr = E1000_I210_TQAVCC(1);
    read_and_print("TQAVCC1", addr, igb_dev);

    print_label("[Transmit Qav Credits]");
    addr = E1000_I210_TQAVHC(0);
    read_and_print("TQAVHC0", addr, igb_dev);
    addr = E1000_I210_TQAVHC(1);
    read_and_print("TQAVHC1", addr, igb_dev);

    print_label("Exiting ...");
    return 0;
}
