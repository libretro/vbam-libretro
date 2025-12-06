#include "core/base/port.h"
#include "gbaGlobals.h"
#include "gbaInline.h"

extern uint32_t dma0Source;
extern uint32_t dma1Source;
extern uint32_t dma2Source;
extern uint32_t dma3Source;

extern uint32_t dma0Dest;
extern uint32_t dma1Dest;
extern uint32_t dma2Dest;
extern uint32_t dma3Dest;

extern uint32_t cpuDmaLatchData[];

extern int cpuDmaTicksToUpdate;
extern int cpuDmaCount;

extern uint32_t cpuDmaSrcMask[4];
extern uint32_t cpuDmaDstMask[4];

struct DMAChannel {
    uint16_t &cnt_h;
    uint16_t &cnt_l;
    uint32_t &src;
    uint32_t &dest;
    uint32_t &latch;
    uint32_t count;
    bool sound;
};

DMAChannel dmaCh[4] = {
    { DM0CNT_H, DM0CNT_L, dma0Source, dma0Dest, cpuDmaLatchData[0], 0, false },
    { DM1CNT_H, DM1CNT_L, dma1Source, dma1Dest, cpuDmaLatchData[1], 0, false },
    { DM2CNT_H, DM2CNT_L, dma2Source, dma2Dest, cpuDmaLatchData[2], 0, false },
    { DM3CNT_H, DM3CNT_L, dma3Source, dma3Dest, cpuDmaLatchData[3], 0, false },
};

static inline void dma_update_addr(uint32_t &addr, int addr_ctrl, int wsize) {
    if (((addr >> 24) >= REGION_ROM0) && ((addr >> 24) < REGION_SRAM)) {
        addr += wsize;
    } else {
        switch (addr_ctrl) {
        case 0:
        case 3:
            addr += wsize;
            break;
        case 1:
            addr -= wsize;
            break;
        }
    }
}

static inline void dma_transfer_word_loop(DMAChannel *dmac,
    int &srcWait, int &destWait)
{
    bool first = true;

    int sm = (int)((dmac->src >> 24) & 0xFF);
    int dm = (int)((dmac->dest >> 24) & 0xFF);

    do {
        bool srcInBios = (sm < REGION_EWRAM);

        uint32_t value = CPUReadMemory(dmac->src);
        if (srcInBios) value = dmac->latch;
        else dmac->latch = value;
        CPUWriteMemory(dmac->dest, value);
        cpuDmaBusValue = value;

        srcWait  += first ? memoryWait32[sm & 15] : memoryWaitSeq32[sm & 15];
        destWait += first ? memoryWait32[dm & 15] : memoryWaitSeq32[dm & 15];

        dma_update_addr(dmac->src, (dmac->cnt_h >> 7 & 3), (2 << ((dmac->cnt_h >> 10) & 1)));
        if (!dmac->sound)
            dma_update_addr(dmac->dest, (dmac->cnt_h >> 5 & 3), (2 << ((dmac->cnt_h >> 10) & 1)));

        first = false;
    } while (--dmac->count > 0);
}

static inline void dma_transfer_halfword_loop(DMAChannel *dmac,
    int &srcWait, int &destWait)
{
    bool first = true;

    int sm = (int)((dmac->src >> 24) & 0xFF);
    int dm = (int)((dmac->dest >> 24) & 0xFF);

    do {
        bool srcInBios = (sm < REGION_EWRAM);

        uint32_t value = CPUReadHalfWord(dmac->src);
        if (srcInBios) value = dmac->latch;
        else dmac->latch = value * 0x00010001;
        CPUWriteHalfWord(dmac->dest, Downcast16(value));
        cpuDmaBusValue = value * 0x00010001;

        srcWait  += first ? memoryWait[sm & 15] : memoryWaitSeq[sm & 15];
        destWait += first ? memoryWait[dm & 15] : memoryWaitSeq[dm & 15];

        dma_update_addr(dmac->src, (dmac->cnt_h >> 7 & 3), (2 << ((dmac->cnt_h >> 10) & 1)));
        if (!dmac->sound) // this dont exist in 16-bit transfer but whatever
            dma_update_addr(dmac->dest, (dmac->cnt_h >> 5 & 3), (2 << ((dmac->cnt_h >> 10) & 1)));
        
        first = false;
    } while (--dmac->count > 0);
}

void CPUCheckDMA(int reason, int dmamask) {
    for (int i = 0; i < 4; i++) {
        DMAChannel *dmac = &dmaCh[i];

        bool enable         = dmac->cnt_h & 0x8000;
        uint8_t startTiming = (dmac->cnt_h >> 12) & 3;

        if (!(enable && (dmamask & (1 << i)))) {
            continue;
        }
        if (startTiming != reason) {
            continue;
        }

        bool transfer32 = (dmac->cnt_h & 0x400);

        if (reason == 3 && (i == 1 || i == 2)) dmac->sound = true;

        if (dmac->sound) dmac->count = 4;
        else dmac->count = dmac->cnt_l;
        if (dmac->count == 0) {
            if (i < 3) dmac->count = 0x4000;
            else dmac->count = 0x10000;
        }

        dmac->src &= cpuDmaSrcMask[i];
        dmac->dest &= cpuDmaDstMask[i];
        if (transfer32) {
            dmac->src &= ~3;
            dmac->dest &= ~3;
        } else {
            dmac->src &= ~1;
            dmac->dest &= ~1;
        }

        cpuDmaRunning = true;
        cpuDmaPC      = reg[15].I;
        cpuDmaCount   = dmac->count;

        int sw = 0;
        int dw = 0;

        if (transfer32 || dmac->sound) {
            dma_transfer_word_loop(    dmac, sw, dw);
        } else {
            dma_transfer_halfword_loop(dmac, sw, dw);
        }

        dmac->sound = false;
        cpuDmaCount = 0;

        cpuDmaTicksToUpdate += 4 + sw + dw;
        cpuDmaRunning = false;

        if (dmac->cnt_h & 0x4000) {
            IF |= (0x0100 << i);
            UPDATE_REG(IO_REG_IF, IF);
            cpuNextEvent = cpuTotalTicks;
        }

        if (((dmac->cnt_h >> 5) & 3) == 3) {
            dmac->dest = READ32LE(&g_ioMem[IO_REG_DMA0SAD + (12 * i)]);
            dmac->dest &= cpuDmaDstMask[i];
            if (transfer32) dmac->dest &= ~3;
            else dmac->dest &= ~1;
        }

        if (!(dmac->cnt_h & 0x200) || (reason == 0)) {
            dmac->cnt_h &= ~0x8000;
            UPDATE_REG(IO_REG_DMA0CTL + (12 * i), dmac->cnt_h);
        }
    }
}
