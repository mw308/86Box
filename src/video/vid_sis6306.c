/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *
 *          Implementation of the SiS 530 integrated 3D VGA (PCI ID 1039:6306).
 *
 *
 * Authors: mw308
 *
 *          Copyright 2026 mw308
 *
 * Implements this 2D functionality: PCI/AGP identity and standard VGA,
 * SiS enhanced packed modes, segment-selection banking, interlaced
 * display handling, 2D acceleration, the SiS 64x64 two-bit hardware
 * cursor, MMIO cursor positioning, and Win9x host-to-VRAM BitBlt staging.
 *
 * 3D acceleration is not yet implemented.
 */

#include <stdint.h>
#include <stdlib.h>

#include <86box/86box.h>
#include <86box/device.h>
#include <86box/i2c.h>
#include <86box/io.h>
#include <86box/mem.h>
#include <86box/pci.h>
#include <86box/video.h>
#include <86box/vid_ddc.h>
#include <86box/vid_svga.h>
#include <86box/vid_svga_render.h>


typedef struct sis6306_t {
    svga_t svga;
    uint8_t pci_slot;
    uint8_t pci_command;
    uint8_t irq_line;

    uint32_t lfb_base;
    uint32_t mmio_base;
    uint32_t io_base;
    uint16_t io_active_base;

    mem_mapping_t lfb_mapping;
    mem_mapping_t mmio_mapping;

    uint8_t mmio_regs[0x10000];
    uint8_t io_regs[0x80];

    void *i2c;
    void *ddc;

    uint8_t ext_seq[256];
    uint8_t ext_seq_unlocked;

    uint8_t sis_crtc_index;
    uint8_t ext_crtc[256];
    uint8_t ext_crtc_unlocked;

    /* SiS segment-selection ports 3CDh/3CBh. */
    uint8_t segment0;
    uint8_t segment1;

    uint8_t  cursor_pattern_valid;
} sis6306_t;


static video_timings_t timing_sis6306 = {
    .type = VIDEO_AGP,
    .write_b = 2, .write_w = 2, .write_l = 1,
    .read_b = 10, .read_w = 10, .read_l = 10
};


/*
 * In SiS enhanced graphics mode the VGA BIOS uses a packed 64 KB window at
 * A0000h together with the 3CBh/3CDh segment registers.  The standard VGA
 * Graphics Controller memory-map value is not the aperture decoder for this
 * path; the IN530 BIOS leaves GR06 at 0Eh while banking through 3CDh/3CBh.
 *
 * The generic SVGA decoder would interpret GR06=0Eh as the B8000h 32 KB
 * standard-VGA window.  Apart from rejecting A0000h accesses, that also means
 * svga_decode_addr() never applies read_bank/write_bank, because generic bank
 * addition is limited to VGA map modes 0 and 1.  Keep standard VGA accesses on
 * the generic path, but provide the byte-packed A0000h bank window explicitly
 * while SR06.D1 (Enhanced Graphics Mode) is set.
 */
static inline int
sis6306_packed_graphics(const sis6306_t *dev)
{
    return !!(dev->ext_seq[0x06] & 0x1e);
}

static inline int
sis6306_enhanced_window_addr(sis6306_t *dev, uint32_t addr, int write,
                             uint32_t *vram_addr)
{
    svga_t   *svga   = &dev->svga;
    uint32_t  legacy = addr & 0x1ffff;
    uint32_t  offset;

    if (!sis6306_packed_graphics(dev) || (legacy >= 0x10000))
        return 0;

    offset = legacy + (write ? svga->write_bank : svga->read_bank);
    if (offset >= svga->vram_max) {
        *vram_addr = 0xffffffff;
        return 1;
    }

    *vram_addr = offset & svga->vram_mask;
    return 1;
}

static uint8_t
sis6306_legacy_readb(uint32_t addr, void *priv)
{
    sis6306_t *dev = (sis6306_t *) priv;
    uint32_t   vram_addr;

    if (!sis6306_enhanced_window_addr(dev, addr, 0, &vram_addr))
        return svga_read(addr, &dev->svga);

    if (vram_addr == 0xffffffff)
        return 0xff;

    return dev->svga.vram[vram_addr];
}

static uint16_t
sis6306_legacy_readw(uint32_t addr, void *priv)
{
    sis6306_t *dev = (sis6306_t *) priv;

    if (!sis6306_packed_graphics(dev))
        return svga_readw(addr, &dev->svga);

    return sis6306_legacy_readb(addr, priv) |
           ((uint16_t) sis6306_legacy_readb(addr + 1, priv) << 8);
}

static uint32_t
sis6306_legacy_readl(uint32_t addr, void *priv)
{
    sis6306_t *dev = (sis6306_t *) priv;

    if (!sis6306_packed_graphics(dev))
        return svga_readl(addr, &dev->svga);

    return sis6306_legacy_readb(addr, priv) |
           ((uint32_t) sis6306_legacy_readb(addr + 1, priv) << 8) |
           ((uint32_t) sis6306_legacy_readb(addr + 2, priv) << 16) |
           ((uint32_t) sis6306_legacy_readb(addr + 3, priv) << 24);
}

static void
sis6306_legacy_writeb(uint32_t addr, uint8_t val, void *priv)
{
    sis6306_t *dev = (sis6306_t *) priv;
    uint32_t   vram_addr;

    if (!sis6306_enhanced_window_addr(dev, addr, 1, &vram_addr)) {
        svga_write(addr, val, &dev->svga);
        return;
    }

    if (vram_addr == 0xffffffff)
        return;

    dev->svga.vram[vram_addr] = val;
    dev->svga.changedvram[vram_addr >> 12] =
        dev->svga.monitor->mon_changeframecount;
}

static void
sis6306_legacy_writew(uint32_t addr, uint16_t val, void *priv)
{
    sis6306_t *dev = (sis6306_t *) priv;

    if (!sis6306_packed_graphics(dev)) {
        svga_writew(addr, val, &dev->svga);
        return;
    }

    sis6306_legacy_writeb(addr, val, priv);
    sis6306_legacy_writeb(addr + 1, val >> 8, priv);
}

static void
sis6306_legacy_writel(uint32_t addr, uint32_t val, void *priv)
{
    sis6306_t *dev = (sis6306_t *) priv;

    if (!sis6306_packed_graphics(dev)) {
        svga_writel(addr, val, &dev->svga);
        return;
    }

    sis6306_legacy_writeb(addr, val, priv);
    sis6306_legacy_writeb(addr + 1, val >> 8, priv);
    sis6306_legacy_writeb(addr + 2, val >> 16, priv);
    sis6306_legacy_writeb(addr + 3, val >> 24, priv);
}


static void
sis6306_update_banks(sis6306_t *dev)
{
    svga_t *svga = &dev->svga;

    /*
     * SR0B bit 3 selects dual-segment mode.
     *
     * Dual-segment mode:
     *   3CDh bits 6:0 = write segment
     *   3CBh bits 6:0 = read segment
     *
     * Single-segment mode:
     *   3CDh bits 7:4 = write segment
     *   3CDh bits 3:0 = read segment
     *
     * Each segment selects a 64 KB bank in the legacy A0000h aperture.
     */
    if (dev->ext_seq[0x0b] & 0x08) {
        svga->write_bank = (uint32_t) (dev->segment0 & 0x7f) << 16;
        svga->read_bank  = (uint32_t) (dev->segment1 & 0x7f) << 16;
    } else {
        svga->write_bank = (uint32_t) ((dev->segment0 >> 4) & 0x0f) << 16;
        svga->read_bank  = (uint32_t) (dev->segment0 & 0x0f) << 16;
    }
}

static uint32_t
sis6306_cursor_color(const sis6306_t *dev, int which)
{
    const int base = which ? 0x17 : 0x14;
    return makecol32(video_6to8[dev->ext_seq[base] & 0x3f],
                     video_6to8[dev->ext_seq[base + 1] & 0x3f],
                     video_6to8[dev->ext_seq[base + 2] & 0x3f]);
}

static uint32_t
sis6306_configured_vram_size(const sis6306_t *dev)
{
    /*
     * SR0C D[4,2:1] is the SiS framebuffer-memory configuration code.
     * The Win9x SiS driver uses the same 3-bit value to select its usable
     * framebuffer size (2/4/8 MB for the configurations used by this chip).
     */
    const unsigned code = ((dev->ext_seq[0x0c] & 0x10) >> 2) |
                          ((dev->ext_seq[0x0c] & 0x06) >> 1);
    uint32_t size;

    switch (code) {
        case 0x0: size = 1U << 20; break;
        case 0x1:
        case 0x5: size = 2U << 20; break;
        case 0x2:
        case 0x6: size = 4U << 20; break;
        case 0x7: size = 8U << 20; break;
        default:  size = dev->svga.vram_max; break;
    }

    if (!size || (size > dev->svga.vram_max))
        size = dev->svga.vram_max;
    return size;
}

static uint32_t
sis6306_cursor_choose_base(sis6306_t *dev)
{
    const uint32_t mem_size = sis6306_configured_vram_size(dev);
    const uint32_t pat = (dev->ext_seq[0x1e] >> 4) & 0x0f;
    uint32_t base = 0;

    /*
     * The SiS Win9x driver reserves the final 16 KB of configured display
     * memory for sixteen 1 KB 64x64x2 cursor patterns.  SR1E[7:4] selects
     * one of those patterns.  In particular pattern F is the final 1 KB;
     * SETCURSOR fills that block with AAh (transparent pixels) and patches
     * the actual AND/XOR cursor into it before enabling SR06.D6.
     */
    if (mem_size >= 0x4000U)
        base = (mem_size - 0x4000U) + (pat << 10);
    base &= dev->svga.vram_mask;

    dev->cursor_pattern_valid = (mem_size >= 0x4000U) &&
                                ((base + 1024U) <= dev->svga.vram_max);


    return base;
}

static void
sis6306_update_cursor(sis6306_t *dev)
{
    svga_t *svga = &dev->svga;
    uint32_t base;

    svga->hwcursor.ena       = !!(dev->ext_seq[0x06] & 0x40);
    svga->hwcursor.cur_xsize = 64;
    svga->hwcursor.cur_ysize = 64;
    if (dev->ext_seq[0x1b] & 0x80) {
        /*
         * In MMIO cursor mode Win9x updates 32-bit cursor position registers
         * at 85F8h (X) and 85FCh (Y).  Position is in the low 12 bits; upper
         * bits are control/off-screen state and must not become coordinates.
         */
        uint32_t mmio_x = (uint32_t) dev->mmio_regs[0x85f8] |
                          ((uint32_t) dev->mmio_regs[0x85f9] << 8) |
                          ((uint32_t) dev->mmio_regs[0x85fa] << 16) |
                          ((uint32_t) dev->mmio_regs[0x85fb] << 24);
        uint32_t mmio_y = (uint32_t) dev->mmio_regs[0x85fc] |
                          ((uint32_t) dev->mmio_regs[0x85fd] << 8) |
                          ((uint32_t) dev->mmio_regs[0x85fe] << 16) |
                          ((uint32_t) dev->mmio_regs[0x85ff] << 24);
        svga->hwcursor.x = mmio_x & 0x0fff;
        svga->hwcursor.y = mmio_y & 0x0fff;
    } else {
        svga->hwcursor.x = dev->ext_seq[0x1a] |
                           ((dev->ext_seq[0x1b] & 0x0f) << 8);
        svga->hwcursor.y = dev->ext_seq[0x1d] |
                           ((dev->ext_seq[0x1e] & 0x07) << 8);
    }

    /*
     * In SiS interlaced modes the hardware cursor vertical coordinate is in
     * field-line units.  This fixes the cursor being stuck in the upper half
	 * of 1024x768.
     */
    if (svga->interlace)
        svga->hwcursor.y <<= 1;
    svga->hwcursor.xoff      = dev->ext_seq[0x1c] & 0x3f;
    svga->hwcursor.yoff      = dev->ext_seq[0x1f] & 0x3f;

    /*
     * The Win9x driver lays the sixteen hardware-cursor patterns out in the
     * final 16 KB of the configured framebuffer; SR1E[7:4] selects a 1 KB
     * slot.  Use the SR0C-configured physical RAM size rather than the full
     * 8 MB PCI aperture.
     */
    base = sis6306_cursor_choose_base(dev);
    svga->hwcursor.addr = (base + (uint32_t) svga->hwcursor.yoff * 16) &
                          svga->vram_mask;
    svga->fullchange = svga->monitor->mon_changeframecount;

}

static void
sis6306_hwcursor_draw_row(svga_t *svga, int displine, uint32_t addr,
                          int xbase, uint32_t col0, uint32_t col1)
{
    uint8_t row[16];

    if ((displine < 0) || (displine >= 2048) ||
        !svga->monitor->target_buffer ||
        !svga->monitor->target_buffer->line[displine])
        return;

    for (int i = 0; i < 16; i++)
        row[i] = svga->vram[(addr + i) & svga->vram_mask];

    for (int x = 0; x < 64; x++) {
        const int group = x >> 3;
        const int within = x & 7;
        const uint16_t bits = ((uint16_t) row[group * 2] << 8) |
                              row[group * 2 + 1];
        const int shift = 14 - (within * 2);
        const int and_bit = (bits >> (shift + 1)) & 1;
        const int xor_bit = (bits >> shift) & 1;
        const int dx = xbase + x;

        if ((dx < 0) || (dx >= 2048))
            continue;

        /* Standard monochrome AND/XOR cursor truth table. */
        if (!and_bit)
            svga->monitor->target_buffer->line[displine][dx] = xor_bit ? col1 : col0;
        else if (xor_bit)
            svga->monitor->target_buffer->line[displine][dx] ^= 0x00ffffff;
    }
}

static void
sis6306_hwcursor_draw(svga_t *svga, int displine)
{
    sis6306_t *dev = (sis6306_t *) svga->priv;
    const uint32_t col0 = sis6306_cursor_color(dev, 0);
    const uint32_t col1 = sis6306_cursor_color(dev, 1);
    const int xbase = svga->hwcursor_latch.x - svga->hwcursor_latch.xoff + svga->x_add;
    const uint32_t base_addr = svga->hwcursor_latch.addr;

    if (!dev->cursor_pattern_valid)
        return;

    if (svga->interlace) {
        const uint32_t addr0 = base_addr;
        const uint32_t addr1 = (base_addr + 16) & svga->vram_mask;

        if (svga->hwcursor_oddeven) {
            sis6306_hwcursor_draw_row(svga, displine, addr1, xbase, col0, col1);
            sis6306_hwcursor_draw_row(svga, displine - 1, addr0, xbase, col0, col1);
        } else {
            sis6306_hwcursor_draw_row(svga, displine, addr0, xbase, col0, col1);
            sis6306_hwcursor_draw_row(svga, displine + 1, addr1, xbase, col0, col1);
        }

        svga->hwcursor_latch.addr = (base_addr + 32) & svga->vram_mask;
    } else {
        sis6306_hwcursor_draw_row(svga, displine, base_addr, xbase, col0, col1);
        svga->hwcursor_latch.addr = (base_addr + 16) & svga->vram_mask;
    }
}

/*
 * The SiS driver uses a true interlaced 1024x768 mode on this monitor (PB A520) but 
 * shows striping on progressive screens because the other parity still contains the
 * previous field.
 *
 * Test this fix which weaves the companion source row into opposite output parity
 * on different monitors.
 */
static void
sis6306_render_packed_base(svga_t *svga)
{
    switch (svga->bpp) {
        case 32:
            svga_render_32bpp_highres(svga);
            break;
        case 24:
            svga_render_24bpp_highres(svga);
            break;
        case 16:
            svga_render_16bpp_highres(svga);
            break;
        case 15:
            svga_render_15bpp_highres(svga);
            break;
        default:
            svga_render_8bpp_highres(svga);
            break;
    }
}

static void
sis6306_render_interlaced_weave(svga_t *svga)
{
    const uint32_t saved_memaddr = svga->memaddr;
    const int      saved_displine = svga->displine;
    const uint32_t row_bytes = (svga->adv_flags & FLAG_NO_SHIFT3) ?
                               svga->rowoffset : (svga->rowoffset << 3);

    sis6306_render_packed_base(svga);

    svga->memaddr = (saved_memaddr + ((saved_displine & 1) ?
                     (0u - row_bytes) : row_bytes)) & svga->vram_display_mask;
    svga->displine = saved_displine ^ 1;
    sis6306_render_packed_base(svga);

    svga->memaddr = saved_memaddr;
    svga->displine = saved_displine;
}

static void
sis6306_recalctimings(svga_t *svga)
{
    sis6306_t *dev = (sis6306_t *) svga->priv;
    const uint8_t mode = dev->ext_seq[0x06];
    const int packed = !!(mode & 0x1e);

    /* SR06.D5 is the SiS interlace enable.  The generic SVGA core handles
       field stepping once this flag is exposed to it. */
    svga->interlace = packed && !!(mode & 0x20);

    if (packed) {
        svga->rowoffset = (uint32_t) svga->crtc[0x13] |
                          ((uint32_t) (dev->ext_seq[0x0a] & 0xf0) << 4);
        if (svga->interlace)
            svga->rowoffset >>= 1;
    }

    /*
     * The Win9x SiS530 driver selects 32-bpp by setting SR09.D7 while
     * leaving SR06 in enhanced-graphics mode (02/82/C2), rather than also
     * asserting SR06.D4.  Treat SR09.D7 as the 32-bpp override whenever a
     * packed graphics mode is active.
     */
    if (packed && (dev->ext_seq[0x09] & 0x80)) {
        svga->bpp    = 32;
        svga->render = svga->interlace ? sis6306_render_interlaced_weave : svga_render_32bpp_highres;
        svga->rowcount = 0;
    } else if (mode & 0x10) {
        svga->bpp    = 24;
        svga->render = svga->interlace ? sis6306_render_interlaced_weave : svga_render_24bpp_highres;
        svga->rowcount = 0;
    } else if (mode & 0x08) {
        svga->bpp       = 16;
        svga->render    = svga->interlace ? sis6306_render_interlaced_weave : svga_render_16bpp_highres;
        svga->rowcount  = 0;
    } else if (mode & 0x04) {
        svga->bpp       = 15;
        svga->render    = svga->interlace ? sis6306_render_interlaced_weave : svga_render_15bpp_highres;
        svga->rowcount  = 0;
    } else if (mode & 0x02) {
        svga->bpp       = 8;
        svga->map8      = svga->pallook;
        svga->render    = svga->interlace ? sis6306_render_interlaced_weave : svga_render_8bpp_highres;
        /* Enhanced packed 8-bpp bypasses the VGA AR12 planar mask. */
        svga->plane_mask = 0x0f;
        svga->rowcount   = 0;
    } else {
        svga->plane_mask = svga->attrregs[0x12] & 0x0f;
    }
}

static void
sis6306_update_graphics_mode(sis6306_t *dev)
{
    const int packed = sis6306_packed_graphics(dev);
    const int old_packed = dev->svga.fb_only;
    const int old_bpp = dev->svga.bpp;

    dev->svga.fb_only       = packed;
    dev->svga.packed_chain4 = packed;

    svga_recalctimings(&dev->svga);

    if (packed && (dev->svga.bpp == 8) && (!old_packed || (old_bpp != 8)))
        dev->svga.dac_mask = 0xff;

    sis6306_update_cursor(dev);
}

static void
sis6306_out(uint16_t addr, uint8_t val, void *priv)
{
    sis6306_t *dev  = (sis6306_t *) priv;
    svga_t    *svga = &dev->svga;
    uint8_t    old;

    if ((((addr & 0xfff0) == 0x03d0) || ((addr & 0xfff0) == 0x03b0)) &&
        !(svga->miscout & 1))
        addr ^= 0x60;

    switch (addr) {
        case 0x03c6:
            svga_out(addr, val, svga);
            return;

        case 0x03cb:
            dev->segment1 = val;
            sis6306_update_banks(dev);
            return;

        case 0x03cd:
            dev->segment0 = val;
            sis6306_update_banks(dev);
            return;

        case 0x03d4:
            /* Keep all eight index bits for the SiS video-extension bank. */
            dev->sis_crtc_index = val;
            svga->crtcreg       = val;
            return;

        case 0x03d5:
            /* CR80 is the video-extension password/identification register. */
            if (dev->sis_crtc_index == 0x80) {
                dev->ext_crtc_unlocked = (val == 0x86);
                return;
            }

            /* CRCB is the second SiS DDC/status path. */
            if (dev->sis_crtc_index == 0xcb) {
                if (dev->ext_crtc_unlocked) {
                    dev->ext_crtc[0xcb] = val & 0x30;
                }
                return;
            }

            if (dev->sis_crtc_index >= 0x20) {
                if (dev->ext_crtc_unlocked) {
                    dev->ext_crtc[dev->sis_crtc_index] = val;
                }
                return;
            }

            /* Standard VGA CRTC programming. */
            if (svga->crtcreg & 0x20)
                return;

            if ((svga->crtcreg < 7) && (svga->crtc[0x11] & 0x80))
                return;

            if ((svga->crtcreg == 7) && (svga->crtc[0x11] & 0x80))
                val = (svga->crtc[7] & ~0x10) | (val & 0x10);

            old = svga->crtc[svga->crtcreg];
            svga->crtc[svga->crtcreg] = val;

            if (old != val) {
                if ((svga->crtcreg < 0x0e) || (svga->crtcreg > 0x10)) {
                    if ((svga->crtcreg == 0x0c) || (svga->crtcreg == 0x0d)) {
                        svga->fullchange = 3;
                        svga->memaddr_latch =
                            ((svga->crtc[0x0c] << 8) | svga->crtc[0x0d]) +
                            ((svga->crtc[8] & 0x60) >> 5);
                    } else {
                        svga->fullchange = changeframecount;
                        svga_recalctimings(svga);
                    }
                }
            }
            return;

        case 0x03c5:
            /* SR05 is the extended-register password/identification register. */
            if (svga->seqaddr == 0x05) {
                dev->ext_seq_unlocked = (val == 0x86);
                return;
            }

            if (svga->seqaddr >= 0x06) {
                if (dev->ext_seq_unlocked) {
                    const uint8_t index = svga->seqaddr;

                    /* SR0D/SR0E are hardware-configuration status registers. */
                    if ((index == 0x0d) || (index == 0x0e)) {
                        return;
                    }

                    const uint8_t old_ext = dev->ext_seq[index];
                    dev->ext_seq[index] = val;

                    /* SR11 bit 0=SCL, bit 1=SDA on the monitor DDC bus. */
                    if (index == 0x11) {
                        if (dev->i2c)
                            i2c_gpio_set(dev->i2c,
                                         !!(val & 0x01), !!(val & 0x02));
                    }

                    if ((index == 0x06) && ((old_ext ^ val) & 0x5e))
                        sis6306_update_graphics_mode(dev);
                    if ((index == 0x09) && ((old_ext ^ val) & 0x80))
                        sis6306_update_graphics_mode(dev);
                    if (((index == 0x0a) || (index == 0x12)) &&
                        (old_ext != val))
                        svga_recalctimings(svga);
                    if ((index == 0x0b) && (old_ext != val)) {
                        if ((old_ext ^ val) & 0x08)
                            sis6306_update_banks(dev);
                    }

                    if (((index >= 0x14) && (index <= 0x1f)) ||
                        (index == 0x23) || (index == 0x38) ||
                        (index == 0x3e))
                        sis6306_update_cursor(dev);
                }
                return;
            }
            break;

        default:
            break;
    }

    svga_out(addr, val, svga);
}

static uint8_t
sis6306_in(uint16_t addr, void *priv)
{
    sis6306_t *dev  = (sis6306_t *) priv;
    svga_t    *svga = &dev->svga;

    if ((((addr & 0xfff0) == 0x03d0) || ((addr & 0xfff0) == 0x03b0)) &&
        !(svga->miscout & 1))
        addr ^= 0x60;

    switch (addr) {
        case 0x03cb: {
            uint8_t ret = (dev->ext_seq[0x0b] & 0x08) ?
                          (dev->segment1 & 0x7f) : 0x00;
            return ret;
        }

        case 0x03cd:
            return dev->segment0;

        case 0x03d4:
            return dev->sis_crtc_index;

        case 0x03d5:
            if (dev->sis_crtc_index == 0x80) {
                uint8_t ret = dev->ext_crtc_unlocked ? 0xa1 : 0x21;
                return ret;
            }

            if (dev->sis_crtc_index == 0xcb) {
                uint8_t ret = dev->ext_crtc_unlocked ?
                              ((dev->ext_crtc[0xcb] & 0x30) | 0x0c) : 0x00;
                return ret;
            }

            if (dev->sis_crtc_index >= 0x20) {
                uint8_t ret = dev->ext_crtc_unlocked ?
                              dev->ext_crtc[dev->sis_crtc_index] : 0x00;
                return ret;
            }

            if (svga->crtcreg & 0x20)
                return 0xff;

            return svga->crtc[svga->crtcreg];

        case 0x03c5:
            if (svga->seqaddr == 0x05) {
                uint8_t ret = dev->ext_seq_unlocked ? 0xa1 : 0x21;
                return ret;
            }

            if (svga->seqaddr >= 0x06) {
                if (!dev->ext_seq_unlocked) {
                    return 0x00;
                }

                /*
                 * IN530 / SiS 530 board straps:
                 *   SR0D D4 = internal AGP bus enabled
                 *   SR0D D0 = shared-framebuffer architecture
                 *   SR0E D6 = integrated VGA enabled
                 *
                 * These are read-only status bits on the real chipset.  The
                 * VGA BIOS uses the shared/local status when programming the
                 * host bridge's shared-memory control.
                 */
                if (svga->seqaddr == 0x0d) {
                    return 0x11;
                }
                if (svga->seqaddr == 0x0e) {
                    return 0x40;
                }

                if (svga->seqaddr == 0x11) {
                    uint8_t ret = dev->ext_seq[0x11] & ~0x03;
                    if (dev->i2c) {
                        if (i2c_gpio_get_scl(dev->i2c))
                            ret |= 0x01;
                        if (i2c_gpio_get_sda(dev->i2c))
                            ret |= 0x02;
                    } else {
                        ret |= 0x03;
                    }
                    return ret;
                }

                return dev->ext_seq[svga->seqaddr];
            }
            break;

        default:
            break;
    }

    return svga_in(addr, svga);
}

static uint8_t
sis6306_iobar_read(uint16_t addr, void *priv)
{
    sis6306_t *dev = (sis6306_t *) priv;
    uint8_t off = (uint8_t) (addr - dev->io_active_base);

    /* BAR2 + 30h..5Fh relocates VGA 3B0h..3DFh. */
    if ((off >= 0x30) && (off <= 0x5f))
        return sis6306_in((uint16_t) (0x0380 + off), dev);

    return dev->io_regs[off & 0x7f];
}

static void
sis6306_iobar_write(uint16_t addr, uint8_t val, void *priv)
{
    sis6306_t *dev = (sis6306_t *) priv;
    uint8_t off = (uint8_t) (addr - dev->io_active_base);

    if ((off >= 0x30) && (off <= 0x5f))
        sis6306_out((uint16_t) (0x0380 + off), val, dev);
    else
        dev->io_regs[off & 0x7f] = val;
}

static void
sis6306_update_mappings(sis6306_t *dev)
{
    io_removehandler(0x03a0, 0x0040,
                     sis6306_in, NULL, NULL,
                     sis6306_out, NULL, NULL,
                     dev);

    if (dev->pci_command & PCI_COMMAND_IO)
        io_sethandler(0x03a0, 0x0040,
                      sis6306_in, NULL, NULL,
                      sis6306_out, NULL, NULL,
                      dev);

    if (dev->pci_command & PCI_COMMAND_MEM)
        mem_mapping_enable(&dev->svga.mapping);
    else
        mem_mapping_disable(&dev->svga.mapping);

    mem_mapping_disable(&dev->lfb_mapping);
    if ((dev->pci_command & PCI_COMMAND_MEM) && dev->lfb_base)
        mem_mapping_set_addr(&dev->lfb_mapping, dev->lfb_base, 8 << 20);

    mem_mapping_disable(&dev->mmio_mapping);
    if ((dev->pci_command & PCI_COMMAND_MEM) && dev->mmio_base)
        mem_mapping_set_addr(&dev->mmio_mapping, dev->mmio_base, 0x10000);

    if (dev->io_active_base) {
        io_removehandler(dev->io_active_base, 0x0080,
                         sis6306_iobar_read, NULL, NULL,
                         sis6306_iobar_write, NULL, NULL, dev);
        dev->io_active_base = 0;
    }
    if ((dev->pci_command & PCI_COMMAND_IO) && dev->io_base &&
        ((dev->io_base & 0xffff0000U) == 0)) {
        dev->io_active_base = (uint16_t) (dev->io_base & 0xff80);
        io_sethandler(dev->io_active_base, 0x0080,
                      sis6306_iobar_read, NULL, NULL,
                      sis6306_iobar_write, NULL, NULL, dev);
    }
}

static inline uint16_t
sis6306_engine_reg16(const sis6306_t *dev, uint32_t off)
{
    return (uint16_t) (dev->mmio_regs[off] |
                       ((uint16_t) dev->mmio_regs[off + 1] << 8));
}

static inline uint32_t
sis6306_engine_reg32(const sis6306_t *dev, uint32_t off)
{
    return (uint32_t) dev->mmio_regs[off] |
           ((uint32_t) dev->mmio_regs[off + 1] << 8) |
           ((uint32_t) dev->mmio_regs[off + 2] << 16) |
           ((uint32_t) dev->mmio_regs[off + 3] << 24);
}

static inline uint8_t
sis6306_engine_rop8(uint8_t rop, uint8_t d, uint8_t p, uint8_t src)
{
    uint8_t out = 0;

    for (int bit = 0; bit < 8; bit++) {
        unsigned idx = (((p >> bit) & 1) << 2) |
                       (((src >> bit) & 1) << 1) |
                       ((d >> bit) & 1);
        if (rop & (1U << idx))
            out |= (uint8_t) (1U << bit);
    }
    return out;
}

static inline unsigned
sis6306_engine_bytespp(const sis6306_t *dev)
{
    switch (dev->svga.bpp) {
        case 8:  return 1;
        case 15:
        case 16: return 2;
        case 24: return 3;
        case 32: return 4;
        default: return 1;
    }
}

static inline uint32_t
sis6306_engine_read_pixel(const sis6306_t *dev, uint32_t addr, unsigned bytespp)
{
    uint32_t v = 0;
    for (unsigned i = 0; i < bytespp; i++) {
        if ((addr + i) >= dev->svga.vram_max)
            break;
        v |= (uint32_t) dev->svga.vram[addr + i] << (i * 8);
    }
    return v;
}

static inline void
sis6306_engine_write_pixel(sis6306_t *dev, uint32_t addr,
                           unsigned bytespp, uint32_t value)
{
    for (unsigned i = 0; i < bytespp; i++) {
        if ((addr + i) >= dev->svga.vram_max)
            break;
        dev->svga.vram[addr + i] = (uint8_t) (value >> (i * 8));
    }
}

static inline uint32_t
sis6306_engine_rop_pixel(uint8_t rop, uint32_t d, uint32_t p,
                         uint32_t src, unsigned bytespp)
{
    uint32_t out = 0;
    for (unsigned i = 0; i < bytespp; i++) {
        uint8_t ob = sis6306_engine_rop8(rop,
                                         (uint8_t) (d >> (i * 8)),
                                         (uint8_t) (p >> (i * 8)),
                                         (uint8_t) (src >> (i * 8)));
        out |= (uint32_t) ob << (i * 8);
    }
    return out;
}

static inline int
sis6306_engine_clip_pixel(const sis6306_t *dev, uint32_t command,
                          int x, int y, uint16_t dst_pitch,
                          uint16_t dst_height, unsigned bytespp)
{
    if (command & (1U << 18)) {
        int16_t left   = (int16_t) sis6306_engine_reg16(dev, 0x8234);
        int16_t top    = (int16_t) sis6306_engine_reg16(dev, 0x8236);
        int16_t right  = (int16_t) sis6306_engine_reg16(dev, 0x8238);
        int16_t bottom = (int16_t) sis6306_engine_reg16(dev, 0x823a);

        if ((x < left) || (x > right) || (y < top) || (y > bottom))
            return 0;
    }

    if (!(command & (1U << 26))) {
        if ((x < 0) || (y < 0))
            return 0;
        if (dst_height && (y >= dst_height))
            return 0;
        if (dst_pitch && bytespp && (x >= (int) (dst_pitch / bytespp)))
            return 0;
    }

    return 1;
}

static inline uint32_t
sis6306_engine_pattern(const sis6306_t *dev, uint32_t command,
                       int x, int y, unsigned bytespp)
{
    uint32_t fg = sis6306_engine_reg32(dev, 0x821c);
    uint32_t bg = sis6306_engine_reg32(dev, 0x8220);

    switch ((command >> 6) & 3) {
        case 0:
            return fg;

        case 1: {
            /* 8x8 color pattern. 8bpp=64 bytes; high-color=128 bytes. */
            uint32_t off = 0x8300 + ((uint32_t) (y & 7) * 8U +
                                     (uint32_t) (x & 7)) * bytespp;
            uint32_t v = 0;
            for (unsigned i = 0; i < bytespp; i++)
                v |= (uint32_t) dev->mmio_regs[(off + i) & 0xffff] << (i * 8);
            return v;
        }

        case 2: {
            uint8_t mask = dev->mmio_regs[0x822c + (y & 7)];
            return (mask & (0x80U >> (x & 7))) ? fg : bg;
        }

        default:
            return fg;
    }
}

static void
sis6306_engine_mark_changed(sis6306_t *dev, uint32_t lo, uint32_t hi)
{
    if ((lo == 0xffffffffU) || (lo >= dev->svga.vram_max))
        return;
    if (hi >= dev->svga.vram_max)
        hi = dev->svga.vram_max - 1;

    for (uint32_t page = lo >> 12; page <= (hi >> 12); page++)
        dev->svga.changedvram[page] = dev->svga.monitor->mon_changeframecount;

    dev->svga.fullchange = dev->svga.monitor->mon_changeframecount;
}

static uint64_t
sis6306_engine_bitblt(sis6306_t *dev, uint32_t command)
{
    svga_t *svga = &dev->svga;
    const unsigned bytespp = sis6306_engine_bytespp(dev);
    const uint32_t src_base = sis6306_engine_reg32(dev, 0x8200) & 0x007fffffU;
    const uint16_t src_pitch = sis6306_engine_reg16(dev, 0x8204) & 0x1fff;
    const int src_y = sis6306_engine_reg16(dev, 0x8208) & 0x0fff;
    const int src_x = sis6306_engine_reg16(dev, 0x820a) & 0x0fff;
    const int dst_y = sis6306_engine_reg16(dev, 0x820c) & 0x0fff;
    const int dst_x = sis6306_engine_reg16(dev, 0x820e) & 0x0fff;
    const uint32_t dst_base = sis6306_engine_reg32(dev, 0x8210) & 0x007fffffU;
    const uint16_t dst_pitch = sis6306_engine_reg16(dev, 0x8214) & 0x1fff;
    const uint16_t dst_height = sis6306_engine_reg16(dev, 0x8216) & 0x1fff;
    const uint16_t width = sis6306_engine_reg16(dev, 0x8218) & 0x1fff;
    const uint16_t height = sis6306_engine_reg16(dev, 0x821a) & 0x1fff;
    const int xstep = (command & (1U << 16)) ? 1 : -1;
    const int ystep = (command & (1U << 17)) ? 1 : -1;
    const uint8_t rop = (command >> 8) & 0xff;
    const unsigned srcsel = (command >> 4) & 3;
    uint32_t changed_lo = 0xffffffffU, changed_hi = 0;
    uint64_t pixels = 0;

    if (!width || !height || !dst_pitch || !bytespp)
        return 0;

    for (uint32_t yy = 0; yy < height; yy++) {
        int dy = dst_y + ystep * (int) yy;
        int sy = src_y + ystep * (int) yy;

        for (uint32_t xx = 0; xx < width; xx++) {
            int dx = dst_x + xstep * (int) xx;
            int sx = src_x + xstep * (int) xx;
            uint32_t da, src = 0, pat, dst, out;

            if (!sis6306_engine_clip_pixel(dev, command, dx, dy,
                                           dst_pitch, dst_height, bytespp))
                continue;
            if ((dx < 0) || (dy < 0))
                continue;

            da = dst_base + (uint32_t) dy * dst_pitch +
                 (uint32_t) dx * bytespp;
            if ((da + bytespp) > svga->vram_max)
                continue;

            if ((srcsel <= 1) && (sx >= 0) && (sy >= 0) && src_pitch) {
                uint32_t sa = src_base + (uint32_t) sy * src_pitch +
                              (uint32_t) sx * bytespp;
                if ((sa + bytespp) <= svga->vram_max)
                    src = sis6306_engine_read_pixel(dev, sa, bytespp);
            }

            pat = sis6306_engine_pattern(dev, command, dx, dy, bytespp);
            dst = sis6306_engine_read_pixel(dev, da, bytespp);
            out = sis6306_engine_rop_pixel(rop, dst, pat, src, bytespp);
            sis6306_engine_write_pixel(dev, da, bytespp, out);

            if (da < changed_lo) changed_lo = da;
            if ((da + bytespp - 1) > changed_hi) changed_hi = da + bytespp - 1;
            pixels++;
        }
    }

    sis6306_engine_mark_changed(dev, changed_lo, changed_hi);
    return pixels;
}

static uint64_t
sis6306_engine_color_expand(sis6306_t *dev, uint32_t command, int enhanced)
{
    svga_t *svga = &dev->svga;
    const unsigned bytespp = sis6306_engine_bytespp(dev);
    const uint32_t src_base = sis6306_engine_reg32(dev, 0x8200) & 0x007fffffU;
    const uint16_t src_pitch = sis6306_engine_reg16(dev, 0x8204) & 0x1fff;
    const int src_y = sis6306_engine_reg16(dev, 0x8208) & 0x0fff;
    const int src_x = sis6306_engine_reg16(dev, 0x820a) & 0x0fff;
    const int dst_y = sis6306_engine_reg16(dev, 0x820c) & 0x0fff;
    const int dst_x = sis6306_engine_reg16(dev, 0x820e) & 0x0fff;
    const uint32_t dst_base = sis6306_engine_reg32(dev, 0x8210) & 0x007fffffU;
    const uint16_t dst_pitch = sis6306_engine_reg16(dev, 0x8214) & 0x1fff;
    const uint16_t dst_height = sis6306_engine_reg16(dev, 0x8216) & 0x1fff;
    const uint16_t width = sis6306_engine_reg16(dev, 0x8218) & 0x1fff;
    const uint16_t height = sis6306_engine_reg16(dev, 0x821a) & 0x1fff;
    const int xstep = (command & (1U << 16)) ? 1 : -1;
    const int ystep = (command & (1U << 17)) ? 1 : -1;
    const uint8_t rop = (command >> 8) & 0xff;
    const uint32_t fg = sis6306_engine_reg32(dev, 0x8224);
    const uint32_t bg = sis6306_engine_reg32(dev, 0x8228);
    const int transparent = !!(command & (1U << 20));
    uint32_t changed_lo = 0xffffffffU, changed_hi = 0;
    uint64_t pixels = 0;

    if (!width || !height || !dst_pitch || !bytespp)
        return 0;

    for (uint32_t yy = 0; yy < height; yy++) {
        int dy = dst_y + ystep * (int) yy;
        int sy = src_y + ystep * (int) yy;

        for (uint32_t xx = 0; xx < width; xx++) {
            int dx = dst_x + xstep * (int) xx;
            int sx = src_x + xstep * (int) xx;
            int mono = 0;
            uint32_t da, src, pat, dst, out;

            if (enhanced) {
                if ((sx >= 0) && (sy >= 0) && src_pitch) {
                    uint32_t sa = src_base + (uint32_t) sy * src_pitch +
                                  ((uint32_t) sx >> 3);
                    if (sa < svga->vram_max)
                        mono = !!(svga->vram[sa] & (0x80U >> (sx & 7)));
                }
            } else {
                uint32_t bit = yy * width + xx;
                uint32_t byte = bit >> 3;
                if (byte < 0x180)
                    mono = !!(dev->mmio_regs[0x8300 + byte] &
                              (0x80U >> (bit & 7)));
            }

            if (transparent && !mono)
                continue;
            if (!sis6306_engine_clip_pixel(dev, command, dx, dy,
                                           dst_pitch, dst_height, bytespp))
                continue;
            if ((dx < 0) || (dy < 0))
                continue;

            da = dst_base + (uint32_t) dy * dst_pitch +
                 (uint32_t) dx * bytespp;
            if ((da + bytespp) > svga->vram_max)
                continue;

            src = mono ? fg : bg;
            pat = sis6306_engine_pattern(dev, command, dx, dy, bytespp);
            dst = sis6306_engine_read_pixel(dev, da, bytespp);
            out = sis6306_engine_rop_pixel(rop, dst, pat, src, bytespp);
            sis6306_engine_write_pixel(dev, da, bytespp, out);
            if (da < changed_lo) changed_lo = da;
            if ((da + bytespp - 1) > changed_hi) changed_hi = da + bytespp - 1;
            pixels++;
        }
    }

    sis6306_engine_mark_changed(dev, changed_lo, changed_hi);
    return pixels;
}

static uint64_t
sis6306_engine_multi_scanline(sis6306_t *dev, uint32_t command)
{
    svga_t *svga = &dev->svga;
    const unsigned bytespp = sis6306_engine_bytespp(dev);
    const unsigned count = sis6306_engine_reg16(dev, 0x8208) & 0x7f;
    const int ystart = sis6306_engine_reg16(dev, 0x820a) & 0x0fff;
    const uint32_t dst_base = sis6306_engine_reg32(dev, 0x8210) & 0x007fffffU;
    const uint16_t dst_pitch = sis6306_engine_reg16(dev, 0x8214) & 0x1fff;
    const uint16_t dst_height = sis6306_engine_reg16(dev, 0x8216) & 0x1fff;
    const uint8_t rop = (command >> 8) & 0xff;
    const unsigned ydir = (command >> 16) & 3;
    const int omit_last = !!(command & (1U << 21));
    uint32_t points_base;
    uint32_t changed_lo = 0xffffffffU, changed_hi = 0;
    uint64_t pixels = 0;

    if (!count || !dst_pitch || !bytespp)
        return 0;

    if (bytespp == 1)
        points_base = 0x8340;
    else if (bytespp == 2)
        points_base = 0x8380;
    else
        points_base = 0x8400;


    for (unsigned line = 0; line < count; line++) {
        uint16_t xs, xe;
        int y;

        if (line == 0) {
            xs = sis6306_engine_reg16(dev, 0x820c) & 0x0fff;
            xe = sis6306_engine_reg16(dev, 0x820e) & 0x0fff;
        } else if (line == 1) {
            xs = sis6306_engine_reg16(dev, 0x8244) & 0x0fff;
            xe = sis6306_engine_reg16(dev, 0x8246) & 0x0fff;
        } else {
            uint32_t poff = points_base + (line - 2) * 4;
            uint32_t point;
            if ((poff + 3) >= 0x10000)
                break;
            point = sis6306_engine_reg32(dev, poff);
            xs = point & 0x0fff;
            xe = (point >> 16) & 0x0fff;
        }

        if (ydir == 0)
            y = ystart - (int) line;
        else if (ydir == 2)
            y = ystart + (int) line;
        else
            y = ystart;

        int xfirst = xs;
        int xlast = xe;
        int xstep = (xlast >= xfirst) ? 1 : -1;
        if (omit_last && (xlast != xfirst))
            xlast -= xstep;

        for (int x = xfirst;; x += xstep) {
            uint32_t da, pat, dst, out;

            if (sis6306_engine_clip_pixel(dev, command, x, y,
                                          dst_pitch, dst_height, bytespp) &&
                (x >= 0) && (y >= 0)) {
                da = dst_base + (uint32_t) y * dst_pitch +
                     (uint32_t) x * bytespp;
                if ((da + bytespp) <= svga->vram_max) {
                    pat = sis6306_engine_pattern(dev, command, x, y, bytespp);
                    dst = sis6306_engine_read_pixel(dev, da, bytespp);
                    out = sis6306_engine_rop_pixel(rop, dst, pat, 0, bytespp);
                    sis6306_engine_write_pixel(dev, da, bytespp, out);
                    if (da < changed_lo) changed_lo = da;
                    if ((da + bytespp - 1) > changed_hi) changed_hi = da + bytespp - 1;
                    pixels++;
                }
            }

            if (x == xlast)
                break;
        }
    }

    sis6306_engine_mark_changed(dev, changed_lo, changed_hi);
    return pixels;
}


static void
sis6306_engine_execute(sis6306_t *dev, uint32_t command)
{
    const unsigned type = command & 0x0f;

    if ((dev->svga.bpp == 8) || (dev->svga.bpp == 15) ||
        (dev->svga.bpp == 16) || (dev->svga.bpp == 24) ||
        (dev->svga.bpp == 32)) {
        switch (type) {
            case 0x0:
                sis6306_engine_bitblt(dev, command);
                break;
            case 0x1:
                sis6306_engine_color_expand(dev, command, 0);
                break;
            case 0x2:
                sis6306_engine_color_expand(dev, command, 1);
                break;
            case 0x3:
                sis6306_engine_multi_scanline(dev, command);
                break;
            default:
                break;
        }
    }
}

static uint8_t
sis6306_mmio_readb_raw(uint32_t addr, sis6306_t *dev)
{
    uint32_t offset = addr & 0xffff;

    switch (offset) {
        case 0x8240: return 0xff;
        case 0x8241: return 0x1f;
        case 0x8242: return 0x00;
        case 0x8243: return 0xe0;
        default:     return dev->mmio_regs[offset];
    }
}

static uint8_t
sis6306_mmio_readb(uint32_t addr, void *priv)
{
    sis6306_t *dev = (sis6306_t *) priv;
    uint8_t ret = sis6306_mmio_readb_raw(addr, dev);
    return ret;
}

static uint16_t
sis6306_mmio_readw(uint32_t addr, void *priv)
{
    sis6306_t *dev = (sis6306_t *) priv;
    uint16_t ret = sis6306_mmio_readb_raw(addr, dev) |
                   ((uint16_t) sis6306_mmio_readb_raw(addr + 1, dev) << 8);
    return ret;
}

static uint32_t
sis6306_mmio_readl(uint32_t addr, void *priv)
{
    sis6306_t *dev = (sis6306_t *) priv;
    uint32_t ret = sis6306_mmio_readb_raw(addr, dev) |
                   ((uint32_t) sis6306_mmio_readb_raw(addr + 1, dev) << 8) |
                   ((uint32_t) sis6306_mmio_readb_raw(addr + 2, dev) << 16) |
                   ((uint32_t) sis6306_mmio_readb_raw(addr + 3, dev) << 24);
    return ret;
}

static void
sis6306_update_cursor_mmio(sis6306_t *dev)
{
    uint32_t xraw, yraw;
    int x, y;

    if (!(dev->ext_seq[0x06] & 0x40) || !(dev->ext_seq[0x1b] & 0x80))
        return;

    xraw = (uint32_t) dev->mmio_regs[0x85f8] |
           ((uint32_t) dev->mmio_regs[0x85f9] << 8) |
           ((uint32_t) dev->mmio_regs[0x85fa] << 16) |
           ((uint32_t) dev->mmio_regs[0x85fb] << 24);
    yraw = (uint32_t) dev->mmio_regs[0x85fc] |
           ((uint32_t) dev->mmio_regs[0x85fd] << 8) |
           ((uint32_t) dev->mmio_regs[0x85fe] << 16) |
           ((uint32_t) dev->mmio_regs[0x85ff] << 24);

    x = (int) (xraw & 0x0fff);
    y = (int) (yraw & 0x0fff);

    dev->svga.hwcursor.x = x;
    dev->svga.hwcursor.y = dev->svga.interlace ? (y << 1) : y;
    dev->svga.fullchange = dev->svga.monitor->mon_changeframecount;
}

static void
sis6306_cursor_mmio_after_write(sis6306_t *dev, uint32_t addr)
{
    const uint32_t off = addr & 0xffff;

    if (((off >= 0x85f8) && (off <= 0x85ff)))
        sis6306_update_cursor_mmio(dev);
}

static void
sis6306_mmio_writeb_raw(uint32_t addr, uint8_t val, sis6306_t *dev)
{
    uint32_t offset = addr & 0xffff;

    if ((offset >= 0x8240) && (offset <= 0x8243))
        return;
    dev->mmio_regs[offset] = val;
}

static void
sis6306_mmio_writeb(uint32_t addr, uint8_t val, void *priv)
{
    sis6306_t *dev = (sis6306_t *) priv;
    sis6306_mmio_writeb_raw(addr, val, dev);
    sis6306_cursor_mmio_after_write(dev, addr);
}

static void
sis6306_mmio_writew(uint32_t addr, uint16_t val, void *priv)
{
    sis6306_t *dev = (sis6306_t *) priv;
    sis6306_mmio_writeb_raw(addr, val, dev);
    sis6306_mmio_writeb_raw(addr + 1, val >> 8, dev);
    sis6306_cursor_mmio_after_write(dev, addr);
}

static void
sis6306_mmio_writel(uint32_t addr, uint32_t val, void *priv)
{
    sis6306_t *dev = (sis6306_t *) priv;
    sis6306_mmio_writeb_raw(addr, val, dev);
    sis6306_mmio_writeb_raw(addr + 1, val >> 8, dev);
    sis6306_mmio_writeb_raw(addr + 2, val >> 16, dev);
    sis6306_mmio_writeb_raw(addr + 3, val >> 24, dev);
    sis6306_cursor_mmio_after_write(dev, addr);

    if ((addr & 0xffff) == 0x823c)
        sis6306_engine_execute(dev, val);
}

static int
sis6306_range_overlap(uint32_t a, uint32_t n, uint32_t b, uint32_t m)
{
    const uint64_t ae = (uint64_t) a + n;
    const uint64_t be = (uint64_t) b + m;
    return ((uint64_t) a < be) && ((uint64_t) b < ae);
}

static void
sis6306_cursor_track_lfb_write(sis6306_t *dev, uint32_t addr, unsigned width)
{
    const uint32_t mem_size = sis6306_configured_vram_size(dev);
    const uint32_t off = addr & dev->svga.vram_mask;
    const uint32_t tail = (mem_size >= 0x4000U) ? (mem_size - 0x4000U) : 0;

    if (!width || (mem_size < 0x4000U))
        return;

    if (sis6306_range_overlap(off, width, tail, 0x4000U)) {
        dev->cursor_pattern_valid = 1;
        dev->svga.fullchange = dev->svga.monitor->mon_changeframecount;
    }
}

static uint8_t
sis6306_lfb_readb(uint32_t addr, void *priv)
{
    sis6306_t *dev = (sis6306_t *) priv;
    return svga_readb_linear(addr, &dev->svga);
}

static uint16_t
sis6306_lfb_readw(uint32_t addr, void *priv)
{
    sis6306_t *dev = (sis6306_t *) priv;
    return svga_readw_linear(addr, &dev->svga);
}

static uint32_t
sis6306_lfb_readl(uint32_t addr, void *priv)
{
    sis6306_t *dev = (sis6306_t *) priv;
    return svga_readl_linear(addr, &dev->svga);
}

static void
sis6306_lfb_writeb(uint32_t addr, uint8_t val, void *priv)
{
    sis6306_t *dev = (sis6306_t *) priv;
    svga_writeb_linear(addr, val, &dev->svga);
    sis6306_cursor_track_lfb_write(dev, addr, 1);
}

static void
sis6306_lfb_writew(uint32_t addr, uint16_t val, void *priv)
{
    sis6306_t *dev = (sis6306_t *) priv;
    svga_writew_linear(addr, val, &dev->svga);
    sis6306_cursor_track_lfb_write(dev, addr, 2);
}

static void
sis6306_lfb_writel(uint32_t addr, uint32_t val, void *priv)
{
    sis6306_t *dev = (sis6306_t *) priv;
    svga_writel_linear(addr, val, &dev->svga);
    sis6306_cursor_track_lfb_write(dev, addr, 4);
}

static uint8_t
sis6306_pci_read(int func, int addr, int len, void *priv)
{
    sis6306_t *dev = (sis6306_t *) priv;
    (void) func;
    (void) len;

    switch (addr) {
        case 0x00: return 0x39;
        case 0x01: return 0x10;
        case 0x02: return 0x06;
        case 0x03: return 0x63;

        /* Datasheet reset dword at 04h: 02200004h. */
        case 0x04: return (dev->pci_command & 0x03) | PCI_COMMAND_L_BM;
        case 0x05: return 0x00;
        case 0x06: return 0x20;
        case 0x07: return 0x02;

        /* Physical IN530 / SiS 6306 revision. */
        case 0x08: return 0x2a;
        case 0x09: return 0x00;
        case 0x0a: return 0x00;
        case 0x0b: return 0x03;

        case 0x0c: return 0x00;
        case 0x0d: return 0x00;
        case 0x0e: return 0x00;
        case 0x0f: return 0x00;

        /* BAR0: observed 8 MB prefetchable framebuffer aperture. */
        case 0x10: return (dev->lfb_base & 0xff) | 0x08;
        case 0x11: return (dev->lfb_base >> 8) & 0xff;
        case 0x12: return (dev->lfb_base >> 16) & 0xff;
        case 0x13: return (dev->lfb_base >> 24) & 0xff;

        /* BAR1: 64 KB non-prefetchable MMIO. */
        case 0x14: return dev->mmio_base & 0xff;
        case 0x15: return (dev->mmio_base >> 8) & 0xff;
        case 0x16: return (dev->mmio_base >> 16) & 0xff;
        case 0x17: return (dev->mmio_base >> 24) & 0xff;

        /* BAR2: 128-byte relocatable VGA I/O aperture. */
        case 0x18: return (dev->io_base & 0xff) | 0x01;
        case 0x19: return (dev->io_base >> 8) & 0xff;
        case 0x1a: return (dev->io_base >> 16) & 0xff;
        case 0x1b: return (dev->io_base >> 24) & 0xff;

        /* SiS subsystem identity used by the reference Windows driver. */
        case 0x2c: return 0x39;
        case 0x2d: return 0x10;
        case 0x2e: return 0x06;
        case 0x2f: return 0x63;

        case 0x30: return 0x00;
        case 0x31: return 0x80;
        case 0x32: return 0xff;
        case 0x33: return 0xff;

        /* PCI Power Management capability, followed by AGP. */
        case 0x34: return 0x40;

        case 0x3c: return dev->irq_line;
        case 0x3d: return 0x01;
        case 0x3e: return 0x00;
        case 0x3f: return 0x00;

        /* PCI Power Management capability, version 1. */
        case 0x40: return 0x01;
        case 0x41: return 0x50;
        case 0x42: return 0x01;
        case 0x43: return 0x00;
        case 0x44:
        case 0x45:
        case 0x46:
        case 0x47:
            return 0x00;

        /* AGP 1.0 capability block. */
        case 0x50: return 0x02;
        case 0x51: return 0x00;
        case 0x52: return 0x10;
        case 0x53: return 0x00;
        case 0x54: return 0x03;
        case 0x55: return 0x00;
        case 0x56: return 0x00;
        case 0x57: return 0x01;
        case 0x58:
        case 0x59:
        case 0x5a:
        case 0x5b:
            return 0x00;

        default:
            return 0x00;
    }
}

static void
sis6306_pci_write(int func, int addr, int len, uint8_t val, void *priv)
{
    sis6306_t *dev = (sis6306_t *) priv;
    (void) func;
    (void) len;

    switch (addr) {
        case 0x04:
            dev->pci_command = (val & (PCI_COMMAND_IO | PCI_COMMAND_MEM)) |
                               PCI_COMMAND_L_BM;
            sis6306_update_mappings(dev);
            break;

        case 0x10:
        case 0x11:
        case 0x12:
        case 0x13: {
            unsigned shift = (addr - 0x10) * 8;
            uint32_t raw = dev->lfb_base;
            raw = (raw & ~(0xffU << shift)) | ((uint32_t) val << shift);
            dev->lfb_base = raw & 0xff800000U;
            sis6306_update_mappings(dev);
            break;
        }

        case 0x14:
        case 0x15:
        case 0x16:
        case 0x17: {
            unsigned shift = (addr - 0x14) * 8;
            uint32_t raw = dev->mmio_base;
            raw = (raw & ~(0xffU << shift)) | ((uint32_t) val << shift);
            dev->mmio_base = raw & 0xffff0000U;
            sis6306_update_mappings(dev);
            break;
        }

        case 0x18:
        case 0x19:
        case 0x1a:
        case 0x1b: {
            unsigned shift = (addr - 0x18) * 8;
            uint32_t raw = dev->io_base;
            raw = (raw & ~(0xffU << shift)) | ((uint32_t) val << shift);
            dev->io_base = raw & 0xffffff80U;
            sis6306_update_mappings(dev);
            break;
        }

        case 0x3c:
            dev->irq_line = val;
            break;

        default:
            break;
    }
}

static void *
sis6306_init(const device_t *info)
{
    sis6306_t *dev = (sis6306_t *) calloc(1, sizeof(sis6306_t));

    video_inform(VIDEO_FLAG_TYPE_SPECIAL, &timing_sis6306);

    /*
     * The physical 6306 PCI resource exposes an 8 MB LFB aperture.  The BIOS
     * still selects its shared-memory size through the SiS host/SEQ registers;
     * keep the full aperture while using the configured size for accesses
     * that depend on the amount of installed framebuffer memory.
     */
    svga_init(info, &dev->svga, dev, 8 << 20,
              sis6306_recalctimings,
              sis6306_in, sis6306_out,
              sis6306_hwcursor_draw, NULL);

    /*
     * Replace only the legacy-memory callbacks.  Standard VGA modes still
     * delegate to the generic SVGA handlers; enhanced mode gets the SiS
     * packed A0000h bank window implemented above.
     */
    mem_mapping_set_handler(&dev->svga.mapping,
                            sis6306_legacy_readb,
                            sis6306_legacy_readw,
                            sis6306_legacy_readl,
                            sis6306_legacy_writeb,
                            sis6306_legacy_writew,
                            sis6306_legacy_writel);
    mem_mapping_set_p(&dev->svga.mapping, dev);

    dev->svga.bpp     = 8;
    dev->svga.miscout = 1;

    dev->pci_command = PCI_COMMAND_L_BM;

    dev->i2c = i2c_gpio_init("ddc_sis6306");
    if (dev->i2c) {
        dev->ddc = ddc_init(i2c_gpio_get_bus(dev->i2c));
        dev->ext_seq[0x11] = 0x03;
        i2c_gpio_set(dev->i2c, 1, 1);
    }

    mem_mapping_add(&dev->lfb_mapping, 0, 0,
                    sis6306_lfb_readb, sis6306_lfb_readw, sis6306_lfb_readl,
                    sis6306_lfb_writeb, sis6306_lfb_writew, sis6306_lfb_writel,
                    NULL, MEM_MAPPING_EXTERNAL, dev);

    mem_mapping_add(&dev->mmio_mapping, 0, 0,
                    sis6306_mmio_readb, sis6306_mmio_readw, sis6306_mmio_readl,
                    sis6306_mmio_writeb, sis6306_mmio_writew, sis6306_mmio_writel,
                    NULL, MEM_MAPPING_EXTERNAL, dev);

    mem_mapping_disable(&dev->svga.mapping);
    mem_mapping_disable(&dev->lfb_mapping);
    mem_mapping_disable(&dev->mmio_mapping);

    pci_add_card(PCI_ADD_AGP,
                 sis6306_pci_read, sis6306_pci_write,
                 dev, &dev->pci_slot);

    sis6306_update_mappings(dev);

    return dev;
}

static void
sis6306_close(void *priv)
{
    sis6306_t *dev = (sis6306_t *) priv;

    io_removehandler(0x03a0, 0x0040,
                     sis6306_in, NULL, NULL,
                     sis6306_out, NULL, NULL,
                     dev);
    if (dev->io_active_base)
        io_removehandler(dev->io_active_base, 0x0080,
                         sis6306_iobar_read, NULL, NULL,
                         sis6306_iobar_write, NULL, NULL, dev);

    if (dev->ddc)
        ddc_close(dev->ddc);
    if (dev->i2c)
        i2c_gpio_close(dev->i2c);

    svga_close(&dev->svga);
    free(dev);
}

static void
sis6306_speed_changed(void *priv)
{
    sis6306_t *dev = (sis6306_t *) priv;
    svga_recalctimings(&dev->svga);
}

static void
sis6306_force_redraw(void *priv)
{
    sis6306_t *dev = (sis6306_t *) priv;
    dev->svga.fullchange = dev->svga.monitor->mon_changeframecount;
}

const device_t sis6306_onboard_device = {
    .name          = "SiS 530 (6306) Onboard AGP",
    .internal_name = "sis6306_onboard",
    .flags         = DEVICE_AGP,
    .local         = 1,
    .init          = sis6306_init,
    .close         = sis6306_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = sis6306_speed_changed,
    .force_redraw  = sis6306_force_redraw,
    .config        = NULL
};
