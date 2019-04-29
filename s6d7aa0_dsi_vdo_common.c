//Copyright © Rubén Espínola

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif
#include "lcm_drv.h"
#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
	#include <string.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
        #include <mt-plat/mt_gpio.h>
#endif

/* Local Constants */
#define FRAME_WIDTH  (720)
#define FRAME_HEIGHT (1280)

/* Local Variables */
#define SET_RESET_PIN(v) (lcm_util.set_reset_pin((v)))
#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

/**
 * REGFLAG_DELAY, used to trigger MDELAY,
 * REGFLAG_END_OF_TABLE, used to mark the end of LCM_setting_table.
 * their values dosen't matter until they,
 * match with any LCM_setting_table->cmd.
 */
#define REGFLAG_DELAY (0xFE)
#define REGFLAG_END_OF_TABLE (0XFF)

/* Local Functions */
#define dsi_set_cmdq_V3(para_tbl,size,force_update)         lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define read_reg_v2(cmd, buffer, buffer_size)	            lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#define write_regs(addr, pdata, byte_nums)	                lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)   lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define wrtie_cmd(cmd)	lcm_util.dsi_write_cmd(cmd)

/* LCM Driver Implementations */

static LCM_UTIL_FUNCS lcm_util = { 0 };

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] =
{
    {0xf0, 2, {0x02,0x5a,0x5a,0x00}},
    {0xf1, 2, {0x5a,0x5a}},
    {0xfc, 2, {0x5a,0x5a}},
    {0xca, 4, {0x00,0x54,0x05,0x28}},
    {0xb1, 6, {0x25,0x40,0xe8,0x10,0x00,0x22}},
	{0xb2, 1, {0x21}},
    {0xe3, 1, {0x26}},
    {0xf2, 4, {0x0c,0x10,0x0e,0x0d}},
    {0xf5, 18, {0x6e,0x78,0x10,0x6a,0x27,0x1d,0x4c,0x4c,0x03,0x03,0x04,0x22,0x11,0x31,0x50,0x2a,0x16,0x75}},
    {0xf6, 7, {0x04,0x8c,0x0f,0x80,0x46,0x00,0x00}},
	{0xf7, 22, {0x04,0x14,0x16,0x18,0x1a,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x01,0x24,0x25,0x0c}},
	{0xf8, 22, {0x05,0x15,0x17,0x19,0x1b,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x01,0x24,0x25,0x0d}},
	{0xed, 11, {0xc0,0x58,0x38,0x58,0x38,0x58,0x38,0x04,0x04,0x11,0x22}},
	{0xee, 24, {0x67,0x89,0x45,0x23,0x55,0x55,0x55,0x55,0x43,0x32,0x87,0xa9,0x33,0x33,0x33,0x33,0x89,0x01,0x00,0x00,0x01,0x89,0x00,0x00}},
	{0xef, 32, {0x3d,0x09,0x00,0x40,0x06,0x67,0x45,0x23,0x01,0x00,0x00,0x00,0x00,0x04,0x80,0x80,0x84,0x07,0x09,0x89,0x08,0x12,0x21,0x21,0x03,0x03,0x44,0x33,0x00,0x00,0x00,0x00}},
	{0xfa, 17, {0x10,0x30,0x15,0x1a,0x11,0x15,0x1c,0x1a,0x1e,0x28,0x2c,0x2b,0x2b,0x2a,0x2a,0x20,0x2e}},
	{0xfb, 17, {0x10,0x30,0x15,0x1a,0x11,0x15,0x1c,0x1a,0x1e,0x28,0x2c,0x2b,0x2b,0x2a,0x2a,0x20,0x2e}},
	{0xc5, 1, {0x21}},
	{0xfe, 1, {0x48}},
	{0xc8, 2, {0x24,0x53}},
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY,120,{}},
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 20,{}},
	{REGFLAG_END_OF_TABLE, 0x00,{}},
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = 
{
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 20, {}},
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY,120, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}},
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    int i;
    for(i = 0; i < count; i++)
    {
        switch (table[i].cmd) {
        case REGFLAG_DELAY :
            MDELAY(table[i].count);
            break;
        case REGFLAG_END_OF_TABLE :
            break;
        default:
            dsi_set_cmdq_V2(table[i].cmd, table[i].count, table[i].para_list, force_update);
        }
    }
}

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
    memset(params, 0, sizeof(LCM_PARAMS));

    params->dpi.dsc_params.initial_offset = 4;
    params->dpi.dsc_params.final_offset = 8;
    params->dpi.msb_io_driving_current = 48;
    params->dsi.horizontal_bllp = 215;
    params->width = 720;
    params->type = 2;
    params->dpi.dsc_params.scale_value = 2;
    params->dpi.dsc_params.line_bpg_offset = 2;
    params->dsi.mode = 720;
    params->height = 1280;
    params->dpi.dsc_params.rc_mode1_size = 1280;
    params->dpi.dsc_params.slice_mode = 3;
    params->dpi.dsc_params.rct_on = 3;
    params->dpi.dsc_params.flatness_minqp = 16;
    params->dpi.io_driving_current = 16;
    params->dpi.lsb_io_driving_current = 16;

}

static void lcm_init(void)
{

    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(20);
    SET_RESET_PIN(1);
    MDELAY(50);
    SET_RESET_PIN(1);
    MDELAY(20);
    SET_RESET_PIN(1);
    MDELAY(120);

    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);

}

static unsigned int lcm_compare_id(void)
{
    return 1;
}

static void lcm_suspend(void)
{
    push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);

    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(20);
    SET_RESET_PIN(1);
    MDELAY(120);

}

/* Get LCM Driver Hooks */
LCM_DRIVER s6d7aa0_dsi_vdo_common_lcm_drv =
{
    .name           = "s6d7aa0_dsi_vdo_common",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_init,
    .compare_id     = lcm_compare_id,
};
