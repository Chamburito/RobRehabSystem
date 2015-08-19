//===========================================================================
//
// NAME:    pdl_if.h
//
// DESCRIPTION:
//
//          PowerDAQ Linux driver firmware interface definition
//
// AUTHOR: Frederic Villeneuve 
//         Alex Ivchenko
//
//---------------------------------------------------------------------------
//      Copyright (C) 2000,2004 United Electronic Industries, Inc.
//      All rights reserved.
//---------------------------------------------------------------------------
// For more informations on using and distributing this software, please see
// the accompanying "LICENSE" file.
//

#ifndef __PDL_IF_H__
#define __PDL_IF_H__

// pdl_fwi.c
u32 pd_dsp_get_status(int board);
u32 pd_dsp_get_flags(int board);
void pd_dsp_set_flags(int board, u32 new_flags);
void pd_dsp_command(int board, int command);
void pd_dsp_cmd_no_ret(int board, u16 command);
void pd_dsp_write(int board, u32 data);
u32 pd_dsp_read(int board);
u32 pd_dsp_cmd_ret_ack(int board, u16 wCmd);
u32 pd_dsp_cmd_ret_value(int board, u16 wCmd);
u32 pd_dsp_read_ack(int board);
u32 pd_dsp_write_ack(int board, u32 dwValue);
u32 pd_dsp_cmd_write_ack(int board, u16 wCmd, u32 dwValue);
u32 pd_dsp_int_status(int board);
u32 pd_dsp_acknowledge_interrupt(int board);
int pd_dsp_startup(int board);
void  pd_init_calibration(int board);
int pd_reset_dsp(int board);
int pd_download_firmware_bootstrap(int board);
int pd_reset_board(int board);
int pd_download_firmware(int board);
int pd_echo_test(int board);
int pd_dsp_reg_write(int board, u32 reg, u32 data);
int pd_dsp_reg_read(int board, u32 reg, u32* data);

// pdl_ain.c
int pd_ain_set_config(int, u32, u32, u32);
int pd_ain_set_cv_clock(int board, u32 clock_divisor);
int pd_ain_set_cl_clock(int board, u32 clock_divisor);
int pd_ain_set_channel_list(int board, u32 num_entries, u32 list[]);
int pd_ain_set_events (int board, u32 dwEvents);
int pd_ain_get_status(int board, u32* status);
int pd_ain_sw_start_trigger(int board);
int pd_ain_sw_stop_trigger(int board);
int pd_ain_set_enable_conversion(int board, int enable);
int pd_ain_get_value(int board, u16* value);
int pd_ain_set_ssh_gain(int board, u32 dwCfg);
int pd_ain_get_samples(int board, int max_samples, u16* buffer, u32* samples);
int pd_ain_reset(int board);
int pd_ain_sw_cl_start(int board);
int pd_ain_sw_cv_start(int board);
int pd_ain_reset_cl(int board);
int pd_ain_clear_data(int board);
int pd_ain_flush_fifo(int board, int from_isr);
int pd_ain_get_xfer_samples(int board, u16* buffer);
int pd_ain_set_xfer_size(int board, u32 size);
int pd_ain_get_BM_ctr(int board, u32 dwCh, u32* pBurstCtr, u32* pFrameCtr);
int pd_ain_set_BM_ctr(int board, u32 dwCh, u32* pdwChList);

// pdl_aio.c
int pd_register_daq_buffer(int board, u32 SubSystem,
                           u32 ScanSize, u32 FrameSize, u32 NumFrames,
                           u16* databuf, int bWrap);
int pd_unregister_daq_buffer(int board, u32 SubSystem);
int pd_clear_daq_buffer(int board, int subsystem);
int pd_get_daq_buffer_size(int board, int subSystem);
int pd_ain_async_init(int board, tAsyncCfg* pAInCfg);
int pd_ain_async_term(int board);
int pd_ain_async_start(int board);
int pd_ain_async_stop(int board);
int pd_ain_async_retrieve(int board);
int pd_ain_get_scans(int board, tScanInfo* pScanInfo);
int pd_aout_async_init(int board, tAsyncCfg* pAOutCfg);
int pd_aout_async_term(int board);
int pd_aout_async_start(int board);
int pd_aout_async_stop(int board);
int pd_aout_get_scans(int board, tScanInfo* pScanInfo);
int pd_get_buf_status(int board, int subsystem, PTBuf_Info pDaqBuf);
int pd_aout_dmaSet(int board, u32 offset, u32 count, u32 source);
int pd_aout_put_xbuf(int board, u32 NumToCopy, u32 *UserBufCopied);


// pdl_ao.c
int pd_aout_set_config(int board, u32 config, u32 posttrig);
int pd_aout_set_cv_clk(int board, u32 dwClkDiv);
int pd_aout_set_events(int board, u32 dwEvents);
int pd_aout_get_status(int board, u32* dwStatus);
int pd_aout_set_enable_conversion(int board, u32 dwEnable);
int pd_aout_sw_start_trigger(int board);
int pd_aout_sw_stop_trigger(int board);
int pd_aout_sw_cv_start(int board);
int pd_aout_clear_data(int board);
int pd_aout_reset(int board);
int pd_aout_put_value(int board, u32 dwValue);
int pd_aout_put_block(int board, u32 dwNumValues, u32* pdwBuf, u32* pdwCount);


// pdl_dio.c
int pd_din_set_config(int board, u32 config);
int pd_din_set_event(int board, u32 events);
int pd_din_clear_events(int board, u32 events);
int pd_din_read_inputs(int board, u32 *pdwValue);
int pd_din_clear_data(int board);
int pd_din_reset(int board);
int pd_din_status(int board, u32 *pdwValue);
int pd_dout_write_outputs(int board, u32 val);
int pd_dio256_write_output(int board, u32 cmd, u32 val);
int pd_dio256_read_input(int board, u32 cmd, u32* val);
int pd_dio_dmaSet(int board, u32 offset, u32 count, u32 source);
int pd_dio256_setIntrMask(int board);
int pd_dio256_getIntrData(int board);
int pd_dio256_intrEnable(int board, u32 enable);
int pd_dio256_make_reg_mask(u32 bank, u32 regMask);
u32 pd_dio256_make_cmd(u32 dwRegister);
int pd_dio256_read_all(int board, u32* pdata);
int pd_dio256_write_all(int board, u32* pdata);


// pdl_dao.c
int pd_ao96_writex(int board, u32 dwDACNum, u16 wDACValue, int dwHold, int dwUpdAll);
int pd_ao32_writex(int board, u32 dwDACNum, u16 wDACValue, int dwHold, int dwUpdAll);
int pd_ao96_reset(int board);
int pd_ao32_reset(int board);
int pd_ao96_write(int board, u16 wChannel, u16 wValue);
int pd_ao32_write(int board, u16 wChannel, u16 wValue);
int pd_ao96_write_hold(int board, u16 wChannel, u16 wValue);
int pd_ao32_write_hold(int board, u16 wChannel, u16 wValue);
int pd_ao96_update(int board);
int pd_ao32_update(int board);
int pd_ao96_set_update_channel(int board, u16 wChannel, u32 Mode);
int pd_ao32_set_update_channel(int board, u16 wChannel, u32 bEnable);
int pd_dio_reset(int board);
int pd_dio_enable_output(int board, u32 dwRegMask);
int pd_dio_latch_all(int board, u32 dwRegister);
int pd_dio_simple_read(int board, u32 dwRegister, u32 *pdwValue);
int pd_dio_read(int board, u32 dwRegister, u32 *pdwValue);
int pd_dio_write(int board, u32 dwRegister, u32 dwValue);
int pd_dio_prop_enable(int board, u32 dwRegMask);
int pd_dio_latch_enable(int board, u32 dwRegister, u32 bEnable);

// pdl_uct.c
int pd_uct_set_config(int board, u32 config);
int pd_uct_set_event(int board, u32 events);
int pd_uct_clear_event(int board, u32 events);
int pd_uct_get_status(int board, u32* status);
int pd_uct_write(int board, u32 value);
int pd_uct_read(int board, u32 config, u32* value);
int pd_uct_set_sw_gate(int board, u32 gate_level);
int pd_uct_sw_strobe(int board);
int pd_uct_reset(int board);

// pdl_event.c
int pd_enable_events(int board, tEvents* pEvents);
int pd_disable_events(int board, tEvents* pEvents);
int pd_set_user_events(int board, u32 subsystem, u32 events);
int pd_clear_user_events(int board, u32 subsystem, u32 events);
int pd_get_user_events(int board, u32 subsystem, u32* events);
int pd_immediate_update(int board);
void pd_debug_show_events (tEvents *Event, char* msg);

// pdl_brd.c
int pd_adapter_enable_interrupt(int board, u32 val);
int pd_adapter_acknowledge_interrupt(int board);
int pd_adapter_get_board_status(int board, tEvents* pEvent);
int pd_adapter_set_board_event1(int board, u32 dwEvents);
int pd_adapter_set_board_event2(int board, u32 dwEvents);
int pd_adapter_eeprom_read(int board, u32 dwMaxSize, u16*pwReadBuf);
int pd_adapter_eeprom_write(int board, u32 dwBufSize, u16* pwWriteBuf);
int pd_cal_dac_write(int board, u32 dwCalDACValue);
int pd_adapter_test_interrupt(int board);

// pdl_int.c
void pd_stop_and_disable_ain(int board);
void pd_process_pd_ain_get_samples(int board, int bFHFState);
void pd_process_ain_move_samples(int board, u32 page, u32 numready);
void pd_stop_and_disable_aout(int board);
void pd_process_pd_aout_put_samples(int board, int bFHFState);
void pd_process_driver_events(int board, tEvents* pEvents);
int pd_notify_user_events(int board, tEvents* pNewFwEvents);
void pd_process_events(int board);

// pdl_init.c
void  pd_init_pd_board(int board);
int pd_enum_devices(int board, char* board_name, int board_name_size, 
                    char *serial_number, int serial_number_size);

// pdl_dspuct.c
int pd_dspct_load(int board,
                  u32 dwCounter, 
                  u32 dwLoad, 
                  u32 dwCompare, 
                  u32 dwMode, 
                  u32 dwReload, 
                  u32 dwInverted,
                  u32 dwUsePrescaler);
int pd_dspct_enable_counter(int board, u32 dwCounter, u32 dwEnable);
int pd_dspct_enable_interrupts(int board, u32 dwCounter, u32 dwCompare, u32 dwOverflow);
int pd_dspct_get_count(int board, u32 dwCounter, u32 *dwCount);
int pd_dspct_get_compare(int board, u32 dwCounter, u32 *dwCompare);
int pd_dspct_get_status(int board, u32 dwCounter, u32 *dwStatus);
int pd_dspct_set_compare(int board, u32 dwCounter, u32 dwCompare);
int pd_dspct_set_load(int board, u32 dwCounter, u32 dwLoad);
int pd_dspct_set_status(int board, u32 dwCounter, u32 dwStatus);
int pd_dsp_PS_load(int board, u32 dwLoad, u32 dwSource);
int pd_dsp_PS_get_count(int board, u32 *dwCount);
u32 pd_dspct_get_count_addr(u32 dwCounter);
u32 pd_dspct_get_load_addr(u32 dwCounter);
u32 pd_dspct_get_status_addr(u32 dwCounter);
u32 pd_dspct_get_compare_addr(u32 dwCounter);

// powerdaq.c
int pd_register_user_isr(int board, TUser_isr user_isr, void* user_param);
int pd_unregister_user_isr(int board);

int pd_notify_event(int board, PD_SUBSYSTEM ss, int event);
int pd_sleep_on_event(int board, PD_SUBSYSTEM, int event, int timeoutms);
TSynchSS* pd_get_synch(int board, PD_SUBSYSTEM ss, int event);

int pd_alloc_contig_memory(int board, tAllocContigMem *allocMemory); 
void pd_dealloc_contig_memory(int board, tAllocContigMem *pDeallocMemory);

unsigned int pd_readl(void *address);
void pd_writel(unsigned int value, void *address);

pd_board_t* pd_get_board_object(int board);
char* pd_get_board_name(int board);
void pd_clean_device(int board);


#endif // _PDL_IF_H

