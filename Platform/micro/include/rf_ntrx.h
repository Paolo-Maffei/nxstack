/*
    NanoStack: MCU software and PC tools for IP-based wireless sensor networking.
		
    Copyright (C) 2006-2007 Sensinode Ltd.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

		Address:
		Sensinode Ltd.
		Teknologiantie 6	
		90570 Oulu, Finland

		E-mail:
		info@sensinode.com
*/


/**
 *
 * \file rf_ntrx.h
 * \brief micro RF driver header for the Nanotron NA5TR1 radio
 *
 *  Micro: RF control function headers.
 *   
 *	
 */


#ifndef RF_NTRX_H
#define RF_NTRX_H
/** Address decoder mode value */

extern portCHAR rf_init(void);
extern portCHAR rf_write(buffer_t *buffer);
extern portCHAR rf_rx_enable(void);
extern portCHAR rf_rx_disable(void);
extern portCHAR rf_mac_get(sockaddr_t *address);
extern int8_t rf_power_set(uint8_t new_power);
extern int8_t rf_channel_set(uint8_t channel);
extern void rf_send_ack(uint8_t pending);
extern int8_t rf_analyze_rssi(void);
extern portCHAR rf_cca_check(uint8_t backoff_count, uint8_t slotted);
extern portCHAR rf_write_no_cca(buffer_t *buffer);

//extern portCHAR rf_address_decoder_mode(uint8_t param);
extern void rf_set_address(sockaddr_t *address);

typedef enum {
	TRX_STATE_IDLE,
	TRX_STATE_WAIT,
	TRX_STATE_SEND,
	TRX_STATE_READ,
	TRX_STATE_RANGING,
	TRX_STATE_DATA
} TRX_STATE;	


#define NTRX_DEFAULT_CHANNEL 1

#define TRUE 0x01
#define FALSE 0x00

#define TRX_BANDWIDTH_80MHz 0
#define TRX_BANDWIDTH_22MHz 1

#define TRX_SD_500ns 	0x00
#define TRX_SD_1us	 	0x01
#define TRX_SD_2us		0x02
#define TRX_SD_4us		0x03
#define TRX_SD_8us		0x04
#define TRX_SD_16us		0x05

#define TRX_500k_S	0x00
#define TRX_1M_S		0x01
#define TRX_2M_S		0x02
#define TRX_31k25_S	0x04
#define TRX_62k5_S	0x05
#define TRX_125k_S	0x06
#define TRX_250k_S	0x07


#define TRX_Version_O			0x01
#define TRX_Version_MSB		0x07
#define TRX_Version_LSB		0x00
#define TRX_Version_I			0x05

#define TRX_Revision_O		0x02
#define TRX_Revision_MSB	0x07
#define TRX_Revision_LSB	0x00
#define TRX_Revision_I		0x01

#define TRX_RamIndex_O	0x0E

#define TRX_RXCmdStop 0x56


#define NA_TypeCodeData_VC_C (0x0)



#define NA_SpiBitOrder_B                              (0)
#define NA_SpiTxDriver_B                              (1)
#define NA_Version_LSB                                (0)
#define NA_WakeUpTimeByte_LSB                         (0)
#define NA_Revision_LSB                               (0)
#define NA_WakeUpTimeWe_LSB                           (1)
#define NA_BattMgmtEnable_B                           (7)
#define NA_BattMgmtThreshold_LSB                      (1)
#define NA_BattMgmtCompare_B                          (0)
#define NA_DioDirection_B                             (0)
#define NA_DioOutValueAlarmEnable_B                   (1)
#define NA_DioAlarmStart_B                            (2)
#define NA_DioAlarmPolarity_B                         (3)
#define NA_DioUsePullup_B                             (4)
#define NA_DioUsePulldown_B                           (5)
#define NA_DioInValueAlarmStatus_LSB                  (0)
#define NA_DioPortWe_LSB                              (0)
#define NA_EnableWakeUpRtc_B                          (0)
#define NA_EnableWakeUpDio_B                          (1)
#define NA_PowerUpTime_LSB                            (4)
#define NA_PowerDownMode_B                            (7)
#define NA_PowerDown_B                                (0)
#define NA_ResetBbClockGate_B                         (1)
#define NA_ResetBbRadioCtrl_B                         (2)
#define NA_UsePullup4Test_B                           (6)
#define NA_UsePulldown4Test_B                         (7)
#define NA_EnableBbCrystal_B                          (0)
#define NA_EnableBbClock_B                            (1)
#define NA_BypassBbCrystal_B                          (3)
#define NA_FeatureClockFreq_LSB                       (4)
#define NA_EnableFeatureClock_B                       (7)
#define NA_UsePullup4Spiclk_B                         (0)
#define NA_UsePulldown4Spiclk_B                       (1)
#define NA_UsePullup4Spissn_B                         (2)
#define NA_UsePulldown4Spissn_B                       (3)
#define NA_UsePullup4Spirxd_B                         (4)
#define NA_UsePulldown4Spirxd_B                       (5)
#define NA_UsePullup4Spitxd_B                         (6)
#define NA_UsePulldown4Spitxd_B                       (7)
#define NA_UsePullup4Por_B                            (0)
#define NA_UsePulldown4Por_B                          (1)
#define NA_UsePullup4Pamp_B                           (2)
#define NA_UsePulldown4Pamp_B                         (3)
#define NA_UsePullup4Ucirq_B                          (4)
#define NA_UsePulldown4Ucirq_B                        (5)
#define NA_UsePullup4Ucrst_B                          (6)
#define NA_UsePulldown4Ucrst_B                        (7)
#define NA_WritePulls4Spi_B                           (0)
#define NA_WritePulls4Pads_B                          (1)
#define NA_TestModes_LSB                              (0)
#define NA_RfTestSelect_LSB                           (0)
#define NA_RamIndex_LSB                               (0)
#define NA_DeviceSelect_LSB                           (4)
#define NA_TxIrqEnable_B                              (0)
#define NA_RxIrqEnable_B                              (1)
#define NA_BbTimerIrqEnable_B                         (2)
#define NA_LoIrqEnable_B                              (3)
#define NA_TxIrqStatus_B                              (4)
#define NA_RxIrqStatus_B                              (5)
#define NA_BbTimerIrqStatus_B                         (6)
#define NA_LoIrqStatus_B                              (7)
#define NA_WakeUpTime_LSB                             (8)
#define NA_TxIntsRawStat_LSB                          (0)
#define NA_TxIntsReset_LSB                            (0)
#define NA_TxTimeSlotEnd_B                            (5)
#define NA_TxTimeSlotTOut_B                           (4)
#define NA_TxUnderrun_B                               (3)
#define NA_TxEnd_B                                    (2)
#define NA_TxBufferRdy_LSB                            (0)
#define NA_RxIntsRawStat_LSB                          (0)
#define NA_RxIntsReset_LSB                            (0)
#define NA_RxTimeSlotEnd_B                            (6)
#define NA_RxTimeSlotTOut_B                           (5)
#define NA_RxOverflow_B                               (4)
#define NA_RxHeaderEnd_B                              (3)
#define NA_RxEnd_B                                    (2)
#define NA_RxBufferRdy_LSB                            (0)
#define NA_LoIntsRawStat_LSB                          (0)
#define NA_LoIntsReset_LSB                            (0)
#define NA_LoTuningReady_B                            (1)
#define NA_LoTuningNeeded_B                           (0)
#define NA_TxIntsEn_LSB                               (0)
#define NA_RxIntsEn_LSB                               (0)
#define NA_LoIntsEn_LSB                               (0)
#define NA_LoRxCapsValue_LSB                          (0)
#define NA_LoTxCapsValue_LSB                          (0)
#define NA_LoEnableFastTuning_B                       (0)
#define NA_LoFastTuningLevel_LSB                      (1)
#define NA_LoEnableLsbNeg_B                           (4)
#define NA_LoEnableRecalib_B                          (5)
#define NA_UseLoRxCaps_B                              (7)
#define NA_LoTargetValue_LSB                          (0)
#define NA_AgcThresHold1_LSB                          (0)
#define NA_AgcThresHold2_LSB                          (0)
#define NA_HoldAgcInBitSync_LSB                       (0)
#define NA_HoldAgcInFrameSync_B                       (7)
#define NA_AgcDeadTime_LSB                            (0)
#define NA_AgcNregLength_LSB                          (6)
#define NA_AgcIntTime_LSB                             (0)
#define NA_AgcValue_LSB                               (0)
#define NA_AgcDefaultEn_B                             (6)
#define NA_AgcHold_B                                  (7)
#define NA_AgcRssiThres_LSB                           (0)
#define NA_AgcGain_LSB                                (0)
#define NA_AgcEnable_B                                (7)
#define NA_FctClockEn_B                               (4)
#define NA_StartFctMeasure_B                          (5)
#define NA_EnableTx_B                                 (7)
#define NA_FctPeriod_LSB                              (0)
#define NA_BasebandTimerStartValue_LSB                (0)
#define NA_SyncWord_LSB                               (0)
#define NA_ToaOffsetMeanAck_LSB                       (0)
#define NA_ToaOffsetMeanAckValid_B                    (7)
#define NA_TxRespTime_LSB                             (0)
#define NA_PhaseOffsetAck_LSB                         (0)
#define NA_PhaseOffsetData_LSB                        (4)
#define NA_ToaOffsetMeanData_LSB                      (0)
#define NA_ToaOffsetMeanDataValid_B                   (7)
#define NA_RxPacketType_LSB                           (0)
#define NA_RxAddrMatch_LSB                            (4)
#define NA_RxCrc1Stat_B                               (6)
#define NA_RxCrc2Stat_B                               (7)
#define NA_RxCorrBitErr_LSB                           (0)
#define NA_RxCorrErrThres_LSB                         (4)
#define NA_RxAddrSegEsMatch_B                         (0)
#define NA_RxAddrSegIsMatch_B                         (1)
#define NA_RxCryptEn_B                                (4)
#define NA_RxCryptId_LSB                              (5)
#define NA_RxCryptSeqN_B                              (7)
#define NA_RxFec1BitErr_LSB                           (0)
#define NA_TxTimeSlotStart_LSB                        (0)
#define NA_TxTimeSlotEnd_LSB                          (0)
#define NA_TxTimeSlotControl_B                        (0)
#define NA_RxTimeSlotControl_B                        (1)
#define NA_RxPacketSlot_LSB                           (0)
#define NA_RxTimeSlotStart_LSB                        (0)
#define NA_RxTimeSlotEnd_LSB                          (0)
#define NA_TxArqCnt_LSB                               (0)
#define NA_TxArqMax_LSB                               (4)
#define NA_CsqUsePhaseShift_B                         (2)
#define NA_CsqMemAddrInit_B                           (5)
#define NA_CsqUseRam_B                                (6)
#define NA_D3lFixnMap_B                               (0)
#define NA_D3lPomEn_B                                 (1)
#define NA_D3lPomLen_LSB                              (2)
#define NA_D3lUpDownEx_B                              (7)
#define NA_LeaveMapThresh1InBitsync_LSB               (0)
#define NA_UseMapThresh1InFramesync_B                 (7)
#define NA_Go2MapThresh1InBitsync_LSB                 (0)
#define NA_D3lFixThres1MapEn_B                        (7)
#define NA_EnableLO_B                                 (0)
#define NA_EnableLOdiv10_B                            (1)
#define NA_EnableCsqClock_B                           (2)
#define NA_EnableExtPA_B                              (4)
#define NA_EnableIntPA_B                              (5)
#define NA_EnableRxClock_B                            (6)
#define NA_EnableRx_B                                 (7)
#define NA_LnaFreqAdjust_LSB                          (0)
#define NA_TxPaBias_LSB                               (4)
#define NA_TxOutputPower0_LSB                         (0)
#define NA_TxOutputPower1_LSB                         (0)
#define NA_RfRxCompValueI_LSB                         (0)
#define NA_RfRxCompValueQ_LSB                         (0)
#define NA_SymbolDur_LSB                              (0)
#define NA_SymbolRate_LSB                             (4)
#define NA_ModulationSystem_B                         (7)
#define NA_UseFec_B                                   (2)
#define NA_TxRxCryptCrc2Mode_B                        (3)
#define NA_TxRxCryptClkMode_LSB                       (4)
#define NA_SwapBbBuffers_B                            (0)
#define NA_TxRxBbBufferMode1_B                        (1)
#define NA_TxRxBbBufferMode0_B                        (2)
#define NA_FdmaEnable_B                               (4)
#define NA_TxRxMode_B                                 (7)
#define NA_TxPreTrailMatrix0_LSB                      (0)
#define NA_TxPreTrailMatrix1_LSB                      (2)
#define NA_TxUnderrunIgnore_B                         (4)
#define NA_TxMacCifsDis_B                             (7)
#define NA_TxVCarrSens_B                              (0)
#define NA_TxPhCarrSenseMode_LSB                      (1)
#define NA_TxVCarrSensAck_B                           (3)
#define NA_TxArq_B                                    (4)
#define NA_Tx3Way_B                                   (5)
#define NA_TxBackOffAlg_B                             (6)
#define NA_TxFragPrio_B                               (7)
#define NA_TxBackOffSeed_LSB                          (0)
#define NA_TxCryptSeqReset_LSB                        (0)
#define NA_TxCryptEn_B                                (4)
#define NA_TxCryptId_LSB                              (5)
#define NA_TxCryptSeqN_B                              (7)
#define NA_TxScrambInit_LSB                           (0)
#define NA_TxScrambEn_B                               (7)
#define NA_TxTransBytes_LSB                           (0)
#define NA_TxPacketType_LSB                           (0)
#define NA_TxAddrSlct_B                               (7)
#define NA_TxCmdStop_B                                (0)
#define NA_TxCmdStart_B                               (1)
#define NA_TxBufferCmd_LSB                            (2)
#define NA_RxCmdStop_B                                (0)
#define NA_RxCmdStart_B                               (1)
#define NA_RxBufferCmd_LSB                            (2)
#define NA_RxCryptSeqReset_LSB                        (0)
#define NA_RxTransBytes_LSB                           (0)
#define NA_RxTimeBCrc1Mode_B                          (0)
#define NA_RxCrc2Mode_B                               (1)
#define NA_RxCrc2Mode_O                               (0x5A)
#define NA_RxArqMode_LSB                              (2)
#define NA_RxAddrSegEsMode_B                          (4)
#define NA_RxAddrSegIsMode_B                          (5)
#define NA_RxAddrSegDevIdL_LSB                        (6)
#define NA_RxDataEn_B                                 (0)
#define NA_RxBrdcastEn_B                              (1)
#define NA_RxTimeBEn_B                                (2)
#define NA_RxAddrMode_B                               (3)
#define NA_RangingPulses_LSB                          (4)
#define NA_PulseDetDelay_LSB                          (0)
#define NA_GateAdjThreshold_LSB                       (0)
#define NA_DownPulseDetectDis_B                       (4)
#define NA_UpPulseDetectDis_B                         (5)
#define NA_GateSizeUnsync_LSB                         (0)
#define NA_GateSizeBitsync_LSB                        (2)
#define NA_GateSizeFramesync_LSB                      (4)
#define NA_GateAdjBitsyncEn_B                         (6)
#define NA_GateAdjFramesyncEn_B                       (7)
#define NA_Go2BitsyncThreshold_LSB                    (0)
#define NA_LeaveBitsyncThreshold_LSB                  (4)
#define NA_RtcTimeBTxAdj_LSB                          (0)
#define NA_RtcTimeBRxAdj_LSB                          (0)
#define NA_RtcCmdWr_B                                 (0)
#define NA_RtcCmdRd_B                                 (1)
#define NA_RtcTimeBAutoMode_B                         (4)
#define NA_RtcTimeBTestMode_B                         (7)
#define NA_AgcAmplitude_LSB                           (0)
#define NA_AgcRangeOffset_LSB                         (0)
#define NA_UseAlternativeAgc_B                        (7)
#define NA_TxRxDigTestMode_B                          (0)
#define NA_DebugMacRxCmd_B                            (2)
#define NA_DebugMacTxCmd_B                            (3)
#define NA_BistBbRamReset_B                           (4)
#define NA_BistBbRamActive_B                          (5)
#define NA_BistCsqRamReset_B                          (6)
#define NA_BistCsqRamActive_B                         (7)
#define NA_DebugMacFsm_LSB                            (0)
#define NA_DebugMacTxRxCtrl_LSB                       (4)
#define NA_DebugBitProcFsm_LSB                        (0)
#define NA_DebugBitProcTx_B                           (4)
#define NA_DebugBitProcRx_B                           (5)
#define NA_DebugRxDetStatus_LSB                       (6)
#define NA_RamStaAddr0_LSB                            (0)
#define NA_RamStaAddr1_LSB                            (0)
#define NA_RamTxDstAddr_LSB                           (0)
#define NA_RamTxLength_LSB                            (0)
#define NA_RamTxFragC_B                               (5)
#define NA_RamTxSeqN_B                                (6)
#define NA_RamTxLCh_B                                 (7)
#define NA_RamRxDstAddr_LSB                           (0)
#define NA_RamRxSrcAddr_LSB                           (0)
#define NA_RamRxLength_LSB                            (0)
#define NA_RamRxFragC_B                               (5)
#define NA_RamRxSeqN_B                                (6)
#define NA_RamRxLCh_B                                 (7)
#define NA_RamRtcTx_LSB                               (0)
#define NA_RamRtcRx_LSB                               (0)
#define NA_RamRtcReg_LSB                              (0)
#define NA_RamTxRxCryptKey_LSB                        (0)
#define NA_RamTxCryptClock_LSB                        (0)
#define NA_RamRxCryptClock_LSB                        (0)
#define NA_RamRxBuffer_LSB                            (0)
#define NA_RamTxBuffer_LSB                            (0)
#define NA_RamTxRxBuffer_LSB                          (0)
#define NA_RamRxTransBuffer_LSB                       (0)
#define NA_RamTxTransBuffer_LSB                       (0)
#define NA_RamTxRxTransBuffer_LSB                     (0)
#define NA_RamCsqDataByte0_LSB                        (0)
#define NA_RamCsqDataByte1_LSB                        (0)
#define NA_RamCsqDataByte2_LSB                        (0)
#define NA_RamD3lPatI_O                               (0x80)
#define NA_RamD3lPatI_LSB                             (0)
#define NA_RamD3lPatQ_O                               (0x180)
#define NA_RamD3lPatQ_LSB                             (0)
#define NA_RamD3lThresDown_LSB                        (0)
#define NA_RamD3lThresUp_LSB                          (0)
#define NA_TxRxPacketLength_LSB                       (0)
#define NA_IrqPolarity_B                              (2)
#define NA_IrqDriver_B                                (3)
#define NA_ClearBasebandTimerInt_B                    (7)
#define NA_ClearBasebandTimerInt_B                    (7)
#define NA_ChirpFilterCaps_LSB                        (0)
#define NA_CsqDitherValue_LSB                         (0)
#define NA_CsqUse4Phases_B                            (3)
#define NA_CsqAsyMode_B                               (4)
#define NA_CsqTest_B                                  (7)
#define NA_CsqSetValue_LSB                            (0)
#define NA_CsqSetIValue_B                             (6)
#define NA_CsqSetQValue_B                             (7)
#define NA_InvertRxClock_B                            (3)
#define NA_Crc2Type_LSB                               (0)
#define NA_ChirpMatrix0_LSB                           (0)
#define NA_ChirpMatrix1_LSB                           (4)
#define NA_ChirpMatrix2_LSB                           (0)
#define NA_ChirpMatrix3_LSB                           (4)
#define NA_RamTxBuffer_O                              (0x380)
#define NA_RamTxLength_O                              (0x98)
#define NA_RamTxDstAddr_O                             (0x90)
#define NA_TxBufferCmd_O                              (0x55)
#define NA_TxBufferCmd_MSB                            (3)
#define NA_RxCrc2Stat_O                               (0x31)
#define NA_RamRxDstAddr_O                             (0xA8)
#define NA_RamRxSrcAddr_O                             (0xB0)
#define NA_RamRxLength_O                              (0xB8)
#define NA_RamRxBuffer_O                              (0x280)
#define NA_RxCmdStart_O                               (0x56)
#define NA_RxBufferCmd_MSB                            (3)





typedef enum
{
// 0x00 = 0x,
	TRX_REG_SpiBitOrder = 0x00,
	TRX_REG_SpiTxDriver = 0x00,
	TRX_REG_IrqPolarity = 0x00,
	TRX_REG_IrqDriver = 0x00,
//0x01 = 0x,
	TRX_REG_WakeUpTimeByte = 0x01,
//0x02 = 0x,
	TRX_REG_WakeUpTimeWe = 0x02,
//0x03 = 0x,
	TRX_REG_BattMgmtEnable = 0x03,
	TRX_REG_BattMgmtThreshold = 0x03,
	TRX_REG_BattMgmtCompare = 0x03,
//0x04 = 0x,
	TRX_REG_DioDirection = 0x04,
	TRX_REG_DioOutValueAlarmEnable = 0x04,
	TRX_REG_DioAlarmStart = 0x04,
	TRX_REG_DioAlarmPolarity = 0x04,
	TRX_REG_DioUsePullup = 0x04,
	TRX_REG_DioUsePulldown = 0x04,
//0x05 = 0x,
	TRX_REG_DioPortWe = 0x05,
//0x06 = 0x,
	TRX_REG_EnableWakeUpRtc = 0x06,
	TRX_REG_EnableWakeUpDio = 0x06,
	TRX_REG_PowerUpTime = 0x06,
	TRX_REG_PowerDownMode = 0x06,
//0x07 = 0x,
	TRX_REG_PowerDown = 0x07,
	TRX_REG_ResetBbClockGate = 0x07,
	TRX_REG_ResetBbRadioCtrl = 0x07,
//0x08 = 0x,
	TRX_REG_EnableBbCrystal = 0x08,
	TRX_REG_EnableBbClock = 0x08,
	TRX_REG_BypassBbCrystal = 0x08,
	TRX_REG_FeatureClockFreq = 0x08,
	TRX_REG_EnableFeatureClock = 0x08,
//0x09 = 0x,
	TRX_REG_UsePullup4Spiclk = 0x09,
	TRX_REG_UsePulldown4Spiclk = 0x09,
	TRX_REG_UsePullup4Spissn = 0x09,
	TRX_REG_UsePulldown4Spissn = 0x09,
	TRX_REG_UsePullup4Spirxd = 0x09,
	TRX_REG_UsePulldown4Spirxd = 0x09,
	TRX_REG_UsePullup4Spitxd = 0x09,
	TRX_REG_UsePulldown4Spitxd = 0x09,
//0x0a = 0x,
	TRX_REG_UsePullup4Por = 0xa,
	TRX_REG_UsePulldown4Por = 0xa,
	TRX_REG_UsePullup4Pamp = 0xa,
	TRX_REG_UsePulldown4Pamp = 0xa,
	TRX_REG_UsePullup4Ucirq = 0xa,
	TRX_REG_UsePulldown4Ucirq = 0xa,
	TRX_REG_UsePullup4Ucrst = 0xa,
	TRX_REG_UsePulldown4Ucrst = 0xa,
//0x0b = 0x,
	TRX_REG_WritePulls4Spi = 0xb,
	TRX_REG_WritePulls4Pads = 0xb,
//0x0c = 0x,
	TRX_REG_TestModes = 0xc,
//0x0d = 0x,
	TRX_REG_RfTestSelect = 0xd,
//0x0e = 0x,
	TRX_REG_RamIndex = 0xe,
	TRX_REG_DeviceSelect = 0xe,
//0x0f = 0x,
	TRX_REG_TxIrqEnable = 0xf,
	TRX_REG_RxIrqEnable = 0xf,
	TRX_REG_BbTimerIrqEnable = 0xf,
	TRX_REG_LoIrqEnable = 0xf,
//0x10 = 0x,
	TRX_REG_TxIntsReset = 0x10,
//0x11 = 0x,
	TRX_REG_RxIntsReset = 0x11,
//0x12 = 0x,
	TRX_REG_LoIntsReset = 0x12,
	TRX_REG_ClearBasebandTimerInt = 0x12,
//0x13 = 0x,
	TRX_REG_TxIntsEn = 0x13,
//0x14 = 0x,
	TRX_REG_RxIntsEn = 0x14,
//0x15 = 0x,
	TRX_REG_LoIntsEn = 0x15,
//0x1c = 0x,
	TRX_REG_LoEnableFastTuning = 0x1c,
	TRX_REG_LoFastTuningLevel = 0x1c,
	TRX_REG_LoEnableLsbNeg = 0x1c,
	TRX_REG_LoEnableRecalib = 0x1c,
	TRX_REG_UseLoRxCaps = 0x1c,
//0x1d = 0x,
	TRX_REG_LoTargetValue = 0x1d,
//0x1f = 0x,
	TRX_REG_AgcThresHold1 = 0x1f,
//0x20 = 0x,
	TRX_REG_AgcThresHold2 = 0x20,
//0x21 = 0x,
	TRX_REG_HoldAgcInBitSync = 0x21,
	TRX_REG_HoldAgcInFrameSync = 0x21,
//0x22 = 0x,
	TRX_REG_AgcDeadTime = 0x22,
	TRX_REG_AgcNregLength = 0x22,
//0x23 = 0x,
	TRX_REG_AgcIntTime = 0x23,
//0x25 = 0x25,
	TRX_REG_AgcValue = 0x25,
	TRX_REG_AgcDefaultEn = 0x25,
	TRX_REG_AgcHold = 0x25,
//0x26 = 0x,
	TRX_REG_AgcRssiThres = 0x26,
	TRX_REG_AgcEnable = 0x26,
//0x27 = 0x,
	TRX_REG_ChirpFilterCaps = 0x27,
	TRX_REG_FctClockEn = 0x27,
	TRX_REG_StartFctMeasure = 0x27,
	TRX_REG_EnableTx = 0x27,
	TRX_REG_FctPeriod = 0x27,
//0x2b	 = 0x,
//0x2e = 0x,
//0x30 = 0x,
//0x31 = 0x,
//0x32 = 0x,
//0x33 = 0x,
//0x37 = 0x,
	TRX_REG_TxTimeSlotControl = 0x37,
//0x3c = 0x,
	TRX_REG_TxArqMax = 0x3c,
//0x3d = 0x,
	TRX_REG_CsqDitherValue = 0x3d,
	TRX_REG_CsqUsePhaseShift = 0x3d,
	TRX_REG_CsqUse4Phases = 0x3d,
	TRX_REG_CsqAsyMode = 0x3d,
	TRX_REG_CsqMemAddrInit = 0x3d,
	TRX_REG_CsqUseRam = 0x3d,
	TRX_REG_CsqTest = 0x3d,
//0x3e = 0x,
	TRX_REG_CsqSetValue = 0x3e,
	TRX_REG_CsqSetIValue = 0x3e,
	TRX_REG_CsqSetQValue = 0x3e,
//0x3f = 0x,
	TRX_REG_D3lFixnMap = 0x3f,
	TRX_REG_D3lPomEn = 0x3f,
	TRX_REG_D3lPomLen = 0x3f,
	TRX_REG_D3lUpDownEx = 0x3f,
//0x40 = 0x,
	TRX_REG_LeaveMapThresh1InBitsync = 0x40,
	TRX_REG_UseMapThresh1InFramesync = 0x40,
//0x41 = 0x,
	TRX_REG_Go2MapThresh1InBitsync = 0x41,
	TRX_REG_D3lFixThres1MapEn = 0x41,
//0x42 = 0x,
	TRX_REG_EnableLO = 0x42,
	TRX_REG_EnableLOdiv10 = 0x42,
	TRX_REG_EnableCsqClock = 0x42,
	TRX_REG_InvertRxClock = 0x42,
	TRX_REG_EnableExtPA = 0x42,
	TRX_REG_EnableIntPA = 0x42,
	TRX_REG_EnableRxClock = 0x42,
	TRX_REG_EnableRx = 0x42,
//0x43 = 0x,
	TRX_REG_LnaFreqAdjust = 0x43,
	TRX_REG_TxPaBias = 0x43,
//0x44 = 0x,
	TRX_REG_TxOutputPower0 = 0x44,
//0x45 = 0x,
	TRX_REG_TxOutputPower1 = 0x45,
//0x46 = 0x,
	TRX_REG_RfRxCompValueI = 0x46,
//0x47 = 0x,
	TRX_REG_RfRxCompValueQ = 0x47,
//0x48 = 0x,
	TRX_REG_SymbolDur = 0x48,
	TRX_REG_SymbolRate = 0x48,
	TRX_REG_ModulationSystem = 0x48,
//0x49 = 0x,
	TRX_REG_Crc2Type = 0x49,
	TRX_REG_UseFec = 0x49,
	TRX_REG_TxRxCryptCrc2Mode = 0x49,
	TRX_REG_TxRxCryptClkMode = 0x49,
//0x4a = 0x,
	TRX_REG_SwapBbBuffers = 0x4a,
	TRX_REG_TxRxBbBufferMode1 = 0x4a,
	TRX_REG_TxRxBbBufferMode0 = 0x4a,
	TRX_REG_FdmaEnable = 0x4a,
	TRX_REG_TxRxMode = 0x4a,
//0x4b = 0x,
	TRX_REG_ChirpMatrix0 = 0x4b,
	TRX_REG_ChirpMatrix1 = 0x4b,
//0x4c = 0x,
	TRX_REG_ChirpMatrix2 = 0x4c,
	TRX_REG_ChirpMatrix3 = 0x4c,
//0x4d = 0x,
	TRX_REG_TxPreTrailMatrix0 = 0x4d,
	TRX_REG_TxPreTrailMatrix1 = 0x4d,
	TRX_REG_TxUnderrunIgnore = 0x4d,
	TRX_REG_TxMacCifsDis = 0x4d,
//0x4e = 0x,
	TRX_REG_TxVCarrSens = 0x4e,
	TRX_REG_TxPhCarrSenseMode = 0x4e,
	TRX_REG_TxVCarrSensAck = 0x4e,
	TRX_REG_TxArq = 0x4e,
	TRX_REG_Tx3Way = 0x4e,
	TRX_REG_TxBackOffAlg = 0x4e,
	TRX_REG_TxFragPrio = 0x4e,
//0x4f = 0x,
	TRX_REG_TxBackOffSeed = 0x4f,
//0x50 = 0x,
	TRX_REG_TxCryptSeqReset = 0x50,
	TRX_REG_TxCryptEn = 0x50,
	TRX_REG_TxCryptId = 0x50,
	TRX_REG_TxCryptSeqN = 0x50,
//0x51 = 0x,
	TRX_REG_TxScrambInit = 0x51,
	TRX_REG_TxScrambEn = 0x51,
//0x54 = 0x,
	TRX_REG_TxPacketType = 0x54,
	TRX_REG_TxAddrSlct = 0x54,
//0x55 = 0x,
	TRX_REG_TxCmdStop = 0x55,
	TRX_REG_TxCmdStart = 0x55,
	TRX_REG_TxBufferCmd = 0x55,
//0x56 = 0x,
	TRX_REG_RxCmdStop = 0x56,
	TRX_REG_RxCmdStart = 0x56,
	TRX_REG_RxBufferCmd = 0x56,
//0x57 = 0x,
	TRX_REG_RxCryptSeqReset = 0x57,
//0x5a = 0x,
	TRX_REG_RxTimeBCrc1Mode = 0x5a,
	TRX_REG_RxCrc2Mode = 0x5a,
	TRX_REG_RxArqMode = 0x5a,
	TRX_REG_RxAddrSegEsMode = 0x5a,
	TRX_REG_RxAddrSegIsMode = 0x5a,
	TRX_REG_RxAddrSegDevIdL = 0x5a,
//0x5b = 0x,
	TRX_REG_RxDataEn = 0x5b,
	TRX_REG_RxBrdcastEn = 0x5b,
	TRX_REG_RxTimeBEn = 0x5b,
	TRX_REG_RxAddrMode = 0x5b,
	TRX_REG_RangingPulses = 0x5b,
//0x5c = 0x,
	TRX_REG_PulseDetDelay = 0x5c,
//0x5d = 0x,
	TRX_REG_GateAdjThreshold = 0x5d,
	TRX_REG_DownPulseDetectDis = 0x5d,
	TRX_REG_UpPulseDetectDis = 0x5d,
//0x5e = 0x,
	TRX_REG_GateSizeUnsync = 0x5e,
	TRX_REG_GateSizeBitsync = 0x5e,
	TRX_REG_GateSizeFramesync = 0x5e,
	TRX_REG_GateAdjBitsyncEn = 0x5e,
	TRX_REG_GateAdjFramesyncEn = 0x5e,
//0x5f = 0x,
	TRX_REG_Go2BitsyncThreshold = 0x5f,
	TRX_REG_LeaveBitsyncThreshold = 0x5f,
//0x60 = 0x,
	TRX_REG_RtcTimeBTxAdj = 0x60,
//0x61 = 0x,
	TRX_REG_RtcTimeBRxAdj = 0x61,
//0x62 = 0x,
	TRX_REG_RtcCmdWr = 0x62,
	TRX_REG_RtcCmdRd = 0x62,
	TRX_REG_RtcTimeBAutoMode = 0x62,
	TRX_REG_RtcTimeBTestMode = 0x62,
//0x63 = 0x,
	TRX_REG_AgcAmplitude = 0x63,
//0x64 = 0x,
	TRX_REG_AgcRangeOffset = 0x64,
	TRX_REG_UseAlternativeAgc = 0x64,
//0x7d = 0x,
	TRX_REG_TxRxDigTestMode = 0x7d,
	TRX_REG_BistBbRamReset = 0x7d,
	TRX_REG_BistBbRamActive = 0x7d,
	TRX_REG_BistCsqRamReset = 0x7d,
	TRX_REG_BistCsqRamActive = 0x7d,
//0x7e = 0x,
//0x7f = 0x,
	TRX_REG_RamTxLength = 0x7f,
	TRX_REG_RamStaAddr0 = 0x7f,
	TRX_REG_SyncWord = 0x7f,
} TRX_W_REG;

typedef enum{
	TRX_REG_Version = 0x01,
	TRX_REG_Revision = 0x02,
	TRX_REG_DioInValueAlarmStatus = 0x04,
	TRX_REG_TxIrqStatus = 0xf,
	TRX_REG_RxIrqStatus = 0xf,
	TRX_REG_BbTimerIrqStatus = 0xf,
	TRX_REG_LoIrqStatus = 0xf,
	TRX_REG_TxIntsRawStat = 0x10,
	TRX_REG_RxIntsRawStat = 0x11,
	TRX_REG_LoIntsRawStat = 0x12,
	TRX_REG_AgcGain = 0x26,
	TRX_REG_ToaOffsetMeanDataValid = 0x2b,
	TRX_REG_PhaseOffsetData = 0x2e,
	TRX_REG_PhaseOffsetAck = 0x2e,
	TRX_REG_ToaOffsetMeanAckValid = 0x30,
	TRX_REG_RxPacketType = 0x31,
	TRX_REG_RxAddrMatch = 0x31,
	TRX_REG_RxCrc1Stat = 0x31,
	TRX_REG_RxCrc2Stat = 0x31,
	TRX_REG_RxCorrBitErr = 0x32,
	TRX_REG_RxCorrErrThres = 0x32,
	TRX_REG_RxAddrSegEsMatch = 0x33,
	TRX_REG_RxAddrSegIsMatch = 0x33,
	TRX_REG_RxCryptEn = 0x33,
	TRX_REG_RxCryptId = 0x33,
	TRX_REG_RxCryptSeqN = 0x33,
	TRX_REG_RxTimeSlotControl = 0x37,
	TRX_REG_TxArqCnt = 0x3c,
	TRX_REG_DebugMacRxCmd = 0x7d,
	TRX_REG_DebugMacTxCmd = 0x7d,
	TRX_REG_DebugMacFsm = 0x7e,
	TRX_REG_DebugMacTxRxCtrl = 0x7e,
	TRX_REG_DebugBitProcFsm = 0x7f,
	TRX_REG_DebugBitProcTx = 0x7f,
	TRX_REG_DebugBitProcRx = 0x7f,
	TRX_REG_DebugRxDetStatus = 0x7f,
	TRX_REG_ToaOffsetMeanData = 0x7f,
	TRX_REG_TxRespTime = 0x7f,
	TRX_REG_ToaOffsetMeanAck = 0x7f
} NTRX_RO_REG;

typedef enum{
	NTRX_SpiBitOrder_MASK = 0x01,
	NTRX_SpiTxDriver_MASK = 0x01 << NA_SpiTxDriver_B,
	NTRX_IrqPolarity_MASK = 0x01 << NA_IrqPolarity_B,
	NTRX_IrqDriver_MASK = 0x01 << NA_IrqDriver_B,
	NTRX_Version_MASK = 1,
	NTRX_WakeUpTimeByte_MASK = 0xff,
	NTRX_Revision_MASK = 1,
	NTRX_WakeUpTimeWe_MASK = 0x07 << NA_WakeUpTimeWe_LSB,
	NTRX_BattMgmtEnable_MASK = 0x01 << NA_BattMgmtEnable_B,
	NTRX_BattMgmtThreshold_MASK = 0x0f << NA_BattMgmtThreshold_LSB,
	NTRX_BattMgmtCompare_MASK = 0x01 << NA_BattMgmtCompare_B,
	NTRX_DioDirection_MASK = 0x01 << NA_DioDirection_B,
	NTRX_DioOutValueAlarmEnable_MASK = 0x01 << NA_DioOutValueAlarmEnable_B,
	NTRX_DioAlarmStart_MASK = 0x01 << NA_DioAlarmStart_B,
	NTRX_DioAlarmPolarity_MASK = 0x01 << NA_DioAlarmPolarity_B,
	NTRX_DioUsePullup_MASK = 0x01 << NA_DioUsePullup_B,
	NTRX_DioUsePulldown_MASK = 0x01 << NA_DioUsePulldown_B,
	NTRX_DioInValueAlarmStatus_MASK = 0x0f << NA_DioInValueAlarmStatus_LSB,
	NTRX_DioPortWe_MASK = 0x0f << NA_DioPortWe_LSB,
	NTRX_EnableWakeUpRtc_MASK = 0x01 << NA_EnableWakeUpRtc_B,
	NTRX_EnableWakeUpDio_MASK = 0x01 << NA_EnableWakeUpDio_B,
	NTRX_PowerUpTime_MASK = 0x07 << NA_PowerUpTime_LSB,
	NTRX_PowerDownMode_MASK = 0x01 << NA_PowerDownMode_B,
	NTRX_PowerDown_MASK = 0x01 << NA_PowerDown_B,
	NTRX_ResetBbClockGate_MASK = 0x01 << NA_ResetBbClockGate_B,
	NTRX_ResetBbRadioCtrl_MASK = 0x01 << NA_ResetBbRadioCtrl_B,
	NTRX_EnableBbCrystal_MASK = 0x01 << NA_EnableBbCrystal_B,
	NTRX_EnableBbClock_MASK = 0x01 << NA_EnableBbClock_B,
	NTRX_BypassBbCrystal_MASK = 0x01 << NA_BypassBbCrystal_B,
	NTRX_FeatureClockFreq_MASK = 0x07 << NA_FeatureClockFreq_LSB,
	NTRX_EnableFeatureClock_MASK = 0x01 << NA_EnableFeatureClock_B,
	NTRX_UsePullup4Spiclk_MASK = 0x01 << NA_UsePullup4Spiclk_B,
	NTRX_UsePulldown4Spiclk_MASK = 0x01 << NA_UsePulldown4Spiclk_B,
	NTRX_UsePullup4Spissn_MASK = 0x01 << NA_UsePullup4Spissn_B,
	NTRX_UsePulldown4Spissn_MASK = 0x01 << NA_UsePulldown4Spissn_B,
	NTRX_UsePullup4Spirxd_MASK = 0x01 << NA_UsePullup4Spirxd_B,
	NTRX_UsePulldown4Spirxd_MASK = 0x01 << NA_UsePulldown4Spirxd_B,
	NTRX_UsePullup4Spitxd_MASK = 0x01 << NA_UsePullup4Spitxd_B,
	NTRX_UsePulldown4Spitxd_MASK = 0x01 << NA_UsePulldown4Spitxd_B,
	NTRX_UsePullup4Por_MASK = 0x01 << NA_UsePullup4Por_B,
	NTRX_UsePulldown4Por_MASK = 0x01 << NA_UsePulldown4Por_B,
	NTRX_UsePullup4Pamp_MASK = 0x01 << NA_UsePullup4Pamp_B,
	NTRX_UsePulldown4Pamp_MASK = 0x01 << NA_UsePulldown4Pamp_B,
	NTRX_UsePullup4Ucirq_MASK = 0x01 << NA_UsePullup4Ucirq_B,
	NTRX_UsePulldown4Ucirq_MASK = 0x01 << NA_UsePulldown4Ucirq_B,
	NTRX_UsePullup4Ucrst_MASK = 0x01 << NA_UsePullup4Ucrst_B,
	NTRX_UsePulldown4Ucrst_MASK = 0x01 << NA_UsePulldown4Ucrst_B,
	NTRX_WritePulls4Spi_MASK = 0x01 << NA_WritePulls4Spi_B,
	NTRX_WritePulls4Pads_MASK = 0x01 << NA_WritePulls4Pads_B,
	NTRX_TestModes_MASK = 0x0f << NA_TestModes_LSB,
	NTRX_RfTestSelect_MASK = 0x0f << NA_RfTestSelect_LSB,
	NTRX_RamIndex_MASK = 0x03 << NA_RamIndex_LSB,
	NTRX_DeviceSelect_MASK = 0x03 << NA_DeviceSelect_LSB,
	NTRX_TxIrqEnable_MASK = 0x01 << NA_TxIrqEnable_B,
	NTRX_RxIrqEnable_MASK = 0x01 << NA_RxIrqEnable_B,
	NTRX_BbTimerIrqEnable_MASK = 0x01 << NA_BbTimerIrqEnable_B,
	NTRX_LoIrqEnable_MASK = 0x01 << NA_LoIrqEnable_B,
	NTRX_TxIrqStatus_MASK = 0x01 << NA_TxIrqStatus_B,
	NTRX_RxIrqStatus_MASK = 0x01 << NA_RxIrqStatus_B,
	NTRX_BbTimerIrqStatus_MASK = 0x01 << NA_BbTimerIrqStatus_B,
	NTRX_LoIrqStatus_MASK = 0x01 << NA_LoIrqStatus_B,
	NTRX_TxIntsRawStat_MASK = 0x3f << NA_TxIntsRawStat_LSB,
	NTRX_TxIntsReset_MASK = 0x3f << NA_TxIntsReset_LSB,
	NTRX_RxIntsRawStat_MASK = 0x7f << NA_RxIntsRawStat_LSB,
	NTRX_RxIntsReset_MASK = 0x7f << NA_RxIntsReset_LSB,
	NTRX_LoIntsRawStat_MASK = 0x03 << NA_LoIntsRawStat_LSB,
	NTRX_LoIntsReset_MASK = 0x03 << NA_LoIntsReset_LSB,
	NTRX_ClearBasebandTimerInt_MASK = 0x01 << NA_ClearBasebandTimerInt_B,
	NTRX_TxIntsEn_MASK = 0x3f << NA_TxIntsEn_LSB,
	NTRX_RxIntsEn_MASK = 0x7f << NA_RxIntsEn_LSB,
	NTRX_LoIntsEn_MASK = 0x03 << NA_LoIntsEn_LSB,
	NTRX_LoEnableFastTuning_MASK = 0x01 << NA_LoEnableFastTuning_B,
	NTRX_LoFastTuningLevel_MASK = 0x07 << NA_LoFastTuningLevel_LSB,
	NTRX_LoEnableLsbNeg_MASK = 0x01 << NA_LoEnableLsbNeg_B,
	NTRX_LoEnableRecalib_MASK = 0x01 << NA_LoEnableRecalib_B,
	NTRX_UseLoRxCaps_MASK = 0x01 << NA_UseLoRxCaps_B,
	NTRX_LoTargetValue_MASK = 2,
	NTRX_AgcThresHold1_MASK = 0xff << NA_AgcThresHold1_LSB,
	NTRX_AgcThresHold2_MASK = 0xff << NA_AgcThresHold2_LSB,
	NTRX_HoldAgcInBitSync_MASK = 0x7f << NA_HoldAgcInBitSync_LSB,
	NTRX_HoldAgcInFrameSync_MASK = 0x01 << NA_HoldAgcInFrameSync_B,
	NTRX_AgcDeadTime_MASK = 0x3f << NA_AgcDeadTime_LSB,
	NTRX_AgcNregLength_MASK = 0x03 << NA_AgcNregLength_LSB,
	NTRX_AgcIntTime_MASK = 2,
	NTRX_AgcValue_MASK = 0x3f << NA_AgcValue_LSB,
	NTRX_AgcDefaultEn_MASK = 0x01 << NA_AgcDefaultEn_B,
	NTRX_AgcHold_MASK = 0x01 << NA_AgcHold_B,
	NTRX_AgcRssiThres_MASK = 0x3f << NA_AgcRssiThres_LSB,
	NTRX_AgcGain_MASK = 0x3f << NA_AgcGain_LSB,
	NTRX_AgcEnable_MASK = 0x01 << NA_AgcEnable_B,
	NTRX_ChirpFilterCaps_MASK = 0x0f << NA_ChirpFilterCaps_LSB,
	NTRX_FctClockEn_MASK = 0x01 << NA_FctClockEn_B,
	NTRX_StartFctMeasure_MASK = 0x01 << NA_StartFctMeasure_B,
	NTRX_EnableTx_MASK = 0x01 << NA_EnableTx_B,
	NTRX_FctPeriod_MASK = 0x0f << NA_FctPeriod_LSB,
	NTRX_ToaOffsetMeanDataValid_MASK = 0x01 << NA_ToaOffsetMeanDataValid_B,
	NTRX_PhaseOffsetData_MASK = 0x07 << NA_PhaseOffsetData_LSB,
	NTRX_PhaseOffsetAck_MASK = 0x07 << NA_PhaseOffsetAck_LSB,
	NTRX_ToaOffsetMeanAckValid_MASK = 0x01 << NA_ToaOffsetMeanAckValid_B,
	NTRX_RxPacketType_MASK = 0x0f << NA_RxPacketType_LSB,
	NTRX_RxAddrMatch_MASK = 0x0f << NA_RxAddrMatch_LSB,
	NTRX_RxCrc1Stat_MASK = 0x01 << NA_RxCrc1Stat_B,
	NTRX_RxCrc2Stat_MASK = 0x01 << NA_RxCrc2Stat_B,
	NTRX_RxCorrBitErr_MASK = 0x0f << NA_RxCorrBitErr_LSB,
	NTRX_RxCorrErrThres_MASK = 0x0f << NA_RxCorrErrThres_LSB,
	NTRX_RxAddrSegEsMatch_MASK = 0x01 << NA_RxAddrSegEsMatch_B,
	NTRX_RxAddrSegIsMatch_MASK = 0x01 << NA_RxAddrSegIsMatch_B,
	NTRX_RxCryptEn_MASK = 0x01 << NA_RxCryptEn_B,
	NTRX_RxCryptId_MASK = 0x03 << NA_RxCryptId_LSB,
	NTRX_RxCryptSeqN_MASK = 0x01 << NA_RxCryptSeqN_B,
	NTRX_TxTimeSlotControl_MASK = 0x01 << NA_TxTimeSlotControl_B,
	NTRX_RxTimeSlotControl_MASK = 0x01 << NA_RxTimeSlotControl_B,
	NTRX_TxArqCnt_MASK = 0x0f << NA_TxArqCnt_LSB,
	NTRX_TxArqMax_MASK = 0x0f << NA_TxArqMax_LSB,
	NTRX_CsqDitherValue_MASK = 0x03 << NA_CsqDitherValue_LSB,
	NTRX_CsqUsePhaseShift_MASK = 0x01 << NA_CsqUsePhaseShift_B,
	NTRX_CsqUse4Phases_MASK = 0x01 << NA_CsqUse4Phases_B,
	NTRX_CsqAsyMode_MASK = 0x01 << NA_CsqAsyMode_B,
	NTRX_CsqMemAddrInit_MASK = 0x01 << NA_CsqMemAddrInit_B,
	NTRX_CsqUseRam_MASK = 0x01 << NA_CsqUseRam_B,
	NTRX_CsqTest_MASK = 0x01 << NA_CsqTest_B,
	NTRX_CsqSetValue_MASK = 0x3f << NA_CsqSetValue_LSB,
	NTRX_CsqSetIValue_MASK = 0x01 << NA_CsqSetIValue_B,
	NTRX_CsqSetQValue_MASK = 0x01 << NA_CsqSetQValue_B,
	NTRX_D3lFixnMap_MASK = 0x01 << NA_D3lFixnMap_B,
	NTRX_D3lPomEn_MASK = 0x01 << NA_D3lPomEn_B,
	NTRX_D3lPomLen_MASK = 0x03 << NA_D3lPomLen_LSB,
	NTRX_D3lUpDownEx_MASK = 0x01 << NA_D3lUpDownEx_B,
	NTRX_LeaveMapThresh1InBitsync_MASK = 0x7f << NA_LeaveMapThresh1InBitsync_LSB,
	NTRX_UseMapThresh1InFramesync_MASK = 0x01 << NA_UseMapThresh1InFramesync_B,
	NTRX_Go2MapThresh1InBitsync_MASK = 0x7f << NA_Go2MapThresh1InBitsync_LSB,
	NTRX_D3lFixThres1MapEn_MASK = 0x01 << NA_D3lFixThres1MapEn_B,
	NTRX_EnableLO_MASK = 0x01 << NA_EnableLO_B,
	NTRX_EnableLOdiv10_MASK = 0x01 << NA_EnableLOdiv10_B,
	NTRX_EnableCsqClock_MASK = 0x01 << NA_EnableCsqClock_B,
	NTRX_InvertRxClock_MASK = 0x01 << NA_InvertRxClock_B,
	NTRX_EnableExtPA_MASK = 0x01 << NA_EnableExtPA_B,
	NTRX_EnableIntPA_MASK = 0x01 << NA_EnableIntPA_B,
	NTRX_EnableRxClock_MASK = 0x01 << NA_EnableRxClock_B,
	NTRX_EnableRx_MASK = 0x01 << NA_EnableRx_B,
	NTRX_LnaFreqAdjust_MASK = 0x7 << NA_LnaFreqAdjust_LSB,
	NTRX_TxPaBias_MASK = 0x7 << NA_TxPaBias_LSB,
	NTRX_TxOutputPower0_MASK = 0x3f << NA_TxOutputPower0_LSB,
	NTRX_TxOutputPower1_MASK = 0x3f << NA_TxOutputPower1_LSB,
	NTRX_RfRxCompValueI_MASK = 0x1f << NA_RfRxCompValueI_LSB,
	NTRX_RfRxCompValueQ_MASK = 0x1f << NA_RfRxCompValueQ_LSB,
	NTRX_SymbolDur_MASK = 0x7 << NA_SymbolDur_LSB,
	NTRX_SymbolRate_MASK = 0x7 << NA_SymbolRate_LSB,
	NTRX_ModulationSystem_MASK = 0x01 << NA_ModulationSystem_B,
	NTRX_Crc2Type_MASK = 0x3 << NA_Crc2Type_LSB,
	NTRX_UseFec_MASK = 0x01 << NA_UseFec_B,
	NTRX_TxRxCryptCrc2Mode_MASK = 0x01 << NA_TxRxCryptCrc2Mode_B,
	NTRX_TxRxCryptClkMode_MASK = 0x0f << NA_TxRxCryptClkMode_LSB,
	NTRX_SwapBbBuffers_MASK = 0x01 << NA_SwapBbBuffers_B,
	NTRX_TxRxBbBufferMode1_MASK = 0x01 << NA_TxRxBbBufferMode1_B,
	NTRX_TxRxBbBufferMode0_MASK = 0x01 << NA_TxRxBbBufferMode0_B,
	NTRX_FdmaEnable_MASK = 0x01 << NA_FdmaEnable_B,
	NTRX_TxRxMode_MASK = 0x01 << NA_TxRxMode_B,
	NTRX_ChirpMatrix0_MASK = 0x07 << NA_ChirpMatrix0_LSB,
	NTRX_ChirpMatrix1_MASK = 0x07 << NA_ChirpMatrix1_LSB,
	NTRX_ChirpMatrix2_MASK = 0x07 << NA_ChirpMatrix2_LSB,
	NTRX_ChirpMatrix3_MASK = 0x07 << NA_ChirpMatrix3_LSB,
	NTRX_TxPreTrailMatrix0_MASK = 0x03 << NA_TxPreTrailMatrix0_LSB,
	NTRX_TxPreTrailMatrix1_MASK = 0x03 << NA_TxPreTrailMatrix1_LSB,
	NTRX_TxUnderrunIgnore_MASK = 0x01 << NA_TxUnderrunIgnore_B,
	NTRX_TxMacCifsDis_MASK = 0x01 << NA_TxMacCifsDis_B,
	NTRX_TxVCarrSens_MASK = 0x01 << NA_TxVCarrSens_B,
	NTRX_TxPhCarrSenseMode_MASK = 0x03 << NA_TxPhCarrSenseMode_LSB,
	NTRX_TxVCarrSensAck_MASK = 0x01 << NA_TxVCarrSensAck_B,
	NTRX_TxArq_MASK = 0x01 << NA_TxArq_B,
	NTRX_Tx3Way_MASK = 0x01 << NA_Tx3Way_B,
	NTRX_TxBackOffAlg_MASK = 0x01 << NA_TxBackOffAlg_B,
	NTRX_TxFragPrio_MASK = 0x01 << NA_TxFragPrio_B,
	NTRX_TxBackOffSeed_MASK = 0xff << NA_TxBackOffSeed_LSB,
	NTRX_TxCryptSeqReset_MASK = 0x0f << NA_TxCryptSeqReset_LSB,
	NTRX_TxCryptEn_MASK = 0x01 << NA_TxCryptEn_B,
	NTRX_TxCryptId_MASK = 0x03 << NA_TxCryptId_LSB,
	NTRX_TxCryptSeqN_MASK = 0x01 << NA_TxCryptSeqN_B,
	NTRX_TxScrambInit_MASK = 0x7f << NA_TxScrambInit_LSB,
	NTRX_TxScrambEn_MASK = 0x01 << NA_TxScrambEn_B,
	NTRX_TxPacketType_MASK = 0x0f << NA_TxPacketType_LSB,
	NTRX_TxAddrSlct_MASK = 0x01 << NA_TxAddrSlct_B,
	NTRX_TxCmdStop_MASK = 0x01 << NA_TxCmdStop_B,
	NTRX_TxCmdStart_MASK = 0x01 << NA_TxCmdStart_B,
	NTRX_TxBufferCmd_MASK = 0x03 << NA_TxBufferCmd_LSB,
	NTRX_RxCmdStop_MASK = 0x01 << NA_RxCmdStop_B,
	NTRX_RxCmdStart_MASK = 0x01 << NA_RxCmdStart_B,
	NTRX_RxBufferCmd_MASK = 0x03 << NA_RxBufferCmd_LSB,
	NTRX_RxCryptSeqReset_MASK = 0x0f << NA_RxCryptSeqReset_LSB,
	NTRX_RxTimeBCrc1Mode_MASK = 0x01 << NA_RxTimeBCrc1Mode_B,
	NTRX_RxCrc2Mode_MASK = 0x01 << NA_RxCrc2Mode_B,
	NTRX_RxArqMode_MASK = 0x03 << NA_RxArqMode_LSB,
	NTRX_RxAddrSegEsMode_MASK = 0x01 << NA_RxAddrSegEsMode_B,
	NTRX_RxAddrSegIsMode_MASK = 0x01 << NA_RxAddrSegIsMode_B,
	NTRX_RxAddrSegDevIdL_MASK = 0x03 << NA_RxAddrSegDevIdL_LSB,
	NTRX_RxDataEn_MASK = 0x01 << NA_RxDataEn_B,
	NTRX_RxBrdcastEn_MASK = 0x01 << NA_RxBrdcastEn_B,
	NTRX_RxTimeBEn_MASK = 0x01 << NA_RxTimeBEn_B,
	NTRX_RxAddrMode_MASK = 0x01 << NA_RxAddrMode_B,
	NTRX_RangingPulses_MASK = 0x0f << NA_RangingPulses_LSB,
	NTRX_PulseDetDelay_MASK = 0x1f << NA_PulseDetDelay_LSB,
	NTRX_GateAdjThreshold_MASK = 0x07 << NA_GateAdjThreshold_LSB,
	NTRX_DownPulseDetectDis_MASK = 0x01 << NA_DownPulseDetectDis_B,
	NTRX_UpPulseDetectDis_MASK = 0x01 << NA_UpPulseDetectDis_B,
	NTRX_GateSizeUnsync_MASK = 0x03 << NA_GateSizeUnsync_LSB,
	NTRX_GateSizeBitsync_MASK = 0x03 << NA_GateSizeBitsync_LSB,
	NTRX_GateSizeFramesync_MASK = 0x03 << NA_GateSizeFramesync_LSB,
	NTRX_GateAdjBitsyncEn_MASK = 0x01 << NA_GateAdjBitsyncEn_B,
	NTRX_GateAdjFramesyncEn_MASK = 0x01 << NA_GateAdjFramesyncEn_B,
	NTRX_Go2BitsyncThreshold_MASK = 0x07 << NA_Go2BitsyncThreshold_LSB,
	NTRX_LeaveBitsyncThreshold_MASK = 0x07 << NA_LeaveBitsyncThreshold_LSB,
	NTRX_RtcTimeBTxAdj_MASK = 0xff << NA_RtcTimeBTxAdj_LSB,
	NTRX_RtcTimeBRxAdj_MASK = 0xff << NA_RtcTimeBRxAdj_LSB,
	NTRX_RtcCmdWr_MASK = 0x01 << NA_RtcCmdWr_B,
	NTRX_RtcCmdRd_MASK = 0x01 << NA_RtcCmdRd_B,
	NTRX_RtcTimeBAutoMode_MASK = 0x01 << NA_RtcTimeBAutoMode_B,
	NTRX_RtcTimeBTestMode_MASK = 0x01 << NA_RtcTimeBTestMode_B,
	NTRX_AgcAmplitude_MASK = 0xff << NA_AgcAmplitude_LSB,
	NTRX_AgcRangeOffset_MASK = 0xff << NA_AgcRangeOffset_LSB,
	NTRX_UseAlternativeAgc_MASK = 0x01 << NA_UseAlternativeAgc_B,
	NTRX_TxRxDigTestMode_MASK = 0x01 << NA_TxRxDigTestMode_B,
	NTRX_DebugMacRxCmd_MASK = 0x01 << NA_DebugMacRxCmd_B,
	NTRX_DebugMacTxCmd_MASK = 0x01 << NA_DebugMacTxCmd_B,
	NTRX_BistBbRamReset_MASK = 0x01 << NA_BistBbRamReset_B,
	NTRX_BistBbRamActive_MASK = 0x01 << NA_BistBbRamActive_B,
	NTRX_BistCsqRamReset_MASK = 0x01 << NA_BistCsqRamReset_B,
	NTRX_BistCsqRamActive_MASK = 0x01 << NA_BistCsqRamActive_B,
	NTRX_DebugMacFsm_MASK = 0x0f << NA_DebugMacFsm_LSB,
	NTRX_DebugMacTxRxCtrl_MASK = 0x0f << NA_DebugMacTxRxCtrl_LSB,
	NTRX_DebugBitProcFsm_MASK = 0x0f << NA_DebugBitProcFsm_LSB,
	NTRX_DebugBitProcTx_MASK = 0x01 << NA_DebugBitProcTx_B,
	NTRX_DebugBitProcRx_MASK = 0x01 << NA_DebugBitProcRx_B,
	NTRX_DebugRxDetStatus_MASK = 0x03 << NA_DebugRxDetStatus_LSB,
	NTRX_RamTxLength_MASK = 2,
	NTRX_RamStaAddr0_MASK = 6,
	NTRX_SyncWord_MASK = 6,
	NTRX_ToaOffsetMeanData_MASK = 2,
	NTRX_TxRespTime_MASK = 2,
	NTRX_ToaOffsetMeanAck_MASK = 2
} TRX_REG_MASK;

static uint8_t TRXShadowReg[] =
{
  0,   0,   0,   0,   0,   0,   0,   6,
  0,   0,   0,   0,   0,   0,   0,   0,
  0,   0,   0,   0,   0,   0,   0,   0,
  0,   0,   0,   0,   0,   0,   0,   3,
  6, 152,  12,   0,   0,  63,  30,   6,
  0,   0, 171, 105, 202, 148, 146, 213,
 44, 171,  48,   0,   0,   0,   0,   0,
  0,   0,   0,   0, 224,   4,   0,   1,
  3,   7,   0,   3,  63,  63,  15,  15,
115,   0,  16,  16,  67,  20,  16,   0,
  0, 255,   0,   0,   0,   0,   0,   0,
  0,   0,  11,  95,   5,   7, 213,  98,
  0,   0,   0,  12,  10,   0,   0,   0,
  0,   0,   0,   0,   0,   0,   0,   0,
  0,   0,   0,   0,   0,   0,   0,   0,
  0,   0,   0,   0,   0,  80,   0,   0
};


// 16
static uint8_t NA5TR1_Rx_80MHz[] =
{
	0x02, 0x63, 0x39, 0xC7, 0x0E, 0x0F, 0xC0, 0x1F, 0xFF, 0xE0, 0x0F, 0xC3, 0xC3, 0x8E, 0x73, 0x39, 
	0x00, 0xCE, 0x63, 0x1C, 0x38, 0x3E, 0x03, 0xFF, 0xFF, 0xFF, 0x01, 0xF0, 0xF0, 0xE7, 0x19, 0x8C
};


// 4
static uint8_t NA5TR1_Rx_22MHz[] =
{
	0xC6, 0x0F, 0xF0, 0x63, 
	0x9C, 0x7F, 0xFE, 0x39
};


// 61
static uint8_t NA5TR1_Tx_80MHz[] =
{
	0x20, 0xE0, 0x1D, 0xDE, 0x27, 0xA3, 0x12, 0x17, 0xF1, 0xB1, 0x10, 0x04, 0xA4, 0xFF, 0x6D, 0x09, 
	0x03, 0xE2, 0xBD, 0x75, 0xD4, 0xC0, 0x0D, 0x2C, 0x7F, 0x35, 0x19, 0x03, 0xC3, 0x57, 0xB1, 0xBF, 
	0x39, 0xA5, 0xCF, 0x02, 0xC2, 0x0F, 0xE2, 0x73, 0xFD, 0xFE, 0x77, 0xEA, 0x5B, 0x4F, 0x85, 0x01, 
	0x80, 0x84, 0x49, 0x10, 0xD7, 0x9E, 0xE4, 0x2A, 0xEE, 0xB1, 0x33, 0x75, 0x75, 
	0x08, 0xF7, 0xF7, 0x48, 0x1A, 0x56, 0xA4, 0x09, 0xCD, 0xE8, 0x81, 0xA4, 0xED, 0x4E, 0x36, 0xA0, 
	0xE4, 0xFC, 0x9F, 0x89, 0x21, 0xC0, 0x97, 0xDE, 0x5F, 0x9A, 0x13, 0x20, 0xD2, 0xE9, 0xAE, 0x5F, 
	0xDC, 0x16, 0x51, 0x60, 0xB1, 0xA6, 0xFA, 0x8E, 0x9F, 0x9E, 0xAC, 0x28, 0x05, 0x52, 0xE0, 0x90, 
	0x40, 0xE1, 0x63, 0xB5, 0xE6, 0xF8, 0xF9, 0xDB, 0xCB, 0xAC, 0x8D, 0x7D, 0x6D, 
	0x7E, 0x79, 0x85, 0x96, 0x6E, 0x51, 0x95, 0xCF, 0x7A, 0x14, 0x5C, 0xEA, 0xDB, 0x42, 0x04, 0x6C, 
	0xEE, 0xE3, 0x62, 0x04, 0x30, 0xB1, 0xFF, 0xCF, 0x56, 0x04, 0x1C, 0x7D, 0xE2, 0xFF, 0xC7, 0x62, 
	0x14, 0x04, 0x30, 0x81, 0xD2, 0xFB, 0xF7, 0xC7, 0x86, 0x45, 0x14, 0x00, 0x08, 0x24, 0x4C, 0x7D, 
	0xA6, 0xCA, 0xE7, 0xF7, 0xFF, 0xFF, 0xFB, 0xF7, 0xEF, 0xE7, 0xE3, 0xDF, 0xDB
};


// 8
static uint8_t NA5TR1_Tx_22MHz[] =
{
	0x22, 0x54, 0x1E, 0xB9, 0x07, 0xC8, 0xA4, 0xB3, 
	0x5C, 0x40, 0xFF, 0xC7, 0xC0, 0x55, 0xFB, 0x8D, 
	0x5E, 0x9C, 0xB7, 0x00, 0x8C, 0xFB, 0xF3, 0xDB
};


#define TxEND (0x01 << NA_TxEnd_B)
#define TX_IRQ_MASK TxEND

#define NTRX_TX_START (0x01 << NA_TxCmdStart_B)
#define NTRX_TX_BUFF0 (0x01 << NA_TxBufferCmd_LSB)
#define NTRX_TX_BUFF1 (0x01 << NA_TxBufferCmd_MSB)


#endif /* RF_NTRX_H */
