#pragma once

#include <arduino.h>
#include "..\include\permission.h"
#include "..\include\esp32_rpc.h"

/**
   * @brief This function sets up the Resistive Bias mode and also selects the master clock frequency.
   * @brief Uses Register: CNFG_GEN-0x10
   * @param En_rbias: Enable and Select Resitive Lead Bias Mode
   * @param Rbiasv: Resistive Bias Mode Value Selection
   * @param Rbiasp: Enables Resistive Bias on Positive Input
   * @param Rbiasn: Enables Resistive Bias on Negative Input
   * @param Fmstr: Selects Master Clock Frequency
   * @returns 0-if no error.  A non-zero value indicates an error.
   *
  */
  int max30003_Rbias_FMSTR_Init(uint8_t En_rbias, uint8_t Rbiasv,
                                uint8_t Rbiasp, uint8_t Rbiasn, uint8_t Fmstr);
 
  /**
   * @brief This function uses sets up the calibration signal internally.  If it is desired to use the internal signal, then
   * @brief this function must be called and the registers set, prior to setting the CALP_SEL and CALN_SEL in the ECG_InitStart
   * @brief and BIOZ_InitStart functions.
   * @brief Uses Register: CNFG_CAL-0x12
   * @param En_Vcal: Calibration Source (VCALP and VCALN) Enable
   * @param Vmode:   Calibration Source Mode Selection
   * @param Vmag:    Calibration Source Magnitude Selection (VMAG)
   * @param Fcal:    Calibration Source Frequency Selection (FCAL)
   * @param Thigh:   Calibration Source Time High Selection
   * @param Fifty:   Calibration Source Duty Cycle Mode Selection
   * @returns 0-if no error.  A non-zero value indicates an error.
   *
   */
  int max30003_CAL_InitStart(uint8_t En_Vcal, uint8_t Vmode, uint8_t Vmag,
                             uint8_t Fcal, uint16_t Thigh, uint8_t Fifty);
 
  /**
   * @brief This function disables the VCAL signal
   * @returns 0-if no error.  A non-zero value indicates an error.
   */
  int max30003_CAL_Stop(void);
 
  /**
   * @brief This function handles the assignment of the two interrupt pins (INTB & INT2B) with various
   * @brief functions/behaviors  of the max30003.  Also, each pin can be configured for different drive capability.
   * @brief Uses Registers: EN_INT-0x02 and EN_INT2-0x03.
   * @param max30003_intrpt_Locatio_t  <argument>:  All the arguments with the aforementioned enumeration essentially
   *        can be configured to generate an interrupt on either INTB or INT2B or NONE.
   * @param max30003_intrpt_type_t  intb_Type:  INTB Port Type (EN_INT Selections).
   * @param max30003_intrpt_type _t int2b_Type:   INT2B Port Type (EN_INT2 Selections)
   * @returns 0-if no error.  A non-zero value indicates an error.
   *
   */
    int max30003_INT_assignment(  
        max30003_intrpt_Location_t en_enint_loc,     max30003_intrpt_Location_t en_eovf_loc,  max30003_intrpt_Location_t en_fstint_loc,
        max30003_intrpt_Location_t en_dcloffint_loc, max30003_intrpt_Location_t en_bint_loc,  max30003_intrpt_Location_t en_bovf_loc,
        max30003_intrpt_Location_t en_bover_loc,     max30003_intrpt_Location_t en_bundr_loc, max30003_intrpt_Location_t en_bcgmon_loc,
        max30003_intrpt_Location_t en_pint_loc,      max30003_intrpt_Location_t en_povf_loc,  max30003_intrpt_Location_t en_pedge_loc,
        max30003_intrpt_Location_t en_lonint_loc,    max30003_intrpt_Location_t en_rrint_loc, max30003_intrpt_Location_t en_samp_loc,
        max30003_intrpt_type_t  intb_Type,           max30003_intrpt_type_t int2b_Type);
 
  /**
   * @brief For max30003/3 ONLY
   * @brief This function sets up the max30003 for the ECG measurements.
   * @brief Registers used:  CNFG_EMUX, CNFG_GEN, MNGR_INT, CNFG_ECG.
   * @param En_ecg: ECG Channel Enable <CNFG_GEN register bits>
   * @param Openp: Open the ECGN Input Switch (most often used for testing and calibration studies) <CNFG_EMUX register bits>
   * @param Openn: Open the ECGN Input Switch (most often used for testing and calibration studies) <CNFG_EMUX register bits>
   * @param Calp_sel: ECGP Calibration Selection <CNFG_EMUX register bits>
   * @param Caln_sel: ECGN Calibration Selection <CNFG_EMUX register bits>
   * @param E_fit: ECG FIFO Interrupt Threshold (issues EINT based on number of unread FIFO records) <CNFG_GEN register bits>
   * @param Clr_rrint: RTOR R Detect Interrupt (RRINT) Clear Behavior <CNFG_GEN register bits>
   * @param Rate: ECG Data Rate
   * @param Gain: ECG Channel Gain Setting
   * @param Dhpf: ECG Channel Digital High Pass Filter Cutoff Frequency
   * @param Dlpf:  ECG Channel Digital Low Pass Filter Cutoff Frequency
   * @returns 0-if no error.  A non-zero value indicates an error.
   *
   */
  int max30003_ECG_InitStart(uint8_t En_ecg, uint8_t Openp, uint8_t Openn,
                             uint8_t Pol, uint8_t Calp_sel, uint8_t Caln_sel,
                             uint8_t E_fit, uint8_t Rate, uint8_t Gain,
                             uint8_t Dhpf, uint8_t Dlpf);
 
  /**
   * @brief For max30003/3 ONLY
   * @brief This function enables the Fast mode feature of the ECG.
   * @brief Registers used: MNGR_INT-0x04, MNGR_DYN-0x05
   * @param Clr_Fast: FAST MODE Interrupt Clear Behavior <MNGR_INT Register>
   * @param Fast: ECG Channel Fast Recovery Mode Selection (ECG High Pass Filter Bypass) <MNGR_DYN Register>
   * @param Fast_Th: Automatic Fast Recovery Threshold
   * @returns 0-if no error.  A non-zero value indicates an error.
   *
   */
  int max30003_ECGFast_Init(uint8_t Clr_Fast, uint8_t Fast, uint8_t Fast_Th);
 
  /**
  * @brief For max30003/3 ONLY
  * @brief This function disables the ECG.
  * @brief Uses Register CNFG_GEN-0x10.
  * @returns 0-if no error.  A non-zero value indicates an error.
  *
  */
  int max30003_Stop_ECG(void);
 
 
  /**
   * @brief For max30003/3/4 ONLY
   * @brief Sets up the device for RtoR measurement
   * @param EN_rtor: ECG RTOR Detection Enable <RTOR1 Register>
   * @param Wndw: R to R Window Averaging (Window Width = RTOR_WNDW[3:0]*8mS) <RTOR1 Register>
   * @param Gain: R to R Gain (where Gain = 2^RTOR_GAIN[3:0], plus an auto-scale option) <RTOR1 Register>
   * @param Pavg: R to R Peak Averaging Weight Factor <RTOR1 Register>
   * @param Ptsf: R to R Peak Threshold Scaling Factor <RTOR1 Register>
   * @param Hoff: R to R minimum Hold Off <RTOR2 Register>
   * @param Ravg: R to R Interval Averaging Weight Factor <RTOR2 Register>
   * @param Rhsf: R to R Interval Hold Off Scaling Factor <RTOR2 Register>
   * @param Clr_rrint: RTOR Detect Interrupt Clear behaviour <MNGR_INT Register>
   * @returns 0-if no error.  A non-zero value indicates an error.
   *
   */
  int max30003_RtoR_InitStart(uint8_t En_rtor, uint8_t Wndw, uint8_t Gain,
                              uint8_t Pavg, uint8_t Ptsf, uint8_t Hoff,
                              uint8_t Ravg, uint8_t Rhsf, uint8_t Clr_rrint);
 
  /**
   * @brief For max30003/3/4 ONLY
   * @brief This function disables the RtoR.  Uses Register CNFG_RTOR1-0x1D
   * @returns 0-if no error.  A non-zero value indicates an error.
   *
   */
  int max30003_Stop_RtoR(void);
 
  /**
   * @brief This is a function that waits for the PLL to lock; once a lock is achieved it exits out. (For convenience only)
   * @returns 0-if no error.  A non-zero value indicates an error.
   *
   */
  int max30003_PLL_lock(void);
 
  /**
   * @brief This function causes the max30003 to reset.  Uses Register SW_RST-0x08
   * @return 0-if no error.  A non-zero value indicates an error.
   *
   */
  int max30003_sw_rst(void);
 
  /**
   * @brief This function provides a SYNCH operation.  Uses Register SYCNH-0x09. Please refer to the data sheet for
   * @brief the details on how to use this.
   * @returns 0-if no error.  A non-zero value indicates an error.
   *
   */
  int max30003_synch(void);
 
  /**
   * @brief This function performs a FIFO Reset.  Uses Register FIFO_RST-0x0A. Please refer to the data sheet
   * @brief for the details on how to use this.
   * @returns 0-if no error.  A non-zero value indicates an error.
   */
  int max300001_fifo_rst(void);
 
  /**
   *
   * @brief This is a callback function which collects all the data from the ECG, BIOZ, PACE and RtoR. It also handles
   * @brief Lead On/Off.  This  function is passed through the argument of max30003_COMMinit().
   * @returns 0-if no error.  A non-zero value indicates an error.
   *
   */
  int max30003_int_handler(void);
 
  /**
   * @brief This is function called from the max30003_int_handler() function and processes all the ECG, BIOZ, PACE
   * @brief and the RtoR data and sticks them in appropriate arrays and variables each unsigned 32 bits.
   * @param ECG data will be in the array (input): max30003_ECG_FIFO_buffer[]
   * @param Pace data will be in the array (input): max30003_PACE[]
   * @param RtoRdata will be in the variable (input): max30003_RtoR_data
   * @param BIOZ data will be in the array (input): max30003_BIOZ_FIFO_buffer[]
   * @param global  max30003_ECG_FIFO_buffer[]
   * @param global  max30003_PACE[]
   * @param global  max30003_BIOZ_FIFO_buffer[]
   * @param global  max30003_RtoR_data
   * @param global  max30003_DCLeadOff
   * @param global  max30003_ACLeadOff
   * @param global  max30003_LeadON
   * @returns 0-if no error.  A non-zero value indicates an error.
   *
   */
  int max30003_FIFO_LeadONOff_Read(void);

  /**
   * @brief This function transfers data to/from SPI
   * @param addr:  Address of the register to write
   * @param ocount:  Count of data to transmit/receive
   * @returns 0-if no error.  A non-zero value indicates an error.
   *
   */
  int32_t SPI_Transmit(const uint8_t *, uint32_t , uint8_t *, uint32_t );

 
  /**
   * @brief This function allows writing to a register.
   * @param addr:  Address of the register to write to
   * @param data:  24-bit data read from the register.
   * @returns 0-if no error.  A non-zero value indicates an error.
   *
   */
  int max30003_reg_write(MAX30003_REG_map_t , uint32_t );
 
  /**
   * @brief This function allows reading from a register
   * @param addr:   Address of the register to read from.
   * @param *return_data: pointer to the value read from the register.
   * @returns 0-if no error.  A non-zero value indicates an error.
   *
   */
  int max30003_reg_read(MAX30003_REG_map_t , uint32_t *);
 
  /**
   * @brief This function enables the DC Lead Off detection. Either ECG or BIOZ can be detected, one at a time.
   * @brief Registers Used:  CNFG_GEN-0x10
   * @param En_dcloff: BIOZ Digital Lead Off Detection Enable
   * @param Ipol: DC Lead Off Current Polarity (if current sources are enabled/connected)
   * @param Imag: DC Lead off current Magnitude Selection
   * @param Vth: DC Lead Off Voltage Threshold Selection
   * @returns 0-if no error.  A non-zero value indicates an error.
   *
   */
  int max30003_Enable_DcLeadOFF_Init(int8_t En_dcloff, int8_t Ipol, int8_t Imag,
                                     int8_t Vth);
 
  /**
   * @brief This function disables the DC Lead OFF feature, whichever is active.
   * @returns 0-if no error.  A non-zero value indicates an error.
   *
   */
  int max30003_Disable_DcLeadOFF(void);
 
  /**
   *
   * @brief This function enables the Lead ON detection. Either ECG or BIOZ can be detected, one at a time.
   * @brief Also, the en_bioz, en_ecg, en_pace setting is saved so that when this feature is disabled through the
   * @brief max30003_Disable_LeadON() function (or otherwise) the enable/disable state of those features can be retrieved.
   * @param Channel: ECG or BIOZ detection
   * @returns 0-if everything is good.  A non-zero value indicates an error.
   *
   */
  int max30003_Enable_LeadON(int8_t Channel);
 
  /**
   * @brief This function turns off the Lead ON feature, whichever one is active.  Also, retrieves the en_bioz,
   * @brief en_ecg, en_pace and sets it back to as it was.
   * @param 0-if everything is good.  A non-zero value indicates an error.
   *
   */
  int max30003_Disable_LeadON(void);
 
  /**
   *
   * @brief This function is toggled every 2 seconds to switch between ECG Lead ON and BIOZ Lead ON detect
   * @brief Adjust LEADOFF_SERVICE_TIME to determine the duration between the toggles.
   * @param CurrentTime - This gets fed the time by RTC_GetValue function
   *
   */
  void max30003_ServiceLeadON(uint32_t currentTime);
 
  /**
   *
   * @brief This function is toggled every 2 seconds to switch between ECG DC Lead Off and BIOZ DC Lead Off
   * @brief Adjust LEADOFF_SERVICE_TIME to determine the duration between the toggles.
   * @param CurrentTime - This gets fed the time by RTC_GetValue function
   *
   */
  void max30003_ServiceLeadoff(uint32_t currentTime);
 
   /**
   * @brief type definition for data interrupt
   */
  typedef void (*PtrFunction_t)(uint32_t id, uint32_t *buffer, uint32_t length);
 
  /**
   * @brief Used to connect a callback for when interrupt data is available
   */
  void onDataAvailable(PtrFunction_t onDataReady);
