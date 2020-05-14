#include <arduino.h>

#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <BluetoothSerial.h>

#include "..\include\permission.h"
#include "..\include\MAX30003.h"
#include "..\include\max30003_fns.h"
#include "..\include\esp32_rpc.h"
#include "..\include\esp32_fifo.h"
#include "..\include\esp32_streaming.h"
#include "..\include\esp32_helpers.h"


//******************************************************************************
int max30003_Rbias_FMSTR_Init(uint8_t En_rbias, uint8_t Rbiasv,
                                        uint8_t Rbiasp, uint8_t Rbiasn,
                                        uint8_t Fmstr) {
  if (max30003_reg_read(CNFG_GEN, &max30003_cnfg_gen.all) == -1) {
    return -1;
  }
 
  max30003_cnfg_gen.bit.en_rbias = En_rbias;
  max30003_cnfg_gen.bit.rbiasv   = Rbiasv;
  max30003_cnfg_gen.bit.rbiasp   = Rbiasp;
  max30003_cnfg_gen.bit.rbiasn   = Rbiasn;
  max30003_cnfg_gen.bit.fmstr    = Fmstr;
 
  if (max30003_reg_write(CNFG_GEN, max30003_cnfg_gen.all) == -1) {
    return -1;
  }
  return 0;
}
 
//******************************************************************************
int max30003_CAL_InitStart(uint8_t En_Vcal, uint8_t Vmode,
                                     uint8_t Vmag, uint8_t Fcal, uint16_t Thigh,
                                     uint8_t Fifty) {
  // CNFG_CAL
  if (max30003_reg_read(CNFG_CAL, &max30003_cnfg_cal.all) == -1) {
    return -1;
  }
 
  max30003_cnfg_cal.bit.vmode = Vmode;
  max30003_cnfg_cal.bit.vmag  = Vmag;
  max30003_cnfg_cal.bit.fcal  = Fcal;
  max30003_cnfg_cal.bit.thigh = Thigh;
  max30003_cnfg_cal.bit.fifty = Fifty;
 
  if (max30003_reg_write(CNFG_CAL, max30003_cnfg_cal.all) == -1) {
    return -1;
  }
 
  // RTOS uses a 32768HZ clock.  32768ticks represents 1secs.  1sec/10 =
  // 100msecs.
  delay(100);
 
  if (max30003_reg_read(CNFG_CAL, &max30003_cnfg_cal.all) == -1) {
    return -1;
  }
 
  max30003_cnfg_cal.bit.en_vcal = En_Vcal;
 
  if (max30003_reg_write(CNFG_CAL, max30003_cnfg_cal.all) == -1) {
    return -1;
  }
 
  // RTOS uses a 32768HZ clock.  32768ticks represents 1secs.  1sec/10 =
  // 100msecs.
  delay(100);
 
  return 0;
}
 
//******************************************************************************
int max30003_CAL_Stop(void) {
 
  if ((max30003_reg_read(CNFG_CAL, &max30003_cnfg_cal.all) == -1)) {
    return -1;
  }
 
  max30003_cnfg_cal.bit.en_vcal = 0; // Disable VCAL, all other settings are left unaffected
 
  if ((max30003_reg_write(CNFG_CAL, max30003_cnfg_cal.all) == -1)) {
    return -1;
  }
 
  return 0;
}
//******************************************************************************
//******************************************************************************
int max30003_INT_assignment(  max30003_intrpt_Location_t en_enint_loc,     max30003_intrpt_Location_t en_eovf_loc,  max30003_intrpt_Location_t en_fstint_loc,
                              max30003_intrpt_Location_t en_dcloffint_loc, max30003_intrpt_Location_t en_bint_loc,  max30003_intrpt_Location_t en_bovf_loc,
                              max30003_intrpt_Location_t en_bover_loc,     max30003_intrpt_Location_t en_bundr_loc, max30003_intrpt_Location_t en_bcgmon_loc,
                              max30003_intrpt_Location_t en_pint_loc,      max30003_intrpt_Location_t en_povf_loc,  max30003_intrpt_Location_t en_pedge_loc,
                              max30003_intrpt_Location_t en_lonint_loc,    max30003_intrpt_Location_t en_rrint_loc, max30003_intrpt_Location_t en_samp_loc,
                              max30003_intrpt_type_t  intb_Type,           max30003_intrpt_type_t int2b_Type) {
  // INT1
  int result;
  uint32_t data;
  result = max30003_reg_read(EN_INT, &data);
  if (result == -1) {
    return -1;
  }
 
  // max30003_en_int2.bit.en_pint       = 0b1;  // Keep this off...
 
  max30003_en_int.bit.en_eint = 0b1 & en_enint_loc;
  max30003_en_int.bit.en_eovf = 0b1 & en_eovf_loc;
  max30003_en_int.bit.en_fstint = 0b1 & en_fstint_loc;
 
  max30003_en_int.bit.en_dcloffint = 0b1 & en_dcloffint_loc;
  max30003_en_int.bit.en_bint = 0b1 & en_bint_loc;
  max30003_en_int.bit.en_bovf = 0b1 & en_bovf_loc;
 
  max30003_en_int.bit.en_bover = 0b1 & en_bover_loc;
  max30003_en_int.bit.en_bundr = 0b1 & en_bundr_loc;
  max30003_en_int.bit.en_bcgmon = 0b1 & en_bcgmon_loc;
 
  max30003_en_int.bit.en_pint = 0b1 & en_pint_loc;
  max30003_en_int.bit.en_povf = 0b1 & en_povf_loc;
  max30003_en_int.bit.en_pedge = 0b1 & en_pedge_loc;
 
  max30003_en_int.bit.en_lonint = 0b1 & en_lonint_loc;
  max30003_en_int.bit.en_rrint = 0b1 & en_rrint_loc;
  max30003_en_int.bit.en_samp = 0b1 & en_samp_loc;
 
  max30003_en_int.bit.intb_type = intb_Type;
 
  if ((max30003_reg_write(EN_INT, max30003_en_int.all) == -1)) {
    return -1;
  }
 
  // INT2
 
  if ((max30003_reg_read(EN_INT2, &max30003_en_int2.all) == -1)) {
    return -1;
  }
 
  max30003_en_int2.bit.en_eint   = 0b1 & (en_enint_loc >> 1);
  max30003_en_int2.bit.en_eovf   = 0b1 & (en_eovf_loc >> 1);
  max30003_en_int2.bit.en_fstint = 0b1 & (en_fstint_loc >> 1);
 
  max30003_en_int2.bit.en_dcloffint = 0b1 & (en_dcloffint_loc >> 1);
  max30003_en_int2.bit.en_bint      = 0b1 & (en_bint_loc >> 1);
  max30003_en_int2.bit.en_bovf      = 0b1 & (en_bovf_loc >> 1);
 
  max30003_en_int2.bit.en_bover  = 0b1 & (en_bover_loc >> 1);
  max30003_en_int2.bit.en_bundr  = 0b1 & (en_bundr_loc >> 1);
  max30003_en_int2.bit.en_bcgmon = 0b1 & (en_bcgmon_loc >> 1);
 
  max30003_en_int2.bit.en_pint  = 0b1 & (en_pint_loc >> 1);
  max30003_en_int2.bit.en_povf  = 0b1 & (en_povf_loc >> 1);
  max30003_en_int2.bit.en_pedge = 0b1 & (en_pedge_loc >> 1);
 
  max30003_en_int2.bit.en_lonint = 0b1 & (en_lonint_loc >> 1);
  max30003_en_int2.bit.en_rrint  = 0b1 & (en_rrint_loc >> 1);
  max30003_en_int2.bit.en_samp   = 0b1 & (en_samp_loc >> 1);
 
  max30003_en_int2.bit.intb_type = int2b_Type;
 
  if ((max30003_reg_write(EN_INT2, max30003_en_int2.all) == -1)) {
    return -1;
  }
 
  return 0;
}
 
//******************************************************************************
int max30003_ECG_InitStart(uint8_t En_ecg, uint8_t Openp,
                                     uint8_t Openn, uint8_t Pol,
                                     uint8_t Calp_sel, uint8_t Caln_sel,
                                     uint8_t E_fit, uint8_t Rate, uint8_t Gain,
                                     uint8_t Dhpf, uint8_t Dlpf) {
 
  // CNFG_EMUX
 
  if (max30003_reg_read(CNFG_EMUX, &max30003_cnfg_emux.all) == -1) {
    return -1;
  }
 
  max30003_cnfg_emux.bit.openp    = Openp;
  max30003_cnfg_emux.bit.openn    = Openn;
  max30003_cnfg_emux.bit.pol      = Pol;
  max30003_cnfg_emux.bit.calp_sel = Calp_sel;
  max30003_cnfg_emux.bit.caln_sel = Caln_sel;
 
  if (max30003_reg_write(CNFG_EMUX, max30003_cnfg_emux.all) == -1) {
    return -1;
  }
 
  /**** ENABLE CHANNELS ****/
  // CNFG_GEN
 
  if (max30003_reg_read(CNFG_GEN, &max30003_cnfg_gen.all) == -1) {
    return -1;
  }
 
  max30003_cnfg_gen.bit.en_ecg = En_ecg; // 0b1
 
  // fmstr is default
 
  if (max30003_reg_write(CNFG_GEN, max30003_cnfg_gen.all) == -1) {
    return -1;
  }
 
  /**** Wait for PLL Lock & References to settle down ****/
 
  max30003_timeout = 0;
 
  do {
    if (max30003_reg_read(STSREG, &max30003_status.all) == -1) // Wait and spin for PLL to lock...
    {
      return -1;
    }
  } while (max30003_status.bit.pllint == 1 && max30003_timeout++ <= 1000);
 
  // MNGR_INT
 
  if (max30003_reg_read(MNGR_INT, &max30003_mngr_int.all) == -1) {
    return -1;
  }
 
  max30003_mngr_int.bit.e_fit = E_fit; // 31
 
  if (max30003_reg_write(MNGR_INT, max30003_mngr_int.all) == -1) {
    return -1;
  }
 
  // CNFG_ECG
 
  if (max30003_reg_read(CNFG_ECG, &max30003_cnfg_ecg.all) == -1) {
    return -1;
  }
 
  max30003_cnfg_ecg.bit.rate = Rate; 
  max30003_cnfg_ecg.bit.gain = Gain;
  max30003_cnfg_ecg.bit.dhpf = Dhpf;
  max30003_cnfg_ecg.bit.dlpf = Dlpf;
 
  if (max30003_reg_write(CNFG_ECG, max30003_cnfg_ecg.all) == -1) {
    return -1;
  }
 
  return 0;
}
 
//******************************************************************************
int max30003_ECGFast_Init(uint8_t Clr_Fast, uint8_t Fast, uint8_t Fast_Th) {
  if (max30003_reg_read(MNGR_INT, &max30003_mngr_int.all) == -1) {
    return -1;
  }
 
  max30003_mngr_int.bit.clr_fast = Clr_Fast;
 
  if (max30003_reg_write(MNGR_INT, max30003_mngr_int.all) == -1) {
    return -1;
  }
 
  if (max30003_reg_read(MNGR_DYN, &max30003_mngr_dyn.all) == -1) {
    return -1;
  }
 
  max30003_mngr_dyn.bit.fast = Fast;
  max30003_mngr_dyn.bit.fast_th = Fast_Th;
 
  if (max30003_reg_write(MNGR_INT, max30003_mngr_int.all) == -1) {
    return -1;
  }
 
  return 0;
}
 
//******************************************************************************
int max30003_Stop_ECG(void) {
 
  if (max30003_reg_read(CNFG_GEN, &max30003_cnfg_gen.all) == -1) {
    return -1;
  }
 
  max30003_cnfg_gen.bit.en_ecg = 0; // Stop ECG
 
  // fmstr is default
 
  if (max30003_reg_write(CNFG_GEN, max30003_cnfg_gen.all) == -1) {
    return -1;
  }
 
  return 0;
}
 
//******************************************************************************
int max30003_RtoR_InitStart(uint8_t En_rtor, uint8_t Wndw,
                                      uint8_t Gain, uint8_t Pavg, uint8_t Ptsf,
                                      uint8_t Hoff, uint8_t Ravg, uint8_t Rhsf,
                                      uint8_t Clr_rrint) {
 
  // MNGR_INT
 
  if (max30003_reg_read(MNGR_INT, &max30003_mngr_int.all) == -1) {
    return -1;
  }
 
  max30003_mngr_int.bit.clr_rrint =
      Clr_rrint; // 0b01 & 0b00 are for interrupt mode...
  // 0b10 is for monitoring mode... it just overwrites the data...
 
  if (max30003_reg_write(MNGR_INT, max30003_mngr_int.all) == -1) {
    return -1;
  }
 
  // RTOR1
  if (max30003_reg_read(CNFG_RTOR1, &max30003_cnfg_rtor1.all) == -1) {
    return -1;
  }
 
  max30003_cnfg_rtor1.bit.wndw = Wndw;
  max30003_cnfg_rtor1.bit.gain = Gain;
  max30003_cnfg_rtor1.bit.en_rtor = En_rtor;
  max30003_cnfg_rtor1.bit.pavg = Pavg;
  max30003_cnfg_rtor1.bit.ptsf = Ptsf;
 
  if (max30003_reg_write(CNFG_RTOR1, max30003_cnfg_rtor1.all) == -1) {
    return -1;
  }
  // RTOR2
 
  if (max30003_reg_read(CNFG_RTOR2, &max30003_cnfg_rtor2.all) == -1) {
    return -1;
  }
  max30003_cnfg_rtor2.bit.hoff = Hoff;
  max30003_cnfg_rtor2.bit.ravg = Ravg;
  max30003_cnfg_rtor2.bit.rhsf = Rhsf;
 
  if (max30003_reg_write(CNFG_RTOR2, max30003_cnfg_rtor2.all) == -1) {
    return -1;
  }
 
  return 0;
}
 
//******************************************************************************
int max30003_Stop_RtoR(void) {
 
  if (max30003_reg_read(CNFG_RTOR1, &max30003_cnfg_rtor1.all) == -1) {
    return -1;
  }
 
  max30003_cnfg_rtor1.bit.en_rtor = 0; // Stop RtoR
 
  if (max30003_reg_write(CNFG_RTOR1, max30003_cnfg_rtor1.all) == -1) {
    return -1;
  }
 
  return 0;
}
 
//******************************************************************************
int max30003_PLL_lock(void) {
  // Spin to see PLLint become zero to indicate a lock.
 
  max30003_timeout = 0;
 
  do {
    if (max30003_reg_read(STSREG, &max30003_status.all) ==
        -1) // Wait and spin for PLL to lock...
    {
      return -1;
    }
 
  } while (max30003_status.bit.pllint == 1 && max30003_timeout++ <= 1000);
 
  return 0;
}
 
//******************************************************************************
int max30003_sw_rst(void) {
  // SW reset for the MAX30001 chip
 
  if (max30003_reg_write(SW_RST, 0x000000) == -1) {
    return -1;
  }
 
  return 0;
}
 
//******************************************************************************
int max30003_synch(void) { // For synchronization
  if (max30003_reg_write(SYNCH, 0x000000) == -1) {
    return -1;
  }
  return 0;
}
 
//******************************************************************************
int max300003_fifo_rst(void) { // Resets the FIFO
  if (max30003_reg_write(FIFO_RST, 0x000000) == -1) {
    return -1;
  }
  return 0;
}

//******************************************************************************
// int max30003_reg_write(uint8_t addr, uint32_t data)
int max30003_reg_write(MAX30003_REG_map_t addr, uint32_t data) {
 
  uint8_t result[4];
  uint8_t data_array[4];
  int32_t success = 0;
 
  data_array[3] = data & 0xff;
  data_array[2] = (data >> 8) & 0xff;
  data_array[1] = (data >> 16) & 0xff;
  data_array[0] = ((addr << 1) | WREG) & 0xff;
 
  success = SPI_Transmit(&data_array[0], 4, &result[0], 4);
 
  if (success != 0) {
    return -1;
  } else {
    return 0;
  }
}
 
//******************************************************************************
// int max30003_reg_read(uint8_t addr, uint32_t *return_data)
int max30003_reg_read(MAX30003_REG_map_t addr, uint32_t *return_data) {

  uint8_t result[4];
  uint8_t data_array[1];
  int32_t success = 0;
 
  data_array[0] = (((addr << 1) & 0xff) | RREG);

  success = SPI_Transmit(&data_array[0], 1, &result[0], 4);

  *return_data = /*result[0] + */ (uint32_t)(result[1] << 16) +
                 (result[2] << 8) + result[3];
  if (success != 0) return -1;
  else return 0;
}
 
//******************************************************************************
int max30003_Enable_DcLeadOFF_Init(int8_t En_dcloff, int8_t Ipol,
                                             int8_t Imag, int8_t Vth) {
  //  the leads are not touching the body
 
  // CNFG_EMUX, Set ECGP and ECGN for external hook up...
 
  if (max30003_reg_read(CNFG_GEN, &max30003_cnfg_gen.all) == -1) {
    return -1;
  }
 
  max30003_cnfg_gen.bit.en_dcloff = En_dcloff;
  max30003_cnfg_gen.bit.ipol = Ipol;
  max30003_cnfg_gen.bit.imag = Imag;
  max30003_cnfg_gen.bit.vth = Vth;
 
  if (max30003_reg_write(CNFG_GEN, max30003_cnfg_gen.all) == -1) {
    return -1;
  }
 
  return 0;
}
 
static uint8_t serialNumber[6];
uint8_t *MAX30001_Helper_getVersion(void) {
  // read the id
  max30003_reg_read(INFO, (uint32_t *)serialNumber);
  // read id twice because it needs to be read twice
  max30003_reg_read(INFO, (uint32_t *)serialNumber);
  return serialNumber;
}

//******************************************************************************
int max30003_Disable_DcLeadOFF(void) {
  if (max30003_reg_read(CNFG_GEN, &max30003_cnfg_gen.all) == -1) {
    return -1;
  }
 
  max30003_cnfg_gen.bit.en_dcloff = 0; // Turned off the dc lead off.
 
  if (max30003_reg_write(CNFG_GEN, max30003_cnfg_gen.all) == -1) {
    return -1;
  }
 
  return 0;
}
 
#if 1
//******************************************************************************
int max30003_Enable_LeadON(int8_t Channel) // Channel: ECG = 0b01, Disable = 0b00
{
 
  if (max30003_reg_read(CNFG_GEN, &max30003_cnfg_gen.all) == -1) {
    return -1;
  }
 
  max30003_cnfg_gen.bit.en_ecg  = 0b0;          // Disable ECG
  max30003_cnfg_gen.bit.en_ulp_lon = Channel;   // ULP lead on detection...
 
  if (max30003_reg_write(CNFG_GEN, max30003_cnfg_gen.all) == -1) {
    return -1;
  }
 
  max30003_reg_read(CNFG_GEN, &max30003_cnfg_gen.all); 
  max30003_reg_read(STSREG, &max30003_status.all);
 
  return 0;
}
//******************************************************************************
int max30003_Disable_LeadON(void) {
 
  if (max30003_reg_read(CNFG_GEN, &max30003_cnfg_gen.all) == -1) {
    return -1;
  }
 
  max30003_cnfg_gen.bit.en_ulp_lon = 0b0;
 
  if (max30003_reg_write(CNFG_GEN, max30003_cnfg_gen.all) == -1) {
    return -1;
  }
 
  return 0;
}
#endif
//******************************************************************************
#define LEADOFF_SERVICE_TIME 0x2000 // 0x1000 = 1 second
#define LEADOFF_NUMSTATES 2
uint32_t leadoffState = 0;
uint32_t max30003_LeadOffoldTime = 0;
void max30003_ServiceLeadoff(uint32_t currentTime) {
 
  uint32_t delta_Time;
 
  delta_Time = currentTime - max30003_LeadOffoldTime;
 
  if (delta_Time > LEADOFF_SERVICE_TIME) {
    switch (leadoffState) {
    case 0: /* switch to ECG DC Lead OFF */
      max30003_Enable_DcLeadOFF_Init(0b01, 0b0, 0b001, 0b00);
      break;
    }
 
    leadoffState++;
    leadoffState %= LEADOFF_NUMSTATES;
 
    max30003_LeadOffoldTime = currentTime;
  }
}

//******************************************************************************
#define LEADON_SERVICE_TIME 0x2000 // 0x1000 = 1 second
#define LEADON_NUMSTATES 2
uint32_t leadOnState = 0;
uint32_t max30003_LeadOnoldTime = 0;
void max30003_ServiceLeadON(uint32_t currentTime) {
 
  uint32_t delta_Time;
 
  delta_Time = currentTime - max30003_LeadOnoldTime;
 
  if (delta_Time > LEADON_SERVICE_TIME) {
    switch (leadOnState) {
    case 0: /* switch to ECG DC Lead ON */
      max30003_Enable_LeadON(0b01);
      break;
    }
 
    leadOnState++;
    leadOnState %= LEADON_NUMSTATES;
 
    max30003_LeadOnoldTime = currentTime;
  }
}
 
//******************************************************************************
int max30003_int_handler(void) {
 
  static uint32_t InitReset = 0;
 
  int8_t return_value;
  bool check_one_more = true;
 
  status_check:
  max30003_reg_read(STSREG, &max30003_status.all);
 
  // Inital Reset and any FIFO over flow invokes a FIFO reset
  if (InitReset == 0 || max30003_status.bit.eovf == 1) {
    max30003_reg_write(FIFO_RST, 0x000000);    // Do a FIFO Reset
    InitReset++;
    return 2;
  }
  return_value = 0;

  if (max30003_status.bit.eint == 1 || max30003_status.bit.rrint == 1
    || max30003_status.bit.dcloffint == 1 || max30003_status.bit.lonint == 1) {
    return_value = return_value | max30003_FIFO_LeadONOff_Read();
  }
  if (check_one_more) {
    check_one_more = false;
    goto status_check;
  }
  return return_value;
}

/**
* @brief Transmit and recieve using esp32 SPI_Transmit()
* @param tx_buf pointer to transmit byte buffer
* @param tx_size number of bytes to transmit
* @param rx_buf pointer to the recieve buffer
* @param rx_size number of bytes to recieve
*/
int32_t SPI_Transmit(const uint8_t *tx_buf, uint32_t tx_size, uint8_t *rx_buf, uint32_t rx_size)
{
  SPI.beginTransaction(SPISettings(SPI_CLOCK_DIV4, MSBFIRST, SPI_MODE0));
  digitalWrite(MAX30003_CS_PIN, LOW);             // CS Low
  for(int i = 0; i < tx_size; i++) {
    SPI.transfer(tx_buf[i]);
  }
  for(int i = 0; i < (rx_size - tx_size); i++) {
      rx_buf[i + 1] = SPI.transfer(0xFF);
  }
  digitalWrite(MAX30003_CS_PIN, HIGH);            // CS High
  SPI.endTransaction();
  return 0;
}

//******************************************************************************
void onDataAvailable(PtrFunction_t onDataReadyPtr) {
  onDataAvailableCallback = onDataReadyPtr;
}

/**
* @brief Used to notify an external function that interrupt data is available
* @param id type of data available
* @param buffer 32-bit buffer that points to the data
* @param length length of 32-bit elements available
*/
void dataAvailable(uint32_t id, uint32_t *buffer, uint32_t length) {
  onDataAvailable(StreamPacketUint32);                    // Set onDataAvailableCallback to StreamPacketUint32(...);
  if (onDataAvailableCallback != NULL) {
    (*onDataAvailableCallback)(id, buffer, length);
  }
}

/**
* @brief Callback handler for SPI async events
* @param events description of event that occurred
*/

/* jma
void spiHandler(int events) { xferFlag = 1; }
//static volatile int xferFlag = 0;                         // flag used to indicate an async xfer has taken place

//******************************************************************************
static int allowInterrupts = 0;
 
void MAX30001Mid_IntB_Handler(void) {
  if (allowInterrupts == 0) return;
  instance->max30003_int_handler();
}
 
void MAX30001Mid_Int2B_Handler(void) {
  if (allowInterrupts == 0) return;
  instance->max30003_int_handler();
}
 
void MAX30001_AllowInterrupts(int state) { 
allowInterrupts = state; 
}

*/
