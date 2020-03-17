/*-----------------------------------------------------------------------
  Copyright (c) 2012-2015 Qualcomm  Technologies, Inc.  All Rights Reserved.
  Qualcomm Technologies Proprietary and Confidential.
  -----------------------------------------------------------------------*/

#ifndef TFADSP_COMMON_H_
#define TFADSP_COMMON_H_

// #include "tfadsp.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

	/************************  NXP Intergration Option  **************************/
#define TFA_INTEGRATION_VERSION   17050901

	/* MCPS_PROFILING */
	/* Check every 1sec */
	/* MCPS = ( frame_for_second * max cycles ) / ( 2 * 1000000 ) : provided Qualcomm, based on max/peak cycles */
	/* MCPS with Avg cycles = ( frame_for_second * average cycles ) / ( 2 * 1000000 ) to compare with average */
	//#define PERFORMANCE_PROFILING

	/*****************************************************************************/

#define TOPOLOGY_TFADSP_ID						0x1000B910
#define MODULE_TFADSP_RX                  		0x1000B911
#define MODULE_TFADSP_TX                   		0x1000B912

#define TFADSP_ENABLE  							0x1000B920
#define TFADSP_RX_SET_COMMAND 					0x1000B921
#define TFADSP_RX_GET_RESULT 					0x1000B922

	/* Begin ACDB parameter list */
#define TFADSP_FWK_SetInputSelector  			0x1000B930
#define TFADSP_FWK_SetOutputSelector  			0x1000B931
#define TFADSP_FWK_SetProgramConfig  			0x1000B932
#define TFADSP_SB_SetLagW  						0x1000B933
#define TFADSP_SB_SetLsModel  					0x1000B934
#define TFADSP_SB_SetAlgoParams  				0x1000B935
#define TFADSP_MBDRC_SetMBDrc  					0x1000B936
#define TFADSP_BQFB_SetCoeffs  					0x1000B937
#define TFADSP_SB_SetExcursionFilters  			0x1000B938
#define TFADSP_FWK_SetGains  					0x1000B939
#define TFADSP_FWK_SetSensesDelay  				0x1000B93a
#define TFADSP_FWK_SetSensesCal  				0x1000B93b
#define TFADSP_FWK_SetvBatFactors  				0x1000B93c
#define TFADSP_FWK_SetAmplifierEq				0x1000B93d
#define TFADSP_FWK_SetHWConfig					0x1000B93e
#define TFADSP_SB_SetRe25C  					0x1000B93f
	/* End ACDB parameter list */

#define TFADSP_FWK_GetTag  						0x1000B940
#define TFADSP_SB_GetRe25C  					0x1000B941
#define TFADSP_SB_SetMemTrack  					0x1000B942
#define TFADSP_SB_GetMemTrack  					0x1000B943
#define TFADSP_SB_ResetRe25C  					0x1000B944
#define TFADSP_SB_SetMuteOff  					0x1000B945
#define TFADSP_SB_SetMuteOn  					0x1000B946

#define PROCESS_1MS_FRAME_NUMBER 48

#define TFADSP_RX_MAX_KPPS 60000
#define TFADSP_RX_MIN_KPPS 140000// 40000

	typedef struct tfadsp_tx_enable_cfg_t
	{
		uint32_t enable_flag;
		/**< Enable flag: 0 = disabled; 1 = enabled. */
	} tfadsp_tx_enable_cfg_t;

	typedef struct tfadsp_rx_enable_cfg_t
	{
		uint32_t enable_flag;
		/**< Enable flag: 0 = disabled; 1 = enabled. */
	} tfadsp_rx_enable_cfg_t;


#ifdef __cplusplus
}
#endif //__cplusplus

#endif /* TFADSP_COMMON_H_ */
