
// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the VCI_CAN_EXPORTS
// symbol defined on the command line. this symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// VCI_CAN_API functions as being imported from a DLL, wheras this DLL sees symbols
// defined with this macro as being exported.
#ifdef VCI_CAN_EXPORTS
#define _EXP __declspec(dllexport)
#else
#define _EXP __declspec(dllimport)
#endif

// This class is exported from the VCI_CAN.dll
class _EXP CVCI_CAN{
  public:
	CVCI_CAN(void);
	// TODO: add your methods here.
};

/***  For BCB ***/
#ifdef	__cplusplus  // for C++ compile use
extern "C" {
#endif	/*	__cplusplus	*/





/* Common Return Code */
#define No_Err						0	//No Error
#define DEV_ModName_Err				1	//The Module Name Error
#define DEV_ModNotExist_Err			2	//The Module doesn't exist in this Port
#define DEV_PortNotExist_Err		3	//The Port doesn't Exist
#define DEV_PortInUse_Err			4	//The Port is in Used
#define DEV_PortNotOpen_Err			5	//The Port doesn't Open
#define CAN_ConfigFail_Err			6	//CAN Config Command Fail
#define CAN_HARDWARE_Err			7	//CAN Hardware Init Fail
#define CAN_PortNo_Err				8	//The Device doesn't support this CAN Port
#define CAN_FIDLength_Err			9	//The CAN Filter-ID Number exceed Max Number
#define CAN_DevDisconnect_Err		10	//The Connection of device is broken
#define CAN_TimeOut_Err				11	//The Config Command Timeout
#define CAN_ConfigCmd_Err			12	//The Config Command doesn't support
#define CAN_ConfigBusy_Err			13	//The Config Command is busy
#define CAN_RxBufEmpty				14	//The CAN Receive Buffer is empty
#define CAN_TxBufFull				15	//The CAN Send Buffer is full
#define CAN_UserDefISRNo_Err		16  //The User Defined ISR No Error (0~7)
#define CAN_HWSendTimerNo_Err		17  //The Hardware SendTimer No. Error (0~4)



/************************      VCI_CAN API Variable      *************************/ 
//Common Define
#define CAN1						1
#define CAN2						2
#define CONFIG_MODE					0
#define COMM_MODE					1
#define TPFLOATMS(fVal)				((DWORD)(fVal*10)+100000000)

//Device Type Define
#define I7565H1						1
#define I7565H2						2

//CANMsg Define
#define MODE_11BIT					0
#define MODE_29BIT					1
#define RTR_0						0
#define RTR_1						1
#define DLC_0						0
#define DLC_1						1
#define DLC_2						2
#define DLC_3						3
#define DLC_4						4
#define DLC_5						5
#define DLC_6						6
#define DLC_7						7
#define DLC_8						8

//ADDMODE
#define ADDITION_MODE				1
#define MULTIPLE_MODE				2

//UserDefISR Define
#define ISRNO_0						0
#define ISRNO_1						1
#define ISRNO_2						2
#define ISRNO_3						3
#define ISRNO_4						4
#define ISRNO_5						5
#define ISRNO_6						6
#define ISRNO_7						7
#define ISR_CANPORT_ALL				0
#define ISR_CANMODE_ALL				2
#define ISR_CANID_ALL				0
	
//CAN Baudrate
#define CANBaud_5K					5000
#define CANBaud_10K					10000
#define CANBaud_20K					20000
#define CANBaud_40K					40000
#define CANBaud_50K					50000
#define CANBaud_80K					80000
#define CANBaud_100K				100000
#define CANBaud_125K				125000
#define CANBaud_200K				200000
#define CANBaud_250K				250000
#define CANBaud_400K				400000
#define CANBaud_500K				500000
#define CANBaud_600K				600000
#define CANBaud_800K				800000
#define CANBaud_1000K				1000000


extern _EXP char* ErrMessage[32];
extern _EXP DWORD CANBaud_Arr[15];


/* ICPDAS VCICAN Series Message Structure Format */
//[1]. VCICAN - Module Info Format
typedef  struct  _VCI_MODULE_INFO{
	char	Mod_ID[12];
	char	FW_Ver[12];
	char	HW_SN[16];
} _VCI_MODULE_INFO, *PVCI_MOD_INFO; 

//[2]. VCICAN - CAN Parameter (CANBaud: Unit(bps))
typedef  struct  _VCI_CAN_PARAM{
	BYTE	DevPort;
	BYTE	DevType;
	DWORD	CAN1_Baud;
	DWORD	CAN2_Baud;
} _VCI_CAN_PARAM, *PVCI_CAN_PARAM;

//[3]. VCICAN - CAN Message Format
typedef  struct  _VCI_CAN_MSG{
	BYTE	Mode;		
	BYTE	RTR;
	BYTE	DLC;
	BYTE	Reserved;
	DWORD	ID;
	DWORD	TimeL;
	DWORD	TimeH;
	BYTE	Data[8];
} _VCI_CAN_MSG, *PVCI_CAN_MSG;

//[4]. VCICAN - CAN FilterID Format
typedef  struct  _VCI_CAN_FilterID{
	WORD	SSFF_Num;		
	WORD	GSFF_Num;
	WORD	SEFF_Num;
	WORD	GEFF_Num;
	WORD	SSFF_FID[512];
	DWORD	GSFF_FID[512];
	DWORD	SEFF_FID[512];
	DWORD	GEFF_FID[512];
} _VCI_CAN_FilterID, *PVCI_CAN_FID;

//[5]. VCICAN - CAN Status Format
typedef struct _VCI_CAN_STATUS{
	DWORD	CurCANBaud;
	BYTE	CANReg;
	BYTE	CANTxErrCnt;
	BYTE	CANRxErrCnt;
	BYTE	MODState;
	DWORD	Reserved;
} _VCI_CAN_STATUS, *PVCI_CAN_STATUS;



/************************      VCI_CAN API Function      *************************/ 
/********************     [1] Init Function      ********************/
_EXP	int __stdcall VCI_OpenCAN(PVCI_CAN_PARAM pCANPARAM);
_EXP	int __stdcall VCI_OpenCAN_NoStruct(BYTE	DevPort, BYTE DevType, 
										   DWORD CAN1_Baud, DWORD CAN2_Baud);
_EXP	int __stdcall VCI_CloseCAN(BYTE DevPort);


/********************     [2] Module Config Function     ********************/
_EXP	int __stdcall VCI_Set_CANFID(BYTE CAN_No, PVCI_CAN_FID pCANFID);
_EXP	int __stdcall VCI_Set_CANFID_NoStruct(BYTE CAN_No, WORD SSFF_Num, WORD GSFF_Num, WORD SEFF_Num, WORD GEFF_Num, 
											  WORD SSFF_FID[512], DWORD GSFF_FID[512], DWORD SEFF_FID[512], DWORD GEFF_FID[512]);
_EXP	int __stdcall VCI_Set_CANFID_AllPass(BYTE CAN_No);
_EXP	int __stdcall VCI_Get_CANFID(BYTE CAN_No, PVCI_CAN_FID pCANFID);
_EXP	int __stdcall VCI_Get_CANFID_NoStruct(BYTE CAN_No, WORD* SSFF_Num, WORD* GSFF_Num, WORD* SEFF_Num, WORD* GEFF_Num, 
											  WORD SSFF_FID[512], DWORD GSFF_FID[512], DWORD SEFF_FID[512], DWORD GEFF_FID[512]);
_EXP	int __stdcall VCI_Get_CANStatus(BYTE CAN_No, PVCI_CAN_STATUS pCANStatus);
_EXP	int __stdcall VCI_Get_CANStatus_NoStruct(BYTE CAN_No, DWORD* CurCANBaud, BYTE* CANReg, BYTE* CANTxErrCnt, BYTE*	CANRxErrCnt, 
												 BYTE* MODState);
_EXP	int __stdcall VCI_Clr_BufOverflowLED(BYTE CAN_No);
_EXP	int __stdcall VCI_Get_MODInfo(PVCI_MOD_INFO pMODInfo);
_EXP	int __stdcall VCI_Get_MODInfo_NoStruct(char Mod_ID[12], char FW_Ver[12], char HW_SN[16]);
_EXP	int __stdcall VCI_Rst_MOD(void);


/********************     [3] Communication Function     ********************/
_EXP	int __stdcall VCI_SendCANMsg(BYTE CAN_No, PVCI_CAN_MSG pCANMsg);
_EXP	int __stdcall VCI_SendCANMsg_NoStruct(BYTE CAN_No, BYTE	Mode, BYTE RTR, BYTE DLC, 
											  DWORD ID, BYTE Data[8]);
_EXP	int __stdcall VCI_RecvCANMsg(BYTE CAN_No, PVCI_CAN_MSG pCANMsg);
_EXP	int __stdcall VCI_RecvCANMsg_NoStruct(BYTE CAN_No, BYTE* Mode, BYTE* RTR, BYTE* DLC, 
											  DWORD* ID, DWORD* TimeL, DWORD* TimeH, BYTE Data[8]);
_EXP	int __stdcall VCI_EnableHWCyclicTxMsg(BYTE CAN_No, PVCI_CAN_MSG pCANMsg, DWORD TimePeriod, DWORD TransmitTimes);
_EXP	int __stdcall VCI_EnableHWCyclicTxMsg_NoStruct(BYTE CAN_No, BYTE Mode, BYTE RTR, BYTE DLC, DWORD ID, 
													   BYTE Data[8], DWORD TimePeriod, DWORD TransmitTimes);
_EXP	int __stdcall VCI_DisableHWCyclicTxMsg(void);
_EXP	int __stdcall VCI_EnableHWCyclicTxMsgNo(BYTE CAN_No, BYTE Mode, BYTE RTR, BYTE DLC, DWORD ID, 
												BYTE Data[8], DWORD TimePeriod, DWORD TransmitTimes, BYTE HW_TimerNo);
_EXP	int __stdcall VCI_EnableHWCyclicTxMsgNo_Ex(BYTE CAN_No, BYTE Mode, BYTE RTR, BYTE DLC, DWORD ID, 
													BYTE Data[8], DWORD TimePeriod, DWORD TransmitTimes, BYTE HW_TimerNo,
													BYTE AddMode, DWORD DLAddVal, DWORD DHAddVal);
_EXP	int __stdcall VCI_DisableHWCyclicTxMsgNo(BYTE HW_TimerNo);

/********************     [4] Software Buffer Function     ********************/
_EXP	int __stdcall VCI_Get_RxMsgCnt(BYTE CAN_No, DWORD* RxMsgCnt);
_EXP	int __stdcall VCI_Get_RxMsgBufIsFull(BYTE CAN_No, BYTE* Flag);
_EXP	int __stdcall VCI_Clr_RxMsgBuf(BYTE CAN_No);


/********************     [5] UserDefine ISR Function     ********************/
_EXP	int __stdcall VCI_Set_UserDefISR(BYTE ISRNo, BYTE CAN_No, BYTE Mode, DWORD CANID, void (*UserDefISR)());
_EXP	int __stdcall VCI_Clr_UserDefISR(BYTE ISRNo);
_EXP	int __stdcall VCI_Get_ISRCANData(BYTE ISRNo, BYTE* DLC, BYTE Data[8]);


/********************     [6] Other Function     ********************/
_EXP	DWORD __stdcall VCI_Get_DllVer(void);
_EXP	void  __stdcall VCI_DoEvents(void);





/***  For BCB ***/
#ifdef	__cplusplus
}
#endif	/*	__cplusplus	*/

