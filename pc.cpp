/*
 * File : pc.cpp
 * @brief: CAN bus communication PC/microcontroller, PC side
 * s-function used is Simulink Scheme
 */


#define S_FUNCTION_NAME  pc
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <windows.h>
#include "stdafx.h"
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <io.h>
#include <string.h>
#include <time.h>
#include "VCI_CAN.h"
#include "definitions.h"

/* structure used for CAN bus communication settings*/
_VCI_CAN_PARAM pCAN;  

/* virtual COM port number */
int port = 2; 

/* PC Frequency */
double PCFreq = 0.0;

/* variable used as counter */
__int64 CounterStart = 0;

/* variable used to check if this is the first execution */
bool first = true; 

/* counter variable */
DWORD conta = 0;

/* variable used to send packets at a lower frequency */
int low_frequency=0; 

/*================*
 * Build checking *
 *================*/

#ifndef  MATLAB_MEX_FILE    
#define ssSetErrorStatus(A,B) printf(B)
#endif

/**
* @brief: function used to count time
* @params: None
* @retval: None
*/
void StartCounter()
{
    LARGE_INTEGER li;
    if(!QueryPerformanceFrequency(&li))
       printf("QueryPerformanceFrequency failed!\n");

    PCFreq = double(li.QuadPart);  //counts/s

    QueryPerformanceCounter(&li);
    CounterStart = li.QuadPart;
}

/**
* @brief: function used to compute difference in time 
* @params: None
* @retval: double (difference variable)
*/
double GetCounter()
{
    LARGE_INTEGER li;
    QueryPerformanceCounter(&li);
    return double(li.QuadPart-CounterStart)/PCFreq;
}

/**
* @brief: function used to compute time elapsed from start
* @params: start variable 
* @retval: time elapsed 
*/
int timeout_old(clock_t start){
    
    /* Time elapsed (in seconds) */
    return ((clock()-start)/CLOCKS_PER_SEC); 
}

/**
* @brief: function used to compute time elapsed from start
* @params: start variable 
* @retval: time elapsed
*/
int timeout(double start){
    
    /* Time elapsed (in seconds) */
    return (GetCounter()-start); 
}


/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *   Setup sizes of the various vectors.
 */
static void mdlInitializeSizes(SimStruct *S)
{
    int_T i;

    ssSetNumSFcnParams(S,0);

    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch will be reported by Simulink */
    }

    if (!ssSetNumInputPorts(S, 9)) return;
    
    for(i = 0;i < 9; i++){
      
          if(i < 3){

             ssSetInputPortDataType(S, i, SS_UINT8);
             ssSetInputPortWidth(S, i, 8);

          }
          else if((i == 6)||(i == 3)){

              ssSetInputPortDataType(S, i,SS_UINT32);
              ssSetInputPortWidth(S, i, 2);

          }
          else{     

              ssSetInputPortDataType(S, i,SS_UINT16);
              ssSetInputPortWidth(S, i, 4);

          }
          ssSetInputPortDirectFeedThrough(S, i, 1);
          ssSetInputPortRequiredContiguous(S,i,1); 
    }

    if (!ssSetNumOutputPorts(S,4)) return;
    ssSetOutputPortWidth(S, 0, 4);
    ssSetOutputPortWidth(S, 1, 1);
    ssSetOutputPortWidth(S, 2, 1);
    ssSetOutputPortWidth(S, 3, 4);
    ssSetOutputPortDataType(S, 0, SS_UINT16);
    ssSetOutputPortDataType(S, 1, SS_DOUBLE);
    ssSetOutputPortDataType(S, 2, SS_DOUBLE);
    ssSetOutputPortDataType(S, 3, SS_UINT16);
    ssSetNumSampleTimes(S, 1);

}

static void mdlInitializeSampleTimes(SimStruct *S)
{
        /* freq hp 100 Hz => T = 0.01 */
        ssSetSampleTime(S,0,0.01);                           
        ssSetOffsetTime(S,0,0.0);
}

/* Change to #undef to remove function */
#define MDL_START  
#if defined(MDL_START) 

/*
* @brief: function executed the first time
* @param: struct for S-function
* @retvalue: None
*/
static void mdlStart(SimStruct *S)
{
     /* variable used to check errors */
     int res;

     /* counter variable initialization */
     conta = 0;
     
     /**
      * CAN port configuration
     **/

     /* CAN device port */
     pCAN.DevPort = 1;
     /* I-7565-H2 type */
     pCAN.DevType = I7565H2;  
     /* CAN1 BaudRate */
     pCAN.CAN1_Baud = CANBaud_1000K;
     /* CAN2 BaudRate */
     pCAN.CAN2_Baud = CANBaud_1000K;
     
     /* variable used to check if CAN port configuration is OK */
     res = VCI_OpenCAN(&pCAN);
     
     /* error check */
     if(res > 0){
         ssSetErrorStatus(S, "Error while opening port\n");
     }
    
     /* clear Rx buffer */
     res = VCI_Clr_RxMsgBuf(port);
   
     /* error check */
     if(res > 0){
     	 ssSetErrorStatus(S, "Buffer nnot cleared correctly\n");
     }
     
     /* check if Rx buffer has been cleared entirely */
     res = VCI_Get_RxMsgCnt(port,&conta);

     /* error check */
     if(res > 0){
     	 ssSetErrorStatus(S, "Error in VCI_Get_RxMsgCnt\n");
     }
    
     /* we start counting from here */
     StartCounter();
    
     /* this is the first time we execute */
     first = true;
     
     #ifndef MATLAB_MEX_FILE
     //    //SetPriorityClass(GetCurrentProcess(),HIGH_PRIORITY_CLASS);
     //SetPriorityClass(GetCurrentProcess(),REALTIME_PRIORITY_CLASS);
     //SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
     #endif
     
}
#endif


/* Function: mdlOutputs =======================================================
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{ 
      /* res: variable used to check errors
           i: variable used in 'for' loops 
      */
      int res,i ;
      /* variable used for synchronization */
      bool is_ok = false;  
      
      /* packets configuration to send data */

      /* dates for gyroscope and Ax(accelerometer)  ID: 0x60F */
      Packet_gyro Inerz1;   
      /* dates for accelerometer Ay, Az and Magnetometer Mx,My   ID: 0x40F */
      Packet_accel_magn Inerz2; 
      /* using the first part only (16 bit) to send Mz and a floating point number for pression data  ID:0x30F */  
      Packet_magn_press Inerz3;   
      /* Packet used for GPS data */
      Packet_GPS Inerz4;
      /* used for GPS data: two floating point numbers. ID: 0x20F */
      Packet_rc1 Inerz5;      
      /* used for GPS data: one floating point number. ID: 0x10F */
      Packet_rc2 Inerz6;      
      
      /* packets not sent */
      Packet_rc3 Inerz7;
      Packet_GPS2 Inerz8;
      Packet_ADC  Inerz9;
          
      /* variable used to send messages */
      _VCI_CAN_MSG CANRx;
      /* memory allocation configuration */
      memset(&CANRx,0,sizeof(CANRx));
      
      /* time variables */
      double time1, seconds;
      
      /* counter variable */
      conta = 0;
      
      /* timeout is 20 seconds */
      
      /* input ports from Simulink Scheme configuration */
      char_T *u1 =(char_T*) ssGetInputPortSignal(S,0);
      char_T *u2 = (char_T*)ssGetInputPortSignal(S,1);
      char_T *u3 = (char_T*)ssGetInputPortSignal(S,2);
      char_T *u4 = (char_T*)ssGetInputPortSignal(S,3);  
      char_T *u5 = (char_T*)ssGetInputPortSignal(S,4);  
      char_T *u6 = (char_T*)ssGetInputPortSignal(S,5);
      char_T *u7 = (char_T*)ssGetInputPortSignal(S,6);
      char_T *u8 = (char_T*)ssGetInputPortSignal(S,7);
      char_T *u9 = (char_T*)ssGetInputPortSignal(S,8);
      
      /* output ports from Simulink Scheme configuration*/
      uint16_T *y1 = (uint16_T*) ssGetOutputPortSignal(S,0); //uscite
      double *y2 = (double*) ssGetOutputPortSignal(S,1);
      uint16_T *y3 = (uint16_T*) ssGetOutputPortSignal(S,2);
      uint16_T *y4= (uint16_T*) ssGetOutputPortSignal(S,3);
   
      /* memory allocation configuration */
      memset(&CANRx,0,sizeof(CANRx));
      
      /* packet Inerz1 configuration */

      /* 11-bit ID */
      Inerz1.Mode=0; 
      /* No-RTR */
      Inerz1.RTR=0;
      /* 8 bytes to send */ 
      Inerz1.DLC=8; 
      /* 1551 value in hex format */
      Inerz1.ID=0x60F;  
      /* Data[0],Data[1]: Gx */
      Inerz1.Data[0]=*u1;
      Inerz1.Data[1]=*(u1+1);
      /* Data[2],Data[3]: Gy */
      Inerz1.Data[2]=*(u1+2);
      Inerz1.Data[3]=*(u1+3);
      /* Data[4],Data[5]: Gz */
      Inerz1.Data[4]=*(u1+4);
      Inerz1.Data[5]=*(u1+5);
      /* Data[6],Data[7]: Ax */
      Inerz1.Data[6]=*(u1+6);
      Inerz1.Data[7]=*(u1+7);
        
      /* packet Inerz2 configuration */

      /* 11-bit ID */
      Inerz2.Mode=0;
      /* No-RTR */
      Inerz2.RTR=0;
      /* 8 bytes to send */
      Inerz2.DLC=8;
      /* 1039 value in hex format */
      Inerz2.ID=0x40F; 
      /* Data[0],Data[1]:Ay */
      Inerz2.Data[0]=*u2;
      Inerz2.Data[1]=*(u2+1);
      /* Data[2],Data[3]: Az */
      Inerz2.Data[2]=*(u2+2);
      Inerz2.Data[3]=*(u2+3);
      /* Data[4],Data[5]:Mx */
      Inerz2.Data[4]=*(u2+4);
      Inerz2.Data[5]=*(u2+5);
      /* Data[6],Data[7]:My */
      Inerz2.Data[6]=*(u2+6);
      Inerz2.Data[7]=*(u2+7);

      /* packet Inerz3 configuration */

      /* 11-bit ID */
      Inerz3.Mode=0;
      /* No-RTR */
      Inerz3.RTR=0;
      /* 8 bytes to send */
      Inerz3.DLC=8;
      /* 783 value in hex format */
      Inerz3.ID=0x30F;  
      /* Data[0],Data[1]:Mz */
      Inerz3.Data[0]=*u3;
      Inerz3.Data[1]=*(u3+1);
      /* Data[2],Data[3],Data[4],Data[5]: pression data (float1) */
      Inerz3.Data[2]=*(u3+2);
      Inerz3.Data[3]=*(u3+3);
      Inerz3.Data[4]=*(u3+4);
      Inerz3.Data[5]=*(u3+5);
      /* other bytes can be used for fixmode data */
      Inerz3.Data[6]=0x00;
      Inerz3.Data[7]=0x00;
    
      /* packet Inerz4 configuration */

      /* 11-bit ID */
      Inerz4.Mode=0;
      /* No-RTR */
      Inerz4.RTR=0;
      /* 8 bytes to send */
      Inerz4.DLC=8;
      /* 527 value in hex format */
      Inerz4.ID=0x20F;   
      /* Data[0],Data[1],Data[2],Data[3]: float2 (Latitude) (GPS) */
      Inerz4.Data[0]=*u4;
      Inerz4.Data[1]=*(u4+1);
      Inerz4.Data[2]=*(u4+2);
      Inerz4.Data[3]=*(u4+3);
      /* Data[4],Data[5],Data[6],Data[7]: float3 (Longitude) (GPS) */
      Inerz4.Data[4]=*(u4+4);
      Inerz4.Data[5]=*(u4+5);
      Inerz4.Data[6]=*(u4+6);
      Inerz4.Data[7]=*(u4+7);
    
      /* packet Inerz5 configuration */

      /* 11-bit ID */
      Inerz5.Mode=0;
      /* No-RTR */
      Inerz5.RTR=0;
      /* 8 bytes to send */
      Inerz5.DLC=8;
      /* 271 value in hex format */
      Inerz5.ID=0x10F;  
      /* Data[0],Data[1]: uint16 (PWM) */
      Inerz5.Data[0]=*u5;
      Inerz5.Data[1]=*(u5+1);
      /* Data[2],Data[3]: uint16 (PWM) */
      Inerz5.Data[2]=*(u5+2);
      Inerz5.Data[3]=*(u5+3);
      /* Data[4],Data[5]: uint16 (PWM) */
      Inerz5.Data[4]=*(u5+4);
      Inerz5.Data[5]=*(u5+5);
      /* Data[6],Data[7]: uint16 (PWM) */
      Inerz5.Data[6]=*(u5+6);
      Inerz5.Data[7]=*(u5+7);
    
      /* packet Inerz6 configuration */

      /* 11-bit ID */
      Inerz6.Mode=0;
      /* No-RTR */
      Inerz6.RTR=0;
      /* 8 bytes to send */
      Inerz6.DLC=8;
      /* 1807 value in hex format */
      Inerz6.ID=0x70F;  
      /* Data[0],Data[1]: uint16 (PWM) */
      Inerz6.Data[0]=*u6;
      Inerz6.Data[1]=*(u6+1);
      /* Data[2],Data[3]: uint16 (PWM) */ 
      Inerz6.Data[2]=*(u6+2);
      Inerz6.Data[3]=*(u6+3);
      /* Data[4],Data[5]: uint16 (PWM) */
      Inerz6.Data[4]=*(u6+4);
      Inerz6.Data[5]=*(u6+5);
      /* Data[6],Data[7]: uint16 (PWM) */
      Inerz6.Data[6]=*(u6+6);
      Inerz6.Data[7]=*(u6+7);
    
      /* packet Inerz7 configuration */

      /* 11-bit ID */
      Inerz7.Mode=0;
      /* No-RTR */
      Inerz7.RTR=0;
      /* 8 bytes to send */
      Inerz7.DLC=8;
      /* 31 value in hex format */
      Inerz7.ID=0x01F;
      /* Data[0],Data[1],Data[2],Data[3]: Hgps */
      Inerz7.Data[0]=*u7;
      Inerz7.Data[1]=*(u7+1);
      Inerz7.Data[2]=*(u7+2);
      Inerz7.Data[3]=*(u7+3);
      /* other bytes are empty */
      Inerz7.Data[4]=0x00;
      Inerz7.Data[5]=0x00;
      Inerz7.Data[6]=0x00;
      Inerz7.Data[7]=0x00;
    
      /* packet Inerz8 configuration */

      /* 11-bit ID */
      Inerz8.Mode=0;
      /* No-RTR */
      Inerz8.RTR=0;
      /* 8 bytes to send */
      Inerz8.DLC=8;
      /* 47 value in hex format */
      Inerz8.ID=0x02F;
      /* uint16 */
      Inerz8.Data[0]=*u8;
      Inerz8.Data[1]=*(u8+1);
      /* uint16 */
      Inerz8.Data[2]=*(u8+2);
      Inerz8.Data[3]=*(u8+3);
      /* uint16 */
      Inerz8.Data[4]=*(u8+4);
      Inerz8.Data[5]=*(u8+5);
      /* uint16 */
      Inerz8.Data[6]=*(u8+6);
      Inerz8.Data[7]=*(u8+7);
    
      /* packet Inerz9 configuration */

      /* 11-bit ID */
      Inerz9.Mode=0;
      /* No-RTR */
      Inerz9.RTR=0;
      /* 8 bytes to send */
      Inerz9.DLC=8;
      /* 63 value in hex format */
      Inerz9.ID=0x03F;
      /* uint16 */
      Inerz9.Data[0]=*u9;
      Inerz9.Data[1]=*(u9+1);
      /* uint16 */
      Inerz9.Data[2]=*(u9+2);
      Inerz9.Data[3]=*(u9+3);
      /* uint16 */
      Inerz9.Data[4]=*(u9+4);
      Inerz9.Data[5]=*(u9+5);
      /* uint16 */
      Inerz9.Data[6]=*(u9+6);
      Inerz9.Data[7]=*(u9+7);
    
      /* count number of messages in Rx buffer */
      res = VCI_Get_RxMsgCnt(port,&conta);

      /* error check*/
      if(res > 0){
  	     ssSetErrorStatus(S, "error in VCI_Get_RxMsgCnt\n");
      }

      /* save number of messages received to clear the buffer*/
      y2[0] = double(conta);
   
      /* the first time we have to clear Rx buffer entirely */
      if(first){
          
            /* count number of messages in Rx buffer */
            res = VCI_Get_RxMsgCnt(port,&conta);

            /* error check */
            if(res > 0){
      		     ssSetErrorStatus(S, "error in VCI_Get_RxMsgCnt\n");
    	    }
   
            /* buffer must be cleared */
            while(conta != 0){
                
                /* take a message from Rx buffer */
                res = VCI_RecvCANMsg(port,&CANRx);

                /* error check */
                if(res > 0){
    		         ssSetErrorStatus(S, "error in VCI_Get_RecvCANMsg\n");
    	        }
                
                /*decrease counter variable */
                conta--;
            }

            /* Rx buffer must be cleared the first time only */
            first=false; 

      }
    
      /**
       * Synchronization 
      */

      /* variable used for timeout */
      time1 = GetCounter(); 
          
      /**
       * We have to wait 'A', 'B' and 'C' messages sent from autopilot 
      */

      /* counter is set to 0 */
      conta = 0;
     
      while(! is_ok ){

           /* check if a message is in Rx buffer */
           while(conta == 0){ 

               res = VCI_Get_RxMsgCnt(port,&conta);

               /* error check */
	   	if(res > 0){
		     ssSetErrorStatus(S, "Error in VCI_Get_RxMsgCnt\n");
	   	}

               /* time elapsed (in seconds) */
               seconds = timeout(time1); 

               /* check if timuout has expired */
               if(seconds > 20){ 
                       break;
               }
           }
      
       
           /* We have to check if timeout has expired or if 'conta' is not 0 */

           /* timeout expired: error condition */
           if(conta == 0){
              ssSetErrorStatus(S, "Timeout expired!\n");
           }
           
           /* otherwise a message is in Rx buffer */
           res = VCI_RecvCANMsg(port,&CANRx);
           
           /* error check */
           if(res > 0){
    	       	ssSetErrorStatus(S, "Error while receiving first message\n");
           }
           
           /* decrease counter variable */
           conta--;
        
           /*
            * other message could be in Rx buffer 
            * so we wait for the 3 messages we wanted to receive
           */
           
           /* check ID message */
           if(CANRx.ID == 0x50F){
               
               /* output values configuration */
               /* 4 uint16 variables (commands to motors) */
               y1[0] = (uint16_T)(CANRx.Data[1] << 8 | CANRx.Data[0]);
               y1[1] = (uint16_T)(CANRx.Data[3] << 8 | CANRx.Data[2]);
               y1[2] = (uint16_T)(CANRx.Data[5] << 8 | CANRx.Data[4]);
               y1[3] = (uint16_T)(CANRx.Data[7] << 8 | CANRx.Data[6]);
              
               /* we have to receive 'B' and 'C' messages */
               while(conta < 2){ 

                   /* we have to wait for 2 messages */
                   /* we wait for 3 messages to avoid problems while receiving messages */
                   
                   /* count number of messages received */
                   res = VCI_Get_RxMsgCnt(port,&conta);
                   
                   /* error check */
                   if(res > 0){  
                        ssSetErrorStatus(S, "error in Rx buffer\n");
                    }

                }
                
                /* take a message from Rx buffer */
                res = VCI_RecvCANMsg(port,&CANRx);

                /*  error check */
      	        if(res > 0){
      	           	ssSetErrorStatus(S, "Error while receiving next message\n");
      	        }
             	  
                /* output data configuration */
                y3[0] = (uint16_T)(CANRx.Data[1] << 8 | CANRx.Data[0]);
                y3[1] = (uint16_T)(CANRx.Data[3] << 8 | CANRx.Data[2]);
                y3[2] = (uint16_T)(CANRx.Data[5] << 8 | CANRx.Data[4]);
                y3[3] = uint16_T)(CANRx.Data[7] << 8 | CANRx.Data[6]);
                
                /* receive a new message */
                res = VCI_RecvCANMsg(port,&CANRx);

                /* error check */
                if(res>0){
        		ssSetErrorStatus(S, "Error while receiving next message\n");
              	}
               
                /* output data configuration */
                y4[0] = (uint16_T)(CANRx.Data[1] << 8 | CANRx.Data[0]);
                y4[1] = (uint16_T)(CANRx.Data[3] << 8 | CANRx.Data[2]);
                y4[2] = (uint16_T)(CANRx.Data[5] << 8 | CANRx.Data[4]);
                y4[3] = (uint16_T)(CANRx.Data[7] << 8 | CANRx.Data[6]);
         
                /* all 3 messages has been received */
                is_ok = true;

            }
           
           if( !is_ok ){

                 /* count number of messages received */
    	           res = VCI_Get_RxMsgCnt(port,&conta);

                 /* error check */
       		 if(res > 0){
		       	ssSetErrorStatus(S, "error in VCI_Get_RxMsgCnt\n");
       		 }
    	   
	         if(conta > 0){
	             
	                 /* clear Rx buffer */
	                 res = VCI_Clr_RxMsgBuf(port);
	
	                 /* error check */
	  	         if(res > 0){
	  	          	ssSetErrorStatus(S, "Buffer not cleared correctly \n");
	     		 }
	    	 }
            }
    	   
            /* counter variable set to 0 */
            conta = 0;

   }
	

    /* now application and autopilot are sinchronized */

    /* elapsed time */
    y2[0] = GetCounter(); 

    /* counter variable is set to 0 */
    conta = 0; 
    
    
    /* send simulated data */

    /* increase variable */
    low_frequency++;
    
    /* send 1st message */
    res = VCI_SendCANMsg(port,&Inerz1);

    if(res > 0){
  	  ssSetErrorStatus(S, "Error while sending 1st message\n");
    }
      
    /* send 2nd message */
    res = VCI_SendCANMsg(port,&Inerz2);

    if(res > 0){
  	  ssSetErrorStatus(S, "Error while sending 2nd message\n");
    }
      
    /* send 3rd message */
    res = VCI_SendCANMsg(port,&Inerz3);

    if(res > 0){
  	ssSetErrorStatus(S, "Error while sending 3rd message\n");
    }
        
    /*One message must be sent at a lower frequency */
    if(low_frequency == 20){
        
        /* send 4th message*/
        res = VCI_SendCANMsg(port,&Inerz4); 

        /* error check */
        if(res > 0){
	        ssSetErrorStatus(S, "Error while sending 4th message\n");
	}

        /* frequency variable is set to 0 */
        low_frequency = 0;
    }
	
    /* send 5th message */
    res = VCI_SendCANMsg(port,&Inerz5);

    /* error check */
    if(res > 0){
	ssSetErrorStatus(S, "Error while sendin 5th message\n");
    }

    /* send 6th message */
    res = VCI_SendCANMsg(port,&Inerz6);

    /* error check */
    if(res > 0){
	ssSetErrorStatus(S, "Error while sending 6th message\n");
    }
    
}


/* Function: mdlTerminate =====================================================
 * Abstract:
 *    No termination needed, but we are required to have this routine.
 */
static void mdlTerminate(SimStruct *S)
{
     /* close CAN port */
     int_T res= VCI_CloseCAN(pCAN.DevPort);

      if(res > 0){
	ssSetErrorStatus(S, "Error while closing CAN port\n");
      }
    
      #ifndef MATLAB_MEX_FILE
      SetPriorityClass(GetCurrentProcess(), NORMAL_PRIORITY_CLASS);
      SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_NORMAL);
      #endif
}



#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
