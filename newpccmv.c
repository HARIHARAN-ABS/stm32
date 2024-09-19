/*
 * newpccmv.c
 *
 *  Created on: 17-Sep-2024
 *      Author: vijayahari
 */
#include "Pc_cmv.h"  //Here certain struct & enumerations format variables of mode params, ramp time, vent types, breath count and
                     //functions such as Inspiration and Expiration time reset variables, Average flow, Task for pccmv mode are been defined.
/********************************************************************************************
// followed by the function calls of the code
//********************************************************************************************/
 static void Inspiration_Time_Blower_Control_PC_CMV(unit16_t); // function for blower control during inspiration phase.
 static void Expiration_Time_Blower_Control_PC_CMV(unit16_t,float,uint16_t);  // function for blower control during Exppiration phase.
 static void Smooth_ramp_pressure();   	// function call for pressure increment in ramp time of the mode.

 /** Creation of freeRtos task to get the variables of PCCMV mode from the nrf53832 bluetooth module
 	 Files to refer: Pc_cmv.h , Bluetooth_Graph.h , sensor.h , oom202.h , task.c, Mode_Initial_Flag_Set.h , main.h
 	                                                                                                                  **/


 void Pc_Cmv_Mode_Packet_Data(RECEIVE_GRAPH_PACKET *Receive_Graph_Packet)	// Task for receiving data from nrf, here RECEIVE GRAPH PACKET is a struct attribute consists of certain set and derived parameters of the mode.
 {
 	uint32_t One_Breathe_time;												// Accessing and mapping the struct variables of PCCMV mode parameters with the nrf bluetooth parameters to receive data in real time operation
 	PC_CMV.PIP_Val                = Receive_Graph_Packet->PIP_PS_Phigh;     //mapping of PIP value in mode param with equivalent value in RECIEVE GRAPH PACKET struct attribute
 	PC_CMV.PEEP_Val               = Receive_Graph_Packet->PEEP_CPAP_Plow;	//mapping of PEEP value in mode param with equivalent value in RECIEVE GRAPH PACKET struct attribute
 	PC_CMV.FIO2_Val               = Receive_Graph_Packet->FiO2;				//mapping of FiO2 value in mode param with equivalent value in RECIEVE GRAPH PACKET struct attribute
 	PC_CMV.RESPIRATORY_RATE_Val   = Receive_Graph_Packet->RR;				//mapping of Respiratory rate  value in mode param with equivalent value in RECIEVE GRAPH PACKET struct attribute
 	One_Breathe_time              = (One_Minite_In_MS / PC_CMV.RESPIRATORY_RATE_Val); // Defining time taken for one breath in cycle (i.e) one minute in millisecond(1000) divided by RR value
 	PC_CMV.INSPIRATION_TIME       = ( ( Receive_Graph_Packet->T_high) * 100 );	//mapping of Inspiration time value in mode param with equivalent value in RECIEVE GRAPH PACKET struct attribute (T_high)*100 due to conversion of seconds to ms.
  	PC_CMV.EXPIRATION_TIME        = (One_Breathe_time - PC_CMV.INSPIRATION_TIME);// Defining Expiration time ,that is difference of total breathtime and insp time.
 	PC_CMV.Rise_Time              = Receive_Graph_Packet->Rise_Time;		// //mapping of Rise time value in mode param with equivalent value in RECIEVE GRAPH PACKET struct attribute

 	Alert_Check_Time              = ((float)PC_CMV.INSPIRATION_TIME/3); // Defining the checking time for alert
 	Vent_Type                     = ET_TUBE_MODE;		//Type of ventilation selection from respective enum.

 	OOM202.offset_minimum         = ( 0x7F & (Receive_Graph_Packet->Vent_Type_Min_O2)); // mapping the O2 sensor minimum offset value to the equivalent value (Vent_Type_Min_O2) in RECIEVE GRAPH PACKET struct attribute.
 	OOM202.offset_maximum         = Receive_Graph_Packet->Max_O2;		//	Similarly, mapping the O2 sensor maximum offset value to the equivalent value (Max_O2) in RECIEVE GRAPH PACKET struct attribute.

     Mode_Initial_Flag_Set         = OPEN;	// Setting the flag state of the Mode_Initial_Flag_Set enum in OPEN condition.
 	vTaskResume(Mode_initial_Flag_Set_Handler); // FreeRtos API for Resuming the task could be referred in task.c file.
 }

 /* The below task is been used to call the two functions insp and exp time blower control **/

 void Pc_Cmv_Task (void *argument)	// task for controlling blower during insp and exp phase of breath cycles.
 {
 	while(1)
 	{
 		switch (Run_Current_Breathe_State)	// calling  current breath cycle enum
 		{
 			case Run_Inspiration_Cycle:		// Enum variable of Run_Current_Breathe_State
 				Inspiration_Time_Blower_Control_PC_CMV(Pressure_Mode_blower_control.BLOWER_DAC_VAL); // function call for insp time blower control with arguments involving Struct variables of Pressure_Mode_blower_control from file PIP_control.h
 			break;
 			case Run_Expiration_Cycle:		// Enum variable of Run_Current_Breathe_State
 				Pressure_Mode_blower_control.BLOWER_DAC_VAL=0;	// Setting the Blower dac value to be Zero in the struct Pressure_Mode_blower_control
 				Expiration_Time_Blower_Control_PC_CMV(Pressure_Mode_blower_control.BLOWER_DAC_VAL,
 											   PEEP_Maintain_Parameter.PEEP_Temp,			//function call for Exsp time blower control with arguments involving Struct variables of Pressure_Mode_blower_control and PEEP maintain paramaeter
 											   PEEP_Maintain_Parameter.Expiratory_Valve_Open_Time);// from files PIP_control.h and Common_Parameters.h
 			break;
 			case No_Run_State:	// Enum variable of Run_Current_Breathe_State
 			break;
 			default:
 			break;
 		}
 		vTaskDelay(Two_Millisecond_Delay);	// FreeRtos API for delaying the task could be referred in task.c file.
 	}
 }

 /**
  * @brief Inspiration_Time_Blower_Control_PC_CMV.
  * This function defines the  dac value in DAC1 pheripheral for control a blower for every 2 milliseconds in inspiration time.
  * */

 static void Inspiration_Time_Blower_Control_PC_CMV(uint16_t Blower_Signal)
 {
 	Blower_Signal(Blower_Signal);	// Blower_Signal is defined in file Macros.h (refer this) with the register of the pin to access it
 }

 /**
   * This function defines the  dac value in DAC1 pheripheral for control a blower for every 2 milliseconds in expiration phase of the breath cycle
   * It also involves the Expiratory valve conntrol (lock and open) to maintain peep valve at an given set pressure instant.
   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   **/

 static void Expiration_Time_Blower_Control_PC_CMV(uint16_t Blower_Signal,float PEEP,uint16_t Expiratory_Valve_Open_Time)
 {
 	Blower_Signal( Blower_Signal);	// Blower signal value which is accessed using the register in form of register. (from Macros.h)
 		if(Peep_Status==PEEP_ACHEIVED)	// Enum value and its variable from file Common_Parameters.h
 		{
 				if(GRAPH_VALUES.pressure < PEEP)		//If pressure value received from bluetooth is less than PEEP value, Blower signal would be of assigned value
 				{										//that is dac=0
 					Blower_Signal( Blower_Signal);
 				}
 				else
 				{
 					Blower_Signal( Blower_Signal);		// else condition also follows same condition which maintains the blower value at zero
 				}
 		 }
 		if(GRAPH_VALUES.pressure<= PEEP)			// Checks for pressure value received from bluetooth, if it less than peep and check for exp valve lock. If it is opened, close the exp valve.
 		{
 			if(Expiratory_Valve_Lock_Delay==OPENED)
 			{
 				Expiratory_Valve_Lock_Delay=CLOSED;
 				vTaskDelay(Expiratory_Valve_Open_Time);		// FreeRtos API for delaying the task could be referred in task.c file
 			}
 			Peep_Status=PEEP_ACHEIVED;			// once the pressure value equals the PEEP thenn it closes the Expirtatory valve.
 			ExpValve_CLOSE();					// This is controlled via assignoing register valjue of the pin corresponding to the Exp valve. Refer to file Macros.h for register .
 		}
 		else if(Peep_Status==PEEP_NOT_ACHEIVED)
 		{
 			ExpValve_OPEN();				// If peep value still not gets achieved then, the exp valve remains opened. refer <Mactros.h>
 		}
 }

 /**
  * the following function consists of variables that needs to get Resetted before starting up of insipiration cycle
  * files to refer: Oxygen_Blending.h , main.h ,
  */

 void PC_CMV_Inspiration_Time_Variables_Reset()
 {
 	Oxygen_Blending_Status=OXYGEN_ACHIEVED;	// checking the status of oxygen blending by accessing enum variable in file Oxygen_Blending.h
 	Blower_Status       = BlOWER_ON;	// turning ON the blower by accessing enum variable in file Vc_cmv.h
 	Find_Average_Flow();	// function call for finding average flow (cumulative/flow count)
 	Breath_Count         = Count_The_Breath;	// Counting breath by accessing enum variable
 	Smooth_Ramp_Pressure();						// calling of function of ramp pressure for execution at this point
 	PIP_Not_Acheieved();						// this function has been called but has no content in it.
 	ExpValve_CLOSE();							// closing of expiratory valve by accessing the registers refer in Macros.h file
 	PIP_AVERAGE();								// this function checks for the average of the pip values obtained by (cumulative/couht), which are given as struct variables in Common_parameters.c file
 	PEEP_AVERAGE();								// this function checks for average of peep with patients trigger and no trigger and produces value by (cumulative/count),which are given as struct variables in Common_parameters.c file
 	O2_DAC_Control(Common_Mode_Parameter._FIO2_Val);  // this function helps in setting Dac value for the setted FiO2 value. It compares it in two modes such as Pressure & volume controlled and executes certain logics for different conditions.
 	Check_Alert();								// checks for alert related logics. refer the file Alert.c
 	Alert_Inspiration_Time_Parameter();			// function call for insp time alert given in Alert.C file
 	LED_Alert();								// function for led ON and OFF at alert happened and not happened cases. refer Alert.c
 	Alert_Status                                              = NO_ALERT; // status of alert from Alert.h file
 	GRAPH_VALUES.Maximum_oxygen_flow                          = 0;      // Setting maximum oxygen flow to zero . refer file database.h
 	Sampled_Parameter_Inspiration_time();		// function for sampling of parameter function and its transmission via bluetooth. Refer files Sampled_Parameters.c , Sampled_Parameters.h , Alert.h
 	if( (PC_CMV.PEEP_Val != PEEP_Maintain_Parameter.PEEP_AVG_VAL ) )	// comparing PEEP and PEEP AVG values...
 		//** the  following involves function call for PEEP maintaining which is done by checking conditions of RR value below and above 30, that involves certain sub- functions that could be referred from file Expiratory_Valve_Control_Mode.c **//
 	{
 		PEEP_Maintain_Parameter.Expiratory_Valve_Open_Time     = PEEP_Control_For_Expiratory_Valve_Open_Time_In_Pressure_Mode(PEEP_Maintain_Parameter.PEEP_AVG_VAL,PC_CMV.RESPIRATORY_RATE_Val,PC_CMV.INSPIRATION_TIME,PC_CMV.PEEP_Val,PEEP_Maintain_Parameter.Expiratory_Valve_Open_Time);
 	}
 	INCREASE_EACH_DAC_SET_CONST(Common_Mode_Parameter._PIP_Val,PC_CMV.RISE_TIME_MS_Val);	// this fn involves increasingthe dac value accordingly with the PIP given and contains sub functions logics for outputs such as acieved slowly & fastly. Refer PIP_Control.c
 	BLOWER_ENDING_DAC_SIGNAL_TUNE(PC_CMV.PIP_Val,PC_CMV.INSPIRATION_TIME,PC_CMV.Rise_Time_percentage);	// this fnis for tuning the Dac signal at the end of the insp to achieve the set values of pressures. Refer PIP_Control.c file
 	BREATH_STATE                                               = INSPIRATION_CYCLE;		// Setting of enum variable from Common_Parameters.h file
 	Peep_Status                   				               = PEEP_NOT_ACHEIVED;		// Setting of enum variable from Common_Parameters.h file
 	Expiratory_Valve_Lock_Delay   			                   = OPENED;				// Setting of enum variable from Common_Parameters.h file
 	PIP_Status                    				               = PIP_NOT_ACHEIVED;    	// Setting of enum variable from Common PIP_Control.h file
 	Set_PIP_Status                                             = SET_PIP_VALUE_NOT_ACHEIVED;	// Setting of enum variable from Common PIP_Control.h file
 	Read_Time_vs_Pressure                                      = READ_TIME_OPEN;		// Setting of enum variable from Common PIP_Control.h file
 	PIP_control_Task.PIP_Control_Event_Occured                 = RESET;					// Setting of enum variable from Common PIP_Control.h file
 	PIP_Average_Parameter.Maximum_PIP_Acheived                 = RESET;					// Setting of enum variable from Common PIP_Control.h file
 	Pressure_Mode_blower_control.BLOWER_DAC_VAL                = DAC_VAL(PC_CMV.PEEP_Val );	// Setting of enum variable from Common PIP_Control.h file
 	Pressure_Mode_blower_control.LAST_BLOWER_DAC_VAL           = Pressure_Mode_blower_control.BLOWER_DAC_VAL;	// Setting of enum variable from Common PIP_Control.h file
 	TIME_Base_parameter._INSPIRATION_TIME                      = PC_CMV.INSPIRATION_TIME;	// mapping of Insp time to PCCMV parameters from struct variables of Time_Base_Parameter.
 	TIME_Base_parameter.INSPIRATION_TIME_ACHEIVED              = RESET;					// Setting insp time achieved value to RESET condition from given struct variable
 	PIP_control_Task.Last_Early_Acheived_Ramp_Time_Percentage  = PIP_control_Task.Early_Acheived_Ramp_Time_Percentage;
 	GRAPH_VALUES.volume                                        = RESET;		// Resetting the value of volume from struct GRAPH_VALUES
 	Volume_max                                                 = RESET;		// Resetting the value of maximum value
 	Next_Half_Breath_Cycle                                     = Generate_Expiration_Cycle;  // setting enum variables of Next_Half_Breath_Cycle
 	Run_Current_Breathe_State                                  = Run_Inspiration_Cycle;		  // setting enum variables of Run_Current_Breathe_State
 	vTaskDelay(PC_CMV.INSPIRATION_TIME);		// FreeRtos API for delaying the task could be referred in task.c file
 }


 /**
  * The below given function gives the smooth raising of pressure over the breath cycle **/

 static void Smooth_Ramp_Pressure()		// function for smooth rise in presssure values
 {
 	if(Smooth_Ramp_Time   == Smooth_Ramp_Time_Val_Pending)	// accessing smooth ramp time struct variable
 	{
 		if( (PC_CMV.PEEP_Val == PEEP_Maintain_Parameter.PEEP_AVG_VAL)  && (PC_CMV.PEEP_Val >= 9) && (PC_CMV.PEEP_Val <= 14) && (PC_CMV.PIP_Val >=30) )
 		{
 			if(PEEP_Maintain_Parameter.Expiratory_Valve_Open_Time  < 2)
 			{
 				if(Smooth_Ramp_Time_Val_Count > 2 )									// by comparing different peep levels and exp valve open time, the ramp time value is been determined
 				{
 					Smooth_Ramp_Time_Val = PC_CMV.PEEP_Val;
 					Smooth_Ramp_Time     = Smooth_Ramp_Time_Val_Finished;
 				}
 				else
 				{
 					Smooth_Ramp_Time_Val_Count++;
 				}
 			}
 		}
     }
 }


 void Find_Average_Flow()           //fn for finding average flow
 {
 Avg_Flow   = Flow_Cummulative_Val/Flow_Count;		// avg flow is given by cumulative /flow count values which are given in Volume_control.h file

 	if(Avg_Flow <=1)
 	{
 		Avg_Flow = 1;					// condition for flow <1 and resp values assigned.
 	}
 	Flow_Count =0;
 	Flow_Cummulative_Val =0;
 }

 void PIP_Not_Acheieved()		// fn for if pip not achieved but not used or called anyhwhere with commanded instructions and mentioned for usage in debugging purpose.
 {
 	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
 }

 /**
  * @brief PIP_Acheieved.
  * This Function used for debugging purpose.
  * */
 void PIP_Acheieved()		//fn for if pip not achieved but not used or called anyhwhere with commanded instructions and mentioned for usage in debugging purpose
 {
 	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
 }

