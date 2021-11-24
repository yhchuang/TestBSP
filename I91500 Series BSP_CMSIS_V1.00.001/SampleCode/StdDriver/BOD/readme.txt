/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) Nuvoton Technology Corp. All rights reserved.                                              */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
---------------------------------------------------------------------------------------------------------
Purpose:
---------------------------------------------------------------------------------------------------------
	This sample code shows how to configure ¡§Brown-Out detection¡¨.
	
---------------------------------------------------------------------------------------------------------
Operation:
---------------------------------------------------------------------------------------------------------
	(1)	Connect the EVB power¡¦s pins VCC and GND to a variable voltage source, like a DC power supply, and set the voltage level at 3.3V.
	(2)	Compile and execute.
	(3)	Program test procedure ¡V
		1.	GPIO(GPA0,GPA1,GPA2,GPA3) will flash to show system standby.
		2.  Configure BOD level and interrupt function.
		3.	User can lower the voltage source of VCC to trigger BOD.
		4.	The BOD event will trigger and chip will reset.
		
---------------------------------------------------------------------------------------------------------
Note:
---------------------------------------------------------------------------------------------------------
