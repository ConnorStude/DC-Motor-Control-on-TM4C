//User input RPM
OS_Wait(&LCD);
Set_Position(0x00);
Display_Msg("Input RPM: ");
OS_Signal(&LCD);

void First_Line_Display() {
	while (indexArray <= 4) {
		if (indexArray == 1) {
			OS_Wait(&LCD);
			Set_Position(0x0b);
			Display_Msg();			//Display ASCII input
			OS_Signal(&LCD); 
		}
		if (indexArray == 2) {
			OS_Wait(&LCD);
			Set_Position(0x0c);
			Display_Msg();			//Display ASCII input
			OS_Signal(&LCD);
		}
		if (indexArray == 3) {
			OS_Wait(&LCD);
			Set_Position(0x0d);
			Display_Msg();			//Display ASCII input
			OS_Signal(&LCD);
		}
		if (indexArray == 4) {
			OS_Wait(&LCD);
			Set_Position(0x0e);
			Display_Msg();			//Display ASCII input
			OS_Signal(&LCD);
		}
	}
	if (indexArray == 0 || indexArray > 4) {
		OS_Wait(&LCD);
		Set_Position(0x0b);
		Display_Msg("    ");
		OS_Signal(&LCD);
	}
	// if # is pressed clear board
}

// Display target speed prompt
OS_Wait(&LCD);
Set_Position(0x40);
Display_Msg("T: ");
OS_Signal(&LCD);

// Display target Speed
targetSpeedString =
OS_Wait(&LCD);
Set_Postion(0x43);
Display_Msg(targetSpeedString);
OS_Signal(&LCD);

// Display current speed prompt
OS_Wait(&LCD);
Set_Position(0x48);
Display_Msg("C: ");
OS_Signal(&LCD);

// Display current speed
currentSpeedString = 
OS_Wait(&LCD);
Set_Position(0x4b);
Display_Msg(currentSpeedString);
OS_Signal(&LCD);