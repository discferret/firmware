/********************************************************************
 FileName:     	HardwareProfile - PIC18F87J50 PIM.h
 Dependencies:	See INCLUDES section
 Processor:	PIC18 USB Microcontrollers
 Hardware:	PIC18F87J50 PIM
 Compiler:  	Microchip C18
 Company:		Microchip Technology, Inc.

 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the “Company”) for its PIC® Microcontroller is intended and
 supplied to you, the Company’s customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

********************************************************************
 File Description:

 Change History:
  Rev   Date         Description
  1.0   11/19/2004   Initial release
  2.1   02/26/2007   Updated for simplicity and to use common
                     coding style
  2.3   09/15/2008   Broke out each hardware platform into its own
                     "HardwareProfile - xxx.h" file
********************************************************************/

#ifndef HARDWARE_PROFILE_H
#define HARDWARE_PROFILE_H

    /*******************************************************************/
    /******** USB stack hardware selection options *********************/
    /*******************************************************************/
    //This section is the set of definitions required by the MCHPFSUSB
    //  framework.  These definitions tell the firmware what mode it is
    //  running in, and where it can find the results to some information
    //  that the stack needs.
    //These definitions are required by every application developed with
    //  this revision of the MCHPFSUSB framework.  Please review each
    //  option carefully and determine which options are desired/required
    //  for your application.

    /** USB ************************************************************/
    //The PIC18F87J50 FS USB Plug-In Module supports the USE_USB_BUS_SENSE_IO
    //feature.  The USE_SELF_POWER_SENSE_IO feature is not implemented on the
    //circuit board, so the USE_SELF_POWER_SENSE_IO define should always be
    //commented for this hardware platform.

	// -- USE_SELF_POWER_SENSE_IO --
	// Defined if the USB device is capable of determining its power state
	// (bus-powered or self-powered).
//    #define USE_SELF_POWER_SENSE_IO
	// Also set the I/O pin in use here (note that if USE_SELF_POWER_SENSE_IO is
	// disabled, self_power will automagically get set to 1 and the I/O pin
	// will be ignored...)
    #define tris_self_power     TRISAbits.TRISA2    // Input
    #if defined(USE_SELF_POWER_SENSE_IO)
    #define self_power          PORTAbits.RA2
    #else
    #define self_power          1
    #endif

	// -- USE_USB_BUS_SENSE_IO --
	// Defined if the USB device is capable of sensing the state of the bus,
	// i.e. Connected or Disconnected. That is, can the device detect whether
	// VBUS = 5V?
//    #define USE_USB_BUS_SENSE_IO
	// Also set the I/O pin in use here (note that if USE_USB_BUS_SENSE_IO is
	// disabled, self_power will automagically get set to 1 and the I/O pin
	// will be ignored...)
    #define tris_usb_bus_sense  TRISBbits.TRISB5    // Input
    #if defined(USE_USB_BUS_SENSE_IO)
    #define USB_BUS_SENSE       PORTBbits.RB5
    #else
    #define USB_BUS_SENSE       1
    #endif
 
    //Uncomment this to make the output HEX of this project 
    //   to be able to be bootloaded using the HID bootloader
	#define PROGRAMMABLE_WITH_USB_HID_BOOTLOADER		


    /*******************************************************************/
    /*******************************************************************/
    /*******************************************************************/
    /******** Application specific definitions *************************/
    /*******************************************************************/
    /*******************************************************************/
    /*******************************************************************/

    /** Board definition ***********************************************/
    //These defintions will tell the main() function which board is
    //  currently selected.  This will allow the application to add
    //  the correct configuration bits as wells use the correct
    //  initialization functions for the board.  These defitions are only
    //  required in the stack provided demos.  They are not required in
    //  final application design.
	#define PLATFORM_DISCFERRET
    #define CLOCK_FREQ 48000000
    #define GetSystemClock()  CLOCK_FREQ   
    #define GetInstructionClock() CLOCK_FREQ   

	/**************
	 * PORT MAPPINGS
	 *
	 * PORTA
	 *   5: nc
	 *   4: nc
	 *   3: nc
	 *   2: nc
	 *   1: nc
	 *   0: nc
	 *
	 * PORTB
	 *   7: ICSP_PGD	Reserved for ICSP
	 *   6: ICSP_PGC	Reserved for ICSP
	 *   5: PMALL	--> Parallel Master Port Address Load Low
	 *   4: PMALH	--> Parallel Master Port Address Load High
	 *   3: nc
	 *   2: nc
	 *   1: nc
	 *   0: nc
	 *
	 * PORTC
	 *   7: RX1		--> UART port RX
	 *   6: TX1		--> UART port TX
	 *   5: FCDATA0	--> FPGA Config Data 0	(MSSP Master Mode SPI Data Out  / MOSI)
	 *   4: =1		--> Tied to VCC			(MSSP Master Mode SPI Data In   / MISO)
	 *   3: FCDCLK	--> FPGA Config Clock	(MSSP Master Mode SPI Clock Out / SCLK)
	 *   2: nc
	 *   1: nc
	 *   0: nc
	 *
	 * PORTD
	 *   7: PMD7	}
	 *   6: PMD6	}
	 *   5: PMD5	}
	 *   4: PMD4	}	Parallel Master Port
	 *   3: PMD3	}	Data Bus
	 *   2: PMD2	}
	 *   1: PMD1	}
	 *   0: PMD0	}
	 *
	 * PORTE
	 *   7: nc
	 *   6: nc
	 *   5: nc
	 *   4: nc
	 *   3: nc
	 *   2: nc
	 *   1: PMWR	--> Parallel Master Port Write
	 *   0: PMRD	--> Parallel Master Port Read
	 *
	 * PORTF
	 *   7: nc
	 *   6: nc
	 *   5: nc
	 *   4: USB D+
	 *   3: USB D-
	 *   2: USBDET	--> =1 if USB plug is connected
	 *
	 * PORTG
	 *   4: nc
	 *   3: nc
	 *   2: nc
	 *   1: nc
	 *   0: FCNCONF	-->	FPGA Configuration nCONFIG
	 *
	 * PORTH
	 *   7: nc
	 *   6: nc
	 *   5: nc
	 *   4: nc
	 *   3: nc
	 *   2: nc
	 *   1: nc
	 *   0: nc
	 *
	 * PORTJ
	 *   7: nc
	 *   6: MCULED	--> =0 to turn LED on
	 *   5: BOOT	--> =0 if Force Bootloader jumper closed
	 *   4: nc
	 *   3: FCDONE	--> FPGA Config Done
	 *   2: FCNSTAT	--> FPGA Config nSTATUS
	 *   1: nc
	 *   0: nc
	 */
	
	#define	PIN_FCDATA0	LATCbits.LATC5
	#define	PIN_FCDCLK	LATCbits.LATC3
	#define	PIN_FCNCONF	LATGbits.LATG0
	#define	PIN_FCDONE	PORTJbits.RJ3
	#define	PIN_FCNSTAT	PORTJbits.RJ2
	
	#define PIN_MCULED	LATJbits.LATJ6
	#define	PIN_BOOT	PORTJbits.RJ5

    /** I/O pin definitions ********************************************/
    #define INPUT_PIN 1
    #define OUTPUT_PIN 0


    /** Configuration options *****************************************/
	// Define this to use 16-bit SPP addressing
//	#define PMP_ADDR_16BIT
	// Define this to slow down the Parallel Master Port
//	#define PMP_SLOW


#endif  //HARDWARE_PROFILE_H
