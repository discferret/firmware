/********************************************************************
 FileName:		main.c
 Dependencies:	See INCLUDES section
 Processor:		PIC18F85J50
 Hardware:		DiscFerret 
 Compiler:  	Microchip C18
 Company:		Red Fox Engineering

 Based on code from Microchip Technology Inc. Original Microchip
 license agreement follows.

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

********************************************************************/

/** INCLUDES *******************************************************/
#include "USB/usb.h"
#include "HardwareProfile.h"
#include "USB/usb_function_generic.h"

/** CONFIGURATION **************************************************/
#if defined(PLATFORM_DISCFERRET)
	// DiscFerret Magnetic Disc Analyser
	#pragma config	XINST	= OFF			// extended instruction set off
	#pragma config	STVREN	= ON			// stack overflow reset enabled
	#pragma config	WDTEN	= OFF			// watchdog timer off
	#pragma config	WDTPS	= 32768			// WDT postscale = 1:32768
	#pragma config	CP0		= OFF			// code protection off
	#pragma config	MODE	= MM			// microcontroller mode, external prog-mem bus disabled
	#pragma config	PMPMX	= DEFAULT		// connect PMP to external memory bus
	// we don't use the CCP, so setting its mux bit would be pointless
	#pragma config	FOSC	= HSPLL			// high speed oscillator, PLL enabled
	#pragma config	PLLDIV	= 5				// PLL divide by 5 for 20MHz osc input
	#pragma config	CPUDIV	= OSC1			// no CPU system clock division (postscaler)
	#pragma config	IESO	= OFF			// two-speed start-up off
	#pragma config	FCMEN	= OFF			// failsafe clock monitor off	
#else
    #error No hardware board defined, see "HardwareProfile.h"
#endif



/** VARIABLES ******************************************************/
#if defined(__18F14K50) || defined(__18F13K50) || defined(__18LF14K50) || defined(__18LF13K50) 
    #pragma udata usbram2
#elif defined(__18F2455) || defined(__18F2550) || defined(__18F4455) || defined(__18F4550)\
    || defined(__18F2458) || defined(__18F2453) || defined(__18F4558) || defined(__18F4553)
    #pragma udata USB_VARIABLES=0x500
#elif defined(__18F4450) || defined(__18F2450)
    #pragma udata USB_VARIABLES=0x480
#else
    #pragma udata
#endif

unsigned char OUTPacket[64];	//User application buffer for receiving and holding OUT packets sent from the host
unsigned char INPacket[64];		//User application buffer for sending IN packets to the host
#pragma udata
USB_HANDLE USBGenericOutHandle;
USB_HANDLE USBGenericInHandle;
#pragma udata

/** PRIVATE PROTOTYPES *********************************************/
static void InitializeSystem(void);
void USBDeviceTasks(void);
void YourHighPriorityISRCode(void);
void YourLowPriorityISRCode(void);
void UserInit(void);
void ProcessIO(void);

/** VECTOR REMAPPING ***********************************************/
#if defined(__18CXX)
	//On PIC18 devices, addresses 0x00, 0x08, and 0x18 are used for
	//the reset, high priority interrupt, and low priority interrupt
	//vectors.  However, the current Microchip USB bootloader 
	//examples are intended to occupy addresses 0x00-0x7FF or
	//0x00-0xFFF depending on which bootloader is used.  Therefore,
	//the bootloader code remaps these vectors to new locations
	//as indicated below.  This remapping is only necessary if you
	//wish to program the hex file generated from this project with
	//the USB bootloader.  If no bootloader is used, edit the
	//usb_config.h file and comment out the following defines:
	//#define PROGRAMMABLE_WITH_USB_HID_BOOTLOADER
	//#define PROGRAMMABLE_WITH_USB_LEGACY_CUSTOM_CLASS_BOOTLOADER
	
	#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)
		#define REMAPPED_RESET_VECTOR_ADDRESS			0x1000
		#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x1008
		#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x1018
	#elif defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)	
		#define REMAPPED_RESET_VECTOR_ADDRESS			0x800
		#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x808
		#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x818
	#else	
		#define REMAPPED_RESET_VECTOR_ADDRESS			0x00
		#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x08
		#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x18
	#endif
	
	#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
	extern void _startup (void);        // See c018i.c in your C18 compiler dir
	#pragma code REMAPPED_RESET_VECTOR = REMAPPED_RESET_VECTOR_ADDRESS
	void _reset (void)
	{
	    _asm goto _startup _endasm
	}
	#endif
	#pragma code REMAPPED_HIGH_INTERRUPT_VECTOR = REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS
	void Remapped_High_ISR (void)
	{
	     _asm goto YourHighPriorityISRCode _endasm
	}
	#pragma code REMAPPED_LOW_INTERRUPT_VECTOR = REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS
	void Remapped_Low_ISR (void)
	{
	     _asm goto YourLowPriorityISRCode _endasm
	}
	
	#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
	//Note: If this project is built while one of the bootloaders has
	//been defined, but then the output hex file is not programmed with
	//the bootloader, addresses 0x08 and 0x18 would end up programmed with 0xFFFF.
	//As a result, if an actual interrupt was enabled and occured, the PC would jump
	//to 0x08 (or 0x18) and would begin executing "0xFFFF" (unprogrammed space).  This
	//executes as nop instructions, but the PC would eventually reach the REMAPPED_RESET_VECTOR_ADDRESS
	//(0x1000 or 0x800, depending upon bootloader), and would execute the "goto _startup".  This
	//would effective reset the application.
	
	//To fix this situation, we should always deliberately place a 
	//"goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS" at address 0x08, and a
	//"goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS" at address 0x18.  When the output
	//hex file of this project is programmed with the bootloader, these sections do not
	//get bootloaded (as they overlap the bootloader space).  If the output hex file is not
	//programmed using the bootloader, then the below goto instructions do get programmed,
	//and the hex file still works like normal.  The below section is only required to fix this
	//scenario.
	#pragma code HIGH_INTERRUPT_VECTOR = 0x08
	void High_ISR (void)
	{
	     _asm goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS _endasm
	}
	#pragma code LOW_INTERRUPT_VECTOR = 0x18
	void Low_ISR (void)
	{
	     _asm goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS _endasm
	}
	#endif	//end of "#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_LEGACY_CUSTOM_CLASS_BOOTLOADER)"

	#pragma code
	
	
	//These are your actual interrupt handling routines.
	#pragma interrupt YourHighPriorityISRCode
	void YourHighPriorityISRCode()
	{
		//Check which interrupt flag caused the interrupt.
		//Service the interrupt
		//Clear the interrupt flag
		//Etc.
        #if defined(USB_INTERRUPT)
	        USBDeviceTasks();
        #endif
	
	}	//This return will be a "retfie fast", since this is in a #pragma interrupt section 
	#pragma interruptlow YourLowPriorityISRCode
	void YourLowPriorityISRCode()
	{
		//Check which interrupt flag caused the interrupt.
		//Service the interrupt
		//Clear the interrupt flag
		//Etc.
	
	}	//This return will be a "retfie", since this is in a #pragma interruptlow section 

#elif defined(__C30__)
    #if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)
        /*
         *	ISR JUMP TABLE
         *
         *	It is necessary to define jump table as a function because C30 will
         *	not store 24-bit wide values in program memory as variables.
         *
         *	This function should be stored at an address where the goto instructions 
         *	line up with the remapped vectors from the bootloader's linker script.
         *  
         *  For more information about how to remap the interrupt vectors,
         *  please refer to AN1157.  An example is provided below for the T2
         *  interrupt with a bootloader ending at address 0x1400
         */
//        void __attribute__ ((address(0x1404))) ISRTable(){
//        
//        	asm("reset"); //reset instruction to prevent runaway code
//        	asm("goto %0"::"i"(&_T2Interrupt));  //T2Interrupt's address
//        }
    #endif
#endif




/** DECLARATIONS ***************************************************/
#pragma code

/******************************************************************************
 * Function:        void main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main program entry point.
 *
 * Note:            None
 *******************************************************************/

#if defined(__18CXX)
void main(void)
#else
int main(void)
#endif
{   
    InitializeSystem();

    #if defined(USB_INTERRUPT)
        USBDeviceAttach();
    #endif

    while(1)
    {
        #if defined(USB_POLLING)
		// Check bus status and service USB interrupts.
        USBDeviceTasks(); // Interrupt or polling method.  If using polling, must call
        				  // this function periodically.  This function will take care
        				  // of processing and responding to SETUP transactions 
        				  // (such as during the enumeration process when you first
        				  // plug in).  USB hosts require that USB devices should accept
        				  // and process SETUP packets in a timely fashion.  Therefore,
        				  // when using polling, this function should be called 
        				  // frequently (such as once about every 100 microseconds) at any
        				  // time that a SETUP packet might reasonably be expected to
        				  // be sent by the host to your device.  In most cases, the
        				  // USBDeviceTasks() function does not take very long to
        				  // execute (~50 instruction cycles) before it returns.
        #endif
    				  

		// Application-specific tasks.
		// Application related code may be added here, or in the ProcessIO() function.
        ProcessIO();        
    }//end while
}//end main


/********************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.                  
 *
 * Note:            None
 *******************************************************************/
static void InitializeSystem(void)
{
	#if defined(PLATFORM_DISCFERRET)
		//On the PIC18F87J50 Family of USB microcontrollers, the PLL will not power up and be enabled
		//by default, even if a PLL enabled oscillator configuration is selected (such as HS+PLL).
		//This allows the device to power up at a lower initial operating frequency, which can be
		//advantageous when powered from a source which is not gauranteed to be adequate for 48MHz
		//operation.  On these devices, user firmware needs to manually set the OSCTUNE<PLLEN> bit to
		//power up the PLL.
	    {
	        unsigned int pll_startup_counter = 600;
	        OSCTUNEbits.PLLEN = 1;  //Enable the PLL and wait 2+ms until the PLL locks before enabling USB module
	        while(pll_startup_counter--);
	    }
	    //Device switches over automatically to PLL output after PLL is locked and ready.
    #endif

	#if defined(PLATFORM_DISCFERRET)
		//Configure all I/O pins to use digital input buffers.  The PIC18F87J50 Family devices
		//use the ANCONx registers to control this, which is different from other devices which
		//use the ADCON1 register for this purpose.
	    WDTCONbits.ADSHR = 1;			// Select alternate SFR location to access ANCONx registers
	    ANCON0 = 0xFF;                  // Default all pins to digital
	    ANCON1 = 0xFF;                  // Default all pins to digital
	    WDTCONbits.ADSHR = 0;			// Select normal SFR locations
    #endif

//	The USB specifications require that USB peripheral devices must never source
//	current onto the Vbus pin.  Additionally, USB peripherals should not source
//	current on D+ or D- when the host/hub is not actively powering the Vbus line.
//	When designing a self powered (as opposed to bus powered) USB peripheral
//	device, the firmware should make sure not to turn on the USB module and D+
//	or D- pull up resistor unless Vbus is actively powered.  Therefore, the
//	firmware needs some means to detect when Vbus is being powered by the host.
//	A 5V tolerant I/O pin can be connected to Vbus (through a resistor), and
// 	can be used to detect when Vbus is high (host actively powering), or low
//	(host is shut down or otherwise not supplying power).  The USB firmware
// 	can then periodically poll this I/O pin to know when it is okay to turn on
//	the USB module/D+/D- pull up resistor.  When designing a purely bus powered
//	peripheral device, it is not possible to source current on D+ or D- when the
//	host is not actively providing power on Vbus. Therefore, implementing this
//	bus sense feature is optional.  This firmware can be made to use this bus
//	sense feature by making sure "USE_USB_BUS_SENSE_IO" has been defined in the
//	HardwareProfile.h file.    
    #if defined(USE_USB_BUS_SENSE_IO)
    tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
    #endif
    
//	If the host PC sends a GetStatus (device) request, the firmware must respond
//	and let the host know if the USB peripheral device is currently bus powered
//	or self powered.  See chapter 9 in the official USB specifications for details
//	regarding this request.  If the peripheral device is capable of being both
//	self and bus powered, it should not return a hard coded value for this request.
//	Instead, firmware should check if it is currently self or bus powered, and
//	respond accordingly.  If the hardware has been configured like demonstrated
//	on the PICDEM FS USB Demo Board, an I/O pin can be polled to determine the
//	currently selected power source.  On the PICDEM FS USB Demo Board, "RA2" 
//	is used for	this purpose.  If using this feature, make sure "USE_SELF_POWER_SENSE_IO"
//	has been defined in HardwareProfile.h, and that an appropriate I/O pin has been mapped
//	to it in HardwareProfile.h.
    #if defined(USE_SELF_POWER_SENSE_IO)
    tris_self_power = INPUT_PIN;	// See HardwareProfile.h
    #endif
    
	USBGenericOutHandle = 0;	
	USBGenericInHandle = 0;		

    UserInit();			//Application related initialization.  See user.c

    USBDeviceInit();	//usb_device.c.  Initializes USB module SFRs and firmware
    					//variables to known states.
}//end InitializeSystem



void UserInit(void)
{
	// Initialise port latches
	LATA = 0b00000000;
	LATB = 0b00000000;
	LATC = 0b00000000;
	LATD = 0b00000000;
	LATE = 0b00000000;
	LATF = 0b00000000;
	LATG = 0b00000000;
	LATH = 0b00000000;
	LATJ = 0b00000000;

	// Turn MCULED off
	PIN_MCULED = 1;
	// Stop FPGA configuration
	PIN_FCNCONF = 1;

	// Set up ports
	// PORTA: All open-circuit; set as output driving low
	TRISA = 0b00000000;
	// PORTB: ICSP on {7,6} O/P, PMP address load on {5,4}, rest n/c
	TRISB = 0b00000000;
	// PORTC: UART RX on 7, UART TX on 6, FPGA CFG data on 5, Data In on 4, CFG CLK on 3, rest n/c
	TRISC = 0b10010000;
	// PORTD: PMP data bus
	TRISD = 0b11111111;
	// PORTE: PMWR and PMRD at {1,0}, rest n/c
	TRISE = 0b00000000;
	// PORTF: USB D+ and D- on {4,3}, USB detect on 2, rest n/c
	TRISF = 0b00011100;
	// PORTG: nCONFIG on 0, rest n/c
	TRISG = 0b00000000;
	// PORTH: all n/c
	TRISH = 0b00000000;
	// PORTJ: MCULED on 6, BOOT on 5, FCDONE on 3, FCNSTAT on 2, rest n/c
	TRISJ = 0b00101100;

	// Enable SSP1 for FPGA microcode loading
	SSP1STAT = 0b01000000;		// CKE=1 (xmit on active->idle edge)
	SSP1CON1 = 0b00100000;		// CKP=0 (clock is idle-low / active-high)
								// and enable the MSSP

	// Configure the Parallel Master Port
#ifdef PMP_ADDR_16BIT
	PMCONH   = 0b00010011;		// PMP off, fully multiplexed, byte-enable off, RD/WR on
#else
	PMCONH   = 0b00001011;		// PMP off, partially multiplexed, byte-enable off, RD/WR on
#endif
	PMCONL   = 0b00100011;		// PMCS as ADDR[15:14], addr latch and RD/WR active-high
	PMMODEH  = 0b00000010;		// no interrupts, no increment, 8-bit, Master Mode 2
#ifdef PMP_SLOW
	PMMODEL  = 0xff;			// slowest possible PMP mode
#else
	PMMODEL  = 0b00000000;		// no additional wait states
#endif
	PMEH     = 0b00000000;		// PMAs are port I/O
	PMEL     = 0b00000011;		// PMAs are port I/O, PMALH and PMALL enabled
	PMSTATH  = 0;
	PMSTATL  = 0;
	PMCONHbits.PMPEN = 1;		// Enable the PMP
}//end UserInit

enum {
	FPGA_E_OK		= 0,
	FPGA_E_TIMEOUT	= -1
};

int fpga_config_start(void)
{
	long tm;

	// Prepare for reconfiguration
	PIN_FCDCLK = 0;
	PIN_FCDATA0 = 0;
	
	// Strobe nCONFIG low
	PIN_FCNCONF = 0;
	
	// Wait for nSTATUS to go low (ACK the CONFIG request)
	tm = GetInstructionClock() / 1000;		// 1ms timeout
	while ((PIN_FCNSTAT) && (tm-- > 0));
	if (tm == 0) return FPGA_E_TIMEOUT;
	
	// Raise nCONFIG again
	PIN_FCNCONF = 1;
	
	// Wait for nSTATUS to go high (end of POR delay)
	tm = GetInstructionClock() / 1000;		// 1ms timeout
	while ((!PIN_FCNSTAT) && (tm-- > 0));
	if (tm == 0) return FPGA_E_TIMEOUT;

	// FPGA now ready to be configured
	return FPGA_E_OK;
}

enum {
	CMD_NOP				= 0,
	CMD_FPGA_INIT		= 1,
	CMD_FPGA_LOAD		= 2,
	CMD_FPGA_POLL		= 3,
	CMD_FPGA_POKE		= 4,
	CMD_FPGA_PEEK		= 5,
	CMD_RAM_ADDR_SET	= 6,
	CMD_RAM_ADDR_GET	= 7,
	CMD_RAM_WRITE		= 8,
	CMD_RAM_READ		= 9,
	CMD_RESET			= 0xfb,
	CMD_SECRET_SQUIRREL	= 0xfc,
	CMD_PROGRAM_SERIAL	= 0xfd,
	CMD_BOOTLOADER		= 0xfe,
	CMD_GET_VERSION		= 0xff
};

enum {
	ERR_OK				= 0,		// Operation completed successfully
	ERR_HARDWARE_ERROR	= 1,		// Hardware error
	ERR_INVALID_LEN		= 2,		// Packet length byte invalid
	ERR_FPGA_NOT_CONF	= 3,		// FPGA not configured
	ERR_BAD_MAGIC		= 4,		// Bad magic word
	ERR_BAD_CHECKSUM	= 5			// Bad checksum
};

enum {
	R_SRAM_ADDR_LOW		= 0,
	R_SRAM_ADDR_HIGH	= 1,
	R_SRAM_ADDR_UPPER	= 2,
	R_SRAM_DATA			= 3,
	R_DRIVE_CONTROL		= 4,
	R_ACQCON			= 5,
	R_ACQ_START_EVT		= 6,
	R_ACQ_STOP_EVT		= 7,
	R_ACQ_START_NUM		= 8,
	R_ACQ_STOP_NUM		= 9,

	R_HSTMD_THR_START	= 0x10,
	R_HSTMD_THR_STOP	= 0x11,

	R_SYNCWORD_START_L	= 0x20,
	R_SYNCWORD_START_H	= 0x21,
	R_SYNCWORD_STOP_L	= 0x22,
	R_SYNCWORD_STOP_H	= 0x23,
	R_MASK_START_L		= 0x24,
	R_MASK_START_H		= 0x25,
	R_MASK_STOP_L		= 0x26,
	R_MASK_STOP_H		= 0x27,
	R_MFM_CLKSEL		= 0x2F,

	R_SCRATCHPAD		= 0x30,

	R_STEP_RATE			= 0xf0,
	R_STEP_COMMAND		= 0xff,

	R_MICROCODE_TYPE_L	= 0x04,
	R_MICROCODE_TYPE_H	= 0x05,
	R_MICROCODE_VER_L	= 0x06,
	R_MICROCODE_VER_H	= 0x07,
	R_STATUS1			= 0x0E,
	R_STATUS2			= 0x0F
};

#ifdef PMP_ADDR_16BIT
#define PMP_ADDR_SET(hi,lo)	{ PMADDRH = hi; PMADDRL = lo; }
#define PMP_ADDR_SETW(w)	{ PMADDRH = ((w) >> 8) & 0xff; PMADDRL = (w) & 0xff; }
#else
#define PMP_ADDR_SET(hi,lo)	{ PMADDRL = lo; }
#define PMP_ADDR_SETW(w)	{ PMADDRL = (w) & 0xff; }
#endif

//#define PMP_WRITE(x)		{ PMDIN1L = x; }

void PMP_WRITE(unsigned char x)
{
	// Wait for PMP to be idle
	while (PMMODEHbits.BUSY);
	// Send data byte
	PMDIN1L = x;
}

unsigned char PMP_READ(void)
{
	unsigned char i;

	// Initiate read
	i = PMDIN1L;
	// Wait for read completion
	while (PMMODEHbits.BUSY);
	// Return the data that the PMP read
	return PMDIN1L;
}

void LongDelay(void)
{
	unsigned char i;
	//A basic for() loop decrementing a 16 bit number would be simpler, but seems to take more code space for
	//a given delay.  So do this instead:	
	for(i = 0; i < 0xFF; i++)
	{
		WREG = 0xFF;
		while(WREG)
		{
			WREG--;
			_asm
			bra	0	//Equivalent to bra $+2, which takes half as much code as 2 nop instructions
			bra	0	//Equivalent to bra $+2, which takes half as much code as 2 nop instructions
			clrwdt
			nop
			_endasm	
		}
	}
	//Delay is ~59.8ms at 48MHz.	
}

/******************************************************************************
 * Function:        void ProcessIO(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is a place holder for other user routines.
 *                  It is a mixture of both USB and non-USB tasks.
 *
 * Note:            None
 *****************************************************************************/
void ProcessIO(void)
{
	unsigned int counter = 0;
	unsigned int i, j;
	unsigned long shifter;
	static char ramread_discard_first = 1;

    //User Application USB tasks below.
    //Note: The user application should not begin attempting to read/write over the USB
    //until after the device has been fully enumerated.  After the device is fully
    //enumerated, the USBDeviceState will be set to "CONFIGURED_STATE".
    if((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1)) return;
    
    //As the device completes the enumeration process, the USBCBInitEP() function will
    //get called.  In this function, we initialize the user application endpoints (in this
    //example code, the user application makes use of endpoint 1 IN and endpoint 1 OUT).
    //The USBGenRead() function call in the USBCBInitEP() function initializes endpoint 1 OUT
    //and "arms" it so that it can receive a packet of data from the host.  Once the endpoint
    //has been armed, the host can then send data to it (assuming some kind of application software
    //is running on the host, and the application software tries to send data to the USB device).
    
    //If the host sends a packet of data to the endpoint 1 OUT buffer, the hardware of the SIE will
    //automatically receive it and store the data at the memory location pointed to when we called
    //USBGenRead().  Additionally, the endpoint handle (in this case USBGenericOutHandle) will indicate
    //that the endpoint is no longer busy.  At this point, it is safe for this firmware to begin reading
    //from the endpoint buffer, and processing the data.  In this example, we have implemented a few very
    //simple commands.  For example, if the host sends a packet of data to the endpoint 1 OUT buffer, with the
    //first byte = 0x80, this is being used as a command to indicate that the firmware should "Toggle LED(s)".
    if(!USBHandleBusy(USBGenericOutHandle))		//Check if the endpoint has received any data from the host.
    {
		// Turn MCU LED on during command processing
		PIN_MCULED = 0;

		switch(OUTPacket[0]) {
			case CMD_NOP:			// NOP
				break;

			case CMD_FPGA_INIT:		// FPGA Load Begin
				if (fpga_config_start() != FPGA_E_OK) {
					INPacket[counter++] = ERR_HARDWARE_ERROR;
				} else {
					INPacket[counter++] = ERR_OK;
				}
				break;

			case CMD_FPGA_LOAD:		// FPGA Config Load
									// TODO: Speed this up! Allow longer packets!
				if (OUTPacket[1] > (USBGEN_EP_SIZE-2)) {
					INPacket[counter++] = ERR_INVALID_LEN;
					break;
				}

				// Second byte is the data length -- send the payload!
				for (i=0; i<OUTPacket[1]; i++) {
					// Clear MSSP shift register if necessary
					if (SSP1STATbits.BF) {
						char foo = SSP1BUF;
					}
					// Send data to MSSP
					SSP1BUF = OUTPacket[i+2];
				}

				// Response: success
				INPacket[counter++] = ERR_OK;
				break;

			case CMD_FPGA_POLL:		// FPGA Config State Poll
				if (PIN_FCDONE) {
					INPacket[counter++] = ERR_OK;
				} else {
					INPacket[counter++] = ERR_FPGA_NOT_CONF;
				}
				break;

			case CMD_FPGA_POKE:		// Write a value into an FPGA register
				// Load the address
				PMP_ADDR_SET(OUTPacket[1], OUTPacket[2]);
				// Initiate write
				PMP_WRITE(OUTPacket[3]);
				// Send response
				INPacket[counter++] = ERR_OK;
				break;

			case CMD_FPGA_PEEK:		// Read a value from an FPGA register
				// Load the address
				PMP_ADDR_SET(OUTPacket[1], OUTPacket[2]);
				// Initiate read
				INPacket[counter++] = ERR_OK;
				i = PMDIN1L;
				// Wait for read completion
				while (PMMODEHbits.BUSY);
				// Get data that the PMP read and store it in the output buffer
				INPacket[counter++] = PMDIN1L;
				break;

			case CMD_RAM_ADDR_SET:	// Set RAM address
				// Accepts: CMD_RAM_GETADDR  ramaddrU  ramaddrH  ramaddrL
				// Returns: status
				PMP_ADDR_SETW(R_SRAM_ADDR_LOW);
				PMP_WRITE(OUTPacket[1]);
				PMP_ADDR_SETW(R_SRAM_ADDR_HIGH);
				PMP_WRITE(OUTPacket[2]);
				PMP_ADDR_SETW(R_SRAM_ADDR_UPPER);
				PMP_WRITE(OUTPacket[3]);
				INPacket[counter++] = ERR_OK;
				// Next RAM read should discard the first byte it reads
				ramread_discard_first = 1;
				break;

			case CMD_RAM_ADDR_GET:	// Get RAM address
				// Accepts: CMD_RAM_GETADDR
				// Returns: status ramaddrU  ramaddrH  ramaddrL
				INPacket[counter++] = ERR_OK;
				PMP_ADDR_SETW(R_SRAM_ADDR_LOW);
				INPacket[counter++] = PMP_READ();
				PMP_ADDR_SETW(R_SRAM_ADDR_HIGH);
				INPacket[counter++] = PMP_READ();
				PMP_ADDR_SETW(R_SRAM_ADDR_UPPER);
				INPacket[counter++] = PMP_READ();
				break;

			case CMD_RAM_WRITE:		// Write a block of data to RAM
									// TODO: Speed this up! Allow longer packets!
				// Accepts: CMD_RAM_WRITE lenLo lenHi payload
				// Returns: status
				i = ((unsigned int)OUTPacket[2] << 8) + (unsigned int)OUTPacket[1];
				if (i > (USBGEN_EP_SIZE - 3)) {
					INPacket[counter++] = ERR_INVALID_LEN;
					break;
				}
				// Select RAM Access port
				PMP_ADDR_SETW(R_SRAM_DATA);
				// Write the payload data to the FPGA
				for (j=3; j<i+3; j++) {
					PMP_WRITE(OUTPacket[j]);
				}
				// Success!
				INPacket[counter++] = ERR_OK;
				break;

			case CMD_RAM_READ:		// Read a block of data from RAM and return it
									// TODO: Speed this up! Allow longer packets!
				// Accepts: CMD_RAM_READ  lenLo lenHi
				// Returns: status  payload
				i = ((unsigned int)OUTPacket[2] << 8) + (unsigned int)OUTPacket[1];
				if (i > (USBGEN_EP_SIZE - 1)) {
					INPacket[counter++] = ERR_INVALID_LEN;
					break;
				}
				// Select RAM Access port
				PMP_ADDR_SETW(R_SRAM_DATA);

				// RAM address recently set?
				if (ramread_discard_first) {
					// Flush the PMD read pipeline
					j = PMDIN1L;
					ramread_discard_first = 0;
				}

				// Read data from the FPGA
				for (j=1; j<i+1; j++) {
					INPacket[j] = PMDIN1L;
				}
				// Success!
				INPacket[0] = ERR_OK;
				counter = i+1;
				break;

			case CMD_RESET:				// Hard Reset
				if ((OUTPacket[1] == 0xDE) && (OUTPacket[2] == 0xAD) &&
					(OUTPacket[3] == 0xBE) && (OUTPacket[4] == 0xEF))
				{
					UCONbits.SUSPND = 0;		//Disable USB module
					UCON = 0x00;				//Disable USB module
					//And wait awhile for the USB cable capacitance to discharge down to disconnected (SE0) state. 
					//Otherwise host might not realize we disconnected/reconnected when we do the reset.
					LongDelay();
					Reset();
				} else {
					// Magic number invalid. Bah!
					INPacket[counter++] = ERR_BAD_MAGIC;
				}																

			case CMD_SECRET_SQUIRREL:	// Secret Squirrel Mode
				// Also known as "ATE Self Test". This allows us to remotely poke and prod
				// the I/Os without interference from the PMP and other peripherals. Core
				// peripherals like the USB bus are exempt :)
				// First turn the PMP off
				PMCONH = 0;
				PMCONL = 0;
				PMMODEH = 0;
				PMMODEL = 0;
				PMEH = 0;
				PMEL = 0;
				PMSTATH = 0;
				PMSTATL = 0;
				// Set port latches
				LATD = OUTPacket[1];
				LATB = (OUTPacket[2] & 0x03) << 4;
				LATE = 0x00;
				// Set TRIS bits
				TRISD = 0x00;
				TRISB = 0x00;
				TRISE = 0x01;	// RE0 used as data input, RE1 as clock
				i = 0;
				shifter = 0;
				do {
					// clock high
					LATEbits.LATE1 = 1;
					_asm nop _endasm;
					_asm nop _endasm;
					_asm nop _endasm;
					_asm nop _endasm;
					// clock low
					LATEbits.LATE1 = 0;
					// get data state
					_asm nop _endasm;
					_asm nop _endasm;
					_asm nop _endasm;
					_asm nop _endasm;
					if (PORTEbits.RE0) {
						shifter = (shifter << 1L) + 1;
					} else {
						shifter = (shifter << 1L);
					}
					i++;
					// break if we've clocked more than 4 times the buffer length
					if (i > 128) break;
				// need to shift in at least 20 bits, then see the shift marker
				} while (((shifter & 0x1FFFFFL) != 0x000001L) || (i < 20));

				// did we find the sync marker?
				if ((shifter & 0x1FFFFFL) != 0x000001L) {
					// no, hardware error :(
					INPacket[counter++] = ERR_HARDWARE_ERROR;
					break;
				}

				// sync mark found. get data bits.
				shifter = 0;
				for (i=0; i<10; i++) {
					// clock high
					LATEbits.LATE1 = 1;
					_asm nop _endasm;
					_asm nop _endasm;
					_asm nop _endasm;
					_asm nop _endasm;
					// clock low
					LATEbits.LATE1 = 0;
					// get data state
					_asm nop _endasm;
					_asm nop _endasm;
					_asm nop _endasm;
					_asm nop _endasm;
					if (PORTEbits.RE0) {
						shifter = (shifter << 1L) + 1;
					} else {
						shifter = (shifter << 1L);
					}
				}
				// return data
				INPacket[counter++] = ERR_OK;
				INPacket[counter++] = (shifter >> 8) & 0xFF;
				INPacket[counter++] = shifter & 0xff;
				break;

			case CMD_PROGRAM_SERIAL:	// Program Serial Number
				if ((OUTPacket[1] == 0xAC) && (OUTPacket[2] == 0xCE) &&
					(OUTPacket[3] == 0x55) && (OUTPacket[4] == 0xED))
				{
					// Do a single-word program for the serial number block
				} else {
					// Magic number invalid. Bah!
					INPacket[counter++] = ERR_BAD_MAGIC;
				}																

			case CMD_BOOTLOADER:	// Jump to bootloader
				if ((OUTPacket[1] == 0xB0) && (OUTPacket[2] == 0x07) &&
					(OUTPacket[3] == 0x10) && (OUTPacket[4] == 0xAD))
				{
					// Bootloader entry signature matches. Jump to the bootloader.
					INTCONbits.GIE = 0;
					_asm goto 0x001C _endasm;
				}
				break;

			case CMD_GET_VERSION:	// Read hardware/firmware/microcode version info
				INPacket[counter++] = ERR_OK;
				INPacket[counter++] = '0';			// Hardware rev
				INPacket[counter++] = 'I';
				INPacket[counter++] = '0';
				INPacket[counter++] = '6';
				INPacket[counter++] = 0x00;			// Firmware version hi
				INPacket[counter++] = 0x1A;			// Firmware version lo
				PMP_ADDR_SETW(R_MICROCODE_TYPE_H);	// Microcode type hi
				INPacket[counter++] = PMP_READ();
				PMP_ADDR_SETW(R_MICROCODE_TYPE_L);	// Microcode type lo
				INPacket[counter++] = PMP_READ();
				PMP_ADDR_SETW(R_MICROCODE_VER_H);	// Microcode version hi
				INPacket[counter++] = PMP_READ();
				PMP_ADDR_SETW(R_MICROCODE_VER_L);	// Microcode version lo
				INPacket[counter++] = PMP_READ();
				break;
		}

		// Send IN buffer data (if any)
		if (counter > 0) {
			USBGenericInHandle = USBGenWrite(USBGEN_EP_NUM, (BYTE*)&INPacket, counter);
		}

		// Turn MCU LED off again
		PIN_MCULED = 1;

        //Re-arm the OUT endpoint for the next packet:
	    //The USBGenRead() function call "arms" the endpoint (and makes it "busy").  If the endpoint is armed, the SIE will 
	    //automatically accept data from the host, if the host tries to send a packet of data to the endpoint.  Once a data 
	    //packet addressed to this endpoint is received from the host, the endpoint will no longer be busy, and the application
	    //can read the data which will be sitting in the buffer.
        USBGenericOutHandle = USBGenRead(USBGEN_EP_NUM,(BYTE*)&OUTPacket,USBGEN_EP_SIZE);
    }
}//end ProcessIO



// ******************************************************************************************************
// ************** USB Callback Functions ****************************************************************
// ******************************************************************************************************
// The USB firmware stack will call the callback functions USBCBxxx() in response to certain USB related
// events.  For example, if the host PC is powering down, it will stop sending out Start of Frame (SOF)
// packets to your device.  In response to this, all USB devices are supposed to decrease their power
// consumption from the USB Vbus to <2.5mA each.  The USB module detects this condition (which according
// to the USB specifications is 3+ms of no bus activity/SOF packets) and then calls the USBCBSuspend()
// function.  You should modify these callback functions to take appropriate actions for each of these
// conditions.  For example, in the USBCBSuspend(), you may wish to add code that will decrease power
// consumption from Vbus to <2.5mA (such as by clock switching, turning off LEDs, putting the
// microcontroller to sleep, etc.).  Then, in the USBCBWakeFromSuspend() function, you may then wish to
// add code that undoes the power saving things done in the USBCBSuspend() function.

// The USBCBSendResume() function is special, in that the USB stack will not automatically call this
// function.  This function is meant to be called from the application firmware instead.  See the
// additional comments near the function.

/******************************************************************************
 * Function:        void USBCBSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Call back that is invoked when a USB suspend is detected
 *
 * Note:            None
 *****************************************************************************/
void USBCBSuspend(void)
{
	//Example power saving code.  Insert appropriate code here for the desired
	//application behavior.  If the microcontroller will be put to sleep, a
	//process similar to that shown below may be used:
	
	//ConfigureIOPinsForLowPower();
	//SaveStateOfAllInterruptEnableBits();
	//DisableAllInterruptEnableBits();
	//EnableOnlyTheInterruptsWhichWillBeUsedToWakeTheMicro();	//should enable at least USBActivityIF as a wake source
	//Sleep();
	//RestoreStateOfAllPreviouslySavedInterruptEnableBits();	//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.
	//RestoreIOPinsToNormal();									//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.

	//IMPORTANT NOTE: Do not clear the USBActivityIF (ACTVIF) bit here.  This bit is 
	//cleared inside the usb_device.c file.  Clearing USBActivityIF here will cause 
	//things to not work as intended.	
	

    #if defined(__C30__)
    #if 0
        U1EIR = 0xFFFF;
        U1IR = 0xFFFF;
        U1OTGIR = 0xFFFF;
        IFS5bits.USB1IF = 0;
        IEC5bits.USB1IE = 1;
        U1OTGIEbits.ACTVIE = 1;
        U1OTGIRbits.ACTVIF = 1;
        Sleep();
    #endif
    #endif
}


/******************************************************************************
 * Function:        void _USB1Interrupt(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the USB interrupt bit is set
 *					In this example the interrupt is only used when the device
 *					goes to sleep when it receives a USB suspend command
 *
 * Note:            None
 *****************************************************************************/
#if 0
void __attribute__ ((interrupt)) _USB1Interrupt(void)
{
    #if !defined(self_powered)
        if(U1OTGIRbits.ACTVIF)
        {
            IEC5bits.USB1IE = 0;
            U1OTGIEbits.ACTVIE = 0;
            IFS5bits.USB1IF = 0;
        
            //USBClearInterruptFlag(USBActivityIFReg,USBActivityIFBitNum);
            USBClearInterruptFlag(USBIdleIFReg,USBIdleIFBitNum);
            //USBSuspendControl = 0;
        }
    #endif
}
#endif

/******************************************************************************
 * Function:        void USBCBWakeFromSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The host may put USB peripheral devices in low power
 *					suspend mode (by "sending" 3+ms of idle).  Once in suspend
 *					mode, the host may wake the device back up by sending non-
 *					idle state signalling.
 *					
 *					This call back is invoked when a wakeup from USB suspend 
 *					is detected.
 *
 * Note:            None
 *****************************************************************************/
void USBCBWakeFromSuspend(void)
{
	// If clock switching or other power savings measures were taken when
	// executing the USBCBSuspend() function, now would be a good time to
	// switch back to normal full power run mode conditions.  The host allows
	// a few milliseconds of wakeup time, after which the device must be 
	// fully back to normal, and capable of receiving and processing USB
	// packets.  In order to do this, the USB module must receive proper
	// clocking (IE: 48MHz clock must be available to SIE for full speed USB
	// operation).
}

/********************************************************************
 * Function:        void USBCB_SOF_Handler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB host sends out a SOF packet to full-speed
 *                  devices every 1 ms. This interrupt may be useful
 *                  for isochronous pipes. End designers should
 *                  implement callback routine as necessary.
 *
 * Note:            None
 *******************************************************************/
void USBCB_SOF_Handler(void)
{
    // No need to clear UIRbits.SOFIF to 0 here.
    // Callback caller is already doing that.
}

/*******************************************************************
 * Function:        void USBCBErrorHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The purpose of this callback is mainly for
 *                  debugging during development. Check UEIR to see
 *                  which error causes the interrupt.
 *
 * Note:            None
 *******************************************************************/
void USBCBErrorHandler(void)
{
    // No need to clear UEIR to 0 here.
    // Callback caller is already doing that.

	// Typically, user firmware does not need to do anything special
	// if a USB error occurs.  For example, if the host sends an OUT
	// packet to your device, but the packet gets corrupted (ex:
	// because of a bad connection, or the user unplugs the
	// USB cable during the transmission) this will typically set
	// one or more USB error interrupt flags.  Nothing specific
	// needs to be done however, since the SIE will automatically
	// send a "NAK" packet to the host.  In response to this, the
	// host will normally retry to send the packet again, and no
	// data loss occurs.  The system will typically recover
	// automatically, without the need for application firmware
	// intervention.
	
	// Nevertheless, this callback function is provided, such as
	// for debugging purposes.
}


/*******************************************************************
 * Function:        void USBCBCheckOtherReq(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        When SETUP packets arrive from the host, some
 * 					firmware must process the request and respond
 *					appropriately to fulfill the request.  Some of
 *					the SETUP packets will be for standard
 *					USB "chapter 9" (as in, fulfilling chapter 9 of
 *					the official USB specifications) requests, while
 *					others may be specific to the USB device class
 *					that is being implemented.  For example, a HID
 *					class device needs to be able to respond to
 *					"GET REPORT" type of requests.  This
 *					is not a standard USB chapter 9 request, and 
 *					therefore not handled by usb_device.c.  Instead
 *					this request should be handled by class specific 
 *					firmware, such as that contained in usb_function_hid.c.
 *
 * Note:            None
 *****************************************************************************/
void USBCBCheckOtherReq(void)
{
}//end


/*******************************************************************
 * Function:        void USBCBStdSetDscHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USBCBStdSetDscHandler() callback function is
 *					called when a SETUP, bRequest: SET_DESCRIPTOR request
 *					arrives.  Typically SET_DESCRIPTOR requests are
 *					not used in most applications, and it is
 *					optional to support this type of request.
 *
 * Note:            None
 *****************************************************************************/
void USBCBStdSetDscHandler(void)
{
    // Must claim session ownership if supporting this request
}//end


/******************************************************************************
 * Function:        void USBCBInitEP(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the device becomes
 *                  initialized, which occurs after the host sends a
 * 					SET_CONFIGURATION (wValue not = 0) request.  This 
 *					callback function should initialize the endpoints 
 *					for the device's usage according to the current 
 *					configuration.
 *
 * Note:            None
 *****************************************************************************/
void USBCBInitEP(void)
{
    USBEnableEndpoint(USBGEN_EP_NUM,USB_OUT_ENABLED|USB_IN_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);
    USBGenericOutHandle = USBGenRead(USBGEN_EP_NUM,(BYTE*)&OUTPacket,USBGEN_EP_SIZE);
}

/********************************************************************
 * Function:        void USBCBSendResume(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB specifications allow some types of USB
 * 					peripheral devices to wake up a host PC (such
 *					as if it is in a low power suspend to RAM state).
 *					This can be a very useful feature in some
 *					USB applications, such as an Infrared remote
 *					control	receiver.  If a user presses the "power"
 *					button on a remote control, it is nice that the
 *					IR receiver can detect this signalling, and then
 *					send a USB "command" to the PC to wake up.
 *					
 *					The USBCBSendResume() "callback" function is used
 *					to send this special USB signalling which wakes 
 *					up the PC.  This function may be called by
 *					application firmware to wake up the PC.  This
 *					function should only be called when:
 *					
 *					1.  The USB driver used on the host PC supports
 *						the remote wakeup capability.
 *					2.  The USB configuration descriptor indicates
 *						the device is remote wakeup capable in the
 *						bmAttributes field.
 *					3.  The USB host PC is currently sleeping,
 *						and has previously sent your device a SET 
 *						FEATURE setup packet which "armed" the
 *						remote wakeup capability.   
 *
 *					This callback should send a RESUME signal that
 *                  has the period of 1-15ms.
 *
 * Note:            Interrupt vs. Polling
 *                  -Primary clock
 *                  -Secondary clock ***** MAKE NOTES ABOUT THIS *******
 *                   > Can switch to primary first by calling USBCBWakeFromSuspend()
 
 *                  The modifiable section in this routine should be changed
 *                  to meet the application needs. Current implementation
 *                  temporary blocks other functions from executing for a
 *                  period of 1-13 ms depending on the core frequency.
 *
 *                  According to USB 2.0 specification section 7.1.7.7,
 *                  "The remote wakeup device must hold the resume signaling
 *                  for at lest 1 ms but for no more than 15 ms."
 *                  The idea here is to use a delay counter loop, using a
 *                  common value that would work over a wide range of core
 *                  frequencies.
 *                  That value selected is 1800. See table below:
 *                  ==========================================================
 *                  Core Freq(MHz)      MIP         RESUME Signal Period (ms)
 *                  ==========================================================
 *                      48              12          1.05
 *                       4              1           12.6
 *                  ==========================================================
 *                  * These timing could be incorrect when using code
 *                    optimization or extended instruction mode,
 *                    or when having other interrupts enabled.
 *                    Make sure to verify using the MPLAB SIM's Stopwatch
 *                    and verify the actual signal on an oscilloscope.
 *******************************************************************/
void USBCBSendResume(void)
{
    static WORD delay_count;
    
    USBResumeControl = 1;                // Start RESUME signaling
    
    delay_count = 1800U;                // Set RESUME line for 1-13 ms
    do
    {
        delay_count--;
    }while(delay_count);
    USBResumeControl = 0;
}


/*******************************************************************
 * Function:        BOOL USER_USB_CALLBACK_EVENT_HANDLER(
 *                        USB_EVENT event, void *pdata, WORD size)
 *
 * PreCondition:    None
 *
 * Input:           USB_EVENT event - the type of event
 *                  void *pdata - pointer to the event data
 *                  WORD size - size of the event data
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called from the USB stack to
 *                  notify a user application that a USB event
 *                  occured.  This callback is in interrupt context
 *                  when the USB_INTERRUPT option is selected.
 *
 * Note:            None
 *******************************************************************/
BOOL USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, void *pdata, WORD size)
{
    switch(event)
    {
        case EVENT_CONFIGURED: 
            USBCBInitEP();
            break;
        case EVENT_SET_DESCRIPTOR:
            USBCBStdSetDscHandler();
            break;
        case EVENT_EP0_REQUEST:
            USBCBCheckOtherReq();
            break;
        case EVENT_SOF:
            USBCB_SOF_Handler();
            break;
        case EVENT_SUSPEND:
            USBCBSuspend();
            break;
        case EVENT_RESUME:
            USBCBWakeFromSuspend();
            break;
        case EVENT_BUS_ERROR:
            USBCBErrorHandler();
            break;
        case EVENT_TRANSFER:
            Nop();
            break;
        default:
            break;
    }      
    return TRUE; 
}
/** EOF main.c ***************************************************************/
