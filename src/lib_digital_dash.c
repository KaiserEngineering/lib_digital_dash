/*************************************************************************************
* @title lib_digital_dash
*
* @copyright KaiserEngineering LLC
*
* @author Matthew Kaiser
*
* @compatibility DD-CAN-001 Rev A
* @description Library to communicate between the vehicle's ECU and Kaiser
* Engineering GUI software.
*
* @usage
* 
* 1) Create a file called digital_dash_config.h and define DIGITAL_DASH_CONFIG
*       1.1) Configure the hardware peripherals supported.
*       1.2) Configure the CAN bus communication.
*       1.3) Define the max packet size for the KE library
*
* 2) Create HAL functions for all the DIGITALDASH_CONFIG callbacks
*
* 3) Initialize all the callback pointers with digitaldash_init()
*
* 4) Start the Digital Dash with digialdash_start()
*
* 5) Call DigitalDash_Add_UART_byte() when a UART byte is received
*
* 6) Call DigitalDash_Add_CAN_Packet() when a CAN packet is received
*
*************************************************************************************/

#include "lib_digital_dash.h"

/* Define bit shifting macros */
#define BITSET(word,nbit)   ((word) |=  (1<<(nbit)))
#define BITCLEAR(word,nbit) ((word) &= ~(1<<(nbit)))
#define BITFLIP(word,nbit)  ((word) ^=  (1<<(nbit)))
#define BITCHECK(word,nbit) ((word) &   (1<<(nbit)))

/* Number of PIDs being streamed */
static volatile uint32_t num_pids = 0;

/* PID array containing all streamed data */
static PID_DATA stream[DD_MAX_PIDS];

/* Core application flags to track current state of hardware */
static volatile uint32_t app_flags = 0;

/* Current state of the Digital Dash */
static volatile DIGITALDASH_OPERATING_STATE state = DD_OP_OFF;

/* Current temperature of the Host */
static volatile uint8_t digitaldash_active_cooling = 0x00;

#ifdef LIB_KE_PROTOCOL_H_
/* Declare a KE packet manager */
static KE_PACKET_MANAGER rasp_pi;
#endif

#ifdef LIB_OBDII_H_
/* Declare an OBDII packet manager */
static OBDII_PACKET_MANAGER obdii;
#endif

#ifdef LIB_CAN_BUS_SNIFFER_H_
/* Declare a CAN Bus decode packet manager */
static CAN_DECODE_PACKET_MANAGER decode;
#endif

/* Configure the Digital Dash to sync the backlight with the vehicle's lighting */
#ifdef DECODE_GAUGE_BRIGHTNESS_SUPPORTED
static PID_DATA gauge_brightness_req = { .pid = DECODE_GAUGE_BRIGHTNESS, .mode = DECODE, .pid_unit = PID_UNITS_PERCENT, .pid_value = 100 };
static PTR_PID_DATA gauge_brightness;
#endif

/* Current LCD backlight brightness */
static uint8_t Brightness = 0;

/* Desired LCD backlight brightness */
static uint8_t Set_Brightness = LCD_MAX_BRIGHTNESS;

/* Non-blocking delay for the Digital Dash app */
static volatile uint32_t digitaldash_delay = 0;

/* Timer to track Digital Dash loss of comm. */
static volatile uint32_t digitaldash_app_wtchdg = 0xFFFFFFFF;

/* Timer to disable the LCD backlight if comm. stops */
static volatile uint32_t digitaldash_bklt_wtchdg = 0xFFFFFFFF;

/* Number of packets before enabling the LCD backlight */
#define KE_UART_THRESHOLD 40

/* Number of packets rx'd from the host */
static uint32_t ke_uart_count = 0;

/* Application callbacks */
DD_GET_SD_CARD_STATE get_sd_card_state    = null;
DD_KE_TX ke_tx                            = null;
DD_ECU_TX ecu_tx                          = null;
DD_SET_BACKLIGHT set_backlight            = null;
DD_FAN_CTRL fan                           = null;
DD_HOST_CTRL host                         = null;
DD_USB_CTRL usb                           = null;
DD_CAN_FILTER filter                      = null;

DIGITALDASH_INIT_STATUS DigitalDash_Config_Null_Check( void );

static void DigitalDash_Reset_PID( PTR_PID_DATA pid )
{
    pid->acquisition_type = PID_UNASSIGNED;
    pid->pid_value        = 0;
    pid->timestamp        = 0;
}

/* Clear ALL of the PIDs, this should only be called when the *
 * entire Digital Dash is reset                               */
static void DigitalDash_Reset_PID_Stream( void )
{
	num_pids = 0x00000000;

	for( uint8_t index = 0; index < DD_MAX_PIDS; index++ )
	{
		stream[index].pid = 0x00;
		stream[index].pid_unit = PID_UNITS_NOT_APPLICABLE;
		stream[index].base_unit = PID_UNITS_NOT_APPLICABLE;
		stream[index].acquisition_type = PID_UNASSIGNED;
		stream[index].pid_value = 0;
		stream[index].timestamp = 0;
	}
}

/* Add a new PID to the data stream, lib_digital_dash is the master, *
 * and will store the PID struct in its stream[] array. This         *
 * function will return the pointer that the requester must save in  *
 * order to pull the data. Lib_digital_dash will also decide what    *
 * library will be best to obtain the data. The library that is      *
 * assigned to data acquisition of the PID must also save the        *
 * pointer and update the value any time it has new data.            *
 * Lib_digital_dash will also be required to track the number of     *
 * requesters per PID to ensure no stream gets cut prematurely.      */
static PTR_PID_DATA DigitalDash_Add_PID_To_Stream( PTR_PID_DATA pid )
{
	/* Declare a NULL pointer */
	PTR_PID_DATA ptr = NULL;

	/* Iterate through every currently streamed PID and check if the *
	 * PID is being streamed                                         */
	for( uint8_t i = 0; i <= num_pids; i++ )
	{
		/* If so, return the pointer */
		if( stream[i].pid == pid->pid  &&
				stream[i].mode == pid->mode )
		{
			/* Increment the number of devices */
			stream[i].devices++;
			return &stream[i];
		}
	}

	/* Clear any data the PID has */
	DigitalDash_Reset_PID( pid );

	/* Copy the PID to the next available stream slot */
	stream[num_pids] = *pid;

	/* Increment the number of devices */
	stream[num_pids].devices++;

	/* Get the pointer of that slot */
	ptr = &stream[num_pids];

	/* Increment the number of PIDs */
	num_pids++;

	#ifdef LIB_CAN_BUS_SNIFFER_H_
	/* Add the PID to the decode stream if supported */
	if( CAN_Sniffer_Add_PID( &decode, ptr ) == PID_SUPPORTED ) {
		ptr->acquisition_type = PID_ASSIGNED_TO_CAN_SNIFFER;
		return ptr;
	}
	#endif

	#ifdef LIB_OBDII_H_
	/* Add the PID to the OBDII stream if supported */
	if( OBDII_add_PID_request( &obdii, ptr ) == OBDII_OK ) {
		ptr->acquisition_type = PID_ASSIGNED_TO_OBDII;
		return ptr;
	}
	#endif

	/* TODO: This should not be reached. For now, the data will just *
	 * never update.                                                 */
	return ptr;
}

/* Clear all variables except the function callbacks */
void DigitalDash_Reset_App( void )
{
    app_flags               = 0x00000000;
    state                   = DD_OP_OFF;
    Brightness              = 0x00;
    Set_Brightness          = LCD_MAX_BRIGHTNESS;
    digitaldash_delay       = 0x00000000;
    digitaldash_bklt_wtchdg = 0x00000000;
    digitaldash_app_wtchdg  = 0xFFFFFFFF;
    ke_uart_count           = 0x00000000;
    DigitalDash_Reset_PID_Stream();
    DigitalDash_Config_Null_Check();
}

#ifdef BKLT_CTRL_ACTIVE
/* Set the LCD brightness if needed */
static void Update_LCD_Brightness( uint8_t value )
{
	/* Verify enough packets have been rx'd */
	if( ke_uart_count < KE_UART_THRESHOLD )
		value = 0;

    /* Check if the brightness value needs to be update */
    if( Brightness != value )
    {
        /* Update the brightness */
        Brightness = value;

        /* The brightness has been set */
        Set_Brightness = value;

        /* Call the HAL function to update the brightness */
        set_backlight( Brightness );
    }
}

/* Reset the backlight watchdog and verify the LCD is on */
static void Refresh_LCD( void )
{
    /* Reset the backlight timeout */
    digitaldash_bklt_wtchdg = LCD_BKLT_TIMEOUT;

    /* Verify the LCD is at the desired brightness */
    Update_LCD_Brightness( Set_Brightness );
}
#endif

/* Update the application flags */
static void update_app_flag( DIGITALDASH_FLAG flag, uint8_t value )
{
    if( value )
        BITSET( app_flags, flag);
    else
        BITCLEAR( app_flags, flag);
}

/* The main application shall call this function to indicate *
 * when the SD card state changes                            */
void dd_update_sd_card_state( SD_CARD_STATE state )
{
    update_app_flag( DD_FLG_SD_CARD, state );
}

/* Return the Digital Dash application flags */
uint32_t digitaldash_get_app_flags( void )
{
    return app_flags;
}

/* Return a single Digital Dash flag */
uint8_t digitaldash_get_flag( DIGITALDASH_FLAG flag )
{
    return (( BITCHECK(app_flags, flag) == 0 ) ? 0 : 1);
}

void DigitalDash_Add_UART_byte( uint8_t byte )
{
    /* Reset the watchdog. The timeout value is the max time between frames */
    digitaldash_app_wtchdg = OS_FRAME_TIMEOUT;

    #ifdef BKLT_CTRL_ACTIVE
        Refresh_LCD();
    #endif

    if( num_pids > 0x00 )
    	ke_uart_count++;

	#ifdef LIB_KE_PROTOCOL_H_
    /* Add the UART byte to the KE packet manager */
    KE_Add_UART_Byte( &rasp_pi, byte );
	#endif
}

/* Copy the CAN packets to the relevant libraries */
void DigitalDash_Add_CAN_Packet( uint16_t id, uint8_t* data )
{
	#ifdef LIB_OBDII_H_
    OBDII_Add_Packet( &obdii, id, data );
	#endif

	#ifdef LIB_CAN_BUS_SNIFFER_H_
    CAN_Sniffer_Add_Packet( &decode, id, data );
	#endif
}

/* Callback to request active cooling, right now this is configured *
 * as a pass-through callback to main. But, logic can be added in   *
 * the future if desired.                                           */
void active_cooling( uint8_t level ) { fan( level ); }

DIGITALDASH_INIT_STATUS DigitalDash_Config_Null_Check( void )
{
    /* Clear the initialized flag */
    update_app_flag( DD_FLG_INIT, DD_NOT_INITIALIZED );

    if( get_sd_card_state == null )
        return DIGITALDASH_INIT_SD_PTR_ERROR;
    if( ke_tx == null )
        return DIGITALDASH_INIT_KE_PTR_ERROR;
    if( ecu_tx == null )
        return DIGITALDASH_INIT_ECU_PTR_ERROR;
	#ifdef BKLT_CTRL_ACTIVE
    if( set_backlight == null )
        return DIGITALDASH_INIT_BKLT_PTR_ERROR;
	#endif
    if( fan == null )
        return DIGITALDASH_INIT_FAN_PTR_ERROR;
    if( host == null )
        return DIGITALDASH_INIT_HOST_PTR_ERROR;
    if( usb == null )
        return DIGITALDASH_INIT_USB_PTR_ERROR;
    if( filter == null )
        return DIGITALDASH_INIT_CAN_FILT_PTR_ERROR;

    /* Set the intialized flag */
    update_app_flag( DD_FLG_INIT, DD_INITIALIZED );

    return DIGITALDASH_INIT_OK;
}

DIGITALDASH_INIT_STATUS digitaldash_init( PDIGITALDASH_CONFIG config )
{
	/* Reset the Digital Dash */
	DigitalDash_Reset_App();

    /* Clear the initialized flag */
    update_app_flag( DD_FLG_INIT, DD_NOT_INITIALIZED );

#ifdef SD_CARD_ACTIVE
    if( config->dd_get_sd_card_state == null )
        return DIGITALDASH_INIT_SD_PTR_ERROR;
    get_sd_card_state = config->dd_get_sd_card_state;
#endif

#ifdef KE_ACTIVE
    if( config->dd_ke_tx == null )
        return DIGITALDASH_INIT_KE_PTR_ERROR;
    ke_tx = config->dd_ke_tx;
#endif

#ifdef ECU_ACTIVE
    if( config->dd_ecu_tx == null )
        return DIGITALDASH_INIT_ECU_PTR_ERROR;
    ecu_tx = config->dd_ecu_tx;
#endif

#ifdef BKLT_CTRL_ACTIVE
    if( config->dd_set_backlight == null )
        return DIGITALDASH_INIT_BKLT_PTR_ERROR;
    set_backlight = config->dd_set_backlight;
#endif

#ifdef FAN_CTRL_ACTIVE
    if( config->dd_fan_ctrl == null )
        return DIGITALDASH_INIT_FAN_PTR_ERROR;
    fan = config->dd_fan_ctrl;
#endif

#ifdef HOST_CTRL_ACTIVE
    if( config->dd_host_ctrl == null )
        return DIGITALDASH_INIT_HOST_PTR_ERROR;
    host = config->dd_host_ctrl;
#endif

#ifdef USB_CTRL_ACTIVE
    if( config->dd_usb == null )
        return DIGITALDASH_INIT_USB_PTR_ERROR;
    usb = config->dd_usb;
#endif

#ifdef CAN_FILT_ACTIVE
    if( config->dd_filter == null )
        return DIGITALDASH_INIT_CAN_FILT_PTR_ERROR;
    filter = config->dd_filter;
#endif

    /* lib_ke_protocol initialization */
    rasp_pi.init.transmit = ke_tx;                                  /* Function call to transmit UART data to the host */
    rasp_pi.init.req_pid = &DigitalDash_Add_PID_To_Stream;          /* Function call to request a PID */
    rasp_pi.init.cooling = &active_cooling;
    rasp_pi.init.firmware_version_major  = FIRMWARE_VERSION_MAJOR;  /* Firmware version */
    rasp_pi.init.firmware_version_minor  = FIRMWARE_VERSION_MINOR;  /* Firmware version */
    rasp_pi.init.firmware_version_hotfix = FIRMWARE_VERSION_HOTFIX; /* Firmware version */

    /* Initialize the KE library */
    if( KE_Initialize( &rasp_pi ) != KE_OK )
        return DIGITALDASH_INIT_KE_INIT_ERROR;

    #ifdef LIB_OBDII_H_
    /* lib_obdii initialization */
    obdii.init.transmit       = ecu_tx;               /* Function call to transmit OBDII data to the vehicle */
    obdii.init.timeout        = ECU_TIMEOUT;          /* Time(ms) before lib_obdii will retry a transmission */
    obdii.init.arbitration_ID = ECU_TX_ID;            /* Transmit ID on the CAN bus */
    obdii.init.IDE            = OBDII_STD_IDE;        /* 11-bit or 29-bit identifier */

    /* Initialize the OBDII library */
    OBDII_Initialize( &obdii );
    #endif

    #ifdef USE_LIB_CAN_BUS_SNIFFER
    /* lib_can_bus_decode initialization */
    decode.filter = filter;
    CAN_Decode_Initialize(&decode);
    #endif

    #ifdef DECODE_GAUGE_BRIGHTNESS_SUPPORTED
    /* Start obtaining the gauge brightness */
    gauge_brightness = DigitalDash_Add_PID_To_Stream( &gauge_brightness_req );
    #endif

    /* Set the initialized flag */
    update_app_flag( DD_FLG_INIT, DD_INITIALIZED );

    return DIGITALDASH_INIT_OK;
}

static void host_power( HOST_PWR_STATE host_state )
{
    if( digitaldash_get_flag( DD_FLG_HOST_PWR ) != host_state )
    {
#if FORCE_USB_ON
        usb( USB_PWR_ENABLED );

        /* Indicate the new state of the host */
        update_app_flag( DD_USB_PWR, USB_PWR_ENABLED );
#else
        usb( host_state );

        /* Indicate the new state of the host */
        update_app_flag( DD_USB_PWR, host_state );
#endif

        /* Enable or disable power */
        host( host_state );

        /* Indicate the new state of the host */
        update_app_flag( DD_FLG_HOST_PWR, host_state );

        /* TODO: allow shutdown time */
       if( host_state == HOST_PWR_ENABLED ) {
    	   /* Power is enabled, and the host is booting */
    	   state = DD_OP_BOOTING;

    	   /* Set the boot time if power on */
		   digitaldash_app_wtchdg = OS_BOOT_TIME_MAX;
       } else {
    	   state = DD_OP_OFF;
       }
    }
}

static void DigitalDash_PowerCylce()
{
    /* Turn off the host */
    host_power( HOST_PWR_DISABLED );

    /* Let the power rails settle */
    digitaldash_delay = POWER_CYCLE_TIME;
}

DIGITALDASH_STATUS digitaldash_service( void )
{
    if( digitaldash_get_flag( DD_FLG_INIT ) == DD_INITIALIZED )
    {
        /* If a delay was requested by the Digital Dash application, block all other functions *
         * until the delay is complete. This will NOT block any other application code         */
        if( digitaldash_delay > 0 ) { /* Do nothing */ }
        
        /* First, check to see if the host is ready to boot by verifying the SD card is *
         * inserted. This only needs to be checked when the OS is on the SD card. If    *
         * the device has an EMMC, this check can be skipped.                           */
        #ifdef SD_CARD_ACTIVE
        else if( digitaldash_get_flag( DD_FLG_SD_CARD ) == SD_NOT_PRESENT )
            get_sd_card_state();
        #endif

        /* All hardware is present, so the Digital Dash is ready to be powered on.      *
         * Enable power to the host, and begin a timer to make sure the device properly *
         * boots and does not hang.                                                     */
        else if( digitaldash_get_flag( DD_FLG_HOST_PWR ) == HOST_PWR_DISABLED )
            host_power( HOST_PWR_ENABLED );

        /* If the application timer expires, reset the hardware                        */
        if( digitaldash_app_wtchdg <= 0 )
            DigitalDash_PowerCylce();

		#ifdef BKLT_CTRL_ACTIVE
        /* Turn off the LCD if no messages are received by LCD_BKLT_TIMEOUT */
        if( digitaldash_bklt_wtchdg <= 0 )
        {
        	/* Reset the UART count */
        	ke_uart_count = 0;

            Update_LCD_Brightness(0);
        } else {
            #ifdef DECODE_GAUGE_BRIGHTNESS_SUPPORTED
            Update_LCD_Brightness( (uint8_t)gauge_brightness->pid_value );
            #else
            Update_LCD_Brightness( LCD_MAX_BRIGHTNESS );
            #endif
        }
		#endif

        /* Service the KE protocol manager */
        KE_Service( &rasp_pi );

        #ifdef USE_LIB_OBDII
        /* Service the OBDII protocol manager */
        OBDII_Service( &obdii );
        #endif

        return DIGITALDASH_OK;
    }

    /* The Digital Dash has not been initialized yet. */
    else
    {
        return DIGITALDASH_NOT_INIT;
    }
}

void digitaldash_tick( void )
{
    if( digitaldash_delay > 0 )
        digitaldash_delay--;

    if( digitaldash_app_wtchdg > 0 )
        digitaldash_app_wtchdg--;

    if( digitaldash_bklt_wtchdg > 0 )
        digitaldash_bklt_wtchdg--;

    KE_tick();

    #ifdef USE_LIB_OBDII
    OBDII_tick();
    #endif

    #ifdef USE_LIB_CAN_BUS_SNIFFER
    CAN_Sniffer_tick();
    #endif
}
