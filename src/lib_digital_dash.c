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

#define EE_VERSION_UUID 68

/* Number of PIDs being streamed */
static volatile uint32_t num_pids = 0;

static uint8_t host_power_state = HOST_PWR_DISABLED;

/* PID array containing all streamed data */
static PID_DATA stream[DD_MAX_PIDS];

/* Core application flags to track current state of hardware */
static volatile uint32_t app_flags = 0;

/* Current state of the Digital Dash */
static volatile DIGITALDASH_OPERATING_STATE state = DD_OP_OFF;

#if USE_KE_PROTOCOL
/* Declare a KE packet manager */
static KE_PACKET_MANAGER coprocessor;
static uint8_t uart_tx_buffer[KE_MAX_TX_PAYLOAD] = {0};
static uint8_t uart_rx_buffer[KE_MAX_RX_PAYLOAD] = {0};
#endif

#if USE_LIB_OBDII
/* Declare an OBDII packet manager */
static OBDII_PACKET_MANAGER obdii;
#endif

#if USE_LIB_CAN_BUS_SNIFFER
/* Declare a CAN Bus sniffer packet manager */
static CAN_SNIFFER_PACKET_MANAGER sniffer;
#endif

#if USE_LIB_VEHICLE_DATA
/* Declare a Vehicle Data packet manager */
static VEHICLE_DATA_MANAGER vehicle;
#endif

/* Configure the Digital Dash to sync the backlight with the vehicle's lighting */
static PID_DATA gauge_brightness_req = { .pid_uuid = SNIFF_GAUGE_ILLUM_LEVEL_UUID, .pid_unit = PID_UNITS_NONE, .pid_value = 100 };
static PTR_PID_DATA gauge_brightness;

static PID_DATA engine_speed_req = { .pid_uuid = MODE1_ENGINE_SPEED_UUID, .pid_unit = PID_UNITS_RPM, .pid_value = 0 };
static PTR_PID_DATA engine_speed;

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

/* Timer to shutdown the Digital Dash */
#if ENABLE_WHEN_ENGINE_ON
static volatile uint32_t digitaldash_shutdown = 60000;
#else
static volatile uint32_t digitaldash_shutdown = 900000;
#endif

/* Timer to re-enable any communication that would effect a test device */
static volatile uint32_t tester_present = 0;

/* Number of packets before enabling the LCD backlight */
#define KE_UART_THRESHOLD 40

#if USE_KE_PROTOCOL
/* Number of packets rx'd from the host */
static uint32_t ke_uart_count = 0;
#endif

#if SPOOF_DATA
#define SPOOF_INTERVAL_T      25 // ms
static uint32_t spoof_count = 0;
#endif

/* Application callbacks */
#if SD_CARD_ACTIVE
DD_GET_SD_CARD_STATE get_sd_card_state    = NULL;
#endif
#if KE_ACTIVE
DD_KE_TX ke_tx                            = NULL;
#endif
#if ECU_ACTIVE
DD_ECU_TX ecu_tx                          = NULL;
#endif
#if BKLT_CTRL_ACTIVE
DD_SET_BACKLIGHT set_backlight            = NULL;
#endif
#if FAN_CTRL_ACTIVE
DD_FAN_CTRL fan                           = NULL;
#endif
#if HOST_CTRL_ACTIVE
DD_HOST_CTRL host                         = NULL;
#endif
#if USB_PWR_CTRL
DD_USB_CTRL usb                           = NULL;
#endif
#if HW_CAN_FILTERS
DD_CAN_FILTER filter                      = NULL;
#endif
#if BACKGROUND_IMG_SAVE
DD_BACKGROUND_IMG_SAVE background_save    = NULL;
#endif
#if SPLASH_OVERRIDE
DD_SPLASH_OVERRIDE splash_override        = NULL;
#endif
#if  BOOTLOADER_ACTIVATION
DD_BOOTLOADER_ACTIVATION bootloader_activate    = NULL;
#endif

DIGITALDASH_INIT_STATUS DigitalDash_Config_NULL_Check( void );
static void host_power( HOST_PWR_STATE host_state );

static uint32_t map(uint32_t in, uint32_t inMin, uint32_t inMax, uint32_t outMin, uint32_t outMax)
{
    return (((in - inMin)*(outMax - outMin))/(inMax - inMin)) + outMin;
}

static void DigitalDash_Reset_PID( PTR_PID_DATA pid )
{
    pid->acquisition_type = PID_UNASSIGNED;
    pid->pid_value        = 0;
    pid->timestamp        = 0;
    pid->pid_min          = INIT_MIN;
    pid->pid_max          = INIT_MAX;
    pid->devices          = 0;
}

/* Clear ALL of the PIDs, this should only be called when the *
 * entire Digital Dash is reset                               */
static void DigitalDash_Reset_PID_Stream( void )
{
	num_pids = 0x00000000;

	for( uint8_t index = 0; index < DD_MAX_PIDS; index++ )
	    lib_pid_clear_PID( &stream[index] );
}

static int DigitalDash_Remove_PID_From_Stream( PTR_PID_DATA pid )
{
    /* Iterate through every currently streamed PID and check if the *
     * PID is being streamed                                         */
    for( uint8_t index = 0; index < DD_MAX_PIDS; index++ )
    {
        /* If so, return a 1 */
        if( &stream[index] == pid )
        {
            /* Decrement the number of devices */
            if( stream[index].devices > 0 )
                stream[index].devices--;

            /* Stop acquiring the PID data if no devices are         *
             * requesting data.                                      */
            if( stream[index].devices == 0 )
            {
                switch( stream[index].acquisition_type )
                {
                    #if USE_LIB_CAN_BUS_SNIFFER
                    /* Remove the PID to the sniffer if supported */
                    case PID_ASSIGNED_TO_CAN_SNIFFER:
                        CAN_Sniffer_Remove_PID( &sniffer, pid );
                        break;
                    #endif

                    #if USE_LIB_OBDII
                    /* Remove the PID to the OBDII stream if supported */
                    case PID_ASSIGNED_TO_OBDII:
                        OBDII_remove_PID_request( &obdii, pid );
                        break;
                    #endif

                    #if USE_LIB_VEHICLE_DATA
                    /* Remove the PID from the Vehicle stream */
                    case PID_ASSIGNED_TO_VEHICLE_DATA:
                        Vehicle_remove_PID_request( &vehicle, pid );
                        break;
                    #endif

                    default:
                        break;
                }

                /* Clear the PID, but DO NOT change the array order, other *
                 * libraries are referencing this data.                    */
                lib_pid_clear_PID( &stream[index] );

                num_pids--;
            }

            return 1;
        }
    }

    return 1;
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
PTR_PID_DATA DigitalDash_Add_PID_To_Stream( PTR_PID_DATA pid )
{
	/* Declare a NULL pointer */
	PTR_PID_DATA ptr = NULL;

	uint8_t slot = 0;

	for( slot = 0; slot < DD_MAX_PIDS; slot++)
	{
	    if( stream[slot].pid_uuid == PID_UNASSIGNED )
	        break;
	}

    // Check if we exceeded max PIDs
    if (slot >= DD_MAX_PIDS) {
        return NULL;  // No space available
    }

	/* Iterate through every currently streamed PID and check if the *
	 * PID is being streamed                                         */
	for( uint8_t i = 0; i < num_pids; i++ )
	{
		/* If so, return the pointer */
		if( stream[i].pid_uuid == pid->pid_uuid )
		{
			/* Increment the number of devices */
			stream[i].devices++;
			return &stream[i];
		}
	}

	/* Clear any data the PID has */
	DigitalDash_Reset_PID( pid );

	/* Copy the PID to the next available stream slot */
	stream[slot] = *pid;

	/* Increment the number of devices */
	stream[slot].devices++;

	/* Get the pointer of that slot */
	ptr = &stream[slot];

	/* Increment the number of PIDs */
	num_pids++;

	/* Update the PID with all relevant data */
	load_pid_data( ptr );

	#if USE_LIB_CAN_BUS_SNIFFER
	/* Add the PID to the sniffer if supported */
	if( CAN_Sniffer_Add_PID( &sniffer, ptr ) == PID_SUPPORTED ) {
		ptr->acquisition_type = PID_ASSIGNED_TO_CAN_SNIFFER;
		return ptr;
	}
	#endif

    #if USE_LIB_VEHICLE_DATA
    /* Service the Vehicle Data manager */
    if( Vehicle_add_parameter( &vehicle, ptr ) == VEHICLE_DATA_OK ) {
        ptr->acquisition_type = PID_ASSIGNED_TO_VEHICLE_DATA;
        return ptr;
    }
    #endif

	#if USE_LIB_OBDII
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
	#if USE_KE_PROTOCOL
    ke_uart_count           = 0x00000000;
	#endif
    DigitalDash_Reset_PID_Stream();
    DigitalDash_Config_NULL_Check();
}

#if BKLT_CTRL_ACTIVE
/* Set the LCD brightness if needed */
static void Update_LCD_Brightness( uint8_t value )
{
	#if USE_KE_PROTOCOL & DIGITALDASH_DATA_ACQ_ONLY
	/* Verify enough packets have been rx'd */
	if( ke_uart_count < KE_UART_THRESHOLD )
		value = 0;
	#endif

    /* Check if the brightness value needs to be update */
    if( Brightness != value )
    {
        /* Update the brightness */
        Brightness = value;

        /* The brightness has been set */
        Set_Brightness = value;

        /* Call the HAL function to update the brightness */
        if( set_backlight != NULL )
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

#if SD_CARD_ACTIVE
/* The main application shall call this function to indicate *
 * when the SD card state changes                            */
void dd_update_sd_card_state( SD_CARD_STATE state )
{
    update_app_flag( DD_FLG_SD_CARD, state );
}
#endif

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

    #if BKLT_CTRL_ACTIVE
        Refresh_LCD();
    #endif

	#if USE_KE_PROTOCOL
    if( num_pids > 0x00 )
    	ke_uart_count++;
	#endif

	#if USE_KE_PROTOCOL
    /* Add the UART byte to the KE packet manager */
    KE_Add_UART_Byte( &coprocessor, byte );
	#endif
}

/* Copy the CAN packets to the relevant libraries */
void DigitalDash_Add_CAN_Packet( uint16_t id, uint8_t* data )
{
	#if USE_LIB_OBDII
    OBDII_Add_Packet( &obdii, id, data );
	#endif

	#if USE_LIB_CAN_BUS_SNIFFER
    CAN_Sniffer_Add_Packet( &sniffer, id, data );
	#endif

    #if USE_LIB_OBDII
    /* TODO: 7E0 is the common tester ID, but others could be used */
    if( id == 0x7E0 )
    {
    	if( is_flow_control_frame(data) ) {} // Ignore flow control frame
    	else {
			tester_present = TESTER_PRESENT_DELAY;

			update_app_flag( DD_TESTER_PRESENT, TESTER_PRESENT );

			#if USE_LIB_OBDII
			OBDII_Pause( &obdii );
			#endif
    	}
    }
    #endif
}

/* Callback to request active cooling, right now this is configured *
 * as a pass-through callback to main. But, logic can be added in   *
 * the future if desired.                                           */
void active_cooling( uint8_t level ) {
#if FAN_CTRL_ACTIVE
	fan( level );
#endif
}

DIGITALDASH_INIT_STATUS DigitalDash_Config_NULL_Check( void )
{
    /* Clear the initialized flag */
    update_app_flag( DD_FLG_INIT, DD_NOT_INITIALIZED );

#if SD_CARD_ACTIVE
    if( get_sd_card_state == NULL )
        return DIGITALDASH_INIT_SD_PTR_ERROR;
#endif
#if KE_ACTIVE
    if( ke_tx == NULL )
        return DIGITALDASH_INIT_KE_PTR_ERROR;
#endif
#if ECU_ACTIVE
    if( ecu_tx == NULL )
        return DIGITALDASH_INIT_ECU_PTR_ERROR;
#endif
#if BKLT_CTRL_ACTIVE
    if( set_backlight == NULL )
        return DIGITALDASH_INIT_BKLT_PTR_ERROR;
#endif
#if FAN_CTRL_ACTIVE
    if( fan == NULL )
        return DIGITALDASH_INIT_FAN_PTR_ERROR;
#endif
#if HOST_CTRL_ACTIVE
    if( host == NULL )
        return DIGITALDASH_INIT_HOST_PTR_ERROR;
#endif
#if USB_PWR_CTRL
    if( usb == NULL )
        return DIGITALDASH_INIT_USB_PTR_ERROR;
#endif
#if HW_CAN_FILTERS
    if( filter == NULL )
        return DIGITALDASH_INIT_CAN_FILT_PTR_ERROR;
#endif

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

#if SD_CARD_ACTIVE
    if( config->dd_get_sd_card_state == NULL )
        return DIGITALDASH_INIT_SD_PTR_ERROR;
    get_sd_card_state = config->dd_get_sd_card_state;
#endif

#if KE_ACTIVE
    if( config->dd_ke_tx == NULL )
        return DIGITALDASH_INIT_KE_PTR_ERROR;
    ke_tx = config->dd_ke_tx;
#endif

#if ECU_ACTIVE
    if( config->dd_ecu_tx == NULL )
        return DIGITALDASH_INIT_ECU_PTR_ERROR;
    ecu_tx = config->dd_ecu_tx;
#endif

#if BKLT_CTRL_ACTIVE
    if( config->dd_set_backlight == NULL )
        return DIGITALDASH_INIT_BKLT_PTR_ERROR;
    set_backlight = config->dd_set_backlight;
#endif

#if FAN_CTRL_ACTIVE
    if( config->dd_fan_ctrl == NULL )
        return DIGITALDASH_INIT_FAN_PTR_ERROR;
    fan = config->dd_fan_ctrl;
#endif

#if HOST_CTRL_ACTIVE
    if( config->dd_host_ctrl == NULL )
        return DIGITALDASH_INIT_HOST_PTR_ERROR;
    host = config->dd_host_ctrl;
#endif

#if USB_CTRL_ACTIVE
    if( config->dd_usb == NULL )
        return DIGITALDASH_INIT_USB_PTR_ERROR;
    usb = config->dd_usb;
#endif

#if HW_CAN_FILTERS
    if( config->dd_filter == NULL )
        return DIGITALDASH_INIT_CAN_FILT_PTR_ERROR;
    filter = config->dd_filter;
#endif

#if BACKGROUND_IMG_SAVE
    if( config->dd_background_save == NULL )
        return DIGITALDASH_INIT_BACKGROUND_SAVE_PTR_ERROR;
    background_save = config->dd_background_save;
#endif

#if SPLASH_OVERRIDE
    if( config->dd_splash_override == NULL )
        return DIGITALDASH_INIT_SPLASH_OVERRIDE_PTR_ERROR;
    splash_override = config->dd_splash_override;
#endif

#if BOOTLOADER_ACTIVATION
    if( config->dd_bootloader_activate == NULL )
        return DIGITALDASH_INIT_BOOTLOADER_ACTIVATION_PTR_ERROR;
    bootloader_activate = config->dd_bootloader_activate;
#endif

#if USE_KE_PROTOCOL
    /* lib_ke_protocol initialization */
	#if DIGITALDASH_GRAPHICS_ONLY
    coprocessor.init.role      = KE_PRIMARY;
	#else
	coprocessor.init.role      = KE_SECONDARY;
	#endif
    coprocessor.init.transmit  = ke_tx;                                 /* Function call to transmit UART data to the coprocessor */
    coprocessor.init.req_pid   = &DigitalDash_Add_PID_To_Stream;        /* Function call to request a PID */
    coprocessor.init.clear_pid = &DigitalDash_Remove_PID_From_Stream;   /* Function call to remove a PID */
    coprocessor.init.cooling   = &active_cooling;                       /* Function call to request active cooling */
    coprocessor.init.config_to_json = &config_to_json;                  /* Function call to construct JSON of the config */
    coprocessor.init.json_to_config = &json_to_config;                  /* Function call to apply JSON data to the config */
    coprocessor.init.options_to_json = &options_to_json;                /* Function call to construct JSON of the option list */
    coprocessor.init.pid_list_to_json = &pid_list_to_json;
    coprocessor.init.get_rgba_crc = &calc_crc32;
    coprocessor.init.enter_bootloader = bootloader_activate;
    coprocessor.init.save_rgba = background_save;                       /* Function call save png bytes to storage */
    coprocessor.init.firmware_version_major  = FIRMWARE_VERSION_MAJOR;  /* Major firmware version */
    coprocessor.init.firmware_version_minor  = FIRMWARE_VERSION_MINOR;  /* Minor firmware version */
    coprocessor.init.firmware_version_hotfix = FIRMWARE_VERSION_HOTFIX; /* Hot fix firmware version */
    coprocessor.tx_buffer_size = KE_MAX_TX_PAYLOAD;
    coprocessor.rx_buffer_size = KE_MAX_RX_PAYLOAD;
    coprocessor.tx_buffer = uart_tx_buffer;
    coprocessor.rx_buffer = uart_rx_buffer;

    /* Initialize the KE library */
    if( KE_Initialize( &coprocessor ) != KE_OK )
        return DIGITALDASH_INIT_KE_INIT_ERROR;
#endif

#if USE_LIB_OBDII
    /* lib_obdii initialization */
    obdii.init.transmit       = ecu_tx;               /* Function call to transmit OBDII data to the vehicle */
    obdii.init.timeout        = ECU_TIMEOUT;          /* Time(ms) before lib_obdii will retry a transmission */
    obdii.init.arbitration_ID = ECU_TX_ID;            /* Transmit ID on the CAN bus */
    obdii.init.IDE            = OBDII_STD_IDE;        /* 11-bit or 29-bit identifier */

    /* Initialize the OBDII library */
    OBDII_Initialize( &obdii );
#endif

#if USE_LIB_CAN_BUS_SNIFFER
    /* lib_can_bus_sniffer initialization */
    sniffer.filter = filter;
    CAN_Sniffer_Initialize(&sniffer);
#endif

#if USE_LIB_VEHICLE_DATA
    vehicle.req_pid   = &DigitalDash_Add_PID_To_Stream;        /* Function call to request a PID */
    vehicle.clear_pid = &DigitalDash_Remove_PID_From_Stream;   /* Function call to remove a PID */

    Vehicle_Init( &vehicle );
#endif

    /* Start obtaining the gauge brightness */
    gauge_brightness = DigitalDash_Add_PID_To_Stream( &gauge_brightness_req );

    engine_speed = DigitalDash_Add_PID_To_Stream( &engine_speed_req );

    /* Set the initialized flag */
    update_app_flag( DD_FLG_INIT, DD_INITIALIZED );

    return DIGITALDASH_INIT_OK;
}

static void host_power( HOST_PWR_STATE host_state )
{
    if( host_power_state != host_state )
    {
#if USB_PWR_CTRL
#if FORCE_USB_ON
        usb( USB_PWR_ENABLED );

        /* Indicate the new state of the host */
        update_app_flag( DD_USB_PWR, USB_PWR_ENABLED );
#else
        if( host_state == HOST_PWR_ENABLED )
            usb( USB_PWR_ENABLED );
        else
            usb( USB_PWR_DISABLED );
#endif
#endif

#if HOST_CTRL_ACTIVE
        /* Enable or disable power */
        host( host_state );
#endif

        /* Indicate the new state of the host */
        host_power_state = host_state;

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

static void default_config(void)
{
	// Set splash screen to 5s
	set_general_splash(0, 5, true);

	// View 0
	#if MAX_VIEWS >= 1
	set_view_enable(0, VIEW_STATE_ENABLED, true);
	set_view_num_gauges(0, 3, true);
	set_view_background(0, VIEW_BACKGROUND_USER1, true);
	set_view_gauge_theme(0, 0, GAUGE_THEME_STOCK_RS, true);
	set_view_gauge_theme(0, 1, GAUGE_THEME_STOCK_RS, true);
	set_view_gauge_theme(0, 2, GAUGE_THEME_STOCK_RS, true);

	set_view_gauge_pid(0, 0, MODE1_INTAKE_AIR_TEMP_UUID, true);
	set_view_gauge_units(0, 0, PID_UNITS_FAHRENHEIT, true);

	set_view_gauge_pid(0, 1, CALC1_BOOST_VACUUM_UUID, true);
	set_view_gauge_units(0, 1, PID_UNITS_PSI, true);

	set_view_gauge_pid(0, 2, MODE1_OIL_TEMP_UUID, true);
	set_view_gauge_units(0, 2, PID_UNITS_FAHRENHEIT, true);
	#endif

	// View 1
	#if MAX_VIEWS >= 2
	set_view_enable(1, VIEW_STATE_ENABLED, true);
	set_view_num_gauges(1, 1, true);
	set_view_background(1, VIEW_BACKGROUND_USER1, true);
	set_view_gauge_theme(1, 0, GAUGE_THEME_LINEAR, true);
	set_view_gauge_theme(1, 1, GAUGE_THEME_LINEAR, true);
	set_view_gauge_theme(1, 2, GAUGE_THEME_LINEAR, true);

	set_view_gauge_pid(1, 0, CALC1_BOOST_VACUUM_UUID, true);
	set_view_gauge_units(1, 0, PID_UNITS_PSI, true);

	set_view_gauge_pid(1, 1, MODE1_INTAKE_AIR_TEMP_UUID, true);
	set_view_gauge_units(1, 1, PID_UNITS_FAHRENHEIT, true);

	set_view_gauge_pid(1, 2, MODE1_OIL_TEMP_UUID, true);
	set_view_gauge_units(1, 2, PID_UNITS_FAHRENHEIT, true);
	#endif

	// View 2
	#if MAX_VIEWS >= 3
	set_view_enable(2, VIEW_STATE_DISABLED, true);
	set_view_num_gauges(2, 3, true);
	set_view_background(2, VIEW_BACKGROUND_USER1, true);
	set_view_gauge_theme(2, 0, GAUGE_THEME_STOCK_RS, true);
	set_view_gauge_theme(2, 1, GAUGE_THEME_STOCK_RS, true);
	set_view_gauge_theme(2, 2, GAUGE_THEME_STOCK_RS, true);

	set_view_gauge_pid(2, 0, MODE1_INTAKE_AIR_TEMP_UUID, true);
	set_view_gauge_units(2, 0, PID_UNITS_FAHRENHEIT, true);

	set_view_gauge_pid(2, 1, CALC1_BOOST_VACUUM_UUID, true);
	set_view_gauge_units(2, 1, PID_UNITS_PSI, true);

	set_view_gauge_pid(2, 2, MODE1_OIL_TEMP_UUID, true);
	set_view_gauge_units(2, 2, PID_UNITS_FAHRENHEIT, true);
	#endif

	// Dynamic 0
	#if MAX_DYNAMICS >= 1
	set_dynamic_enable(0, DYNAMIC_STATE_ENABLED, true);
	set_dynamic_pid(0, CALC1_CRUISE_CONTROL_OFF_BUTTON_TOGGLE_UUID, true);
	set_dynamic_units(0, PID_UNITS_NONE, true);
	set_dynamic_priority(0, DYNAMIC_PRIORITY_HIGH, true);
	set_dynamic_compare(0, DYNAMIC_COMPARISON_GREATER_THAN, true);
	set_dynamic_threshold(0, 0, true);
	set_dynamic_view_index(0, 1, true);
	#endif

	// Dynamic 1
	#if MAX_DYNAMICS >= 2
	set_dynamic_enable(1, DYNAMIC_STATE_DISABLED, true);
	set_dynamic_pid(1, CALC1_BOOST_VACUUM_UUID, true);
	set_dynamic_units(1, PID_UNITS_PSI, true);
	set_dynamic_priority(1, DYNAMIC_PRIORITY_MEDIUM, true);
	set_dynamic_compare(1, DYNAMIC_COMPARISON_GREATER_THAN, true);
	set_dynamic_threshold(1, 10, true);
	set_dynamic_view_index(1, 1, true);
	#endif

	// Dynamic 2
	#if MAX_DYNAMICS >= 3
	set_dynamic_enable(2, DYNAMIC_STATE_ENABLED, true);
	set_dynamic_pid(2, CALC1_BOOST_VACUUM_UUID, true);
	set_dynamic_units(2, PID_UNITS_PSI, true);
	set_dynamic_priority(2, DYNAMIC_PRIORITY_LOW, true);
	set_dynamic_compare(2, DYNAMIC_COMPARISON_GREATER_THAN, true);
	set_dynamic_threshold(2, 10, true);
	set_dynamic_view_index(2, 0, true);
	#endif

	char msg[ALERT_MESSAGE_LEN] = "Alert";

	// Alert 0
	#if MAX_ALERTS >= 1
	set_alert_enable(0, ALERT_STATE_DISABLED, true );
	set_alert_pid(0, MODE1_ENGINE_SPEED_UUID, true );
	set_alert_units(0, PID_UNITS_RPM, true );
	set_alert_compare(0, ALERT_COMPARISON_GREATER_THAN_OR_EQUAL_TO, true );
	set_alert_threshold(0, 6500, true );
	snprintf(msg, ALERT_MESSAGE_LEN, "Exceeded Redline!");
	set_alert_message(0, msg, true);
	#endif

	// Alert 1
	#if MAX_ALERTS >= 2
	set_alert_enable(1, ALERT_STATE_DISABLED, true );
	set_alert_pid(1, MODE1_OIL_TEMP_UUID, true );
	set_alert_units(1, PID_UNITS_FAHRENHEIT, true );
	set_alert_compare(1, ALERT_COMPARISON_GREATER_THAN_OR_EQUAL_TO, true );
	set_alert_threshold(1, 300, true );
	snprintf(msg, ALERT_MESSAGE_LEN, "Oil Overtemp!");
	set_alert_message(1, msg, true);
	#endif

	// Alert 2 - MAX_ALERTS
	#if MAX_ALERTS >= 3
	for( uint8_t i = 2; i < MAX_ALERTS; i++)
	{
		set_alert_enable(i, ALERT_STATE_DISABLED, true );
		set_alert_pid(i, MODE1_OIL_TEMP_UUID, true );
		set_alert_units(i, PID_UNITS_FAHRENHEIT, true );
		set_alert_compare(i, ALERT_COMPARISON_GREATER_THAN_OR_EQUAL_TO, true );
		set_alert_threshold(i, 300, true );
		snprintf(msg, ALERT_MESSAGE_LEN, "Alert");
		set_alert_message(i, msg, true);
	}
	#endif

	// Update the EE version
	set_general_ee_version(0, EE_VERSION_UUID, true);
}

DIGITALDASH_STATUS digitaldash_service( void )
{
    if( digitaldash_get_flag( DD_FLG_INIT ) == DD_INITIALIZED )
    {
        /* If a delay was requested by the Digital Dash application, block all other functions *
         * until the delay is complete. This will NOT block any other application code         */
        if( digitaldash_delay > 0 ) {
        	if( digitaldash_get_flag( DD_GUI_ACTIVE ) == GUI_IS_ACTIVE )
				ui_service();
        }

        /* Turn off the host */
        else if( (digitaldash_shutdown <= 0) &&
                (host_power_state == HOST_PWR_ENABLED) )
        {
			#if SAFE_SHUTDOWN
            host_power( HOST_PWR_SLEEP );
			#else
        	host_power( HOST_PWR_DISABLED );
			#endif
        }

		#if SD_CARD_ACTIVE
        /* First, check to see if the host is ready to boot by verifying the SD card is *
         * inserted. This only needs to be checked when the OS is on the SD card. If    *
         * the device has an EMMC, this check can be skipped.                           */
        else if( digitaldash_get_flag( DD_FLG_SD_CARD ) == SD_NOT_PRESENT )
            get_sd_card_state();
		#endif

        /* All hardware is present, so the Digital Dash is ready to be powered on.      *
         * Enable power to the host, and begin a timer to make sure the device properly *
         * boots and does not hang.                                                     */
        else if( (digitaldash_shutdown > 0) &&
                (host_power_state != HOST_PWR_ENABLED) ) {
            host_power( HOST_PWR_ENABLED );
			#if FAN_CTRL_ACTIVE
            fan( FAN_MED );
			#endif
        }

        /* If the application timer expires, reset the hardware                        */
        else if( digitaldash_app_wtchdg <= 0 )
            DigitalDash_PowerCylce();

		#if DIGITALDASH_GRAPHICS
		else if( digitaldash_get_flag( DD_SETTINGS_LOADED ) == SETTINGS_NOT_LOADED ) {
	        // Load all settings from EEPROM
	        load_settings();

	        if( get_general_ee_version(0) != EE_VERSION_UUID )
	      	  default_config();

			update_app_flag( DD_SETTINGS_LOADED, SETTINGS_LOADED );
		}

		else if( digitaldash_get_flag( DD_GUI_ACTIVE ) == GUI_IS_INACTIVE ) {
			build_ui();
			if( splash_override() )
				skip_splash();

			digitaldash_delay = 50;

			update_app_flag( DD_GUI_ACTIVE, GUI_IS_ACTIVE );
		}

		#endif

        else {
			#if USE_KE_PROTOCOL
            /* Service the KE protocol manager */
            KE_Service( &coprocessor );
			#endif

			#if USE_LIB_OBDII
            /* Service the OBDII protocol manager */
            OBDII_Service( &obdii );
			#endif

			#if USE_LIB_VEHICLE_DATA
            /* Service the Vehicle Data manager */
            Vehicle_service( &vehicle );
			#endif

			#if DIGITALDASH_GRAPHICS
            ui_service();
			#endif

			#if BKLT_CTRL_ACTIVE
			#if USE_KE_PROTOCOL & DIGITALDASH_DATA_ACQ_ONLY
			/* Turn off the LCD if no messages are received by LCD_BKLT_TIMEOUT */
			if( digitaldash_bklt_wtchdg <= 0 )
			{

				/* Reset the UART count */
				ke_uart_count = 0;


				Update_LCD_Brightness(0);
			} else {
			#endif
				uint8_t brightness_adjusted = (uint8_t)gauge_brightness->pid_value;

				// Clamp the ford backlight value
				if(brightness_adjusted >= FORD_MAX_DAY_BRIGHTNESS)
					brightness_adjusted = FORD_MAX_DAY_BRIGHTNESS;
				else if (brightness_adjusted <= FORD_MIN_NIGHT_BRIGHTNESS)
					brightness_adjusted = FORD_MIN_NIGHT_BRIGHTNESS;

				// During night use the ford value w/clamping
				// During the day, scale to 100% minus 5% each tick below max
				if(brightness_adjusted >= FORD_MIN_DAY_BRIGHTNESS)
					brightness_adjusted = LCD_MAX_BRIGHTNESS - ((FORD_MAX_DAY_BRIGHTNESS - brightness_adjusted)*5);

				// Clamp the brightness to the LCD limita
				if(brightness_adjusted <= LCD_MIN_BRIGHTNESS)
					brightness_adjusted = LCD_MIN_BRIGHTNESS;
				else if (brightness_adjusted >= LCD_MAX_BRIGHTNESS)
					brightness_adjusted = LCD_MAX_BRIGHTNESS;

				/* Default to max brightness if no data has been RX'd */
				if( gauge_brightness->timestamp == 0 )
					brightness_adjusted = LCD_MAX_BRIGHTNESS;

				if( digitaldash_get_flag( DD_GUI_ACTIVE ) )
					Update_LCD_Brightness( brightness_adjusted );
			#if USE_KE_PROTOCOL & DIGITALDASH_DATA_ACQ_ONLY
			}
			#endif
			#else
			Update_LCD_Brightness( LCD_MAX_BRIGHTNESS );
			#endif
        }

        /*
        if( (engine_speed->pid_value >= 500) )
            digitaldash_shutdown = ENGINE_OFF_SHUTDOWN_TIME;
            */

		return DIGITALDASH_OK;
    }

    /* The Digital Dash has not been initialized yet. */
    else
    {
        return DIGITALDASH_NOT_INIT;
    }
}

#if SPOOF_DATA
float engine_rpm = 900;
float turbo = 0;
float oil_temp = 0;
float baro = 101.4;
float pid_map = 0;
#define TEMP_VARIATION_RANGE 10.0f
#endif

void digitaldash_tick( void )
{
    #if SPOOF_DATA
    spoof_count = (spoof_count + 1) % SPOOF_INTERVAL_T;
    if( spoof_count == 0 )
    {
        for( uint8_t i = 0; i < DD_MAX_PIDS; i++ )
        {
            if( stream[i].pid_uuid == MODE1_ENGINE_SPEED_UUID )
            {
                stream[i].timestamp++;
                engine_rpm += 10;
                stream[i].pid_value = engine_rpm;
                if( engine_rpm >= 8000 )
                    engine_rpm = 900;
            } else if ( stream[i].pid_uuid == MODE1_BOOST_UUID )
            {
                stream[i].timestamp++;
                turbo += 0.5;
                stream[i].pid_value = turbo;
                if( turbo >= 255 )
                    turbo = 0;
            } else if ( stream[i].pid_uuid == MODE1_OIL_TEMP_UUID )
            {
                stream[i].timestamp++;

                // Generate a random float between -5 and +5
                float variation = ((float)(rand() % (int)(TEMP_VARIATION_RANGE * 20 + 1)) / 10.0f) - TEMP_VARIATION_RANGE;

                oil_temp += variation;
                stream[i].pid_value = oil_temp;

                float middle = (stream[i].upper_limit - stream[i].lower_limit)/2;

                if( oil_temp >= stream[i].upper_limit )
                    oil_temp = middle;
                if( oil_temp <= stream[i].lower_limit )
                    oil_temp = middle ;
            } else if ( stream[i].pid_uuid == MODE1_MANIFOLD_ABS_PRESS_UUID )
            {
                stream[i].timestamp++;
                pid_map += 0.65;
                stream[i].pid_value = pid_map;
                if( pid_map >= 253 )
                    pid_map = 0;
            } else if ( stream[i].pid_uuid == MODE1_BAROMETRIC_PRESSURE_UUID )
            {
                stream[i].timestamp++;
                stream[i].pid_value = baro;
            }
        }
    }
    #endif

    if( digitaldash_delay > 0 )
        digitaldash_delay--;

	#ifndef SPOOF_DATA
    if( digitaldash_app_wtchdg > 0 )
        digitaldash_app_wtchdg--;
	#endif

    if( digitaldash_bklt_wtchdg > 0 )
        digitaldash_bklt_wtchdg--;

	#ifndef SPOOF_DATA
    if( digitaldash_shutdown > 0 )
        digitaldash_shutdown--;
	#endif

	#if USE_LIB_OBDII
    if( tester_present > 0 ) {
        tester_present--;
    }
    /* Check if a tester was previously present */
    else if( digitaldash_get_flag( DD_TESTER_PRESENT ) == TESTER_PRESENT )
    {
        /* The timer expired, therefore it is assumed no tester is present */
        update_app_flag( DD_TESTER_PRESENT, NO_TESTER_PRESENT );

        /* Allow OBDII communication now that it is the only device present */
        OBDII_Continue( &obdii );
    }
	#endif

	#if USE_KE_PROTOCOL
    KE_tick();
	#endif

	#if USE_LIB_OBDII
    OBDII_tick();
	#endif

	#if USE_LIB_CAN_BUS_SNIFFER
    CAN_Sniffer_tick();
	#endif

	#if USE_LIB_VEHICLE_DATA
    Vehicle_tick();
	#endif

	#if DIGITALDASH_GRAPHICS
    ui_tick();
	#endif
}
