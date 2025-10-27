#ifndef LIB_DIGITAL_DASH_H
#define LIB_DIGITAL_DASH_H

#define DIGITALDASH_GRAPHICS_ONLY (DIGITALDASH_GRAPHICS & !DIGITALDASH_DATA_ACQ)
#define DIGITALDASH_DATA_ACQ_ONLY (DIGITALDASH_DATA_ACQ & !DIGITALDASH_GRAPHICS)

#include "../ke_conf.h"
#include "stddef.h"
#if DIGITALDASH_GRAPHICS
#include "lvgl.h"
#include "ui.h"
#include "ke_config.h"
#endif

#if USE_KE_PROTOCOL
#include "lib_ke_protocol.h"
#endif

#if USE_UNIT_CONVERSION
#include "lib_unit_conversion.h"
#endif

#if USE_LIB_OBDII
#include "lib_obdii.h"
#endif

#if USE_LIB_CAN_BUS_SNIFFER
#include "lib_CAN_bus_sniffer.h"
#endif

#if USE_LIB_VEHICLE_DATA
#include "lib_vehicle_data.h"
#endif

#if USE_LIB_PID
#include "lib_pid.h"
#endif

#if SD_CARD_ACTIVE
/********************************************************************
* SD Card detection variables                                       *
********************************************************************/
typedef enum _sd_card_state {
    SD_NOT_PRESENT,
    SD_PRESENT
} SD_CARD_STATE, *PSD_CARD_STATE;
#endif

#if HOST_CTRL_ACTIVE
/********************************************************************
* Host power enable                                                 *
********************************************************************/
typedef enum _host_pwr_state {
    HOST_PWR_DISABLED,
    HOST_PWR_SLEEP,
    HOST_PWR_ENABLED
} HOST_PWR_STATE, *PHOST_PWR_STATE;
#endif

#if USB_PWR_CTRL
/********************************************************************
* USB power enable                                                 *
********************************************************************/
typedef enum _usb_pwr_state {
    USB_PWR_DISABLED,
    USB_PWR_ENABLED
} USB_PWR_STATE, *PUSB_PWR_STATE;
#endif

#if FAN_CTRL_ACTIVE
/********************************************************************
* Fan power enable                                                 *
********************************************************************/
typedef enum _fan_pwr_state {
    FAN_OFF,
    FAN_MIN,
    FAN_MED,
    FAN_MAX
} FAN_PWR_STATE, *PFAN_PWR_STATE;
#endif

/********************************************************************
* Digital Dash Initialization status                                 *
********************************************************************/
typedef enum _dd_initialized_state {
    DD_NOT_INITIALIZED,
    DD_INITIALIZED
} DD_INITIALIZED_STATE, *PDD_INITIALIZED_STATE;

/********************************************************************
* Core application flag definitions                                 *
********************************************************************/

typedef enum _digitaldash_operating_state {
    DD_OP_OFF,
	DD_OP_SHUTTING_DOWN,
    DD_OP_BOOTING,
    DD_OP_IDLE,
    DD_OP_STREAM,
    DD_OP_ERROR,
} DIGITALDASH_OPERATING_STATE, DIGITALDASH_OPERATING_STATE;

typedef enum _digitaldash_flags{

    DD_FLG_SD_CARD,
    DD_FLG_HOST_PWR,
    DD_FLG_INIT,
    DD_GUI_ACTIVE,
        #define GUI_IS_INACTIVE           0
        #define GUI_IS_ACTIVE             1
    DD_KE_ACTIVITY,
        #define KE_FLAG_NO_CHANGE         0
        #define KE_FLAG_UPDATE            1
    DD_OBDII_ACTIVITY,
        #define OBDII_FLAG_NO_CHANGE      0
        #define OBDII_FLAG_UPDATE         1
    DD_CAN_DECODE_ACTIVITY,
        #define CAN_DECODE_FLAG_NO_CHANGE 0
        #define CAN_DECODE_FLAG_UPDATE    1
    DD_LCD_STATE,
        #define LCD_DISABLED              0
        #define LCD_ENABLED               1
    DD_USB_PWR,
    DD_TESTER_PRESENT,
        #define TESTER_PRESENT            0
        #define NO_TESTER_PRESENT         1
	DD_SETTINGS_LOADED
		#define SETTINGS_NOT_LOADED       0
		#define SETTINGS_LOADED           1

} DIGITALDASH_FLAG, *PDIGITALDASH_FLAG;

/********************************************************************
* SD Card detection functions                                       *
********************************************************************/
#if SD_CARD_ACTIVE
/* The main application shall call this function to indicate *
 * when the SD card state changes                           */
void dd_update_sd_card_state( SD_CARD_STATE state );

/* The digital dash will call this function to get the       *
 * current SD card state. The main application shall call    *
 * "dd_update_sd_card_state" when this callback occurs.     */
typedef void (*DD_GET_SD_CARD_STATE)( void );
#endif

#if KE_ACTIVE
/* The digital dash will call this function to send a send  *
* a transmission to the KE host device                     */
typedef int (*DD_KE_TX)( uint8_t tx[], uint32_t len );
#endif

#if ECU_ACTIVE
/* The digital dash will call this function to send a send  *
 * a transmission to the connected ECU                      */
typedef uint8_t (*DD_ECU_TX)( uint8_t tx[], uint8_t len );
#endif

#if BKLT_CTRL_ACTIVE
/* The digital dash will call this function to set the      *
 * backlight brightness                                     */
typedef void (*DD_SET_BACKLIGHT)( uint8_t brightness );
#endif

#if FAN_CTRL_ACTIVE
/* The digital dash will call this function to enable the   *
 * fan                                                      */
typedef void (*DD_FAN_CTRL)( FAN_PWR_STATE state );
#endif

#if HOST_CTRL_ACTIVE
/* The digital dash will call this function when the host is *
 * ready to be powered on                                    */
typedef void (*DD_HOST_CTRL)( HOST_PWR_STATE state );
#endif

#if USB_PWR_CTRL
/* The digital dash will call this function when USB power   *
 * should be powered on                                      */
typedef void (*DD_USB_CTRL)( USB_PWR_STATE state );
#endif

#if HW_CAN_FILTERS
typedef void (*DD_CAN_FILTER)( uint16_t id, uint8_t enable );
#endif

#if BACKGROUND_IMG_SAVE
typedef bool (*DD_BACKGROUND_IMG_SAVE)( char *image_buffer, uint32_t image_size, uint8_t idx );
#endif

#if SPLASH_OVERRIDE
typedef bool (*DD_SPLASH_OVERRIDE)( void );
#endif

#if BOOTLOADER_ACTIVATION
typedef void (*DD_BOOTLOADER_ACTIVATION)( void );
#endif

typedef enum _digitaldash_init_status {
    DIGITALDASH_INIT_ERROR,
    DIGITALDASH_INIT_SD_PTR_ERROR,
    DIGITALDASH_INIT_KE_PTR_ERROR,
    DIGITALDASH_INIT_ECU_PTR_ERROR,
    DIGITALDASH_INIT_BKLT_PTR_ERROR,
    DIGITALDASH_INIT_FAN_PTR_ERROR,
    DIGITALDASH_INIT_HOST_PTR_ERROR,
    DIGITALDASH_INIT_USB_PTR_ERROR,
    DIGITALDASH_INIT_CAN_FILT_PTR_ERROR,
    DIGITALDASH_INIT_KE_INIT_ERROR,
	DIGITALDASH_INIT_BACKGROUND_SAVE_PTR_ERROR,
	DIGITALDASH_INIT_SPLASH_OVERRIDE_PTR_ERROR,
	DIGITALDASH_INIT_BOOTLOADER_ACTIVATION_PTR_ERROR,
    DIGITALDASH_INIT_OK
} DIGITALDASH_INIT_STATUS, *PDIGITALDASH_INIT_STATUS;

typedef enum _digitaldash_start_status {
    DIGITALDASH_START_ERROR,
    DIGITALDASH_START_NOT_INIT,
    DIGITALDASH_START_WAITING,
    DIGITALDASH_START_OK
} DIGITALDASH_START_STATUS, *PDIGITALDASH_START_STATUS;

typedef enum _digitaldash_poll_status {
    DIGITALDASH_POLL_ERROR,
    DIGITALDASH_POLL_OK
} DIGITALDASH_POLL_STATUS, *PDIGITALDASH_POLL_STATUS;

typedef enum _digitaldash_idle_status {
    DIGITALDASH_IDLE_ERROR,
    DIGITALDASH_IDLE_OK
} DIGITALDASH_IDLE_STATUS, *PDIGITALDASH_IDLE_STATUS;

typedef enum _digitaldash_status {
    DIGITALDASH_ERROR,
    DIGITALDASH_NOT_INIT,
    DIGITALDASH_OK
} DIGITALDASH_STATUS, *PDIGITALDASH_STATUS;

typedef struct _digitaldash_config {

#if SD_CARD_ACTIVE
    /* The digital dash will call this function to get the      *
    * current SD card state. The main application shall call    *
    * "dd_update_sd_card_state" when this callback occurs.      */
    DD_GET_SD_CARD_STATE dd_get_sd_card_state;
#endif

#if KE_ACTIVE
    /* The digital dash will call this function to send a send  *
    * a transmission to the KE host device                      */
    DD_KE_TX dd_ke_tx;
#endif

#if ECU_ACTIVE
    /* The digital dash will call this function to send a send  *
    * a transmission to the connected ECU                       */
    DD_ECU_TX dd_ecu_tx;
#endif

#if BKLT_CTRL_ACTIVE
    /* The digital dash will call this function to set the      *
    * backlight brightness                                      */
    DD_SET_BACKLIGHT dd_set_backlight;
#endif

#if FAN_CTRL_ACTIVE
    /* The digital dash will call this function to enable the   *
    * fan                                                       */
    DD_FAN_CTRL dd_fan_ctrl;
#endif

#if HOST_CTRL_ACTIVE
    /* The digital dash will call this function when the host is *
    * ready to be powered on                                     */
    DD_HOST_CTRL dd_host_ctrl;
#endif

#if USB_PWR_CTRL
    /* The digital dash will call this function when USB power   *
     * should be powered on                                      */
    DD_USB_CTRL dd_usb;
#endif

#if HW_CAN_FILTERS
    /* The digital dash will call this function when a new CAN   *
     * bus filter is needed                                      */
    DD_CAN_FILTER dd_filter;
#endif

#if BACKGROUND_IMG_SAVE
    /* The digital dash will call this function when a           *
     * background image has been transfered as RGBA data and     *
     * needs to be saved                                         */
    DD_BACKGROUND_IMG_SAVE dd_background_save;
#endif

#if SPLASH_OVERRIDE
	DD_SPLASH_OVERRIDE dd_splash_override;
#endif

#if BOOTLOADER_ACTIVATION
	DD_BOOTLOADER_ACTIVATION dd_bootloader_activate;
#endif

} DIGITALDASH_CONFIG, *PDIGITALDASH_CONFIG;

/* Clear all variables except the function callbacks */
void DigitalDash_Reset_App( void );

/* The Digital Dash must be configured be usage, refer to       *
 * @DIGITALDASH_CONFIG for needed callbacks                    */
DIGITALDASH_INIT_STATUS digitaldash_init( PDIGITALDASH_CONFIG config );

DIGITALDASH_STATUS digitaldash_service( void );

/* Return the Digital Dash application flags                   */
uint32_t digitaldash_get_app_flags( void );

uint8_t digitaldash_get_flag( DIGITALDASH_FLAG flag );

void digitaldash_tick( void );

void DigitalDash_Add_CAN_Packet( uint16_t id, uint8_t* data );

void DigitalDash_Add_UART_byte( uint8_t byte );

int DigitalDash_Remove_PID_From_Stream( PTR_PID_DATA pid, uint8_t device );
PTR_PID_DATA DigitalDash_Add_PID_To_Stream( PTR_PID_DATA pid, uint8_t device );
uint8_t DigitalDash_Pause_PID_In_Stream( PTR_PID_DATA pid, uint8_t device );
uint8_t DigitalDash_Resume_PID_In_Stream( PTR_PID_DATA pid, uint8_t device );

#ifdef TEST
void KE_Flag_Callback( uint16_t flag, uint8_t bit );
#endif


#endif // LIB_DIGITAL_DASH_H
