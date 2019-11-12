/**    IAP : internal Flash memory access library
 *
 *        The internal Flash memory access is described in the LPC1768 and LPC11U24 usermanual.
 *            http://www.nxp.com/documents/user_manual/UM10360.pdf
 *            http://www.nxp.com/documents/user_manual/UM10462.pdf
 *
 *               LPC1768 --
 *                    Chapter  2: "LPC17xx Memory map"
 *                    Chapter 32: "LPC17xx Flash memory interface and programming"
 *                    refering Rev. 01 - 4 January 2010
 *
 *               LPC11U24 --
 *                    Chapter  2: "LPC11Uxx Memory mapping"
 *                    Chapter 20: "LPC11Uxx Flash programming firmware"
 *                    refering Rev. 03 - 16 July 2012
 *
 *        Released under the MIT License: http://mbed.org/license/mit
 *
 *        revision 1.0  09-Mar-2010   1st release
 *        revision 1.1  12-Mar-2010   chaged: to make possible to reserve flash area for user
 *                                            it can be set by USER_FLASH_AREA_START and USER_FLASH_AREA_SIZE in IAP.h
 *        revision 2.0  26-Nov-2012   LPC11U24 code added
 *        revision 2.1  26-Nov-2012   EEPROM access code imported from Suga koubou san's (http://mbed.org/users/okini3939/) library
 *                                            http://mbed.org/users/okini3939/code/M0_EEPROM_test/
 *        revision 3.0  09-Jan-2015   LPC812 and LPC824 support added
 *        revision 3.1  13-Jan-2015   LPC1114 support added
 *        revision 3.1.1 16-Jan-2015  Target MCU name changed for better compatibility across the platforms
 */


#ifndef        MBED_IAP
#define        MBED_IAP


#define 	USER_FLASH_AREA_SIZE 	4096
#define		USER_FLASH_AREA_START 	0xF000
#define     USER_FLASH_AREA_START_STR( x )      STR( x )
#define     STR( x ) #x

#define TARGET_SECTOR	15	//second last sector of the FLASH

//#include    "mbed.h"

/**    Error code by IAP routine
 *
 *        Table 588 "ISP Return Codes Summary", Section 7.15 "ISP Return Codes", usermanual
 */

enum error_code {
    CMD_SUCCESS,
    INVALID_COMMAND,
    SRC_ADDR_ERROR,
    DST_ADDR_ERROR,
    SRC_ADDR_NOT_MAPPED,
    DST_ADDR_NOT_MAPPED,
    COUNT_ERROR,
    INVALID_SECTOR,
    SECTOR_NOT_BLANK,
    SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION,
    COMPARE_ERROR,
    BUSY,
    PARAM_ERROR,
    ADDR_ERROR,
    ADDR_NOT_MAPPED,
    CMD_LOCKED,
    INVALID_CODE,
    INVALID_BAUD_RATE,
    INVALID_STOP_BIT,
    CODE_READ_PROTECTION_ENABLED
};



/*
 *  IAP routine entry
 *
 *        "IAP commands"
 */

/**    IAP class
 *
 *        Interface for internal flash memory access
 */

    /**    Constructor for IAP
     *
     */
   // int iap_entry( reinterpret_cast<IAP_call>(IAP_LOCATION) ), cclk_kHz( SystemCoreClock / 1000 ) {}

    /** Reinvoke ISP
     *
     *  @return    error code
     */
    int reinvoke_isp( void );

    /** Read part identification number
     *
     *  @return    device ID
     *  @see       read_serial()
     */
    int read_ID( void );

    /** Read device serial number
     *
     *  @return    device serial number
     *  @see       read_ID()
     */
    int *read_serial( void );

    /** Blank check sector(s)
     *
     *  @param    start    a Start Sector Number
     *  @param    end      an End Sector Number (should be greater than or equal to start sector number).
     *  @return error code: CMD_SUCCESS | BUSY | SECTOR_NOT_BLANK | INVALID_SECTOR
     */
    int blank_check( int start, int end );

    /** Erase Sector(s)
     *
     *  @param    start    a Start Sector Number
     *  @param    end      an End Sector Number (should be greater than or equal to start sector number).
     *  @return   error code: CMD_SUCCESS | BUSY | SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION | INVALID_SECTOR
     */
    int erase( int start, int end );

    /** Prepare sector(s) for write operation
     *
     *  @param    start    a Start Sector Number
     *  @param    end      an End Sector Number (should be greater than or equal to start sector number).
     *  @return   error code: CMD_SUCCESS | BUSY | INVALID_SECTOR
     */
    int prepare( int start, int end );

    /** Copy RAM to Flash
     *
     *  @param    source_addr    Source RAM address from which data bytes are to be read. This address should be a word boundary.
     *  @param    target_addr    Destination flash address where data bytes are to be written. This address should be a 256 byte boundary.
     *  @param    size           Number of bytes to be written. Should be 256 | 512 | 1024 | 4096.
     *  @return   error code: CMD_SUCCESS | SRC_ADDR_ERROR (Address not a word boundary) | DST_ADDR_ERROR (Address not on correct boundary) | SRC_ADDR_NOT_MAPPED | DST_ADDR_NOT_MAPPED | COUNT_ERROR (Byte count is not 256 | 512 | 1024 | 4096) | SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION | BUSY
     */
    int write( char *source_addr, char *target_addr, int size );

    /** Compare <address1> <address2> <no of bytes>
     *
     *  @param    source_addr Starting flash or RAM address of data bytes to be compared. This address should be a word boundary.
     *  @param    target_addr Starting flash or RAM address of data bytes to be compared. This address should be a word boundary.
     *  @param    size         Number of bytes to be compared; should be a multiple of 4.
     *  @return   error code: CMD_SUCCESS | COMPARE_ERROR | COUNT_ERROR (Byte count is not a multiple of 4) | ADDR_ERROR | ADDR_NOT_MAPPED
     */
    int compare( char *source_addr, char *target_addr, int size );

    /** Read Boot code version number
     *
     *  @return   2 bytes of boot code version number
     */
    int read_BootVer( void );

    /** Get user reserved flash start address
     *
     *  @return    start address of user reserved flash memory
     *  @see       reserved_flash_area_size()
     */

    char *reserved_flash_area_start( void );

    /** Get user reserved flash size
     *
     *  @return    size of user reserved flash memory
     *  @see       reserved_flash_area_start()
     */
    int   reserved_flash_area_size( void );

	#define     IAP_LOCATION    0x1fff1ff1
    typedef     void (*IAP_call)(unsigned int [], unsigned int []);

    unsigned int    IAP_command[ 5 ];
    unsigned int    IAP_result[ 5 ];
    int             cclk_kHz;


   // typedef void iap_call_type(int *args, int *results);
   // static const
   // void (*iap_call)(int *a, int *r) = (iap_call_type *) 0x7FFFFFF1;



#endif    //  #ifndef  MBED_IAP
