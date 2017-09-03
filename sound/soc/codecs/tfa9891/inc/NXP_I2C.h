/*
 *Copyright 2015 NXP Semiconductors
 *
 *Licensed under the Apache License, Version 2.0 (the "License");
 *you may not use this file except in compliance with the License.
 *You may obtain a copy of the License at
 *
 *http://www.apache.org/licenses/LICENSE-2.0
 *
 *Unless required by applicable law or agreed to in writing, software
 *distributed under the License is distributed on an "AS IS" BASIS,
 *WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *See the License for the specific language governing permissions and
 *limitations under the License.
 */

#ifndef NXP_I2C_H
#define NXP_I2C_H

//Version
#define TFA98XX_HAL_REV_MAJOR           (2)
#define TFA98XX_HAL_REV_MINOR           (10)
#define TFA98XX_HAL_REV_REVISION        (1)
#define TFA98XX_HAL_REV_STR		"2.10.1"

/* The maximum I2C message size allowed for read and write buffers, incl the slave address */
#define NXP_I2C_MAX_SIZE 254 // TODO remove this'NXP_I2C_MAX_SIZE, its platform value

#ifdef __cplusplus
extern "C" {
#endif

/* A glue layer.
 * The NXP components will use the functions defined in this API to do the actual calls to I2C
 * Make sure you implement this to use your I2C access functions (which are SoC and OS dependent)
 */

/**
 * A list of supported I2C errors that can be returned.
 */
enum NXP_I2C_Error {
	NXP_I2C_UnassignedErrorCode,		/**< UnassignedErrorCode*/
	NXP_I2C_Ok,				/**< no error */
	NXP_I2C_NoAck, 		/**< no I2C Ack */
	NXP_I2C_TimeOut,		/**< bus timeout */
	NXP_I2C_ArbLost,		/**< bus arbritration lost*/
	NXP_I2C_NoInit,		/**< init was not done */
	NXP_I2C_UnsupportedValue,		/**< UnsupportedValu*/
	NXP_I2C_UnsupportedType,		/**< UnsupportedType*/
	NXP_I2C_NoInterfaceFound,		/**< NoInterfaceFound*/
	NXP_I2C_NoPortnumber,			/**< NoPortnumber*/
	NXP_I2C_BufferOverRun,			/**< BufferOverRun*/
	NXP_I2C_ErrorMaxValue				/**<impossible value, max enum */
};

typedef enum NXP_I2C_Error NXP_I2C_Error_t;

/**
 * Open and register a target interface.
 *
 *  @param target interfacename
 *  @return filedescripter (if relevant)
 */
int NXP_I2C_Open(char *targetname);

/**
 * Close and un-register the target interface.
 *
 *  Note that the target may block the caller when it has not been properly
 *  shutdown.
 */
int NXP_I2C_Close(void);

/**
 *  Returns the maximum number of bytes that can be transfered in one burst transaction.
 *
 *  @param None
 *  @return max burst size in bytes
 */
int NXP_I2C_BufferSize(void);

/**
 * Execute a write, followed by I2C restart and a read of num_read_bytes bytes.
   The read_buffer must be big enough to contain num_read_bytes.
 *
 *  @param sla = slave address
 *  @param num_write_bytes = size of data[]
 *  @param write_data[] = byte array of data to write
 *  @param num_read_bytes = size of read_buffer[] and number of bytes to read
 *  @param read_buffer[] = byte array to receive the read data
 *  @return NXP_I2C_Error_t enum
 */
NXP_I2C_Error_t NXP_I2C_WriteRead(  unsigned char sla,
				int num_write_bytes,
				const unsigned char write_data[],
				int num_read_bytes,
				unsigned char read_buffer[] );

/**
 * Enable/disable I2C transaction trace.
 *
 *  @param on = 1, off = 0
 *
 */
void NXP_I2C_Trace(int on);

/**
 * Use tracefile to write trace output.
 *
 *  @param filename: 0=stdout,  "name"=filename, -1=close file
 *  @return filedescripter or -1 on error
 */
void NXP_I2C_Trace_file(char *filename);

/**
 * Read back the pin state.
 *
 *  @param pin number
 *  @return pin state
 */
int NXP_I2C_GetPin(int pin);

/**
 * Set the pin state.
 *
 *  @param pin number
 *  @param pin value to set
 *  @return NXP_I2C_Error_t enum
 */
NXP_I2C_Error_t NXP_I2C_SetPin(int pin, int value);

/**
 * return HAL revision as 3 integers (by reference)
 *
 * @param pointer to major
 * @param pointer to minor
 * @param pointer to revision
 *
 */
void NXP_I2C_rev(int *major, int *minor, int *revision);

/**
 * Read back version info as a string.
 *
 *  @param string buffer to hold the string (will not exceed 1024 bytes)
 *  @return NXP_I2C_Error_t enum
 */
NXP_I2C_Error_t NXP_I2C_Version(char *data);

/**
 * Return the string for the error.
 *
 *  @param  error code
 *  @return string describing NXP_I2C_Error_t enum
 */
const char *NXP_I2C_Error_string(NXP_I2C_Error_t error);

#ifdef __cplusplus
}
#endif

#endif // NXP_I2C_H
