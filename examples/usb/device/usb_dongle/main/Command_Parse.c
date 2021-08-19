/**
 * @copyright Copyright 2021 Espressif Systems (Shanghai) Co. Ltd.
 *
 *      Licensed under the Apache License, Version 2.0 (the "License");
 *      you may not use this file except in compliance with the License.
 *      You may obtain a copy of the License at
 *
 *               http://www.apache.org/licenses/LICENSE-2.0
 * 
 *      Unless required by applicable law or agreed to in writing, software
 *      distributed under the License is distributed on an "AS IS" BASIS,
 *      WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *      See the License for the specific language governing permissions and
 *      limitations under the License.
 */

#include <stdint.h>
#include <string.h>
#include "esp_log.h"
#include "esp_err.h"

#include "FreeRTOS_CLI.h"

#include "data_back.h"

/* Dimensions the buffer into which input characters are placed. */
#define cmdMAX_INPUT_SIZE	80

/* Dimensions the buffer into which string outputs can be placed. */
#define cmdMAX_OUTPUT_SIZE	1024

/* Dimensions the buffer passed to the recvfrom() call. */
#define cmdSOCKET_INPUT_BUFFER_SIZE 80

static const char *TAG = "Command Parse";

void Command_Parse(char* Cmd)
{
size_t lBytes, lByte;
signed char cInChar, cInputIndex = 0;
static char cInputString[ cmdMAX_INPUT_SIZE ], cOutputString[ cmdMAX_OUTPUT_SIZE ], cLocalBuffer[ cmdSOCKET_INPUT_BUFFER_SIZE ];
BaseType_t xMoreDataToFollow;
uint8_t *data = (uint8_t *)heap_caps_malloc(256, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
char *format_buf = (char *)malloc(64);

    lBytes = strlen(Cmd);
    memcpy(cLocalBuffer, Cmd, lBytes);

    /* Process each received byte in turn. */
    lByte = 0;
    while( lByte < lBytes )
    {
        /* The next character in the input buffer. */
        cInChar = cLocalBuffer[ lByte ];
        lByte++;

        /* Newline characters are taken as the end of the command string. */
        if( cInChar == '\n' )
        {
            size_t lenth = sprintf(format_buf, "\r\n");
            esp_data_back(format_buf, lenth, ENABLE_FLUSH);
            /* Process the input string received prior to the newline. */
            do
            {
                memset(data, 0x00, 256);
                /* Pass the string to FreeRTOS+CLI. */
                xMoreDataToFollow = FreeRTOS_CLIProcessCommand( cInputString, cOutputString, cmdMAX_OUTPUT_SIZE );
                printf("%s%d\r\n", cOutputString, strlen(cOutputString));
                /* Send the output generated by the command's implementation. */
                memcpy(data, (uint8_t*)cOutputString, strlen(cOutputString));
                esp_data_back(data, strlen(cOutputString), DISABLE_FLUSH);
            } while( xMoreDataToFollow != pdFALSE ); /* Until the command does not generate any more output. */

            /* All the strings generated by the command processing have been sent.
            Clear the input string ready to receive the next command. */
            lenth = sprintf(format_buf, ">");
            esp_data_back(format_buf, lenth, ENABLE_FLUSH);
            cInputIndex = 0;
            memset( cInputString, 0x00, cmdMAX_INPUT_SIZE );
            free(data);
        }
        else
        {
            if( cInChar == '\r' )
            {
                /* Ignore the character.  Newlines are used to detect the end of the input string. */
            }
            else if( cInChar == '\b' )
            {
                /* Backspace was pressed.  Erase the last character in the string - if any. */
                if( cInputIndex > 0 )
                {
                    cInputIndex--;
                    cInputString[ cInputIndex ] = '\0';
                }
            }
            else
            {
                /* A character was entered.  Add it to the string entered so far.
                When a \n is entered the complete string will be passed to the command interpreter.*/
                if( cInputIndex < cmdMAX_INPUT_SIZE )
                {
                    cInputString[ cInputIndex ] = cInChar;
                    cInputIndex++;
                }
            }
        }
    }
}
