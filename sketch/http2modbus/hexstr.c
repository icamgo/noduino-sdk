/*
 *  Copyright (c) 2015 - 2025 MaiKe Labs
 *
 *	This program is free software: you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation, either version 3 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
*/
#include "user_config.h"
#include <string.h>

/*
 * convert hexstring to len bytes of data
 * returns 0 on success, -1 on error
 * data is a buffer of at least len bytes
 * hexstring is upper or lower case hexadecimal, NOT prepended with "0x"
 *
 */
int str2hex(uint8_t *data, const char *hexstr, uint32_t len)
{
    unsigned const char *pos = hexstr;
    char *endptr;
    uint32_t count = 0;

    if ((hexstr[0] == '\0') || (strlen(hexstr) % 2)) {
        //hexstr contains no data
        //or hexstr has an odd length
        return -1;
    }

    for(count = 0; count < len; count++) {
        char buf[5] = {'0', 'x', pos[0], pos[1], 0};
        data[count] = strtol(buf, &endptr, 0);
        pos += 2 * sizeof(char);

        if (endptr[0] != '\0') {
            //non-hexadecimal character encountered
            return -1;
        }
    }

    return 0;
}
