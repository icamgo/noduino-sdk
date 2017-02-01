/*
 *  Copyright (c) 2017 - 2025 MaiKe Labs
 *
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
#ifndef __USR_CONFIG_H__
#define __USR_CONFIG_H__

// Output normal info
#define	INFO			1

// Output debug message
//#define	DEBUG			1

#ifdef INFO
#define UPNP_INFO( format, ... ) os_printf( "UPNP: "format, ## __VA_ARGS__ )
#else
#define UPNP_INFO( format, ... )
#endif

#ifdef DEBUG
#define UPNP_DEBUG( format, ... ) os_printf( "UPNP: "format, ## __VA_ARGS__ )
#else
#define UPNP_DEBUG( format, ... )
#endif

#endif
