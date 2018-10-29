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
#ifndef	__XKEY_H__
#define	__XKEY_H__

#define XKEY_NUM        1

#define XKEY_IO_MUX     PERIPHS_IO_MUX_MTDI_U
#define XKEY_IO_NUM     12
#define XKEY_IO_FUNC    FUNC_GPIO12

void xkey_init();

#endif
