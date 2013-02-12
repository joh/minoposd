/**
 ******************************************************************************
 *
 * @file       RSSI.h
 * @author     Joerg-D. Rothfuchs
 * @brief      Implements RSSI measurement
 * 	       on the Ardupilot Mega MinimOSD using built-in ADC reference.
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/> or write to the 
 * Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */


// !!! For using this, you have to solder a little bit on the MinimOSD, see the wiki !!!


#ifndef RSSI_H_
#define RSSI_H_


#define RSSI_PIN			2

#define RSSI_LOW			0			// initial value of rssi_low
#define RSSI_HIGH			1023			// initial value of rssi_high

#define CURRENT_RSSI(x)			((x) - RSSI_LOW) / ((RSSI_HIGH - RSSI_LOW) / 100.0)

void RSSI_init(void);
void RSSI_print(void);


#endif /* RSSI_H_ */
