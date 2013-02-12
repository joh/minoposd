/**
 ******************************************************************************
 *
 * @file       RSSI.ino
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


#include "RSSI.h"


void RSSI_init(void)
{
	analogReference(INTERNAL);			// INTERNAL: a built-in reference, equal to 1.1 volts on the ATmega168 or ATmega328
}


void RSSI_print(void)
{
    static int rssi = 0;
    
    if (RSSI_LOW == 0) {							// the uncalibrated raw RSSI
        rssi = analogRead(RSSI_PIN);						// reads raw RSSI
        osd.printf("%c%4i", 0xE1, rssi);
    } else {									// the calibrated RSSI in 0-100%
        rssi = (int) CURRENT_RSSI(analogRead(RSSI_PIN)) * .2 + rssi * .8;	// reads calibrated RSSI
	// TODO range clipping 0-150 einbauen
        osd.printf("%c%3i%c", 0xE1, rssi, 0x25);
    }
}
