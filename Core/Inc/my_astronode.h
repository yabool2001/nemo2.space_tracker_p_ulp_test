/*
 * my_astronode.h
 *
 *  Created on: Oct 23, 2023
 *      Author: mzeml
 */

#ifndef ASTROCAST_INC_MY_ASTRONODE_H_
#define ASTROCAST_INC_MY_ASTRONODE_H_

#include <string.h>
#include "astronode_application.h"
#include "astronode_definitions.h"
#include "astronode_transport.h"


bool my_astro_handle_evt ( void ) ;
bool my_astro_log ( void ) ;
void send_debug_logs ( char* ) ;

#endif /* ASTROCAST_INC_MY_ASTRONODE_H_ */
