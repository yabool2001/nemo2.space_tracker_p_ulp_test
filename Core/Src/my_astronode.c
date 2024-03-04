/*
 * my_astronode.c
 *
 *  Created on: Oct 23, 2023
 *      Author: mzeml
 */
#include "my_astronode.h"

bool my_astro_handle_evt ( void )
{
	send_debug_logs ( "my_astronode.c,my_astro_handle_evt,evt pin is high." ) ;
	astronode_send_evt_rr () ;
	if (is_sak_available () )
	{
	  astronode_send_sak_rr () ;
	  astronode_send_sak_cr () ;
	  send_debug_logs ( "my_astronode.c,my_astro_handle_evt,message has been acknowledged." ) ;
	  //astronode_send_per_rr () ;
	}
	if ( is_astronode_reset () )
	{
	  send_debug_logs ( "my_astronode.c,my_astro_handle_evt,terminal has been reset." ) ;
	  astronode_send_res_cr () ;
	}
	if ( is_command_available () )
	{
	  send_debug_logs ( "my_astronode.c,my_astro_handle_evt,unicast command is available" ) ;
	  astronode_send_cmd_cr () ;
	}
	return true ;
}
bool my_astro_log ( void )
{
	astronode_send_rtc_rr ();
	astronode_send_nco_rr () ;
	//astronode_send_lcd_rr () ;
	//astronode_send_end_rr () ;
	//astronode_send_per_rr () ;
	return true ;
}
