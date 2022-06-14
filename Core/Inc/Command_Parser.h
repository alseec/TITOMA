#ifndef __COMMAND_PARSER_INC_
#define __COMMAND_PARSER_INC_

#include <stdint.h>
#include <stdlib.h>

#define PROTOCOL_PREAMBLE '*'
#define PROTOCOL_POSAMBLE '#'

void parse_command(uint8_t *);


#endif //__COMMAND_PARSER_INC_//
