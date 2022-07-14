#ifndef __LOADER_H__
#define __LOADER_H__

#include <stdint.h>

typedef struct loader_s
{
	char *rom;
	char *base;
	char *sram;
	char *state;
	int ramloaded;
} loader_t;


extern loader_t loader;

void loader_init(uint8_t *s);
void loader_unload();
int rom_load();
int sram_load(FILE* f);
int sram_save(FILE* f);
void state_load(int n);
void state_save(int n);



#endif
