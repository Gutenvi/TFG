
#include "tmr.h"

int *main_candump();
void *exploraentradas();
void *imprimeentradas();
int main_cansend();
char* gettimeprog();
char* getactualtime();
#define DEBOUNCE_TIME 20;
double debounceTime;
void d_isr();
void a_isr();
tmr_t* a_tmr;
tmr_t* d_tmr;
void parseCANdata();

	FILE *log110;
	FILE *log120;
	FILE *log130;
	FILE *log140;