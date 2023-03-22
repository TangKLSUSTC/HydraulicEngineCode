/*

WARNING:		This file only uses in rt-thread nano version or no-os system.

*/
/*	Common defines of device drivers */


/*
	Bits macros
*/
#define READ_BITS(Instance, Number)			(Instance>>Number)&0b1

#define WRITE_BITS(Instance, Number, x)	\
				do{															\
				if (x==0)												\
				{																\
					Instance&=~(1<<Number);				\
				}																\
				else if (x==1)									\
				{																\
					Instance|=(1<<Number);				\
				}																\
				}while(0)
				

				
				
				
/*								End of File										*/