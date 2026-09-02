#include "soem/soem.h"

int ecx_readIDNmap(ecx_contextt *context, uint16 slave, uint32 *Osize, uint32 *Isize)
{
   (void)context;
   (void)slave;

   if (Osize != 0)
   {
      *Osize = 0U;
   }

   if (Isize != 0)
   {
      *Isize = 0U;
   }

   return 0;
}
