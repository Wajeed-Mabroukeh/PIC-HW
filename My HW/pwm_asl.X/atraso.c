#include <xc.h>
#include <stdio.h>
void atraso_ms1(unsigned int valor)
{
unsigned int  i;
unsigned char j;

 for (i =0; i< valor; i++)
 {
 
  for (j =0 ; j < 200; j++)
   {
      CLRWDT();
      CLRWDT();
      CLRWDT();
      CLRWDT();
      CLRWDT();
      

   }
 }
}
