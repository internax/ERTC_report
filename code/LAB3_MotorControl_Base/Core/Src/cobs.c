/*
 * Riccardo Antonello (riccardo.antonello@unipd.it)
 * 
 * December 15, 2017
 *
 * Dept. of Information Engineering, University of Padova 
 *
 */

#include <stdint.h>

/*	COBS encoding routine  */
#define FinishBlock(X) (*code_ptr = (X), code_ptr = dst++, code = 0x01)

void cobsEncode(const uint8_t *ptr, int length, uint8_t *dst)
{
    const uint8_t *end = ptr + length;
    uint8_t *code_ptr = dst++;
    uint8_t code = 0x01;
    
    while (ptr < end)
    {
        if (*ptr == 0)
            FinishBlock(code);
        else
        {
            *dst++ = *ptr;
            if (++code == 0xFF)
                FinishBlock(code);
        }
        ptr++;
    }
    
    FinishBlock(code);
}


/*	COBS decoding routine */
void cobsDecode(const uint8_t *ptr, int length, uint8_t *dst)
{
  const uint8_t *end = ptr + length;
  int i, code;

  while (ptr < end)
  {
    code = *ptr++;

    for (i = 1; i < code; i++)
      *dst++ = *ptr++;

    if (code < 0xFF)
      *dst++ = 0;
  }
}
