#include <hal.h>

void ILI9341_init(SPIDriver *spip);
void clearDisplay(SPIDriver *spip, uint16_t color);
void ILI9341_identification(SPIDriver *spip);
