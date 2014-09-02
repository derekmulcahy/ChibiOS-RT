*****************************************************************************
** ChibiOS/RT port for ARM-Cortex-M4 MCHCK K20 .                           **
*****************************************************************************

The demo runs on an MKHCK K20 board. It use the I2C bus to retrieve the
current temperature from a TMP101NA device once per second.
The TMP101NA is at I2C address 0x48. The result is sent to the UART at 38400
baud.

The pin connections are

  D7 is connected to TX
  D6 is connected to RX
  B0 is connected to SCL
  B1 is connected to SDA
