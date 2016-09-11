List of components needed for this sketch:
 * Arduino Duemilanove At328
 * Easy driver - stepper motor driver and motor (sparkfun)
 * Sparkfun Logic level converter - bi-directional
 * MPU9250 breakout board (sparkfun)

Hookup instructions easy driver:
 * Supply 2A >5V power and ground to easy driver
 * Pull MS1/MS2 both low through pull-down resistors
 * Connect 2 pole/2 pair connections to driver (check resistance between pairs to find each coil pair
 * Connect arduino pin 2 to step pin
 * Connect arduino pin 3 to step dir pin
 * Connect arduino pin 4 to enable (LOW enables)

Hookup instructions for MPU9250:
 * NOTE: all connections must be made through the level converter or
   damage to the MPU chip will result. DO NOT expose to 5V!
 * Supply 3.3V power and ground to MPU
 * Connect Arduino pin 13 to SCL (clock)
 * Connect Arduino pin 12 to SDO/ADO
 * Connect Arduino pin 11 to SDI/SDA
 * Connect Arduino pin 10 to NCS/CS (chip select)