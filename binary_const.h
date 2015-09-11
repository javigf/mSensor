/* 
Binary constant generator macro
By Tom Torfs - donated to the public domain

From http://bytes.com/topic/c/answers/216333-binary-constant-macros
*/

/* All macro's evaluate to compile-time constants */

/* *** helper macros *** /

/* turn a numeric literal into a hex constant
(avoids problems with leading zeroes)
8-bit constants max value 0x11111111, always fits in unsigned long
*/
#define HEX__(n) 0x##n##LU

/* 8-bit conversion function */
#define B8__(x) ((x&0x0000000FLU)?1:0) \
+((x&0x000000F0LU)?2:0) \
+((x&0x00000F00LU)?4:0) \
+((x&0x0000F000LU)?8:0) \
+((x&0x000F0000LU)?16:0) \
+((x&0x00F00000LU)?32:0) \
+((x&0x0F000000LU)?64:0) \
+((x&0xF0000000LU)?128:0)

/* *** user macros *** /

/* for upto 8-bit binary constants */
#define B8(d) ((unsigned char)B8__(HEX__(d)))

/* for upto 16-bit binary constants, MSB first */
#define B16(dmsb,dlsb) (((unsigned short)B8(dmsb)<<8) \
+ B8(dlsb))

/* for upto 32-bit binary constants, MSB first */
#define B32(dmsb,db2,db3,dlsb) (((unsigned long)B8(dmsb)<<24) \
+ ((unsigned long)B8(db2)<<16) \
+ ((unsigned long)B8(db3)<<8) \
+ B8(dlsb))

/* Sample usage:
B8(01010101) = 85
B16(10101010,01010101) = 43605
B32(10000000,11111111,10101010,01010101) = 2164238933
*/


// Register map: see ADXL345 datasheet page 14
const int R_DEVID = 0;
const int R_THRESH_TAP = 29;
const int R_OFSX = 30;
const int R_OFSY = 31;
const int R_OFSZ = 32;
const int R_DUR = 33;
const int R_LATENT = 34;
const int R_WINDOW = 35;
const int R_THRESH_ACT = 36;
const int R_THRESH_INACT = 37;
const int R_TIME_INACT = 38;
const int R_ACT_INACT_CTL = 39;
const int R_THRESH_FF = 40;
const int R_TIME_FF = 41;
const int R_TAP_AXES = 42;
const int R_ACT_TAP_STATUS = 43;
const int R_BW_RATE = 44;
const int R_POWER_CTL = 45;
const int R_INT_ENABLE = 46;
const int R_INT_MAP = 47;
const int R_INT_SOURCE = 48;
const int R_DATA_FORMAT = 49;
const int R_DATAX0 = 50;
const int R_DATAX1 = 51;
const int R_DATAY0 = 52;
const int R_DATAY1 = 53;
const int R_DATAZ0 = 54;
const int R_DATAZ1 = 55;
const int R_FIFO_CTL = 56;
const int R_FIFO_STATUS = 57;

