/***************************************************************************\
*                                                                           *
* 1. CPU                                                                    *
*                                                                           *
\***************************************************************************/
#define CPU                      atmega328p

#ifndef F_CPU
#define F_CPU                    16000000UL
#endif

#define MOTHERBOARD

/***************************************************************************\
*                                                                           *
* 2. PINOUTS                                                                *
*                                                                           *
\***************************************************************************/
#define X_STEP_PIN               DIO3
#define X_DIR_PIN                DIO2
//#define X_MIN_PIN                DIO9
//#define X_MAX_PIN                DIO21
//#define X_ENABLE_PIN             DIO8
//#define X_INVERT_DIR
//#define X_INVERT_MIN
//#define X_INVERT_MAX
//#define X_INVERT_ENABLE

#define Y_STEP_PIN               DIO5
#define Y_DIR_PIN                DIO4
//#define Y_MIN_PIN                DIO10
//#define Y_MAX_PIN                DIO26
//#define Y_ENABLE_PIN             DIO8
//#define Y_INVERT_DIR
//#define Y_INVERT_MIN
//#define Y_INVERT_MAX
//#define Y_INVERT_ENABLE

//#define Z_STEP_PIN               DIO4
//#define Z_DIR_PIN                DIO7
//#define Z_MIN_PIN                DIO11
//#define Z_MAX_PIN                DIO31
//#define Z_ENABLE_PIN             DIO8
//#define Z_INVERT_DIR
//#define Z_INVERT_MIN
//#define Z_INVERT_MAX
//#define Z_INVERT_ENABLE

//#define E_STEP_PIN               DIO12
//#define E_DIR_PIN                DIO13
//#define E_ENABLE_PIN             DIO8
//#define E_INVERT_DIR
//#define E_INVERT_ENABLE

//#define PS_ON_PIN                DIO14
//#define PS_INVERT_ON
//#define PS_MOSFET_PIN            xxxx
//#define STEPPER_ENABLE_PIN       DIO8
//#define STEPPER_INVERT_ENABLE


//HPC6602 Cartridge Pinouts
#define INK_PINA AIO0
#define INK_PINB AIO1
#define INK_PINC AIO2
#define INK_PIND AIO3

#define INK_PULSE DIO6
#define LED_DEBUG DIO7

/***************************************************************************\
*                                                                           *
* 3. COMMUNICATION OPTIONS                                                  *
*                                                                           *
\***************************************************************************/
#define BAUD                     115200
