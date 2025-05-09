/*****************************************************************************/
// RC Values
// - These are defined by the limits of your transmitter
/*****************************************************************************/
#define RC_MIN 172
#define RC_MID 992
#define RC_MAX 1811

/*****************************************************************************/
// RC Deadband
// - Set a range of values to be considered "off"
// - Anything in between these values is "off"
/*****************************************************************************/
#define RC_DEADBAND_LOW 962
#define RC_DEADBAND_HIGH 978

/*****************************************************************************/
// RC Channel Mapping
// - The channels are N - 1 where N is the defined channel number on the transmitter
// - Ex: CH1 on Tx is 0 here, CH2 is 1
/*****************************************************************************/
#define CH_DRIVE_MAIN 0
// 1: Channel 2 is directly connected to dome servo
// 2: Channel 3 is directly connected to dome servo 
#define CH_DRIVE_S2S 3
#define CH_FLYWHEEL 4
#define CH_DOME_SPIN 5
// 6: Channel 7 is the RC relay
#define CH_SOUND_TRIGGER 7
#define CH_SOUND_BANK 8
#define CH_DRIVE_EN 9
#define CH_DIRECTION 10
#define CH_ROLL_OFFSET 11
#define CH_FLYWHEEL_EN 12
#define CH_S2S_OFFSET 13
#define CH_RANDOM_SOUND 14

/*****************************************************************************/
// Main drive motor controller
/*****************************************************************************/
#define DRIVE_R_PWM_PIN 13
#define DRIVE_L_PWM_PIN 12
#define DRIVE_EN_PIN 29
#define MAX_DRIVE_SPEED 125 // 0 - 255

/*****************************************************************************/
// Flywheel motor controller
/*****************************************************************************/
#define FLYWHEEL_R_PWM_PIN 8
#define FLYWHEEL_L_PWM_PIN 9

/*****************************************************************************/
// Side to side motor controller
/*****************************************************************************/
#define S2S_R_PWM_PIN 6
#define S2S_L_PWM_PIN 7
#define S2S_EN_PIN 33
#define S2S_POT_PIN A0
#define S2S_MAX_ANGLE 40
#define S2S_EASING 0.9
#define S2S_OFFSET 30 // Negative value here will tilt the drive to the right

/*****************************************************************************/
// Dome servos and spin motor
/*****************************************************************************/
#define DOME_SPIN_A_PIN 10
#define DOME_SPIN_B_PIN 11
#define DOME_POT_PIN A4
#define DOME_SPIN_SPEED 125 // 0 - 255 for spinning completely around
#define DOME_TURN_SPEED 100 // 0 - 255 for looking around
#define DOME_POT_OFFSET -60
