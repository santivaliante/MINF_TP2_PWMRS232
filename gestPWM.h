#ifndef GestPWM_H
#define GestPWM_H
/*--------------------------------------------------------*/
// GestPWM.h
/*--------------------------------------------------------*/
//	Description :	Gestion des PWM 
//			        TP1 2022-2023
//
//	Auteur 		: 	C. HUBER / MRI / SVE
//
//	Version		:	V1.1
//	Compilateur	:	XC32 V1.42 + Harmony 1.08
//
/*--------------------------------------------------------*/

#include <stdint.h>



/*--------------------------------------------------------*/
// Définition des fonctions prototypes
/*--------------------------------------------------------*/

#define VAL_MAX_ADC 1023
#define VAL_MOYENNE 10

#define VAL_MAX_VITESSE 99
#define VAL_MAX_VITESSEABS 198
#define VAL_MAX_ANGLE 90
#define VAL_MAX_ANGLEABS 180


#define OFFSET_600US_SERVO 3000 // (35000 / 7) * 0.6 = 3000
#define MULT_FACT_SERVO 50 // (((35000 / 7) * 2.4) - 3000) / 180 = 50  

#define OFFSET_600US_SERVO 3000 // (35000 / 7) * 0.6 = 3000 // 3000 correspond à 600us (temps min impultion)
#define MULT_FACT_SERVO 50 // (((35000 / 7) * 2.4) - 3000) = 50  

#define MULT_FACT_MOTOR 20 // 2000 / 100 = 20

typedef struct {
    uint8_t absSpeed;    // vitesse 0 à 99
    uint8_t absAngle;    // Angle  0 à 180
    int8_t SpeedSetting; // consigne vitesse -99 à +99
    int8_t AngleSetting; // consigne angle  -90 à +90
} S_pwmSettings;

extern S_pwmSettings PWMData, PWMDataToSend; 

void GPWM_Initialize(S_pwmSettings *pData);

// Ces 3 fonctions ont pour paramètre un pointeur sur la structure S_pwmSettings.
void GPWM_GetSettings(S_pwmSettings *pData);	// Obtention vitesse et angle
void GPWM_DispSettings(S_pwmSettings *pData, int Remote);	// Affichage
void GPWM_ExecPWM(S_pwmSettings *pData);		// Execution PWM et gestion moteur.
void GPWM_ExecPWMSoft(S_pwmSettings *pData);		// Execution PWM software.


#endif
