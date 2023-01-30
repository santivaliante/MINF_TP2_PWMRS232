/*--------------------------------------------------------*/
// GestPWM.c
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

#include "GestPWM.h"
#include "Mc32DriverLcd.h"
#include "Mc32DriverAdc.h"
#include "bsp.h"
#include "peripheral/oc/plib_oc.h"
#include "framework/driver/oc/drv_oc_static.h"
#include "framework/driver/tmr/drv_tmr_static.h"



S_pwmSettings PWMData;            //Pour les settings
S_pwmSettings PWMDataToSend;      //Pour les donnee a envoyer
S_ADCResults AdcResult;           //Pour la lecture des ADC
static uint16_t tabMoyenne0 [10]; //Tableau pour moyenne glissante canal 0
static uint16_t tabMoyenne1 [10]; //Tableau pour moyenne glissante canal 1

void GPWM_Initialize(S_pwmSettings *pData)
{
    // Start the 4 Timers
    DRV_TMR0_Start();
    DRV_TMR1_Start();
    DRV_TMR2_Start();
    
    // Start the 2 Output Compares
    DRV_OC0_Start();
    DRV_OC1_Start();
    // Init les data 
    PWMData.AngleSetting = 0;
    PWMData.SpeedSetting = 0;
    PWMData.absAngle = 0;
    PWMData.absSpeed = 0;
    // Init l'état du pont en H
    BSP_EnableHbrige();
    // Init les data du tableau
    int i;
    for (i=0 ; i<= 9 ; i++)  //Mettre à 0 les 10 cases
    {
        tabMoyenne0 [i] = 0;
        tabMoyenne1 [i] = 0;
    }
}

// Obtention vitesse et angle (mise a jour des 4 champs de la structure)
void GPWM_GetSettings(S_pwmSettings *pData)	
{ 
    //Lecture des ADCs
    AdcResult = BSP_ReadAllADC();       
      
    //Variables Moyenne glissante
    static uint8_t count = 0;       //Compteur stockage ADC
    int moyenne0 = 0;               //Reception de la val moyenne CH0  SPEED         
    int moyenne1 = 0;               //Reception de la val moyenne CH1  ANGLE
    uint8_t i;                      //Variable boucle for
    
    //Injection val ADC dans le tableaux en fonction du compteur
    tabMoyenne0[count] = AdcResult.Chan0;
    tabMoyenne1[count] = AdcResult.Chan1;
    count++;                       //Incrémentation du compteur
    
    //Test de la valeur max du compteur
    if (count > VAL_MOYENNE - 1)
    {
        count = 0; //Remise à 0 du compteur
    }
    
    // Calcul la moyenne glissante
    for(i = 0 ; i < VAL_MOYENNE; i++)
    {
        moyenne0 += tabMoyenne0[i];
        moyenne1 += tabMoyenne1[i];
    }
    moyenne0 = moyenne0 / VAL_MOYENNE;
    moyenne1 = moyenne1 / VAL_MOYENNE;
    
    // Conversions
    //absSpeed = (moyenne * 198) / 1023 - 99 (calcul non-signé)
    pData->absSpeed = abs((moyenne0 * VAL_MAX_VITESSEABS) / VAL_MAX_ADC - VAL_MAX_VITESSE);
    //SpeedSetting = (moyenne * 198) / 1023 - 99 (calcul signé)
    pData->SpeedSetting = (signed)(moyenne0 * VAL_MAX_VITESSEABS) / VAL_MAX_ADC - VAL_MAX_VITESSE;
    //absAngle = (moyenne * 180) / 1023 
    pData->absAngle = (moyenne1 * VAL_MAX_ANGLEABS) / VAL_MAX_ADC ;
    //AngleSetting = absAngle - 90
    pData->AngleSetting = (int8_t) pData->absAngle - VAL_MAX_ANGLE;
}

// Affichage des information en exploitant la structure
void GPWM_DispSettings(S_pwmSettings *pData, int Remote)
{
    if (Remote == 1)
    {
        // Clear the line 2, 3 and 4
        lcd_ClearLine(2);
        lcd_ClearLine(3);
        lcd_ClearLine(4);
        // Go to the line 1
        lcd_gotoxy(1,1);
        printf_lcd("Remote Settings");
    }
    else
    {
        // Clear the line 2, 3 and 4
        lcd_ClearLine(2);
        lcd_ClearLine(3);
        lcd_ClearLine(4);
         // Go to the line 1
        lcd_gotoxy(1,1);
        printf_lcd("Local Settings");        
    }
        lcd_gotoxy(1,2);
        printf_lcd("SpeedSetting = %3d",pData->SpeedSetting);
        // Go to the line 3
        lcd_gotoxy(1,3);
        printf_lcd("absSpeed     =  %2d",abs(pData->absSpeed));
        // Go to the line 4
        lcd_gotoxy(1,4);
        printf_lcd("Angle        = %3d",pData->AngleSetting);
}

// Execution PWM et gestion moteur à partir des info dans structure
void GPWM_ExecPWM(S_pwmSettings *pData)
{
    //Gestion du sens de rotation du moteur en fonction de la datasheet du TB6612FNG
    if (pData->SpeedSetting == 0)
    {
        //Moteur ne tourne pas
        AIN1_HBRIDGE_W = 0;
        AIN2_HBRIDGE_W = 0;
    }
    else if (pData->SpeedSetting > 0)
    {
        //Moteur tourne a droite
        AIN1_HBRIDGE_W = 0;
        AIN2_HBRIDGE_W = 1;
    }
    else
    {
        //Moteur tourne a gauche
        AIN1_HBRIDGE_W = 1;
        AIN2_HBRIDGE_W = 0;
    }
    
    // Update the period register of the 2 OCs
    PLIB_OC_PulseWidth16BitSet(OC_ID_2, (MULT_FACT_MOTOR * pData->absSpeed));
    PLIB_OC_PulseWidth16BitSet(OC_ID_3, (OFFSET_600US_SERVO + MULT_FACT_SERVO * pData->absAngle));
    // Calculations are explained next to the defines 
}

// Execution PWM software
void GPWM_ExecPWMSoft(S_pwmSettings *pData){
    
    static uint8_t counter = 0;
    
    // Define the duty cycle (T_on) of the PWM signal depending on the value of the absolute speed
    if(counter >= pData->absSpeed){
        
        BSP_LEDStateSet(BSP_LED_2, 0);
    }
    
    // Define the period of the PWM
    if(counter > 99){
        if((pData->absSpeed) > 0) BSP_LEDStateSet(BSP_LED_2, 1);
        counter = 0;
    }
    
    // Incrementation counter, 1 = 35us
    counter++;
}


