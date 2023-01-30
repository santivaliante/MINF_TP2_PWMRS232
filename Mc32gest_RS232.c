// Mc32Gest_RS232.C
// Canevas manipulatio TP2 RS232 SLO2 2017-18
// Fonctions d'émission et de réception des message
// CHR 20.12.2016 ajout traitement int error
// CHR 22.12.2016 evolution des marquers observation int Usart
// SCA 03.01.2018 nettoyé réponse interrupt pour ne laisser que les 3 ifs

#include <xc.h>
#include <sys/attribs.h>
#include "system_definitions.h"
// Ajout CHR
#include <GenericTypeDefs.h>
#include "app.h"
#include "GesFifoTh32.h"
#include "Mc32gest_RS232.h"
#include "gestPWM.h"
#include "Mc32CalCrc16.h"


typedef union {
        uint16_t val;
        struct {uint8_t lsb;
                uint8_t msb;} shl;
} U_manip16;


// Definition pour les messages
#define MESS_SIZE  5
// avec int8_t besoin -86 au lieu de 0xAA
#define STX_code  (-86)

#define START 0xAA //Valeur de start

// Structure décrivant le message
typedef struct {
    uint8_t Start;
    int8_t  Speed;
    int8_t  Angle;
    uint8_t MsbCrc;
    uint8_t LsbCrc;
} StruMess;


// Struct pour émission des messages
StruMess TxMess;
// Struct pour réception des messages
StruMess RxMess;

// Declaration des FIFO pour réception et émission
#define FIFO_RX_SIZE ( (4*MESS_SIZE) + 1)  // 4 messages
#define FIFO_TX_SIZE ( (4*MESS_SIZE) + 1)  // 4 messages

int8_t fifoRX[FIFO_RX_SIZE];
// Declaration du descripteur du FIFO de réception
S_fifo descrFifoRX;


int8_t fifoTX[FIFO_TX_SIZE];
// Declaration du descripteur du FIFO d'émission
S_fifo descrFifoTX;


// Initialisation de la communication sérielle
void InitFifoComm(void)
{    
    // Initialisation du fifo de réception
    InitFifo ( &descrFifoRX, FIFO_RX_SIZE, fifoRX, 0 );
    // Initialisation du fifo d'émission
    InitFifo ( &descrFifoTX, FIFO_TX_SIZE, fifoTX, 0 );
    
    // Init RTS 
    RS232_RTS = 1;   // interdit émission par l'autre
   
} // InitComm

 
// Valeur de retour 0  = pas de message reçu donc local (data non modifié)
// Valeur de retour 1  = message reçu donc en remote (data mis à jour)
int GetMessage(S_pwmSettings *pData)
{
    //Variables
    static int commStatus = 0;  //Etat de connexion
    static uint8_t nbrCycles;   //Compteur de cycles
    uint8_t NbCharToRead;       //Nbr de caractere a lire
    int8_t RxC;                 //Reception caractere du fifo
    uint16_t ValCRC = 0;        //Recomposition du CRC lu
    uint16_t ValCRC16 = 0xFFFF; //Pour le calcul du CRC
    
    // Traitement de réception à introduire ICI
    // Lecture et décodage fifo réception
    // ...
    
    //Lecture du nbr de caractere
    NbCharToRead = GetReadSize(&descrFifoRX);
    
    // Si NbCharToRead >= taille message alors traite (5 octets)
    if (NbCharToRead >= MESS_SIZE)
    {
        //Début du mesage
        GetCharFromFifo(&descrFifoRX, &RxC);
        
        //Si le carctere  = 0xAA
        if (RxC == STX_code)  
        {
            //Stockage de Start
            RxMess.Start = RxC;
            
            //Stockage de la vitesse
            GetCharFromFifo(&descrFifoRX, &RxC);         
            RxMess.Speed = RxC;
            
            //Stockage de l'angle
            GetCharFromFifo(&descrFifoRX, &RxC);
            RxMess.Angle = RxC;
            
            //Stockage du CRC MSB
            GetCharFromFifo(&descrFifoRX, &RxC);
            RxMess.MsbCrc = RxC;
            
            //Stockage du CRC LSB
            GetCharFromFifo(&descrFifoRX, &RxC);
            RxMess.LsbCrc = RxC;
            
            //Recomposition du CRC
            ValCRC = (RxMess.LsbCrc) | ((RxMess.MsbCrc) << 8);
            
            //Calcul du CRC sur tout le message
            ValCRC16 = updateCRC16(ValCRC16, STX_code);
            ValCRC16 = updateCRC16(ValCRC16, RxMess.Speed);
            ValCRC16 = updateCRC16(ValCRC16, RxMess.Angle);
            
            
            //Si les CRC ne sont pas égaux, aucun champs serra mis à jour
            if (ValCRC == ValCRC16) 
            {
                //Indicateur de reception à 1
                commStatus = 1; 
                
                //Remet a 0 le nbr de cycle
                nbrCycles = 0; 
                
                //Mettre à jour les champs dans la structure
                pData->SpeedSetting = RxMess.Speed;
                pData->AngleSetting = RxMess.Angle;
            }
            else
            {
              //S'il y a une erreur.
              LED6_W = !LED6_R; // Toggle Led6     
            }
        }
    }
    else
    {      
        //Attendre 10 cycles avant de switcher en mode local
        
        //Si 10 cycles passées 
        if (nbrCycles > 9) 
        {
            //Remet a 0 le nbr de cycle
            nbrCycles = 0; 
            
            //Indicateur de reception à 0
            commStatus = 0;
         }
        
        //Incrémentation du nbr de cycles  
        nbrCycles++;  
    }
    
    // Gestion controle de flux de la réception
    if(GetWriteSpace ( &descrFifoRX) >= (2*MESS_SIZE)) {
        // autorise émission par l'autre
        RS232_RTS = 0;
    }
    return commStatus;
} // GetMessage


// Fonction d'envoi des messages, appel cyclique
void SendMessage(S_pwmSettings *pData)
{
   
    int8_t freeSize;                //Réceptionne la place disponible en écriture
    uint16_t ValCRC16 = 0xFFFF;     //Calcul CRC
    static uint8_t nbrCycles = 0;   //Compteur de cycle 
    
    // Traitement émission à introduire ICI
    // Formatage message et remplissage fifo émission
    // ...
    
     //Si 5 cycle sont passés, l'envoie est autorisé (Ménager la carte distante)
    if (nbrCycles > 4)
    {
        nbrCycles = 0; //Remise à 0 du compteur
        
        //Calcul CRC
        ValCRC16 = updateCRC16(ValCRC16, START);
        ValCRC16 = updateCRC16(ValCRC16, pData->SpeedSetting);
        ValCRC16 = updateCRC16(ValCRC16, pData->AngleSetting);
        
        //Test si place pour ecrire 1 message
        freeSize = GetWriteSpace (&descrFifoTX);       
        if (freeSize >= MESS_SIZE)
        {  
            //Compose le message
            
            //Valeur de START (0xAA)
            TxMess.Start = START;     
            //Valeur de la vitesse
            TxMess.Speed = pData->SpeedSetting;  
            //Valeur de l'angle
            TxMess.Angle = pData->AngleSetting;  
            //Valeur MSB du CRC avec un décalage de 8bits
            TxMess.MsbCrc = (ValCRC16 >>8); 
            //Valeur LSB du CRC avec un masque sur 8bits
            TxMess.LsbCrc = (ValCRC16 & 0x00FF); 

            //Depose le message dans le fifo
            PutCharInFifo(&descrFifoTX, TxMess.Start);
            PutCharInFifo(&descrFifoTX, TxMess.Speed);
            PutCharInFifo(&descrFifoTX, TxMess.Angle);
            PutCharInFifo(&descrFifoTX, TxMess.MsbCrc);
            PutCharInFifo(&descrFifoTX, TxMess.LsbCrc);
        }
       
        // Gestion du controle de flux
        // si on a un caractère à envoyer et que CTS = 0
        freeSize = GetReadSize(&descrFifoTX);
        if ((RS232_CTS == 0) && (freeSize > 0))
        {
            // Autorise int émission    
            PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);                
        }
    }
    else
    {
        nbrCycles ++;
    }
}


// Interruption USART1
// !!!!!!!!
// Attention ne pas oublier de supprimer la réponse générée dans system_interrupt
// !!!!!!!!
 void __ISR(_UART_1_VECTOR, ipl5AUTO) _IntHandlerDrvUsartInstance0(void)
{
    USART_ERROR UsartStatus;  
    
    int8_t c;
    uint8_t freeSize, TXsize;
    int8_t i_cts = 0;
    BOOL TxBuffFull;

    // Marque début interruption avec Led3
    LED3_W = 1;
    
    // Is this an Error interrupt ?
    if ( PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_ERROR) &&
                 PLIB_INT_SourceIsEnabled(INT_ID_0, INT_SOURCE_USART_1_ERROR) ) {
        /* Clear pending interrupt */
        PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_ERROR);
        // Traitement de l'erreur à la réception.
    }
   

    // Is this an RX interrupt ?
    if ( PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_RECEIVE) &&
                 PLIB_INT_SourceIsEnabled(INT_ID_0, INT_SOURCE_USART_1_RECEIVE) ) {
         
        // Oui Test si erreur parité ou overrun
        UsartStatus = PLIB_USART_ErrorsGet(USART_ID_1);

        if ( (UsartStatus & (USART_ERROR_PARITY |
                             USART_ERROR_FRAMING | USART_ERROR_RECEIVER_OVERRUN)) == 0) {

            // Traitement RX à faire ICI
            // Lecture des caractères depuis le buffer HW -> fifo SW
			//  (pour savoir s'il y a une data dans le buffer HW RX : PLIB_USART_ReceiverDataIsAvailable())
			//  (Lecture via fonction PLIB_USART_ReceiverByteReceive())
            // ...
            
            // transfert dans le FIFO software
            // de tous les char reçus
            while (PLIB_USART_ReceiverDataIsAvailable(USART_ID_1)){
                c = PLIB_USART_ReceiverByteReceive(USART_ID_1); //Lecture buffer hardware
                PutCharInFifo(&descrFifoRX, c); //Mise des data dans la fifo software
            }
            LED4_W = !LED4_R; // Toggle Led4 --> Indication Début traitement interruption RX        
            //Si le buffer est vide --> Lever le flag
            PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_RECEIVE);
        } else {
            // Suppression des erreurs
            // La lecture des erreurs les efface sauf pour overrun
            if ( (UsartStatus & USART_ERROR_RECEIVER_OVERRUN) == USART_ERROR_RECEIVER_OVERRUN) {
                   PLIB_USART_ReceiverOverrunErrorClear(USART_ID_1);
            }
        }

        // Traitement controle de flux reception
        freeSize = GetWriteSpace(&descrFifoRX); //Réceptionne la place disponible en écriture
        if (freeSize <= 6) //Si fifo pleine
        {
            RS232_RTS = 1; //Ne plus autoriser l'envoie
        }
    }// end if RX
  
    // Is this an TX interrupt ?
    if ( PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT) &&
                 PLIB_INT_SourceIsEnabled(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT))
    {
       
        // Traitement TX à faire ICI
        // Envoi des caractères depuis le fifo SW -> buffer HW
            
        // Avant d'émettre, on vérifie 3 conditions :
        //  Si CTS = 0 autorisation d'émettre (entrée RS232_CTS)
        //  S'il y a un caratères à émettre dans le fifo
        //  S'il y a de la place dans le buffer d'émission (PLIB_USART_TransmitterBufferIsFull)
        //   (envoi avec PLIB_USART_TransmitterByteSend())
       
        // ...
       
        TXsize = GetReadSize (&descrFifoTX);
        // i_cts = input(RS232_CTS);
        // On vérifie 3 conditions :
        // Si CTS = 0 (autorisation d'émettre)
        // Si il y a un caractère à émettre
        // Si le txreg est bien disponible
        i_cts = RS232_CTS;
        // Il est possible de déposer un caractère
        // tant que le tampon n'est pas plein
        TxBuffFull = PLIB_USART_TransmitterBufferIsFull(USART_ID_1);
        
        if ( (i_cts == 0) && ( TXsize > 0 ) && TxBuffFull == false ) 
        {   
            do 
            {
                GetCharFromFifo(&descrFifoTX, &c);
                PLIB_USART_TransmitterByteSend(USART_ID_1, c);
                i_cts = RS232_CTS;
                TXsize = GetReadSize(&descrFifoTX);
                TxBuffFull = PLIB_USART_TransmitterBufferIsFull(USART_ID_1);       
            }while ( (i_cts == 0) && ( TXsize > 0 ) && TxBuffFull==false );
            
             LED5_W = !LED5_R;// Toggle Led5 --> Indication Début traitement interruption TX
            // Clear the TX interrupt Flag (Seulement apres TX) 
                PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
            if (TXsize == 0) {
                // disable TX interrupt
                // (pour éviter une int inutile)
                PLIB_INT_SourceDisable(INT_ID_0,
                INT_SOURCE_USART_1_TRANSMIT);
            }
            else
            {
                // disable TX interrupt (pour éviter une interrupt. inutile si plus rien à transmettre)
                PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
            }
        } 
        // Marque fin interruption avec Led3
        LED3_W = 0;
    }
}




