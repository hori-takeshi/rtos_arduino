/* File: LCD_m0.h 1.0  */

// Libreria Scritta per Arduino m0 Pro Ver. 1.0
// Funzioni base implementate:
// - Inizializzazione a 4 Bit 					inizializza_LCD()
// - Pulizia Display						Pulisci()
// - Scrittura carattere in posizione desiderata 		Scrivi_Car_PS()
// - Scrittura carattere in posizione corrente			Scrivi_Car_PC();
// - Scrittura testo con scelta della riga			Scrivi_Testo()
// - Attivazione / Disattivazione cursore			cursore_ON();
//								cursore_OFF();

//				BUON LAVORO MARIO MOTTULA						
#ifndef LCD_m0_h
#define LCD_m0_h


#include "Arduino.h"
       


class LCD_m0
{

public:
	
	void Inizializza_LCD(int pin_RS,int pin_EN,int pin_D4,int pin_D5,int pin_D6,int pin_D7);
	void Cursore_ON();
	void Cursore_OFF();
        void SetCommandPin(int D7,int D6,int D5,int D4);   //Funzione interna
        void SetDataPin1(int D7, int D6, int D5, int D4);  //Funzione interna
        void SetDataPin2(int D7, int D6, int D5, int D4);  //Funzione Interna
        void Scrivi_Car_PC(char c);  		           //Scrive carattere nella posizione corrente		
        void Scrivi_Car_PS(char c, int rig, int pos);      //Scrive carattere nella posizione Scelta rig=riga pos = colon.
        void Scrivi_Testo(String stringa, int rig);	   //Scrive del testo specificando la riga
        void Pulisci();					   //Pulisce il Display
        void Posizione(int pos);			   //Funzione Interna				
private:					
	int pin_RS;
	int pin_EN;
	int pin_D4;
	int pin_D5;
	int pin_D6;
	int pin_D7;
        int _pin_RS, _pin_EN,_pin_D4,_pin_D5,_pin_D6,_pin_D7;
        int i,bin[8],bin_inv[8],n,m;
        int D7;
	int D6;
	int D5;
	int D4;
	char c,carattere;
        String stringa;
	int rig;
	int pos;
	
};

#endif

	