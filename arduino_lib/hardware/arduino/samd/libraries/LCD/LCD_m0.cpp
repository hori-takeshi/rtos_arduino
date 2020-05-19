//   Versione 1.0 By Mario Mottula

#include "Arduino.h"
#include "LCD_m0.h"
#define H HIGH 
#define L LOW 


//************************************************************
//* Funzione di inizializzazione display
//* 4 bit di dati
//* 2 righe e font Standard 5x8
//************************************************************
void LCD_m0::Inizializza_LCD(int pin_RS, int pin_EN,int pin_D4,int pin_D5, int pin_D6, int pin_D7)

{
  _pin_RS = pin_RS;
  _pin_EN = pin_EN;
  _pin_D7 = pin_D7;
  _pin_D6 = pin_D6;
  _pin_D5 = pin_D5;
  _pin_D4 = pin_D4;

  
  //ritardo per stabilizzazione Vdc
  delay(250);
  pinMode(pin_RS, OUTPUT);
  pinMode(pin_EN, OUTPUT);
  pinMode(pin_D7, OUTPUT);
  pinMode(pin_D6, OUTPUT);
  pinMode(pin_D5, OUTPUT);
  pinMode(pin_D4, OUTPUT);
 
  delay(50);
  SetCommandPin(0,0,1,1); 
  delay(10);
  SetCommandPin(0,0,1,1); 
  delay(1);
  SetCommandPin(0,0,1,1); 
   delay(50);
//impostazione interfaccia 4 bit
  SetCommandPin(0,0,1,0);
  delay(5);
   
//impostazione font e righe (2 righe Font 5x8)
   
  SetCommandPin(0,0,1,0);
  SetCommandPin(1,0,0,0);
  delay(5);
	
// impostazione Off display

   SetCommandPin(0,0,0,0); 
   SetCommandPin(1,0,0,0); 
   delay(5);

   
// impostazione On display cursore OFF
   
   SetCommandPin(0,0,0,0); 
   SetCommandPin(1,1,0,0); 
   delay(5);

// pulizia display
   
   SetCommandPin(0,0,0,0); 
   SetCommandPin(0,0,0,0); 
   delay(5);

  
//shift a sinistra cursore (Entry mode da SX a DX)
  
   SetCommandPin(0,0,0,0); 
   SetCommandPin(0,1,1,0); 
   



  
   }


//************************************************************
//*	Funzione di invio Comandi
//************************************************************
void LCD_m0::SetCommandPin(int D7,int D6,int D5, int D4)
{
  digitalWrite(_pin_RS, L); 

  digitalWrite(_pin_D7, D7); 
  digitalWrite(_pin_D6, D6); 
  digitalWrite(_pin_D5, D5); 
  digitalWrite(_pin_D4, D4);  
  digitalWrite(_pin_EN, H);
  delay(1); 
  digitalWrite(_pin_EN, L);  


}

//************************************************************
//*	Funzione di invio Dati primi 4 bit
//************************************************************

void LCD_m0::SetDataPin1(int D7,int D6,int D5, int D4)
{
  
 
  digitalWrite(_pin_EN, H);
  delay(1);
  digitalWrite(_pin_RS , H); 

  digitalWrite(_pin_D7, D7); 
  digitalWrite(_pin_D6, D6); 
  digitalWrite(_pin_D5, D5); 
  digitalWrite(_pin_D4, D4);  

  
  digitalWrite(_pin_EN, L);
  delay(1);
  digitalWrite(12, H); 

}
//************************************************************
//*	Funzione di invio Dati secondi 4 bit
//************************************************************

void LCD_m0::SetDataPin2(int D7,int D6,int D5, int D4)
{
  
  digitalWrite(_pin_EN, H);
  delay(1);
  digitalWrite(_pin_RS , H); 

  digitalWrite(_pin_D7, D7); 
  digitalWrite(_pin_D6, D6); 
  digitalWrite(_pin_D5, D5); 
  digitalWrite(_pin_D4, D4);  


  digitalWrite(_pin_EN, L);
  delay(1);
  digitalWrite(_pin_RS , L); 


}




//************************************************************
//* Funzione di attivazione Cursore
//************************************************************
void LCD_m0::Cursore_ON()

{
   SetCommandPin(0,0,0,0); 
   SetCommandPin(1,1,1,0); 
}



//************************************************************
//* Funzione di disattivazione Cursore
//************************************************************

void LCD_m0::Cursore_OFF()
{
	SetCommandPin(0,0,0,0); 
  	SetCommandPin(1,1,0,0); 
   
}



//************************************************************
//* Funzione di scrittura carattere in posizione corrente
//* c = carattere da scrivere in posizione corrente
//************************************************************

void LCD_m0::Scrivi_Car_PC(char c)
{
 carattere = c;
  
 for(i=0;i<8;i++) 
	{
  	bin[i]=((c>>i)&1);
	}

 for(i=0;i<8;i++) 
     {
      bin_inv[i]=bin[7-i];
	}
 SetDataPin1(bin_inv[0],bin_inv[1],bin_inv[2],bin_inv[3]);
   delay(50);
 SetDataPin2(bin_inv[4],bin_inv[5],bin_inv[6],bin_inv[7]);

}


//************************************************************
//* Funzione di scrittura di un carattere su riga e posizione
//* desiderate.
//* c = carttere da scrivere 
//* rig = intero con il numero di riga
//* pos = intero con il numero della posizione 
//************************************************************
void LCD_m0::Scrivi_Car_PS(char c,int rig,int pos)
{
 carattere = c;
 //SerialUSB.begin(9600);
 //SerialUSB.println(carattere);
 if(rig==1){ 
 			SetCommandPin(1,0,0,0);//selezione riga1
			Posizione(pos);
		}
 if(rig==2){
			SetCommandPin(1,1,0,0);//selezione riga 2
			Posizione(pos);
		}
 
 for(i=0;i<8;i++) 
	{
  	bin[i]=((c>>i)&1);
     
	}
 for(i=0;i<8;i++) 
     {
     
  	bin_inv[i]=bin[7-i];
     
	}
 SetDataPin1(bin_inv[0],bin_inv[1],bin_inv[2],bin_inv[3]);
   delay(50);
 SetDataPin2(bin_inv[4],bin_inv[5],bin_inv[6],bin_inv[7]);

}


void LCD_m0::Posizione(int pos)
{
 switch (pos)  
  {  
  case 0:  
  SetCommandPin(0,0,0,0);   
  break;  

  case 1:  
  SetCommandPin(0,0,0,0);    
  break;  
  
  case 2:  
  SetCommandPin(0,0,0,1);    
  break;  
  
  case 3:  
  SetCommandPin(0,0,1,0);   
  break;  

  case 4:  
  SetCommandPin(0,0,1,1);    
  break;  
  
  case 5:  
  SetCommandPin(0,1,0,0);    
  break;  
  
  case 6:  
  SetCommandPin(0,1,0,1);    
  break;  

  case 7:  
  SetCommandPin(0,1,1,0);    
  break;  

  case 8:  
  SetCommandPin(0,1,1,1);    
  break; 
  
  case 9:  
  SetCommandPin(1,0,0,0);   
  break;  

  case 10:  
  SetCommandPin(1,0,0,1);    
  break;  
  
  case 11:  
  SetCommandPin(1,0,1,0);    
  break;  
  
  case 12:  
  SetCommandPin(1,0,1,1);    
  break;  

  case 13:  
  SetCommandPin(1,1,0,0);    
  break;  
  
  case 14:  
  SetCommandPin(1,1,0,1);   
  break;  
  
  case 15:  
  SetCommandPin(1,1,1,0);   
  break;  

  case 16: 
  SetCommandPin(1,1,1,1);   
  break;  


  }  
}



//*********************************************************
//*Permette di scrivere un 'intera riga di testo
//*stringa = Testo da scrivere  Formato String
//*rig = riga sulla quale scrivere il testo  Formato int
//*********************************************************

void LCD_m0::Scrivi_Testo(String stringa,int rig)
{
 
n = stringa.length();


 if(rig==1){ 
 			SetCommandPin(1,0,0,0);
			SetCommandPin(0,0,0,0);

			for(m=0; m<=n-1; m++){
		               			c = stringa[m];	  
		     					
 							Scrivi_Car_PC(c);
 							}
		  }


 if(rig==2){ 
                SetCommandPin(1,1,0,0);//selezione riga1
			SetCommandPin(0,0,0,0);//offset

 			for(m=0; m<=n-1; m++){
		               			c = stringa[m];	  
		     					
 							Scrivi_Car_PC(c);

 							}
		  }

}



void LCD_m0::Pulisci()
{

   SetCommandPin(0,0,0,0); 
   SetCommandPin(0,0,0,1); 

}

