
/*Programma di esempio per la gestione di un display LCD a duer righe con controller Hitachi HD44780

Verrà visualizzata sullaprima riga la Scritta Ciao con il cursore ON
Mentre nella seconda riga verrà visualizzata la scritta Mondo !! con cursore OFF
Dopo 3 secondi si vedranno lampeggiare i simboli * 

Buon Lavoro !! Mario Mottula

Per info: mmottula@hotmail.com

Di seguito la Configurazione dei pin Standard:

 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 5
 * LCD D5 pin to digital pin 4
 * LCD D6 pin to digital pin 3
 * LCD D7 pin to digital pin 2
 * LCD R/W pin to ground */

//Inserisco la libreria LCD_m0 nel programma
#include <LCD_m0.h>

//Rinomino per comodità la libreria LCD_m0 in LCD
LCD_m0 LCD;
void setup() {
//Attendo 50ms per la stabilizzazione della tensione di alimentazione
delay(50);
//Richiamo la Routine di Inizializzazione passando come parametri i pin di Reset , Enable e di Dati
LCD.Inizializza_LCD(12,11,5,4,3,2); //D4,D5,D6,D7

}

void loop() {
  //Ad ogni ciclo posso eseguire la pulizia del display
  LCD.Pulisci();
  delay(1000);
  //Accendo il cursore
  LCD.Cursore_ON();
  delay(1000);
  //Scrivo il Testo desiderato
  LCD.Scrivi_Testo("     Ciao       ",1);
  //Spengo il cursore
  LCD.Cursore_OFF();
  LCD.Scrivi_Testo("    Mondo !!    ",2);
  delay(3000);
  //Scrive il carattere * nella posizione desiderata
  LCD.Scrivi_Car_PS('*',1,4);
  LCD.Scrivi_Car_PS('*',1,11);
  delay(500);
  LCD.Scrivi_Car_PS(' ',1,4);
  LCD.Scrivi_Car_PS(' ',1,11);
  delay(500);
  LCD.Scrivi_Car_PS('*',1,4);
  LCD.Scrivi_Car_PS('*',1,11);
  delay(2000);
  Blink();      //Lampeggio che indica la fine della routine
}

void Blink()

{
  digitalWrite(13,HIGH);
  delay(250);
  digitalWrite(13,LOW);
  delay(250);
  }

