//////////////////////////////////////////////////////////////////////////////
// *
// * Predmetni projekat iz predmeta OAiS DSP 2
// * Godina: 2017
// *
// * Zadatak: Ekvalizacija audio signala
// * Autor:
// *                                                                          
// *                                                                          
/////////////////////////////////////////////////////////////////////////////

#include "stdio.h"
#include "ezdsp5535.h"
#include "ezdsp5535_i2c.h"
#include "aic3204.h"
#include "ezdsp5535_aic3204_dma.h"
#include "ezdsp5535_i2s.h"
#include "ezdsp5535_sar.h"
#include "print_number.h"
#include "math.h"

#include "iir.h"
#include "processing.h"

/* Frekvencija odabiranja */
#define SAMPLE_RATE 8000L

#define PI 3.14159265

float frekvencije[6] = {175, 660, 345, 2560, 660, 4000}; //prvi parametar je LowShelving, drugi i treci su za prvi peek, cetvrti i peti za drugi peek, poslednji je za high-shelving

float my_alpha[4] = {0.933535305303274, 0.872932052778240, 0.769408802609314, 0};
float my_beta[2] = {0.966600102092809, 0.535826795948905};

/* Niz za smestanje ulaznih i izlaznih odbiraka */
#pragma DATA_ALIGN(sampleBufferL,4)
Int16 sampleBufferL[AUDIO_IO_SIZE];
#pragma DATA_ALIGN(sampleBufferR,4)
Int16 sampleBufferR[AUDIO_IO_SIZE];

Int16 dirakBuff[AUDIO_IO_SIZE];
Int16 outputEQ_LEFT[AUDIO_IO_SIZE];
Int16 outputEQ_RIGHT[AUDIO_IO_SIZE];
Int16 shelvingHighOutput[4]; //Koeficijenti High pass filtra
Int16 shelvingLowOutput[4]; //Koeficijenti Low pass filtra
Int16 shelvingPeekOutput1[6]; //Koeficijenti prvog Peek filtra
Int16 shelvingPeekOutput2[6]; //Koeficijenti drugog Peek filtra

//HISTORI ZA LEVI BAFER
Int16 history_x_HP_L[1] = {0};
Int16 history_y_HP_L[1] = {0};
Int16 history_x_LP_L[1] = {0};
Int16 history_y_LP_L[1] = {0};
Int16 history_x_P1_L[2] = {0, 0};
Int16 history_y_P1_L[2] = {0, 0};
Int16 history_x_P2_L[2] = {0, 0};
Int16 history_y_P2_L[2] = {0, 0};

//HISTORI ZA DESNI BAFER
Int16 history_x_HP_R[1] = {0};
Int16 history_y_HP_R[1] = {0};
Int16 history_x_LP_R[1] = {0};
Int16 history_y_LP_R[1] = {0};
Int16 history_x_P1_R[2] = {0, 0};
Int16 history_y_P1_R[2] = {0, 0};
Int16 history_x_P2_R[2] = {0, 0};
Int16 history_y_P2_R[2] = {0, 0};

Int16 k[4] = { 16000, 16000, 16000, 16000 };
int k_i = 0;


Uint16 getKey() {
    static Uint16 old = NoKey;
    Uint16 key = EZDSP5535_SAR_getKey();
    if (key == old) {
        return NoKey;
    } else {
        old = key;
        return key;
    }
}

void printNewValue() {
	setWritePointerToFirstChar();
	/* Indeks broja k */
	printChar('0' + k_i);
	printChar(' ');
	/* Vrednost broja k */
	if (k[k_i] == 32767) {
		printChar('1');
		printChar('.');
		printChar('0');
	} else {
		printChar('0');
		printChar('.');
		printChar('1' + k[k_i] / 3277);
	}
}

void main( void )
{   
	int i;

    /* Inicijalizaija razvojne ploce */
    EZDSP5535_init( );

    /* Inicijalizacija kontrolera za ocitavanje vrednosti pritisnutog dugmeta*/
    EZDSP5535_SAR_init();

    /* Inicijalizacija LCD kontrolera */
    initPrintNumber();

	printf("\n Ekvalizacija audio signala \n");
		
    /* Inicijalizacija veze sa AIC3204 kodekom (AD/DA) */
    aic3204_hardware_init();
	
    /* Inicijalizacija AIC3204 kodeka */
	aic3204_init();

    aic3204_dma_init();
    

    //GENERISANJE DIRAKA
    dirakBuff[0] = 16000;
    for(i = 1; i < AUDIO_IO_SIZE; i++) {
    	dirakBuff[i] = 0;
    }




    /* Postavljanje vrednosti frekvencije odabiranja i pojacanja na kodeku */
    set_sampling_frequency_and_gain(SAMPLE_RATE, 0);

    calculateShelvingCoeff(my_alpha[0], shelvingLowOutput);
    calculateShelvingCoeff(my_alpha[3], shelvingHighOutput);
    calculatePeekCoeff(my_alpha[1], my_beta[0], shelvingPeekOutput1);
    calculatePeekCoeff(my_alpha[2], my_beta[1], shelvingPeekOutput2);


    while(1)
    {
    	aic3204_read_block(sampleBufferL, sampleBufferR);

    	Uint16 key = getKey();

		switch (key) {
			case SW1:
				(k_i + 1 == 4) ? k_i = 0 : ++k_i;
				printNewValue();
				break;
			case SW2:
				k[k_i] -= 3277;
				if (k[k_i] < 0) {
					k[k_i] = 32767;
				}
				printNewValue();
				break;
		}

		setWritePointerToFirstChar();

    	for(i = 0; i<AUDIO_IO_SIZE; i++) {

    		outputEQ_LEFT[i] = shelvingLP(sampleBufferL[i], shelvingLowOutput, history_x_LP_L, history_y_LP_L, k[0]);
    		outputEQ_LEFT[i] = shelvingPeek(outputEQ_LEFT[i], shelvingPeekOutput1, history_x_P1_L, history_y_P1_L, k[1]);
    		outputEQ_LEFT[i] = shelvingPeek(outputEQ_LEFT[i], shelvingPeekOutput2, history_x_P2_L, history_y_P2_L, k[2]);
    		outputEQ_LEFT[i] = shelvingHP(outputEQ_LEFT[i], shelvingHighOutput, history_x_HP_L, history_y_HP_L, k[3]);

			outputEQ_RIGHT[i] = shelvingLP(sampleBufferR[i], shelvingLowOutput, history_x_LP_R, history_y_LP_R, k[0]);
			outputEQ_RIGHT[i] = shelvingPeek(outputEQ_RIGHT[i], shelvingPeekOutput1, history_x_P1_R, history_y_P1_R, k[1]);
			outputEQ_RIGHT[i] = shelvingPeek(outputEQ_RIGHT[i], shelvingPeekOutput2, history_x_P2_R, history_y_P2_R, k[2]);
			outputEQ_RIGHT[i] = shelvingHP(outputEQ_RIGHT[i], shelvingHighOutput, history_x_HP_R, history_y_HP_R, k[3]);
    	}


		aic3204_write_block(sampleBufferR, sampleBufferR);
	}

    	
	/* Prekid veze sa AIC3204 kodekom */
    aic3204_disable();

    printf( "\n***Kraj programa***\n" );
	SW_BREAKPOINT;
}


