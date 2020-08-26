/* USER CODE BEGIN Header */

/*
 * drolja sa frekvencijom regulacije 25KHz za ulazne i bat, a 12.5KHz za izlazne
 * svaka prekidna je podeljena na 3 dela: priprema ADC, usrednjavanje izmerenog, regulacija
 * prekidnu uvek zove DMA, a koja se prekidna izvrsava se odlucuje pomocu brojaca dma_cnt
 */

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */

volatile int dma_cnt = 0, i1=0, i2=0;
volatile float v_pv1, i_pv1, vold_pv1 = 0, iold_pv1 = 0, e_pv1, eold_pv1 = 0, d_pv1, dold_pv1 = 0.8, p_pv1, pold_pv1 = 0;			//za prvi ulazni buck
volatile float v_pv2, i_pv2, vold_pv2 = 0, iold_pv2 = 0, e_pv2, eold_pv2 = 0, d_pv2, dold_pv2 = 0.8, p_pv2, pold_pv2 = 0;			//za drugi ulazni buck
volatile float v_dc;																												//za link
volatile float v_5v, e_5v, eold_5v = 0, d_5v, dold_5v = 0.835;																		//za 5v izlaz
volatile float v_3v3, e_3v3, eold_3v3 = 0, d_3v3, dold_3v3 = 0.55;																	//za 3v3 izlaz
volatile float v_bat, i_bat, e_bat, eold_bat = 0, d_bat, dold_bat = 0.75;															//za bateriju

volatile int zaokruzi;
volatile float step;
const float step_mppt = 0.0002;

//koeficijenti za razdelnike - svi podlozni promenama
const float v_dc_coef = 0.0026333;			//0-9V
const float v_pv_coef = 0.00629;			//0-25V
const float i_pv_coef = 0.0004884;			//?????
const float v_3v3_coef = 0.0018156;			//0-6.6V
const float v_5v_coef = 0.0022055;			//0-8V

//koeficijenti za regulaciju
const float reg1_3v3 = 0.002;
const float reg2_3v3 = 0.001;
const float reg1_5v = 0.002;
const float reg2_5v = 0.001;
const float reg1_bat = 0.002;
const float reg2_bat = 0.001;
const float reg_pv = 0.005;				//za pv pomeranje
const float ref_3v3 = 3.3;					//referenca koja ne moze da se upise sa vmov

//bufferi za DMA:
uint32_t pv1_buffer[40], pv2_buffer[40], bat_buffer[24], out3v3_buffer[16], out5v_buffer[16];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void DMA1_Channel1_IRQHandler(void)
{
	if (DMA1 -> ISR & 2) {							//transfer complete se obradjuje
		DMA1 -> IFCR = 2;

		ADC1 -> CR |= 0x10;							//ADC stop (mora OR a ne dodela da se ne bi pojebala kalibracija
		DMA1_Channel1 -> CCR &= 0xFFFFFFFE;			//disable DMA

    	dma_cnt++;									//uvecaj brojac radi pracenja u kojoj si prekidnoj

		while (DMA1_Channel1 -> CCR & 1);			//cekaj disable
		//while (ADC1 -> CR & 4);	//nije neophodno cekati da se adc stopira jer ima dosta instrukcija izmedju


		/* KORISTAN DEO KODA */
		switch (dma_cnt) {								//provera u kojoj smo prekidnoj

		/* pv1 */
		case 1:
		case 5:
			/* priprema sledeceg merenja */
			DMA1_Channel1 -> CMAR = (int) bat_buffer;	//postavi buffer za upis
			DMA1_Channel1 -> CNDTR = 24;				//koliko upisa
			ADC1 -> SQR1 = 0x001C8142;					//kanali 5,8,7 (Udc, Ub, Ib)
			DMA1_Channel1 -> CCR |= 1;					//enable DMA
		    ADC1 -> CR |= 0x4;							//ADC start
			/* kraj pripreme merenja */

			/* usrednjavanje */
			v_dc  = (pv1_buffer[2] + pv1_buffer[7] + pv1_buffer[12] + pv1_buffer[17] + pv1_buffer[22] + pv1_buffer[27] + pv1_buffer[32] + pv1_buffer[37])>>3;
			asm("ldr r0, =v_dc;"
					"ldr r1, [r0];"
			  		"ldr r2, =v_dc_coef;"
			  		"ldr r3, [r2];"
			   		"vmov s2, r1;"
			   		"vmov s4, r3;"
			   		"vmul.f32 s2, s2, s4;"
			   		"vmov r1, s2;"
			   		"str r1, [r0]");
			v_pv1 = (pv1_buffer[1] + pv1_buffer[6] + pv1_buffer[11] + pv1_buffer[16] + pv1_buffer[21] + pv1_buffer[26] + pv1_buffer[31] + pv1_buffer[36])>>3;
			asm("ldr r0, =v_pv1;"
			   		"ldr r1, [r0];"
			  		"ldr r2, =v_pv_coef;"
			   		"ldr r3, [r2];"
			   		"vmov s2, r1;"
			   		"vmov s4, r3;"
			   		"vmul.f32 s2, s2, s4;"
			   		"vmov r1, s2;"
			   		"str r1, [r0]");
			v_pv2 = (pv1_buffer[0] + pv1_buffer[5] + pv1_buffer[10] + pv1_buffer[15] + pv1_buffer[20] + pv1_buffer[25] + pv1_buffer[30] + pv1_buffer[35])>>3;
			asm("ldr r0, =v_pv2;"
			   		"ldr r1, [r0];"
			  		"ldr r2, =v_pv_coef;"
			   		"ldr r3, [r2];"
			   		"vmov s2, r1;"
			   		"vmov s4, r3;"
			   		"vmul.f32 s2, s2, s4;"
			   		"vmov r1, s2;"
			   		"str r1, [r0]");
			i_pv1 = (pv1_buffer[4] + pv1_buffer[9] + pv1_buffer[14] + pv1_buffer[19] + pv1_buffer[24] + pv1_buffer[29] + pv1_buffer[34] + pv1_buffer[39])>>3;
			asm("ldr r0, =i_pv1;"
			   		"ldr r1, [r0];"
			  		"ldr r2, =i_pv_coef;"
			   		"ldr r3, [r2];"
			   		"vmov s2, r1;"
			   		"vmov s4, r3;"
			   		"vmul.f32 s2, s2, s4;"
			   		"vmov r1, s2;"
			   		"str r1, [r0]");
			i_pv2 = (pv1_buffer[3] + pv1_buffer[8] + pv1_buffer[13] + pv1_buffer[18] + pv1_buffer[23] + pv1_buffer[28] + pv1_buffer[33] + pv1_buffer[38])>>3;
			asm("ldr r0, =i_pv2;"
			   		"ldr r1, [r0];"
			  		"ldr r2, =i_pv_coef;"
			   		"ldr r3, [r2];"
			   		"vmov s2, r1;"
			   		"vmov s4, r3;"
			   		"vmul.f32 s2, s2, s4;"
			   		"vmov r1, s2;"
			   		"str r1, [r0]");
			/* kraj usrednjavanja */

			/* regulacija */

			/* PV POMERANJE */

//			//brze ovako nego asm
//		if(i1==0){
//			e_pv1 = 6.0 - v_dc;
//
//			//step = 0.00005 * e_pv1;
//			asm("ldr r0, =step;"
//					"ldr r2, =reg_pv;"
//					"ldr r4, =e_pv1;"
//					"ldr r3, [r2];"
//					"ldr r5, [r4];"
//					"vmov s4, r3;"
//					"vmov s2, r5;"
//					"vmul.f32 s2, s2, s4;"
//					"vmov r1, s2;"
//					"str r1, [r0]");
//
//			//p_pv1 = i_pv1 * v_pv1;
//			asm("ldr r0, =p_pv1;"
//					"ldr r2, =i_pv1;"
//					"ldr r4, =v_pv1;"
//					"ldr r3, [r2];"
//					"ldr r5, [r4];"
//					"vmov s4, r3;"
//					"vmov s2, r5;"
//					"vmul.f32 s2, s2, s4;"
//					"vmov r1, s2;"
//					"str r1, [r0]");
//
//			if (p_pv1 > pold_pv1) {
//				if (v_pv1 > vold_pv1) {
//					d_pv1 = dold_pv1 - step;
//				}
//				else {
//					d_pv1 = dold_pv1 + 0.01;
//				}
//			}
//			else {
//				if (v_pv1 < vold_pv1) {
//					d_pv1 = dold_pv1 - step;
//				}
//				else {
//					d_pv1 = dold_pv1 + 0.01;
//				}
//			}
//
//			//ogranicavanje djutija
//			if (d_pv1 > 0.9) d_pv1 = 0.9;
//			else if (d_pv1 < 0.2) d_pv1 = 0.2;
//
//			//ovo je brze ovako nego u asm
//			zaokruzi = (volatile int) (d_pv1 * 200);
//			if (d_pv1 * 200 - zaokruzi >= 0.5f) zaokruzi++;
//			TIM1 -> CCR1 = zaokruzi;
//
//			dold_pv1 = d_pv1;
//			pold_pv1 = p_pv1;
//			vold_pv1 = v_pv1;
//			i1=0;
//			}else{
//				i1++;
//			}

			/* MPPT */

//			//p_pv1 = i_pv1 * v_pv1;
//			asm("ldr r0, =p_pv1;"
//					"ldr r2, =i_pv1;"
//					"ldr r4, =v_pv1;"
//					"ldr r3, [r2];"
//					"ldr r5, [r4];"
//					"vmov s4, r3;"
//					"vmov s2, r5;"
//					"vmul.f32 s2, s2, s4;"
//					"vmov r1, s2;"
//					"str r1, [r0]");
//
//			if (i_pv1 < 0.001 && v_pv1 > 0) {
//				//d_pv1 = dold_pv1 + 100 * step_mppt;
//				d_pv1 = dold_pv1 + 0.01;
//			}
//			else {
//				if (p_pv1 > pold_pv1) {
//					if (v_pv1 > vold_pv1) d_pv1 = dold_pv1 - step_mppt;
//					else d_pv1 = dold_pv1 + step_mppt;
//				}
//				else {
//					if (v_pv1 < vold_pv1) d_pv1 = dold_pv1 - step_mppt;
//					else d_pv1 = dold_pv1 + step_mppt;
//				}
//			}
//
//			//ogranicavanje djutija
//			if (d_pv1 > 0.9) d_pv1 = 0.9;
//			else if (d_pv1 < 0.1) d_pv1 = 0.1;
//
//			//ovo je brze ovako nego u asm
//			zaokruzi = (volatile int) (d_pv1 * 200);
//			if (d_pv1 * 200 - zaokruzi > 0.5f) zaokruzi++;
//			TIM1 -> CCR1 = zaokruzi;
//
//			dold_pv1 = d_pv1;
//			pold_pv1 = p_pv1;
//			vold_pv1 = v_pv1;

			/* kraj regulacije */

 			break;


		/* baterija */
		case 2:
		case 6:
			/* priprema sledeceg merenja */
			DMA1_Channel1 -> CMAR = (int) pv2_buffer;	//postavi buffer za upis
			DMA1_Channel1 -> CNDTR = 40;				//koliko upisa
			ADC1 -> SQR1 = 0x01144084;					//kanali 2,4,5,1,3 (Upv1, Upv2, Udc, Ipv1, Ipv2)
			ADC1 -> SQR2 = 0x00000003;
			DMA1_Channel1 -> CCR |= 1;					//enable DMA
		    ADC1 -> CR |= 0x4;							//ADC start
			/* kraj pripreme merenja */

		    /* usrednjavanje */
			v_dc = (bat_buffer[0] + bat_buffer[3] + bat_buffer[6] + bat_buffer[9] + bat_buffer[12] + bat_buffer[15] + bat_buffer[18] + bat_buffer[21])>>3;
			asm("ldr r0, =v_dc;"
					"ldr r1, [r0];"
					"ldr r2, =v_dc_coef;"
					"ldr r3, [r2];"
					"vmov s2, r1;"
					"vmov s4, r3;"
					"vmul.f32 s2, s2, s4;"
					"vmov r1, s2;"
					"str r1, [r0]");

			//FALI USREDNJAVANJE NAPONA I STRUJE BATERIJE

			/* kraj usrednjavanja */

			/* regulacija */
////			e_bat = 6.0 - v_dc;
//			asm("ldr r0, =e_bat;"
//					"vmov.f32 s2, #6.0;"
//					"ldr r2, =v_dc;"
//					"ldr r3, [r2];"
//					"vmov s4, r3;"
//					"vsub.f32 s2, s2, s4;"
//					"vmov r1, s2;"
//					"str r1, [r0]");
//
////			d_bat = dold_bat + 0.002 * e_bat - 0.001 * eold_bat;
//			asm("ldr r0, =d_bat;"
//					"ldr r2, =dold_bat;"
//					"ldr r4, =e_bat;"
//					"ldr r6, =reg1_bat;"
//					"ldr r1, [r2];"
//					"ldr r3, [r4];"
//					"ldr r5, [r6];"
//					"vmov s1, r1;"
//					"vmov s3, r3;"
//					"vmov s5, r5;"
//					"vmul.f32 s3, s3, s5;"
//					"vadd.f32 s1, s1, s3;"
//					"ldr r4, =eold_bat;"
//					"ldr r6, =reg2_bat;"
//					"ldr r3, [r4];"
//					"ldr r5, [r6];"
//					"vmov s3, r3;"
//					"vmov s5, r5;"
//					"vmul.f32 s3, s3, s5;"
//					"vsub.f32 s1, s1, s3;"
//					"vmov r1, s1;"
//					"str r1, [r0]");
//
//			if (d_bat > 0.9) d_bat = 0.9;
//			else if (d_bat < 0.1) d_bat = 0.1;
//
//			//ovo je brze ovako nego u asm
//			zaokruzi = (volatile int) (d_bat * 200);
//			if (d_bat * 200 - zaokruzi > 0.5f) zaokruzi++;
//			TIM8 -> CCR1 = zaokruzi;						//ovo je D za boost, ne treba ovako, nego treba ovde da se upisuje u CCR2
//
//			eold_bat = e_bat;
//			dold_bat = d_bat;

			/* kraj regulacije */

			break;


		/* pv2 */
		case 3:
		case 7:
			/* priprema sledeceg merenja */
			if (dma_cnt == 3) {
				DMA1_Channel1 -> CMAR = (int) out5v_buffer;		//postavi buffer za upis
				ADC1 -> SQR1 = 0x10410403;						//kanal 16,16,16,16 (U5v)
			}
			else {
				DMA1_Channel1 -> CMAR = (int) out3v3_buffer;	//postavi buffer za upis
				ADC1 -> SQR1 = 0x0B2CB2C3;						//kanal 11,11,11,11 (U3v3)
			}
			DMA1_Channel1 -> CNDTR = 16;				//koliko upisa
			DMA1_Channel1 -> CCR |= 1;					//enable DMA
		    ADC1 -> CR |= 0x4;							//ADC start
			/* kraj pripreme merenja */

			/* usrednjavanje */
		    v_dc = (pv2_buffer[2] + pv2_buffer[7] + pv2_buffer[12] + pv2_buffer[17] + pv2_buffer[22] + pv2_buffer[27] + pv2_buffer[32] + pv2_buffer[37])>>3;
			asm("ldr r0, =v_dc;"
					"ldr r1, [r0];"
					"ldr r2, =v_dc_coef;"
					"ldr r3, [r2];"
					"vmov s2, r1;"
					"vmov s4, r3;"
					"vmul.f32 s2, s2, s4;"
					"vmov r1, s2;"
					"str r1, [r0]");
			v_pv2 = (pv2_buffer[1] + pv2_buffer[6] + pv2_buffer[11] + pv2_buffer[16] + pv2_buffer[21] + pv2_buffer[26] + pv2_buffer[31] + pv2_buffer[36])>>3;
			asm("ldr r0, =v_pv2;"
					"ldr r1, [r0];"
					"ldr r2, =v_pv_coef;"
					"ldr r3, [r2];"
					"vmov s2, r1;"
					"vmov s4, r3;"
					"vmul.f32 s2, s2, s4;"
					"vmov r1, s2;"
					"str r1, [r0]");
			v_pv1 = (pv2_buffer[0] + pv2_buffer[5] + pv2_buffer[10] + pv2_buffer[15] + pv2_buffer[20] + pv2_buffer[25] + pv2_buffer[30] + pv2_buffer[35])>>3;
			asm("ldr r0, =v_pv1;"
					"ldr r1, [r0];"
					"ldr r2, =v_pv_coef;"
					"ldr r3, [r2];"
					"vmov s2, r1;"
					"vmov s4, r3;"
					"vmul.f32 s2, s2, s4;"
					"vmov r1, s2;"
					"str r1, [r0]");
			i_pv2 = (pv2_buffer[4] + pv2_buffer[9] + pv2_buffer[14] + pv2_buffer[19] + pv2_buffer[24] + pv2_buffer[29] + pv2_buffer[34] + pv2_buffer[39])>>3;
			asm("ldr r0, =i_pv2;"
					"ldr r1, [r0];"
					"ldr r2, =i_pv_coef;"
					"ldr r3, [r2];"
					"vmov s2, r1;"
					"vmov s4, r3;"
					"vmul.f32 s2, s2, s4;"
					"vmov r1, s2;"
					"str r1, [r0]");
			i_pv1 = (pv2_buffer[3] + pv2_buffer[8] + pv2_buffer[13] + pv2_buffer[18] + pv2_buffer[23] + pv2_buffer[28] + pv2_buffer[33] + pv2_buffer[38])>>3;
			asm("ldr r0, =i_pv1;"
					"ldr r1, [r0];"
					"ldr r2, =i_pv_coef;"
					"ldr r3, [r2];"
					"vmov s2, r1;"
					"vmov s4, r3;"
					"vmul.f32 s2, s2, s4;"
					"vmov r1, s2;"
					"str r1, [r0]");
			/* kraj usrednjavanja */

			/* regulacija */

			/* PV POMERANJE */

//			if(i2==0){
//			//brze ovako nego asm
//			e_pv2 = 6.0 - v_dc;
//
//			//step = 0.00005 * e_pv2;
//			asm("ldr r0, =step;"
//					"ldr r2, =reg_pv;"
//					"ldr r4, =e_pv2;"
//					"ldr r3, [r2];"
//					"ldr r5, [r4];"
//					"vmov s4, r3;"
//					"vmov s2, r5;"
//					"vmul.f32 s2, s2, s4;"
//					"vmov r1, s2;"
//					"str r1, [r0]");
//
////			p_pv2 = i_pv2 * v_pv2;
//			asm("ldr r0, =p_pv2;"
//					"ldr r2, =i_pv2;"
//					"ldr r4, =v_pv2;"
//					"ldr r3, [r2];"
//					"ldr r5, [r4];"
//					"vmov s4, r3;"
//					"vmov s2, r5;"
//					"vmul.f32 s2, s2, s4;"
//					"vmov r1, s2;"
//					"str r1, [r0]");
//
//			if (p_pv2 > pold_pv2) {
//				if (v_pv2 > vold_pv2) {
//					d_pv2 = dold_pv2 - step;
//				}
//				else {
//					d_pv2 = dold_pv2 + 0.01;
//				}
//			}
//			else {
//				if (v_pv2 < vold_pv2) {
//					d_pv2 = dold_pv2 - step;
//				}
//				else {
//					d_pv2 = dold_pv2 + 0.01;
//				}
//			}
//
//			if (d_pv2 > 0.9) d_pv2 = 0.9;
//			else if (d_pv2 < 0.2) d_pv2 = 0.2;
//
//			//ovo je brze ovako nego u asm
//			zaokruzi = (volatile int) (d_pv2 * 200);
//			if (d_pv2 * 200 - zaokruzi > 0.5f) zaokruzi++;
//			TIM1 -> CCR2 = 200 - zaokruzi;
//
//			dold_pv2 = d_pv2;
//			pold_pv2 = p_pv2;
//			vold_pv2 = v_pv2;
//			i2=0;
//			}else{
//				i2++;
//			}

			/* MPPT */

////			p_pv2 = i_pv2 * v_pv2;
//			asm("ldr r0, =p_pv2;"
//					"ldr r2, =i_pv2;"
//					"ldr r4, =v_pv2;"
//					"ldr r3, [r2];"
//					"ldr r5, [r4];"
//					"vmov s4, r3;"
//					"vmov s2, r5;"
//					"vmul.f32 s2, s2, s4;"
//					"vmov r1, s2;"
//					"str r1, [r0]");
//
//			if (i_pv2 < 0.001 && v_pv2 > 0) {
//				//d_pv2 = dold_pv2 + 100 * step_mppt;
//				d_pv2 = dold_pv2 + 0.01;
//			}
//			else {
//				if (p_pv2 > pold_pv2) {
//					if (v_pv2 > vold_pv2) d_pv2 = dold_pv2 - step_mppt;
//					else d_pv2 = dold_pv2 + step_mppt;
//				}
//				else {
//					if (v_pv2 < vold_pv2) d_pv2 = dold_pv2 - step_mppt;
//					else d_pv2 = dold_pv2 + step_mppt;
//				}
//			}
//
//			//ogranicavanje djutija
//			if (d_pv2 > 0.9) d_pv2 = 0.9;
//			else if (d_pv2 < 0.1) d_pv2 = 0.1;
//
//			//ovo je brze ovako nego u asm
//			zaokruzi = (volatile int) (d_pv2 * 200);
//			if (d_pv2 * 200 - zaokruzi > 0.5f) zaokruzi++;
//			TIM1 -> CCR2 = 200 - zaokruzi;
//
//			dold_pv2 = d_pv2;
//			pold_pv2 = p_pv2;
//			vold_pv2 = v_pv2;

			/* kraj regulacije */

			break;


		/* out 5V */
		case 4:
			/* priprema sledeceg merenja */
			DMA1_Channel1 -> CMAR = (int) pv1_buffer;	//postavi buffer za upis
			DMA1_Channel1 -> CNDTR = 40;				//koliko upisa
			ADC1 -> SQR1 = 0x03142104;					//kanali 4,2,5,3,1 (Upv2, Upv1, Udc, Ipv2, Ipv1)
			ADC1 -> SQR2 = 0x00000001;
			DMA1_Channel1 -> CCR |= 1;					//enable DMA
		    ADC1 -> CR |= 0x4;							//ADC start
			/* kraj pripreme merenja */

		    /* usrednjavanje */
			v_5v = (out5v_buffer[8] + out5v_buffer[9] + out5v_buffer[10] + out5v_buffer[11] + out5v_buffer[12] + out5v_buffer[13] + out5v_buffer[14] + out5v_buffer[15])>>3;
			asm("ldr r0, =v_5v;"
					"ldr r1, [r0];"
					"ldr r2, =v_5v_coef;"
					"ldr r3, [r2];"
					"vmov s2, r1;"
					"vmov s4, r3;"
					"vmul.f32 s2, s2, s4;"
					"vmov r1, s2;"
					"str r1, [r0]");
			/* kraj usrednjavanja */

			/* regulacija */
//			e_5v = 5.0 - v_5v;
			asm("ldr r0, =e_5v;"
					"vmov.f32 s2, #5.0;"
					"ldr r2, =v_5v;"
					"ldr r3, [r2];"
					"vmov s4, r3;"
					"vsub.f32 s2, s2, s4;"
					"vmov r1, s2;"
					"str r1, [r0]");

			if (e_5v < 0.03 && e_5v > -0.03) e_5v = 0;

//			d_5v = dold_5v + 0.002 * e_5v - 0.001 * eold_5v;
			asm("ldr r0, =d_5v;"
					"ldr r2, =dold_5v;"
					"ldr r4, =e_5v;"
					"ldr r6, =reg1_5v;"
					"ldr r1, [r2];"
					"ldr r3, [r4];"
					"ldr r5, [r6];"
					"vmov s1, r1;"
					"vmov s3, r3;"
					"vmov s5, r5;"
					"vmul.f32 s3, s3, s5;"
					"vadd.f32 s1, s1, s3;"
					"ldr r4, =eold_5v;"
					"ldr r6, =reg2_5v;"
					"ldr r3, [r4];"
					"ldr r5, [r6];"
					"vmov s3, r3;"
					"vmov s5, r5;"
					"vmul.f32 s3, s3, s5;"
					"vsub.f32 s1, s1, s3;"
					"vmov r1, s1;"
					"str r1, [r0]");

			if (d_5v > 0.9) d_5v = 0.9;
			else if (d_5v < 0.1) d_5v = 0.1;

			//ovo je brze ovako nego u asm
			zaokruzi = (volatile int) (d_5v * 200);
			if (d_5v * 200 - zaokruzi >= 0.5f) zaokruzi++;
			TIM8 -> CCR3 = zaokruzi;

			eold_5v = e_5v;
			dold_5v = d_5v;
			/* kraj regulacije */

			break;


		/* out 3V3 */
		case 8:
			/* priprema sledeceg merenja */
			DMA1_Channel1 -> CMAR = (int) pv1_buffer;	//postavi buffer za upis
			DMA1_Channel1 -> CNDTR = 40;				//koliko upisa
			ADC1 -> SQR1 = 0x03142104;					//kanali 4,2,5,3,1 (Upv2, Upv1, Udc, Ipv2, Ipv1)
			ADC1 -> SQR2 = 0x00000001;
			DMA1_Channel1 -> CCR |= 1;					//enable DMA
		    ADC1 -> CR |= 0x4;							//ADC start
			/* kraj pripreme merenja */

			/* usrednjavanje */
			v_3v3 = (out3v3_buffer[8] + out3v3_buffer[9] + out3v3_buffer[10] + out3v3_buffer[11] + out3v3_buffer[12] + out3v3_buffer[13] + out3v3_buffer[14] + out3v3_buffer[15])>>3;
			asm("ldr r0, =v_3v3;"
			   		"ldr r1, [r0];"
			  		"ldr r2, =v_3v3_coef;"
			   		"ldr r3, [r2];"
			   		"vmov s2, r1;"
			   		"vmov s4, r3;"
			   		"vmul.f32 s2, s2, s4;"
			   		"vmov r1, s2;"
			   		"str r1, [r0]");
			/* kraj usrednjavanja */

			/* regulacija */
//			e_3v3 = 3.3 - v_3v3;
			asm("ldr r0, =e_3v3;"
					"ldr r4, =ref_3v3;"
					"ldr r2, =v_3v3;"
					"ldr r1, [r2];"
					"ldr r3, [r4];"
					"vmov s4, r1;"
					"vmov s2, r3;"
					"vsub.f32 s2, s2, s4;"
					"vmov r1, s2;"
					"str r1, [r0]");

			if (e_3v3 < 0.03 && e_3v3 > -0.03) e_3v3 = 0;

//			d_3v3 = dold_3v3 + 0.002 * e_3v3 - 0.001 * eold_3v3;
			asm("ldr r0, =d_3v3;"
					"ldr r2, =dold_3v3;"
					"ldr r4, =e_3v3;"
					"ldr r6, =reg1_3v3;"
					"ldr r1, [r2];"
					"ldr r3, [r4];"
					"ldr r5, [r6];"
					"vmov s1, r1;"
					"vmov s3, r3;"
					"vmov s5, r5;"
					"vmul.f32 s3, s3, s5;"
					"vadd.f32 s1, s1, s3;"
					"ldr r4, =eold_3v3;"
					"ldr r6, =reg2_3v3;"
					"ldr r3, [r4];"
					"ldr r5, [r6];"
					"vmov s3, r3;"
					"vmov s5, r5;"
					"vmul.f32 s3, s3, s5;"
					"vsub.f32 s1, s1, s3;"
					"vmov r1, s1;"
					"str r1, [r0]");

			if (d_3v3 > 0.9) d_3v3 = 0.9;
			else if (d_3v3 < 0.1) d_3v3 = 0.1;

			//ovo je brze ovako nego u asm
			zaokruzi = (volatile int) (d_3v3 * 200);
			if (d_3v3 * 200 - zaokruzi >= 0.5f) zaokruzi++;
			TIM1 -> CCR3 = zaokruzi;

			eold_3v3 = e_3v3;
			dold_3v3 = d_3v3;
			/* kraj regulacije */

			dma_cnt = 0;
			break;
		}
		/* KRAJ KORISNOG DELA KODA */

	}
	else {

		if (DMA1 -> ISR & 4) {						//half transfer se ignorise
			DMA1 -> IFCR = 4;

			while (DMA1 -> ISR & 4);
		}
		else {										//transfer error upali DMA opet
			DMA1 -> IFCR = 8;

			DMA1_Channel1 -> CCR |= 1;				//enable
		}
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

  //postavljanje brzine kanala
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_2CYCLES_5);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_2, LL_ADC_SAMPLINGTIME_2CYCLES_5);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_3, LL_ADC_SAMPLINGTIME_2CYCLES_5);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_4, LL_ADC_SAMPLINGTIME_2CYCLES_5);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_5, LL_ADC_SAMPLINGTIME_12CYCLES_5);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_7, LL_ADC_SAMPLINGTIME_24CYCLES_5);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_8, LL_ADC_SAMPLINGTIME_47CYCLES_5);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_11, LL_ADC_SAMPLINGTIME_47CYCLES_5);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_16, LL_ADC_SAMPLINGTIME_47CYCLES_5);

  //startovanje
  HAL_ADC_Start_DMA(&hadc1, pv1_buffer, 40);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 37;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO2;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED2;
  htim1.Init.Period = 200;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 40;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 160;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 110;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 12;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED2;
  htim8.Init.Period = 200;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 150;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 250;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 167;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 12;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMAMUX1_OVR_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMAMUX1_OVR_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMAMUX1_OVR_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5 
                           PE6 PE7 PE14 PE15 
                           PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC13 PC14 PC15 PC4 
                           PC5 PC9 PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PF0 PF1 PF2 PF3 
                           PF4 PF5 PF6 PF7 
                           PF8 PF9 PF10 PF11 
                           PF12 PF13 PF14 PF15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PH0 PH1 PH3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PA7 PA8 PA9 PA10 
                           PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB10 PB12 PB13 
                           PB4 PB5 PB6 PB7 
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PG0 PG1 PG2 PG3 
                           PG4 PG5 PG6 PG7 
                           PG8 PG9 PG10 PG11 
                           PG12 PG13 PG14 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11 
                           PD12 PD13 PD14 PD15 
                           PD0 PD1 PD2 PD3 
                           PD4 PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
