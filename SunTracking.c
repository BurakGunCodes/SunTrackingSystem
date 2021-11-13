

#define PI          3.14159265
#define HALF_PI     1.57079
#define TWO_PI      6.283185
#define DEG_TO_RAD  0.01745329
#define RAD_TO_DEG  57.2957786

#define HM5983_Write_Address 0x3C
#define HM5983_Read_Address  0x3D


#define Register_A_Address      0x00
#define Register_B_Address      0x01
#define Register_Mode_Address   0x02






#include "main.h"
#include "stm32f4xx_hal.h"
#include "fatfs.h"
#include "usb_host.h"
#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "mpu6050_map.h"
#include "stdbool.h"


ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c2_rx;
DMA_HandleTypeDef hdma_i2c3_rx;

UART_HandleTypeDef huart4;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART4_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_ADC3_Init(void);
void MX_USB_HOST_Process(void);

void DS1307_initially();
void MPU6050_initially();
void HM5983_initial();

void egim();
long map(long x, long in_min, long in_max, long out_min, long out_max);
void MPU6050_Readings();
void RTC_Read();

void motor_sag(void);
void motor_sol(void);
void motor_ust(void);
void motor_alt(void);

void LDR_Kalibrasyon();
void Oranlar();
void yon();


uint8_t BCD2DEC(uint8_t data);
uint8_t DEC2BCD(uint8_t data);

int  sizeofbuffer(char  *x);

extern ApplicationTypeDef Appli_state;

FIL   fp;     //file handle
FATFS fatfs;  //structure with file system information



uint32_t ret;//return variable

FRESULT file_return;
int a = 1;
int size;

uint8_t saat,dakika,saniye;
uint8_t gun,ay,yil,tarih;
uint8_t received_data[7];
uint8_t send_data;

char uart_buffer[150]="";
char *AY = "";
char *GUN="";
char  uyari_mesaji[30]="" ;
char motorr[10]="";

uint8_t TxBuffer[2];
uint8_t RxBuffer[7];
uint8_t DataBuffer[14];
int16_t DataBuffer16[7];
uint8_t MPU6050_INT_State = 0;



int16_t gyro_x_temp, gyro_y_temp, gyro_z_temp, accel_x_temp, accel_y_temp, accel_z_temp, temp_raw;
float temp, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z;

float deneme,deneme1;

int xAng ;
int yAng ;
int zAng ;



int minVal=265; 
int maxVal=402;

int x,y,z;
int size;
char data;

int xAng ;
int yAng ;
int zAng ;
int int_temp;


uint8_t Register_A_Data;
uint8_t Register_B_Data;
uint8_t Register_Mode_Data;

int tempH;
int tempL;
int pusula_temp;

uint8_t  ptemp[2];
uint8_t  pusula_data[6];


int   pusula_x;
int   pusula_y;
int   pusula_z;
float pusula_yon;

int	Sol_LDR ;
int	Sag_LDR;
int	Ust_LDR ;
int	 Alt_LDR;
int  Bos_LDR;

uint8_t  adc_buffer[5] ;

float panel_gerilimi;
int   usb_panel_gerilimi;


float	 Sol_Ort ;
float	 Sag_Ort;
float	 Ust_Ort ;
float	 Alt_Ort;

int ust_alt_fark;
int sag_sol_fark;

bool  kalibre = 0;
bool uart_receive_data = 0 ;

uint8_t  uart_receive_buffer[15] ;

int fark_deneme   = 0 ;
int fark_deneme_2 = 0 ;





int sayac = 0 ;
int sol_dizi[1000];
int sag_dizi[1000];
int alt_dizi[1000];
int ust_dizi[1000];

float  sol_oran[1000];
float  sag_oran[1000];
float  alt_oran[1000];
float  ust_oran[1000];

uint8_t sag_limit,sol_limit;


int Sol_LDR_Min[450]  ; //pf5
int Sag_LDR_Min[450] ; //pf3
int Ust_LDR_Min[450]; //pc1
int Alt_LDR_Min[450] ; //pf4
int min;
int t;


int main(void)//Pg2 pg9 input olarak aktif edildi
{



  HAL_Init();

  HAL_GPIO_EXTI_Callback(GPIO_PIN_11);

  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_UART4_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_ADC3_Init();
  MX_USB_HOST_Init();
  MX_FATFS_Init();

  MPU6050_initially(); // e�im sens�r�

  DS1307_initially(); //saat modulu


  //HM5983_initial();  //sensor d�zg�n �al��mad�


   file_return = f_mount( &fatfs,"" ,0);

  f_open(&fp, "tez.txt",	FA_OPEN_APPEND | FA_WRITE);
  f_close(&fp);

  HAL_ADC_Start_DMA(&hadc3,(uint32_t*)&adc_buffer,5);
  HAL_ADC_Start_IT(&hadc3);

/*
for(int t=0;t<=450;t++){


    HAL_ADC_Start_IT(&hadc3);

	Sol_LDR_Min[t] = adc_buffer[4] ; //pf5
	Sag_LDR_Min[t] = adc_buffer[1] ; //pf3
	Ust_LDR_Min[t] = adc_buffer[2]; //pc1
	Alt_LDR_Min[t]= adc_buffer[3] ; //pf4

	motor_ust();
}

min = Ust_LDR_Min[0];
for(int t=1;t<=450;t++){

	if (Ust_LDR_Min[0]  <= Ust_LDR_Min[t] ){

		min = Ust_LDR_Min[0];
}else{
	min = Ust_LDR_Min[t];
}

}
*/
for(int t =0;t<=320;t++){
	motor_ust();
}

//LDR_Kalibrasyon();

    while(1)
    {
        HAL_ADC_Start_IT(&hadc3);
        sag_limit = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_9);
        sol_limit = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_2);

        sayac++;


        while (Appli_state ==APPLICATION_DISCONNECT)
        {

        sprintf(uyari_mesaji ,"[%02d %s 20%02d  %02d:%02d:%02d]  !!!!  USB kayit aygiti baglantisi koptu  !!!!\n\n",tarih,AY,yil,saat,dakika,saniye);
    		HAL_UART_Transmit_IT(&huart4, (uint8_t*)uyari_mesaji,sizeofbuffer(uyari_mesaji));
    		HAL_Delay(75);

        }

        while (Appli_state ==APPLICATION_START)
        {

        sprintf(uyari_mesaji ,"[%02d %s 20%02d  %02d:%02d:%02d]  !!!!  USB kayit aygiti baglantisi yapildi  !!!!\n\n",tarih,AY,yil,saat,dakika,saniye);
    		HAL_UART_Transmit_IT(&huart4, (uint8_t*)uyari_mesaji,sizeofbuffer(uyari_mesaji));
    		HAL_Delay(75);

        }

        MX_USB_HOST_Process();





        HAL_I2C_Mem_Read_DMA(&hi2c1, 0xD1, 0, I2C_MEMADD_SIZE_8BIT, received_data, 7);   //rtc modulunden bilgi al�n�o
        HAL_Delay(60);
        MPU6050_Readings();  // mpu6050 e�im modulunden bilgi

        Oranlar();
        sprintf(uart_buffer, "ust :%d alt :%d  ust_oran : %f  alt_oran :%f     sayac:%d  panel gerilimi :%f\n",Ust_LDR,Alt_LDR, ust_oran[sayac] , alt_oran[sayac],sayac,panel_gerilimi );
    	  HAL_UART_Transmit_IT(&huart4, (uint8_t*)uart_buffer,sizeofbuffer(uart_buffer));
        HAL_Delay(50);


    	 usb_panel_gerilimi = (int)panel_gerilimi;//usb float de�eri yaz�lmad� integer'a �evrildi.

    	 f_open(&fp, "tez.txt",	FA_OPEN_APPEND | FA_WRITE);
    	 f_printf(&fp, "[%02d %s 20%02d  %02d:%02d:%02d] x:%d�  y:%d�  z:%d�  temp:%d�C  panel gerilim %d V \n",tarih,AY,yil,saat,dakika,saniye,x,y,z,int_temp,usb_panel_gerilimi);
    	 f_close(&fp);




    }



}








// ldr de�erlerinin bir �nceki de�er ile aras�nda ki oranlara bak�l�yor.Parazit ���klara s�rekli olarak d�nmeyi engelliyor.
void Oranlar()
{ 

	  sol_dizi[sayac] = Sol_LDR;
    sol_oran[sayac]= abs(   sol_dizi[sayac] - sol_dizi[sayac - 1 ] ) ;
    sol_oran[sayac]  /=   sol_dizi[sayac - 1 ]  ;

    sag_dizi[sayac] = Sag_LDR;
    sag_oran[sayac]= abs(   sag_dizi[sayac] - sag_dizi[sayac - 1 ] ) ;
    sag_oran[sayac]  /=    sag_dizi[sayac - 1 ]  ;

	  ust_dizi[sayac] = Sol_LDR;
    ust_oran[sayac]= abs(   ust_dizi[sayac] - ust_dizi[sayac - 1 ] ) ;
    ust_oran[sayac]  /=   ust_dizi[sayac - 1 ]  ;

	  alt_dizi[sayac] = Sol_LDR;
    alt_oran[sayac]= abs(   alt_dizi[sayac] - alt_dizi[sayac - 1 ] ) ;
    alt_oran[sayac]  /=   alt_dizi[sayac - 1 ]  ;

    if(sayac == 999) sayac = 0;

	  if(   (Ust_LDR - Alt_LDR > 5) && ( (float)ust_oran[sayac]<0.0001)  )
    {
		  motor_alt();
	  }

	  if (  (Alt_LDR- Ust_LDR> 5)   && ( (float)alt_oran[sayac]<0.0001))
    {
		  motor_ust();
	  }


}


// ldr lerden al�nan de�erler bulundugu ortamda kalibre edilioyor ��nk� de�erleri e�itleniyor.ldr ler ayn� dirence sahip de�il
// kalibre s�ras�nda bir de�eri di�erine yak�nla�t�rma de�il iki de�eride orta nokta �ekme i�lemi yap�l�yor.
// burada fark�n yar�s� bulunuyor ve ADC okuma i�leminde kullan�l�yor.
void LDR_Kalibrasyon()
{

	ust_alt_fark  = abs(Ust_LDR - Alt_LDR);
	sag_sol_fark  = abs(Sag_LDR - Sol_LDR);

	if(Ust_LDR > Alt_LDR)
  {
		Alt_LDR += ust_alt_fark;
		fark_deneme_2 = (ust_alt_fark) / 2 ;
	}
  else
  {
		Ust_LDR += ust_alt_fark;
		fark_deneme_2 = - (ust_alt_fark / 2) ;
	}

	if(Sag_LDR > Sol_LDR)
  {
		Sol_LDR += sag_sol_fark;
		fark_deneme = (sag_sol_fark)/2 ;
	}
  else
  {
		Sag_LDR += sag_sol_fark;
		fark_deneme = -(sag_sol_fark)/2 ;
	}

}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){



	if( hadc -> Instance  == ADC3)
  {

		Sol_LDR = adc_buffer[4] + fark_deneme; //pf5
		Sag_LDR = adc_buffer[1] - fark_deneme;//pf3
		Ust_LDR = adc_buffer[2] - fark_deneme_2;//pc1
		Alt_LDR= adc_buffer[3] + fark_deneme_2;//pf4
		Bos_LDR = adc_buffer[0];//pf10

	}

	  
    panel_gerilimi = Bos_LDR  * 2.9   ; // st cihaz�n�n lojik de�eri.sistemde ki y�ke g�re de�i�iklik g�steriyor.
    panel_gerilimi /= 255 ;
	  panel_gerilimi *= 10 ;





}

void DS1307_initially()
{
  	send_data = 0x00 ; //saniye
  	HAL_I2C_Mem_Write_IT(&hi2c1, 0xD0, 0x00,I2C_MEMADD_SIZE_8BIT,(uint8_t*)& send_data,1);
  	HAL_Delay(10);

  	send_data = 0x00;//dakika
  	HAL_I2C_Mem_Write_IT(&hi2c1, 0xD0, 0x01,I2C_MEMADD_SIZE_8BIT, (uint8_t*)& send_data,1);
  	HAL_Delay(10);;

  	send_data = 0x12 ;//saat
  	HAL_I2C_Mem_Write_IT(&hi2c1, 0xD0, 0x02,I2C_MEMADD_SIZE_8BIT, (uint8_t*)& send_data,1);
  	HAL_Delay(10);

  	send_data = 0x00 ;//g�n
  	HAL_I2C_Mem_Write_IT(&hi2c1, 0xD0, 0x03,I2C_MEMADD_SIZE_8BIT, (uint8_t*)& send_data,1);
  	HAL_Delay(10);

  	send_data = 0x00 ;//tarih
  	HAL_I2C_Mem_Write_IT(&hi2c1, 0xD0, 0x04,I2C_MEMADD_SIZE_8BIT, (uint8_t*)& send_data,1);
  	HAL_Delay(10);

  	send_data = 0x00 ;//ay
  	HAL_I2C_Mem_Write_IT(&hi2c1, 0xD0, 0x05,I2C_MEMADD_SIZE_8BIT, (uint8_t*)& send_data,1);
  	HAL_Delay(10);

  	send_data = 0x18 ;//yil
  	HAL_I2C_Mem_Write_IT(&hi2c1, 0xD0, 0x06,I2C_MEMADD_SIZE_8BIT, (uint8_t*)& send_data,1);
  	HAL_Delay(10);

  	send_data = 0x10 ;//kontrol
  	HAL_I2C_Mem_Write_IT(&hi2c1, 0xD0, 0x07,I2C_MEMADD_SIZE_8BIT, (uint8_t*) &send_data,1);
  	HAL_Delay(10);
  }

void MPU6050_initially()
{
	TxBuffer[0] = 0x80;
	HAL_I2C_Mem_Write(&hi2c2, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, TxBuffer, 1, 1000);
	HAL_Delay(10);

	TxBuffer[0] = 0x00;
	HAL_I2C_Mem_Write(&hi2c2, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, TxBuffer, 1, 1000);
	HAL_Delay(10);

	TxBuffer[0] = 0xF8;
	HAL_I2C_Mem_Write(&hi2c2, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_FIFO_EN, 1, TxBuffer, 1, 1000);
	HAL_Delay(10);

	TxBuffer[0] = 0x10;
	HAL_I2C_Mem_Write(&hi2c2, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_FIFO_EN, 1, TxBuffer, 1, 1000);
	HAL_Delay(10);

	HAL_I2C_Mem_Read(&hi2c2, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, RxBuffer, 1, 1000);
	HAL_Delay(10);

	RxBuffer[0] |= 0x18;
	HAL_I2C_Mem_Write(&hi2c2, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 1, RxBuffer, 1, 1000);
	HAL_Delay(10);

	HAL_I2C_Mem_Read(&hi2c2, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_INT_ENABLE, I2C_MEMADD_SIZE_8BIT, RxBuffer, 1, 1000);
	HAL_Delay(10);
	RxBuffer[0] |= 0x11;
	HAL_I2C_Mem_Write(&hi2c2, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_INT_ENABLE, 1, RxBuffer, 1, 1000);
	HAL_Delay(10);

	HAL_I2C_Mem_Read(&hi2c2, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_INT_PIN_CFG, I2C_MEMADD_SIZE_8BIT, RxBuffer, 1, 1000);
	HAL_Delay(10);

	RxBuffer[0] |= 0x30;
	HAL_I2C_Mem_Write(&hi2c2, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_INT_PIN_CFG, 1, RxBuffer, 1, 1000);
	HAL_Delay(10);

	HAL_I2C_Mem_Read(&hi2c2, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_CONFIG, I2C_MEMADD_SIZE_8BIT, RxBuffer, 1, 1000);
	HAL_Delay(10);

	RxBuffer[0] |= 0x06;
	HAL_I2C_Mem_Write(&hi2c2, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_CONFIG, 1, RxBuffer, 1, 1000);
	HAL_Delay(10);
}

void HM5983_initial()
{
	Register_A_Data=0x70;
	HAL_I2C_Mem_Write_IT(&hi2c3, 0x3C,0x00,I2C_MEMADD_SIZE_8BIT,(uint8_t*)&Register_A_Data, 1);
	HAL_Delay(10);

	Register_B_Data=0xA0;
	HAL_I2C_Mem_Write_IT(&hi2c3, 0x3C,0x01,I2C_MEMADD_SIZE_8BIT, (uint8_t*)&Register_B_Data, 1);
	HAL_Delay(10);

  Register_Mode_Data=0x00;
	HAL_I2C_Mem_Write_IT(&hi2c3, 0x3C,0x02,I2C_MEMADD_SIZE_8BIT, (uint8_t*)&Register_Mode_Data, 1);
	HAL_Delay(10);
}




void motor_sag()
{
	for(int t = 0 ;t<= 4 ; t++)
  {
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_5);
    HAL_Delay(3);
	}

}

void motor_sol()
{
	for(int t = 0 ;t<= 4 ; t++)
  {
	  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_SET);
	  HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_5);
	  HAL_Delay(3);
	}
}

void motor_ust()
{
	for(int t = 0 ;t<= 4 ; t++)
  {
	  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET);
	  HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_6);
	  HAL_Delay(3);
}
}

void motor_alt()
{
	for(int t = 0 ;t<= 4 ; t++)
  {
	  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_SET);
	  HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_6);
	  HAL_Delay(3);
	}
}

  void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){

  	if(hi2c ->Instance ==hi2c1.Instance)
    {
      saniye = BCD2DEC(received_data[0]);
      dakika = BCD2DEC(received_data[1]);
      saat =  BCD2DEC(received_data[2]);

      gun = BCD2DEC(received_data[3]);
      tarih = BCD2DEC(received_data[4]);
      ay = BCD2DEC(received_data[5]) ;
      yil = BCD2DEC(received_data[6]);


  	switch( ay){
  	case 0:
  		AY ="Ocak" ;
  				break;
  	case 1:
  		AY = "Subat";
  		break;
  	case 2:
  		AY = "Mart";
  		break;
  	case 3:
  		AY = "Nisan";
  		break;
  	case 4:
  		AY = "Mayis";
  		break;
  	case 5:
  		AY = "Haziran";
  		break;
  	case 6:
  		AY = "Temmuz";
  		break;
  	case 7:
  		AY = "Agustos";
  		break;
  	case 8:
  		AY = "Eylul";
  		break;
  	case 9:
  		AY = "Ekim";
  		break;
  	case 10:
  		AY = "Kasim";
  		break;
  	case 11:
  		AY = "Aralik";
  		break;

  	}
  	switch( gun){
  	case 0:
  		GUN ="Pazartesi" ;
  		break;
  	case 1:
  		GUN = "Sali";
  		break;
  	case 2:
  		GUN = "Carsamba";
  		break;
  	case 3:
  		GUN = "Persembe";
  		break;
  	case 4:
  		GUN = "Cuma";
  		break;
  	case 5:
  		GUN = "Cumartesi";
  		break;
  	case 6:
  		GUN = "Pazar";
  		break;

  	}
  }
  	if(hi2c -> Instance == hi2c3.Instance){

/*     	tempH=ptemp[0];
  		tempL=ptemp[1];
  		pusula_temp=  ( (  (tempH<<8) + tempL)>>7) +25;
*/

   		pusula_x = (pusula_data[0]*256) + pusula_data[1] ;
      pusula_z = (pusula_data[2]*256) + pusula_data[3] ;
   		pusula_y = (pusula_data[4]*256) + pusula_data[5] ;

/*
   	  if (pusula_x > 0x07FF) pusula_x = 0xFFFF - pusula_x;
   	  if (pusula_z > 0x07FF) pusula_z = 0xFFFF - pusula_z;
   	  if (pusula_y > 0x07FF) pusula_y = 0xFFFF - pusula_y;

*/
   	pusula_yon= 0.0 ;

double kc = pusula_y/pusula_x;

double  yon_rad = atan(kc);

float sapma =   -  0.095;

yon_rad = yon_rad +  sapma;
if(yon_rad < 0 )
{ 
  yon_rad = yon_rad +  6.28; 
}

if(yon_rad >  6.28 )
{
   yon_rad = yon_rad -6.28; 
}

pusula_yon = yon_rad * 57.3  ;


/*
  		if(pusula_y > 0){
  			pusula_yon=  ( (90 - ( atan(kc)* 57.3))) ;
  				}
  		if(pusula_y < 0){
  			pusula_yon = 	( (270 - ( atan(kc)* 57.3 ))) ;
  		}
  		if(pusula_y==0 && x<0 ){
  			pusula_yon= 180.0;
  		}
  		if(pusula_y ==0 && x > 0 ){
  			pusula_yon= 0.0 ;
  		}

*/
  	}

  }

uint8_t BCD2DEC(uint8_t data)
{
  return (data>>4)*10 + (data & 0x0f) ;
}

uint8_t DEC2BCD(uint8_t data)
{
  return (data/10)<<4|(data%10);
}


void MPU6050_Readings()
{

	HAL_I2C_Mem_Read(&hi2c2, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, DataBuffer, 14, 10000);

	for(int i=0;i<7;i++)
	{
		DataBuffer16[i] = (int16_t)(((uint16_t)DataBuffer[2*i]<<8) | DataBuffer[2*i + 1]);
	}

  accel_x_temp = DataBuffer16[0];
  accel_x = (float)accel_x_temp / 2048.0F;
  accel_y_temp = DataBuffer16[1];
  accel_y = (float)accel_y_temp / 2048.0F;
  accel_z_temp = DataBuffer16[2];
  accel_z = (float)accel_z_temp / 2048.0F;

  temp_raw = DataBuffer16[3];
  temp = temp_raw / 340.0;
  temp += 36.53;
  int_temp = (int)temp;

  int xAng = map(accel_x_temp,minVal,maxVal,-90,90);
  int yAng = map(accel_y_temp,minVal,maxVal,-90,90);
  int zAng = map(accel_z_temp,minVal,maxVal,-90,90);




  x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
  y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
  z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);



}

void RTC_Read(){

}

int  sizeofbuffer(char  *x)
{
	 int size = 0 ;
  for(int t=0;t<=150;t++)
  {
 	  if(x[t] == '\0')
    {
 	  size = t ;
    break;
 		}
  }
 return size ;
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 10);
}

/* ADC3 init function */
static void MX_ADC3_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_8B;
  hadc3.Init.ScanConvMode = ENABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 5;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
sConfig.Channel = ADC_CHANNEL_2;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
{
  _Error_Handler(__FILE__, __LINE__);
}

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C3 init function */
static void MX_I2C3_Init(void)
{

  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* UART4 init function */
static void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PG3 PG4 PG5 PG6 
                           PG7 PG8 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);



  /*Configure GPIO pins : PA11 PA12 PA13 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
