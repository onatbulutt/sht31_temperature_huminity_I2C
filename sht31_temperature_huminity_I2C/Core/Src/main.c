/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
const uint16_t DisplayIndex_Table[16] =
{
	0x04,
	0x02,
	0x01,
	0x08,
	0x20,
	0x40,
	0x80,
	0x10,
};

const uint8_t BitTest_Table[8] =
{
	0x80,
	0x40,
	0x20,
	0x10,
	0x08,
	0x04,
	0x02,
	0x01,
};

const uint8_t Segment_Table[127] =
{
0x00,		//		00						00000000
0x00,		//		01						00000000
0x00,		//		02						00000000
0x00,		//		03						00000000
0x00,		//		04						00000000
0x00,		//		05						00000000
0x00,		//		06						00000000
0x00,		//		07						00000000
0x00,		//		08						00000000
0x00,		//		09						00000000
0x00,		//		10						00000000
0x00,		//		11						00000000
0x00,		//		12						00000000
0x00,		//		13						00000000
0x00,		//		14						00000000
0x00,		//		15						00000000
0x00,		//		16						00000000
0x00,		//		17						00000000
0x00,		//		18						00000000
0x00,		//		19						00000000
0x00,		//		20						00000000
0x00,		//		21						00000000
0x00,		//		22						00000000
0x00,		//		23						00000000
0x00,		//		24						00000000
0x00,		//		25						00000000
0x00,		//		26						00000000
0x00,		//		27						00000000
0x00,		//		28						00000000
0x00,		//		29						00000000
0x00,		//		30						00000000
0x00,		//		31						00000000
0x00,		//		32			SPACE		00000000	OK
0x00,		//		33			!			00000000
0x00,		//		34			"			00000000
0x00,		//		35			#			00000000
0x00,		//		36			$			00000000
0x00,		//		37			%			00000000
0x00,		//		38			&			00000000
0x00,		//		39			'			00000000
0x00,		//		40			(			00000000
0x00,		//		41			)			00000000
0x00,		//		42			*			00000000
0x00,		//		43			+			00000000
0x00,		//		44			,			00000000
0x02,		//		45			-			00000010	OK
0x00,		//		46			.			00000000
0x4A,		//		47			/			01001010
0xFC,		//		48			0			11111100	OK
0x60,		//		49			1			01100000	OK
0xDA,		//		50			2			11011010	OK
0xF2,		//		51			3			11110010	OK
0x66,		//		52			4			01100110	OK
0xB6,		//		53			5			10110110	OK
0xBE,		//		54			6			10111110	OK
0xE0,		//		55			7			11100000	OK
0xFE,		//		56			8			11111110	OK
0xF6,		//		57			9			11110110	OK
0x00,		//		58			:			00000000
0x00,		//		59			;			00000000
0x00,		//		60			<			00000000
0x00,		//		61			=			00000000
0x00,		//		62			>			00000000
0x00,		//		63			?			00000000
0x00,		//		64			@			00000000
0xEE,		//		65			A			11101110	OK
0x00,		//		66			B			00000000
0x9C,		//		67			C			10011100	OK
0x00,		//		68			D			00000000
0x9E,		//		69			E			10011110	OK
0x8E,		//		70			F			10001110	OK
0x00,		//		71			G			00000000
0x6E,		//		72			H			01101110	OK
0x60,		//		73			I			01100000	OK
0x70,		//		74			J			01110000
0x00,		//		75			K			00000000
0x1C,		//		76			L			00011100	OK
0x00,		//		77			M			00000000
0x00,		//		78			N			00000000
0xFC,		//		79			O			11111100	OK
0xCE,		//		80			P			11001110	OK
0x00,		//		81			Q			00000000
0x00,		//		82			R			00000000
0xB6,		//		83			S			10110110	OK
0x00,		//		84			T			00000000
0x7C,		//		85			U			01111100	OK
0x00,		//		86			V			00000000
0x00,		//		87			W			00000000
0x00,		//		88			X			00000000
0x00,		//		89			Y			00000000
0x00,		//		90			Z			00000000
0x00,		//		91			[			00000000
0x00,		//		92			\			00000000
0x00,		//		93			]			00000000
0x00,		//		94			^			00000000
0x00,		//		95			_			00000000
0x00,		//		96			`			00000000
0x00,		//		97			a			00000000
0x3E,		//		98			b			00111110	OK
0x1A,		//		99			c			00011010	OK
0x7A,		//		100			d			01111010	OK
0x00,		//		101			e			00000000
0x00,		//		102			f			00000000
0xF6,		//		103			g			11110110	OK
0x2E,		//		104			h			00101110	OK
0x20,		//		105			i			00100000	OK
0x70,		//		106			j			01110000
0x1E,		//		107			k			00011110	OK
0x00,		//		108			l			00000000
0x00,		//		109			m			00000000
0x2A,		//		110			n			00101010	OK
0x3A,		//		111			o			00111010	OK
0x00,		//		112			p			00000000
0x00,		//		113			q			00000000
0x0A,		//		114			r			00001010	OK
0x00,		//		115			s			00000000
0x1E,		//		116			t			00011110	OK
0x38,		//		117			u			00111000	OK
0x00,		//		118			v			00000000
0x00,		//		119			w			00000000
0x00,		//		120			x			00000000
0x76,		//		121			y			01110110	OK
0x00,		//		122			z			00000000
0x00,		//		123			{			00000000
0x00,		//		124			|			00000000
0x00,		//		125			}			00000000
0x00		//		126			~			00000000
};
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
__IO uint8_t	  DisplayIndex=0;
__IO uint8_t	  DisplayBuffer[20];
__IO uint16_t	  SerialData_Anot=0;
__IO uint16_t	  SerialData_Katot=0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint16_t temp_register=0x2c06; //value of address of temp register
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//HAL_StatusTypeDef
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
HAL_StatusTypeDef status;
HAL_StatusTypeDef status2;
uint8_t readFlag=0;
uint8_t sensorFlag=0;
uint8_t readyFlag=0;
uint16_t sensorTempValue;
uint16_t sensorHuminityValue;
uint8_t temperature;
uint8_t huminity;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
void SensorTest(void);
void Read(void);
void UnSigned_Numeric_Print(uint32_t Value, uint8_t Digit, uint8_t Display);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      SensorTest();
	  Read();
	  //HAL_Delay(1000);
	  UnSigned_Numeric_Print(temperature, 3, 1);
	  UnSigned_Numeric_Print(huminity, 3, 2);
	  //UnSigned_Numeric_Print(12, 3, 2);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /** I2C Fast mode Plus enable
  */
  __HAL_SYSCFG_FASTMODEPLUS_ENABLE(I2C_FASTMODEPLUS_I2C1);
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */




/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_6|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}
void ADC_Init(void)
{


}
/* USER CODE BEGIN 4 */
void SensorTest(void)
{
	if(HAL_I2C_IsDeviceReady(&hi2c1,0x45<<1,4,100)!=HAL_OK)
	{
	  readyFlag=1;
	}
	else
	{
		readyFlag=0;
	}
}

void Read(void)
{

	uint8_t txData[2]={0x2c,0x06};//sensörden sıcaklık verisi almak için gerekli adres değeri(msb,lsb) datasheetden bulundu
	uint8_t rxData[6] = {0}; //sensörden gelecek olan veri 6 bayt lık bir değer.
	// sırasıyla: temperature msb,temperature lsb, checksum, huminity msb,huminity lsb
	status= HAL_I2C_Master_Transmit(&hi2c1, (0x44<<1)|0x00, txData, 2, HAL_MAX_DELAY);//sensöre sıcaklık ve nem değerleri istediğimi iletiyorum.
	if(status!=HAL_OK) //eğer gönderdiğim komut gitmemişse
	{
		readFlag=0;
	}
	else //eğer gönderdiğim komut gitmişse
	{
		readFlag=1;
		status2= HAL_I2C_Master_Receive(&hi2c1, (0x44<<1)|0x01, rxData, 6, HAL_MAX_DELAY);//sensör bana sıcaklık ve nem değerleri alıyorum
		if(status2!=HAL_OK)
			{
				sensorFlag=0;
			}
			else
			{
				sensorFlag=1;
				sensorTempValue= (rxData[0]<<8)| rxData[1]; //temperature verisi sensörden 16 bit olarak geliyor. sensörün özelliği
				//sensörün msb bytını 8 bit sola kaydırıp lsb bytını yanına ekledim. böylece msb-lsb sıralamasını yapmış oldum.
				sensorHuminityValue= (rxData[3]<<8)| rxData[4];
				huminity = (100.0 * sensorTempValue / 65535.0);//sıcaklık hesaplarken yapılan şeyler burda da geçerlidir.
				temperature = (175.0 * sensorTempValue / 65535.0) - 45.0;
			}


	}
}

void DisplayIndex_Control(void)// 7 adet katotumuz var....
{
  // Display Taramasi 0..7 Olarak Yapiliyor.
  DisplayIndex++;
  if (DisplayIndex >=8) DisplayIndex=0;//DisplayIndex 8 mi ? 8 ise sifirla.
}
//-----------------------------------------------------------------------------------------------------------------//
//       Display Drive
//-----------------------------------------------------------------------------------------------------------------//
//DISPLAY TARAMADA DISPLAYLERE GIDEN BILGININ ISLENMESI RUTINI
void Display_Drive(void)
{
  uint8_t 	i;    // 8 bitlik unsigned 'i'  degiskeni tanimlanmis
  uint16_t 	Temp; // 8 bitlik bir Temp Registeri Tanimlaniyor.Temp deiskeni Herbir Segmenti Temsil ediyor.

  SerialData_Anot  = Segment_Table[DisplayBuffer[DisplayIndex]];
  SerialData_Katot = DisplayIndex_Table[DisplayIndex];


  // SEGMENT KONTROL BOLUMU
  DSP_STB_LO;  // Strobe Baslangiçta LOW'lanmistir
  DSP_SI_LO;   // Data Baslangiçta LOW'lanmistir
  DSP_CLK_LO;  // Clock Baslangiçta LOW'lanmistir
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();

  //--------------------------------------------------------------------------------
  //--------------------------------------------------------------------------------
  // KATOT BILGISINI GONDERIYORUZ................................................
  for (i=0;i<8;i++)// KATOT YUKLENIYOR
  {
	if((SerialData_Katot&BitTest_Table[i])!=0x00) DSP_SI_HI;
	if((SerialData_Katot&BitTest_Table[i])==0x00) DSP_SI_LO;
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
	DSP_CLK_HI;
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
	DSP_CLK_LO;
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
  }


  //--------------------------------------------------------------------------------
  //--------------------------------------------------------------------------------
  // ANOT1 BILGISINI GONDERIYORUZ..................................................
  Temp = 0;

  if((SerialData_Anot&0x80)==0x80) Temp=Temp|0x01;//A			// Tablodaki Segment Siramiz ABCDEFGP Seklinde Giderken,
  if((SerialData_Anot&0x40)==0x40) Temp=Temp|0x02;//B			// Hardawaredeki Segment Siramiz XXXXXXXX Seklinde Oldugundan,
  if((SerialData_Anot&0x20)==0x20) Temp=Temp|0x04;//C			// Bitlerin Yerini Degistirdik.....
  if((SerialData_Anot&0x10)==0x10) Temp=Temp|0x08;//D
  if((SerialData_Anot&0x08)==0x08) Temp=Temp|0x10;//E
  if((SerialData_Anot&0x04)==0x04) Temp=Temp|0x20;//F
  if((SerialData_Anot&0x02)==0x02) Temp=Temp|0x40;//G
  if((SerialData_Anot&0x01)==0x01) Temp=Temp|0x80;//DP

  SerialData_Anot=Temp;

  for (i=0;i<8;i++)//
  {
	 if((SerialData_Anot&BitTest_Table[i])!=0x00) DSP_SI_HI;
	 if((SerialData_Anot&BitTest_Table[i])==0x00) DSP_SI_LO;
	 __NOP();
	 __NOP();
	 __NOP();
	 __NOP();
	 __NOP();
	 __NOP();
	 	__NOP();
	 	__NOP();
	 	__NOP();
	 	__NOP();
	 DSP_CLK_HI;
	 __NOP();
	 __NOP();
	 __NOP();
	 __NOP();
	 __NOP();
	 __NOP();
	 	__NOP();
	 	__NOP();
	 	__NOP();
	 	__NOP();
	 DSP_CLK_LO;
	 __NOP();
	 __NOP();
	 __NOP();
	 __NOP();
	 __NOP();
	 __NOP();
	 	__NOP();
	 	__NOP();
	 	__NOP();
	 	__NOP();
  }
//--------------------
//--------------------
  DSP_STB_HI;
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  	__NOP();
  	__NOP();
  	__NOP();
  	__NOP();
  DSP_STB_LO;
}
//-----------------------------------------------------------------------------------------------------------------//
//
//-----------------------------------------------------------------------------------------------------------------//
//POZITIF BIR DEGER ÜST DISPLAY'E BASTIRILACAKSA  BU RUTINI KULLANIYORUZ. ÖRN:UnSigned_Numeric_Print_ÜST(XX,4)
void UnSigned_Numeric_Print(uint32_t Value, uint8_t Digit, uint8_t Display)
{
  uint16_t		Buffer[5]; 				          // 8 bitlik 4 lü bir dizi tanimlaniyor

  Buffer[0]=0x30+(Value%10);					  // Birler Hanesi     // Buffer[x] dizisi Haneleri Ayarlaniyor
  Buffer[1]=0x30+(Value/10)%10;					  // Onlar Hanesi      // Yazilan Value Degeri 10 Bölünüp kalani yaziliyor
  Buffer[2]=0x30+(Value/100)%10;				  // Yüzler Hanesi     //mesela yüzler hanesine 200 yazilacak Sayi 100 e bölünür ve sonucu direk yazar ve

  if(Display == 0x01)
  {
  	if(Digit >= 1) DisplayBuffer[2]=Buffer[0];
  	else DisplayBuffer[2]=' ';
  	if(Digit >= 2) DisplayBuffer[1]=Buffer[1];
  	else DisplayBuffer[1]=' ';
  	if(Digit >= 3)DisplayBuffer[0] =Buffer[2];
  	else DisplayBuffer[0]=' ';
  }

  if(Display == 0x02)
  {
  	if(Digit >= 1) DisplayBuffer[5]=Buffer[0];
  	else DisplayBuffer[5]=' ';
  	if(Digit >= 2) DisplayBuffer[4]=Buffer[1];
  	else DisplayBuffer[4]=' ';
  	if(Digit >= 3)DisplayBuffer[3] =Buffer[2];
  	else DisplayBuffer[3]=' ';
  }
}
//-----------------------------------------------------------------------------------------------------------------//
//      Key Input
//-----------------------------------------------------------------------------------------------------------------//
// BUTONLARIMIZ _ BUTONLARIMIZI DISPLAY KATOTLARINA BAGLADIK......
/*
void SysTick_Key_Input(void)
{
  __NOP(); // __NOP(lari) koymaminizin nedeni butonlarin bagli oldugu transistörün yavas olmasi,
  __NOP(); // ayni anda katot seçiliyor ve hangi katotun seçildigini okumamiz için biraz bekletmemiz gerekiyor.
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();

  if(GPIO_ReadInputDataBit(GPIOB, BUTTON_1_PIN) == Bit_SET)
  {
  	KeyTime[DisplayIndex]++;
  	if(KeyTime[DisplayIndex] >=7)												//Döngu=8 milisaniye ise 14x4=56 milisaniye Buton Filtresi..... burada dikkat edilmesi gerekn bisey var.display                             //taramasi sistemdeki toplam display sayisi süresince bi tur bitiriyor. Yani mesela burada 8 adet display taradik ve timer interruptimiz 1 msn'ye kurulu oldugu için her 1 msn de 1 sadece 1 tanesi tarayip digerine geçiyor. Burada ben toplam 7 defa  her bir displayi tarattirmis oldum yani toplamda 7x8=56 msn süre geçmis oldu ve bu süreyi Buton filtresi olarak kullanmis oldum.
  	{
  		KeyTime[DisplayIndex]=7;
  		KeyFlag = KeyFlag | KeyFlag_Table[DisplayIndex]; // keyflag0'i KeyFlag0_Table'daki Siralamayaa göre OR'laniyor ve hangi butona basildiysa o butonun degeri Keyflag0'a tanmis oluyor.
  	}
  }
  else
  {
  	KeyTime[DisplayIndex]=0;
  	KeyFlag = KeyFlag & ~KeyFlag_Table[DisplayIndex]; //
  }
}
*/
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
