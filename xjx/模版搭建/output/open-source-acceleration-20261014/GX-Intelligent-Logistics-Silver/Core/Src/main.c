/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
//                            _ooOoo_  
//                           o8888888o  
//                           88" . "88  
//                           (| -_- |)  
//                            O\ = /O  
//                        ____/`---'\____  
//                      .   ' \\| |// `.  
//                       / \\||| : |||// \  
//                     / _||||| -:- |||||- \  
//                       | | \\\ - /// | |  
//                     | \_| ''\---/'' | |  
//                      \ .-\__ `-` ___/-. /  
//                   ___`. .' /--.--\ `. . __  
//               ."" '< `.___\_< | >_/___.' >'"".  
//               | | : `- \`.;`\ _ /`;.`/ - ` :| |  
//                 \ \ `-. \_ __\ /__ _/ .-` / /  
//         ======`-.____`-.___\_____/___.-`____.-'======  
//                            `=---='  
//  
//         .............................................  
//                  ���汣��             ����BUG 
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bjdj.h"
#include "oled.h"
#include "Motor.h"
#include "my_uart.h"
#include "Nixie.h"
#include "servo.h"
#include "smp.h"
#include  "string.h"
#include "JiXieBi.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

int rx=0;
uint8_t RX=255,Rec;
uint8_t RX_2[32];    //���ܶ�ά��Ļ���ռ�
uint8_t smp_Rec;//������ݮ����Ϣ
uint8_t RX_dj;//���ܶ����������Ϣ
uint8_t RX_pjdj[4];//���ܲ���������������Ϣ
uint8_t RX_hd;//���ܻҶȴ�����������Ϣ
double jl=200;//��ʼ��ʻ����(�����þ���mm)
int jxb_num=0;//��е��λ�ñ�־��0����ʼλ��, 1����  , 2���� , 3���� ��
double pos_X=0,pos_Y=0;//��ͼ��������
float jxb_T[51]={0,0.3,2.4,2,1.2,0.3,3.6,4.1,5.1,3,3.7,4.8,0.3,4,3.4,4,0.6,0.6,0.6,1.3,1.3,1.3,0.8,0.8,1.9,1.3,1.9,2,1.9,2};//��е�ۻ���ʱ���
float ylq_num=20;
int cjg_num[4]={0,3,2,1};//��Ӧ��ɫ���ŵ�λ�ã��죬�̣�����,1����ߣ�2���м䣬3���ұ�
int zcq_num[4]={0,3,2,1};//��Ӧ��ɫ���ŵ�λ�ã��죬�̣�����,1����ߣ�2���м䣬3���ұ�
int cjg_sb_num=2;
int zcq_sb_num=2;


uint8_t m2_num[6]={0};//�����ά����Ϣ
float angle_value; //�Ƕ�ֵ(�����)

JiXieBi_t woDeJiXieBi;//�����е��

bool zp_xz=true;//ת����ת�����־��true-˳ʱ�룬false-��ʱ�룩

float yddw[][3]={//----��Ӧx,y,z���꣨�Ի�е�۵����ŷ���ΪX��������Y��Ϊ��ֱ��������Ϊ������Z��Ϊˮƽ�����Ϸ���---------צ�����ĵ�����----//////------------�����������ѧ��ѧ���Լ������-------------
  {210,0,0},//�м���鴹ֱͶӰ--0
  {190.32,88.75,0},//�ұ���鴹ֱͶӰ--1
	{190.32,-88.75,0},//�����鴹ֱͶӰ--2+
    
	{230,0,100},//��ץȡ״̬���ߣ�--3
	{230,0,70}, //��ץȡ״̬���ͣ�--4
	{330,0,100}, //���ȥ��ת״̬ --5
	
	{0,330,100}, //���ȥ��ת״̬(�м�) --6
    {-330,0,100},  //ת�̴�ץȡ״̬ --7
	{-385,0,-40}, //ת��ץȡ���Ϻ�����״̬ --8
	{-280,-175,-40} //ת��ץȡ���� --9
  
};//�����е���˶���λ��


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
void lanya_num(uint8_t *RX);
void JD_Angle_ReceiveData(uint8_t rec);// ����Z�������ǽǶ����ݺ���
void JiXieBi_PeiZhi(void);//��е�����ú���
void wk_cz(bool n);//����ת������λ��
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)//���ڷ��ͻص�����
{
    if(huart->Instance==USART1)//(�������������)
    {


    }
		 if(huart->Instance==USART2)//(��ά���ȡ)
    {	


    }
	if(huart->Instance==UART4)//�����߶�������壩
	{


	}
	if(huart->Instance==UART5)//����ݮ�ɣ�
	{
		// ������ݮ�ɵ�����
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)// ������
{
	if(huart->Instance == USART1)// ������������崮��
	{
		//HAL_UART_Receive_IT(&huart1, RX_pjdj, 4); // ��������
    }                                                                                                                                                                                                                                           
	if(huart->Instance == USART3)//�ж��Ƿ�Ϊ����3(����������)
	{
		//bool---->false;
		Data_Analyse(Rec);
		HAL_UART_Receive_IT(&huart3,&Rec, 1);
	}
	if(huart->Instance == USART2)//�ж��Ƿ�Ϊ����2------------------------------------(��ݮ��)
    {
		smp_por(smp_Rec);
		HAL_UART_Receive_IT(&huart2,&smp_Rec, 1);//��ݮ����Ϣ	 
    }
	
    if(huart->Instance == UART4)//�ж��Ƿ�Ϊ����4(���߶��)
    {
		
    }
	 if(huart->Instance == UART5)//�ж��Ƿ�Ϊ����5------------------------------------(������Z��Ƕ�����)-------���2#STM32�ϵ�������������ɨ��ģ�飩
	{
		JD_Angle_ReceiveData(RX_hd);//���ݽ�������
		HAL_UART_Receive_IT(&huart5,&RX_hd, 1);//�������˵�������Z��Ƕ�  
	}
}





/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void jxb_pos(int num,bool zt);
void cd_cs(void);//�������̴���
void UI_OLED(void);






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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
	
	
	MAX7219_Init();                      //����ܳ�ʼ��
	
	HAL_UART_Init(&huart2);              //��ʼ������
	HAL_UART_Receive_IT(&huart2,&smp_Rec, 1); //�����жϺ���----------------------------------(��ݮ��)
	
	 
	
	HAL_UART_Init(&huart3);              //��ʼ������
	HAL_UART_Receive_IT(&huart3,&Rec, 1);//�����жϺ��������������֣�


	HAL_UART_Init(&huart5);              //��ʼ������
  HAL_UART_Receive_IT(&huart5,&RX_hd, 1);//�����жϺ���---------------------------------��STM32-2#-������Z��Ƕ����ݺͶ�������˫��STM32����������

	
	OLED_Init();                         //OLED��ʼ
	OLED_Clear();                        //����
 
    Gyro_Init();   //������Z��Ƕȳ�ʼ������
    HAL_Delay(500);//��ʱ�ȴ������ǳ�ʼ�����


	cd_cs();//�������̴���
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		lanya_num(&RX);
		UI_OLED();//OLED_UI����
		
		// Nixie_rwm("321+123");
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
}

/* USER CODE BEGIN 4 */

/**************************************************************************��*������*��************************************************************************************/
/**
  * @brief    ��е�����ú���
  */
void JiXieBi_PeiZhi(void)
{
    /* ���û�е���������� */
    woDeJiXieBi.OA = 66.71f;   /* ��������һ�ؽڵĳ��ȣ���λmm */
    woDeJiXieBi.AB = 110.0f;   /* ��۳��ȣ���λmm */
    woDeJiXieBi.BC = 150.0f;   /* С�۳��ȣ���λmm */
    woDeJiXieBi.theta = 77.01f; /* OA��XYƽ��ļнǣ��� */
    /* ���ó�ʼ�Ƕȣ��ȣ� */
    woDeJiXieBi.alpha0 = 0.0f;   /* ������ת���ʼ�Ƕ� */
    woDeJiXieBi.beta0 = 16.30f;   /* ��۳�ʼ�Ƕ� *///ԭ��21.03f
    woDeJiXieBi.gamma0 = 19.62f;  /* С�۳�ʼ�Ƕ� *///ԭ��23.58f
    /* ���õ����ַ */
    woDeJiXieBi.dizuoAddr = 5;   /* ������ת�����ַ */
    woDeJiXieBi.dabiAddr = 6;    /* ��۵����ַ */
    woDeJiXieBi.xiaobiAddr = 7;  /* С�۵����ַ */
    /* �������������� - ����ʵ�ʵ���ͼ��ٱȵ��� */
    woDeJiXieBi.dizuoMaiChong = 3200.0/360.0;  /* ������ת��ÿ�ȶ�Ӧ�������� */
    woDeJiXieBi.dabiMaiChong = 16*3200.0*10/360.0;   /* ���ÿ�ȶ�Ӧ�������� */
    woDeJiXieBi.xiaobiMaiChong = 16*3200.0*10/360.0; /* С��ÿ�ȶ�Ӧ�������� */
    /* ���������У׼�������� */
    woDeJiXieBi.diubuJianCe = 0;        /* ���ö������ */
    woDeJiXieBi.pengzhuangJiaoZhun = 0;  /* ������ײ����У׼ */

    woDeJiXieBi.diubuYuZhi = 0.5f;      /* ���������ֵ0.5�� */
    /* ��ʼ����е�� */
    JiXieBi_ChuShiHua(&woDeJiXieBi);
}






/**
  * @brief    UI������ʾ����
  */
void UI_OLED(void)
{
	static uint8_t n=0;
	if(n!=RX)
    {
		OLED_ShowNum(0,0,RX,3,16,1); // ��ʾ������������	
		n=RX; // ����n��ֵ
	}          
         
		 OLED_Showdecimal(0,2,pos_x,4,2,16,1);
		 OLED_Showdecimal(0,4,pos_y,4,2,16,1);
		 OLED_Showdecimal(0,6,zangle,4,2,16,1);
		 OLED_Showdecimal(62,2,pos_X,4,2,16,1);
		 OLED_Showdecimal(62,4,pos_Y,4,2,16,1);		
		 
		 OLED_Showdecimal(60,6,angle_value,4,2,16,1);//�Ƕ�ֵ(�����)
		 //OLED_ShowNum(60,6,hd_num[0],3,16,1);//�Ҷȴ���������
		 //OLED_ShowNum(85,6,hd_num[1],3,16,1);//�Ҷȴ��������� 
	     //OLED_ShowChar(72,6,Rec,16,0);
		 //OLED_ShowString(25,0,(char*)smp_Rec,16,0);//������ݮ�ɷ��Ĵ���
		
         OLED_Showdecimal(25,0,smp_num[0],3,1,16,1);
		 OLED_Showdecimal(65,0,smp_num[1],3,1,16,1);
		 //OLED_ShowNum(106,0,smp_num[2],1,16,1);

		 //OLED_Showdecimal(0,0,smp_num[0],3,2,16,1);
         //OLED_Showdecimal(50,0,smp_num[1],3,2,16,1);
         //OLED_ShowNum(100,0,smp_num[2],1,16,1);        
		  
}

/**
  * @brief    ��������
  * @param    RX  ���������͵�ָ��    
  */
void lanya_num(uint8_t *RX)
{
	
	//Nixie_showNum(1,*RX%10,0);           //�������ʾ
    if(*RX<5)
	{
		if(*RX<=4)
		{
      	 Motor_pos(*RX,jl,0,0);//С�����λ��ģʽ
			//Motor_pos_s(*RX,jl,&pos_X,&pos_Y,3,0);
		}
		else
		{
			Motor_Stop();
		}
		//Motor_pos_s(*RX,jl);
	}
	else if(*RX==5)
		 {
		   Data_stop(&pos_X,&pos_Y);//��������
		 }
	else if(*RX>5)
		 { 
		   switch(*RX)
			 {
			   case 6:
				 {
					
				   break;
				 }
				 case 7:
				 {
					
				   break;
				 }
				 case 8:
				 {
					
				   break;
				 }
				 case 9:
				 {
				
				   break;
				 }
				 case 10:
				 {
					
				   break;
				 }
				 case 11:
				 {
					
				   break;
				 }
				  case 12:
				 {
					 //jxb_pos_s(1);�ϰ�
				   break;
				 }
				 case 13:
				 {
					 //jxb_pos_s(2);�ϰ�
				   break;
				 }
				 case 14:
				 {
					 //jxb_pos_s(3);�ϰ�
				   break;
				 }
				 case 15://��λ
				 {
					 
				   break;
				 }
				 case 16:
				 {
					 Motor_jd_90(&pos_X,&pos_Y,1);
				   break;
				 }
				 case 17:
				 {
					//cd_cs5();
					//cd_cs4();
					//cd_cs2();
					//cd_cs3();
				   break;
				 }
				  case 18:
				 {
					 Motor_jd_90(&pos_X,&pos_Y,0);
				   break;
				 }
				  case 19:
				 {
					 //smp_2m_dq();
					 smp_fs(1);
					 
				   break;
				 }
				 default:
					 break;
			 }
		 }
	*RX=255;
}

/**
 * @brief  ��������֡��ʽ��0x55 + 2�ֽڽǶ�����-----------------------������Ƕ����ݷ�����---------------------------------
 * @brief  ��������֡��ʽ��0xFF + 6�ֽڵ���������Ϣ
 * @param rec ���յ��ĵ����ֽ�
 */
void JD_Angle_ReceiveData(uint8_t rec)
{
	static uint8_t ch;
	static uint8_t frame_data[2]; // �洢2�ֽڽǶ�����
	static uint8_t count = 0;
	static int16_t angle_received;
	ch = rec;
	switch(count)
	{
		case 0: // �ȴ�֡ͷ 0x55
			if(ch == 0x55)
				count++;
			else if(ch == 0xFF)
				count=100;//��ת�����������
			else
				count = 0;
			break;
			
		case 1: // ���սǶȵ��ֽ�
			frame_data[0] = ch;
			count++;
			break;
			
		case 2: // ���սǶȸ��ֽ�
			frame_data[1] = ch;
			
			// ����Ƕ����ݣ��з���16λ������
			angle_received = (int16_t)((frame_data[1] << 8) | frame_data[0]);
			
			// ת��Ϊʵ�ʽǶ�ֵ������100�ָ�С����
			angle_value = (float)angle_received / 100.0f;
			count = 0; // ����״̬��
			break;

		case 100:case 101:case 102:case 103:case 104:case 105://���������
	    {
			m2_num[count-100] = ch;
			count++;
			if(count == 106)
			{
				count=0;
			}
			break;
		}
		default:
			count = 0;
			break;
	}
}
	


/**
  * @brief    ��е��ϸ�ֿ��ƣ�λ��ģʽ��
  * @param    num     :������ID����¼��TXT�ļ���  
  * @param    zt      :������ִ��״̬��1-�л��壬0-�޻��壩
  */
void jxb_pos(int num,bool zt)
{
   runActionGroup(num,1);
   UI_OLED();
   if(zt)
   {
	   HAL_Delay(jxb_T[num]*1000); // ��ʱ�ȴ���е�۶������
   }
}

/**
  * @brief    ת�����λ�ø���
  * @param    n:true-˳ʱ�룬false-��ʱ��
  */
void wk_cz(bool n)
{ 
  if(n)
  {
	for(int i=0;i<3;i++)
    {
	smp_wk[i]=(smp_wk[i]+1)%3;//����
    }
  }
  else
  {
	for(int i=0;i<3;i++)
	{
	  smp_wk[i]=smp_wk[i]-1<0?smp_wk[i]-1+3:smp_wk[i]-1;//����
	}
  }
}

/**
  * @brief    视觉等待超时包装函数
  * @param    cmd      发送给树莓派的命令
  * @param    timeout_ms  超时时间(ms), 默认3000ms
  * @return   true=成功收到数据, false=超时
  */
bool smp_wait_with_timeout(int cmd, uint32_t timeout_ms)
{
    smp_fs(cmd);
    uint32_t deadline = HAL_GetTick() + timeout_ms;
    while(!smp_bool && HAL_GetTick() < deadline) {
        HAL_Delay(5);
        if(HAL_GetTick() > deadline - 500) {
            smp_fs(cmd); // 超时前重发一次
        }
    }
    if(!smp_bool) {
        // 超时: 使用默认值
        smp_num[0] = 0;
        smp_num[1] = 0;
        smp_num[2] = 0;
        smp_wk[0] = 0;
        smp_wk[1] = 1;
        smp_wk[2] = 2;
        return false;
    }
    smp_fs(2); // 关闭树莓派
    smp_bool = false;
    return true;
}

 /**
  * @brief    ���������˶���һ��ʽ������////////������ɫ��1����ɫ���󣩣�2����ɫ���У���3����ɫ���ң���-------------��û�����-----------------
  */
void cd_cs(void)
{
      Motor_pos_s(3,155,&pos_X,&pos_Y,2,0,0);
	  Motor_pos_s(1,650+95,&pos_X,&pos_Y,3,0,0);//����ά��崦
	
	  smp_2m_dq();  //��ȡ��ά��
	 
      while(!(m2_num[0]*m2_num[1]*m2_num[2]*m2_num[3]*m2_num[4]*m2_num[5]))
	  {
          HAL_Delay(5);//�ȴ���ά����Ϣ�������
	  }
	  Nixie_rwm((char*)m2_num);//��ʾ�����ά��
	  HAL_Delay(200);//�ȴ���ά����ʾ
	  jxb_pos(1,false);//��е��Ԥ��������ʶ�����˳��
     
	  Motor_pos_s(1,800-58-95,&pos_X,&pos_Y,3,0,0);//��һ�ε�ԭ����
		Motor_jd_s(&pos_X,&pos_Y,0);
	
	  //-----------------------------------------------------------------------------�������ϡ�------------------------------------------------------------------------------------------------
	  Motor_pos_s(4,60,&pos_X,&pos_Y,3,0,0);//����Ԥ��������λ��
       //��е��ʶ��+��ȡ
      //jxb_pos(1,true);//��е��Ԥ��������ʶ�����˳��
	  smp_fs(0);//��ݮ��ʶ�������ɫ
      while(!smp_bool)
	  {
        HAL_Delay(5);//�ȴ���ɫ��Ϣ�������
		smp_fs(0);
	  }
	  smp_fs(2);//�ر���ݮ��
      smp_bool=false;//�����־λ
       
	  wk_cz(zp_xz);//����ת�����λ��(��ʼ��)
      
	  for(int i=0;i<3;i++)
	  { 
        jxb_pos(smp_wk[m2_num[i]-1]+16,true);//��е�۵���Ӧ�����¸�λ�õȴ�ץȡ
        HAL_Delay(100);//�ȴ���е���ȶ�
		smp_fs(2+m2_num[i]);
		while(!smp_bool)
	    {
         HAL_Delay(5);//�ȴ���ɫ��Ϣ�������
		 smp_fs(2+m2_num[i]);
	    }
		smp_fs(2);//�ر���ݮ��
		smp_bool=false;//�����־λ
				
		jxb_pos(smp_wk[m2_num[i]-1]+19,true);

		if(i==2)//��ץ�߷�
		{
		 jxb_pos(m2_num[i]+1,false);//��е��ץ�����--�ص�����ץȡλ��
         Motor_pos_s(3,60,&pos_X,&pos_Y,3,0,0);//�˳�Ԥ��������λ��
		}
		else
		jxb_pos(m2_num[i]+1,true);//��е��ץ�����--�ص�����ץȡλ��

        wk_cz(zp_xz);//����ת�����λ��
	  }

      //��е��ʶ��+��ȡ
	  //Motor_pos_s(3,60,&pos_X,&pos_Y,3,0,0);//�˳�Ԥ��������λ��


		//----------------------------------------------------------------------�������ϡ�-------------------------------------------------------------------------------------------
		
	  Motor_pos_s(2,400-58,&pos_X,&pos_Y,3,0,0);
	  Motor_pos_s(3,860,&pos_X,&pos_Y,3,0,0);
      jxb_pos(22,false);
	  Motor_pos_s(3,860,&pos_X,&pos_Y,3,0,0);//��һ�ε��ּӹ���
//--------------------------------------------------------------------------------����һ�δּӹ�������--�á�-----------------------------------------------------------------------------------------------------
      //��ݮ�ɶ�λ//     
      //jxb_pos(22,true);
	  HAL_Delay(200);//�ȶ���е��
	  smp_fs(1);
	  while(!smp_bool)
      {
		HAL_Delay(5);
		smp_fs(1);//��ݮ��ʶ�������ɫ
	  }
	  smp_fs(2);//�ر���ݮ��
	  smp_bool=false;//�����־λ
      
	  //jxb_pos(5);//��е�۳��ӹ�Ԥ��

	  Motor_smp(&pos_X,&pos_Y,smp_num[0],smp_num[1],1,0);//��ݮ��΢��
      //�ȷ�//
      for(int i=0;i<3;i++)
	  {
          jxb_pos(m2_num[i]+5,true);//��е�۷�
	  }	  
	  //�ȷ�//
	  //����//
	  for(int i=0;i<3;i++)
	  {

        if(i!=2)
		{
			jxb_pos(m2_num[i]+8,true);//��е��ץ
		}
        if(i==2)//��ץ�߷�
		{
			jxb_pos(m2_num[i]+8,false);//��е��ץ
			HAL_Delay(1500);//�ȴ���е���ȶ�
            Motor_pos_s(1,825,&pos_X,&pos_Y,3,0,0);
		}
	  }
	  //����	  
//--------------------------------------------------------------------------------����һ�δּӹ�������--�á�-----------------------------------------------------------------------------------------------------	
	  //Motor_pos_s(1,825,&pos_X,&pos_Y,3,0,0);
      jxb_pos(23,false);
	  Motor_pos_s(4,825,&pos_X,&pos_Y,3,0,0);  
//----------------------------------------------------------------------------------����һ���ݴ������š�------------------------------------------------------------------------------------------------		
	  //��ݮ�ɶ�λ//     
      //jxb_pos(23,true);
	  HAL_Delay(200);//�ȶ���е��
	  smp_fs(1);
	  while(!smp_bool)
      {
		HAL_Delay(5);
		smp_fs(1);//��ݮ��ʶ�������ɫ
	  }
	  smp_fs(2);//�ر���ݮ��
	  smp_bool=false;//�����־λ
      
	  //jxb_pos(12);//��е�۳��ӹ�Ԥ��
	  Motor_smp(&pos_X,&pos_Y,smp_num[0],smp_num[1],2,0);//��ݮ��΢��
	  //�������
	  for(int i=0;i<3;i++)
      {
		if(i!=2)
		jxb_pos(m2_num[i]+12,true);//��е�۷�
		else
		{
			jxb_pos(m2_num[i]+12,false);//��е�۷�
			HAL_Delay(3300);//�ȴ���е���ȶ�
            Motor_pos_s(4,895,&pos_X,&pos_Y,3,0,0);
		}
	  }
	 //�������
//--------------------------------------------------------------����һ���ݴ������š�---------------------------------------------------------------------------------------------
    jxb_pos(1,false);//��е��Ԥ��������ʶ�����˳��
	Motor_pos_s(2,425+58,&pos_X,&pos_Y,3,0,0);//�ڶ��ε�ԭ����
			
//-------------------------------------------------------------------�������ϡ�------------------------------------------------------------------------------------------------
    Motor_pos_s(4,60,&pos_X,&pos_Y,3,0,0);//����Ԥ��������λ��
       //��е��ʶ��+��ȡ
      //jxb_pos(1,true);//��е��Ԥ��������ʶ�����˳��
	  smp_fs(0);//��ݮ��ʶ�������ɫ
     while(!smp_bool)
	  {
        HAL_Delay(5);//�ȴ���ɫ��Ϣ�������
		smp_fs(0);
	  }
	  smp_fs(2);//�ر���ݮ��
      smp_bool=false;//�����־λ


	  for(int i=0;i<3;i++)
	  { 
        jxb_pos(smp_wk[m2_num[i+3]-1]+16,true);//��е�۵���Ӧ�����¸�λ�õȴ�ץȡ
        HAL_Delay(100);//�ȴ���е���ȶ�
		smp_fs(2+m2_num[i+3]);
		while(!smp_bool)
	    {
         HAL_Delay(5);//�ȴ���ɫ��Ϣ�������
		 smp_fs(2+m2_num[i+3]);
	    }
		smp_fs(2);//�ر���ݮ��
		smp_bool=false;//�����־λ

		jxb_pos(smp_wk[m2_num[i+3]-1]+19,true);
		if(i==2)
		{
		 jxb_pos(m2_num[i+3]+1,false);//��е��ץ�����--�ص�����ץȡλ��
         Motor_pos_s(3,60,&pos_X,&pos_Y,3,0,0);//�˳�Ԥ��������λ��
		}
		else
		jxb_pos(m2_num[i+3]+1,true);//��е��ץ�����--�ص�����ץȡλ��

        wk_cz(zp_xz);//����ת�����λ��
	  }

      //��е��ʶ��+��ȡ
	  //Motor_pos_s(3,60,&pos_X,&pos_Y,3,0,0);//�˳�Ԥ��������λ��
//----------------------------------------------------------------------�������ϡ�--------------------------------------------------------------------------------------------- 
	  Motor_pos_s(2,400-58,&pos_X,&pos_Y,3,0,0);
	  Motor_pos_s(3,860,&pos_X,&pos_Y,1,0,0);
      jxb_pos(22,false);
	  Motor_pos_s(3,860,&pos_X,&pos_Y,3,0,0);//��һ�ε��ּӹ���
	  Motor_jd_s(&pos_X,&pos_Y,0);//��е��У׼
//--------------------------------------------------------------------------------���ڶ��δּӹ�������--�á�-----------------------------------------------------------------------------------------------------
      //jxb_pos(22,true);
	  HAL_Delay(200);//�ȶ���е��
	  smp_fs(1);
	  while(!smp_bool)
      {
		HAL_Delay(5);
		smp_fs(1);//��ݮ��ʶ�������ɫ
	  }
	  smp_fs(2);//�ر���ݮ��
	  smp_bool=false;//�����־λ
      
	  //jxb_pos(5);//��е�۳��ӹ�Ԥ��

	  Motor_smp(&pos_X,&pos_Y,smp_num[0],smp_num[1],1,0);//��ݮ��΢��
      //�ȷ�//
      for(int i=0;i<3;i++)
	  {
          jxb_pos(m2_num[i+3]+5,true);//��е�۷�
	  }	  
	  //�ȷ�//
	  //����//
	  for(int i=0;i<3;i++)
	  {
		  if(i!=2)
          jxb_pos(m2_num[i+3]+8,true);//��е��ץ
		  else
		  {
			jxb_pos(m2_num[i+3]+8,false);//��е��ץ
			HAL_Delay(1500);//�ȴ���е���ȶ�
			Motor_pos_s(1,825,&pos_X,&pos_Y,3,0,0);
		  }
	  }
	  //����	  
//--------------------------------------------------------------------------------���ڶ��δּӹ�������--�á�-----------------------------------------------------------------------------------------------------	
	  //Motor_pos_s(1,825,&pos_X,&pos_Y,3,0,0);
	  jxb_pos(23+m2_num[3],false);
	  Motor_pos_s(4,825,&pos_X,&pos_Y,3,0,0);//��һ�ε��ݴ���		
	  Motor_jd_s(&pos_X,&pos_Y,0);//��е��У׼
//-----------------------------------------------------------------------------------���ڶ����ݴ������š�------------------------------------------------------------------------------------------------
	 //�������----���
       jxb_pos(26+m2_num[3],true);
	   for(int i=4;i<6;i++)
	   {
		   jxb_pos(23+m2_num[i],true);
		   HAL_Delay(100);//�ȴ���е���ȶ�
		   jxb_pos(26+m2_num[i],true);
	   }


	 //�������----���
//--------------------------------------------------------------���ڶ����ݴ������š�---------------------------------------------------------------------------------------------
	  Motor_pos_s(4,895,&pos_X,&pos_Y,3,0,0);
	  Motor_pos_s(2,1875,&pos_X,&pos_Y,3,0,0);
	  Motor_pos_s(4,155,&pos_X,&pos_Y,3,0,0);//����ͣ��
	
}




/**************************************************************************��*������*��************************************************************************************/
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
