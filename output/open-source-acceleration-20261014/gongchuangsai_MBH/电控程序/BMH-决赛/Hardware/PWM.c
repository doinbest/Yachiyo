#include "stm32f4xx.h"                  // Device header
#include "yyb_move.h"
#include "math.h"
#include "servo.h"
#include "delay.h"

typedef struct {
    float start_angle;    	//锟斤拷什么锟角度匡拷始
    float target_angle;	  	//目锟斤拷锟斤拷锟斤拷转锟斤拷锟劫角讹拷
    float current_angle;  	//锟斤拷前锟侥角讹拷锟角讹拷锟斤拷
    int moving_flag;    // 锟斤拷值锟斤拷锟铰憋拷志锟斤拷1: 锟斤拷锟斤拷锟剿讹拷锟斤拷0: 锟斤拷锟斤拷
    float t;            // 时锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷诓锟街碉拷锟斤拷锟?
	float time;		  //锟斤拷锟斤拷锟斤拷值锟剿讹拷锟斤拷锟斤拷锟绞憋拷锟?
    void (*SetServoAngle)(float angle);  // 锟斤拷锟矫碉拷前锟角度的猴拷锟斤拷指锟斤拷
} Servo_t;

Servo_t yuantai = {77.4f, 0.0f, 77.4f, 0, 0, 0, Servo_SetAngle4_yuntai};
Servo_t zhuashou = {2.7f, 2.7f, 2.7f, 0, 0, 0, Servo_SetAngle2_zhuashou};
Servo_t wukuaipingtai = {20.0f, 20.0f, 20.0f, 0, 0, 0,Servo_SetAngle3_wukuaipingtai};

void PWM_Init(void)
{
	/* 锟斤拷锟斤拷时锟斤拷 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);			// 锟斤拷锟斤拷TIM2锟斤拷时锟斤拷
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);			// 锟斤拷锟斤拷GPIOA锟斤拷时锟斤拷
	
	/* GPIO锟斤拷始锟斤拷 */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;					// 锟斤拷锟斤拷模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3; // 锟斤拷锟斤拷PA1, PA2, PA3
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;					// 锟斤拷锟斤拷锟斤拷锟?
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;				// 锟斤拷使锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
	GPIO_Init(GPIOA, &GPIO_InitStructure);							// 锟斤拷始锟斤拷GPIOA
	
	// 锟斤拷锟斤拷GPIO锟斤拷锟矫癸拷锟斤拷为TIM2
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);  // PA1 -> TIM2_CH2
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);  // PA2 -> TIM2_CH3
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);  // PA3 -> TIM2_CH4

	/* 锟斤拷锟斤拷时锟斤拷源 */
	TIM_InternalClockConfig(TIM2);		// 选锟斤拷TIM2为锟节诧拷时锟斤拷
	
	/* 时锟斤拷锟斤拷元锟斤拷始锟斤拷 */
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;				
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;     // 时锟接凤拷频锟斤拷选锟今不凤拷频
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; // 锟斤拷锟斤拷锟斤拷模式锟斤拷选锟斤拷锟斤拷锟较硷拷锟斤拷
	TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;				// 锟斤拷锟斤拷锟斤拷锟节ｏ拷锟斤拷ARR锟斤拷值锟斤拷锟斤拷锟斤拷PWM锟斤拷锟斤拷
	TIM_TimeBaseInitStructure.TIM_Prescaler = 84 - 1;				// 预锟斤拷频锟斤拷锟斤拷锟斤拷锟斤拷PWM频锟斤拷
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;            // 锟截革拷锟斤拷锟斤拷锟斤拷
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);             // 锟斤拷始锟斤拷TIM2时锟斤拷锟斤拷元
	
	/* 锟斤拷锟斤拷冉铣锟绞硷拷锟?*/ 
	TIM_OCInitTypeDef TIM_OCInitStructure;							
	TIM_OCStructInit(&TIM_OCInitStructure);                         
	
	// 锟斤拷锟斤拷通锟斤拷
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;               // 锟斤拷锟斤拷冉锟侥Ｊ斤拷锟窖★拷锟絇WM模式1
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;       // 锟斤拷锟斤拷锟斤拷裕锟窖★拷锟轿拷锟?
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   // 锟斤拷锟绞癸拷锟?
	TIM_OCInitStructure.TIM_Pulse = 0;							    // 锟斤拷锟斤拷占锟秸憋拷
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);                        // 锟斤拷始锟斤拷TIM2锟斤拷锟斤拷锟斤拷冉锟酵拷锟?
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);                        // 锟斤拷始锟斤拷TIM2锟斤拷锟斤拷锟斤拷冉锟酵拷锟?
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);                        // 锟斤拷始锟斤拷TIM2锟斤拷锟斤拷锟斤拷冉锟酵拷锟?
    
	NVIC_InitTypeDef NVIC_InitStructure;
    // 锟斤拷锟斤拷NVIC
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
	/* TIM使锟斤拷 */
	TIM_Cmd(TIM2, ENABLE);			// 使锟斤拷TIM2锟斤拷锟斤拷时锟斤拷锟斤拷始锟斤拷锟斤拷
	TIM_SetCompare2(TIM2, 520);		// 抓锟斤拷
	TIM_SetCompare3(TIM2, 648);		// 锟斤拷锟狡教?
	TIM_SetCompare4(TIM2, 930);		// 锟斤拷台
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}

void PWM_SetCompare2(uint16_t Compare)
{
	TIM_SetCompare2(TIM2, Compare);		// 锟斤拷锟斤拷CCR2锟斤拷值
}

void PWM_SetCompare3(uint16_t Compare)
{
	TIM_SetCompare3(TIM2, Compare);		// 锟斤拷锟斤拷CCR3锟斤拷值
}

void PWM_SetCompare4(uint16_t Compare)
{
	TIM_SetCompare4(TIM2, Compare);		// 锟斤拷锟斤拷CCR4锟斤拷值
}

// 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷目锟斤拷嵌龋锟斤拷锟皆变化锟斤拷锟斤拷********
void set_servo_angle(Servo_t *servo, float target)
{
    servo->target_angle = target;
    servo->moving_flag = 1;
}

// 锟斤拷值锟斤拷锟斤拷
float interpolate(float start, float target, float t) {
    if (t < 200) {
        return (25.0f / 8.0f) * target * pow(t / 1000.0f, 2) + start;
    } else if (t < 800) {
        return (5.0f / 4.0f) * target * (t / 1000.0f) - target / 8.0f + start;
    } else if (t < 1000) {
        return (-25.0f / 8.0f) * target * pow(t / 1000.0f, 2) + (25.0f / 4.0f) * target * (t / 1000.0f) - (17.0f / 8.0f) * target + start;
    } else {
        return start + target;
    }
}

// 锟斤拷值锟斤拷锟铰猴拷锟斤拷
void servo_update(Servo_t *servo) {
	double tau,s;
    if (servo->moving_flag == 1) {
		servo->t +=20;
		tau = servo->t / servo->time;
			if (tau <= 0.0) {
				servo->current_angle = servo->start_angle;
			} else if (tau >= 1.0) {
				servo->current_angle = servo->target_angle;
				servo->start_angle = servo->target_angle;
				servo->t = 0;
				servo->moving_flag = 0;
			} else {
				s = 6 * tau * tau * tau * tau * tau
				  - 15 * tau * tau * tau * tau
				  + 10 * tau * tau * tau;
				servo->current_angle = servo->start_angle + (servo->target_angle - servo->start_angle) * s;
			}	
        servo->SetServoAngle(servo->current_angle);
    }
}


//278锟斤拷为抓锟斤拷锟斤拷位锟斤拷
//???锟斤拷为锟斤拷锟矫碉拷锟斤拷锟狡教拷锟轿伙拷锟?
//64.8锟斤拷为锟斤拷始位锟斤拷
void set_yuantai_Angle(float target, float time){
    Servo_t *servo = &yuantai;
	if(target<=45.8f){
		target = 45.8f;
	}
    servo->target_angle = target;
	servo->time = time;
    servo->moving_flag = 1;
    // 锟饺达拷锟剿讹拷锟斤拷桑锟阶拷锟斤拷锟斤拷锟斤拷为锟饺达拷moving_flag为0
//    while(servo->moving_flag != 0){}
//    delay_ms1(50);
}

//2.7锟斤拷为锟斤拷始位锟斤拷
//10锟斤拷为抓锟斤拷锟斤拷位锟斤拷
//60锟斤拷锟脚匡拷锟斤拷爪
void set_zhuashou_Angle(float target, float time){
	target -=1.0f; 
    Servo_t *servo = &zhuashou;
    servo->target_angle = target;
	servo->time = time;
    servo->moving_flag = 1;
    // 锟饺达拷锟剿讹拷锟斤拷桑锟阶拷锟斤拷锟斤拷锟斤拷为锟饺达拷moving_flag为0
    while(servo->moving_flag != 0){}
    delay_ms1(30);
}

void set_zhuashou_Angle_kai(void)
{
	set_zhuashou_Angle(25.0f, 300);
}

void set_zhuashou_Angle_he(void)
{
	set_zhuashou_Angle(1.0f, 300);
}

//20.0f锟饺凤拷锟斤拷锟斤拷锟揭?
//20.0f+113.0f锟饺凤拷锟斤拷锟斤拷锟斤拷
//20.0f+113.0f+117.0f锟饺凤拷锟斤拷锟斤拷锟斤拷锟?
void set_wukuaipingtai_Angle(float target, float time){
    Servo_t *servo = &wukuaipingtai;
    servo->target_angle = target;
	servo->time = time;
    servo->moving_flag = 1;
//    // 锟饺达拷锟剿讹拷锟斤拷桑锟阶拷锟斤拷锟斤拷锟斤拷为锟饺达拷moving_flag为0
    while(servo->moving_flag != 0){}
    delay_ms1(50);
}

void set_wukuaipingtai(uint8_t task){
    if(task == 0x01){
		set_wukuaipingtai_Angle(20.0f,300);
	}else if(task == 0x02){
		set_wukuaipingtai_Angle(20.0f+124.0f,300);
	}else if(task == 0x03){
		set_wukuaipingtai_Angle(20.0f+123.0f+115.0f,300);
	}
}

//涓嶉樆濉炵増鏈?鍙彂閫佺洰鏍?涓嶇瓑寰呭畬鎴?鍙笌鍏朵粬杩愬姩骞惰
void set_wukuaipingtai_Angle_nb(float target, float time){
    Servo_t *servo = &wukuaipingtai;
    servo->target_angle = target;
	servo->time = time;
    servo->moving_flag = 1;
}

void set_zhuashou_Angle_nb(float target, float time){
	target -=1.0f; 
    Servo_t *servo = &zhuashou;
    servo->target_angle = target;
	servo->time = time;
    servo->moving_flag = 1;
}

//绛夊緟鑸垫満杩愬姩瀹屾垚
void servo_wait_done(Servo_t *servo){
    while(servo->moving_flag != 0){}
}

// 统一锟斤拷锟斤拷锟斤拷锟斤拷锟脚凤拷锟侥革拷锟铰猴拷锟斤拷
void TIM2_IRQHandler(void) {
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        // 锟街憋拷锟斤拷锟斤拷锟斤拷锟斤拷欧锟斤拷牟锟街?
        servo_update(&yuantai);
        servo_update(&zhuashou);
        servo_update(&wukuaipingtai);

        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}

/*前锟斤拷锟斤拷锟斤拷锟斤拷锟街憋拷为
	平锟狡碉拷目锟斤拷位锟矫ｏ拷65.0f 锟斤拷 -122.0 之锟斤拷
	平锟狡碉拷锟劫度ｏ拷 1000 rpm
	平锟狡的硷拷锟劫度ｏ拷200rpm
  锟叫硷拷锟斤拷锟斤拷锟斤拷锟斤拷锟街憋拷为
	锟斤拷锟斤拷锟斤拷目锟斤拷位锟矫ｏ拷0.0f 锟斤拷 130.0f
	锟斤拷锟斤拷锟斤拷锟劫度ｏ拷1000 rpm
	锟斤拷锟斤拷锟侥硷拷锟劫度ｏ拷200 rpm
  锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷直锟轿?
	锟斤拷台锟斤拷目锟斤拷位锟矫ｏ拷0 锟斤拷 360
	锟斤拷台锟斤拷转锟劫ｏ拷13锟斤拷锟斤拷

*/
int move_all(float target_pos, uint16_t speed, uint8_t accel,float target_pos1, uint16_t speed1, uint16_t accel1,float target, float speed_of_yuntai)
{	
//	accel1 = 200;
//	accel = 200;
//	speed1 = 2000;
	target = target + 9.2f;
	double time1 = pingtui_control(target_pos,speed,accel);
	double time2 = shengjiang_control(target_pos1,speed1,accel1);
//	double time = 100 * fabs(target - yuantai.current_angle)/speed_of_yuntai;
	double time;
	if(fabs(target - yuantai.current_angle) <= 28){
		time = 120 * fabs(target - yuantai.current_angle)/13.0f;
	}else if(fabs(target - yuantai.current_angle) <= 50){
		time = 100 * fabs(target - yuantai.current_angle)/13.0f;
	}else if(fabs(target - yuantai.current_angle) <= 100){
		time = 85 * fabs(target - yuantai.current_angle)/13.0f;
	}else if(fabs(target - yuantai.current_angle) <= 150){
		time = 70 * fabs(target - yuantai.current_angle)/13.0f;
	}else if(fabs(target - yuantai.current_angle) <= 250){
		time = 60 * fabs(target - yuantai.current_angle)/13.0f;
	}else{
		time = 50 * fabs(target - yuantai.current_angle)/13.0f;
	}
//	time = time * (2-0.1*speed_of_yuntai);
	set_yuantai_Angle(target,time);
	double max = fmax(time, fmax(time1, time2));
	delay_ms1((int)max);
	return (int)max;
}


int move_all1(float target_pos, uint16_t speed, uint8_t accel,float target_pos1, uint16_t speed1, uint8_t accel1,float target, float speed_of_yuntai)
{	
	accel1 = 240;
	accel = 230;
	speed1 = 2000;
	target = target + 9.2f;
	double time1 = pingtui_control(target_pos,speed,accel);
	double time2 = shengjiang_control(target_pos1,speed1,accel1);
//	double time = 100 * fabs(target - yuantai.current_angle)/speed_of_yuntai;
	double time;
	if(fabs(target - yuantai.current_angle) <= 28){
		time = 120 * fabs(target - yuantai.current_angle)/13.0f;
	}else if(fabs(target - yuantai.current_angle) <= 50){
		time = 100 * fabs(target - yuantai.current_angle)/13.0f;
	}else if(fabs(target - yuantai.current_angle) <= 100){
		time = 85 * fabs(target - yuantai.current_angle)/13.0f;
	}else if(fabs(target - yuantai.current_angle) <= 150){
		time = 70 * fabs(target - yuantai.current_angle)/13.0f;
	}else if(fabs(target - yuantai.current_angle) <= 250){
		time = 60 * fabs(target - yuantai.current_angle)/13.0f;
	}else{
		time = 50 * fabs(target - yuantai.current_angle)/13.0f;
	}
	set_yuantai_Angle(target,time);
	double max = fmax(time, fmax(time1, time2));
	return (int)max;
}

