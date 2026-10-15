#include "Motor.h"
#include "oled.h"



//С����      1     2
//(�������)       
//              ��
//            3     4
//����
// ���������Ʋ����ṹ�壬�����������
typedef struct {
    int motor_id;//�����ţ�1 2 3 4
    int direction;//������� CW  ��ʾ˳ʱ�룬CWW ��ʾ��ʱ��
} MotorControlParam;
// ���岻ͬ�����Ӧ�ĵ�����Ʋ�������
const MotorControlParam motor_params[4][2] = {
    // ���� 1����
    {{2, CWW}, {3, CW}},
    // ���� 2����
    {{2, CW}, {3, CWW}},
    // ���� 3����
    {{1, CWW}, {4, CW}},
    // ���� 4����
    { {1, CW}, {4, CWW}}
};

/**
  * @brief    С��λ��ģʽ������ǰ����
  * @param    fx  ��С���������1������2������3������4������
  * @param    jl  ���˶����� (��λ��mm)  
  * @param    num ���Ƕ�У���־λ (false:��У��,true:У��)
  * @param    jd_num :�����������׼�Ƕ�
  */
void Motor_pos(int fx,double jl,bool num,double jd_num)
{   
  
    int zs = (int)(jl * 3200*16 / (lzjl * M_PI));
    if(fx==3 || fx==4)//��������ҷ���
    {
        for (int i = 0; i < 2; i++) {
            Emm_V5_Pos_Control(motor_params[fx - 1][i].motor_id, motor_params[fx - 1][i].direction, vel, acc, zs, 0, 1);
            HAL_Delay(5);// ��ʱ����
        }
        Emm_V5_Synchronous_motion(0);
    }
    else
    {// ����������Ʋ������飬���ο��Ƶ��
        for (int i = 0; i < 2; i++) {
            Emm_V5_Pos_Control(motor_params[fx - 1][i].motor_id, motor_params[fx - 1][i].direction, vel, acc, zs, 0, 1);
            HAL_Delay(5);// ��ʱ����
        }                                                                                                                                                                                      
        Emm_V5_Synchronous_motion(0);
   }

    Motor_Delay(jl); // ��ʱ���� 
    if (num) {
        Motor_jd(angle_value, 1, jd_num); // �Ƕ�����
    }
}



/**
  * @brief    С��ǿ��ֹͣ     
  */
void Motor_Stop()
{
  Emm_V5_Stop_Now(0,0);// ֹͣ���е���˶�
}



//--------------------------------------------------------------------------------------ʱ�䲹��������--------------------------------------------------------------------------------------------//
/**
 * @brief ����ת������ʱ�䣨ms�������Ӽ��١�Ħ��������
 * @param steps ���루mm��
 * @return ��ʱ�䣨ms��
 */
int calc_time_for_steps(double steps) {
    double v = vel * M_PI * lzjl / 60.0; // mm/s
    double a = acc * M_PI * lzjl / 60.0; // mm/s^2

    if (v <= 0 || a <= 0) return 1000;

    // ��������ٶ��ܴﵽʱ�ļ��پ���
    double t_acc = v / a;
    double s_acc = 0.5 * a * t_acc * t_acc;

    double s_total = steps;

    if (s_total < 2 * s_acc) {
        // ����̫�̣��ﲻ������ٶȣ������ٶ�����
        double t = sqrt(s_total / a) * 2; // ����+����ʱ��
        // ����
        t *= FRICTION_FACTOR * ERROR_MARGIN;
        return (int)(t * 1000 + 0.5);
    } else {
        // ���������ٶ�����
        double s_flat = s_total - 2 * s_acc;
        double t_flat = s_flat / v;
        double total_time = 2 * t_acc + t_flat;
        total_time *= FRICTION_FACTOR * ERROR_MARGIN;
        return (int)(total_time * 1000 + 0.5);
    }
}
/**
  * @brief    С������λ��ָ���ʱ�䲹��
  * @param    steps  �����ƶ��ľ��� (��λ��mm)
  * @note     �ڲ�����ݲ��������Ӧ����ʱ��ms����Ȼ����� HAL_Delay ��ʱ
  */
void Motor_Delay(uint32_t steps)
{
    if(steps >=300) {
       HAL_Delay(calc_time_for_steps(steps)*0.53+350); // ��ʱ����
    }
    else
    {
        HAL_Delay(calc_time_for_steps(steps)*0.52+200); // ��ʱ����
    }

    
}

 /**
  * @brief    С�����ε��Ƕȵ�ʱ�䲹��
  */
void Motor_Delay_jd(double jd_num)
{
  HAL_Delay(300+jd_num*8);// ��ʱ����
  if(jd_num>40)// ����Ƕȴ���40��
  {
    HAL_Delay(jd_num*17);// ��ʱ����
  }
}
//--------------------------------------------------------------------------------------ʱ�䲹��������--------------------------------------------------------------------------------------------//

/**
  * @brief    С���߾�λ��ģʽ���������������ָ�����λ��
  * @param    fx     ��С���������1������2������3������4������
  * @param    jl     ���˶����� (��λ��mm) 
  * @param    pos_X  ��С������x��λ��
  * @param    pos_Y  ��С������y��λ��
  * @param    key    ��΢��������־λ��0������,1:ֻ���˶����ҷ���2��ֻ���˶�����3�����ޣ�
  * @param    jd_num :�����������׼�Ƕȣ����ڽǶ�������(�����ʹ�ýǶ�����Ļ��ͺ�jd_xs��ֵһ��)--------------�ر�ע������---------------
  * @param    jd_xs  :С����ʻ�������꣬���������������ı仯��(�����ʹ�ýǶ�����Ļ��ͺ�jd_num��ֵһ��)-------------�ر�ע������---------------
  */
void Motor_pos_s(int fx,double jl,double*pos_X,double*pos_Y,int key,double jd_num,double jd_xs)
{
// ���ڴ洢����ֵ
int deltaX = 0;
int deltaY = 0;
// ���ݲ�ͬ�����׼�ǶȺͲ�ͬ�ķ����������ֵ
switch ((int)jd_xs)  // �����������׼�Ƕ�
{
case 0:
    switch (fx) {
        case 1: deltaY = jl; break;
        case 2: deltaY = -jl; break;
        case 3: deltaX = -jl; break;
        case 4: deltaX = jl; break;
    }
    break;
case 90:
    switch (fx) {
        case 1: deltaX = -jl; break;
        case 2: deltaX = jl; break;
        case 3: deltaY = -jl; break;
        case 4: deltaY = jl; break;
    }
    break;
case 180:
    switch (fx) {
        case 1: deltaY = -jl; break;
        case 2: deltaY = jl; break;
        case 3: deltaX = jl; break;
        case 4: deltaX = -jl; break;
    }
    break;
case -90:
    switch (fx) {
        case 1: deltaX = jl; break;
        case 2: deltaX = -jl; break;
        case 3: deltaY = jl; break;
        case 4: deltaY = -jl; break;
    }
    break;
}
// ���µ�ͼ�ľ�������
*pos_X += deltaX;
*pos_Y += deltaY;

    // ��һ�δ�λ��
    Motor_pos(fx, jl, 1, jd_num);

// ��λ�����ҷ�������
if (key == 1 || key == 3) {
    double error_x = pos_x - *pos_X;// �������
    double error_y = pos_y - *pos_Y;// �������

    if(fabs(error_x)>0.1 || fabs(error_y)>0.1 )
    {
        if (fx == 1 || fx == 2)
        {
            if (error_x < 0) {
                Motor_pos(4, fabs(error_x), 0, jd_num);
            } else {
                Motor_pos(3, error_x, 0, jd_num);
            }
        } 
        else 
        {
            if (error_y < 0) {
                Motor_pos(1, fabs(error_y), 0, jd_num);
            } else {
                Motor_pos(2, error_y, 0, jd_num);
            }
        }
    }
}
    
 

    // ��λ�Ʒ�������
    if (key == 2 || key == 3) {
        double pos_x1 = pos_x;// ��¼��ʼλ��
        double pos_y1 = pos_y;// ��¼��ʼλ��
        double error;
        int direction;

        switch (fx) {
            case 1:
                error = *pos_Y - pos_y1;
                direction = error > 0 ? 1 : 2;
                break;
            case 2:
                error = pos_y1 - *pos_Y;
                direction = error > 0 ? 2 : 1;
                break;
            case 3:
                error = pos_x1 - *pos_X;
                direction = error > 0 ? 3 : 4;
                break;
            case 4:
                error = *pos_X - pos_x1;
                direction = error > 0 ? 4 : 3;
                break;
            default:
                return;
        }
        if(fabs(error)>0.1)//������
        Motor_pos(direction, fabs(error), 1, jd_num);// ����λ��
    }
}
/**
  * @brief    С��λ������(��̬)Ĭ��Ϊǰ������
  * @param    pos_X:��ͼ����ֵ����Xֵ
  * @param    pos_Y:��ͼ����ֵ����Yֵ
  * @param    jd_num :�����������׼�Ƕ�
  */
void Motor_jd_s(double*pos_X,double*pos_Y,double jd_num)
{
    // ���㵱ǰλ��������λ�õ����
    double error_x = pos_x - *pos_X;
    double error_y = pos_y - *pos_Y;
    switch ((int)jd_num)  // �����������׼�Ƕ�
    {
    case 0:// Ĭ�Ϸ���
       {
            // X���������
                if(fabs(error_x) > 0.1) {
                    if (error_x < 0) {
                        Motor_pos(4, fabs(error_x), 0, jd_num);  // �����ƶ�
                    } else {
                        Motor_pos(3, error_x, 0, jd_num);        // �����ƶ�
                    }
                }
                
                // Y���������
                if(fabs(error_y) > 0.1) {
                    if (error_y < 0) {
                        Motor_pos(1, fabs(error_y), 0, jd_num);  // ��ǰ�ƶ�
                    } else {
                        Motor_pos(2, error_y, 0, jd_num);        // ����ƶ�
                    }
                }
        break;
       }
    case 90:
       {
       	// ����90�ȵ����
                // X���������
                if(fabs(error_x) > 0.1) {
                    if (error_x < 0) {
                        Motor_pos(2, fabs(error_x), 0, jd_num);  // �����ƶ�
                    } else {
                        Motor_pos(1, error_x, 0, jd_num);        // �����ƶ�
                    }
                }
                
                // Y���������
                if(fabs(error_y) > 0.1) {
                    if (error_y < 0) {
                        Motor_pos(4, fabs(error_y), 0, jd_num);  // ��ǰ�ƶ�
                    } else {
                        Motor_pos(3, error_y, 0, jd_num);        // ����ƶ�
                    }
                }


       	break;
       }
    case 180:
       {
       	// ����180�ȵ����
               	// X���������
                if(fabs(error_x) > 0.1) {
                    if (error_x < 0) {
                        Motor_pos(3, fabs(error_x), 0, jd_num);  // �����ƶ�
                    } else {
                        Motor_pos(4, error_x, 0, jd_num);        // �����ƶ�
                    }
                }
                
                // Y���������
                if(fabs(error_y) > 0.1) {
                    if (error_y < 0) {
                        Motor_pos(2, fabs(error_y), 0, jd_num);  // ��ǰ�ƶ�
                    } else {
                        Motor_pos(1, error_y, 0, jd_num);        // ����ƶ�
                    }
                }
        break;
       }
    case -90:
       {
       	// ����-90�ȵ����
                // X���������
                if(fabs(error_x) > 0.1) {
                    if (error_x < 0) {
                        Motor_pos(1, fabs(error_x), 0, jd_num);  // �����ƶ�
                    } else {
                        Motor_pos(2, error_x, 0, jd_num);        // �����ƶ�
                    }
                }
                
                // Y���������
                if(fabs(error_y) > 0.1) {
                    if (error_y < 0) {
                        Motor_pos(3, fabs(error_y), 0, jd_num);  // ��ǰ�ƶ�
                    } else {
                        Motor_pos(4, error_y, 0, jd_num);        // ����ƶ�
                    }
                }

        break;
       }
    }
   
}

 /**
  * @brief    С��λ�ýǶ�����
  * @param    jd  ��С��Ŀǰ�ĽǶ� (��λ����)
  * @param    n   :true(����150),false(����80)
  * @param    jd_num :�����������׼�Ƕ�(jd-jd_num=Ҫ��ת�ĽǶ�)---(��Χ��179.99�ȵ�-179.99��)��180������ǣ�
  */
void Motor_jd(double jd,bool n,double jd_num)
{
	double ja_z;// ��ʼ�ǶȲ�
	ja_z = jd - jd_num;// �����ʼ�ǶȲ�
	// ȷ���ǶȲ��� -180 �� 180 ��֮�䣬�Եõ���С��ת��
	if (ja_z > 179.99) {
		ja_z -= 360;
	}
	else if (ja_z < -179.99) {
		ja_z += 360;
	}

	int zs_pos = n?150:80;
	int zs=(int)(fabs(ja_z)/bs_jd);

    if(zs>=5)
    {
        if(ja_z>0)
        {
                // ���������Ʋ������飬����ѭ�����Ƶ��
                const uint8_t motor_ids[4] = {1, 2, 3, 4};// ������������� 1 2 3 4
                const uint8_t motor_directions[4] = {0, 0, 0, 0};// �������������� 0:˳ʱ�� 1:��ʱ��
                    // ѭ�����Ƶ��
                for (uint8_t i = 0; i < 4; i++) {
                    Emm_V5_Pos_Control(motor_ids[i], motor_directions[i], zs_pos, 80, zs, 0, 1);
                    HAL_Delay(5);// ��ʱ����
                }
                Emm_V5_Synchronous_motion(0);// ͬ���˶�          
                Motor_Delay_jd(ja_z); //��ʱ����        
        }
        else
        {
                // ��������źͷ������飬����ѭ������
                const uint8_t motor_ids[4] = {1, 2, 3, 4};// ������������� 1 2 3 4
                const uint8_t motor_directions[4] = {1, 1, 1, 1};// �������������� 0:˳ʱ�� 1:��ʱ��

                // ѭ�����Ƶ��
                for (int i = 0; i < 4; i++) {
                    Emm_V5_Pos_Control(motor_ids[i], motor_directions[i], zs_pos, 80, zs, 0, 1);
                    HAL_Delay(5);// ��ʱ����
                }
                Emm_V5_Synchronous_motion(0);
                Motor_Delay_jd(ja_z); // ��ʱ����         

        }
    }
}

/**
  * @brief    С��λ�ýǶ�+-90��
  * @param      n:true:˳ʱ��-90�ȣ�false:��ʱ��90��
  */
void Motor_jd_90(double*pos_X,double*pos_Y,bool n)
{
	if(n)
	{                    
	    Motor_jd(angle_value,1,-90);//�Ƕȴ�����
        HAL_Delay(2000);
		Motor_jd(angle_value,1,-90);//�Ƕ�ϸ����  
        Motor_jd_s(pos_X,pos_Y,-90);//λ������ 
	}
	else
	{
		Motor_jd(angle_value,1,90);//�Ƕȴ�����
        HAL_Delay(2000);
		Motor_jd(angle_value,1,90);//�Ƕ�ϸ���� 
        Motor_jd_s(pos_X,pos_Y,90);//λ������
	}
}

/**
  * @brief    С��λ�ýǶ�180��
  */
void Motor_jd_180(double*pos_X,double*pos_Y)
{    
	      Motor_jd(angle_value,1,180);//�Ƕȴ�����
          HAL_Delay(2800);
		  Motor_jd(angle_value,1,180);//
          Motor_jd_s(pos_X,pos_Y,180);//λ������
}

/**
  * @brief    С��λ�ýǶ�0��
  */
 void Motor_jd_0(double*pos_X,double*pos_Y)
 {    
		   Motor_jd(angle_value,1,0);//�Ƕȴ�����
           HAL_Delay(2500);
		   Motor_jd(angle_value,1,0);//
           Motor_jd_s(pos_X,pos_Y,0);//λ������
 }

/**
  * @brief    С����λƫ������(3�����꣺�������ꡢ����С�����ꡢ��ݮ��ʶ������)
  * @param    pos_X:��ͼ����ֵ����Xֵ
  * @param    pos_Y:��ͼ����ֵ����Yֵ
  * @param    x:ƫ��X(��ݮ�ɷ������������ֵ)--mm
  * @param    y:ƫ��Y(��ݮ�ɷ������������ֵ)--mm
  * @param    num:��־λ��1���ּӹ�����2���ݴ�����
  * @param    jd_num :�����������׼�Ƕ�
  */
void Motor_smp(double*pos_X,double*pos_Y,double x,double y,int num,double jd_num)
{

	//��ݮ�ɶ�λ
    switch (num)
    {
    case 1 :
    {
     if(x>0)
     {
        Motor_pos_s(1,x,pos_X,pos_Y,3,jd_num,jd_num);
     }
     else
     {
        Motor_pos_s(2,fabs(x),pos_X,pos_Y,3,jd_num,jd_num);
     } 
     
     if(y>0)
     {
        Motor_pos_s(3,y,pos_X,pos_Y,3,jd_num,jd_num);
     }
     else
     {
        Motor_pos_s(4,fabs(y),pos_X,pos_Y,3,jd_num,jd_num);
     }
     break;
    }
    case 2:
    {
      if(x>0)
     {
        Motor_pos_s(4,x,pos_X,pos_Y,3,jd_num,jd_num);
     }
     else
     {
        Motor_pos_s(3,fabs(x),pos_X,pos_Y,3,jd_num,jd_num);
     } 
     
     if(y>0)
     {
        Motor_pos_s(1,y,pos_X,pos_Y,3,jd_num,jd_num);
     }
     else
     {
        Motor_pos_s(2,fabs(y),pos_X,pos_Y,3,jd_num,jd_num);
     }
     break;   
    } 
    }

}


// ==================== 导航优化函数 ====================

/**
  * @brief    对角线运动 (利用麦克纳姆轮全向能力)
  * @param    dx     X方向分量 (mm, 正=右, 负=左)
  * @param    dy     Y方向分量 (mm, 正=前, 负=后)
  */
void Motor_pos_diag(double dx, double dy, double*pos_X, double*pos_Y, int key, double jd_num, double jd_xs)
{
    *pos_X += dx;
    *pos_Y += dy;

    double distance = sqrt(dx * dx + dy * dy);
    if (distance < 1.0) return;

    int zs = (int)(distance * 3200 * 16 / (lzjl * M_PI));
    double ratio_x = fabs(dx) / distance;
    double ratio_y = fabs(dy) / distance;

    uint8_t dir_m1, dir_m2, dir_m3, dir_m4;
    if (dx >= 0 && dy >= 0) {
        dir_m1 = CW;  dir_m2 = CWW; dir_m3 = CWW; dir_m4 = CW;
    } else if (dx >= 0 && dy < 0) {
        dir_m1 = CWW; dir_m2 = CW;  dir_m3 = CW;  dir_m4 = CWW;
    } else if (dx < 0 && dy >= 0) {
        dir_m1 = CWW; dir_m2 = CW;  dir_m3 = CWW; dir_m4 = CW;
    } else {
        dir_m1 = CW;  dir_m2 = CWW; dir_m3 = CW;  dir_m4 = CWW;
    }

    Emm_V5_Pos_Control(1, dir_m1, vel, acc, (int)(zs * ratio_x), 0, 1);
    HAL_Delay(5);
    Emm_V5_Pos_Control(2, dir_m2, vel, acc, (int)(zs * ratio_y), 0, 1);
    HAL_Delay(5);
    Emm_V5_Pos_Control(3, dir_m3, vel, acc, (int)(zs * ratio_y), 0, 1);
    HAL_Delay(5);
    Emm_V5_Pos_Control(4, dir_m4, vel, acc, (int)(zs * ratio_x), 0, 1);
    HAL_Delay(5);
    Emm_V5_Synchronous_motion(0);

    Motor_Delay(distance);

    if (key == 1 || key == 3) {
        double error_x = pos_x - *pos_X;
        if (fabs(error_x) > 0.1) {
            Motor_pos(error_x > 0 ? 3 : 4, fabs(error_x), 0, jd_num);
        }
    }
    if (key == 2 || key == 3) {
        double error_y = pos_y - *pos_Y;
        if (fabs(error_y) > 0.1) {
            Motor_pos(error_y > 0 ? 2 : 1, fabs(error_y), 0, jd_num);
        }
    }
}

/**
  * @brief    自适应单遍角度校正 (替代两遍校正, 节省2-3秒)
  */
void Motor_jd_adaptive(double jd, bool n, double jd_num, int max_retries)
{
    for (int retry = 0; retry < max_retries; retry++) {
        Motor_jd(angle_value, n, jd_num);
        HAL_Delay(200);

        double residual = angle_value - jd_num;
        if (residual > 179.99) residual -= 360;
        if (residual < -179.99) residual += 360;

        if (fabs(residual) < 0.5) break;
    }
}

/**
  * @brief    优化的 +-90度转向
  */
void Motor_jd_90_opt(double*pos_X, double*pos_Y, bool n)
{
    double target = n ? -90 : 90;
    Motor_jd_adaptive(angle_value, 1, target, 3);
    Motor_jd_s(pos_X, pos_Y, target);
}

/**
  * @brief    优化的 180度转向
  */
void Motor_jd_180_opt(double*pos_X, double*pos_Y)
{
    Motor_jd_adaptive(angle_value, 1, 180, 3);
    Motor_jd_s(pos_X, pos_Y, 180);
}

/**
  * @brief    优化的 0度转向
  */
void Motor_jd_0_opt(double*pos_X, double*pos_Y)
{
    Motor_jd_adaptive(angle_value, 1, 0, 3);
    Motor_jd_s(pos_X, pos_Y, 0);
}

