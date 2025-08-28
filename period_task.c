#include "period_task.h"
#include "All_variable.h"
#include "ethercat.h"
#include "AD_DA.h"
#include "CalMeter.h"
#include "LZY_J_identity.h"
#include "RediusCal.h"
#include "RisingEdges.h"
#include "Speed_Control_test.h"
#include "TLC7225.h"
#include "Vel_Given.h"
#include "base_function.h"
#include "flash_rw.h"
#include "speed_ratio_test.h"
#include "spi_w5500_hal.h"
#include "torquegiven.h"
#include "usart.h"
#include "w5500modbustcpclient.h"
#include "Vol_AirP_demarcate.h"
#include "arm_math.h"
#include "math.h"

void QiDong_StateMachine()
{
    // ��������
//   b_RisingQidong_b=Posedge(PC_LCD_Bottons.Qidong, 26);
    if (warning_flag ==0)
    {
        Physical_Led.RGY_RedLed_o    = !Physical_Led.Qidong_led;
        Physical_Led.RGY_GreenLed_o_o =Physical_Led.Qidong_led;
        Physical_Led.RGY_YellowLed_o =false; 
    }
    else
    {
        Physical_Led.RGY_RedLed_o    =false;
        Physical_Led.RGY_GreenLed_o_o=false;
        Physical_Led.RGY_YellowLed_o =true; 
    }
    switch (QiDongState)
    {
        case 0:  
            if (b_RisingQidong||b_RisingQidong_PC||b_QiDongFromSpeedUp)
            {   b_QiDongFromSpeedUp = 0;
                if ( !(b_slavesconnect_done==1 && b_slavespower_done==1))                    
                { 
                    warning_flag=1;//�ŷ�δʹ��
                    remain_io_state(50, &Physical_Led.RGY_Beep_o);
                    break;
                }
                else if (!Physical_Led.QiZhangZhou_led || !Physical_Led.QiZhangHuan_led
                    || !Physical_Led.ZuoGuDing_led  || !Physical_Led.YouGuDing_led)       
                {
                    remain_io_state(50, &Physical_Led.RGY_Beep_o);
                    warning_flag=2;//����δ����
                    break;
                }
                else
                {
                    QiDongState=1;
                    Physical_Led.Qidong_led =true;
                    warning_flag=0;
                    firstRunState=2;
                }
            }//if (b_RisingQidong||b_RisingQidong_PC||b_QiDongFromSpeedUp)��
            if (!(b_slavesconnect_done==1 && b_slavespower_done==1))                       
                { 
                    warning_flag=1;//�ŷ�δʹ��
                    break;
                }
                else if (!Physical_Led.QiZhangZhou_led || !Physical_Led.QiZhangHuan_led
                    || !Physical_Led.ZuoGuDing_led  || !Physical_Led.YouGuDing_led)   
                { 
                    warning_flag=2;//����δ����
                    break;
                }
                 else
                {
                    if (firstRunState==1){
                    QiDongState=1;
                    Physical_Led.Qidong_led =true;
                    firstRunState=2;
                        }
                    warning_flag=0;
                }
            break;
        
        case 1:           
            if ((b_RisingQidong||b_RisingQidong_PC))
            {
                Physical_Led.Qidong_led=false;
                QiDongState=0;
                warning_flag=0;
            }
            if ( !(b_slavesconnect_done==1 && b_slavespower_done==1))         
            { 
                warning_flag=1;//�ŷ�δʹ��
                Physical_Led.Qidong_led=false;
                QiDongState=0;
                break;
            }
            else if (!Physical_Led.QiZhangZhou_led || !Physical_Led.QiZhangHuan_led|| !Physical_Led.ZuoGuDing_led  || !Physical_Led.YouGuDing_led)   
            { 
                warning_flag=2;//����δ����
                Physical_Led.Qidong_led=false;
                QiDongState=0;   
            }
                    
          break;
     }//switch��        
}

void state_identify()//�Ƿ��о���-״̬��
{
  
  static uint8 OperationStatesCount=0;
  static  float Length_before=0;//��Ĥǰ������
    
    if(b_RisingQiZhangZhou_LED)//�������ᣬѭ����ʼ������о���
        {OperationStates=1;} 
    if(b_FallingQiZhangZhou_LED)
        {OperationStates=0;} 
    b_Rising_intel_manipulate_PC=Posedge(PC_LCD_Bottons.intel_manipulate,57);
    b_Rising_intel_manipulate2_PC=Posedge(PC_LCD_Bottons.intel_manipulate2,58);
        
    if (OperationStates!=20&&OperationStates!=21&&OperationStates!=22)
    {Max_linear_speed_formal=Max_linear_speed_UI;}
    if(Fj_Tension_Filter>60)
    {
        OperationStates=8;//��������״̬
    }
    
    if (Axis[1].Alarm_signal==128||tracker[7].x2<-90.0)
    {
        OperationStates=7;//��ͣ״̬
    }
    tracker[7].ratio = 10006;
    tracker[7].fhan = fhan(tracker[7].output-fabs(QY_speed), tracker[7].x2, tracker[7].ratio, main_task_period/10); 
    tracker[7].output = tracker[7].output + main_task_period / 1000 * tracker[7].x2;
    tracker[7].x2 = tracker[7].x2 + main_task_period / 1000 * tracker[7].fhan;
//    
//    tracker[8].ratio = 10006;
//    tracker[8].fhan = fhan(tracker[8].output-tracker[7].x2, tracker[8].x2, tracker[8].ratio, main_task_period/10); 
//    tracker[8].output = tracker[8].output + main_task_period / 1000 * tracker[8].x2;
//    tracker[8].x2 = tracker[8].x2 + main_task_period / 1000 * tracker[8].fhan;
    switch (OperationStates)
    {
        case 0://�޾���
            state_display=0;
            intel_manipulate_state_display=0;//��ʼ��һ�����ܲ�����ʾ
            break;
        case 1://�о���   
            if(Physical_Led.QiZhangZhou_led)
            {state_display=1;}//�о���״̬
            else{state_display=0;}
            if (QY_v_mode==1 && Tension_mode==1) //&& PhaseRunning<3
            //����������봩������״̬
            { 
                OperationStates=2;
            } 
            if (b_Rising_intel_manipulate_PC)//һ�����ܰ�ť���Զ�����
            {OperationStates=20;
            OperationStatesCount=0;}
            if (b_Rising_intel_manipulate2_PC)//һ�����ܰ�ť2������
            {
                speed_state=1;
                intel_manipulate_state_display=1;//һ�����ܲ�����ʾ:��������
            }
            break;
        case 2://�о��ϣ�������������״̬
            state_display=24;//�����ֶ���������״̬
            if (b_RisingQiZhangHuan_LED || b_RisingLength_clear || (fj_circles>3))
            {OperationStates=3;}
//            if (b_Rising_intel_manipulate_PC)//һ�����ܰ�ť���Զ�����
//            {OperationStates=20;
//             OperationStatesCount=0;}   
            break;
        case 20://�о��ϣ������Զ���������״̬���Զ�����ǰ,�����������ᣬ�����͡�������
            state_display=23;                   //�����Զ���������״̬
            intel_manipulate_state_display=3;   //һ�����ܲ�����ʾ:�����Զ�����
            Physical_Led.YouGuDing_led=1;
            Physical_Led.ZuoGuDing_led=1;
            Physical_Led.QiZhangHuan_led=1;
            firstRunState=1;                    //������������
            Max_linear_speed_formal=Max_linear_speed_ChuanMo; 
            OperationStates=21;
        case 21://�Զ��������ӳ�֮������
            OperationStatesCount+=1;
        if (OperationStatesCount>=10)
        {
            Length_before=Length_now;
            speed_state=1;
            OperationStates=22;
            OperationStatesCount=0;
        }
        break;
        case 22://�Զ�����������ָ��������ֹͣ�Զ�����
            Qy_Lead_S=auto_stop_meter(smooth_Target_linear_Speed_filter,0,Stop_time,tracker[3].ratio,Max_linear_speed_UI);
            Delta_Length=Len_ChuanMo+Length_before-Qy_Lead_S; 
            if (Delta_Length<=Length_now)
            {
//                Max_linear_speed_formal=Max_linear_speed_UI;
//                b_RisingLength_clear=1��
                speed_state=3;
                OperationStates=2;
                intel_manipulate_state_display=0;//һ�����ܲ�����ʾ
            }
            
            if (b_RisingSpeedup_PC||b_Rising_intel_manipulate_PC)//���¡����١���ť���߰��¡�һ�����ܰ�ť-���١�
            {
                OperationStates=2;
                Max_linear_speed_formal=Max_linear_speed_UI;
                intel_manipulate_state_display=1;//һ�����ܲ�����ʾ:��������
//                speed_state=1;
//                state_display=23;//��������
            }
            if (b_RisingSpeeddown_PC||b_Rising_intel_manipulate2_PC)//���¡����١���ť���߰��¡�һ�����ܰ�ť2-���١�
            {
                OperationStates=2;
                intel_manipulate_state_display=2;//һ�����ܲ�����ʾ:���ڼ���
//                speed_state=2;
//                state_display=24;//���ڼ���
            }
        break;
        case 3://����״̬��������
            if (QY_v_mode==0 && Tension_mode==1)
            {
               OperationStates=5;   //�о��ϣ�����ͣ
            }
            if (QY_v_mode==1 && Tension_mode==1 )
            {OperationStates=4;}    //�о��ϣ�������������
        break;
        case 4://������������״̬
            //��״̬��ʾ��
            if(speed_state==0)
            {state_display=2;}  //������������״̬
            else if (speed_state==1)
            {state_display=21;} //������������״̬
            else if (speed_state==2)
            {state_display=22;} //������������״̬
//            state_display=2;  //������������״̬
            //��״̬��ʾ��
            if(b_FallingQiZhangZhou_LED)//���������ᣬ����޾���
            {//�޾���״̬ 
            OperationStates=0;}//ѭ������
            if (QY_v_mode==0 && Tension_mode==1 )
                {OperationStates=5;}//�о��ϣ�����ֹͣ    
            if (Tension_mode==0)
            {  
               OperationStates=6;//����״̬
            }
           //��2�����ܰ�ť�Ĳ�����
           if(speed_state==0)
           {
               if (b_Rising_intel_manipulate_PC)//һ�����ܰ�ť������
               {
                     intel_manipulate_state_display=1;//һ�����ܲ�����ʾ:��������
                     speed_state=1;
               }
               if (b_Rising_intel_manipulate2_PC)//һ�����ܰ�ť������
               {
                     intel_manipulate_state_display=2;//һ�����ܲ�����ʾ:���ڼ���
                     speed_state=2;
               }
           }
           else if(speed_state==1)
           {
               if (b_Rising_intel_manipulate_PC)//һ�����ܰ�ť������
               {
                     intel_manipulate_state_display=0;//һ�����ܲ�����ʾ
                     speed_state=0; 
               }
               if (b_Rising_intel_manipulate2_PC)//һ�����ܰ�ť������
               {
                     intel_manipulate_state_display=2;//һ�����ܲ�����ʾ:���ڼ���
                     speed_state=2;
               }
           }
           else if(speed_state==2)
           {
               if (b_Rising_intel_manipulate_PC)//һ�����ܰ�ť������
               {
                     intel_manipulate_state_display=1;//һ�����ܲ�����ʾ:��������
                     speed_state=1;
               }
               if (b_Rising_intel_manipulate2_PC)//һ�����ܰ�ť������
               {
                     intel_manipulate_state_display=0;//һ�����ܲ�����ʾ
                     speed_state=0; 
               }
           }
           //��2�����ܰ�ť�Ĳ�����
        //���Զ�ͣ����           
        if (Fj_filter<=stop_to_20_Fj_R)// ���Զ����ٵ�20��/��
            {
                OperationStates=41;
            }
            break; 
        case 40: 
            if(b_FallingQiZhangZhou_LED)//���������ᣬ����޾���
            {//�޾���״̬ 
            OperationStates=0;}//ѭ������
            if (QY_v_mode==0 && Tension_mode==1 )
                {OperationStates=5;}//�о��ϣ�����ֹͣ    
            if (Tension_mode==0)
            {               
               OperationStates=6;//����״̬
            }
            break;
        case 41:
            state_display=8;//�Զ�ͣ����
            if (Target_linear_Speed>20)
            {   
                speed_state=3;
                OperationStates=42;
            }
            else
            {OperationStates=40;}
            break;
        case 42:
            if(Target_linear_Speed<=20)
            {
                Target_linear_Speed=20;
                speed_state=0;
//                speed_state=3;
                OperationStates=40;
            }
            if (Tension_mode==0)
            {               
               OperationStates=6;//����״̬
            }
            break; 
       //���Զ�ͣ����     
        case 5://����ֹͣ״̬
            state_display=3;//����ֹͣ״̬
            intel_manipulate_state_display=0;//һ�����ܲ�����ʾ
            if(b_FallingQiZhangZhou_LED)//���������ᣬ����޾���
                {state_display=0;//�޾���״̬ 
                OperationStates=0;}//ѭ������
            if (QY_v_mode==1 && Tension_mode==1 )
                {OperationStates=4;}//�о��ϣ�������������
            if (Tension_mode==0 )
            {
               OperationStates=6;
            }
            if (b_Rising_intel_manipulate_PC)//һ�����ܰ�ť������
            {    
                 intel_manipulate_state_display=1;//һ�����ܲ�����ʾ:��������
                 speed_state=1;
            }         
            break; 
        case 6://����״̬
            state_display=14; //����״̬
            Tension_switch_display=0;

            warning_flag=13;//����״̬
            state_display=3;

            Target_linear_Speed = 0.0;//��������ٶ�ָ���ʵ�ʻ���û�ж�
            smooth_Target_linear_Speed_filter=0;
            tracker[3].output=0;
            tracker[3].fhan=0;
            tracker[3].x2=0;
            speed_state = 0;
            OperationStates=0;//�뿪����״̬
            break;
        case 7://��ͣ״̬
            state_display=15; //����״̬
            Tension_switch_display=0;

            warning_flag=14;//����״̬
            Target_linear_Speed = 0.0;//��������ٶ�ָ���ʵ�ʻ���û�ж�
            smooth_Target_linear_Speed_filter=0;
            tracker[3].output=0;
            tracker[3].fhan=0;
            tracker[3].x2=0;
            speed_state = 0;
            OperationStates=0;//�뿪����״̬
            break;
        case 8://��������״̬
            state_display=16; //����״̬
            Tension_switch_display=0;

            warning_flag=15;//����״̬
            Target_linear_Speed=0;//��������ٶ�ָ���ʵ�ʻ���û�ж�
            smooth_Target_linear_Speed_filter=0;
            tracker[3].output=0;
            tracker[3].fhan=0;
            tracker[3].x2=0;
            speed_state = 0;
            OperationStates=0;//�뿪����״̬
            break;
            
    }//switch��
    // ��ǣ���ٶ�ʶ���
    if (fabs(Act_Linear_Speed) < 0.3) // ��ǰ�ٶ�ʶ��
    {
        QY_v_mode = 0;
    }
    else if (fabs(Act_Linear_Speed) > 0.6) // ��ǰ�ٶ�ʶ��
    {
        QY_v_mode = 1;
    }
    // ��ǣ���ٶ�ʶ���
    //��С״̬ʶ���
    state_str[4]=Physical_Led.QiZhangZhou_led+'0';//��λ
    state_str[3]=speed_state+'0';//ʮλ
    state_str[2]=QY_v_mode+'0';
    state_str[1]=Tension_mode+'0';
    state_str[0]=(b_slavesconnect_done==1 && b_slavespower_done==1)+'0';  
    new_state_display=atoi(state_str);//�ַ���������
    //��С״̬ʶ���
    state_machine_display=(float)state_display;
}
    

void period__task()
{
    RisingEdges();
    Set_DA_Output();
    Get_SjTension();
        // ������״̬����
    updataspeedstate();
    
     if (Tension_switch_display)
    {
        VelocityGiven_forward();
    }      
//        state_identify(); // ״̬ʶ��
//        QiDong_StateMachine();//������ť��״̬��
//        Fj_RadiusCaldown();
//        Sj_RadiusCaldown();
//        Qy_Length_Cal();
//        Sj_Length_Cal();
        Vel_Given_period_task(Max_linear_speed_formal);		
//    switch ((int)Test_state_num)
//    {
//    case 1:               // ��������
//        state_identify(); // ״̬ʶ��
//        QiDong_StateMachine();//������ť��״̬��
//        Fj_RadiusCaldown();
//        Sj_RadiusCaldown();
//        Qy_Length_Cal();
//        Sj_Length_Cal();
//        Vel_Given_period_task(Max_linear_speed_formal);
//        break;
//    case 2: // ���ᶯ��ѧ����
//        test_finish = 0;
//        Lzyshibie();
//        break;
//    case 3: // ���Ӱ뾶����ʵ��
//        Vel_Given_period_task(Max_linear_speed_UI);
//        break;
//    case 4: // �ٱȲⶨʵ��
//        if (Vol_val > 9.5)
//            Vol_val = 9.5;
//        if (Vol_val < 0)
//            Vol_val = 0;
//        TLC7225_Val_set(Vol_val, Vol_tongdao);
//        TLC7225_Val_Load();
//        if (b_RisingLoad_VA) 
//        {
//            Vol_Array[VA_index] = Vol_val;
//            Airpress_Array[VA_index] = Airpress_val;
//            b_RisingLoad_VA = 0;
//            VA_index++;
//            VA_Test_Progress = VA_index / 10;
//        }
//        if (VA_index == 10)
//        {
//            myLeast[(int)Vol_tongdao] = Vol_AirP_demarcate(Vol_Array, Airpress_Array, 10);
//            VA_index = 0;
//        }
//        break;
//    case 5: // �ٶȿ���ʵ��
//        Vel_Given_period_task(Max_linear_speed_UI);
//        //			printf("%.2f\r\n,%.2f\r\n,%.2f\r\n,%.2f\r\n,%.2f\r\n",Sample_Vel[0],Sample_Vel[1],Sample_Vel[2],Sample_Vel[3],Sample_Vel[4]);
//        //			Sample_Count = 0;
//        break;
//    default:
//        printf("error code");
//        break;
//    }
}
