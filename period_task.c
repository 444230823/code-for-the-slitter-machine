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
    // 启动开关
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
                    warning_flag=1;//伺服未使能
                    remain_io_state(50, &Physical_Led.RGY_Beep_o);
                    break;
                }
                else if (!Physical_Led.QiZhangZhou_led || !Physical_Led.QiZhangHuan_led
                    || !Physical_Led.ZuoGuDing_led  || !Physical_Led.YouGuDing_led)       
                {
                    remain_io_state(50, &Physical_Led.RGY_Beep_o);
                    warning_flag=2;//气胀未开启
                    break;
                }
                else
                {
                    QiDongState=1;
                    Physical_Led.Qidong_led =true;
                    warning_flag=0;
                    firstRunState=2;
                }
            }//if (b_RisingQidong||b_RisingQidong_PC||b_QiDongFromSpeedUp)的
            if (!(b_slavesconnect_done==1 && b_slavespower_done==1))                       
                { 
                    warning_flag=1;//伺服未使能
                    break;
                }
                else if (!Physical_Led.QiZhangZhou_led || !Physical_Led.QiZhangHuan_led
                    || !Physical_Led.ZuoGuDing_led  || !Physical_Led.YouGuDing_led)   
                { 
                    warning_flag=2;//气胀未开启
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
                warning_flag=1;//伺服未使能
                Physical_Led.Qidong_led=false;
                QiDongState=0;
                break;
            }
            else if (!Physical_Led.QiZhangZhou_led || !Physical_Led.QiZhangHuan_led|| !Physical_Led.ZuoGuDing_led  || !Physical_Led.YouGuDing_led)   
            { 
                warning_flag=2;//气胀未开启
                Physical_Led.Qidong_led=false;
                QiDongState=0;   
            }
                    
          break;
     }//switch的        
}

void state_identify()//是否有卷料-状态机
{
  
  static uint8 OperationStatesCount=0;
  static  float Length_before=0;//穿膜前的米数
    
    if(b_RisingQiZhangZhou_LED)//开气胀轴，循环开始，则→有卷料
        {OperationStates=1;} 
    if(b_FallingQiZhangZhou_LED)
        {OperationStates=0;} 
    b_Rising_intel_manipulate_PC=Posedge(PC_LCD_Bottons.intel_manipulate,57);
    b_Rising_intel_manipulate2_PC=Posedge(PC_LCD_Bottons.intel_manipulate2,58);
        
    if (OperationStates!=20&&OperationStates!=21&&OperationStates!=22)
    {Max_linear_speed_formal=Max_linear_speed_UI;}
    if(Fj_Tension_Filter>60)
    {
        OperationStates=8;//张力超限状态
    }
    
    if (Axis[1].Alarm_signal==128||tracker[7].x2<-90.0)
    {
        OperationStates=7;//急停状态
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
        case 0://无卷料
            state_display=0;
            intel_manipulate_state_display=0;//初始化一键智能操作显示
            break;
        case 1://有卷料   
            if(Physical_Led.QiZhangZhou_led)
            {state_display=1;}//有卷料状态
            else{state_display=0;}
            if (QY_v_mode==1 && Tension_mode==1) //&& PhaseRunning<3
            //如果机器进入穿带运行状态
            { 
                OperationStates=2;
            } 
            if (b_Rising_intel_manipulate_PC)//一键智能按钮：自动穿带
            {OperationStates=20;
            OperationStatesCount=0;}
            if (b_Rising_intel_manipulate2_PC)//一键智能按钮2：升速
            {
                speed_state=1;
                intel_manipulate_state_display=1;//一键智能操作显示:正在升速
            }
            break;
        case 2://有卷料：机器穿带运行状态
            state_display=24;//机器手动穿带运行状态
            if (b_RisingQiZhangHuan_LED || b_RisingLength_clear || (fj_circles>3))
            {OperationStates=3;}
//            if (b_Rising_intel_manipulate_PC)//一键智能按钮：自动穿带
//            {OperationStates=20;
//             OperationStatesCount=0;}   
            break;
        case 20://有卷料：机器自动穿带运行状态。自动穿带前,开启【气胀轴，环】和【启动】
            state_display=23;                   //机器自动穿带运行状态
            intel_manipulate_state_display=3;   //一键智能操作显示:正在自动穿带
            Physical_Led.YouGuDing_led=1;
            Physical_Led.ZuoGuDing_led=1;
            Physical_Led.QiZhangHuan_led=1;
            firstRunState=1;                    //【启动】灯亮
            Max_linear_speed_formal=Max_linear_speed_ChuanMo; 
            OperationStates=21;
        case 21://自动穿带：延迟之后，升速
            OperationStatesCount+=1;
        if (OperationStatesCount>=10)
        {
            Length_before=Length_now;
            speed_state=1;
            OperationStates=22;
            OperationStatesCount=0;
        }
        break;
        case 22://自动穿带：到达指定米数，停止自动穿带
            Qy_Lead_S=auto_stop_meter(smooth_Target_linear_Speed_filter,0,Stop_time,tracker[3].ratio,Max_linear_speed_UI);
            Delta_Length=Len_ChuanMo+Length_before-Qy_Lead_S; 
            if (Delta_Length<=Length_now)
            {
//                Max_linear_speed_formal=Max_linear_speed_UI;
//                b_RisingLength_clear=1；
                speed_state=3;
                OperationStates=2;
                intel_manipulate_state_display=0;//一键智能操作显示
            }
            
            if (b_RisingSpeedup_PC||b_Rising_intel_manipulate_PC)//按下【升速】按钮或者按下【一键智能按钮-升速】
            {
                OperationStates=2;
                Max_linear_speed_formal=Max_linear_speed_UI;
                intel_manipulate_state_display=1;//一键智能操作显示:正在升速
//                speed_state=1;
//                state_display=23;//正在升速
            }
            if (b_RisingSpeeddown_PC||b_Rising_intel_manipulate2_PC)//按下【减速】按钮或者按下【一键智能按钮2-减速】
            {
                OperationStates=2;
                intel_manipulate_state_display=2;//一键智能操作显示:正在减速
//                speed_state=2;
//                state_display=24;//正在减速
            }
        break;
        case 3://过渡状态，不久留
            if (QY_v_mode==0 && Tension_mode==1)
            {
               OperationStates=5;   //有卷料：机器停
            }
            if (QY_v_mode==1 && Tension_mode==1 )
            {OperationStates=4;}    //有卷料：机器正常运行
        break;
        case 4://机器正常运行状态
            //↓状态显示↓
            if(speed_state==0)
            {state_display=2;}  //机器正常运行状态
            else if (speed_state==1)
            {state_display=21;} //机器升速运行状态
            else if (speed_state==2)
            {state_display=22;} //机器减速运行状态
//            state_display=2;  //机器正常运行状态
            //↑状态显示↑
            if(b_FallingQiZhangZhou_LED)//若关气胀轴，则→无卷料
            {//无卷料状态 
            OperationStates=0;}//循环结束
            if (QY_v_mode==0 && Tension_mode==1 )
                {OperationStates=5;}//有卷料：机器停止    
            if (Tension_mode==0)
            {  
               OperationStates=6;//断料状态
            }
           //↓2个智能按钮的操作↓
           if(speed_state==0)
           {
               if (b_Rising_intel_manipulate_PC)//一键智能按钮：升速
               {
                     intel_manipulate_state_display=1;//一键智能操作显示:正在升速
                     speed_state=1;
               }
               if (b_Rising_intel_manipulate2_PC)//一键智能按钮：减速
               {
                     intel_manipulate_state_display=2;//一键智能操作显示:正在减速
                     speed_state=2;
               }
           }
           else if(speed_state==1)
           {
               if (b_Rising_intel_manipulate_PC)//一键智能按钮：升速
               {
                     intel_manipulate_state_display=0;//一键智能操作显示
                     speed_state=0; 
               }
               if (b_Rising_intel_manipulate2_PC)//一键智能按钮：减速
               {
                     intel_manipulate_state_display=2;//一键智能操作显示:正在减速
                     speed_state=2;
               }
           }
           else if(speed_state==2)
           {
               if (b_Rising_intel_manipulate_PC)//一键智能按钮：升速
               {
                     intel_manipulate_state_display=1;//一键智能操作显示:正在升速
                     speed_state=1;
               }
               if (b_Rising_intel_manipulate2_PC)//一键智能按钮：减速
               {
                     intel_manipulate_state_display=0;//一键智能操作显示
                     speed_state=0; 
               }
           }
           //↑2个智能按钮的操作↑
        //↓自动停机↓           
        if (Fj_filter<=stop_to_20_Fj_R)// 先自动降速到20米/分
            {
                OperationStates=41;
            }
            break; 
        case 40: 
            if(b_FallingQiZhangZhou_LED)//若关气胀轴，则→无卷料
            {//无卷料状态 
            OperationStates=0;}//循环结束
            if (QY_v_mode==0 && Tension_mode==1 )
                {OperationStates=5;}//有卷料：机器停止    
            if (Tension_mode==0)
            {               
               OperationStates=6;//断料状态
            }
            break;
        case 41:
            state_display=8;//自动停机中
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
               OperationStates=6;//断料状态
            }
            break; 
       //↑自动停机↑     
        case 5://机器停止状态
            state_display=3;//机器停止状态
            intel_manipulate_state_display=0;//一键智能操作显示
            if(b_FallingQiZhangZhou_LED)//若关气胀轴，则→无卷料
                {state_display=0;//无卷料状态 
                OperationStates=0;}//循环结束
            if (QY_v_mode==1 && Tension_mode==1 )
                {OperationStates=4;}//有卷料：机器正常运行
            if (Tension_mode==0 )
            {
               OperationStates=6;
            }
            if (b_Rising_intel_manipulate_PC)//一键智能按钮：升速
            {    
                 intel_manipulate_state_display=1;//一键智能操作显示:正在升速
                 speed_state=1;
            }         
            break; 
        case 6://断料状态
            state_display=14; //断料状态
            Tension_switch_display=0;

            warning_flag=13;//断料状态
            state_display=3;

            Target_linear_Speed = 0.0;//如果发出速度指令，但实际机器没有动
            smooth_Target_linear_Speed_filter=0;
            tracker[3].output=0;
            tracker[3].fhan=0;
            tracker[3].x2=0;
            speed_state = 0;
            OperationStates=0;//离开断料状态
            break;
        case 7://急停状态
            state_display=15; //断料状态
            Tension_switch_display=0;

            warning_flag=14;//断料状态
            Target_linear_Speed = 0.0;//如果发出速度指令，但实际机器没有动
            smooth_Target_linear_Speed_filter=0;
            tracker[3].output=0;
            tracker[3].fhan=0;
            tracker[3].x2=0;
            speed_state = 0;
            OperationStates=0;//离开断料状态
            break;
        case 8://张力超限状态
            state_display=16; //断料状态
            Tension_switch_display=0;

            warning_flag=15;//断料状态
            Target_linear_Speed=0;//如果发出速度指令，但实际机器没有动
            smooth_Target_linear_Speed_filter=0;
            tracker[3].output=0;
            tracker[3].fhan=0;
            tracker[3].x2=0;
            speed_state = 0;
            OperationStates=0;//离开断料状态
            break;
            
    }//switch的
    // ↓牵引速度识别↓
    if (fabs(Act_Linear_Speed) < 0.3) // 当前速度识别
    {
        QY_v_mode = 0;
    }
    else if (fabs(Act_Linear_Speed) > 0.6) // 当前速度识别
    {
        QY_v_mode = 1;
    }
    // ↑牵引速度识别↑
    //↓小状态识别↓
    state_str[4]=Physical_Led.QiZhangZhou_led+'0';//个位
    state_str[3]=speed_state+'0';//十位
    state_str[2]=QY_v_mode+'0';
    state_str[1]=Tension_mode+'0';
    state_str[0]=(b_slavesconnect_done==1 && b_slavespower_done==1)+'0';  
    new_state_display=atoi(state_str);//字符串→整型
    //↑小状态识别↑
    state_machine_display=(float)state_display;
}
    

void period__task()
{
    RisingEdges();
    Set_DA_Output();
    Get_SjTension();
        // 升降速状态更新
    updataspeedstate();
    
     if (Tension_switch_display)
    {
        VelocityGiven_forward();
    }      
//        state_identify(); // 状态识别
//        QiDong_StateMachine();//启动按钮的状态机
//        Fj_RadiusCaldown();
//        Sj_RadiusCaldown();
//        Qy_Length_Cal();
//        Sj_Length_Cal();
        Vel_Given_period_task(Max_linear_speed_formal);		
//    switch ((int)Test_state_num)
//    {
//    case 1:               // 正常工作
//        state_identify(); // 状态识别
//        QiDong_StateMachine();//启动按钮的状态机
//        Fj_RadiusCaldown();
//        Sj_RadiusCaldown();
//        Qy_Length_Cal();
//        Sj_Length_Cal();
//        Vel_Given_period_task(Max_linear_speed_formal);
//        break;
//    case 2: // 单轴动力学试验
//        test_finish = 0;
//        Lzyshibie();
//        break;
//    case 3: // 辊子半径矫正实验
//        Vel_Given_period_task(Max_linear_speed_UI);
//        break;
//    case 4: // 速比测定实验
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
//    case 5: // 速度控制实验
//        Vel_Given_period_task(Max_linear_speed_UI);
//        //			printf("%.2f\r\n,%.2f\r\n,%.2f\r\n,%.2f\r\n,%.2f\r\n",Sample_Vel[0],Sample_Vel[1],Sample_Vel[2],Sample_Vel[3],Sample_Vel[4]);
//        //			Sample_Count = 0;
//        break;
//    default:
//        printf("error code");
//        break;
//    }
}
