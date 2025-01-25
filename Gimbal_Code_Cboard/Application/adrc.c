/**
  ******************************************************************************
  * @file    adrc.c
  * @author  张曦梁
  * @version V1.0
  * @date    2021/10/24
  * @brief   ADRC计算函数
  ******************************************************************************
  * @attention
  *	
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Header.h"

AdrcTypeDef Gimbal_adrc[2];//YAW0 PITCH1
const float ADRC_Unit[3][16]=
{
/*TD跟踪微分器   改进最速TD,h0=N*h      扩展状态观察器ESO           扰动补偿           非线性组合*/
/*  r      h      N               beta_01    beta_02    beta_03     b0         beta_0    beta_1    beta_2      N1   C   alpha1  alpha2  zeta   b*/
 {300000 ,0.002 , 15,              500,      83333,     2000000,  0.001f,    0.02f,    2.3f,     0.001f,    5,   5,   0.8f,   1.5f,   50,   0},
 {300000 ,0.002 , 10,              500,      83333,     2000000,  0.001f,    0.02f,    1.0f,     0.00014f,   5,   5,   0.8f,   1.5f,   50,   1},
 {300000 ,0.002 , 20,              500,      83000,     10000,    0.001f,    0.02f,    3.0f,     0.002f,     5,   5,   0.8f,   1.5f,   50,   0},
};


s16 Sign_ADRC(float Input)
{
    s16 output=0;
    if(Input>1E-6f) 
			output=1;
    else if(Input<-1E-6f) 
			output=-1;
    else 
			output=0;
    return output;
}

s16 Fsg_ADRC(float x,float d)
{
  s16 output=0;
  output=(Sign_ADRC(x+d)-Sign_ADRC(x-d))/2;
  return output;
}

void ADRC_Init(AdrcTypeDef *fhan_Input1,AdrcTypeDef *fhan_Input2)
{
  fhan_Input1->r=ADRC_Unit[0][0];
  fhan_Input1->h=ADRC_Unit[0][1];
  fhan_Input1->N0=(u16)(ADRC_Unit[0][2]);
  fhan_Input1->beta_01=ADRC_Unit[0][3];
  fhan_Input1->beta_02=ADRC_Unit[0][4];
  fhan_Input1->beta_03=ADRC_Unit[0][5];
  fhan_Input1->b0=ADRC_Unit[0][6];
  fhan_Input1->beta_0=ADRC_Unit[0][7];
  fhan_Input1->beta_1=ADRC_Unit[0][8];
  fhan_Input1->beta_2=ADRC_Unit[0][9];
  fhan_Input1->N1=(u16)(ADRC_Unit[0][10]);
  fhan_Input1->c=ADRC_Unit[0][11];

  fhan_Input1->alpha1=ADRC_Unit[0][12];
  fhan_Input1->alpha2=ADRC_Unit[0][13];
  fhan_Input1->zeta=ADRC_Unit[0][14];
  fhan_Input1->b=ADRC_Unit[0][15];

  fhan_Input2->r=ADRC_Unit[1][0];
  fhan_Input2->h=ADRC_Unit[1][1];
  fhan_Input2->N0=(u16)(ADRC_Unit[1][2]);
  fhan_Input2->beta_01=ADRC_Unit[1][3];
  fhan_Input2->beta_02=ADRC_Unit[1][4];
  fhan_Input2->beta_03=ADRC_Unit[1][5];
  fhan_Input2->b0=ADRC_Unit[1][6];
  fhan_Input2->beta_0=ADRC_Unit[1][7];
  fhan_Input2->beta_1=ADRC_Unit[1][8];
  fhan_Input2->beta_2=ADRC_Unit[1][9];
  fhan_Input2->N1=(u16)(ADRC_Unit[1][10]);
  fhan_Input2->c=ADRC_Unit[1][11];

  fhan_Input2->alpha1=ADRC_Unit[1][12];
  fhan_Input2->alpha2=ADRC_Unit[1][13];
  fhan_Input2->zeta=ADRC_Unit[1][14];
  fhan_Input2->b=ADRC_Unit[1][15];
}

//ADRC最速微分跟踪器TD,改进的算法fhan
void Fhan_ADRC(AdrcTypeDef *fhan_Input,float expect_ADRC)//安排ADRC过渡过程
{
  float d=0,a0=0,y=0,a1=0,a2=0,a=0;
  float x1_delta=0;//ADRC状态跟踪误差项
  x1_delta=fhan_Input->x1-expect_ADRC;//用x1-v(k)代替x1得到离散更新公式
  fhan_Input->h0=fhan_Input->N0*fhan_Input->h;//用h0代替h,解决最速微分跟踪器超调部分
  d=fhan_Input->r*fhan_Input->h0*fhan_Input->h0;//d=rh^2;
  a0=fhan_Input->h0*fhan_Input->x2;//a0=h*x2
  y=x1_delta+a0;//y=x1+a0
  a1=sqrt(d*(d+8*Func_Abs(y)));//a1=sqrt(d*(d+8*ABS(y))])
  a2=a0+Sign_ADRC(y)*(a1-d)/2;//a2=a0+sign(y)*(a1-d)/2;
  a=(a0+y)*Fsg_ADRC(y,d)+a2*(1-Fsg_ADRC(y,d));
  fhan_Input->fh=-fhan_Input->r*(a/d)*Fsg_ADRC(a,d)
                  -fhan_Input->r*Sign_ADRC(a)*(1-Fsg_ADRC(a,d));//得到最速微分加速度跟踪量
  fhan_Input->x1+=fhan_Input->h*fhan_Input->x2;//更新最速状态追踪量x1
  fhan_Input->x2+=fhan_Input->h*fhan_Input->fh;//更新最速状态追踪量微分x2
}

//原点附近有连线性段的连续幂次函数
float Fal_ADRC(float e,float alpha,float zeta)
{
    s16 s=0;
    float fal_output=0;
    s=(Sign_ADRC(e+zeta)-Sign_ADRC(e-zeta))/2;
    fal_output=e*s/(powf(zeta,1-alpha))+powf(Func_Abs(e),alpha)*Sign_ADRC(e)*(1-s);
    return fal_output;
}




/***************扩展状态观察器********************/
//状态观测器参数beta01=1/h  beta02=1/(3*h^2)  beta03=2/(8^2*h^3) ...
void ESO_ADRC(AdrcTypeDef *fhan_Input)
{
  fhan_Input->e=fhan_Input->z1-fhan_Input->y;//状态误差

  fhan_Input->fe=Fal_ADRC(fhan_Input->e,0.5,fhan_Input->h);//非线性函数,提取跟踪状态与当前误差
  fhan_Input->fe1=Fal_ADRC(fhan_Input->e,0.25,fhan_Input->h);

  /*************扩展状态量更新**********/
  fhan_Input->z1+=fhan_Input->h*(fhan_Input->z2-fhan_Input->beta_01*fhan_Input->e);
  fhan_Input->z2+=fhan_Input->h*(fhan_Input->z3
                                 -fhan_Input->beta_02*fhan_Input->fe
                                   +fhan_Input->b*fhan_Input->u);
 //ESO估计状态加速度信号,进项扰动补偿
  fhan_Input->z3+=fhan_Input->h*(-fhan_Input->beta_03*fhan_Input->fe1);
}

void Nolinear_Conbination_ADRC(AdrcTypeDef *fhan_Input)
{
  float temp_e2=0;
  temp_e2=Func_Limit(fhan_Input->e2,5000,-5000);
  fhan_Input->u0=fhan_Input->beta_1*Fal_ADRC(fhan_Input->e1,fhan_Input->alpha1,fhan_Input->zeta)
                +fhan_Input->beta_2*Fal_ADRC(temp_e2,fhan_Input->alpha2,fhan_Input->zeta);

}


void ADRC_Control(AdrcTypeDef *fhan_Input,float feedback_ADRC,float expect_ADRC)
{
//	   if(expect_ADRC - feedback_ADRC>4096)
//			expect_ADRC=expect_ADRC-8192;
//		 else if( - expect_ADRC + feedback_ADRC>4096)
//			expect_ADRC=8192+expect_ADRC;
    /*自抗扰控制器第一步*/
      /*****
      安排过渡过程,输入为期望,
      由TD跟踪微分器得到:
      过度期望信号x1,过度期望信号微分x2
      ******/
      Fhan_ADRC(fhan_Input,expect_ADRC);

    /*自抗扰控制器第二步*/
      /************系统输出值为反馈量,状态反馈,ESO扩张状态观察器输入*********/
      fhan_Input->y=feedback_ADRC;
      /*****
      得到反馈信号的扩张状态:
      1状态信号z1;
      2状态速度信号z2;
      3状态加速度信号z3?
      z1,z2与TD得到的信号x1,x2做差,经过非线性函数映射，再乘beta
      得到未加入扰动补偿的u
      *********/
      ESO_ADRC(fhan_Input);//加速度会漂移，所以没用z3
    /*自抗扰控制器第三步*/
      /********状态误差反馈***/
      fhan_Input->e0+=fhan_Input->e1*fhan_Input->h;//状态积分项
      fhan_Input->e1=fhan_Input->x1-fhan_Input->z1;//状态偏差项
      fhan_Input->e2=fhan_Input->x2-fhan_Input->z2;//状态微分项
      /********线性组合*******/
     /*
      fhan_Input->u0=//fhan_Input->beta_0*fhan_Input->e0
                    +fhan_Input->beta_1*fhan_Input->e1
                    +fhan_Input->beta_2*fhan_Input->e2;
     */
      Nolinear_Conbination_ADRC(fhan_Input);
      /**********扰动补偿*******/
//      fhan_Input->u=fhan_Input->u0
//                   -fhan_Input->z3/fhan_Input->b0;
      //陀螺仪漂移，未加入扰动补偿
      fhan_Input->u=Func_Limit(fhan_Input->u0,300,-300);
}


