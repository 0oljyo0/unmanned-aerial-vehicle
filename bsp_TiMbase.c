
// �߼���ʱ��TIMx,x[6,7]��ʱ��ʼ������
#include "bsp_TiMbase.h" 

/// TIMx,x[1,8]�ж����ȼ�����
void TIMx_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
    NVIC_InitStructure.NVIC_IRQChannel = macTIM_IRQ2;	  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*
 * TIM_Period / Auto Reload Register(ARR) = 1000   TIM_Prescaler--71 
 * �ж�����Ϊ = 1/(72MHZ /72) * 1000 = 1ms
 *
 * TIMxCLK/CK_PSC --> TIMxCNT --> TIM_Period(ARR) --> �ж� ��TIMxCNT����Ϊ0���¼��� 
 */

/*
 * ע�⣺TIM_TimeBaseInitTypeDef�ṹ��������5����Ա��TIM6��TIM7�ļĴ�������ֻ��
 * TIM_Prescaler��TIM_Period������ʹ��TIM6��TIM7��ʱ��ֻ���ʼ����������Ա���ɣ�
 * ����������Ա��ͨ�ö�ʱ���͸߼���ʱ������.
 *-----------------------------------------------------------------------------
 * TIM_Prescaler         ����
 * TIM_CounterMode			 TIMx,x[6,7]û�У���������
 * TIM_Period            ����
 * TIM_ClockDivision     TIMx,x[6,7]û�У���������
 * TIM_RepetitionCounter TIMx,x[1,8,15,16,17]����
 *-----------------------------------------------------------------------------
 */
void TIMx_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		
		// ����TIMx_CLK,x[1,8],���ڲ�ʱ��CK_INT=72M
    macTIM_APBxClock_FUN2  (macTIM_CLK2, ENABLE);
	
		// �Զ���װ�ؼĴ����ܵ�ֵ(����ֵ)
    TIM_TimeBaseStructure.TIM_Period=100;
	
    // �ۼ� TIM_Period��Ƶ�ʺ����һ�����»����ж�
	  // ʱ��Ԥ��Ƶ��Ϊ71����������������ʱ��CK_CNT = CK_INT / (71+1)=1M
    TIM_TimeBaseStructure.TIM_Prescaler= 17;
	
		// ʱ�ӷ�Ƶ���� ��û�õ����ù�
    //TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
		
		// ����������ģʽ������Ϊ���ϼ���
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
		
		// �ظ���������ֵ��û�õ����ù�
		//TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
	
	  // ��ʼ����ʱ��TIMx, x[1,8]
    TIM_TimeBaseInit(macTIMx2, &TIM_TimeBaseStructure);
		
		// ����������жϱ�־λ
    TIM_ClearFlag(macTIMx2, TIM_FLAG_Update);
	  
		// �����������ж�
    TIM_ITConfig(macTIMx2,TIM_IT_Update,ENABLE);
		
		// ʹ�ܼ�����
    TIM_Cmd(macTIMx2, ENABLE);																		
    
		// ��ʱ�ر�TIMx,x[1,8]��ʱ�ӣ��ȴ�ʹ��
    macTIM_APBxClock_FUN2  (macTIM_CLK2, DISABLE);   
}
/*********************************************END OF FILE**********************/
