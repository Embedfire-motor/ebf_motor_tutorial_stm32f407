.. vim: syntax=rst

直流电机位置环控制实现
==========================================

位置控制模式一般是通过编码器产生的脉冲的个数来确定转动的角度或者是转的圈数，
由于位置模式可以对速度和位置都有很严格的控制，
所以一般应用于定位装置。应用领域如数控机床、印刷机械等等。

本章通过我们前面学习的位置式PID和增量式PID两种控制方式分别来实现位置环的控制，
如果还不知道什么是位置式PID和增量式PID，请务必先学习前面PID算法的通俗解说这一章节。

硬件设计
--------------

关于详细的硬件分析在直流有刷电机和编码器的使用章节中已经讲解过，这里不再做分析，
如有不明白请参考前面章节，这里只给出接线表。

L298N驱动板
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: 电机与L298N驱动板连接
    :widths: 40 40
    :header-rows: 1

    * - 电机
      - L298N驱动板
    * - M+
      - 电机输出：1
    * - M-
      - 电机输出：2

.. list-table:: 电机与主控板连接
    :widths: 40 40
    :header-rows: 1

    * - 电机
      - 主控板
    * - 5V
      - VENC
    * - GND
      - GND
    * - A
      - PC6
    * - B
      - PC7

主控板上的J27需要用跳冒将VENC连接到5V。

.. list-table:: L298N驱动板与主控板连接
    :widths: 40 40
    :header-rows: 1

    * - L298N驱动板
      - 主控板
    * - PWM1
      - PA9
    * - PWM2
      - PA8
    * - GND
      - GND
    * - ENA
      - PG12

在L298N驱动板与主控板连接中，ENA可以不接PG12，使用跳冒连接到5V。

MOS管搭建驱动板
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: 电机与MOS管搭建驱动板连接
    :widths: 20 20
    :header-rows: 1

    * - 电机
      - MOS管搭建驱动板
    * - M+
      - M+
    * - 5V
      - 编码器电源：+
    * - GND
      - 编码器电源：-
    * - A
      - A
    * - B
      - B
    * - M-
      - M-

.. list-table:: MOS管搭建驱动板与主控板连接
    :widths: 20 20
    :header-rows: 1

    * - MOS管搭建驱动板
      - 主控板
    * - PWM1
      - PA9
    * - PWM2
      - PA8
    * - SD
      - PG12
    * - A
      - PC6
    * - B
      - PC7
    * - 电源输入：5V
      - 5V
    * - 电源输入：GND
      - GND

推荐使用配套的牛角排线直接连接驱动板和主控板。

直流电机位置环控制-位置式PID实现
------------------------------------------

软件设计
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

这里只讲解核心的部分代码，有些变量的设置，头文件的包含等并没有涉及到，
还有一些在前章节章节分析过的代码在这里也不在重复讲解，完整的代码请参考本章配套的工程。

编程要点
"""""""""""""""""""""""""""""""""

(1) 配置定时器可以输出PWM控制电机
(2) 配置定时器可以读取编码器的计数值
(3) 配置基本定时器可以产生定时中断来执行PID运算
(4) 编写位置式PID算法
(5) 编写速度控制函数
(6) 增加上位机曲线观察相关代码
(7) 编写按键控制代码

软件分析
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

在编程要点中的1和2在前章节已经讲解过，这里就不在讲解，如果不明白请先学习前面相关章节的内容。
这里主要分析PID算法的控制实现部分。

.. code-block:: c
   :caption: bsp_basic_tim.h-宏定义
   :linenos:

    #define BASIC_TIM           		  TIM6
    #define BASIC_TIM_CLK_ENABLE()   	__TIM6_CLK_ENABLE()

    #define BASIC_TIM_IRQn				    TIM6_DAC_IRQn
    #define BASIC_TIM_IRQHandler    	TIM6_DAC_IRQHandler

    /* 累计 TIM_Period个后产生一个更新或者中断*/		
      //当定时器从0计数到BASIC_PERIOD_COUNT-1，即为BASIC_PERIOD_COUNT次，为一个定时周期
    #define BASIC_PERIOD_COUNT    (50*50)

    //定时器时钟源TIMxCLK = 2 * PCLK1  
    //				PCLK1 = HCLK / 4 
    //				=> TIMxCLK=HCLK/2=SystemCoreClock/2=84MHz
    #define BASIC_PRESCALER_COUNT   (1680)

    /* 获取定时器的周期，单位ms */
    //#define __HAL_TIM_GET_PRESCALER(__HANDLE__)      ((__HANDLE__)->Instance->PSC)    // Get TIM Prescaler.
    //#define GET_BASIC_TIM_PERIOD(__HANDLE__)    (1.0/(HAL_RCC_GetPCLK2Freq()/(__HAL_TIM_GET_PRESCALER(__HANDLE__)+1)/(__HAL_TIM_GET_AUTORELOAD(__HANDLE__)+1))*1000)

    /* 以下两宏仅适用于定时器时钟源TIMxCLK=84MHz，预分频器为：1680-1 的情况 */
    #define SET_BASIC_TIM_PERIOD(T)     __HAL_TIM_SET_AUTORELOAD(&TIM_TimeBaseStructure, (T)*50 - 1)    // 设置定时器的周期（1~1000ms）
    #define GET_BASIC_TIM_PERIOD()      ((__HAL_TIM_GET_AUTORELOAD(&TIM_TimeBaseStructure)+1)/50.0)     // 获取定时器的周期，单位ms

这里封装了定时器的一些相关的宏，使用宏定义非常方便程序升级、移植。使用SET_BASIC_TIM_PERIOD(T)这个宏可以设置定时器的周期，
这样可以通过按键或者上位机来设置这个定时器的中断周期，使用GET_BASIC_TIM_PERIOD()这个宏可以得到定时器的当前周期，
不过使用的两个宏是有要求的，需要定时器时钟源的频率是84MHz，且预分频系数为1680。
如果更换定时器和修改预分频器则需要重新计算这个宏里面的参数.我们来看一下当前宏中周期的计算:84000000/1680/50 = 1000,
84000000为时钟源的频率，1680为预分频系数，50为自动重装载值，1000为定时器产生更新中断的频率，
当定时器也(84000000/1680)Hz的频率计数到50时刚好是1ms，所以只要设置自动重装载值为50的n倍减一时，
就可以得到n毫秒的更新中断，注意n是1到1000的正整数。

.. code-block:: c
   :caption: bsp_basic_tim.c-定时器配置函数
   :linenos:

    static void TIM_Mode_Config(void)
    {
      // 开启TIMx_CLK,x[6,7] 
      BASIC_TIM_CLK_ENABLE(); 

      TIM_TimeBaseStructure.Instance = BASIC_TIM;
      /* 累计 TIM_Period个后产生一个更新或者中断*/		
      //当定时器从0计数到BASIC_PERIOD_COUNT-1，即为BASIC_PERIOD_COUNT次，为一个定时周期
      TIM_TimeBaseStructure.Init.Period = BASIC_PERIOD_COUNT - 1;       

      //定时器时钟源TIMxCLK = 2 * PCLK1  
      //				PCLK1 = HCLK / 4 
      //				=> TIMxCLK=HCLK/2=SystemCoreClock/2=84MHz
      // 设定定时器频率为=TIMxCLK/BASIC_PRESCALER_COUNT
      TIM_TimeBaseStructure.Init.Prescaler = BASIC_PRESCALER_COUNT - 1;	
      TIM_TimeBaseStructure.Init.CounterMode = TIM_COUNTERMODE_UP;           // 向上计数
      TIM_TimeBaseStructure.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;     // 时钟分频

      // 初始化定时器TIMx, x[2,3,4,5]
      HAL_TIM_Base_Init(&TIM_TimeBaseStructure);

      // 开启定时器更新中断
      HAL_TIM_Base_Start_IT(&TIM_TimeBaseStructure);	
    }

首先定义两个定时器初始化结构体，定时器模式配置函数主要就是对这两个结构体的成员进行初始化，
然后通过调用的初始化函数HAL_TIM_Base_Init()把这些参数写入定时器的寄存器中。
有关结构体的成员介绍请参考定时器详解章节。
最后通过调用函数HAL_TIM_Base_Start_IT()使能定时器的更新中断。

.. code-block:: c
   :caption: bsp_basic_tim.c-定时器初始
   :linenos:

    void TIMx_Configuration(void)
    {
      TIMx_NVIC_Configuration();	
      
      TIM_Mode_Config();
      
    #if defined(PID_ASSISTANT_EN)
      uint32_t temp = GET_BASIC_TIM_PERIOD();     // 计算周期，单位ms
      
      set_computer_value(SEED_PERIOD_CMD, CURVES_CH1, &temp, 1);     // 给通道 1 发送目标值
    #endif

    }

该函数主要配置了定时器的中断设置和定时器模式配置，最后调用set_computer_value()函数设置了上位机的周期值，
这里只是同步一下上位机显示的周期值。PID_ASSISTANT_EN是用于选择是否使用上位机的宏，
当我们在调试阶段时可以定义这个宏，方便使用上位机（野火调试助手-PID调试助手）来观察电机的运行效果，
在完成调试后我们可以直接不定义这个宏，这样就去掉了上位机相关部分。

.. code-block:: c
   :caption: stm32f4xx_it.c-定时器更新中断回调函数
   :linenos:

    void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
    {
      if(htim==(&TIM_EncoderHandle))
      {
        /* 判断当前计数器计数方向 */
        if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&TIM_EncoderHandle))
          /* 下溢 */
          Encoder_Overflow_Count--;
        else
          /* 上溢 */
          Encoder_Overflow_Count++;
      }
      else if(htim==(&TIM_TimeBaseStructure))
      {
        motor_pid_control();
      }
    }

其中当htim=(&TIM_EncoderHandle)时是编码器定时器计数器溢出，当htim=(&TIM_TimeBaseStructure)时是基本定时器，
在这里调用motor_pid_control()进行PID的周期性的控制。
