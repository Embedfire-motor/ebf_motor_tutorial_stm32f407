.. vim: syntax=rst

直流有刷电机
==========================================

直流有刷电机具有控制简单、成本低的特点，但在能够满足必要的性能，低成本和足够的可靠性的前提下
，直流有刷电机是一个很好的选择。如便宜的玩具以及一些功能简单的应用场合，如汽车的电动座椅等。
在使用MOSFET/IGBT开关就可以对电机进行控制，所以整个电机控制系统相当便宜。就可以让电极提供足够好的性能。
此外，基本的BDC电机在电源和电机之间只需要两根电缆，这样就可以节省配线和连接器所需的空间，
并降低电缆和连接器的成本。

直流电机转速快，扭矩小，所以用减速来降低转速，提高扭矩，已就是直流减速电机，实物图见下图，
即齿轮减速电机，是在普通直流电机的基础上，加上配套齿轮减速箱。齿轮箱不同的减速比可以提供不同
的转速和力矩。在实际使用中减速电机使用的最为广泛，所以本章节将主要介绍直流有刷减速电机。

.. image:: ../media/dc_gear_motor.jpg
   :align: center
   :alt: 减速电机实物图

本章节将介绍直流有刷电机的工作原理、电机参数和驱动电路，最后通过实验来实现电机远动的简单控制。


直流有刷电机工作原理
------------------------------------------

在分析原理前我们先复习一下左手定则，如下图所示。

.. image:: ../media/left-hand_rule.jpg
   :align: center
   :alt: 左手定则

左手定则是判断通电导体处于磁场中时，所受安培力 F (或运动)的方向、磁感应强度B的方向以及通电导
体棒的电流I三者方向之间的关系的定律。通过左手定则可以知道通电导体在磁场中的受力方向。判断方法是：
伸开左手，使拇指与其他四指垂直且在一个平面内，让磁感线从手心流入，四指指向电流方向，
大拇指指向的就是安培力方向（即导体受力方向）。
   
有刷直流电机在其电枢上绕有大量的线圈，所产生强大的磁场与外部磁场相互作用产生旋转运动。
磁极方向的跳转是通过移动固定位置的接触点来完成的，该接触点在电机转子上与电触点相对连接。
这种固定触点通常由石墨制成，与铜或其他金属相比，在大电流短路或断路/起动过程中石墨不会
熔断或者与旋转触点焊接到一起，并且这个触点通常是弹簧承载的，所以能够获得持续的接触压力。
在这里我们将通过其中一组线圈和一对磁极来分析其工作原理，如下图所示。

.. image:: ../media/motor_working_principle.png
   :align: center
   :alt: 有刷电机工作原理图

图中C和D两片半圆周的铜片构成换向器，两个弹性铜片靠在换向器两侧的A和B是电刷，电源通过电刷向导线框供电，
线框中就有电流通过，在线框两侧放一对磁极N和S，形成磁场，磁力线由N到S。线框通有电流时，两侧导线就
会受到磁场的作用力，方向依左手定则判断，红色和蓝色线框部分分别会受到力F\ :sub:`1`\和F\ :sub:`2`\，
这两个力的方向相反，这使得线框会转动，当线框转过90°时，换向器改变了线框电流的方向，产生的安培力方向不变，
于是导线框会连续旋转下去，这就是直流电动机的工作原理。

直流有刷减速电机几个重要参数
------------------------------------------

- 空载转速：正常工作电压下电机不带任何负载的转速（单位为r/min（转/分））。
  空载转速由于没有反向力矩，所以输出功率和堵转情况不一样，该参数只是提供一个电机在规定
  电压下最大转速的作用。
- 空载电流：正常工作电压下电机不带任何负载的工作电流（单位mA（毫安））。越好的电机，在空载时，该值越小。
- 负载转速：正常工作电压下电机带负载的转速。
- 负载力矩：正常工作电压下电机带负载的力矩 （N·m（牛米））。
- 负载电流：负载电流是指电机拖动负载时实际检测到的定子电流数值。
- 堵转力矩：在电机受反向外力使其停止转动时的力矩。如果电机堵转现象经常出现，
  则会损坏电机，或烧坏驱动芯片。所以大家选电机时，这是除转速外要考虑的参数。
  堵转时间一长，电机温度上升的很快，这个值也会下降的很厉害。
- 堵转电流：在电机受反向外力使其停止转动时的电流，此时电流非常大，时间稍微就可能会烧毁电机，
  在实际使用时应尽量避免。
- 减速比：是指没有减速齿轮时转速与有减速齿轮时转速之比。
- 功率：般指的是它的额定功率（单位W（瓦）），即在额定电压下能够长期正常运转的最大功率，
  也是指电动机在制造厂所规定的额定情况下运行时, 其输出端的机械功率。


直流有刷电机驱动设计与分析
------------------------------------------

我们先来想一个问题，假设你手里现在有一个直流电机和一节电池，当你把电机的两根电源
线和电池的电源连接在一起时，这时电机可以正常旋转，当想要电机反向旋转时，只需要把两根电
源线交换一下就可以了。但是当我们实际应用中如果也想实现正转和反转的控制也需要交换电源线吗？
显然这样的方法是不可行的。这时候我们可以用一个叫做“H桥电路”来驱动电机。

控制电路原理设计与分析
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

如下图所示，是使用4个三极管搭建的H桥电路。

.. image:: ../media/H-bridge_circuit_stop.png
   :align: center
   :alt: 三极管搭建H桥电路图

上图中，H桥式电机驱动电路包括4个三极管和一个电机。要使电机运转，必须导通对角线上的一对三极管。
根据不同三极管对的导通情况，电流可能会从左至右或从右至左流过电机，从而控制电机的转向。

.. image:: ../media/H-bridge_circuit_CW.png
   :align: center
   :alt: 三极管搭建H桥顺时针转动

上图中，当Q\ :sub:`1`\和Q\ :sub:`4`\导通时，电流将经过Q\ :sub:`1`\从左往右流过电机，
在经过Q\ :sub:`4`\流到电源负极，这时图中电机可以顺时针转动。

.. image:: ../media/H-bridge_circuit_CCW.png
   :align: center
   :alt: 三极管搭建H桥逆时针转动

上图中，当Q\ :sub:`3`\和Q\ :sub:`2`\导通时，电流将经过Q\ :sub:`3`\从右往左流过电机，
在经过Q\ :sub:`2`\流到电源负极，这时图中电机可以逆时针转动。

当Q\ :sub:`1`\和\ :sub:`2`\同时导通时，电流将从电源先后经过Q\ :sub:`1`\和Q\ :sub:`2`\，
然后直接流到电源负极，在这个回路中除了三极管以外就没有其他负载，这时电流可能会达到最大值，此时可能会烧毁
三极管，同理，当Q\ :sub:`3`\和\ :sub:`4`\同时导通时，也会出现相同的状况。这样的情况肯定是不能发生的，
但是我们写程序又是三分写代码七分在调试，这就难免会有写错代码将同一测得三极管导通的情况，为此我们就需要
从硬件上来避免这个问题。下面电路图是改进后的驱动电路图。

.. image:: ../media/H-bridge_circuit_Improve.png
   :align: center
   :alt: 三极管搭建H桥改进电路

**根据驱动芯片来讲解**

驱动芯片与驱动电机设计与分析
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

直流有刷减速电机控制实现
-----------------------------------

速度控制原理
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

脉冲宽度调制（Pulse width modulation，PWM）信号，即PWM是一种按一定的规则对
各脉冲的宽度进行调制，既可改变电路输出电压的大小，也可改变输出频率。PWM通过一定的频率
来改变通电和断电的时间，从而控制电路输出功率，在电机控制上：当“通电”时间相对于“断电”时间长时，
电机旋转速度快，当“通电”时间相对于“断电时间”短时，电机旋转速度慢。其中，
通电时间/(通断时间+断电时间)=占空比，即，高电平占整个周期的百分比，如下图所示：

.. image:: ../media/pwm_explain.png
   :align: center
   :alt: PWM详解

上图中：T\ :sub:`1`\为高电平时间，T\ :sub:`2`\为低电平时间，T是周期。

D(占空比) = T\ :sub:`1`\/T*100%

设电机的速度为V，最大速度为V\ :sub:`max`\。

则：V=V\ :sub:`max`\*D

当占空比D（0≤D≤1）的大小改变时，速度V也会改变，所以只要改变占空比就能达到控制的目的。

硬件设计
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

软件设计
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

这里只讲解核心的部分代码，有些变量的设置，头文件的包含等并没有涉及到，完整的代码请
参考本章配套的工程。我们创建了四个文件：bsp_general_tim.c、bsp_general_tim.h、
bsp_motor_control.c和bsp_motor_control.h文件用来存定时器驱动和电机控制程序及相关宏定义

编程要点
"""""""""""""""""

(1) 定时器 IO 配置

(2) 定时器时基结构体TIM_TimeBaseInitTypeDef配置

(3) 定时器输出比较结构体TIM_OCInitTypeDef配置

(4) 根据定时器定义电机控制相关函数

.. code-block:: c
   :caption: bsp_general_tim.h-宏定义
   :linenos:

    /*宏定义*/
    #define PWM_TIM                        	TIM1
    #define PWM_TIM_GPIO_AF                 GPIO_AF1_TIM1
    #define PWM_TIM_CLK_ENABLE()  					__TIM1_CLK_ENABLE()

    #define PWM_CHANNEL_1                   TIM_CHANNEL_1
    #define PWM_CHANNEL_2                   TIM_CHANNEL_2

    /* 累计 TIM_Period个后产生一个更新或者中断*/		
    /* 当定时器从0计数到PWM_PERIOD_COUNT，即为PWM_PERIOD_COUNT+1次，为一个定时周期 */
    #define PWM_PERIOD_COUNT     (1000)

    /* 通用控制定时器时钟源TIMxCLK = HCLK=168MHz */
    /* 设定定时器频率为=TIMxCLK/(PWM_PRESCALER_COUNT+1) */
    #define PWM_PRESCALER_COUNT     (9)

    /*PWM引脚*/
    #define PWM_TIM_CH1_GPIO_PORT           GPIOA
    #define PWM_TIM_CH1_PIN                 GPIO_PIN_8

    #define PWM_TIM_CH2_GPIO_PORT           GPIOA
    #define PWM_TIM_CH2_PIN                 GPIO_PIN_9

    #define PWM_TIM_CH3_GPIO_PORT           GPIOA
    #define PWM_TIM_CH3_PIN                 GPIO_PIN_10

使用宏定义非常方便程序升级、移植。如果使用不同的定时器IO，修改这些宏即可。

定时器复用功能引脚初始化

.. code-block:: c
   :caption: 定时器复用功能引脚初始化
   :linenos:

    static void TIMx_GPIO_Config(void) 
    {
    GPIO_InitTypeDef GPIO_InitStruct;
      
      /* 定时器通道功能引脚端口时钟使能 */
      
      __HAL_RCC_GPIOA_CLK_ENABLE();
      __HAL_RCC_GPIOA_CLK_ENABLE();
      
      /* 定时器通道1功能引脚IO初始化 */
      /*设置输出类型*/
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      /*设置引脚速率 */ 
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
      /*设置复用*/
      GPIO_InitStruct.Alternate = PWM_TIM_GPIO_AF;
      
      /*选择要控制的GPIO引脚*/	
      GPIO_InitStruct.Pin = PWM_TIM_CH1_PIN;
      /*调用库函数，使用上面配置的GPIO_InitStructure初始化GPIO*/
      HAL_GPIO_Init(PWM_TIM_CH1_GPIO_PORT, &GPIO_InitStruct);

      GPIO_InitStruct.Pin = PWM_TIM_CH2_PIN;	
      HAL_GPIO_Init(PWM_TIM_CH2_GPIO_PORT, &GPIO_InitStruct);
      
    }

定时器通道引脚使用之前必须设定相关参数，这选择复用功能，并指定到对应的定时器。
使用GPIO之前都必须开启相应端口时钟。

.. code-block:: c
   :caption: 定时器模式配置
   :linenos:

    TIM_HandleTypeDef  TIM_TimeBaseStructure;
    static void TIM_PWMOUTPUT_Config(void)
    {
      TIM_OC_InitTypeDef  TIM_OCInitStructure;  
      
      /*使能定时器*/
      PWM_TIM_CLK_ENABLE();
      
      TIM_TimeBaseStructure.Instance = PWM_TIM;
      /* 累计 TIM_Period个后产生一个更新或者中断*/		
      //当定时器从0计数到PWM_PERIOD_COUNT，即为PWM_PERIOD_COUNT+1次，为一个定时周期
      TIM_TimeBaseStructure.Init.Period = PWM_PERIOD_COUNT - 1;
      // 通用控制定时器时钟源TIMxCLK = HCLK/2=84MHz 
      // 设定定时器频率为=TIMxCLK/(PWM_PRESCALER_COUNT+1)
      TIM_TimeBaseStructure.Init.Prescaler = PWM_PRESCALER_COUNT - 1;	
      
      /*计数方式*/
      TIM_TimeBaseStructure.Init.CounterMode = TIM_COUNTERMODE_UP;
      /*采样时钟分频*/
      TIM_TimeBaseStructure.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
      /*初始化定时器*/
      HAL_TIM_PWM_Init(&TIM_TimeBaseStructure);
      
      /*PWM模式配置*/
      TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;
      TIM_OCInitStructure.Pulse = 0;
      TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_LOW;
      TIM_OCInitStructure.OCNPolarity = TIM_OCPOLARITY_LOW;
      TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_SET;
      TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_RESET;
      
      /*配置PWM通道*/
      HAL_TIM_PWM_ConfigChannel(&TIM_TimeBaseStructure, &TIM_OCInitStructure, PWM_CHANNEL_1);
      /*开始输出PWM*/
      HAL_TIM_PWM_Start(&TIM_TimeBaseStructure,PWM_CHANNEL_1);
      
      /*配置脉宽*/
      TIM_OCInitStructure.Pulse = PWM_PERIOD_COUNT/2;    // 默认占空比为50%
      /*配置PWM通道*/
      HAL_TIM_PWM_ConfigChannel(&TIM_TimeBaseStructure, &TIM_OCInitStructure, PWM_CHANNEL_2);
      /*开始输出PWM*/
      HAL_TIM_PWM_Start(&TIM_TimeBaseStructure,PWM_CHANNEL_2);
    }

首先定义两个定时器初始化结构体，定时器模式配置函数主要就是对这两个结构体的成员进行初始化，然后通过相
应的初始化函数把这些参数写入定时器的寄存器中。有关结构体的成员介绍请参考定时器详解章节。

不同的定时器可能对应不同的APB总线，在使能定时器时钟是必须特别注意。通用控制定时器属于APB1，
定时器内部时钟是84MHz。

在时基结构体中我们设置定时器周期参数为PWM_PERIOD_COUNT（5599），频率为15KHz，使用向上计数方式。
因为我们使用的是内部时钟，所以外部时钟采样分频成员不需要设置，重复计数器我们没用到，也不需要设置。

在输出比较结构体中，设置输出模式为PWM1模式，通道输出高电平有效，设置脉宽为ChannelPulse，
ChannelPulse是我们定义的一个无符号16位整形的全局变量，用来指定占空比大小，实际上脉宽就是设定比较寄
存器CCR的值，用于跟计数器CNT的值比较。

最后使用HAL_TIM_PWM_Start函数让计数器开始计数和通道输出。

.. code-block:: c
   :caption: bsp_motor_control.h-电机方向控制枚举
   :linenos:

    /* 电机方向控制枚举 */
    typedef enum
    {
      MOTOR_FWD = 0,
      MOTOR_REV,
    }motor_dir_t;

在这里枚举了两个变量，用于控制电机的正转与反转。**注意**：在这里并不规定什么方向是正转与反转，这个
是你自己定义的。

.. code-block:: c
   :caption: 变量定义
   :linenos:

    /* 私有变量 */
    static motor_dir_t direction  = MOTOR_FWD;     // 记录方向
    static uint16_t    dutyfactor = 0;             // 记录占空比

定义两个私有变量，direction用于记录电机旋转方向，dutyfactor用于记录当前设置的占空比。

.. code-block:: c
   :caption: 定时器到电机控制的宏接口
   :linenos:
   
    /* 设置速度（占空比） */
    #define SET_FWD_COMPAER(ChannelPulse)     TIM1_SetPWM_pulse(PWM_CHANNEL_1,ChannelPulse)    // 设置比较寄存器的值
    #define SET_REV_COMPAER(ChannelPulse)     TIM1_SetPWM_pulse(PWM_CHANNEL_2,ChannelPulse)    // 设置比较寄存器的值

    /* 使能输出 */
    #define MOTOR_FWD_ENABLE()      HAL_TIM_PWM_Start(&TIM_TimeBaseStructure,PWM_CHANNEL_1);    // 使能 PWM 通道 1
    #define MOTOR_REV_ENABLE()      HAL_TIM_PWM_Start(&TIM_TimeBaseStructure,PWM_CHANNEL_2);    // 使能 PWM 通道 2

    /* 禁用输出 */
    #define MOTOR_FWD_DISABLE()     HAL_TIM_PWM_Stop(&TIM_TimeBaseStructure,PWM_CHANNEL_1);     // 禁用 PWM 通道 1
    #define MOTOR_REV_DISABLE()     HAL_TIM_PWM_Stop(&TIM_TimeBaseStructure,PWM_CHANNEL_2);     // 禁用 PWM 通道 2

使用宏定义非常方便程序升级、移植。如果使用不同的定时器IO，修改这些宏即可。

.. code-block:: c
   :caption: 设置电机速度
   :linenos:

    void set_motor_speed(uint16_t v)
    {
      dutyfactor = v;
      
      if (direction == MOTOR_FWD)
      {
        SET_FWD_COMPAER(dutyfactor);     // 设置速度
      }
      else
      {
        SET_REV_COMPAER(dutyfactor);     // 设置速度
      }
    }

根据电机的旋转方向来设置电机的速度（占空比），并记录下设置的占空比，方便在切换旋转
方向时设置另一路为相同的占空比。

.. code-block:: c
   :caption: 设置电机方向
   :linenos:

    void set_motor_direction(motor_dir_t dir)
    {
      direction = dir;
      
      if (direction == MOTOR_FWD)
      {
        SET_FWD_COMPAER(dutyfactor);     // 设置速度
        SET_REV_COMPAER(0);              // 设置占空比为 0
      }
      
      else
      {
        SET_FWD_COMPAER(0);              // 设置速度
        SET_REV_COMPAER(dutyfactor);     // 设置占空比为 0
      }
    }

将一路PWM的占空比设置为0，另一路用于设置速度。

.. code-block:: c
   :caption: main
   :linenos:

    int main(void) 
    {
      __IO uint16_t ChannelPulse = 0;
      uint8_t i = 0;
      
      /* 初始化系统时钟为168MHz */
      SystemClock_Config();
      
      /* 初始化按键GPIO */
      Key_GPIO_Config();

      /* 通用定时器初始化并配置PWM输出功能 */
      TIMx_Configuration();
      
      TIM1_SetPWM_pulse(PWM_CHANNEL_1,0);
      TIM1_SetPWM_pulse(PWM_CHANNEL_2,0);
      
      while(1)
      {
        /* 扫描KEY1 */
        if( Key_Scan(KEY1_GPIO_PORT, KEY1_PIN) == KEY_ON)
        {
          /* 增大占空比 */
          ChannelPulse += 50;
          
          if(ChannelPulse > PWM_PERIOD_COUNT)
            ChannelPulse = PWM_PERIOD_COUNT;
          
          set_motor_speed(ChannelPulse);
        }
        
        /* 扫描KEY2 */
        if( Key_Scan(KEY2_GPIO_PORT, KEY2_PIN) == KEY_ON)
        {
          if(ChannelPulse < 50)
            ChannelPulse = 0;
          else
            ChannelPulse -= 50;
          
          set_motor_speed(ChannelPulse);
        }
        
        /* 扫描KEY3 */
        if( Key_Scan(KEY3_GPIO_PORT, KEY3_PIN) == KEY_ON)
        {
          /* 转换方向 */
          set_motor_direction( (++i % 2) ? MOTOR_FWD : MOTOR_REV);
        }
      }
    }

首先初始化系统时钟，然后初始化定时器和按键，将占空比设置为0，即电机默认不转动。
在死循环里面扫描按键，KEY1按键按下增加速度（占空比），KEY2按键按下减少速度（占空比），
KEY3按键按下切换电机旋转方向。

下载验证
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

如果有条件的话，这里我们先不连接电机，先通过示波器连接到开发板的PWM输出引脚上，通过示波器来观察PWM
的变化情况:

- 使用DAP连接开发板到电脑；
- 使用示波器的CH1连接到PA15，CH2连接到PB3，注意示波器要与开发板供地；
- 给开发板供电，编译下载配套源码，复位开发板。

上电后我们通过示波器可以观察到两个通道都是低电平，当按下KEY1时，可以增加CH1通道的占空比，如下图所示。

.. image:: ../media/dc_motor_duty_cycle1.jpg
   :align: center
   :alt: 示波器观察PWM输出情况

在上图中黄色波形为CH1通道，蓝色波形为CH2通道，按下一次KEY1后，周期设置为500，所以CH1的占空比为
500/5600*100%=9%。通过波形计算也与理论相符，这说明我们的PWM的配置是正确的，其中CH2通道的波形
一直为低电平。当CH1和CH2都为低电平时，电机停止转动。当CH1上的平均电压大于电机的启动电压后电机就
可以转动了，电源电压为12V，占空比为D,则平均电压为：12V*D。当按下KEY3后两通道输出相反，CH1一直为
低电平，CH2为PWM波，电机反向转动。

在确定PWM输出正确后我们就可以接上电机进行验证我们的程序了，实物连接如下图所示。

.. image:: ../media/dc_motor_key_control.jpg
   :align: center
   :alt: 电机连接实物图
