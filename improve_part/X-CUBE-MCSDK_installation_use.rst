.. vim: syntax=rst

X-CUBE-MCSDK 软件获取与安装
==========================================

X-CUBE-MCSDK是ST（意法半导体）推出的STM32电机控制软件开发套件（MCSDK），使用该软件对电机进行配置后，
可以直接生成源码工程，简化了开发过程，软件共有两种版本：X-CUBE-MCSDK和X-CUBE-MCSDK-FUL，
其中X-CUBE-MCSDK在一些核心的算法使用lib的形式提供给用户，X-CUBE-MCSDK-FUL则是完整的源码，
两个软件可以在ST官网搜索MC SDK下载，不过其中X-CUBE-MCSDK-FUL在注册并获得批准后方可下载，
X-CUBE-MCSDK则不需要批准可以直接下载。也可以在我们提供的资料中下载。

下载后可以得到名为：X-CUBE-MCSDK-FUL_5.4.4.exe的安装包，双击安装即可，
安装过程很简单这里就不在截图说明。安装完成后会在桌面生成MotorControl Workbench 5.4.4
和 Motor Profiler 5.4.4两个软件，Motor Profiler 5.4.4是用于自动测量电机参数的软件，
不过使用该软件需要使用ST相关的主板和电机驱动板才可以，这里就不介绍该软件的时候方法。
MotorControl Workbench 5.4.4才是我们需要的软件，可以使用该软件配置电机驱动板等参数后就可以生成源代码。

在安装好X-CUBE-MCSDK-FUL后我们还需要安装STM32CubeMX，因为X-CUBE-MCSDK-FUL在配置完成后