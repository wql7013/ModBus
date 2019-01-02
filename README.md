# ModBus
Library for Modbus, master/slave, ASCII/RTU, multi-platform, C language


#### ModBus 主/从机协议

   - 支持 ASCII / RTU 两种模式

   - 可以创建多个实例

   - 读写寄存器使用非堵塞式, 通过绑定回调函数获取结果

#### 使用方法:

##### 主机

   1. 调用ModBus_setup配置

   2. 在串口接收中断函数中调用ModBus_readByteFromOuter

   3. 循环调用ModBus_Master_loop

   4. 读写从机寄存器

      - 调用ModBus_getRegister读目标设备寄存器值

      - 调用ModBus_setRegister写目标设备单寄存器

      - 调用ModBus_setRegisters写目标设备多寄存器

##### 从机

   1. 调用ModBus_setup配置参数

   2. 调用ModBus_attachRegisterHandler绑定获取和设置寄存器的函数

   3. 在loop中调用ModBus_Slave_loop

##### 函数形参看头文件对外接口部分
