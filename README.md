# t3_robotV1_openCR
## V0.2
### 修改
* 新增刹车灯功能
* 可以通过sonar_msg查看下列内容：
|dacValue[0]|dacValue[1]|linValue[0]|linValue[1]|encoder[2]|encoder[3]|
|-|:-:|:-:|:-:|:-:|-:
|下发左DAC|下发右DAC|上位机下发左转速|上位机下发右转速|实际左转速|实际右转速|
|dacValue[2]|dacValue[3]|空|空|空|空|
|delta左自增DAC|delta右自增DAC|空|空|空|空|
