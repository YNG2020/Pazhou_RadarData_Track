# 项目说明

## 预测性能

## 算法
算法流程图见下：
<img src="https://github.com/YNG2020/Pazhou_RadarData_Track/blob/main/%E7%AE%97%E6%B3%95%E6%B5%81%E7%A8%8B.svg?sanitize=true" width="1440px">
### 整体思路介绍
整体思路就是先做标定，拿到雷达坐标系的原点的经纬度数据，以及利用3车道的标定数据，确定4车道的边界所在。然后再进行目标跟踪和识别。这一部分在前文已经通过算法流程图说得十分详细了。算法伪代码如下：<br>
开始;<br>
通过标定数据获取雷达坐标系原点的经纬度信息;<br>
将全体雷达数据按记录的时间划分为n_Gap帧;<br>
初始化一些参数与用于记录追踪情况的向量tracer_buffer;<br>
  while(当前处理的帧cnt < n_Gap?) {<br>
    从雷达数据中提取帧cnt的全体数据curFrameAllData;<br>
    创建OKIndex;<br>
    if (数据位置在四车道内 && RCS > RCSMin?) {<br>
      将对应下标push到OKIndex中;<br>
    }<br>
    对curFrameAllDat中的下标在OKIndex中的数据，<br>
    按照速度，纵向距离，横向距离，RCS升序排序，<br>
    得到curFrameData;<br>
    while(在tracer_buffer中有目标未被处理?) {<br>
      if (在curFrameData找到能与目标相匹配的数据?) {<br>
        在curFrameData中拿走对应数据点;<br>
        在进行卡夫曼滤波后记录结果;<br>
      }<br>
      else {<br>
        在对应的变量上的连续追踪失败次数+1;<br>
        if (连续追踪失败次数>maxFailTime?) {<br>
          将目标从追踪队列内移除;<br>
        }<br>
        else {<br>
          保留目标在追踪队列内;<br>
          连续追踪失败次数+1;<br>
        }<br>
      }<br>
      处理目标数+1;<br>
    }<br>
    while (在得到curFrameData中有数据未被处理?) {<br>
      if (数据的速度不等于0?) {<br>
        if (在剩余的数据中，能找到多个速度相近，且位置接近的数据?) {<br>
          在curFrameData中拿走对应数据点;<br>
          用这些数据构成一个追踪目标;<br>
          在进行卡夫曼滤波后记录结果;<br>
        }<br>
      }<br>
      处理数据数+1;<br>
    }<br>
  }<br>
while (有结果未输出?) {<br>
  if (该结果对应的车辆被记录追踪的次数>1?)<br>
    逐条输出结果;<br>
}<br>
结束;<br>

### 方法的创新点
算法的主体部分都只是用了一些直观和常用的算法，但是由于这一问题存在许多超参数，因此我们在线下创建了许多评价指标，如平均目标连续追踪长度等，让我们能有方向地进行参数调整。

### 算法的其他细节

## 其他注意事项
