Solar System Animation 测试
---------------------------

## Desc



## 测试 Prompt


```markdown

创建一个符合以下规格的HTML5 Canvas太阳系动画：

0. **要求**
- 所有代码应在一个HTML文件中
- 使用canvas绘制动画
- 全屏显示动画

1. **天体要求**
- 包括：太阳，9颗行星
- 由于太阳过大, 因此所有星球只要展示相对大小即可, 比如太阳最大, 木星次之
- 行星按照距离太阳的远近排列, 从内到外依次是水星, 金星, 地球, 火星, 木星, 土星, 天王星, 海王星, 冥王星
- 注意各个星球的轨道间距要大于星球本身的直径，以免造成视觉上的重叠

2. **视觉设计**
- 每个星球使用4种颜色做像素填充，仅填充一次
  • 太阳：#f2831f, #f15d22, #d94125, #a41d22
  • 水星：#5a5856, #bfbdbc, #8c8a89, #f4f6f8
  • 金星：#868a8d, #d9b392, #f3dbc3, #414141
  • 地球：#1f386f, #0b1725, #386f61, #dac0a5
  • 火星：#dabd9e, #8c5d4b, #f27c5f, #c36d5c
  • 木星：#282411, #c08137, #bfb09c, #a6705b
  • 土星：#f3cf89, #dab778, #736b59, #c1a480
  • 天王星：#3f575a, #688a8c, #95bbbe, #cfecf0
  • 海王星：#647ba5, #7595bf, #4e5d73, #789ebf
  • 冥王星：#d8cbbb, #f4ebdc, #402a16, #a79f97
- 轨道线：半透明白色圆圈
- 标签：
  • 始终面向摄像机的星球文字标签
  • 格式：[星球名称]

3. **运动模拟**
- 时间压缩：1秒真实时间=10个地球日, 地球绕太阳一周为365个地球日
- 行星公转轨道使用圆形轨道即可
- 层次结构：
  • 所有行星围绕太阳运行

4. **技术实现**
- 使用requestAnimationFrame实现平滑动画

5. **性能优化**
- 离屏canvas用于静态元素（轨道线）
- 使用Web Workers进行位置计算

6. **计数器**
- 在左上角显示当前FPS指示器（FPS）和平均FPS指示器（AVG FPS），以及当前地球日计数 (Earth Day)
- 使用黑色文字和半透明白色圆角背景


注意，不要省略星球视觉设计的代码，我需要你实现全部的代码。
```


## 评分规则


## 测试结果


## 可视化结果


## 结论



## Winner

