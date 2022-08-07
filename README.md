# InnerRect
Find the max inner Rect in image

在Opencv中找到内接最大矩形。



实现方法：

1.处理图片位灰度图，

1，先找到图形中主方向的角度。通过仿射转换，主方向作为x轴。

2.在转换完的图形中找到灰度值位255的区域，为了处理效率，没有按照像素直接处理，而是将区域分为一个一个的小栅格区域。每个区域记录是否是有效值。

3.经过上述处理后，可以将255区域看做直方图处理，对每一行找最大的面积。

4.最后找到的最大面积记录四点即位最大内接矩形的四个点。

5.将坐标根据开始的仿射，再转换为原图的坐标。



测试图：

![image](https://github.com/RedCrab1/InnerRect/blob/main/test_image.png)

最后生成结果：

![image](https://github.com/RedCrab1/InnerRect/blob/main/InnerRect_image.png)