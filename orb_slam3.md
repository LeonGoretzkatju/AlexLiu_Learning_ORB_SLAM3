## ORB_SLAM3

## 1.ORB_SLAM3的创新点

1. ORB_SLAM3是第一个基于特征的紧耦合的VIO系统，仅依赖于最大后验估计(包括IMU在初始化时)代码部分对应于MLPNP部分，在IMU的初始化部分引入了map。

2. 系统根据改进召回的新的重定位模块来构建的混合地图，因为这个模块可以让ORB-SLAM3在特征点较少的场景中长期运行：当里程计失败的时候，系统会重新构建地图并将这个地图和原来构建的地图对齐。如果是原有的DBOW2的话需要匹配三个连续的关键帧，而ORB_SLAM3对候选的关键帧第一次就进行几何一致性检测，然后利用三个共视的关键帧进行局部的一致性检验，这种策略提升了召回率，并增强了数据关联，提高了地图准确性，但是这是以显著提高计算成本作为代价的，所以导致ORB_SLAM3在内存不足的情况下无法运行。

3. ORB_SLAM3是第一个能够在所有算法阶段重用所有先前信息的系统。这样的机制就可以在光束法平差的时候用有共视关系的关键帧，即使两帧在时间相差很远，或者来自原来的建图过程，精度有了很大提升。

4. ORB_SLAM3是第一个可以对短期、中期、长期数据进行数据关联的视和视觉惯导的系统。在已知地图的环境中可以无漂移的运行，其中**混合地图数据关联**－这个可以保证我们进行地图匹配和进行BA优化，这也达到了构建一个地图，然后可以在地图中进行精确的定位的目的。

   短期的数据关联：在最近的几帧中匹配地图元素。如同视觉里程计中做的一样，丢弃过于久远的帧，这会导致有累计的漂移。

   中期的数据关联：匹配相机累计误差小的地图，这也可以用在BA中，当系统在已经建好的地图中运行的时候可以达到零漂移。

   长期的数据关联：利用场景重识别来匹配当前的观测和先前的观测，不用管累计误差而且即使跟踪失败也可以实现,长期的匹配可以利用位姿图优化重新设置漂移，为了更准确也可以利用BA。这是在大场景中精度保证的关键。

5. ORB_SLAM3是可以解决纯视觉或者视觉惯导的完整的混合地图的SLAM系统，Atlas代表的是地图的集合，可以把其中的地图应用到所有的建图过程中，如场景重识别、相机重定位、闭环检测和精确的地图融合，即该系统为增量的SLAM系统，作者在IROS2019的一篇论文中首次提出了Atlas的概念：《ORBSLAM-atlas: a robust and accurate multi-map system》，而ORB_SLAM3添加了visual-inertial的混合地图系统来实现场景重识别。

6. 不限制相机模型，只需提供投影，反投影及Jacobian方程（程序中提供了针孔与鱼眼模型）

## 2.ORB_SLAM3的整体框架

![image-20210205163837752](C:\Users\T430\AppData\Roaming\Typora\typora-user-images\image-20210205163837752.png)

### 1.Atlas

 Atlas(地图集)由几乎无限数量的地图组成，每个地图都有自己的关键帧、地图点、共视图和生成树。每个地图参考帧固定在建立这个地图时的第一个相机中，并且它独立于ORB中的其他地图（non-active map）参考。传入的视频只更新地图册中的一个地图，为活动地图（active map），其余地图称为非活动地图（non active map）。该图集还包含一个所有地图的唯一DBoW2识别数据库，该数据库存储识别任何地图中任何关键帧的所有信息。好处是可以处理无限数量的子地图，这些子地图又有着唯一的词袋数据库，其包含了所有子地图的关键帧，因此可以进行高效的多地图位姿识别，并可以建立多映射的算法，当新的子地图创建时，可以将创建的新的映射与原来的映射合并，也可以将两个地图与一个公共地图进行无缝合并，合并后的新映射代替这里原有的映射，这样可以防止比如在跟踪时轨迹突然出现断点，断点前后的地图进行合并，还可以防止相机位姿不确定性导致的BA错误，提高稳定性。

### 2.跟踪线程

tracking部分与Atlas部分相配合，决定当前帧是否为关键帧，为Atlas输送新的关键帧，并且对该帧计算最小化重投影误差，VI模式中，通过IMU残差计算本体的速度与IMU偏差。如果跟踪丢失，尝试在Atlas所有地图中重定位，如果成功那个地图将成为活动的。如果几帧过后失败，则重新开始一个新的地图，这也使得ORB_SLAM3系统中有着非常多的子地图，并且也有着对于子地图进行融合的操作。

### 3.局部地图线程

添加新的关键帧与MapPoint到活动的地图中，删除冗余，利用滑动窗口通过BA更新地图。VI模式中IMU的参数在这个线程初始化与更新，使用的是作者提出的最大后验估计技术（MLPNP）。

### 4.回环与地图融合线程

 每添加一个关键帧，就探测活动的地图与其他地图的共有区域，如果检测到，执行回环矫正，如果不属于同一个地图，则将他们融合成一个。在矫正后另开一个线程进行整体的BA进一步更新地图且不影响实时性。

### 5.在monocular-IMU情况下的程序流程

因为实验室目前是希望在相机-IMU情况下进行定位以及建立地图，我将系统的流程图总结如下

![system](F:\计算机视觉与SLAM\system.png)

## 3.ORB_SLAM3特征点提取部分程序流程

### 1.overview

- 图像金字塔算法
- 四叉树算法
- 高斯模糊
- 计算描述子
- 特征点畸变校正
- 分配特征点

### 2.代码分析

![image-20210205154446361](C:\Users\T430\AppData\Roaming\Typora\typora-user-images\image-20210205154446361.png)

采用图像金字塔的原因是：相机在移动的场景中，与物体的距离时刻都在变化，因此是一个多尺度问题，图像金字塔是图像中多尺度表达方式的一种，是一种以多分辨率来解释图像的有效但概念简单的结构。利用图像金字塔结构，描述了多个距离下看同一图像的问题。举例来说，人眼可以很容易的在不同距离下看到并识别同个物体，但是计算机如何识别不同距离，我们需要一个办法帮机器在不同尺度下对同个物体有一样的认知能力。ORB_SLAM3中金字塔的层数为8

![image-20210205160136218](C:\Users\T430\AppData\Roaming\Typora\typora-user-images\image-20210205160136218.png)

计算图像金字塔部分对应于程序的ORBextractor::ComputePyramid部分，具体代码如下：

```c++
/**
	 * 构建图像金字塔
	 * @param image 输入原图像，这个输入图像所有像素都是有效的，也就是说都是可以在其上提取出FAST角点的
	 */
    void ORBextractor::ComputePyramid(cv::Mat image)
    {
        for (int level = 0; level < nlevels; ++level)
        {
            float scale = mvInvScaleFactor[level];
            Size sz(cvRound((float)image.cols*scale), cvRound((float)image.rows*scale));//限制尺寸
            Size wholeSize(sz.width + EDGE_THRESHOLD*2, sz.height + EDGE_THRESHOLD*2);
            Mat temp(wholeSize, image.type()), masktemp;
            mvImagePyramid[level] = temp(Rect(EDGE_THRESHOLD, EDGE_THRESHOLD, sz.width, sz.height));

        // Compute the resized image
		//计算第level0层以上resize后的图像
        if( level != 0 )
        {
			//将上一层金字塔图像根据设定sz缩放到当前层级
            resize(mvImagePyramid[level-1],	//输入图像
				   mvImagePyramid[level], 	//输出图像
				   sz, 						//输出图像的尺寸
				   0, 						//水平方向上的缩放系数，0自动计算
				   0,  						//垂直方向上的缩放系数，0自动计算
				   cv::INTER_LINEAR);		//图像缩放的差值算法类型，这里的是线性插值算法

			//把源图像拷贝到目的图像的中央，四面填充指定的像素。图片如果已经拷贝到中间，只填充边界
			//EDGE_THRESHOLD指的这个边界的宽度，由于这个边界之外的像素不是原图像素而是程序生成出来的，所以不能够在EDGE_THRESHOLD之外提取特征点			
            copyMakeBorder(mvImagePyramid[level], 					//源图像
						   temp, 									//目标图像（此时其实就已经有大了一圈的尺寸了）
						   EDGE_THRESHOLD, EDGE_THRESHOLD, 			//top & bottom 需要扩展的border大小
						   EDGE_THRESHOLD, EDGE_THRESHOLD,			//left & right 需要扩展的border大小
                           BORDER_REFLECT_101+BORDER_ISOLATED);     //扩充方式，opencv给出的解释：
			//BORDER_ISOLATED	表示对整个图像进行操作
        }
        else
        {
			//对于底层图像，直接就扩充边界了 copyMakeBorder是OpenCV自带边框处理函数
            copyMakeBorder(image,			//这里是原图像
						   temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                           BORDER_REFLECT_101);//这里采用对称法    
        }
    }

    }
```

特征点平均分配：金字塔层数越高，图像的面积越小，所提取的特征点数量就越少，按照面积将特征点分配到每一层，ORB_SLAM3::System时候调用Tracking，又调用ParseORBParamFile，又调用ORBextractor构造初始化提取器mpIniORBextractor，先平均，再计算金字塔。

```c++
//图片降采样缩放系数的倒数
    float factor = 1.0f / scaleFactor;
	//每个单位缩放系数所希望的特征点个数
    float nDesiredFeaturesPerScale = nfeatures*(1 - factor)/(1 - (float)pow((double)factor, (double)nlevels));

	//用于在特征点个数分配的，特征点的累计计数清空
    int sumFeatures = 0;
	//开始逐层计算要分配的特征点个数，顶层图像除外（看循环后面）
    for( int level = 0; level < nlevels-1; level++ )
    {
		//分配 cvRound : 返回个参数最接近的整数值
        mnFeaturesPerLevel[level] = cvRound(nDesiredFeaturesPerScale);
		//累计
        sumFeatures += mnFeaturesPerLevel[level];
		//乘系数
        nDesiredFeaturesPerScale *= factor;
    }
    //由于前面的特征点个数取整操作，可能会导致剩余一些特征点个数没有被分配，所以这里就将这个余出来的特征点分配到最高的图层中
    mnFeaturesPerLevel[nlevels-1] = std::max(nfeatures - sumFeatures, 0);
```

特征点的分配公式：

![image-20210205163700144](C:\Users\T430\AppData\Roaming\Typora\typora-user-images\image-20210205163700144.png)

图像金字塔对图像进行了缩放，假如要把该层的图像特征点移到其他层上，就要对应的放大图像，同时相机与图像的距离也要对应着进行缩放，保证其尺度不变性。

```c++
cv::Mat PC = Pos - Ow;
    const float dist = cv::norm(PC);
    const int level = (pFrame -> Nleft == -1) ? pFrame->mvKeysUn[idxF].octave
                                              : (idxF < pFrame -> Nleft) ? pFrame->mvKeys[idxF].octave
                                                                         : pFrame -> mvKeysRight[idxF].octave;
    const float levelScaleFactor =  pFrame->mvScaleFactors[level];
    const int nLevels = pFrame->mnScaleLevels;

    mfMaxDistance = dist*levelScaleFactor;//最大物距
    mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];//最小物距
```

 ORB-SLAM3中使用四叉树来快速筛选特征点，筛选的目的是非极大值抑制，取局部特征点邻域中FAST角
点相应值最大的点，而如何搜索到这些扎堆的特征点，则采用的是四叉树的快分思想，递归找到成群的点，并
从中找到相应值最大的点，从而使得特征点不会过于集中于图片中的某一部分

如果图片的宽度比较宽，就先把分成左右w/h份。一般的640×480的图像开始的时候只有一个node。
如果node里面的点数>1，把每个node分成四个node，如果node里面的特征点为空，就不要了，删掉。
新分的node的点数>1，就再分裂成4个node。如此，一直分裂。终止条件为：node的总数量> 规定的数量阈值 ，或者无法再进行分裂。然后从每个node里面选择一个质量最好的FAST点。

![image-20210205171435483](C:\Users\T430\AppData\Roaming\Typora\typora-user-images\image-20210205171435483.png)

提取出特征点后，需要计算描述子，而在计算之前需要进行高斯模糊，所谓”模糊”，可以理解成每一个像素都取周边像素的平均值。在数值上，这是一种”平滑化”。在图形上，就相当于产生”模糊”效果，”中间点”失去细节。

程序中采用的是OpenCV内置GaussianBlur(workingMat, workingMat, Size(7, 7), 2, 2, BORDER_REFLECT_101);

计算描述子

图像的特征点可以简单的理解为图像中比较显著显著的点，如轮廓点，较暗区域中的亮点，较亮区域中的暗点等。ORB采用的是哪种描述子呢？是用FAST（features from accelerated segment test）FAST是一种角点，主要检测局部像素灰度变化明显的地方，以速度快著称。它的思想是：如果一个像素与邻域的像素差别较大（过亮或过暗），那么它更可能是角点。相比于其他角点检测算法，FAST只需比较像素亮度的大小，十分快捷。它的检测过程如下（如图）：其中采用了非极大抑制

![image-20210205175339949](C:\Users\T430\AppData\Roaming\Typora\typora-user-images\image-20210205175339949.png)

在得到特征点后，我们需要以某种方式描述这些特征点的属性。这些属性的输出我们称之为该特征点的描述子（Feature DescritorS）。ORB采用BRIEF算法来计算一个特征点的描述子。BRIEF算法的核心思想是在关键点P的周围以一定模式选取N个点对，把这N个点对的比较结果组合起来作为描述子。计算特征描述子的步骤分四步：

1. 以关键点P为圆心，以d为半径做圆O
2. 在圆O内某一模式选取N个点对。这里为方便说明，N=4，实际应用中N可以取512。
3. 定义操作T![image-20210205180034333](C:\Users\T430\AppData\Roaming\Typora\typora-user-images\image-20210205180034333.png)
4. 分别对已选取的点对进行T操作，将得到的结果进行组合。![image-20210205180108573](C:\Users\T430\AppData\Roaming\Typora\typora-user-images\image-20210205180108573.png)

ORB_SLAM3中的代码实现如下：

```c++
//原始的BRIEF描述子不具有方向信息，通过加入特征点的方向来计算描述子，称之为Steer BRIEF，具有较好旋转不变特性
	//具体地，在计算的时候需要将这里选取的随机点点集的x轴方向旋转到特征点的方向。
	//获得随机“相对点集”中某个idx所对应的点的灰度,这里旋转前坐标为(x,y), 旋转后坐标(x',y')推导:
    // x'= xcos(θ) - ysin(θ),  y'= xsin(θ) + ycos(θ)
    #define GET_VALUE(idx) center[cvRound(pattern[idx].x*b + pattern[idx].y*a)*step + cvRound(pattern[idx].x*a - pattern[idx].y*b)]        
    // y'* step
    // x'
	//brief描述子由32*8位组成
	//其中每一位是来自于两个像素点灰度的直接比较，所以每比较出8bit结果，需要16个随机点，这也就是为什么pattern需要+=16的原因
    for (int i = 0; i < 32; ++i, pattern += 16)
    {
		
        int t0, 	//参与比较的一个特征点的灰度值
			t1,		//参与比较的另一个特征点的灰度值	
			val;	//描述子这个字节的比较结果
		
        t0 = GET_VALUE(0); t1 = GET_VALUE(1);
        val = t0 < t1;							//描述子本字节的bit0
        t0 = GET_VALUE(2); t1 = GET_VALUE(3);
        val |= (t0 < t1) << 1;					//描述子本字节的bit1
        t0 = GET_VALUE(4); t1 = GET_VALUE(5);
        val |= (t0 < t1) << 2;					//描述子本字节的bit2
        t0 = GET_VALUE(6); t1 = GET_VALUE(7);
        val |= (t0 < t1) << 3;					//描述子本字节的bit3
        t0 = GET_VALUE(8); t1 = GET_VALUE(9);
        val |= (t0 < t1) << 4;					//描述子本字节的bit4
        t0 = GET_VALUE(10); t1 = GET_VALUE(11);
        val |= (t0 < t1) << 5;					//描述子本字节的bit5
        t0 = GET_VALUE(12); t1 = GET_VALUE(13);
        val |= (t0 < t1) << 6;					//描述子本字节的bit6
        t0 = GET_VALUE(14); t1 = GET_VALUE(15);
        val |= (t0 < t1) << 7;					//描述子本字节的bit7

        //保存当前比较的出来的描述子的这个字节
        desc[i] = (uchar)val;
    }//通过对随机点像素灰度的比较，得出BRIEF描述子，一共是32*8=256位

    //为了避免和程序中的其他部分冲突在，在使用完成之后就取消这个宏定义
    #undef GET_VALUE
}
```

在当前关键点P周围以一定模式选取N个点对，组合这N个点对的T操作的结果就为最终的描述子。当我们选取点对的时候，是以当前关键点为原点，以水平方向为X轴，以垂直方向为Y轴建立坐标系。但是实际运动过程中相机运动包含了平移和旋转，当图片发生旋转时，坐标系不变，同样的取点模式取出来的点却不一样，计算得到的描述子也不一样，这是不符合要求的。因此需要重新建立坐标系，使新的坐标系可以跟随图片的旋转而旋转。这样以相同的取点模式取出来的点将具有一致性。
ORB在计算BRIEF描述子时建立的坐标系是以关键点为圆心，以关键点P和取点区域的质心Q的连线为X轴建立2维坐标系。P为关键点。圆内为取点区域，每个小格子代表一个像素。现在我们把这块圆心区域看做一块木板，木板上每个点的质量等于其对应的像素值。根据积分的知识我们可以求出这个密度不均匀木板的质心Q。

计算步骤如下：

![image-20210205180924295](C:\Users\T430\AppData\Roaming\Typora\typora-user-images\image-20210205180924295.png)

特征点畸变校正

实际上，现实中使用的相机由于镜头中镜片因为光线的通过产生的不规则的折射，镜头畸变（lens distortion）总是存在的，即根据理想针孔成像模型计算出来的像点坐标与实际坐标存在偏差。畸变的引入使得成像模型中的几何变换关系变为非线性，增加了模型的复杂度，但更接近真实情形。畸变导致的成像失真可分为径向失真和切向失真两类。ORB_SLAM3采用的是OpenCV内置的cv::undistortPoints

特征点分配：

将图片分割为64*48大小的栅格，并将关键点按照位置分配到相应栅格中，从而降低匹配时的复杂度，实现加速计算。通过网格的形式快速分配特征点。

### 一些思考

- 作者原论文中的特征点去畸变是在计算图像金字塔，计算FAST以及BRIEF描述子后进行的，那为什么不在计算图像金字塔之前就进行去畸变呢？把去畸变后的图像作为输入
- ORB_SLAM3中对于特征点的提取采用对的FAST特征提取，原理是去找图像中亮度变化比较大的点，但是在实际的室内场景中平面元素很多，这样的话亮度变化较大的点非常少，导致实际中很难持续跟踪以及建立地图，目前公开的数据集中是在墙壁旁边放一些标定板，强行增加特征点，我觉得如果要让ORB_SLAM3在实际工业场景以及室内特征的识别更鲁棒性，需要识别一些更多的室内特征，比如线特征以及面特征，这部分我正在学习SLAM中的室内约束，阅读《SLAM中的几何与学习方法》并尝试理解吸收

