# 任务三 · OpenCV 轮廓检测

## 一、做法

先猜那个网站（One Last Image）怎么做的：**边缘检测 + 双色上色**。先用边缘检测（Canny / XDoG 这类）把图里的"线"提出来，再按一套规则把线分成两色（暖色和冷色），最后合成到白底上。所以它的核心和我这道题要做的轮廓检测是同一件事。

我的做法，只用最基础的几个 OpenCV 函数：

```
彩色图 ──灰度──► 灰图 ──高斯模糊──► 去噪 ──Canny──► 边缘图
                                                      │
                                        findContours 把边缘连成一条条轮廓
                                                      │
                                        drawContours 画到白底上
```

为什么这么排：Canny 只认亮度变化，所以先转灰度（省算力也更稳）；先模糊是因为照片噪点也算"边缘"，不抹掉会满屏碎线。

## 二、代码

`opencv_ws/task3/src/contour_detect.cpp`，一共 39 行。

```cpp
// 任务三：对彩色图片做轮廓检测并绘制。
// 流程：读图 → 灰度 → 高斯模糊 → Canny 边缘 → findContours 找轮廓 → drawContours 画出来。
// 用法：./contour_detect <图片> <输出图> <Canny低阀值> <Canny高阀值>

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>   // imread / imwrite：读图和存图
#include <opencv2/imgproc.hpp>     // cvtColor / GaussianBlur / Canny / findContours / drawContours

int main(int argc, char** argv) {
    // 四个参数都有默认值，直接跑 ./contour_detect 也能用
    const std::string input = argc > 1 ? argv[1] : "images/official_input.jpg";
    const std::string output = argc > 2 ? argv[2] : "output/contours.png";
    const double canny_low = argc > 3 ? std::atof(argv[3]) : 35.0;
    const double canny_high = argc > 4 ? std::atof(argv[4]) : 105.0;

    const cv::Mat color = cv::imread(input, cv::IMREAD_COLOR);
    if (color.empty()) {
        std::cerr << "读不到图片: " << input << '\n';
        return 1;
    }

    // 1) 灰度化：Canny 只关心亮度变化，不需要颜色
    cv::Mat gray;
    cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);

    // 2) 高斯模糊：5x5 的核。抹掉噪点，避免检出满屏碎线
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 1.5);

    // 3) Canny 双阀值找边缘：低于低阀值的丢掉，高于高阀值的保留，
    //    卡在中间的看它有没有连着强边缘
    cv::Mat edges;
    cv::Canny(gray, edges, canny_low, canny_high);

    // 4) 找轮廓：RETR_LIST 取所有轮廓；CHAIN_APPROX_SIMPLE 只保留拐点，省内存
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    // 5) 画到白底上：contourIdx 传 -1 表示一次把所有轮廓都画出来，线宽 1
    cv::Mat canvas(color.size(), CV_8UC3, cv::Scalar(255, 255, 255));
    cv::drawContours(canvas, contours, -1, cv::Scalar(0, 0, 0), 1);

    cv::imwrite(output, canvas);

    std::cout << input << " -> " << output << "  轮廓 " << contours.size() << " 条\n";
    return 0;
}
```

编译运行：

```bash
cd /workspaces/RMCS/opencv_ws/task3
cmake -S . -B build
cmake --build build -j8
./build/contour_detect images/official_input.jpg output/official_contours.png
```

## 三、和官方输出对比

**怎么拿到的对比素材**：官方仓库（[itorr/one-last-image](https://github.com/itorr/one-last-image)）里带了演示原图，我把它下载下来当输入；再把官方这个工具的网页跑起来、生成线稿、把结果图截回容器当"官方输出"。两张都是 1377×817，同一张图，可以直接比。

![原图 / 官方输出 / 我的结果](task3/output/official_compare.png)

左：原始彩色图 ｜ 中：官方输出线稿 ｜ 右：我的轮廓结果

**量化对比**（用"线条像素是否重叠"来算，允许 5 像素的偏差）：

| 指标 | 数值 | 说明 |
|---|---|---|
| 官方线像素 | 126043（占画面 11.2%） | 官方线稿的"墨" |
| 我的线像素 | 30889（占画面 2.75%） | 我的线明显更细 |
| **我的线落在官方线 5px 内** | **99.4%** | 我画出来的线几乎都在官方线的位置上 |
| **官方线被我覆盖** | **58.3%** | 官方线里有 41.7% 我没画出来 |

**结论**：**位置对得上，密度不够**。

99.4% 说明我检出的轮廓基本都落在官方线稿该有的位置上——不存在"画歪了"或"画到空白处"的情况。58.3% 说明官方线稿比我的更密：它除了主轮廓，还有一层很淡的"调子线"（头发丝、布料纹理那类浅描），Canny 只认亮度突变，这些淡线抓不到。

按题目要求"不要求渐变色一样，但轮廓差不多"——主轮廓已经对上了，差的是淡描那部分。想补密就把阈值调低（下一节）。

其他测试图（同样的程序，换个输入而已）：

![蝴蝶](task3/output/butterfly.png)

## 四、调参

```bash
./build/contour_detect <图片> <输出图> <Canny低阀> <Canny高阀>
```

| 参数 | 调大 | 调小 |
|---|---|---|
| Canny 高阀 | 线更少、更干净 | 线更多、更碎 |
| Canny 低阀 | 弱边更容易被丢掉，线条更断 | 断线更容易接上强边 |

在同一张官方原图上实测：

| 阈值 | 线像素 | 精度（线落在官方线上的比例） | 覆盖（官方线被我画出的比例） |
|---|---|---|---|
| 60 / 180 | 19675 | 99.9% | 41.5% |
| 45 / 135 | 23438 | 99.9% | 47.7% |
| **35 / 105（默认）** | 30889 | 99.4% | **58.3%** |
| 30 / 90 | 33689 | 99.0% | 61.9% |
| 25 / 75 | 39199 | 96.6% | 67.4% |
| 20 / 60 | 47254 | 91.9% | 73.5% |

规律很清楚：**阈值调低，覆盖率上去了，但精度会掉**——因为开始把不该画的地方也当成线了（20/60 时精度已经掉到 91.9%）。

默认取 35/105 是折中点：精度还有 99.4%，覆盖率比 60/180 高了近 17 个百分点。

另外还可以调的地方：把 `GaussianBlur` 的核从 5×5 改大（更干净但丢细节）或改小（细节多但噪点多）；把 `findContours` 的 `RETR_LIST` 换成 `RETR_EXTERNAL`（只留最外层轮廓，内部纹理全丢，变成简笔画效果）。
