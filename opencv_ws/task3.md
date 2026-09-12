# 任务三 · OpenCV 轮廓检测

## 一、做法

先说那个网站（One Last Image），我猜它是**边缘检测 + 双色上色**：先用边缘检测（Canny、XDoG 这类）把线提出来，再按一套规则把线分成暖色和冷色，最后合到白底上。它的底子跟这道题要做的轮廓检测就是一回事。

我自己就按最基础的做法来，只用五个函数：

彩色图 → 灰度 → 高斯模糊 → Canny → findContours → drawContours

顺序不是随便排的。Canny 只认亮度变化，所以先转灰度；不先模糊的话照片的噪点也算边缘，出来就是满屏碎线。

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

```bash
cd /workspaces/RMCS/opencv_ws/task3
cmake -S . -B build
cmake --build build -j8
./build/contour_detect images/official_input.jpg output/official_contours.png
```

## 三、和官方输出比

光看着像不算数，得拿官方的原图跑一遍，再跟官方的输出比。

素材是这么弄的：官方仓库（[itorr/one-last-image](https://github.com/itorr/one-last-image)）里就带演示原图，我把它下载下来当输入；再把官方那个工具跑起来、生成线稿、把结果图取回容器当"官方输出"。两张都是 1377×817，同一张图，可以直接比。

![原图 / 官方输出 / 我的结果](task3/output/official_compare.png)

左：原始彩色图 ｜ 中：官方输出线稿 ｜ 右：我的轮廓结果

拿"线条像素有没有重叠"来算，允许 5 像素偏差：

| 指标 | 数值 | 我怎么看 |
|---|---|---|
| 官方线像素 | 126043（占画面 11.2%） | 官方线稿的"墨" |
| 我的线像素 | 30889（占画面 2.75%） | 我的线明显更细 |
| **我的线落在官方线 5px 内** | **99.4%** | 我画的线几乎都落在官方线的位置上 |
| **官方线被我覆盖** | **58.3%** | 官方线里有 41.7% 我没画出来 |

结论：**位置对得上，密度不够**。

99.4% 说明我检出的轮廓基本都在官方线该在的地方，没有画歪、也没有画到空白处。58.3% 是官方线稿比我密——它除了主轮廓，还有一层很淡的"调子线"（发丝、布料纹理那种浅描），Canny 只认亮度突变，这些淡线抓不到。题目要求"不要求渐变色一样、但轮廓差不多"，主轮廓这块是达标的。

换个输入也一样（同一份程序）：

![蝴蝶](task3/output/butterfly.png)

## 四、调参

```bash
./build/contour_detect <图片> <输出图> <Canny低阀> <Canny高阀>
```

| 参数 | 调大 | 调小 |
|---|---|---|
| Canny 高阀 | 线更少、更干净 | 线更多、更碎 |
| Canny 低阀 | 弱边更容易被丢掉，线条更断 | 断线更容易接上强边 |

同一张官方原图上我实测了一轮：

| 阈值 | 线像素 | 精度（线落在官方线上） | 覆盖（官方线被我画出） |
|---|---|---|---|
| 60 / 180 | 19675 | 99.9% | 41.5% |
| 45 / 135 | 23438 | 99.9% | 47.7% |
| **35 / 105（默认）** | 30889 | 99.4% | **58.3%** |
| 30 / 90 | 33689 | 99.0% | 61.9% |
| 25 / 75 | 39199 | 96.6% | 67.4% |
| 20 / 60 | 47254 | 91.9% | 73.5% |

规律很干脆：**阈值调低，覆盖率上去、精度下来**。20/60 那档精度已经掉到 91.9%，开始把不该画的地方当线了。

所以我默认给 35/105：精度还有 99.4%，覆盖率比 60/180 高了近 17 个百分点，这是我觉得最划算的一档。

另外还能动的地方：`GaussianBlur` 的核从 5×5 改大（更干净但丢细节）、改小（细节多但噪点多）；`findContours` 的 `RETR_LIST` 换成 `RETR_EXTERNAL`，只留最外层轮廓、内部纹理全丢，就是简笔画效果。
