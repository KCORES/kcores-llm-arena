#!/usr/bin/env python3
import os
import subprocess
import time
import datetime
import signal
import threading
import select
import psutil  # 如果系统没有安装，需要先 pip install psutil
from pathlib import Path

# 配置参数
MODEL_DIR = "/workspace/llms/QwQ-32B-GGUF/"
RESULTS_DIR = "/workspace/results/"
LLAMA_CLI_PATH = "/workspace/apps/llama.cpp/build/bin/llama-cli"

# 模型列表
MODELS = [
    "qwq-32b-q2_k-00001-of-00004.gguf",
    "qwq-32b-q4_k_m-00001-of-00005.gguf",
    "qwq-32b-q5_k_m-00001-of-00006.gguf",
    "qwq-32b-q8_0-00001-of-00009.gguf",
]

# 基准测试列表
BENCHMARKS = {
    "benchmark-ball-bouncing-inside-spinning-hexagon": """Write a Python program that shows 20 balls bouncing inside a spinning heptagon:
- All balls have the same radius.
- All balls have a number on it from 1 to 20.
- All balls drop from the heptagon center when starting.
- Colors are: #f8b862, #f6ad49, #f39800, #f08300, #ec6d51, #ee7948, #ed6d3d, #ec6800, #ec6800, #ee7800, #eb6238, #ea5506, #ea5506, #eb6101, #e49e61, #e45e32, #e17b34, #dd7a56, #db8449, #d66a35
- The balls should be affected by gravity and friction, and they must bounce off the rotating walls realistically. There should also be collisions between balls.
- The material of all the balls determines that their impact bounce height will not exceed the radius of the heptagon, but higher than ball radius.
- All balls rotate with friction, the numbers on the ball can be used to indicate the spin of the ball.
- The heptagon is spinning around its center, and the speed of spinning is 360 degrees per 5 seconds.
- The heptagon size should be large enough to contain all the balls.
- Do not use the pygame library; implement collision detection algorithms and collision response etc. by yourself. The following Python libraries are allowed: tkinter, math, numpy, dataclasses, typing, sys.
- All codes should be put in a single Python file.""",
    "mandelbrot-set-meet-libai-benchmark": """请完成编程比赛，比赛内容如下：

请使用html, css, javascript, 创建一个动画，要求如下：

- 使用 canvas 绘制动画
- 请使用全屏展示动画
- 所有代码需要放在同一个HTML文件里面
- 动画内容是 ASCII 风格的 Mandelbrot Set, Mandelbrot Set 的主要图形初始大小为屏幕的50%，每渲染一次每帧放大 0.5%，总计渲染200次，200次后重置动画循环渲染
- Mandelbrot set 的候选字符使用李白的诗《静夜思》
- 需要对诗词进行去重并保留原有的字符顺序，并且不包含标点符号，可以循环使用
- Mandelbrot Set 的 Main cardioid and period bulbs 部分留空
- 动画的中心应使始终为 Main cardioid and period bulbs 的交界处
- 动画字体大小8px, 字体渲染排列也是8px，无间距
- Mandelbrot Set 字符颜色候选集为（由浅到深色）：#eaf4fc，#eaedf7，#e8ecef，#ebf6f7， #bbc8e6， #bbbcde， #8491c3， #867ba9， #68699b， #706caa， #5654a2， #4d4398， #4a488e， #274a78 ， #2a4073， #223a70， #192f60， #1c305c， #17184b， #0f2350
- 字符从 mandelbrot set的最外围使用最深的颜色，然后依次变浅
- 左上角有一个显示当前FPS的指示器（FPS），以及一个统计平均FPS的指示器（AVG FPS）, 以及一个当前渲染帧序号指示器（CURRENT FRAME: n/200），平均帧率在200帧渲染完毕后计算和更新，黑色字体，背景使用半透明白色圆角

请仔细还原上面每一个要求，每个要求的还原程度均作为采分点  
同时，尽量优化渲染性能，提高FPS。以平均30FPS为满分
运行环境使用Chrome浏览器。CPU为8核心CPU，因此最大可用线程数为8""",
    "benchmark-mars-mission": """Generate code for an animated 3d plot of a launch from earth landing on mars and then back to earth at the next launch window""",
    "benchmark-solar-system": """创建一个符合以下规格的HTML5 Canvas太阳系动画：

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


注意，不要省略星球视觉设计的代码，我需要你实现全部的代码。""",
}

# 创建结果目录
os.makedirs(RESULTS_DIR, exist_ok=True)


def run_test(model, benchmark_name, prompt, turn):
    """运行单个测试并返回耗时"""
    model_path = os.path.join(MODEL_DIR, model)
    log_file = f"{benchmark_name}-{model}-turn-{turn}.log"
    log_path = os.path.join(RESULTS_DIR, log_file)

    # 构建命令 - 不使用tee，而是通过Python处理输出
    cmd = f'{LLAMA_CLI_PATH} -m {model_path} -ngl 128 -sm layer -mg 2 -c 4096 -n 16384 -t 12 -tb 12 -b 4096 -ub 512 --temp 0.7  --mlock --numa distribute --prompt "{prompt}"'

    start_time = time.time()
    print(f"开始测试: {benchmark_name} - {model} - 轮次 {turn}")
    print(f"日志保存至: {log_path}")

    # 创建日志文件
    with open(log_path, "w", encoding="utf-8") as log_file:
        # 启动进程
        process = subprocess.Popen(
            cmd,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            bufsize=1,
            universal_newlines=True,
            errors="replace",
        )

        # 上次输出时间
        last_output_time = time.time()

        # 监控输出
        while process.poll() is None:
            # 检查是否有新输出
            ready, _, _ = select.select([process.stdout], [], [], 0.1)

            if ready:
                output = process.stdout.readline()
                if output:
                    # 有新输出，更新时间
                    print(output.rstrip())
                    log_file.write(output)
                    log_file.flush()
                    last_output_time = time.time()

            # 检查是否超过5秒无输出
            if time.time() - last_output_time > 5:
                print(f"检测到5秒无输出，终止当前测试: {benchmark_name} - {model}")

                # 获取进程ID
                pid = process.pid
                print(f"终止进程ID: {pid}")

                # 使用系统命令终止进程及其子进程
                try:
                    # 获取所有子进程
                    parent = psutil.Process(pid)
                    children = parent.children(recursive=True)

                    # 终止所有子进程
                    for child in children:
                        print(f"终止子进程: {child.pid}")
                        subprocess.run(f"kill -9 {child.pid}", shell=True)

                    # 终止主进程
                    subprocess.run(f"kill -9 {pid}", shell=True)
                except Exception as e:
                    print(f"终止进程时出错: {e}")
                    # 尝试使用Python的方法终止
                    try:
                        process.kill()
                    except:
                        pass
                break

        # 读取剩余输出
        try:
            remaining_output, _ = process.communicate(timeout=1)
            if remaining_output:
                print(remaining_output.rstrip())
                log_file.write(remaining_output)
        except subprocess.TimeoutExpired:
            print("无法获取剩余输出，强制终止")
            subprocess.run(f"kill -9 {process.pid}", shell=True)

    end_time = time.time()
    elapsed = end_time - start_time
    print(f"测试完成: 耗时 {elapsed:.2f} 秒")
    return elapsed


def format_time(seconds):
    """将秒数格式化为可读时间"""
    return str(datetime.timedelta(seconds=int(seconds)))


def main():
    total_tests = len(MODELS) * len(BENCHMARKS) * 3
    completed_tests = 0
    total_time = 0

    print(f"开始测试 - 总计 {total_tests} 个测试")

    for model in MODELS:
        for benchmark_name, prompt in BENCHMARKS.items():
            for turn in range(1, 4):
                # 运行测试并记录时间
                elapsed = run_test(model, benchmark_name, prompt, turn)
                total_time += elapsed
                completed_tests += 1

                # 计算预计剩余时间
                avg_time_per_test = total_time / completed_tests
                remaining_tests = total_tests - completed_tests
                estimated_time_left = avg_time_per_test * remaining_tests

                # 显示进度
                progress = completed_tests / total_tests * 100
                print(f"进度: {progress:.2f}% ({completed_tests}/{total_tests})")
                print(f"预计剩余时间: {format_time(estimated_time_left)}")
                print("-" * 50)

                # 在测试之间添加5秒的休息间隔，除非是最后一个测试
                if completed_tests < total_tests:
                    print("休息5秒后继续下一个测试...")
                    time.sleep(5)

    print(f"所有测试完成! 总耗时: {format_time(total_time)}")


if __name__ == "__main__":
    main()
