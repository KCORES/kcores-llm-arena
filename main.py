import os
import time
from pathlib import Path
from dotenv import load_dotenv
from openai import OpenAI
import httpx # 导入 httpx 库，用于配置代理
import re # 导入正则表达式模块

# 加载 .env 文件中的环境变量
# 这会查找当前目录或父目录中的 .env 文件
load_dotenv()

# 从环境变量获取 API 配置
# 确保你的 .env 文件中有 BASE_URL 和 API_KEY
BASE_URL = os.getenv("BASE_URL")
API_KEY = os.getenv("API_KEY")

# 检查环境变量是否设置
if not BASE_URL or not API_KEY:
    print("错误：请确保 .env 文件中设置了 BASE_URL 和 API_KEY")
    exit(1)

HTTP_PROXY= os.getenv("HTTP_PROXY")


# 初始化 OpenAI 客户端
# 使用你的自定义 URL 和 API 密钥
try:
    if HTTP_PROXY:
        print(f"正在配置使用代理: {HTTP_PROXY}")
        # 创建配置了代理的 httpx 客户端
        # 警告: verify=False 会禁用 SSL 证书验证，这可能带来安全风险。
        # 仅在你完全信任代理服务器并且无法配置正确的证书时使用。
        # 在生产环境中，强烈建议配置正确的 CA 证书或让 httpx 使用系统信任的证书。
        http_client = httpx.Client(verify=False,proxy="http://127.0.0.1:10889")
        # 将自定义的 http_client 传递给 OpenAI
        client = OpenAI(base_url=BASE_URL, api_key=API_KEY, http_client=http_client)
        print("OpenAI 客户端已配置代理。")
    else:
        print("未检测到代理设置，将直接连接。")
        # 如果没有代理，则按原方式初始化
        client = OpenAI(base_url=BASE_URL, api_key=API_KEY)
        print("OpenAI 客户端初始化完成（无代理）。")
except Exception as e:
    print(f"初始化 OpenAI 客户端时出错: {e}")
    exit(1)


# --- 从 scripts/auto-benchmark.py 提取的 BENCHMARKS ---
BENCHMARKS = {
    "benchmark-ball-bouncing-inside-spinning-heptagon": """Write a Python program that shows 20 balls bouncing inside a spinning heptagon:
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
# --- BENCHMARKS 结束 ---

# 定义输出代码的文件扩展名
# 这有助于根据 benchmark 类型确定是 .py 还是 .html
BENCHMARK_EXTENSIONS = {
    "benchmark-ball-bouncing-inside-spinning-heptagon": ".py",
    "mandelbrot-set-meet-libai-benchmark": ".html",
    "benchmark-mars-mission": ".py", # 假设是 Python 实现
    "benchmark-solar-system": ".html",
}

PATHS={"benchmark-ball-bouncing-inside-spinning-heptagon":"benchmark-ball-bouncing-inside-spinning-heptagon",
           "mandelbrot-set-meet-libai-benchmark":"benchmark-mandelbrot-set-meet-libai",
           "benchmark-mars-mission":"benchmark-mars-mission",
           "benchmark-solar-system":"benchmark-solar-system"}

def get_available_models(openai_client: OpenAI, filter_keyword: str = None) -> list[str]:
    """
    从 OpenAI 兼容 API 获取可用模型列表，并可选择性地进行本地筛选。

    Args:
        openai_client: 已初始化的 OpenAI 客户端实例。
        filter_keyword: 用于筛选模型名称的关键字（不区分大小写）。如果为 None，则返回所有模型。

    Returns:
        包含符合条件的模型 ID 的列表。
        如果 API 调用失败，则返回空列表。
    """
    try:
        models_response = openai_client.models.list()
        # 直接迭代 models_response，而不是 models_response.data
        model_ids = [model.id for model in models_response]

        if filter_keyword:
            filter_keyword_lower = filter_keyword.lower()
            filtered_models = [
                model_id for model_id in model_ids
                if filter_keyword_lower in model_id.lower()
            ]
            return filtered_models
        else:
            return model_ids
    except Exception as e:
        print(f"获取模型列表时出错: {e}")
        return []

def generate_code(openai_client: OpenAI, model_name: str, prompt: str) -> str | None:
    """
    使用指定的模型和提示生成代码。

    Args:
        openai_client: 已初始化的 OpenAI 客户端实例。
        model_name: 要使用的模型 ID。
        prompt: 发送给模型的提示。

    Returns:
        生成的代码字符串，如果 API 调用失败则返回 None。
    """
    print(f"  正在使用模型 '{model_name}' 生成代码...")
    try:
        # 使用 Chat Completions API
        # 注意：你可能需要根据你的模型和 API 提供商调整参数（如 max_tokens, temperature）,此处默认temperature=0.6,max_tokens=8192
        # 这里使用了一个基本的示例
        response = openai_client.chat.completions.create(
            model=model_name,
            messages=[
                {"role": "system", "content": "You are a helpful assistant designed to write code based on prompts."},
                {"role": "user", "content": prompt}
            ],
            temperature=0.6, # 可以调整创造性
            max_tokens=8192, # 根据需要调整最大输出长度
        )
        # 提取生成的代码
        # 假设模型直接在 message content 中返回代码
        # 你可能需要根据模型的实际输出来调整这里的逻辑，比如解析代码块
        generated_content = response.choices[0].message.content
        # 使用正则表达式查找所有 ```...``` 代码块
        # 这个正则表达式会匹配 ``` 开始，可选的语言标识符（如 python），然后是非贪婪匹配内容，最后是 ``` 结束
        # re.DOTALL 让 . 匹配包括换行符在内的所有字符
        code_blocks = re.findall(r"```(?:\s*\w*\s*\n)?(.*?)```", generated_content, re.DOTALL)

        if not code_blocks:
            # 如果没有找到任何代码块，尝试返回原始内容（去除首尾空格）
            # 或者可以返回空字符串，取决于你希望的行为
            print("  未找到 ```...``` 代码块，不符合测试要求，将直接返回None结束本次测试")
            # return "" # 如果希望在找不到时返回空
            return None # 返回None

        # 找到最长的代码块
        # 我们比较的是去除首尾空格后的代码块内容的长度
        longest_code_block = max(code_blocks, key=lambda code: len(code.strip()))

        print(f"  找到了 {len(code_blocks)} 个代码块，选择了最长的。")
        return longest_code_block.strip() # 返回最长代码块的内容（去除首尾空格）

    except Exception as e:
        print(f"  使用模型 '{model_name}' 生成代码时出错: {e}")
        return None

def save_code_to_file(benchmark_name: str, model_name: str, turn: int, code: str):
    """
    将生成的代码保存到指定的文件。

    Args:
        benchmark_name: Benchmark 的名称，用于确定文件夹。
        model_name: 模型的名称。
        turn: 当前的测试轮次。
        code: 要保存的代码字符串。
    """
    # 确定文件扩展名
    file_extension = BENCHMARK_EXTENSIONS.get(benchmark_name, ".txt") # 默认为 .txt

    # 清理模型名称，移除可能不适合文件名的字符（例如 '/')
    safe_model_name = model_name.replace("/", "_").replace(":", "_")

    # 构建输出目录和文件路径
    # 文件将保存在 benchmark 名称对应的文件夹下的 src 子目录中
    output_dir = Path(PATHS[benchmark_name]) / "src"
    output_dir.mkdir(parents=True, exist_ok=True) # 确保目录存在
    file_name = f"{benchmark_name}-{safe_model_name}-turn-{turn}{file_extension}"
    output_path = output_dir / file_name

    # 写入文件
    try:
        with open(output_path, "w", encoding="utf-8") as f:
            f.write(code)
        print(f"  代码已保存至: {output_path}")
    except IOError as e:
        print(f"  保存文件 {output_path} 时出错: {e}")


def main(filter_keyword=None):
    need_replace=False
    print("开始生成测试代码...")

    # --- 模型选择 ---
    # 1. 获取所有可用模型
    print("正在获取可用模型列表...")
    all_models = get_available_models(client)
    if not all_models:
        print("无法获取模型列表，请检查 API 连接和配置。")
        return

    print("\n可用模型:")
    for i, model_id in enumerate(all_models):
        print(f"{i+1}. {model_id}")

    # 2. 让用户选择或过滤模型
    # 示例：使用包含 "Mistral" 的模型 (根据你的要求)
    if filter_keyword is None:
        filter_keyword = input("\n请输入要筛选的模型关键字（也可作为模型制造商前缀）（例如 'Mistral','OpenAI',”gemini“），或直接按回车以选择所有模型: ").strip()
        need_replace=input("保存代码文件时是否需要去除模型制造商前缀？（请确保你的模型以模型制造商商名称开始，例如Mistral-ministral-3b-latest，去除后变为ministral-3b-latest）(y/n): ").strip().lower() == 'y'
    selected_models = [m for m in all_models if m.lower().strip().startswith(filter_keyword.lower())]


    if not selected_models:
        print("\n没有根据筛选条件找到或选择任何模型。")
        return

    print(f"\n将使用以下模型生成代码: {', '.join(selected_models)}")
    # --- 模型选择结束 ---


    # 循环遍历 Benchmarks 和选定的模型
    total_tasks = len(BENCHMARKS) * len(selected_models) * 3
    completed_tasks = 0
    failed_model_name=[]

    for model_name in selected_models:
        print(f" 使用模型: {model_name}")
        for benchmark_name, prompt in BENCHMARKS.items():
            model_failed=False
            if model_failed:
                failed_model_name.append(model_name)
                break
            print(f"\n处理 Benchmark: {benchmark_name}")
            for turn in range(1, 4): # 每个项目测试三次
                print(f"  轮次 {turn}:")
                completed_tasks += 1
                print(f"  进度: {completed_tasks}/{total_tasks}")

                save_model_name = model_name
                if need_replace:
                    save_model_name = model_name[len(filter_keyword) + 1:]

                #检查文件是否存在
                normal_path=f"{PATHS[benchmark_name]}/src/{benchmark_name}-{save_model_name}-turn-{turn}{BENCHMARK_EXTENSIONS[benchmark_name]}"
                high_score_path=f"{PATHS[benchmark_name]}/src/{benchmark_name}-{save_model_name}-turn-{turn}-high-score{BENCHMARK_EXTENSIONS[benchmark_name]}"
                if os.path.exists(normal_path) or os.path.exists(high_score_path):
                    print(f"  文件 {benchmark_name}-{model_name}-turn-{turn} 已存在，跳过生成。")
                    continue
                # 生成代码
                generated_code = None
                retry_count = 0
                while generated_code is None:
                    if retry_count in range(1,4):
                        print(f"  生成代码失败，现在是第{retry_count}次重试")
                    elif retry_count > 3:
                        print(f"  生成代码失败，以超出重试次数，跳过此模型和基准")
                        model_failed=True
                        break
                    generated_code=generate_code(client, model_name, prompt)

                if generated_code is not None:
                    # 保存代码到文件
                    save_code_to_file(benchmark_name, save_model_name, turn, generated_code)
                else:
                    break

                # 防止过于频繁的 API 调用（可选）
                time.sleep(1) # 休眠1秒

    print("\n所有代码生成任务完成!")
    if len(failed_model_name)>0:
        print(f"以下模型生成代码失败:")
        for i,model in enumerate(failed_model_name):
            print(f"{i+1}. {model}")

if __name__ == "__main__":
    main()
