Gemma-3-27b Speed Test
----------------------

by karminski-牙医

**gemma-3-27b-it-4bit**

```
python -m mlx_vlm.generate --model /Volumes/WORK_2/works/huggingface.co/mlx-community/gemma-3-27b-it-4bit  --prompt "what is your model name and version?"

Prompt: what is your model name and version?
<pad><pad><pad><pad>..........
==========
Prompt: 9 tokens, 27.138 tokens-per-sec
Generation: 256 tokens, 29.651 tokens-per-sec
Peak memory: 16.312 GB


```

**gemma-3-27b-it-6bit**


```
python -m mlx_vlm.generate --model /Volumes/WORK_2/works/huggingface.co/mlx-community/gemma-3-27b-it-6bit  --prompt "what is your model name and version?"

Prompt: what is your model name and version?
<pad><pad><pad><pad>..........
==========
Prompt: 9 tokens, 20.607 tokens-per-sec
Generation: 256 tokens, 20.521 tokens-per-sec
Peak memory: 23.060 GB

```

**gemma-3-27b-it-8bit**


```
python -m mlx_vlm.generate --model /Volumes/WORK_2/works/huggingface.co/mlx-community/gemma-3-27b-it-8bit  --prompt "what is your model name and version?"

Prompt: what is your model name and version?
<pad><pad><pad><pad>..........
==========
Prompt: 9 tokens, 15.945 tokens-per-sec
Generation: 256 tokens, 18.725 tokens-per-sec
Peak memory: 29.812 GB


```


**gemma-3-27b-it-bf16**


```
python -m mlx_vlm.generate --model /Volumes/WORK_2/works/huggingface.co/mlx-community/gemma-3-27b-it-bf16  --prompt "what is your model name and version?"

Prompt: what is your model name and version?

I'm Gemma, a large language model trained by Google DeepMind. The Gemma team are my creators. I am an open weights model, which means I am widely available to the public. I don't have versions in the same way software does, but I am constantly being updated by the Gemma team.
<end_of_turn><end_of_turn><end_of_turn>..........
==========
Prompt: 9 tokens, 5.269 tokens-per-sec
Generation: 256 tokens, 11.586 tokens-per-sec
Peak memory: 55.161 GB


```



## Gemma-3-27b 测试结果

测试基于 Mac Studio M2 Ultra 128GB

| 模型                | Prompt速度 (tokens/sec) | Generation速度 (tokens/sec) | Peak Memory (GB) |
| ------------------- | ----------------------- | --------------------------- | ---------------- |
| gemma-3-27b-it-4bit | 27.138                  | 29.651                      | 16.312           |
| gemma-3-27b-it-6bit | 20.607                  | 20.521                      | 23.060           |
| gemma-3-27b-it-8bit | 15.945                  | 18.725                      | 29.812           |
| gemma-3-27b-it-bf16 | 5.269                   | 11.586                      | 55.161           |

从生成来看, 好像所有版本都出了问题, 量化版本有大量的```<pad>``` 而没有输出文本, BF16 则在输出完毕后增加了大量的 ```<end_of_turn>```. 这些都是非预期的.

所以建议还是等等官方修复.
