#!/bin/bash
echo -e 'installing cuda-libraries-dev-12-1 ...'

sudo apt-get install cuda-libraries-dev-12-1

nvcc --version

ls -la /usr/local/cuda/lib64/libcublas*

echo -e 'downloading llama.cpp ...'

git clone https://github.com/ggml-org/llama.cpp

cd llama.cpp

echo -e 'building llama.cpp ...'

cmake -B build -DGGML_CUDA=ON -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc
cmake --build build --config Release -j 

echo -e 'done'



