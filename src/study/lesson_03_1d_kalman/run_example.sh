#!/bin/bash

# 编译并运行一维卡尔曼滤波示例

echo "编译中..."
mkdir -p build
cd build
cmake ..
make

if [ $? -eq 0 ]; then
    echo ""
    echo "运行示例..."
    echo ""
    ./simple_kalman
else
    echo "编译失败！"
    exit 1
fi

