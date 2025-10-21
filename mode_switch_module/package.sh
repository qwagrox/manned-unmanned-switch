#!/bin/bash
# 打包脚本

echo "正在打包模式切换模块..."

# 创建发布目录
RELEASE_DIR="mode_switch_module_release_$(date +%Y%m%d)"
mkdir -p $RELEASE_DIR

# 复制核心文件
cp -r core $RELEASE_DIR/
cp -r simulator $RELEASE_DIR/
cp -r examples $RELEASE_DIR/
cp -r tests $RELEASE_DIR/
cp -r docs $RELEASE_DIR/
cp README.md $RELEASE_DIR/

# 创建requirements.txt
cat > $RELEASE_DIR/requirements.txt << 'REQUIREMENTS'
numpy>=1.20.0
matplotlib>=3.3.0
REQUIREMENTS

# 创建setup.py
cat > $RELEASE_DIR/setup.py << 'SETUP'
from setuptools import setup, find_packages

setup(
    name="mode_switch_module",
    version="1.0.0",
    description="前装无人驾驶拖拉机模式切换模块",
    packages=find_packages(),
    install_requires=[
        "numpy>=1.20.0",
        "matplotlib>=3.3.0",
    ],
    python_requires=">=3.8",
)
SETUP

# 打包
tar -czf ${RELEASE_DIR}.tar.gz $RELEASE_DIR

echo "打包完成: ${RELEASE_DIR}.tar.gz"
echo "文件大小: $(du -h ${RELEASE_DIR}.tar.gz | cut -f1)"
