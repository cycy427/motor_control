from setuptools import setup, find_packages

setup(
    name="nubotddsmsg",          # 包名称（pip install时用）
    version="1.0.0",            # 版本号
    packages=find_packages(),    # 自动发现所有Python包
    package_dir={"": "."},       # 从当前目录查找包
    install_requires=[],         # 依赖的其他包（如需要）
    python_requires=">=3.6",     # Python版本要求
)
