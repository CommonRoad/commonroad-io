import os
import subprocess

# 设置目标目录，包含所有 .rst 文件
directory = "./"

for filename in os.listdir(directory):
    if filename.endswith(".rst"):
        input_path = os.path.join(directory, filename)
        output_path = os.path.join(directory, f"{os.path.splitext(filename)[0]}.md")

        # 使用 subprocess 调用 rst2md 命令行工具进行转换
        subprocess.run(["rst2md", input_path, output_path])
