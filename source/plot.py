import pandas as pd
import matplotlib.pyplot as plt

# 从文件读取数据（假设是CSV格式）
df = pd.read_csv('/home/jz/Documents/ads_fpv_ws/src/acados_nmpc_controller/source/delay_data.csv')  # 替换为你的实际文件名

# 创建图表
plt.figure(figsize=(15, 7))

# 绘制折线图
plt.plot(df['elapsed time'], df['value'], 
         marker='', linestyle='-', linewidth=1, color='b', alpha=0.7)

# 添加标题和标签
plt.title('NMPC Controller Delay Over Time', fontsize=16)
plt.xlabel('Elapsed Time (seconds)', fontsize=14)
plt.ylabel('Control Delay (milliseconds)', fontsize=14)

# 设置网格和美观的样式
plt.grid(True, linestyle='--', alpha=0.5)
plt.gca().set_facecolor('#f5f5f5')  # 浅灰色背景

# 优化x轴显示
plt.xticks(fontsize=10, rotation=45)
plt.yticks(fontsize=10)
plt.tight_layout()

# 显示图表
plt.show()