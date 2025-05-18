from tqdm import tqdm
import time
import numpy as np
import tkinter as tk
from tkinter import ttk

# Fixing random state for reproducibility
np.random.seed(19680801)

dt = 0.001
t = np.arange(0, 10, dt)
nse1 = np.random.randn(len(t))
nse2 = np.random.randn(len(t))

# 創建主視窗
root = tk.Tk()
root.title("進度顯示")
root.geometry("400x200")

# 計算螢幕中心位置並設置視窗位置
screen_width = root.winfo_screenwidth()
screen_height = root.winfo_screenheight()
x = (screen_width - 400) // 2
y = (screen_height - 200) // 2
root.geometry(f"400x200+{x}+{y}")

# 創建一個框架來容納溫度標籤和進度條
main_frame = ttk.Frame(root)
main_frame.pack(expand=True, fill='both', padx=20, pady=10)

# 創建 CPU 頻率標籤（靠左對齊）
freq_label = ttk.Label(main_frame, text="CPU 頻率: -- MHz")
freq_label.pack(anchor='w', pady=0, padx=(35, 0))

# 創建系統時脈標籤（靠左對齊）
clock_label = ttk.Label(main_frame, text="系統時脈: -- MHz")
clock_label.pack(anchor='w', pady=0, padx=(35, 0))

# 創建溫度標籤（靠左對齊）
temp_label = ttk.Label(main_frame, text="溫度: --°C")
temp_label.pack(anchor='w', pady=0, padx=(35, 0))

# 創建進度條
progress = ttk.Progressbar(main_frame, length=300, mode='determinate', maximum=100)
progress.pack(pady=10)

# 創建標籤顯示進度百分比
label = ttk.Label(main_frame, text="0%")
label.pack(pady=5)

# 控制變量
is_running = False

# 更新進度條的函數
def update_progress(i):
    progress['value'] = i
    label['text'] = f"{i}%"
    root.update()

# 執行進度條的函數
def run_progress():
    global is_running
    is_running = True
    
    # 重置進度條
    progress['value'] = 0
    label['text'] = "0%"
    
    # 主循環
    for i in range(100):
        if not is_running:  # 如果被中斷，直接返回
            return
        nse1 = np.random.randn(len(t))
        s1 = np.sin(2 * np.pi * 10 * t) + nse1
        delay = max(0.001, 0.005 * np.sin(2 * np.pi * i / 100))
        update_progress(i + 1)
        time.sleep(delay)
    
    is_running = False

# 重新執行按鈕的處理函數
def restart_progress():
    global is_running
    is_running = False  # 停止當前執行
    root.after(100, run_progress)  # 延遲100ms後重新開始

# 創建重跑按鈕
restart_button = ttk.Button(main_frame, text="重新執行", command=restart_progress)
restart_button.pack(pady=2)  # 減少上下間距

# 設置按鈕樣式
style = ttk.Style()
style.configure('TButton', padding=0)  # 移除按鈕內部填充

# 更新溫度的函數
def update_temperature():
    try:
        import subprocess
        temp = subprocess.check_output(['vcgencmd', 'measure_temp']).decode()
        temp = temp.replace('temp=', '').replace("'C", '°C')
        temp_label['text'] = f"Raspberry Pi 溫度: {temp}"
        
        # 讀取 CPU 頻率
        freq = subprocess.check_output(['vcgencmd', 'measure_clock', 'arm']).decode()
        freq = int(freq.split('=')[1]) // 1000000  # 轉換為 MHz
        freq_label['text'] = f"CPU 頻率: {freq} MHz"
        
        # 讀取系統時脈
        clock = subprocess.check_output(['vcgencmd', 'measure_clock', 'core']).decode()
        clock = int(clock.split('=')[1]) // 1000000  # 轉換為 MHz
        clock_label['text'] = f"系統時脈: {clock} MHz"
    except:
        temp_label['text'] = "Raspberry Pi 溫度: 無法讀取"
        freq_label['text'] = "CPU 頻率: 無法讀取"
        clock_label['text'] = "系統時脈: 無法讀取"
    root.after(1000, update_temperature)  # 每秒更新一次

# 首次執行
update_temperature()  # 開始更新溫度
run_progress()

# 完成後保持視窗開啟
root.mainloop()