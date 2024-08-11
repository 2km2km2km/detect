import os

# 设置CPU亲和性
def set_cpu_affinity(cpu_ids):
    pid = os.getpid()  # 获取当前进程ID
    os.sched_setaffinity(pid, cpu_ids)  # 设置亲和性

# 将进程绑定到CPU 0和CPU 1
set_cpu_affinity([5, 6])

while(1):
    print(1)