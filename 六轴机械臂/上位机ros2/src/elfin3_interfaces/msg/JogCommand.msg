# 按 J1 到 J6 的固定顺序排列，每个值必须为 NEGATIVE、STOPPED 或 POSITIVE。
int8 NEGATIVE=-1
int8 STOPPED=0
int8 POSITIVE=1
int8[6] directions

# 键盘点动输入有效期间必须保持为 true。
bool deadman

# 单调递增的 uint32 指令序号，达到最大值后允许自然回绕。
uint32 sequence
