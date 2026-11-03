from pathlib import Path
import hashlib
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parent
source = root / 'vofa+.csv'
digest = hashlib.sha256(source.read_bytes()).hexdigest()
d = pd.read_csv(source)
assert list(d.columns) == ['I0', 'I1'] and not d.isna().any().any()
assert np.isfinite(d.to_numpy()).all()
rows = []
for _, block in d.groupby(d.I1.ne(d.I1.shift()).cumsum()):
    pwm = int(block.I1.iloc[0])
    if pwm == 0:
        continue
    assert len(block) == 50
    tail = block.I0.iloc[25:]
    mean = tail.mean()
    rows.append(dict(pwm_percent=pwm, samples=len(tail), encoder_mean=mean,
                     encoder_std=tail.std(), encoder_min=tail.min(), encoder_max=tail.max(),
                     wheel_rpm=mean*1.2, wheel_speed_m_s=mean*.004))
s = pd.DataFrame(rows)
assert s.pwm_percent.tolist() == list(range(2,101,2))
fit = s[s.pwm_percent >= 20]
a,b = np.polyfit(fit.pwm_percent,fit.encoder_mean,1)
res = fit.encoder_mean-(a*fit.pwm_percent+b)
r2 = 1-(res**2).sum()/((fit.encoder_mean-fit.encoder_mean.mean())**2).sum()
s.to_csv(root/'motor1_calibration.csv',index=False,float_format='%.6f',encoding='utf-8-sig')
plt.rcParams.update({'font.size':11})
fig,axes=plt.subplots(1,2,figsize=(12,4.6),layout='constrained')
axes[0].errorbar(s.pwm_percent,s.encoder_mean,yerr=s.encoder_std,fmt='o',ms=3,label='Last 25 samples / step')
axes[0].plot(fit.pwm_percent,a*fit.pwm_percent+b,label='Linear fit: 20-100%')
axes[0].set(xlabel='PWM (%)',ylabel='Encoder counts / 20 ms',title='Motor 1: encoder calibration')
axes[0].legend()
axes[1].plot(s.pwm_percent,s.wheel_rpm,'o-',ms=3,color='#148276')
axes[1].set(xlabel='PWM (%)',ylabel='Wheel RPM',title='2500 counts/rev; circumference 0.20 m')
second=axes[1].secondary_yaxis('right',functions=(lambda rpm:rpm*.2/60,lambda v:v*60/.2))
second.set_ylabel('Wheel peripheral speed (m/s)')
for ax in axes: ax.grid(alpha=.25)
fig.savefig(root/'motor1_calibration.png',dpi=160)
table='| PWM (%) | Counts / 20 ms | Wheel RPM | Speed (m/s) |\n|---:|---:|---:|---:|\n'
for row in s[s.pwm_percent.isin([12,14,20,30,40,50,60,70,80,90,100])].itertuples():
    table+=f'| {row.pwm_percent} | {row.encoder_mean:.2f} | {row.wheel_rpm:.2f} | {row.wheel_speed_m_s:.4f} |\n'
report=f'''# 电机 1 PWM 标定

来源：vofa+.csv，{len(d)} 行；SHA256：{digest}。
I0 为每采样周期编码增量，I1 为 PWM 百分数。原始文件未修改。

## 方法及条件

按固件采样周期 20 ms、轮子每圈 2500 计数、周长 0.20 m 换算。
CSV 无时间戳，不能独立验证实际采样周期或是否丢包。
2% 至 100% 每档有 50 点；每档剔除前 25 点，取后 25 点作为稳定段估计。
起始和结束的 PWM=0 区间不参与拟合，避免混入停车过渡。
标准差为每档后 25 点的样本标准差，不是多次实验的重复性误差。

RPM = 编码增量 × 1.2；轮缘线速度(m/s) = 编码增量 × 0.004。
线速度为轮缘速度；车辆实际地速还取决于打滑。

## 结果

{table}

本次上升扫描从 14% 档开始持续转动；低档偶发计数不视为持续启动。
这不代表每次从静止启动都能使用同一个阈值。

对 20%～100% 的 41 个稳定段均值作等权线性拟合，P 为 0～100 的 PWM 百分数：

- 编码增量 ≈ {a:.6f} P {b:+.6f}
- 车轮 RPM ≈ {a*1.2:.6f} P {b*1.2:+.6f}
- 轮缘线速度(m/s) ≈ {a*.004:.8f} P {b*.004:+.8f}
- R² = {r2:.8f}；最大绝对残差 = {abs(res).max():.3f} 计数/20 ms。

可将 P ≈ (目标 RPM {(-b*1.2):+.6f}) / {a*1.2:.6f} 作为后续前馈初值，
只适用于本次测试条件和拟合范围；零速直接输出 0，低速启动需要单独处理，不能用线性外推替代启动阈值。
最终用编码器闭环修正供电、负载等变化，另一个电机及反转方向需要分别标定。

![标定曲线](motor1_calibration.png)
'''
(root/'motor1_calibration.md').write_text(report,encoding='utf-8')
assert hashlib.sha256(source.read_bytes()).hexdigest() == digest
print(table)
print(f'counts = {a:.6f} P {b:+.6f}; R2={r2:.8f}; max residual={abs(res).max():.3f}')
