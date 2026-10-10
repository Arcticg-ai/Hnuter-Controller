"""Scientific plots of new, actually executed trials only."""
import csv,json,os
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parent;O=R/'results';O.mkdir(exist_ok=True)
os.environ['MPLCONFIGDIR']=str(O/'mpl_cache')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams.update({'font.family':font_manager.FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc').get_name(),'axes.unicode_minus':False,'axes.spines.top':False,'axes.spines.right':False,'font.size':11,'figure.facecolor':'white'})
G=['A','B','C','D','C_shadow'];names=['A 固定','B 被动','C 主动','D 理想','影子组'];colors=['#64748b','#2563eb','#0891b2','#8b5cf6','#d97706']
s=json.loads((O/'summary.json').read_text());rows=list(csv.DictReader((O/'all_attempts.csv').open()));rows=[r for r in rows if r['batch'].startswith('batch_matrix_') and r['attitude_reference']==s['primary_attitude_reference']]
fig,axs=plt.subplots(2,2,figsize=(14,10));fig.subplots_adjust(hspace=.4,wspace=.27,bottom=.13,top=.9)
for ax,key,title in [(axs[0,0],'rmse','公共评价：PX4 估计位置'),(axs[0,1],'truth_rmse','公共评价：对齐 Gazebo 真值')]:
 for i,g in enumerate(G):
  v=[100*float(r[key]) for r in rows if r['group']==g and r[key]]
  if v:
   ax.scatter(i+np.linspace(-.12,.12,len(v)),v,color=colors[i]);ax.errorbar(i,np.mean(v),yerr=np.std(v,ddof=1) if len(v)>1 else 0,fmt='_',color='black',capsize=5)
   ax.text(i,1.01,f'n={len(v)}/5',transform=ax.get_xaxis_transform(),ha='center')
 ax.set_xticks(range(5),names);ax.set_ylabel('3D RMSE（cm）');ax.set_title(title,pad=25);ax.grid(axis='y',alpha=.2);ax.set_ylim(bottom=0)
ax=axs[1,0]
for i,g in enumerate(G):
 r=[x for x in rows if x['group']==g];p=sum(x['status']=='PASS' for x in r);acc=sum(x['accepted']=='True' for x in r)
 ax.bar(i-.16,p,.3,color='#94a3b8');ax.text(i-.16,p+.1,f'{p}/{len(r)}',ha='center')
 if g in ['B','C','C_shadow']:ax.bar(i+.16,acc,.3,color=colors[i]);ax.text(i+.16,acc+.1,f'{acc}/{len(r)}',ha='center')
ax.set_xticks(range(5),names);ax.set_ylim(0,6);ax.set_title('全部尝试：灰色=完整飞行，彩色=质量验收');ax.set_ylabel('次数')
ax=axs[1,1]
for i,(comparison,label) in enumerate([('B minus C','B − C：主动相对被动'),('C_shadow minus C','影子组 − C：补偿收益')]):
 x=next(x for x in s['paired'] if x['comparison']==comparison and x['metric']=='rmse')
 if x['mean'] is not None:
  v=x['mean']*100;lo,hi=np.array(x['bootstrap95'])*100
  ax.errorbar(v,1-i,xerr=[[v-lo],[hi-v]],fmt='o',capsize=6,color=colors[i+1]);ax.text(v,1-i+.2,f'{v:+.3f} [{lo:.3f},{hi:.3f}]，n={len(x["seeds"])}',ha='center')
ax.axvline(0,ls='--',color='#94a3b8');ax.set_yticks([1,0],['主动 vs 被动','补偿 vs 不补偿']);ax.set_ylim(-.5,1.5);ax.set_xlabel('配对 RMSE 差（cm）；正值有利于 C');ax.set_title('小样本 bootstrap 95% 区间')
fig.suptitle('第二轮实际 SITL：五组 × 五种子，参考配置 '+s['primary_attitude_reference'],fontsize=19)
fig.text(.07,.035,'本图使用同一姿态参考配置、遥测解锁门禁与恢复流程。另一姿态配置及第一轮结果另列，不混合统计。\n每个点为一次实际运行；失败保留，缺失指标不填零。区间仅描述本轮小样本，不能推广到硬件。',fontsize=10)
for ext in ['png','pdf','svg']:fig.savefig(O/('comparison.'+ext),dpi=180)
plt.close(fig)
# Fixed seed 0 illustration: never chosen by outcome score.
from pyulog import ULog
rr=next((x for x in rows if x['group']=='C' and x['seed']=='0'),None)
if rr and (Path(rr['run'])/'trajectory.csv').exists():
 p=Path(rr['run']);tr=list(csv.DictReader((p/'trajectory.csv').open()));t0=float(tr[0]['px4_time_s']);u=ULog(str(next((p/'rootfs/log').rglob('*.ulg'))));d=u.get_dataset('hnuter_control_status').data;t=d['timestamp']*1e-6-t0
 sr=list(csv.DictReader((p/'actuator_sensor.csv').open()));st=np.array([float(x['t'])-t0 for x in sr]);sf=np.array([float(x['thrust_up']) for x in sr]);tt=np.array([float(x['t']) for x in tr])
 fig,ax=plt.subplots(4,1,figsize=(13,11),sharex=True)
 ax[0].plot(t,d['effective_mass'],label='实际固件 m_eff');ax[0].axhline(5.4,ls='--',color='gray',label='质量真值（仅离线）');ax[0].set_ylabel('质量 kg');ax[0].legend()
 ax[1].plot(st,sf,lw=.7,label='实测关节重建世界向上力');ax[1].plot(t,-d['force_body[2]'],alpha=.5,lw=.7,label='期望机体向上力');ax[1].set_ylabel('力 N');ax[1].legend()
 ax[2].plot(tt,[float(x['z_ref']) for x in tr],ls='--',label='参考');ax[2].plot(tt,[float(x['z_up']) for x in tr],label='PX4估计');ax[2].set_ylabel('高度 m');ax[2].legend()
 ax[3].plot(tt,[100*float(x['error']) for x in tr]);ax[3].set_ylabel('位置误差 cm');ax[3].set_xlabel('解锁后 s')
 mask=(t>=16)&(t<=57);sm=(st>=16)&(st<=57);tm=(tt>=16)&(tt<=57)
 force_vis=np.r_[sf[sm],-d['force_body[2]'][mask]];ax[1].set_ylim(float(min(force_vis))-1,float(max(force_vis))+1)
 height_vis=np.r_[np.array([float(x['z_ref']) for x in tr])[tm],np.array([float(x['z_up']) for x in tr])[tm]];ax[2].set_ylim(float(min(height_vis))-.03,float(max(height_vis))+.03)
 ax[3].set_ylim(0,1.1*max(np.array([100*float(x['error']) for x in tr])[tm]))
 for a in ax:a.set_xlim(16,57);a.axvspan(35,39,color='orange',alpha=.12);a.grid(alpha=.2)
 fig.suptitle('第二轮 C / 种子0：质量、实际响应与更新瞬态',fontsize=18);fig.tight_layout(rect=[0,.04,1,.95]);fig.text(.08,.01,'橙色为公共更新观察窗。期望机体系力与重建世界系力坐标不同；曲线差还含姿态、执行器动态和噪声，不等同于质量切换误差。',fontsize=9)
 for ext in ['png','pdf','svg']:fig.savefig(O/('transition.'+ext),dpi=180)
 plt.close(fig)
print('New figures saved to',O)
