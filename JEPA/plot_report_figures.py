"""Chinese scientific figures from existing SITL evidence; no simulator/control calls."""
import csv
import hashlib
import json
import os
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'results/figures_cn'
OUT.mkdir(parents=True, exist_ok=True)
os.environ['MPLCONFIGDIR'] = str(ROOT / 'results/mpl_cache')
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch
from pyulog import ULog

FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
font_manager.fontManager.addfont(FONT)
plt.rcParams.update({'font.family':font_manager.FontProperties(fname=FONT).get_name(),
    'font.size':11,'axes.titlesize':13,'axes.labelsize':11,'legend.fontsize':9,
    'axes.spines.top':False,'axes.spines.right':False,'axes.edgecolor':'#cbd5e1',
    'axes.labelcolor':'#334155','text.color':'#172554','xtick.color':'#475569',
    'ytick.color':'#475569','figure.facecolor':'white','savefig.facecolor':'white',
    'axes.unicode_minus':False,'pdf.fonttype':42,'svg.fonttype':'path'})
GROUPS = ['A','B','C','D','C_shadow']
NAMES = ['A\n固定标称','B\n被动补偿','C\n主动补偿','D\n已知质量','影子组\n主动不补偿']
COLORS = ['#64748b','#2563eb','#0891b2','#8b5cf6','#d97706']
sources = set()
outputs = []


def read_csv(path):
    path=Path(path);sources.add(str(path))
    with path.open() as f:return list(csv.DictReader(f))


def read_json(path):
    path=Path(path);sources.add(str(path));return json.loads(path.read_text())


ROWS=read_csv(ROOT/'results/all_trials.csv')
SUMMARY=read_json(ROOT/'results/summary.json')
NOISY=[r for r in ROWS if r['phase']=='noisy']


def run(group,seed=0,phase='noisy'):
    return Path(next(r['run'] for r in ROWS if r['phase']==phase and r['group']==group and int(r['seed'])==seed))


def grid(ax):ax.grid(axis='y',color='#e2e8f0',linewidth=.7);ax.set_axisbelow(True)


def save(fig,name,title,foot):
    fig.suptitle(title,fontsize=20,fontweight='bold',x=.055,ha='left',y=.98)
    fig.text(.055,.017,foot,fontsize=10,color='#475569',va='bottom')
    for ext in ('png','svg','pdf'):
        p=OUT/f'{name}.{ext}';fig.savefig(p,dpi=180);outputs.append(p)
    plt.close(fig)


def overview():
    fig,axs=plt.subplots(2,2,figsize=(15,10))
    fig.subplots_adjust(left=.08,right=.97,top=.89,bottom=.12,wspace=.33,hspace=.49)
    for ax,key,title in [(axs[0,0],'position_rmse_m','① 公共评价：PX4 估计位置误差'),
                         (axs[0,1],'truth_position_rmse_m','② 公共评价：对齐后的 Gazebo 真值误差')]:
        for i,g in enumerate(GROUPS):
            vals=[float(r[key])*100 for r in NOISY if r['group']==g and r[key]]
            assert len(vals) in (4,5)
            ax.scatter(i+np.linspace(-.12,.12,len(vals)),vals,s=45,color=COLORS[i],zorder=4)
            ax.errorbar(i,np.mean(vals),yerr=np.std(vals,ddof=1),fmt='_',ms=20,color='#0f172a',capsize=5,lw=1.3,zorder=5)
            ax.text(i,1.015,f'n={len(vals)}/5',ha='center',transform=ax.get_xaxis_transform(),fontsize=9)
        ax.set_title(title,pad=31,loc='left');ax.set_ylabel('三维位置 RMSE（cm）')
        ax.set_xticks(range(5),NAMES);ax.set_ylim(bottom=0);grid(ax)
    ax=axs[1,0];x=np.arange(5)
    complete=[sum(r['flight_pass']=='True' for r in NOISY if r['group']==g) for g in GROUPS]
    accept=[sum(r['mass_accepted']=='True' for r in NOISY if r['group']==g) for g in GROUPS]
    ax.bar(x-.17,complete,width=.32,color='#94a3b8',label='完整飞行成功')
    for i,g in enumerate(GROUPS):
        ax.text(i-.17,complete[i]+.1,f'{complete[i]}/5',ha='center',fontsize=10)
        if g in ('B','C','C_shadow'):
            ax.bar(i+.17,accept[i],width=.32,color=COLORS[i]);ax.text(i+.17,accept[i]+.1,f'{accept[i]}/5',ha='center',fontsize=10)
        else:ax.text(i+.17,.15,'不适用',rotation=90,ha='center',fontsize=8,color='#64748b')
    ax.set_title('③ 全部 25 次尝试：成功和验收分开统计',loc='left');ax.set_xticks(x,NAMES)
    ax.set_ylim(0,6.3);ax.set_yticks(range(6));ax.set_ylabel('次数 / 每组 5 次');grid(ax)
    ax.legend(handles=[Patch(facecolor='#94a3b8',label='完整飞行'),Patch(facecolor=COLORS[2],label='质量验收')],ncol=2,frameon=False,loc='upper left')
    ax=axs[1,1]
    for i,(contrast,label) in enumerate([('B minus C','主动相对被动\nB − C，配对 n=4'),('C_shadow minus C','补偿相对不补偿\n影子组 − C，配对 n=3')]):
        c=next(c for c in SUMMARY['paired_contrasts'] if c['metric']=='position_rmse_m' and c['comparison']==contrast)
        value=c['mean_difference_m']*100;lo,hi=np.array(c['bootstrap_95pct_interval_m'])*100
        ax.errorbar(value,1-i,xerr=[[value-lo],[hi-value]],fmt='o',color=COLORS[2+i*2],capsize=7,ms=8,lw=2)
        ax.text(value,1-i+.20,f'{value:+.3f} cm  [{lo:.3f}, {hi:.3f}]',ha='center',fontsize=10)
    ax.axvline(0,color='#94a3b8',ls='--');ax.set_yticks([1,0],['主动 vs 被动\n共同完成 4 对','补偿 vs 不补偿\n共同完成 3 对'])
    ax.set_ylim(-.55,1.65);ax.set_xlim(-1.1,1.55);ax.set_xlabel('前组 RMSE − 后组 RMSE（cm）；正值有利于 C')
    ax.set_title('④ 主动优势与补偿收益：区间均跨过零',loc='left');ax.grid(axis='x',alpha=.2)
    save(fig,'01_overview','负载 +20%：精度、可靠性和收益一起看',
         '实际 PX4/Gazebo SITL；固定增益；每组 5 个噪声/延迟种子。点为单次结果，黑线为均值 ± 样本标准差。\n上排只含完成子集：3 次未完成，其中 2 次未确认落地。右下为小样本配对 bootstrap 95% 区间，仅描述本场景。')


def mass_and_outcomes():
    fig=plt.figure(figsize=(15,9));gs=fig.add_gridspec(2,2,left=.08,right=.96,top=.87,bottom=.14,wspace=.32,hspace=.50)
    ax=fig.add_subplot(gs[0,0])
    nominal=ROOT/'runs/sitl_nominal_sensor_shadow_final_20261010_173321'
    for p,true,col,label in [(nominal,4.49999998,COLORS[0],'标称 4.5 kg'),(run('C',phase='low'),5.39999998,COLORS[2],'加载 5.4 kg')]:
        tr=read_csv(p/'trajectory.csv');w=read_csv(p/'mass_windows.csv')
        start=float(next(r['px4_time_s'] for r in tr if r['phase']=='CALIBRATION'))
        ax.plot([float(r['t'])-start for r in w],[float(r['mass_kg']) for r in w],color=col,lw=2,label=label)
        ax.axhline(true,color=col,ls='--',alpha=.6)
        m=read_json(p/'metrics.json')['mass'];ax.annotate(f"{m['estimate_kg']:.5f} kg",(15,m['estimate_kg']),xytext=(-8,10),textcoords='offset points',ha='right',color=col)
    ax.set_xlim(0,16.3);ax.set_ylim(4.35,5.57);ax.set_title('① 低噪声交叉检查：不同负载得到不同估计',loc='left')
    ax.set_xlabel('校准开始后时间（s）');ax.set_ylabel('质量（kg）');ax.legend(frameon=False,loc='center right');grid(ax)
    ax=fig.add_subplot(gs[1,0])
    for i,g in enumerate(['B','C','C_shadow']):
        rr=[r for r in NOISY if r['group']==g and r['mass_accepted']=='True'];v=[float(r['mass_error_percent']) for r in rr]
        ax.scatter(i+np.linspace(-.06,.06,len(v)),v,s=55,color=COLORS[GROUPS.index(g)])
        ax.text(i,.29,f'验收 {len(rr)}/5\nMAE={np.mean(np.abs(v)):.3f}%',ha='center',fontsize=10)
    ax.axhline(0,color='#94a3b8',ls='--');ax.set_xticks(range(3),['B 被动','C 主动','影子组']);ax.set_ylim(-.30,.44)
    ax.set_xlim(-.4,2.4);ax.set_ylabel('有符号质量误差（%）');ax.set_title('② 仅已接受子集的精度：同时标明验收率',loc='left');grid(ax)
    ax=fig.add_subplot(gs[:,1]);matrix=np.zeros((5,5));labels=[]
    for i,g in enumerate(GROUPS):
        ll=[]
        for seed in range(5):
            r=next(r for r in NOISY if r['group']==g and int(r['seed'])==seed)
            if r['flight_pass']!='True':status=3;label='触发保护\n'+('已上锁' if r['landing_confirmed']=='True' else '落地未确认')
            elif r['mass_accepted']=='True':status=1;label='已验收\n'+('不补偿' if g=='C_shadow' else '已补偿')
            elif g in ('B','C','C_shadow'):status=2;label='遥测缺失\n拒绝校准'
            else:status=0;label='完成\n不辨识'
            if r['sensor_samples']=='0' and status==0:label='完成*\n不辨识'
            matrix[i,seed]=status;ll.append(label)
        labels.append(ll)
    cmap=ListedColormap(['#e2e8f0','#bce7df','#fde6b6','#f6c7c7'])
    ax.imshow(matrix,cmap=cmap,vmin=-.5,vmax=3.5,aspect='auto')
    for i in range(5):
        for j in range(5):ax.text(j,i,labels[i][j],ha='center',va='center',fontsize=10,color='#243447')
    ax.set_xticks(range(5),[f'种子 {i}' for i in range(5)]);ax.set_yticks(range(5),NAMES)
    ax.set_xticks(np.arange(-.5,5,1),minor=True);ax.set_yticks(np.arange(-.5,5,1),minor=True);ax.grid(which='minor',color='white',lw=4);ax.tick_params(which='minor',bottom=False,left=False)
    ax.set_title('③ 全部噪声试验：失败不删除',loc='left',pad=20)
    save(fig,'02_mass_and_outcomes','质量可以辨识；能否取得有效数据同样关键',
         '左上：实际低噪声日志，虚线为离线评价真值，未输入在线估计器。左下：只含验收子集，不代表总体精度。\n右侧：A/D 无需辨识；* 表示关节遥测缺失。3 次触发保护均在更新前，其中 2 次未确认落地。')


def transition():
    p=run('C');tr=read_csv(p/'trajectory.csv');w=read_csv(p/'mass_windows.csv')
    log=next((p/'rootfs/log').rglob('*.ulg'));sources.add(str(log));d=ULog(str(log)).get_dataset('hnuter_control_status').data
    t0=float(tr[0]['px4_time_s']);t=d['timestamp']*1e-6-t0;mask=(t>=16)&(t<=57)
    rr=[r for r in tr if 16<=float(r['t'])<=57];a=lambda k:np.array([float(r[k]) for r in rr])
    fig,axs=plt.subplots(4,1,figsize=(14,12),sharex=True)
    fig.subplots_adjust(left=.10,right=.96,top=.89,bottom=.12,hspace=.25)
    axs[0].plot([float(r['t'])-t0 for r in w],[float(r['mass_kg']) for r in w],lw=2,color=COLORS[1],label='独立估计质量')
    axs[0].plot(t[mask],d['effective_mass'][mask],lw=2,color='#ea580c',label='固件有效质量 m_eff')
    axs[0].axhline(5.39999998,ls='--',color='#64748b',label='真实质量（仅离线评价）');axs[0].set_ylabel('质量（kg）');axs[0].legend(ncol=3,frameon=False,loc='lower right')
    axs[1].plot(a('t'),a('z_ref'),color='#334155',ls='--',label='解析高度参考')
    axs[1].plot(a('t'),a('z_up'),color=COLORS[2],lw=1.2,label='PX4 估计高度');axs[1].set_ylabel('高度（m）');axs[1].legend(ncol=2,frameon=False)
    axs[2].plot(t[mask],-d['force_body[2]'][mask],color=COLORS[2],lw=.65);axs[2].set_ylabel('期望机体向上力（N）')
    axs[3].plot(a('t'),100*a('error'),color=COLORS[1],lw=1.3);axs[3].set_ylabel('三维位置误差（cm）');axs[3].set_xlabel('解锁后时间（s）')
    for ax in axs:
        ax.axvspan(18,34,color='#dbeafe',alpha=.45);ax.axvspan(35,38.1,color='#ffedd5',alpha=.65);ax.axvspan(40,56,color='#dcfce7',alpha=.26)
        ax.set_xlim(16,57);grid(ax)
    axs[0].text(26,1.10,'校准 18–34 s',transform=axs[0].get_xaxis_transform(),ha='center',color=COLORS[1])
    axs[0].text(36.5,1.10,'质量平滑更新',transform=axs[0].get_xaxis_transform(),ha='center',color='#c2410c')
    axs[0].text(48,1.10,'公共评价 40–56 s',transform=axs[0].get_xaxis_transform(),ha='center',color='#15803d')
    save(fig,'03_transition','真实闭环过程：先辨识，再平滑改变固件质量',
         '固定示例：噪声组 C / seed 0；展示 16–57 s，未平滑原始数据；完整运行和其他种子另有日志。\n第三幅为控制器期望力，仅用于观察更新瞬态，未用于质量估计。该示例的质量转换代数力误差 < 8e-6 N。')


def trajectory():
    fig,axs=plt.subplots(5,2,figsize=(14,13),sharex=True,sharey='col')
    fig.subplots_adjust(left=.10,right=.97,top=.89,bottom=.10,hspace=.27,wspace=.20)
    for i,g in enumerate(GROUPS):
        rr=[r for r in read_csv(run(g)/'trajectory.csv') if r['phase']=='EVALUATION']
        a=lambda k:np.array([float(r[k]) for r in rr]);t=a('t')-40
        axs[i,0].plot(t,a('z_ref'),color='#475569',ls='--',lw=1.1,label='参考')
        axs[i,0].plot(t,a('z_up'),color=COLORS[i],lw=1.3,label='PX4 估计高度')
        axs[i,1].plot(t,a('error')*100,color=COLORS[i],lw=1.2)
        axs[i,0].set_ylabel(NAMES[i]+'\n高度（m）');axs[i,1].set_ylabel('位置误差（cm）')
        for ax in axs[i]:ax.set_xlim(0,16);grid(ax)
    axs[0,0].set_title('同一解析高度参考',loc='left',pad=18);axs[0,0].legend(ncol=2,frameon=False)
    axs[0,1].set_title('三维跟踪误差（PX4 状态估计）',loc='left',pad=18)
    axs[-1,0].set_xlabel('公共评价开始后时间（s）');axs[-1,1].set_xlabel('公共评价开始后时间（s）')
    save(fig,'04_shared_trajectory','五组公共测试轨迹：相同种子 0、相同 16 秒评价窗口',
         '采用所有组都完成的 seed 0，仅用于查看轨迹形态；不代表五种子总体表现。每列使用相同坐标尺度。\n曲线为原始日志，不做滤波。PX4 估计位置与 Gazebo 真值指标分别报告，不能互相替代。')


def main():
    overview();mass_and_outcomes();transition();trajectory()
    manifest={'scope':'Existing actual SITL records only; no simulation rerun or data modification',
              'sources':{p:hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in sorted(sources)},
              'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'outputs':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in outputs}}
    (OUT/'manifest.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n')
    titles=['五组总体对比','质量辨识与逐次成败','固件质量更新瞬态','五组公共轨迹']
    names=['01_overview','02_mass_and_outcomes','03_transition','04_shared_trajectory']
    text=['# 本轮 SITL 中文图集','', '数据来自已有实际运行日志；没有重新启动仿真。原始报告及失败记录保持不变。','',
          '复现：`python3 /home/hnuter/px4_ws_ros2/JEPA/plot_report_figures.py`','',
          '每张图提供 PNG、SVG 和 PDF。点、样本量、条件子集和失败都在图中标明。', '']
    for title,name in zip(titles,names):
        text.extend([f'## {title}','',f'![{title}]({OUT/name}.png)','',f'[SVG]({OUT/name}.svg) · [PDF]({OUT/name}.pdf)',''])
    (OUT/'README.md').write_text('\n'.join(text)+'\n')
    with zipfile.ZipFile(OUT/'figures_bundle.zip','w',compression=zipfile.ZIP_DEFLATED) as z:
        for p in outputs+[OUT/'README.md',OUT/'manifest.json',Path(__file__)]:z.write(p,p.name)
    print(json.dumps({'output_directory':str(OUT),'figures':len(names),'formats':['png','svg','pdf'],'source_files':len(sources)},ensure_ascii=False))


if __name__=='__main__':main()
