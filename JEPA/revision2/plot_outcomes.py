"""Show all fifty formal outcomes, including failures with accurate estimates."""
import csv,json,os
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parent;O=R/'results';os.environ['MPLCONFIGDIR']=str(O/'mpl_cache')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.colors import ListedColormap
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams.update({'font.family':font_manager.FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc').get_name(),'axes.unicode_minus':False})
rows=[r for r in csv.DictReader((O/'all_attempts.csv').open()) if r['batch'].startswith('batch_matrix_')]
G=['A','B','C','D','C_shadow'];names=['A 固定','B 被动','C 主动','D 理想','主动不补偿']
fig,axs=plt.subplots(1,2,figsize=(14,7));fig.subplots_adjust(left=.09,right=.97,wspace=.32,top=.81,bottom=.17)
for ax,ref in zip(axs,['legacy','level']):
 matrix=np.full((5,5),4.);labels=[['未完成']*5 for _ in G]
 for i,g in enumerate(G):
  for j in range(5):
   r=next((r for r in rows if r['group']==g and int(r['seed'])==j and r['attitude_reference']==ref),None)
   if r is None:continue
   if r['status']!='PASS':matrix[i,j]=3;labels[i][j]='退出\n'+('落地上锁' if r['landed']=='True' and r['disarmed']=='True' else '恢复未确认')
   elif g in ['A','D']:matrix[i,j]=0;labels[i][j]='完成\n不辨识'
   elif r['accepted']=='True':matrix[i,j]=1;labels[i][j]='完成\n已验收'
   else:matrix[i,j]=2;labels[i][j]='完成\n校准拒绝'
 ax.imshow(matrix,cmap=ListedColormap(['#e2e8f0','#bce7df','#fde6b6','#f6c7c7','#f8fafc']),vmin=-.5,vmax=4.5,aspect='auto')
 for i in range(5):
  for j in range(5):ax.text(j,i,labels[i][j],ha='center',va='center',fontsize=10)
 ax.set_xticks(range(5),[f'种子 {i}' for i in range(5)]);ax.set_yticks(range(5),names);ax.set_title('原普通位置入口（主要对照）' if ref=='legacy' else '显式水平参考（独立对照）',pad=20,fontsize=14)
 ax.set_xticks(np.arange(-.5,5,1),minor=True);ax.set_yticks(np.arange(-.5,5,1),minor=True);ax.grid(which='minor',color='white',lw=4);ax.tick_params(which='minor',bottom=False,left=False)
fig.suptitle('第二轮完整试验矩阵：所有尝试都保留',fontsize=21)
fig.text(.08,.055,'每个配置 5 组 × 5 种子；相同固件、增益、负载、公共轨迹与保护门限。两种姿态参考不混合统计。\n退出仍算飞行失败，即使恢复已成功、质量曾被准确估计。注入恢复与缺失遥测负向测试另列。',fontsize=11)
for ext in ['png','pdf','svg']:fig.savefig(O/('outcomes.'+ext),dpi=180)
plt.close(fig)
