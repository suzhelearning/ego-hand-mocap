"""Render the README pipeline diagram with Pillow and Noto CJK fonts."""
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
BOLD = '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc'
im = Image.new('RGB', (1600, 1760), '#f4f7fb')
d = ImageDraw.Draw(im)
def font(size, bold=False):return ImageFont.truetype(BOLD if bold else FONT, size)
def text(x,y,s,size=24,color='#344256',bold=False):d.text((x,y),s,font=font(size,bold),fill=color)
def box(x,y,w,h,title,lines,state):
    color={'已验证':'#16745a','部分完成':'#9b650d','待接通':'#5866b3'}[state]
    d.rounded_rectangle((x,y,x+w,y+h),radius=18,fill='white',outline='#d3dce7',width=2)
    d.rounded_rectangle((x,y,x+7,y+h),radius=3,fill=color)
    text(x+24,y+17,title,27,'#172d48',True)
    text(x+w-144,y+22,state,22,color,True)
    for i,line in enumerate(lines):text(x+24,y+65+i*34,line)
def arrow(points):
    d.line(points,fill='#718198',width=4)
    x,y=points[-1];px,py=points[-2]
    if y>py:d.polygon([(x,y),(x-8,y-13),(x+8,y-13)],fill='#718198')
    elif x>px:d.polygon([(x,y),(x-13,y-8),(x-13,y+8)],fill='#718198')
    else:d.polygon([(x,y),(x+13,y-8),(x+13,y+8)],fill='#718198')
text(70,35,'EGO × OptiTrack：手部参考数据 Pipeline',42,'#172d48',True)
text(70,101,'采集 → 两路独立处理 → 时间 / 空间对齐 → 统一关节 → 评估',25)
text(70,143,'状态依据：2026-10-30 本地代码与产物检查；已验证不等同于物理精度已标定。',21)
box(70,205,1460,150,'Stage 0  实验准备与标定',['相机参数、头戴刚体与手部 marker 布局；明确左右手、单位和坐标定义。','相机—头戴刚体外参、RGB 曝光相位仍需完成独立标定。'],'部分完成')
arrow([(800,355),(800,385)])
box(70,385,1460,170,'Stage 1  统一采集',['默认 take：EGO 视频 / H5 → Ubuntu；Motive 原生 Take → Windows remote/<时间点>.tak','保存 session.json、双路时钟探针、控制事件和哈希。h5 为独立的刚体采集模式。','已验证开始 / 停止；不支持同一 Take 的暂停续录。'],'已验证')
arrow([(420,555),(420,625)]);arrow([(1180,555),(1180,585)])
box(70,625,700,235,'Stage 3A  视觉预测路线',['EGO RGB + 原始逐帧时间戳','预处理 → 检测 / 时序推理 → 手部预测','推理源码、detector、权重和 MANO 已到位','本会话推理 / 输出接口接入待验证'],'待接通')
box(830,585,700,160,'Stage 2  Label 与 C3D 导出',['Motive 人工标注 → C3D → 归档 / 哈希校验','保留缺失和裁剪信息；本条仅 Label、未补点'],'已验证')
arrow([(1180,745),(1180,790)])
box(830,790,700,215,'Stage 3B  动捕参考路线',['20 个左手 marker → MoSh++ Stage I / II','已生成 1328 帧 MANO 参数及 778 顶点网格','7 个 EGO marker 留给头戴刚体位姿估计','统一 21 点导出仍待接入'],'部分完成')
d.line([(420,860),(420,1025),(1180,1025),(1180,1005)],fill='#718198',width=4)
arrow([(420,1025),(420,1045)]);arrow([(1180,1025),(1180,1045)])
box(70,1045,700,195,'Stage 4  时间对齐',['双路时钟映射；补齐 Take / C3D 时间锚点','以 RGB 为基准最近邻配对，不插值标签','保留时间差、覆盖范围和缺失有效性'],'部分完成')
box(830,1045,700,195,'Stage 5  空间与关节统一',['头戴位姿 + 固定外参 → 相机坐标','统一米制、左右手、MANO 与 21 点顺序','区分世界坐标、相机坐标和腕部相对坐标'],'待接通')
arrow([(420,1240),(420,1280),(800,1280),(800,1320)])
arrow([(1180,1240),(1180,1280),(800,1280)])
box(70,1320,1460,190,'Stage 6  可视化与评估',['RGB 双路投影、三维模型叠加、逐帧误差曲线及质量掩码。','分别报告绝对位置误差与腕部对齐误差；完整预测 / 参考评估尚未接通。','marker 拟合残差不是关节真值误差；缺失帧的模型输出属于估计。'],'部分完成')
text(70,1560,'统一归档：data/sessions/<session_id>/',27,'#172d48',True)
text(70,1610,'ego.h5（视觉 / OKVIS） · mocap.h5（marker / MANO） · aligned.h5（对齐 / 评估）',24)
text(70,1675,'当前待补：视觉结果接入 / Take 时间锚点 / 外参独立验证 / 统一 21 点导出',24,'#9b650d',True)
out=ROOT/'docs/images/pipeline.png';out.parent.mkdir(parents=True,exist_ok=True);im.save(out)
print(out)
