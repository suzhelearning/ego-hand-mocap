"""Render the reference/algorithm routes and fixed cam0-FLU comparison."""
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
root=Path(__file__).resolve().parents[1]
im=Image.new('RGB',(1600,1240),'#f4f7fb');d=ImageDraw.Draw(im)
def text(x,y,s,size=25,color='#263a52'):d.text((x,y),s,font=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',size),fill=color)
def box(x,y,w,h,title,lines):
 d.rounded_rectangle((x,y,x+w,y+h),16,fill='white',outline='#8da7bd',width=2);text(x+20,y+15,title,29)
 for i,l in enumerate(lines):text(x+20,y+63+i*36,l,23)
def arrow(x,y,xx,yy):
 d.line((x,y,xx,yy),fill='#67849c',width=4);d.polygon([(xx,yy),(xx-8,yy-13),(xx+8,yy-13)],fill='#67849c')
text(60,28,'EGO × Mocap：mocap 虚影与 SLAM / 视觉手实体',40)
text(60,92,'2026-11-02｜统一时空与定义；保留参考质量、缺失和算法偏差',25)
box(60,150,1480,145,'Stage 0–2  标定 → 联合采集 → 本地汇总',['安装外参 / 时间锚 / 左右手与关节定义；EGO 视频、IMU、H5 + Motive Take 或 NatNet H5','Take 标注后导出 C3D；保留原始时间、索引、探针、质量与资产哈希'])
arrow(420,295,420,345);arrow(1180,295,1180,345)
box(60,345,710,200,'Stage 3A  Mocap 参考路线',['EGO marker → 设备刚体位姿 T_WB','Hand marker → 手部参考 / MANO / 21 点','参考本身保留标注、跟踪与拟合质量'])
box(830,345,710,200,'Stage 3B  EGO 算法路线',['双目 RGB + IMU → cam0 FLU 位姿 T_LF','RGB → 相机系视觉手 p_C','SLAM 与视觉手分别保存有效性'])
arrow(420,545,420,590);arrow(1180,545,1180,590)
box(60,590,1480,145,'Stage 4  统一时间、空间与语义',['固定世界变换 + 固定安装外参；统一设备原点、米制、左右手与关节顺序','SLAM 首个有效时刻对应动捕位姿 → 固定 T_WL；后续不再用动捕修正算法'])
arrow(800,735,800,790)
box(60,790,710,220,'Stage 5 · mocap-benchmark 虚影',['设备刚体 T_WB_mocap','手部轨迹 / MANO 参考','后续增加 obj；保留参考质量'])
box(830,790,710,220,'Stage 5 · EGO 算法实体',['设备：T_WL × T_LF × T_FB','手：T_WL × T_LF × T_FC × p_C','视觉手始终跟随 SLAM，保留漂移和偏差'])
text(60,1050,'归档：ego.h5（传感器/视觉/SLAM） · mocap.h5（参考） · aligned.h5（配对/配准/比较）',25)
text(60,1100,'当前示例：固定初始化统一对比已接通；安装外参和时间仍为同会话经验拟合，非独立精度认证。',25,'#915c12')
text(60,1150,'pipeline run：视觉 → SLAM → MANO → 对齐导出；阶段指纹续跑，三 H5 直接重建 Web。',25,'#915c12')
out=root/'docs/images/pipeline.png';out.parent.mkdir(parents=True,exist_ok=True);im.save(out);print(out)
