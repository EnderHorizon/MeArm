import math
U=F=8.0; BH=6.0; CL=6.3; W=0.4; BZ=4.64
SH=122.0; EL=50.0
minR,maxR=11.0,22.5
b_rng=(0,180); sh_rng=(20,120); el_rng=(50,145)
STANDBY=(90,20,95); PEN=5.0
SPEED=60.0   # 舵机速度 度/秒 (构造默认)

def PosToAngle(x,y,z):
    d=math.hypot(x,y)
    if d<minR or d>maxR: return STANDBY
    L=math.sqrt(d*d-W*W)-CL
    base=math.atan2(y,x)+math.asin(W/d)+math.radians(BZ)
    h=z-BH; dd=math.sqrt(L*L+h*h)
    sh=math.acos((U*U+L*L+h*h-F*F)/(2*U*dd))+math.atan2(h,L)
    el=sh+math.acos((U*U+F*F-(L*L+h*h))/(2*U*F))
    return (math.degrees(base),SH-math.degrees(sh),math.degrees(el)-EL)

def FK(a,b,c):
    b=SH-b; c=c+EL; b,c=math.radians(b),math.radians(c)
    L=math.sqrt(U*U+F*F-2*U*F*math.cos(c-b))
    a1=math.acos((U*U+L*L-F*F)/(2*U*L)); a2=b-a1
    D=L*math.cos(a2); h=L*math.sin(a2)
    a=math.radians(a-BZ)-math.atan2(W,D+CL)
    Length=math.sqrt(W*W+(D+CL)**2)
    return (Length*math.cos(a),Length*math.sin(a),BH+h)

pts=[(10,15),(7,18),(0,15),(-6,13),(-10,17)]
# 折线161个waypoint的角度(float)
wps=[]
for i in range(4):
    x0,y0=pts[i]; x1,y1=pts[i+1]
    for k in range(40):
        t=k/40; wps.append(PosToAngle(x0+(x1-x0)*t, y0+(y1-y0)*t, PEN))
wps.append(PosToAngle(*pts[4],PEN))
teach_idx=[0,40,80,120,160]

# 存入strd时被uint8_t截断
wps_u8=[(int(a),int(b),int(c)) for (a,b,c) in wps]

print("=== A) uint8_t截断造成的位置误差(在5个示教点) ===")
for idx in teach_idx:
    ideal=wps[idx]; trunc=wps_u8[idx]
    p_i=FK(*ideal); p_t=FK(*trunc)
    e=math.hypot(p_t[0]-p_i[0],p_t[1]-p_i[1])
    print(f"  示教点{teach_idx.index(idx)+1}: float角({ideal[0]:.1f},{ideal[1]:.1f},{ideal[2]:.1f}) -> 截断({trunc[0]},{trunc[1]},{trunc[2]})  位置差{e*10:.1f}mm")

print()
print("=== B) 播放动态: 舵机以60°/s追踪waypoint, 喂入间隔=loop周期 ===")
def sim_play(dt):
    cur=list(wps_u8[0])          # 从第一个点起
    err_at_teach=[]; maxdev=0
    for n,tgt in enumerate(wps_u8):
        # 每个loop: 先更新舵机(朝当前target动), 这里target就是上一个已喂入的
        # 简化为: 喂入新target后, 舵机用一步update逼近
        for j in range(3):
            lo,hi=[b_rng,sh_rng,el_rng][j]
            step=SPEED*dt
            if tgt[j]>cur[j]: cur[j]=min(tgt[j],cur[j]+step)
            elif tgt[j]<cur[j]: cur[j]=max(tgt[j],cur[j]-step)
        if n in teach_idx:
            p_t=FK(*tgt); p_a=FK(*cur)
            e=math.hypot(p_a[0]-p_t[0],p_a[1]-p_t[1])
            err_at_teach.append(e)
    return err_at_teach

for dt in [0.015,0.03,0.06]:
    errs=sim_play(dt)
    print(f"  loop周期{int(dt*1000)}ms: 各示教点跟踪误差(mm) = {[round(e*10,1) for e in errs]}")

print()
print("=== C) 叠加舵机物理精度(±1°)后的示教点命中误差 ===")
import random
random.seed(1)
for trial in range(3):
    errs=[]
    for idx in teach_idx:
        tgt=wps_u8[idx]
        jittered=tuple(tgt[j]+random.uniform(-1,1) for j in range(3))
        p_t=FK(*tgt); p_a=FK(*jittered)
        errs.append(math.hypot(p_a[0]-p_t[0],p_a[1]-p_t[1])*10)
    print(f"  试验{trial+1}: 示教点误差(mm) = {[round(e,1) for e in errs]}  最大{max(errs):.1f}mm")
