import agibot_gdk
import time

# slam = agibot_gdk.Slam()
pnc = agibot_gdk.Pnc()
time.sleep(1)  # 等待初始化

target = agibot_gdk.NaviReq()
target.target.position.x = 9.1016431138415435
target.target.position.y =  -4.1492533213758431
target.target.position.z = -0.056417739896994348
target.target.orientation.x = -0.00075885244457369618
target.target.orientation.y = 0.0015111685240119493
target.target.orientation.z = -0.61837471643313713
target.target.orientation.w = 0.7858815754227203
pnc.normal_navi(target)
