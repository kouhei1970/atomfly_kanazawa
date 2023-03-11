#%matplotlib widget
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from matplotlib.widgets import TextBox
import numpy as np
import control.matlab as matlab

#コプター諸元
W = (0.04302-0.00169)*9.8
Ct=0.57e-7
Km =371.5465478054084
Tm=0.03085581296207573
#mm=W*0.7/2
larm=0.033
#Iy=2*mm*larm**2
Iy=2.19e-5
Tf=50


# Figureの設定
fig = plt.figure(figsize=(13,9))
axcolor = 'gold'
gs = fig.add_gridspec(28, 33)

# グラフ描画位置の設定
ax_openloop_gain = fig.add_subplot(gs[0:4,0:10])
ax_openloop_phase = fig.add_subplot(gs[5:9,0:10])
ax_closeloop_gain = fig.add_subplot(gs[10:14,0:10])
ax_closeloop_phase = fig.add_subplot(gs[15:19,0:10])
ax_openloop_gain2 = fig.add_subplot(gs[0:4,12:22])
ax_openloop_phase2 = fig.add_subplot(gs[5:9,12:22])
ax_closeloop_gain2 = fig.add_subplot(gs[10:14,12:22])
ax_closeloop_phase2 = fig.add_subplot(gs[15:19,12:22])

ax_slider_k = fig.add_subplot(gs[20,0:8], facecolor=axcolor)
ax_slider_t = fig.add_subplot(gs[21,0:8], facecolor=axcolor)
ax_slider_iy = fig.add_subplot(gs[22,0:8], facecolor=axcolor)
ax_slider_f = fig.add_subplot(gs[23,0:8], facecolor=axcolor)
ax_slider_p = fig.add_subplot(gs[24,0:8], facecolor=axcolor)
ax_slider_i = fig.add_subplot(gs[25,0:8], facecolor=axcolor)
ax_slider_d = fig.add_subplot(gs[26,0:8], facecolor=axcolor)
ax_eta= fig.add_subplot(gs[27,0:8], facecolor=axcolor)
ax_slider_p2 = fig.add_subplot(gs[20,12:20], facecolor=axcolor)
ax_slider_i2 = fig.add_subplot(gs[21,12:20], facecolor=axcolor)
ax_slider_d2 = fig.add_subplot(gs[22,12:20], facecolor=axcolor)
ax_eta2= fig.add_subplot(gs[23,12:20], facecolor=axcolor)

ax_step = fig.add_subplot(gs[0:4,24:33])
ax_step2 = fig.add_subplot(gs[5:9,24:33])

#PM GMの表示部分
ax_gcf = fig.add_subplot(gs[11,26:32],facecolor="white")
ax_gcf.spines['right'].set_visible(False)
ax_gcf.spines['left'].set_visible(False)
ax_gcf.spines['top'].set_visible(False)
ax_gcf.spines['bottom'].set_visible(False)
ax_gcf.tick_params(labelbottom=False, labelleft=False, labelright=False, labeltop=False, bottom=False, left=False, right=False, top=False)

ax_pm = fig.add_subplot(gs[12,26:32],facecolor="white")
ax_pm.spines['right'].set_visible(False)
ax_pm.spines['left'].set_visible(False)
ax_pm.spines['top'].set_visible(False)
ax_pm.spines['bottom'].set_visible(False)
ax_pm.tick_params(labelbottom=False, labelleft=False, labelright=False, labeltop=False, bottom=False, left=False, right=False, top=False)

ax_pcf = fig.add_subplot(gs[13,26:32],facecolor="white")
ax_pcf.spines['right'].set_visible(False)
ax_pcf.spines['left'].set_visible(False)
ax_pcf.spines['top'].set_visible(False)
ax_pcf.spines['bottom'].set_visible(False)
ax_pcf.tick_params(labelbottom=False, labelleft=False, labelright=False, labeltop=False, bottom=False, left=False, right=False, top=False)

ax_gm = fig.add_subplot(gs[14,26:32],facecolor="white")
ax_gm.spines['right'].set_visible(False)
ax_gm.spines['left'].set_visible(False)
ax_gm.spines['top'].set_visible(False)
ax_gm.spines['bottom'].set_visible(False)
ax_gm.tick_params(labelbottom=False, labelleft=False, labelright=False, labeltop=False, bottom=False, left=False, right=False, top=False)

ax_gcf2 = fig.add_subplot(gs[16,26:32],facecolor="white")
ax_gcf2.spines['right'].set_visible(False)
ax_gcf2.spines['left'].set_visible(False)
ax_gcf2.spines['top'].set_visible(False)
ax_gcf2.spines['bottom'].set_visible(False)
ax_gcf2.tick_params(labelbottom=False, labelleft=False, labelright=False, labeltop=False, bottom=False, left=False, right=False, top=False)

ax_pm2 = fig.add_subplot(gs[17,26:32],facecolor="white")
ax_pm2.spines['right'].set_visible(False)
ax_pm2.spines['left'].set_visible(False)
ax_pm2.spines['top'].set_visible(False)
ax_pm2.spines['bottom'].set_visible(False)
ax_pm2.tick_params(labelbottom=False, labelleft=False, labelright=False, labeltop=False, bottom=False, left=False, right=False, top=False)

ax_pcf2 = fig.add_subplot(gs[18,26:32],facecolor="white")
ax_pcf2.spines['right'].set_visible(False)
ax_pcf2.spines['left'].set_visible(False)
ax_pcf2.spines['top'].set_visible(False)
ax_pcf2.spines['bottom'].set_visible(False)
ax_pcf2.tick_params(labelbottom=False, labelleft=False, labelright=False, labeltop=False, bottom=False, left=False, right=False, top=False)

ax_gm2 = fig.add_subplot(gs[19,26:32],facecolor="white")
ax_gm2.spines['right'].set_visible(False)
ax_gm2.spines['left'].set_visible(False)
ax_gm2.spines['top'].set_visible(False)
ax_gm2.spines['bottom'].set_visible(False)
ax_gm2.tick_params(labelbottom=False, labelleft=False, labelright=False, labeltop=False, bottom=False, left=False, right=False, top=False)


#ax3 = fig.add_subplot(gs[23:31,0:22])
#ax4 = fig.add_subplot(gs[32:40,0:22])

# Sliderの設定
k_m = Slider(ax_slider_k, 'Km', Km/10, Km*10, valinit=Km, valstep=1)
i_y = Slider(ax_slider_iy, 'Iy', Iy/10, Iy*10, valinit=Iy, valstep=1e-7, valfmt="%6.3e")
t_m = Slider(ax_slider_t, 'Tm', Tm/10, Tm*10, valinit=Tm, valstep=1e-4)
t_f = Slider(ax_slider_f, 'Tf', 5, 250, valinit=Tf, valstep=1)

k_p = Slider(ax_slider_p, 'P', 0.1, 3.0, valinit=0.65, valstep=0.01)
t_i = Slider(ax_slider_i, 'I', 0.0, 2.0, valinit=0.7, valstep=0.01)
t_d = Slider(ax_slider_d, 'D', 0.01, 0.05, valinit=0.03, valstep=0.001)
eta = Slider(ax_eta, 'Eta', 0.01, 0.2, valinit=0.125, valstep=0.001)

k_p2 = Slider(ax_slider_p2, 'P', 1, 25, valinit=12, valstep=0.1)
t_i2 = Slider(ax_slider_i2, 'I', 0.1, 5, valinit=4, valstep=0.0001)
t_d2 = Slider(ax_slider_d2, 'D', 0.00, 0.05, valinit=0.04, valstep=0.0001)
eta2 = Slider(ax_eta2, 'Eta', 0.01, 0.2, valinit=0.125, valstep=0.001)

#gcf_text =TextBox(ax_gcf, "GCF(rad/s):", textalignment="right")
#ax_gcf.text(0.0,0.2,"GCF(rad/s):%7.3f"%(0.0))
#pm_text = TextBox(ax_pm, "PM(deg):", textalignment="right") 
#pcf_text =TextBox(ax_pcf, "PCF(rad/s):", textalignment="right")
#gm_text = TextBox(ax_gm, "GM(dB) :", textalignment="right")

#gcf_text2 =TextBox(ax_gcf2, "GCF(rad/s):", textalignment="right")
#pm_text2 = TextBox(ax_pm2, "PM(deg):", textalignment="right") 
#pcf_text2 =TextBox(ax_pcf2, "PCF(rad/s):", textalignment="right")
#gm_text2 = TextBox(ax_gm2, "GM(dB) :", textalignment="right")

#伝達関数

#周波数範囲
fmin = -2
fmax = 4

#---- Rate control ---- 
#Drone
tau_f=1/(t_f.val*2*np.pi)
Kcopter = 0.033*np.sqrt(W*Ct)*k_m.val/i_y.val
omega = np.logspace(fmin,fmax,100)
copter_q=matlab.tf([Kcopter], [t_m.val, 1, 0])
sensor=matlab.tf([1],[tau_f, 1])
plant_gain, plant_phase, f = matlab.bode(copter_q*sensor, omega, plot=False)

#Controller
Controller=matlab.tf([k_p.val*t_i.val*t_d.val*(1+eta.val), k_p.val*(t_i.val+eta.val*t_d.val), k_p.val],\
                     [eta.val*t_i.val*t_d.val, t_i.val, 0])
ctrl_gain, ctrl_phase, f = matlab.bode(Controller, omega, plot=False)

#openloop
openloop=copter_q*Controller*sensor
openloop_gain, openloop_phase, f = matlab.bode(openloop, omega, plot=False)
gm,pm,pcf,gcf = matlab.margin(openloop)
ax_gcf.text(0.0,0.2,"GCF(rad/s):%7.3f"%(gcf))
ax_pm.text(0.0,0.2,"PM(deg):%7.3f"%(pm))
ax_pcf.text(0.0,0.2,"PCF(rad/s):%7.3f"%(pcf))
ax_gm.text(0.0,0.2,"GM(dB):%7.3f"%(gcf))

#gcf_text.set_val("{:6.3f}".format(gcf))
#pm_text.set_val("{:6.3f}".format(pm))
#pcf_text.set_val("{:6.3f}".format(pcf))
#gm_text.set_val("{:6.3f}".format(gm))

#closeloop
closeloop=matlab.feedback(copter_q*Controller,sensor)
closeloop_gain, closeloop_phase, f = matlab.bode(closeloop, omega, plot=False)

#impulse
t=np.linspace(0,0.2,200)
#impulse_y, impulse_t = matlab.impulse(closeloop,t)

#step
step_y,step_t = matlab.step(closeloop,t)


# グラフ描画
#openloop bode
ax_openloop_gain.set_xscale('log')
ax_openloop_gain.set_xlim(10**fmin,10**fmax)
ax_openloop_gain.set_ylim(-120,120)

ax_openloop_phase.set_xscale('log')
ax_openloop_phase.set_xlim(10**fmin,10**fmax)
ax_openloop_phase.set_ylim(-270,90)
ax_openloop_phase.set_yticks([90, 0,-90,-180,-270])



plant_gain_line, = ax_openloop_gain.plot(f, 20*np.log10(plant_gain), lw=1)
plant_phase_line, =ax_openloop_phase.plot(f, plant_phase*180/np.pi, lw=1)

ctrl_gain_line, = ax_openloop_gain.plot(f, 20*np.log10(ctrl_gain), lw=1)
ctrl_phase_line, =ax_openloop_phase.plot(f, ctrl_phase*180/np.pi, lw=1)

openloop_gain_line, = ax_openloop_gain.plot(f, 20*np.log10(openloop_gain), c="r" , lw=2)
openloop_phase_line, =ax_openloop_phase.plot(f, openloop_phase*180/np.pi, c="r", lw =2)

#closeloop bode
ax_closeloop_gain.set_xscale('log')
ax_closeloop_gain.set_xlim(10**fmin,10**fmax)
ax_closeloop_gain.set_ylim(-120,40)

ax_closeloop_phase.set_xscale('log')
ax_closeloop_phase.set_xlim(10**fmin,10**fmax)
ax_closeloop_phase.set_ylim(-270,90)
ax_closeloop_phase.set_yticks([90, 0,-90,-180,-270])

closeloop_gain_line, = ax_closeloop_gain.plot(f, 20*np.log10(closeloop_gain), lw=2, c='g')
closeloop_phase_line, =ax_closeloop_phase.plot(f, 360+closeloop_phase*180/np.pi, lw=2, c='g')


ax_step.set_xlim(0,0.2)
ax_step.set_ylim(0,1.5)
step_line, = ax_step.plot(step_t, step_y, c="r", lw =2)


#---- Angle control ----
#Drone
copter_pitch=closeloop
integral = matlab.tf([1],[1,0])
plant = integral*copter_pitch
plant_gain2, plant_phase2, f = matlab.bode(plant, omega, plot=False)

#Controller
Controller2=matlab.tf([k_p2.val*t_i2.val*t_d2.val*(1+eta2.val), k_p2.val*(t_i2.val+eta2.val*t_d2.val), k_p2.val],\
                     [eta2.val*t_i2.val*t_d2.val, t_i2.val, 0])
ctrl_gain2, ctrl_phase2, f = matlab.bode(Controller2, omega, plot=False)

#openloop
openloop2=plant*Controller2
openloop_gain2, openloop_phase2, f = matlab.bode(openloop2, omega, plot=False)
gm2,pm2,pcf2,gcf2 = matlab.margin(openloop2)
ax_gcf2.text(0.0,0.2,"GCF(rad/s):%7.3f"%(gcf2))
ax_pm2.text(0.0,0.2,"PM(deg):%7.3f"%(pm2))
ax_pcf2.text(0.0,0.2,"PCF(rad/s):%7.3f"%(pcf2))
ax_gm2.text(0.0,0.2,"GM(dB):%7.3f"%(gcf2))
#gcf_text2.set_val("{:6.3f}".format(gcf2))
#pm_text2.set_val("{:6.3f}".format(pm2))
#pcf_text2.set_val("{:6.3f}".format(pcf2))
#gm_text2.set_val("{:6.3f}".format(gm2))

#closeloop
closeloop2=matlab.feedback(openloop2,1)
closeloop_gain2, closeloop_phase2, f = matlab.bode(closeloop2, omega, plot=False)

#impulse
t2=np.linspace(0,10,200)
#impulse_y2, impulse_t2 = matlab.impulse(closeloop2,t2)

#step
step_y2,step_t2 = matlab.step(closeloop2,t2)


# グラフ描画
#openloop bode
ax_openloop_gain2.set_xscale('log')
ax_openloop_gain2.set_xlim(10**fmin,10**fmax)
ax_openloop_gain2.set_ylim(-120,120)

ax_openloop_phase2.set_xscale('log')
ax_openloop_phase2.set_xlim(10**fmin,10**fmax)
ax_openloop_phase2.set_ylim(-270,90)
ax_openloop_phase2.set_yticks([90, 0,-90,-180,-270])

plant_gain_line2, = ax_openloop_gain2.plot(f, 20*np.log10(plant_gain2), lw=1)
plant_phase_line2, =ax_openloop_phase2.plot(f, plant_phase2*180/np.pi, lw=1)

ctrl_gain_line2, = ax_openloop_gain2.plot(f, 20*np.log10(ctrl_gain2), lw=1)
ctrl_phase_line2, =ax_openloop_phase2.plot(f, ctrl_phase2*180/np.pi, lw=1)

openloop_gain_line2, = ax_openloop_gain2.plot(f, 20*np.log10(openloop_gain2), c="r" , lw=2)
openloop_phase_line2, =ax_openloop_phase2.plot(f, openloop_phase2*180/np.pi, c="r", lw =2)

#closeloop bode
ax_closeloop_gain2.set_xscale('log')
ax_closeloop_gain2.set_xlim(10**fmin,10**fmax)
ax_closeloop_gain2.set_ylim(-120,40)

ax_closeloop_phase2.set_xscale('log')
ax_closeloop_phase2.set_xlim(10**fmin,10**fmax)
ax_closeloop_phase2.set_ylim(-270,90)
ax_closeloop_phase2.set_yticks([90, 0,-90,-180,-270])

closeloop_gain_line2, = ax_closeloop_gain2.plot(f, 20*np.log10(closeloop_gain2), lw=2, c='g')
closeloop_phase_line2, =ax_closeloop_phase2.plot(f, closeloop_phase2*180/np.pi, lw=2, c='g')

ax_step2.set_xlim(0,5)
#ax_step2.set_ylim(0,1.5)
step_line2, = ax_step2.plot(step_t2, step_y2, c="r", lw =2)

ax_openloop_gain.grid()
ax_openloop_phase.grid()
ax_closeloop_gain.grid()
ax_closeloop_phase.grid()
ax_openloop_gain2.grid()
ax_openloop_phase2.grid()
ax_closeloop_gain2.grid()
ax_closeloop_phase2.grid()
ax_step.grid()
ax_step2.grid()


def update(slider_val):
    global copter_q
    global copter_pitch
    global Controller
    global Controller2
    global openloop
    global closeloop
    global openloop2
    global closeloop2

    
    # ---- Rate Control ----
    #Drone
    tau_f=1/(t_f.val*2*np.pi)
    Kcopter = 0.033*np.sqrt(W*Ct)*k_m.val/i_y.val
    omega = np.logspace(fmin,fmax,100)
    copter_q=matlab.tf([Kcopter], [t_m.val, 1, 0])
    sensor=matlab.tf([1],[tau_f, 1])
    plant_gain, plant_phase, f = matlab.bode(copter_q*sensor, omega, plot=False)

    #Controller
    Controller=matlab.tf([k_p.val*t_i.val*t_d.val*(1+eta.val), k_p.val*(t_i.val+eta.val*t_d.val), k_p.val],\
                         [eta.val*t_i.val*t_d.val, t_i.val, 0])
    ctrl_gain, ctrl_phase, f = matlab.bode(Controller, omega, plot=False)

    #openloop
    openloop=copter_q*Controller*sensor
    openloop_gain, openloop_phase, f = matlab.bode(openloop, omega, plot=False)
    gm,pm,pcf,gcf = matlab.margin(openloop)
    #gcf_text.set_val("{:6.3f}".format(gcf))
    ax_gcf.cla()
    ax_pm.cla()
    ax_pcf.cla()
    ax_gm.cla()
    ax_gcf.text(0.0,0.2,"GCF(rad/s):%7.3f"%(gcf))
    ax_pm.text(0.0,0.2,"PM(deg):%7.3f"%(pm))
    ax_pcf.text(0.0,0.2,"PCF(rad/s):%7.3f"%(pcf))
    ax_gm.text(0.0,0.2,"GM(dB):%7.3f"%(gcf))

    #closeloop
    closeloop=matlab.feedback(copter_q*Controller,sensor)
    closeloop_gain, closeloop_phase, f = matlab.bode(closeloop, omega, plot=False)
    
    #step
    t=np.linspace(0,0.2,200)
    step_y,step_t = matlab.step(closeloop,t)
    
    # xとyの値を更新
    plant_gain_line.set_xdata(f)
    plant_gain_line.set_ydata(20*np.log10(plant_gain))
    plant_phase_line.set_xdata(f)
    plant_phase_line.set_ydata(plant_phase*180/np.pi)    

    ctrl_gain_line.set_xdata(f)
    ctrl_gain_line.set_ydata(20*np.log10(ctrl_gain))
    ctrl_phase_line.set_xdata(f)
    ctrl_phase_line.set_ydata(ctrl_phase*180/np.pi)    

    openloop_gain_line.set_xdata(f)
    openloop_gain_line.set_ydata(20*np.log10(openloop_gain))
    openloop_phase_line.set_xdata(f)
    openloop_phase_line.set_ydata(openloop_phase*180/np.pi)    

    closeloop_gain_line.set_xdata(f)
    closeloop_gain_line.set_ydata(20*np.log10(closeloop_gain))
    closeloop_phase_line.set_xdata(f)
    closeloop_phase_line.set_ydata(closeloop_phase*180/np.pi)    
    
    step_line.set_xdata(step_t)
    step_line.set_ydata(step_y)

    #---- Angle control ----
    #Drone
    copter_pitch=closeloop
    integral = matlab.tf([1],[1,0])
    plant = integral*copter_pitch
    plant_gain2, plant_phase2, f = matlab.bode(plant, omega, plot=False)

    #Controller
    Controller2=matlab.tf([k_p2.val*t_i2.val*t_d2.val*(1+eta2.val), k_p2.val*(t_i2.val+eta2.val*t_d2.val), k_p2.val],\
                         [eta2.val*t_i2.val*t_d2.val, t_i2.val, 0])
    ctrl_gain2, ctrl_phase2, f = matlab.bode(Controller2, omega, plot=False)

    #openloop
    openloop2=plant*Controller2
    openloop_gain2, openloop_phase2, f = matlab.bode(openloop2, omega, plot=False)
    gm2,pm2,pcf2,gcf2 = matlab.margin(openloop2)
    ax_gcf2.cla()
    ax_pm2.cla()
    ax_pcf2.cla()
    ax_gm2.cla()
    ax_gcf2.text(0.0,0.2,"GCF(rad/s):%7.3f"%(gcf2))
    ax_pm2.text(0.0,0.2,"PM(deg):%7.3f"%(pm2))
    ax_pcf2.text(0.0,0.2,"PCF(rad/s):%7.3f"%(pcf2))
    ax_gm2.text(0.0,0.2,"GM(dB):%7.3f"%(gcf2))

    #closeloop
    closeloop2=matlab.feedback(openloop2,1)
    closeloop_gain2, closeloop_phase2, f = matlab.bode(closeloop2, omega, plot=False)

    #impulse
    t2=np.linspace(0,10,200)
    #impulse_y2, impulse_t2 = matlab.impulse(closeloop2,t)

    #step
    step_y2,step_t2 = matlab.step(closeloop2,t2)
    
     # xとyの値を更新
    plant_gain_line2.set_xdata(f)
    plant_gain_line2.set_ydata(20*np.log10(plant_gain2))
    plant_phase_line2.set_xdata(f)
    plant_phase_line2.set_ydata(plant_phase2*180/np.pi)    

    ctrl_gain_line2.set_xdata(f)
    ctrl_gain_line2.set_ydata(20*np.log10(ctrl_gain2))
    ctrl_phase_line2.set_xdata(f)
    ctrl_phase_line2.set_ydata(ctrl_phase2*180/np.pi)    

    openloop_gain_line2.set_xdata(f)
    openloop_gain_line2.set_ydata(20*np.log10(openloop_gain2))
    openloop_phase_line2.set_xdata(f)
    openloop_phase_line2.set_ydata(openloop_phase2*180/np.pi)    

    closeloop_gain_line2.set_xdata(f)
    closeloop_gain_line2.set_ydata(20*np.log10(closeloop_gain2))
    closeloop_phase_line2.set_xdata(f)
    closeloop_phase_line2.set_ydata(closeloop_phase2*180/np.pi)    
    
    step_line2.set_xdata(step_t2)
    step_line2.set_ydata(step_y2)

    # グラフの再描画
    fig.canvas.draw_idle()
    
    
# スライダーの値が変更された場合の処理を呼び出し
k_m.on_changed(update)
i_y.on_changed(update)
t_m.on_changed(update)
t_f.on_changed(update)
k_p.on_changed(update)
t_i.on_changed(update)
t_d.on_changed(update)
eta.on_changed(update)
k_p2.on_changed(update)
t_i2.on_changed(update)
t_d2.on_changed(update)
eta2.on_changed(update)


plt.show()
