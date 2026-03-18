import sys, uselect, micropython
from utime import ticks_ms, ticks_diff
from task_share import Share

S0=micropython.const(0)
S1=micropython.const(1)
S2=micropython.const(2)   # tune
S3=micropython.const(3)   # calibration
S4=micropython.const(4)   # 1st line follow
S5=micropython.const(5)   # forward drive
S6=micropython.const(6)   # turn
S7=micropython.const(7)   # drive to wall
S9=micropython.const(9)   # back up
S10=micropython.const(10) # spin to find line
S12=micropython.const(12) # 3rd line follow
S13=micropython.const(13) # 4th line follow (biased)

LF_MS=micropython.const(10000)
ME=micropython.const(30)
CAL_MS=micropython.const(200)
MPC=0.15303
PR=">: "

class _IO:
    def __init__(self):
        self._p=uselect.poll(); self._p.register(sys.stdin,uselect.POLLIN)
    def any(self):
        try: return 1 if self._p.poll(0) else 0
        except: return 0
    def read1(self):
        if not self.any(): return None
        try:
            c=sys.stdin.read(1)
            if not c: return None
            return c.encode() if isinstance(c,str) else c
        except: return None
    def write(self,s):
        try: sys.stdout.write(s)
        except: pass

class task_user:
    def __init__(self,leftMotorGo,rightMotorGo,lineFollowEnable,armEnable,
                 lineSensor,lineError=None,leftEffortCmd=None,rightEffortCmd=None,
                 leftEncoder=None,rightEncoder=None,leftMotorDriver=None,
                 rightMotorDriver=None,imu_heading=None,imu_yawrate=None,
                 imu_calib=None,right_offset=0.0,ctrl=None,s_hat_share=None,
                 psi_hat_share=None,omL_hat_share=None,omR_hat_share=None,
                 bump_sensors=None,user_btn=None):
        self._st=S0
        self._lgo=leftMotorGo;self._rgo=rightMotorGo
        self._lfen=lineFollowEnable;self._arm=armEnable
        self._sen=lineSensor;self._lerr=lineError
        self._lf_t0=0;self._lf_pt=0
        self._lefc=leftEffortCmd;self._refc=rightEffortCmd
        self._le=leftEncoder;self._re=rightEncoder
        self._ld=leftMotorDriver;self._rd=rightMotorDriver
        self._imuh=imu_heading;self._imuw=imu_yawrate;self._imuc=imu_calib
        self._roff=float(right_offset);self._ctrl=ctrl
        self._bump=bump_sensors;self._ubtn=user_btn;self._ubtn_prev=1
        self._sx=100.0;self._sy=800.0
        self._oset=False;self._fx=False;self._fy=True
        self._otrg=0.0;self._oti0=0.0;self._otnxt=S7
        self._bk0=[0,0];self._wa=None;self._ba=None;self._cpt=0;self._lft=False
        self._rlatch=None;self._otl=False;self._arc=False
        self._fwd_dist=75.0;self._fwd_nxt=S7;self._arc=True;self._lf_tag="LF";self._turn_trg=170.0;self._lf_zero=0;self._lf_min_dist=0.0;self._lf_s0=None;self._s5_cap=False;self._s5_h=None
        self._io=_IO()

    def _dis(self):
        self._arm.put(False);self._lfen.put(False)
        self._lgo.put(False);self._rgo.put(False)
        if self._lefc: self._lefc.put(0.0)
        if self._refc: self._refc.put(0.0)

    def _ms(self):
        if self._ld: self._ld.set_effort(0)
        if self._rd: self._rd.set_effort(0)

    def _eu(self):
        if self._le: self._le.update()
        if self._re: self._re.update()
        if self._ctrl:
            try: self._ctrl._odo_update()
            except: pass

    def _op(self):
        if not self._ctrl: return None
        try:
            x,y,h,d=self._ctrl.get_odometry()
            if self._fx: x=2.0*self._sx-x
            if self._fy: y=2.0*self._sy-y
            return x,y,h,d
        except: return None

    def _cancel(self,msg):
        self._dis();self._ms()
        if self._ctrl:
            x,y,h,d=self._ctrl.get_odometry()
            self._io.write("X={:.1f} Y={:.1f} H={:.1f} D={:.1f}\r\n".format(x,y,h,d))
        self._io.write(msg+PR);self._st=S1

    def _chk(self,msg):
        if self._io.any():
            self._io.read1();self._cancel(msg);return True
        return False

    def _imu_h(self):
        if self._imuh is None: return 0.0
        v=self._imuh.get()
        return float(v) if v is not None else 0.0

    def _olog(self,tag):
        p=self._op()
        err=self._lerr.get() if self._lerr else 0.0
        if p:
            self._io.write("{},{:+.4f},{:.1f},{:.1f},{:.1f}\r\n".format(
                tag,err,p[0],p[1],p[2]))

    def _drv(self,de):
        if self._ld: self._ld.enable();self._ld.set_effort(de)
        if self._rd: self._rd.enable();self._rd.set_effort(de-self._roff)

    def run(self):
        while True:
            if self._st==S0:
                self._dis()
                self._io.write("\r\nREADY\r\n"+PR)
                self._st=S1

            elif self._st==S1:
                if self._io.any():
                    b=self._io.read1()
                    if b:
                        try: ch=b.decode()
                        except: ch=""
                        if ch in("f","F"):
                            self._arm.put(True);self._lfen.put(True);self._lgo.put(True);self._rgo.put(True)
                            self._lf_t0=ticks_ms();self._lft=True
                            if self._le: self._le.zero()
                            if self._re: self._re.zero()
                            if self._ctrl and hasattr(self._ctrl,"set_odometry"):
                                try: self._ctrl.set_odometry(0.0,0.0,0.0)
                                except: pass
                            self._io.write("TUNE\r\n")
                            while self._io.any(): self._io.read1()
                            self._st=S2
                        elif ch in("c","C"):
                            self._dis();self._wa=None;self._ba=None
                            self._cpt=ticks_ms()-CAL_MS
                            self._io.write("\r\nCAL\r\n")
                            while self._io.any(): self._io.read1()
                            self._st=S3
                        elif ch in("o","O"):
                            self._oset=False;self._rlatch=None
                            self._fwd_dist=50.0;self._arc=True;self._fwd_nxt=S7;self._turn_trg=170.0;self._lf_tag="LF"
                            self._arm.put(True);self._lfen.put(True);self._lgo.put(True);self._rgo.put(True)
                            self._lf_t0=ticks_ms();self._lft=False
                            if self._le: self._le.zero()
                            if self._re: self._re.zero()
                            self._io.write("OBS\r\n")
                            while self._io.any(): self._io.read1()
                            self._st=S4
                        else:
                            self._dis();self._io.write("\r\nSTOP\r\n"+PR)
                        yield self._st;continue
                if self._ubtn is not None:
                    v=self._ubtn.value()
                    if v==0 and self._ubtn_prev==1:
                        self._ubtn_prev=0
                        self._oset=False;self._rlatch=None
                        self._fwd_dist=50.0;self._arc=True;self._fwd_nxt=S7;self._turn_trg=170.0;self._lf_tag="LF"
                        self._arm.put(True);self._lfen.put(True);self._lgo.put(True);self._rgo.put(True)
                        self._lf_t0=ticks_ms();self._lft=False
                        if self._le: self._le.zero()
                        if self._re: self._re.zero()
                        self._io.write("OBS\r\n")
                        self._st=S4;yield self._st;continue
                    else:
                        self._ubtn_prev=v

            elif self._st==S2:
                el=ticks_diff(ticks_ms(),self._lf_t0)
                if self._chk("\r\nCANCEL\r\n"): yield self._st;continue
                n=ticks_ms()
                if ticks_diff(n,self._lf_pt)>=20:
                    self._lf_pt=n
                    err=self._lerr.get() if self._lerr else 0.0
                    if self._lft and self._ctrl:
                        self._io.write("{:+.4f}\r\n".format(err))
                dn=False
                if self._lft and self._ctrl:
                    _,_,_,d=self._ctrl.get_odometry()
                    if d>=2000.0: dn=True
                if el>=LF_MS: dn=True
                if dn:
                    self._dis();self._io.write("\r\nLF DONE\r\n"+PR);self._st=S1

            elif self._st==S3:
                if self._io.any():
                    b=self._io.read1()
                    if b:
                        try: ch=b.decode()
                        except: ch=""
                        rd=self._sen.get_raw_readings()
                        if ch in("w","W"):
                            self._wa=int(sum(rd)/len(rd))
                            self._io.write("W={}\r\n".format(self._wa))
                        elif ch in("b","B"):
                            self._ba=int(max(rd))
                            self._io.write("B={}\r\n".format(self._ba))
                        elif ch in("x","X","q","Q"):
                            self._io.write("\r\nEXIT\r\n"+PR)
                            self._st=S1;yield self._st;continue
                        if self._wa is not None and self._ba is not None:
                            self._io.write("w={} b={}\r\n".format(self._wa,self._ba))
                n=ticks_ms()
                if ticks_diff(n,self._cpt)>=CAL_MS:
                    self._cpt=n
                    rd=self._sen.get_raw_readings()
                    self._io.write(",".join(str(int(r)) for r in rd)+"\r\n")

            elif self._st==S4:
                if not self._oset and self._ctrl and hasattr(self._ctrl,"set_odometry"):
                    try: self._ctrl.set_odometry(self._sx,self._sy,0.0);self._oset=True
                    except: pass
                if self._rlatch is not None and self._ctrl and hasattr(self._ctrl,"set_odometry"):
                    try: self._ctrl.set_odometry(self._rlatch[0],self._rlatch[1],self._rlatch[2])
                    except: pass
                    self._rlatch=None
                if self._chk("\r\nCANCEL\r\n"): yield self._st;continue
                if self._lf_s0 is None:
                    self._lf_s0=[self._le.get_position() if self._le else 0,self._re.get_position() if self._re else 0]
                err=self._lerr.get() if self._lerr else 0.0
                lf_dist=(abs((self._le.get_position() if self._le else 0)-self._lf_s0[0])+abs((self._re.get_position() if self._re else 0)-self._lf_s0[1]))*MPC/2.0
                if err==0.0 and lf_dist>=self._lf_min_dist:
                    self._lf_zero+=1
                    if self._lf_zero>=3:
                        self._lf_zero=0
                        self._eu()
                        self._bk0[0]=self._le.get_position() if self._le else 0
                        self._bk0[1]=self._re.get_position() if self._re else 0
                        self._io.write("E0>F{:.0f}\r\n".format(self._fwd_dist))
                        self._lf_pt=ticks_ms()
                        self._st=S5;yield self._st;continue
                else:
                    self._lf_zero=0
                n=ticks_ms()
                if ticks_diff(n,self._lf_pt)>=20:
                    self._lf_pt=n;self._olog(self._lf_tag)

            elif self._st==S5:
                if self._chk("\r\nCANCEL\r\n"): self._dis();yield self._st;continue
                self._eu()
                lp=self._le.get_position() if self._le else 0
                rp=self._re.get_position() if self._re else 0
                if self._s5_cap:
                    self._bk0[0]=lp;self._bk0[1]=rp;self._s5_cap=False
                    if self._s5_h is not None and self._ctrl:
                        try:
                            x5,y5,_,_=self._ctrl.get_odometry()
                            self._ctrl.set_odometry(x5,y5,self._s5_h)
                        except: pass
                    self._s5_h=None
                dist=(abs(lp-self._bk0[0])+abs(rp-self._bk0[1]))*MPC/2.0
                n=ticks_ms()
                if ticks_diff(n,self._lf_pt)>=20:
                    self._lf_pt=n
                    self._io.write("FWD {:.1f}\r\n".format(dist))
                if dist>=self._fwd_dist:
                    self._dis();self._ms()
                    p=self._op();h=p[2] if p else 0.0
                    self._otrg=self._turn_trg;self._oti0=h;self._otnxt=self._fwd_nxt;self._otl=(h>self._turn_trg)
                    self._io.write("FWD>T{:.0f} h={:.1f}\r\n".format(self._turn_trg,h))
                    self._lf_pt=ticks_ms()
                    self._st=S6;yield self._st;continue

            elif self._st==S6:
                if self._chk("\r\nCANCEL\r\n"): yield self._st;continue
                self._eu()
                p=self._op();h=p[2] if p else 0.0
                rem=((h-self._otrg)%360.0) if self._otl else ((self._otrg-h)%360.0)
                n=ticks_ms()
                if ticks_diff(n,self._lf_pt)>=20:
                    self._lf_pt=n
                    self._io.write("T{:.0f} r={:.1f}\r\n".format(self._otrg,rem))
                if rem<2.0 or rem>358.0:
                    self._ms()
                    nxt=self._otnxt
                    self._io.write("DONE h={:.1f}->S{}\r\n".format(h,nxt))
                    if nxt==S12:
                        if self._ctrl:
                            try:
                                x,y,h2,d=self._ctrl.get_odometry()
                                self._rlatch=(x,y,h2)
                                self._s5_h=h2
                            except: pass
                        self._lf_s0=None;self._lf_zero=0
                        self._arm.put(True);self._lfen.put(True)
                        self._lgo.put(True);self._rgo.put(True)
                    elif nxt==S13:
                        self._bk0[0]=self._le.get_position() if self._le else 0
                        self._bk0[1]=self._re.get_position() if self._re else 0
                        self._lf_zero=0
                    elif nxt==S5:
                        self._s5_cap=True
                        p5=self._op();self._s5_h=p5[2] if p5 else None
                        self._arm.put(True);self._lfen.put(True)
                        self._lgo.put(True);self._rgo.put(True)
                    elif nxt==S1:
                        self._dis();self._ms()
                        self._io.write("OBS DONE\r\n"+PR)
                    self._lf_pt=ticks_ms();self._st=nxt;yield self._st;continue
                else:
                    ef=min(20.0,max(10.0,rem*0.10))
                    if self._arc:
                        base=15.0
                        if self._otl:
                            if self._ld: self._ld.enable();self._ld.set_effort(-(base-ef))
                            if self._rd: self._rd.enable();self._rd.set_effort(-(base+ef)-self._roff)
                        else:
                            if self._ld: self._ld.enable();self._ld.set_effort(-(base+ef))
                            if self._rd: self._rd.enable();self._rd.set_effort(-(base-ef)-self._roff)
                    else:
                        if self._otl:
                            if self._ld: self._ld.enable();self._ld.set_effort(ef)
                            if self._rd: self._rd.enable();self._rd.set_effort(-ef)
                        else:
                            if self._ld: self._ld.enable();self._ld.set_effort(-ef)
                            if self._rd: self._rd.enable();self._rd.set_effort(ef)

            elif self._st==S7:
                if self._chk("\r\nCANCEL\r\n"): yield self._st;continue
                if self._bump and self._bump.any():
                    self._ms()
                    self._io.write("BUMP\r\n")
                    self._eu()
                    self._bk0[0]=self._le.get_position() if self._le else 0
                    self._bk0[1]=self._re.get_position() if self._re else 0
                    self._lf_pt=ticks_ms()
                    self._st=S9;yield self._st;continue
                self._eu()
                p=self._op();h_now=p[2] if p else 170.0
                h_err=h_now-170.0
                if h_err>180.0: h_err-=360.0
                if h_err<-180.0: h_err+=360.0
                corr=max(-10.0,min(10.0,h_err*0.5))
                ef=float(ME)
                if self._ld: self._ld.enable();self._ld.set_effort(-ef+corr)
                if self._rd: self._rd.enable();self._rd.set_effort(-ef-corr-self._roff)
                n=ticks_ms()
                if ticks_diff(n,self._lf_pt)>=20:
                    self._lf_pt=n;self._olog("DRV")


            elif self._st==S9:
                if self._chk("\r\nCANCEL\r\n"): yield self._st;continue
                be=float(ME)
                if self._ld: self._ld.enable();self._ld.set_effort(be)
                if self._rd: self._rd.enable();self._rd.set_effort(be)
                self._eu()
                lp=self._le.get_position() if self._le else 0
                rp=self._re.get_position() if self._re else 0
                dist=(abs(lp-self._bk0[0])+abs(rp-self._bk0[1]))*MPC/2.0
                n=ticks_ms()
                if ticks_diff(n,self._lf_pt)>=20:
                    self._lf_pt=n
                    self._io.write("BK {:.1f}\r\n".format(dist))
                if dist>=10.0:
                    self._ms()
                    p=self._op();h=p[2] if p else 180.0
                    self._otrg=90.0;self._oti0=h;self._otnxt=S10;self._otl=(h>90.0);self._arc=False
                    self._io.write("BK>T90 h={:.1f}\r\n".format(h))
                    self._lf_pt=ticks_ms()
                    self._st=S6;yield self._st;continue

            elif self._st==S10:
                if self._chk("\r\nCANCEL\r\n"): yield self._st;continue
                ef2=float(ME)
                p=self._op();h_now=p[2] if p else 90.0
                h_err=h_now-90.0
                if h_err>180.0: h_err-=360.0
                if h_err<-180.0: h_err+=360.0
                corr=max(-10.0,min(10.0,h_err*0.5))
                if self._ld: self._ld.enable();self._ld.set_effort(-ef2+corr)
                if self._rd: self._rd.enable();self._rd.set_effort(-ef2-corr-self._roff)
                self._eu()
                err=self._sen.calculate_error() if self._sen else 0.0
                n=ticks_ms()
                if ticks_diff(n,self._lf_pt)>=20:
                    self._lf_pt=n
                    p=self._op()
                    self._io.write("STRT,{:+.4f},{:.1f},{:.1f},{:.1f}\r\n".format(err,p[0] if p else 0.0,p[1] if p else 0.0,p[2] if p else 0.0))
                if abs(err)>0.01:
                    self._ms()
                    if self._ctrl:
                        try:
                            x,y,h,d=self._ctrl.get_odometry()
                            self._rlatch=(x,y,h)
                        except: pass
                    self._fwd_dist=20.0;self._arc=False;self._fwd_nxt=S12;self._lf_tag="LF2";self._turn_trg=170.0;self._lf_min_dist=200.0;self._lf_s0=None
                    self._io.write("LINE {:.2f}\r\n".format(err))
                    self._arm.put(True);self._lfen.put(True)
                    self._lgo.put(True);self._rgo.put(True)
                    self._lf_pt=ticks_ms();self._st=S4;yield self._st;continue

            elif self._st==S12:
                if self._rlatch is not None and self._ctrl and hasattr(self._ctrl,"set_odometry"):
                    try:
                        h_restore=self._s5_h if self._s5_h is not None else self._rlatch[2]
                        self._ctrl.set_odometry(self._rlatch[0],self._rlatch[1],h_restore)
                    except: pass
                    self._rlatch=None;self._s5_h=None
                if self._chk("\r\nCANCEL\r\n"): yield self._st;continue
                if self._lf_s0 is None:
                    self._lf_s0=[self._le.get_position() if self._le else 0,self._re.get_position() if self._re else 0]
                err=self._lerr.get() if self._lerr else 0.0
                lf_dist=(abs((self._le.get_position() if self._le else 0)-self._lf_s0[0])+abs((self._re.get_position() if self._re else 0)-self._lf_s0[1]))*MPC/2.0
                if err==0.0 and lf_dist>=200.0:
                    p3=self._op();h3=p3[2] if p3 else 0.0
                    if h3>=330.0 or h3<=30.0:
                        self._eu()
                        if self._ctrl:
                            try:
                                x3,y3,_,_=self._ctrl.get_odometry()
                                self._ctrl.set_odometry(x3,y3,0.0)
                            except: pass
                        self._bk0[0]=self._le.get_position() if self._le else 0
                        self._bk0[1]=self._re.get_position() if self._re else 0
                        self._fwd_dist=10.0;self._arc=False;self._fwd_nxt=S13;self._turn_trg=190.0
                        self._io.write("E0>F10>T\r\n")
                        self._lf_pt=ticks_ms()
                        self._st=S5;yield self._st;continue
                n=ticks_ms()
                if ticks_diff(n,self._lf_pt)>=20:
                    self._lf_pt=n;self._olog("LF3")

            elif self._st==S13:
                if self._chk("\r\nCANCEL\r\n"): yield self._st;continue
                lp=self._le.get_position() if self._le else 0
                rp=self._re.get_position() if self._re else 0
                dist=(abs(lp-self._bk0[0])+abs(rp-self._bk0[1]))*MPC/2.0
                err=float(self._lerr.get()) if self._lerr else 0.0
                bias_err=0.2 if dist<200.0 else 0.0
                steer=max(0.0,min(15.0,(err-bias_err)*20.0))
                ef=float(ME)
                if self._ld: self._ld.enable();self._ld.set_effort(-ef-steer)
                if self._rd: self._rd.enable();self._rd.set_effort(-ef+steer-self._roff)
                self._eu()
                n=ticks_ms()
                if ticks_diff(n,self._lf_pt)>=20:
                    self._lf_pt=n
                    self._io.write("LF4 {:.2f}\r\n".format(err))
                if err==0.0 and dist>=200.0:
                    self._lf_zero+=1
                    if self._lf_zero>=3:
                        self._lf_zero=0
                        self._ms()
                        p=self._op();h=p[2] if p else 0.0
                        self._fwd_dist=150.0;self._arc=False;self._fwd_nxt=S1;self._turn_trg=0.0
                        self._otrg=270.0;self._oti0=h;self._otnxt=S5
                        self._otl=False
                        self._io.write("E0>T270>F150>T0\r\n")
                        self._lf_pt=ticks_ms();self._st=S6;yield self._st;continue
                else:
                    self._lf_zero=0

            yield self._st
