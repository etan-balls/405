import sys, uselect, micropython
from utime import ticks_ms, ticks_diff, sleep_ms
from task_share import Share

S0=micropython.const(0)
S1=micropython.const(1)
S4=micropython.const(4)
S5=micropython.const(5)
S9=micropython.const(9)
S11=micropython.const(11)
S12=micropython.const(12)
S13=micropython.const(13)
S14=micropython.const(14)
S15=micropython.const(15)

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
                 bump_sensors=None):
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
        self._bump=bump_sensors
        self._wx=1062.5;self._sx=100.0;self._sy=800.0
        self._oset=False;self._fx=False;self._fy=True
        self._opas=False;self._otrg=0.0;self._oti0=0.0;self._otnxt=S12
        self._bk0=[0,0];self._wa=None;self._ba=None;self._cpt=0;self._lft=False
        self._rlatch=None;self._oblk=False;self._otl=False
        self._s3y=500.0;self._s3t=180.0;self._s3n=S12;self._s3l=False
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
        if p:
            self._io.write("{},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f}\r\n".format(
                tag,p[0]-self._sx,p[1]-self._sy,p[0],p[1],p[2],p[3]))

    def _drv(self,de):
        if self._ld: self._ld.enable();self._ld.set_effort(de)
        if self._rd: self._rd.enable();self._rd.set_effort(de-self._roff)

    def run(self):
        while True:
            if self._st==S0:
                self._dis()
                self._io.write("\r\nREADY f=tune o=obs c=cal\r\n"+PR)
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
                            self._io.write("TUNE 2m x=cancel\r\nerr,X,Y,h,d\r\n")
                            while self._io.any(): self._io.read1()
                            self._st=S4
                        elif ch in("c","C"):
                            self._dis();self._wa=None;self._ba=None
                            self._cpt=ticks_ms()-CAL_MS
                            self._io.write("\r\nCAL: w=white b=black x=exit\r\n")
                            while self._io.any(): self._io.read1()
                            self._st=S5
                        elif ch in("o","O"):
                            self._oset=False;self._opas=False
                            self._arm.put(True);self._lfen.put(True);self._lgo.put(True);self._rgo.put(True)
                            self._lf_t0=ticks_ms();self._lft=False
                            if self._le: self._le.zero()
                            if self._re: self._re.zero()
                            self._io.write("OBS: line follow\r\nerr,lx,ly,X,Y,h,d\r\n")
                            while self._io.any(): self._io.read1()
                            self._st=S9
                        else:
                            self._dis();self._io.write("\r\nSTOPPED\r\n"+PR)
                        yield self._st;continue

            elif self._st==S4:
                el=ticks_diff(ticks_ms(),self._lf_t0)
                if self._chk("\r\nLF CANCEL\r\n"): yield self._st;continue
                n=ticks_ms()
                if ticks_diff(n,self._lf_pt)>=50:
                    self._lf_pt=n
                    err=self._lerr.get() if self._lerr else 0.0
                    if self._lft and self._ctrl:
                        x,y,h,d=self._ctrl.get_odometry()
                        self._io.write("{:+.4f},{:.1f},{:.1f},{:.1f},{:.1f}\r\n".format(err,x,y,h,d))
                dn=False
                if self._lft and self._ctrl:
                    _,_,_,d=self._ctrl.get_odometry()
                    if d>=2000.0: dn=True
                if el>=LF_MS: dn=True
                if dn:
                    self._dis();self._io.write("\r\nLF DONE\r\n"+PR);self._st=S1

            elif self._st==S9:
                if not self._oset and self._ctrl and hasattr(self._ctrl,"set_odometry"):
                    try: self._ctrl.set_odometry(self._sx,self._sy,0.0);self._oset=True
                    except: pass
                if self._rlatch is not None and self._ctrl and hasattr(self._ctrl,"set_odometry"):
                    try: self._ctrl.set_odometry(self._rlatch[0],self._rlatch[1],self._rlatch[2])
                    except: pass
                    self._rlatch=None
                if self._chk("\r\nOBS CANCEL\r\n"): yield self._st;continue
                if self._ctrl and not self._opas:
                    try:
                        p=self._op()
                        if p and p[1]<600.0:
                            self._dis();self._ms();self._opas=True
                            self._otrg=90.0;self._oti0=p[2];self._otnxt=S13;self._otl=False
                            self._s3y=500.0;self._s3t=180.0;self._s3n=S12;self._s3l=False
                            self._io.write("TURN h={:.1f} trg=90 Y={:.0f}\r\n".format(p[2],p[1]))
                            sleep_ms(300);self._lf_pt=ticks_ms()
                            self._st=S11;yield self._st;continue
                    except: pass
                if self._bump and self._bump.any():
                    if self._ctrl and hasattr(self._ctrl,"set_odometry"):
                        try:
                            x,y,h,d=self._ctrl.get_odometry()
                            self._ctrl.set_odometry(self._wx,y,180.0)
                        except: pass
                    self._cancel("\r\nOBS bump\r\n");yield self._st;continue
                if self._oblk and self._sen:
                    try:
                        rd=self._sen.get_raw_readings()
                        bk=self._sen.black;wh=self._sen.white
                        allb=True
                        for i in range(len(rd)):
                            b_i=bk[i] if isinstance(bk,(list,tuple)) else bk
                            w_i=wh[i] if isinstance(wh,(list,tuple)) else wh
                            mid=(b_i+w_i)/2.0
                            if rd[i]<mid: allb=False;break
                        if allb:
                            self._oblk=False;self._dis();self._ms()
                            if self._ctrl and hasattr(self._ctrl,"set_odometry"):
                                try: self._ctrl.set_odometry(1175.0,1300.0,90.0)
                                except: pass
                            self._s3y=100.0;self._s3t=180.0;self._s3n=S9;self._s3l=False
                            self._io.write("ALLBLK->STRY\r\n")
                            sleep_ms(300);self._lf_pt=ticks_ms()
                            self._st=S13;yield self._st;continue
                    except: pass
                n=ticks_ms()
                if ticks_diff(n,self._lf_pt)>=50:
                    self._lf_pt=n
                    err=self._lerr.get() if self._lerr else 0.0
                    p=self._op()
                    if p:
                        self._io.write("{:+.4f},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f}\r\n".format(
                            err,p[0]-self._sx,p[1]-self._sy,p[0],p[1],p[2],p[3]))

            elif self._st==S11:
                if self._chk("\r\nTURN CANCEL\r\n"): yield self._st;continue
                self._eu()
                p=self._op();h=p[2] if p else 0.0
                rem=((h-self._otrg)%360.0) if self._otl else ((self._otrg-h)%360.0)
                n=ticks_ms()
                if ticks_diff(n,self._lf_pt)>=100:
                    self._lf_pt=n
                    self._io.write("TURN h={:.1f} trg={:.1f} rem={:.1f}\r\n".format(h,self._otrg,rem))
                if rem<2.0 or rem>358.0:
                    self._ms();sleep_ms(300)
                    nxt=self._otnxt
                    self._io.write("TURN DONE h={:.1f}->S{}\r\n".format(h,nxt))
                    if nxt==S12 or nxt==S9:
                        if nxt==S9:
                            rl=None
                            if self._ctrl:
                                try:
                                    x,y,h2,d=self._ctrl.get_odometry()
                                    rl=(x,y,h2)
                                except: pass
                            self._rlatch=rl
                        self._arm.put(True);self._lfen.put(True)
                        self._lgo.put(True);self._rgo.put(True)
                    self._lf_pt=ticks_ms();self._st=nxt;yield self._st;continue
                else:
                    ef=min(20.0,max(10.0,rem*0.10))
                    if self._otl:
                        if self._ld: self._ld.enable();self._ld.set_effort(ef)
                        if self._rd: self._rd.enable();self._rd.set_effort(-ef)
                    else:
                        if self._ld: self._ld.enable();self._ld.set_effort(-ef)
                        if self._rd: self._rd.enable();self._rd.set_effort(ef)

            elif self._st==S12:
                if self._chk("\r\nDRV CANCEL\r\n"): yield self._st;continue
                if self._bump and self._bump.any():
                    self._arm.put(False);self._ms()
                    if self._ctrl and hasattr(self._ctrl,"set_odometry"):
                        try:
                            x,y,h,d=self._ctrl.get_odometry()
                            self._ctrl.set_odometry(self._wx,y,180.0)
                        except: pass
                    self._io.write("BUMP->BACK\r\n")
                    self._eu()
                    self._bk0[0]=self._le.get_position() if self._le else 0
                    self._bk0[1]=self._re.get_position() if self._re else 0
                    sleep_ms(300);self._lf_pt=ticks_ms()
                    self._st=S14;yield self._st;continue
                n=ticks_ms()
                if ticks_diff(n,self._lf_pt)>=100:
                    self._lf_pt=n;self._olog("DRV")

            elif self._st==S13:
                if self._chk("\r\nSTRY CANCEL\r\n"): yield self._st;continue
                self._drv(-float(ME));self._eu()
                p=self._op();n=ticks_ms()
                if ticks_diff(n,self._lf_pt)>=100:
                    self._lf_pt=n;self._olog("STRY")
                if p and p[1]<self._s3y:
                    self._ms()
                    self._otrg=self._s3t;self._oti0=p[2];self._otnxt=self._s3n;self._otl=self._s3l
                    self._io.write("TURN h={:.1f} trg={:.0f} Y={:.0f}\r\n".format(p[2],self._s3t,p[1]))
                    sleep_ms(300);self._lf_pt=ticks_ms()
                    self._st=S11;yield self._st;continue

            elif self._st==S14:
                if self._chk("\r\nBACK CANCEL\r\n"): yield self._st;continue
                be=float(ME)
                if self._ld: self._ld.enable();self._ld.set_effort(be)
                if self._rd: self._rd.enable();self._rd.set_effort(be)
                self._eu()
                lp=self._le.get_position() if self._le else 0
                rp=self._re.get_position() if self._re else 0
                dist=(abs(lp-self._bk0[0])+abs(rp-self._bk0[1]))*MPC/2.0
                n=ticks_ms()
                if ticks_diff(n,self._lf_pt)>=100:
                    self._lf_pt=n;self._io.write("BACK d={:.1f}\r\n".format(dist))
                if dist>=50.0:
                    self._ms();self._io.write("BACK DONE->SEEK\r\n")
                    sleep_ms(200);self._lf_pt=ticks_ms()
                    self._st=S15;yield self._st;continue

            elif self._st==S15:
                if self._chk("\r\nSEEK CANCEL\r\n"): yield self._st;continue
                if self._ld: self._ld.enable();self._ld.set_effort(20.0)
                if self._rd: self._rd.enable();self._rd.set_effort(-20.0)
                self._eu()
                err=self._sen.calculate_error() if self._sen else 0.0
                n=ticks_ms()
                if ticks_diff(n,self._lf_pt)>=100:
                    self._lf_pt=n
                    p=self._op();h=p[2] if p else 0.0
                    self._io.write("SEEK h={:.1f} e={:.2f}\r\n".format(h,err))
                if abs(err)>0.01:
                    self._ms()
                    rl=None
                    if self._ctrl:
                        try:
                            x,y,h,d=self._ctrl.get_odometry()
                            rl=(x,y,h)
                        except: pass
                    self._rlatch=rl;self._oblk=True
                    self._io.write("LINE e={:.2f}->LF\r\n".format(err))
                    sleep_ms(200)
                    self._arm.put(True);self._lfen.put(True)
                    self._lgo.put(True);self._rgo.put(True)
                    self._lf_pt=ticks_ms();self._st=S9;yield self._st;continue

            elif self._st==S5:
                if self._io.any():
                    b=self._io.read1()
                    if b:
                        try: ch=b.decode()
                        except: ch=""
                        rd=self._sen.get_raw_readings()
                        if ch in("w","W"):
                            self._wa=int(sum(rd)/len(rd))
                            self._io.write("WHITE={}\r\n".format(self._wa))
                        elif ch in("b","B"):
                            self._ba=int(max(rd))
                            self._io.write("BLACK={}\r\n".format(self._ba))
                        elif ch in("x","X","q","Q"):
                            self._io.write("\r\nExit cal\r\n"+PR)
                            self._st=S1;yield self._st;continue
                        if self._wa is not None and self._ba is not None:
                            self._io.write("w={} b={}\r\n".format(self._wa,self._ba))
                n=ticks_ms()
                if ticks_diff(n,self._cpt)>=CAL_MS:
                    self._cpt=n
                    rd=self._sen.get_raw_readings()
                    self._io.write(",".join(str(int(r)) for r in rd)+"\r\n")

            yield self._st
