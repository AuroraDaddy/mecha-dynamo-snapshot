'use strict';
// ============ engine_phys.py 포팅(가솔린 SI) ============
const P0=101325,T0=298,RHO0=1.205,LHV=44e6,AFR_S=14.7,R_AIR=287;
const GAMMA=1.34,CV=R_AIR/(GAMMA-1),TWALL=450,HEAT_SCALE=1.15,COMB_EFF=0.97;
const cl=(x,a,b)=>x<a?a:(x>b?b:x), ss=x=>{x=cl(x,0,1);return x*x*(3-2*x)};
const TURBO={Small:{thr:1600,fdes:4200,etap:.74,cap:1.3},Med:{thr:2700,fdes:5800,etap:.76,cap:1.9},Large:{thr:3800,fdes:7400,etap:.78,cap:2.6}};
function intakeTune(P,rpm){const c=345,L=(350+90)/1000,dur=240+P.cam*80,te=(dur/360)*(60/Math.max(rpm,200)),tw=2*L/c,ph=1.83*tw/Math.max(te,1e-6);
 return cl(1+0.16*Math.cos(2*Math.PI*(ph-0.5)),0.86,1.18)}
function ve(P,rpm){const cam=P.cam,rp=3000+cam*5000,vm=0.86+cam*0.12,span=2400+cam*1700,x=(rpm-rp)/span;
 let v=vm*Math.exp(-0.5*x*x); if(rpm<rp)v*=(1-cam*0.35*(1-rpm/Math.max(rp,1)));
 v*=intakeTune(P,rpm); v*=1+0.0*(rpm/P.redline-0.5)*2;   // exhaust=Sport(0.0)
 const thr=0.05+0.95*(P.throttle/100); return Math.max(0.12,Math.min(1.05,v))*thr}
function compressor(P,rpm){
 if(P.induction==='NA')return{boost:0,eta:1,state:'',Tchg:T0};
 const tgt=P.boost;
 if(P.induction==='Super'){const drive=cl(0.55+0.45*rpm/P.redline,0,1.6),b=Math.min(tgt,tgt*drive),eta=0.62,
  PR=1+b/1.013,Tchg=T0*(1+(Math.pow(PR,0.286)-1)/eta); return{boost:b,eta,state:'belt',Tchg}}
 const tb={...TURBO[P.turbo_size]},trim=0.5;
 tb.fdes*=(0.7+0.6*trim);tb.thr*=(0.8+0.4*trim);tb.cap*=(0.85+0.3*trim);
 const spool=ss((rpm-tb.thr)/1300), v=ve({...P,induction:'NA'},rpm), fr=(rpm*Math.max(v,0.3))/(tb.fdes*0.9);
 let cap,st;
 if(fr<0.35){cap=tgt*0.35;st='surge'}else if(fr<=1.15){cap=tgt;st='on boost'}
 else{cap=tgt*Math.max(0.15,1-(fr-1.15)/0.7);st='choke'}
 cap=Math.min(cap,tb.cap); let b=Math.max(0,Math.min(spool*tgt,cap));
 if(rpm<tb.thr*1.05)st='spooling';
 const PR=1+b/1.013;
 let eta=tb.etap*Math.exp(-0.5*Math.pow((fr-1)/0.5,2)-0.5*Math.pow((PR-1.9)/1.3,2));
 eta=cl(eta,0.45,tb.etap);
 const Tchg=b>0?T0*(1+(Math.pow(PR,0.286)-1)/Math.max(eta,0.4)):T0;
 return{boost:b,eta,state:st,Tchg}}
const optSpark=(rpm,P)=>Math.min(36,8+rpm/360+P.cam*4);
function effAfr(afr){const d=afr-12.6,sp=d<0?2:3.6;let e=Math.exp(-0.5*Math.pow(d/sp,2));
 if(afr>17.5)e*=Math.max(0,1-(afr-17.5)/2.5); if(afr<10.5)e*=Math.max(0,1-(10.5-afr)/2.5); return Math.min(1,e)}
function knockIdx(P,rpm,sp){const c=compressor(P,rpm),bv=c.boost;
 let ki=(P.cr-9.3)*0.85+Math.max(0,sp-optSpark(rpm,P))*0.10
  +(P.throttle/100)*0.9-(rpm/9000)*0.9-(P.octane-90)/8;
 if(P.afr>14.8)ki+=(P.afr-14.8)*0.08; ki+=bv*1.4;
 if(bv>0)ki+=(c.Tchg-T0)/120*0.4; return ki}
function effSpark(P,rpm){let sp=optSpark(rpm,P);          // ign_auto=On 고정(노킹 자동 회피)
 for(let i=0;i<22;i++){if(knockIdx(P,rpm,sp)<=0.97||sp<=5)break;sp-=2} return Math.max(5,sp)}
function cycle(P,rpm,steps=110){
 const bore=P.bore/1000,strk=P.strk/1000,Vd=Math.PI/4*bore*bore*strk,CR=P.cr,Vc=Vd/(CR-1),
  r=strk/2,rod=r*3.6;
 const Vol=thd=>{const t=thd*Math.PI/180,x=r*(1-Math.cos(t))+rod-Math.sqrt(Math.max(0,rod*rod-Math.pow(r*Math.sin(t),2)));
  return Vc+Math.PI/4*bore*bore*x};
 const c=compressor(P,rpm),boost=c.boost,thr=0.06+0.94*P.throttle/100,
  Pman=P0*(1+boost/1.013)*thr,Tchg=c.Tchg,
  Tman=boost===0?T0:(T0+(Tchg-T0)*0.28),Pexh=P0*1.13,
  rho=RHO0*(Pman/P0)*(T0/Tman),air=rho*Vd*ve(P,rpm),afr=P.afr,Spm=2*strk*rpm/60;
 const Q=(air/AFR_S)*LHV*effAfr(afr)*COMB_EFF,spark=effSpark(P,rpm),soc=360-spark,
  dur=56+P.cam*26+rpm/600;
 const xb=thd=>{if(thd<=soc)return 0;const z=(thd-soc)/dur;return z>=1?1:1-Math.exp(-5*z*z)};
 let V=Vol(180),T=Tman,m=Pman*V/(R_AIR*T),th=180,peakP=Pman,work=0,KI=0,knk=false;
 for(let i=1;i<=steps;i++){const th2=180+360*i/steps,V2=Vol(th2),dV=V2-V,dth=th2-th;
  let Pp=m*R_AIR*T/V; const dt=(dth/360)*(60/Math.max(rpm,200)),
   w=2.28*Spm+(th>soc?0.6:0)*Spm,
   hc=HEAT_SCALE*3.26*Math.pow(bore,-0.2)*Math.pow(Pp/1000,0.8)*Math.pow(T,-0.55)*Math.pow(Math.max(w,1),0.8),
   xp=(V-Vc)/(Math.PI/4*bore*bore),Aw=Math.PI*bore*bore/2+Math.PI*bore*xp,
   dQl=hc*Aw*(T-TWALL)*dt,dQ=Q*(xb(th2)-xb(th));
  T+=(dQ-dQl-Pp*dV)/(m*CV); V=V2; th=th2; Pp=m*R_AIR*T/V; peakP=Math.max(peakP,Pp); work+=Pp*dV;
  if(xb(th)<0.92&&th>200){const Tu=Tman*Math.pow(Pp/Pman,(GAMMA-1)/GAMMA),
   tau=1e-4*Math.pow(Pp/30e5,-1.7)*Math.exp(3800/Tu)*Math.exp((P.octane-90)*0.13);
   KI+=dt/Math.max(tau,1e-6); if(KI>=1)knk=true}}
 const pump=(Pman-Pexh)*Vd,imep=work/Vd/1e5+pump/Vd/1e5;
 return{imep,peakP:peakP/1e5,knocking:knk,boost,state:c.state,Tchg}}
function torqueComp(P,rpm){const bore=P.bore/1000,strk=P.strk/1000,Vd=Math.PI/4*bore*bore*strk*P.cyl,
  cy=cycle(P,rpm),Wi=cy.imep*1e5*Vd,Ti=Wi/(4*Math.PI),
  Spm=2*strk*rpm/60,fmep=(0.30+0.004*cy.peakP+0.025*Spm+0.0009*Spm*Spm)*1e5,
  Tf=fmep*Vd/(4*Math.PI),om=rpm*2*Math.PI/60;
 let Tpar=0;
 if(P.induction==='Super'){const b=compressor(P,rpm).boost;Tpar=b*(Vd*1000)*(rpm/1000)*0.55*1000/Math.max(om,1)}
 return{Ti,Tf,Tpar,om,cy}}
function driveTorque(P,rpm){const t=torqueComp(P,rpm);return{tq:t.Ti-t.Tf-t.Tpar,cy:t.cy}}
const gearRatio=(P,g)=>{const n=P.gears,f=3.45,t=0.72;return g<1?1:f*Math.pow(t/f,(Math.min(g,n)-1)/(n-1))};
