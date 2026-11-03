function analyze_tests(dataRoot)
root=fileparts(mfilename('fullpath'));if nargin<1,dataRoot=root;end
out=fullfile(root,'output');if ~isfolder(out),mkdir(out);end;figdir=fullfile(out,'figures');if ~isfolder(figdir),mkdir(figdir);end
names={'Matched_SPWM','Matched_SVPWM','PWM_10kHz','PWM_40kHz','Sampling_100us','DeadTime_2us'};
all=struct;
for k=1:numel(names)
 name=names{k};file=find_record(dataRoot,name);if ~isfile(file),continue;end
 a=load(file);s=a.s;r=struct('settings',a.settings);early=startsWith(name,{'Current','DQ'});
 if early,t0=.3;t1=.5;vb=380*sqrt(2/3);ib=20000/380*sqrt(2/3);else,t0=1.6;t1=2;vb=60/sqrt(3);ib=.1;end
 t=s.Vabc_pu.time;v=s.Vabc_pu.data;i=s.Iabc_pu.data;
 mask=t>=t0&t<t1;tw=t(mask);vw=v(mask,:);iw=i(mask,:);fs=1/median(diff(tw));
 if early
  % Fit the output frequency; use ten coherent periods for harmonic analysis.
  ix=1:20:numel(tw);fc=(40:.1:85);err=arrayfun(@(f)fitres(tw(ix),vw(ix,1),f),fc);[~,best]=min(err);f=fminbnd(@(f)fitres(tw(ix),vw(ix,1),f),fc(best)-.1,fc(best)+.1);
  T=10/f;tr=linspace(t1-T,t1,200001)';tr(end)=[];
  vw=interp1(t,v,tr);iw=interp1(t,i,tr);tw=tr;N=numel(tr);fs=N/T;
 else,f=50;end
 r.fundamental_Hz=f;r.window_s=[tw(1),tw(end)+1/fs];
 r.phase_voltage=harm(vw,vb,10*(early)+20*(~early));r.phase_current=harm(iw,ib,10*(early)+20*(~early));
 r.line_voltage=harm(vw(:,[1,2,3])-vw(:,[2,3,1]),vb,10*(early)+20*(~early));
 d=s.Idq0;td=d.time;yd=d.data;sel=td>=t0&td<=t1;
 if early,target=a.settings.reference_after;qref=a.settings.q_reference;ts=.2;before=a.settings.reference_before;else,target=.126;qref=0;ts=.3;before=.076;end
 r.Id_mean=mean(yd(sel,1));r.Iq_mean=mean(yd(sel,2));r.Id_error=target-r.Id_mean;r.Iq_error=qref-r.Iq_mean;
 r.Id_error_pct=100*r.Id_error/target;
 if early,tm=td;ym=yd(:,1);else
  edges=(0:.02:2)';tm=edges(2:end);ym=zeros(numel(tm),1);
  for j=1:numel(tm),q=td>=edges(j)&td<edges(j+1);ym(j)=mean(yd(q,1));end
 end
 if abs(target-before)>0
  direction=sign(target-before);step=abs(target-before);q=tm>=ts;tx=tm(q);z=direction*(ym(q)-before)/step;
  a10=find(z>=.1,1);a90=find(z>=.9,1);if isempty(a10)||isempty(a90),r.rise_s=NaN;else,r.rise_s=tx(a90)-tx(a10);end
  r.overshoot_pct=max(0,(max(z)-1)*100);outside=find(abs(z-1)>.02*abs(target)/step);if isempty(outside),r.settling_s=0;elseif outside(end)==numel(z),r.settling_s=NaN;else,r.settling_s=tx(outside(end)+1)-ts;end
  r.q_excursion_pu=max(abs(yd(td>=ts,2)-qref));
 end
 if early
  p=s.PLL_frequency;q=p.time>=t0;pmean=mean(p.data(q));r.PLL_mean_Hz=pmean;r.PLL_error_nominal_Hz=pmean-50;r.PLL_error_output_Hz=pmean-f;
  bad=find(abs(p.data-50)>.5);if isempty(bad),r.PLL_lock_s=0;elseif bad(end)==numel(p.time),r.PLL_lock_s=NaN;else,r.PLL_lock_s=p.time(bad(end)+1);end
  beforestep=p.time<.2;tp=p.time(beforestep);yp=p.data(beforestep);bad=find(abs(yp-50)>.5);if isempty(bad),r.PLL_startup_lock_s=0;elseif bad(end)==numel(tp),r.PLL_startup_lock_s=NaN;else,r.PLL_startup_lock_s=tp(bad(end)+1);end
  r.PLL_final_range_Hz=[min(p.data(q)),max(p.data(q))];
  if strcmp(name,'DQNonzero'),r.phase_current.THD40_pct=[];r.phase_current.THD20_pct=[];r.line_voltage.THD40_pct=[];r.line_voltage.THD20_pct=[];end
 end
 all.(name)=r;
 % Consistent, legible figures; references and outputs share their axes.
 h=figure('Visible','off','Color','white','Position',[40 40 1100 660]);tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
 for axis=1:2
  nexttile;plot(td,yd(:,axis),'LineWidth',1);hold on;
  if axis==1
   if early,yr=before+(target-before)*(td>=ts);else,yr=interp1(s.Id_ref.time,s.Id_ref.data,td,'previous','extrap');end
   plot(td,yr,'--','Color',[.75 .2 .15],'LineWidth',1);ylabel('i_d (p.u.)');legend('Measured current','Reference','Location','best');
  else,plot(td,qref+zeros(size(td)),'--','Color',[.75 .2 .15],'LineWidth',1);ylabel('i_q (p.u.)');end
  xlim([0 t1]);xlabel('Time (s)');grid on;box on;set(gca,'FontName','Times New Roman','FontSize',16);
 end
 exportgraphics(h,fullfile(figdir,[name '_dq.png']),'Resolution',240);close(h);
 if early
  h=figure('Visible','off','Color','white','Position',[40 40 1100 660]);tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
  for axis=1:2,nexttile;if axis==1,plot(t(1:10:end),v(1:10:end,:));ylabel('Phase voltage (p.u.)');else,plot(t(1:10:end),i(1:10:end,:));ylabel('Phase current (p.u.)');end;grid on;xlabel('Time (s)');xlim([0 .5]);set(gca,'FontName','Times New Roman','FontSize',16);end
  exportgraphics(h,fullfile(figdir,[name '_abc.png']),'Resolution',240);close(h);
  if strcmp(name,'CurrentStepUp')
   h=figure('Visible','off','Color','white','Position',[40 40 1100 430]);plot(s.PLL_frequency.time,s.PLL_frequency.data,'LineWidth',1);hold on;yline(50,'--');grid on;xlabel('Time (s)');ylabel('Frequency (Hz)');xlim([0 .5]);set(gca,'FontName','Times New Roman','FontSize',16);exportgraphics(h,fullfile(figdir,'PLL.png'),'Resolution',240);close(h);
  end
 end
 fprintf('ANALYZED %s\n',name);clear a s;
end
fid=fopen(fullfile(out,'simulation_metrics.json'),'w');fwrite(fid,jsonencode(all,PrettyPrint=true));fclose(fid);
end
function r=harm(x,base,fbin)
N=size(x,1);a=abs(fft(x))/N*sqrt(2)*base;h=a((1:40)*fbin+1,:);
r=struct('rms',sqrt(mean(x.^2))*base,'fundamental_rms',h(1,:),'fundamental_peak',sqrt(2)*h(1,:),'THD40_pct',100*sqrt(sum(h(2:40,:).^2))./h(1,:),'THD20_pct',100*sqrt(sum(h(2:20,:).^2))./h(1,:));
end
function e=fitres(t,x,f)
A=[ones(size(t)),sin(2*pi*f*t),cos(2*pi*f*t)];e=sum((x-A*(A\x)).^2);
end

function file=find_record(root,name)
file=fullfile(root,[name '.mat']);
if isfile(file),return;end
hits=dir(fullfile(root,'**',[name '.mat']));
if isempty(hits),file='';return;end
assert(isscalar(hits),'More than one record found for %s.',name);
file=fullfile(hits(1).folder,hits(1).name);
end
