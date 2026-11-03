function analyze_quantizers(dataRoot)
root=fileparts(mfilename('fullpath'));if nargin<1,dataRoot=root;end
out=fullfile(root,'output');if ~isfolder(out),mkdir(out);end
all=struct;names={'ADC_SPWM','ADC_SVPWM','DQFixedFrame'};old={'Matched_SPWM','Matched_SVPWM','DQNonzero'};
for k=[1 2]
 file=find_record(dataRoot,names{k});if isempty(file),continue;end
 a=load(file);s=a.s;
 if k<=2,t0=1.6;t1=2;vb=60/sqrt(3);ib=.1;else,t0=.3;t1=.5;vb=380*sqrt(2/3);ib=20000/380*sqrt(2/3);end
 t=s.Vabc_pu.time;q=t>=t0&t<t1;tw=t(q);v=s.Vabc_pu.data(q,:);i=interp1(s.Iabc_pu.time,s.Iabc_pu.data,tw);
 r=struct('settings',a.settings,'end_s',max(t),'window_s',[t0,t1]);
 r.current=harm(tw,i*ib);r.line_voltage=harm(tw,(v(:,[1,2,3])-v(:,[2,3,1]))*vb);
 td=s.Idq0.time;yd=s.Idq0.data;z=td>=t0&td<t1;
 if k<=2,ref=[.126,0];else,ref=[1,.2];end
 r.mean_dq=mean(yd(z,1:2));r.error_dq=ref-r.mean_dq;r.ripple_dq_rms=std(yd(z,1:2),1);
 if k<=2
  edges=(0:.02:2)';tm=edges(2:end);ym=zeros(size(tm));
  for j=1:numel(tm),z=td>=edges(j)&td<edges(j+1);ym(j)=mean(yd(z,1));end
  z=tm>=.3;tx=tm(z);response=(ym(z)-.076)/.05;
  t10=find(response>=.1,1);t90=find(response>=.9,1);r.rise_s=tx(t90)-tx(t10);
  r.overshoot_pct=max(0,100*(max(response)-1));bad=find(abs(response-1)>.02*.126/.05);
  if bad(end)==numel(response),r.settling_s=NaN;else,r.settling_s=tx(bad(end)+1)-.3;end
  for label={'IabcADC','VabcADC'}
   if isfield(s,label{1})
    yy=s.(label{1}).data;scale=6/4096;if strcmp(label{1},'VabcADC'),scale=2.4/4096;end
    r.([label{1} '_integer_grid_error'])=max(abs(yy(:)/scale-round(yy(:)/scale)));
   end
  end
 else
  ix=1:20:numel(tw);f=fminbnd(@(f)fitres(tw(ix),v(ix,1),f),48,52);
  r.fitted_frequency_Hz=f;r.frame_frequency_Hz=mean(s.PLL_frequency.data);
 end
 all.(names{k})=r;
 b=load(find_record(dataRoot,old{k}));
 nrows=2+(k==3);h=figure('Visible','off','Color','w','Position',[30,30,1000,280*nrows]);
 tiledlayout(nrows,1,'TileSpacing','compact','Padding','compact');
 for j=1:2
  nexttile;plot(b.s.Idq0.time(1:20:end),b.s.Idq0.data(1:20:end,j),'Color',[.4 .4 .4]);hold on;
  plot(td(1:20:end),yd(1:20:end,j),'Color',[0 .35 .7]);
  if k<=2&&j==1,yr=.076+.05*(td>=.3);else,yr=ref(j)+zeros(size(td));end
  plot(td(1:20:end),yr(1:20:end),'--','Color',[.75 .15 .1]);
  ylabel(['i_' char('c'+j) ' (p.u.)']);if j==2,ylabel('i_q (p.u.)');end
  xlabel('Time (s)');grid on;xlim([0,t1]);set(gca,'FontName','Times New Roman','FontSize',15);
  if j==1
   if k<=2,legend('Original quantizer','12-bit quantizer','Reference','Location','best');else,legend('Output-driven PLL','Fixed 50 Hz frame','Reference','Location','best');end
  end
 end
 if k==3
  nexttile;plot(b.s.PLL_frequency.time(1:20:end),b.s.PLL_frequency.data(1:20:end),'Color',[.4 .4 .4]);hold on;
  plot(s.PLL_frequency.time(1:20:end),s.PLL_frequency.data(1:20:end),'Color',[0 .35 .7]);
  xlabel('Time (s)');ylabel('Frame frequency (Hz)');grid on;xlim([0,t1]);set(gca,'FontName','Times New Roman','FontSize',15);
 end
 exportgraphics(h,fullfile(out,[names{k} '.png']),'Resolution',180);close(h);
end
fid=fopen(fullfile(out,'analyze_quantizers.json'),'w');assert(fid>=0);fwrite(fid,jsonencode(all,PrettyPrint=true));fclose(fid);
end
function r=harm(t,x)
 N=numel(t);fbin=round(50*N*median(diff(t)));a=abs(fft(x))/N*sqrt(2);H=a((1:40)*fbin+1,:);
 r=struct('rms',sqrt(mean(x.^2)),'fundamental_rms',H(1,:),'THD40_pct',100*sqrt(sum(H(2:40,:).^2))./H(1,:));
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
