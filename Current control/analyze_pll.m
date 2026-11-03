function analyze_pll(dataRoot)
root=fileparts(mfilename('fullpath'));if nargin<1,dataRoot=root;end
out=fullfile(root,'output');if ~isfolder(out),mkdir(out);end;all=struct;
a=load(find_record(dataRoot,'PLLKnownInput'));s=a.s;t=round(s.estimated_frequency.Time*1e6)/1e6;f=s.estimated_frequency.Data;angle=s.estimated_angle.Data;
truth=interp1(s.time,s.frequency,t,'previous');theta=interp1(s.time,s.angle,t);
phase=atan2(sin(angle-theta),cos(angle-theta))*180/pi;
r=struct('settings',a.settings,'end_s',max(t));windows=[.15,.2;.35,.4;.55,.6];events=[0,.2,.4];
for j=1:3
 q=t>=windows(j,1)&t<windows(j,2);r.mean_frequency_error_Hz(j)=mean(f(q)-truth(q));r.max_frequency_error_Hz(j)=max(abs(f(q)-truth(q)));r.mean_phase_error_deg(j)=mean(phase(q));
 q=t>=events(j)&t<events(j)+.2;tx=t(q);err=f(q)-truth(q);bad=find(abs(err)>.5);
 if isempty(bad),r.frequency_settling_s(j)=0;elseif bad(end)==numel(tx),r.frequency_settling_s(j)=NaN;else,r.frequency_settling_s(j)=tx(bad(end)+1)-events(j);end
 bad=find(abs(phase(q))>1);if isempty(bad),r.phase_settling_s(j)=0;elseif bad(end)==numel(tx),r.phase_settling_s(j)=NaN;else,r.phase_settling_s(j)=tx(bad(end)+1)-events(j);end
end
all.PLLKnownInput=r;
fid=fopen(fullfile(out,'PLLKnownInput.json'),'w');assert(fid>=0);fwrite(fid,jsonencode(all.PLLKnownInput,PrettyPrint=true));fclose(fid);
end

function file=find_record(root,name)
file=fullfile(root,[name '.mat']);
if isfile(file),return;end
hits=dir(fullfile(root,'**',[name '.mat']));
if isempty(hits),file='';return;end
assert(isscalar(hits),'More than one record found for %s.',name);
file=fullfile(hits(1).folder,hits(1).name);
end
