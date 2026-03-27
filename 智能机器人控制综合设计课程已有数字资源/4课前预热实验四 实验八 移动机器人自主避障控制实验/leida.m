laser=rossubscriber('/scan');
scan=receive(laser);
tic;            
while toc<180   %stoptime£¨s£©
    scan=receive(laser);
    xy=readCartesian(scan);
    plot(scan)
      
end;