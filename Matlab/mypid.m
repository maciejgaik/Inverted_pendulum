function f = mypid(state, pidArr, dt)
persistent prev_errorTh;
persistent prev_errorX;
persistent iTh;
persistent iX;

if isempty(prev_errorTh)==1
    prev_errorTh=0;
end
if isempty(prev_errorX)==1
    prev_errorX=0;
end
if isempty(iTh)==1
    iTh=0;
end
if isempty(iX)==1
    iX=0;
end

KpTh=pidArr(1,1); KpX=pidArr(1,2);
KiTh=pidArr(2,1); KiX=pidArr(2,2);
KdTh=pidArr(3,1); KdX=pidArr(3,2);

destTh=pi;
destX=0;

valueTh=state(3);
valueX=state(1);

errorTh = destTh - valueTh;
errorX = valueX;

pTh = KpTh * errorTh;
iTh = iTh + KiTh * errorTh * dt;
dTh = KdTh * (errorTh - prev_errorTh) / dt;
fTh = pTh + iTh + dTh;

pX = KpX * errorX;
iX = iX + KiX * errorX * dt;
dX = KdX * (errorX - prev_errorX) / dt;
fX = pX + iX + dX;

f = fX + fTh;

prev_errorTh = errorTh;
prev_errorX = errorX;
