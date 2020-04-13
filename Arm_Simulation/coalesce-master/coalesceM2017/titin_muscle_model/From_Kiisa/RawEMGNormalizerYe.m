function [normEMG] = RawEMGNormalizerYe(SignalIn)
%%EMGNormalizer: This function converts a raw EMG signal to percentage of
% activation coeficent between 0-1. This is accomplished by:
% 1. Flitering the low level noises using a high-pass of 1hz
% 2. Flitering the high level noises using a low-pass of 1hz
% 3. Rectifying
% 4. Dividing by half the maxaium valutary contraction (MVC)
% 5. Subtracting the min signal to zero
%
% Author: Christopher D. Whitney (cw729@nau.ed)
% Last Modified: Sep. 14th 2016

  Size = length(SignalIn);
  normEMG = zeros(Size,1);
  
%  
%   % Normalized Method (Thomas S. Buchanan 2004)
%   [b,a] = butter(4,380/5000,'high');
%   [d,c] = butter(4,120/5000,'low');
%     
%   % apply high-pass
%   normEMG = filtfilt(b,a,normEMG);
%   
%   normEMG = abs(normEMG);
%   
%   % apply low-passs
%   normEMG = filtfilt(d,c,normEMG);
    
    [bb,aa]=butter(4,[20/(5000/2) 380/(5000/2)]);
    normEMG=filtfilt(bb,aa,SignalIn);
    abs_data=abs(normEMG);
    [bbb,aaa]=butter(4,10/(5000/2),'low');
    normEMG=filtfilt(bbb,aaa,abs_data);
    
    normEMG = normEMG - min(normEMG);
    
    peak = max( normEMG(50:end) );
    normEMG = normEMG ./ peak;
    normEMG = normEMG - 0.1;
    normEMG(normEMG(:,1) < 0.1) = 0;
    
    wndw = 600;
    normEMG = filtfilt(ones(wndw,1)/wndw, 1, normEMG);
end
