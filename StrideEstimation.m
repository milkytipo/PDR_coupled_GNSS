function [strideLength] = StrideEstimation (nameAbbr, fitMode, sampleFreq, acc_xyz, update_points)

% get step frequency and acceleration variance
accMag = sum(acc_xyz'.^2).^0.5'*9.7937;
% accMag = acc_xyz;
step_counter = nnz(update_points);
step_fre = zeros(1,step_counter);
acc_var = zeros(1,step_counter);
strideLength = zeros(1,step_counter);

for j=1:step_counter-1
       
    st1 = update_points(j);
    st2 = update_points(j+1);
    t1 = st1/sampleFreq;
    t2 = st2/sampleFreq;
    step_fre(j) = 1/(t2-t1);
    
    acc_mean = mean(accMag(st1:st2));
    for st =st1:st2
        acc_var(j) = acc_var(j) + ((accMag(st)-acc_mean)^2)/(st2-st1);
    end
    
    switch nameAbbr
        case 'mjb'
            if fitMode == 1
                strideLength(j) = 0.2074 * step_fre(j) + 0.2963;
            else strideLength(j) = 0.1397 * step_fre(j) + 0.008823 * acc_var(j) + 0.3735;
               
            end
        case 'lzq'
            if fitMode == 1
                strideLength(j) = 0.1424 * step_fre(j) + 0.02322 * acc_var(j) + 0.3336;
            else
                strideLength(j) = 0.1424 * step_fre(j) + 0.02322 * acc_var(j) + 0.3336;
            end
        case 'dwf'
            if fitMode == 1
                strideLength(j) = 0.2494 * step_fre(j) + 0.2477;
            else
                strideLength(j) = 0.2384 * step_fre(j) + 0.001096 * acc_var(j) + 0.2613;
            end
        case 'qjc'
            if fitMode == 1
                strideLength(j) = 0.344 * step_fre(j) + 0.1103;
            else
%                 strideLength(j) = 0.2384 * step_fre(j) + 0.001096 * acc_var(j) + 0.2613;
            end
             case 'lcx'
            if fitMode == 1
                strideLength(j) = 0.4221 * step_fre(j) -0.06118;
            else
                strideLength(j) = 0.2384 * step_fre(j) + 0.001096 * acc_var(j) + 0.2613;
            end
        case 'ldh'
            if fitMode == 1
                strideLength(j) = 0.2984 * step_fre(j) + 0.1808;
            else
                strideLength(j) = 0.1793 * step_fre(j) + 0.01122 * acc_var(j) + 0.3338;
            end
            
    end
    
end