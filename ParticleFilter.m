classdef ParticleFilter < handle
%                        Particle information (Particles)
% ------------------------------------------------------------------------
% weight|current position|last position|SL param|heading bais|life |   
%    1  |   2       3    |    4      5 |   6   7|      8     |  9  |       
%       |                |             |        |            |     |
% ------------------------------------------------------------------------
    %% Public properties
    properties (Access = public)
        ParticleNumber = 500;
        Particles     = [];
        ori_location  = [0 0];
        last_heading   = 0;
        current_heading = 0;
        STEP_CALI     = true;
        HEADING_CALI  = true; 
        BINARY_WEIGHT = true;
        stride_std    = 0.15
        heading_std   = 10/180*pi
        SL_param      = [0.244 0];
        SL_param_std  = [0.07 0.04];    
        resample_threshold = 1;% 1-> always resample;0-> never resample
%       [0.244 0.257;0.3827 0.0243;0.1175 0.4273;0.3293 0.0888];
    end
    %% Public methods
    methods (Access = public)
        function obj = ParticleFilter(varargin)
            for i = 1:2:nargin
                if  strcmp(varargin{i}, 'ParticleNumber'), obj.ParticleNumber = varargin{i+1};
                elseif strcmp(varargin{i}, 'OriLocX'), obj.ori_location(1,1) = varargin{i+1};
                elseif strcmp(varargin{i}, 'OriLocY'), obj.ori_location(1,2) = varargin{i+1};    
                elseif strcmp(varargin{i}, 'OriHeading'), obj.last_heading = varargin{i+1};obj.current_heading = varargin{i+1};    
                elseif  strcmp(varargin{i}, 'StepCali'), obj.STEP_CALI = varargin{i+1};
                elseif  strcmp(varargin{i}, 'HeadingCali'), obj.HEADING_CALI = varargin{i+1};
                elseif  strcmp(varargin{i}, 'BinaryWeight'), obj.BINARY_WEIGHT = varargin{i+1};
                elseif  strcmp(varargin{i}, 'HeadingStd'), obj.heading_std = varargin{i+1};    
                elseif  strcmp(varargin{i}, 'SLParam_K'),  obj.SL_param(1,1) = varargin{i+1};
                elseif  strcmp(varargin{i}, 'SLParam_B'),  obj.SL_param(1,2) = varargin{i+1};    
                elseif  strcmp(varargin{i}, 'SLParamStd_K'), obj.SL_param_std(1,1) = varargin{i+1};
                elseif  strcmp(varargin{i}, 'SLParamStd_B'), obj.SL_param_std(1,2) = varargin{i+1};
                elseif  strcmp(varargin{i}, 'ResampleThreshold'), obj.resample_threshold = varargin{i+1};    
                else error('Invalid argument');
                end
            end;
            % initial particle weights
            obj.Particles = zeros(obj.ParticleNumber,9);
            if obj.BINARY_WEIGHT == true
                obj.Particles(:,1) = 1;  % weight
            else
                obj.Particles(:,1) = 1/obj.ParticleNumber;
            end
            % initial particle locations
            % current location
            obj.Particles(:,2) = normrnd(obj.ori_location(1),obj.stride_std,obj.ParticleNumber,1); 
            obj.Particles(:,3) = normrnd(obj.ori_location(2),obj.stride_std,obj.ParticleNumber,1);
            % last location
            obj.Particles(:,4:5) = obj.Particles(:,2:3);
            % stride length Model
            obj.Particles(:,6:7) = obj.getSLParam(obj.SL_param,obj.SL_param_std,obj.ParticleNumber);
            % heading error
            obj.Particles(:,8) = normrnd(0,obj.heading_std,obj.ParticleNumber,1);
            obj.Particles(:,9) = ones(obj.ParticleNumber,1);
        end
        function isturn = IsTurn(obj)
            headingDiff =  atan(tan(obj.current_heading - obj.last_heading))*180/pi;
%             disp(['Heading difference is ',num2str(headingDiff)]);
            if headingDiff  > 25
                isturn =  true;
%                 disp('Turning Detected!');
            else
                isturn =  false;
            end
        end
        function obj = SateUpdate(obj, heading_now, stride_freq)
        % after this function, weight of particle is not equal & can be zero
            % update last heading
            obj.last_heading   = obj.current_heading;    
            p = obj.Particles;
            % last_p = current_p
            p(:,4:5) = p(:,2:3);
            % update heading error
            p(:,8)   = normrnd(0,obj.heading_std,obj.ParticleNumber,1);
            % calculate current headings
            if obj.HEADING_CALI == true
                if obj.BINARY_WEIGHT ==true
                    survivor_heading_bias = mean(obj.Particles(:,8).*obj.getParticleWeights);
                else
                    survivor_heading_bias = sum(obj.Particles(:,8).*obj.getParticleWeights);
                end
                obj.current_heading = survivor_heading_bias + heading_now;
                current_headings = p(:,8) + obj.current_heading;
            else
                obj.current_heading = heading_now;
                current_headings = p(:,8) + obj.current_heading;
            end
            SLs = stride_freq*p(:,6) + p(:,7);
            %%
            p(:,2:3) = [SLs.*sin(current_headings) SLs.*cos(current_headings)] + obj.getParticleCurrentLoc;  

            % weight update
            for i=1:obj.ParticleNumber
                p(i,1) = p(i,1)*obj.RouteAccessable(p(i,2:3),p(i,4:5));
            end
            survivorIdx = find(p(:,1)>0);
            deadIdx = p(:,1)==0;
            % update every particles' life
            p(survivorIdx,9) = p(survivorIdx,9)+1;
            p(deadIdx,9) = 1;
            obj.Particles = p;
        end
        function obj = Resample(obj)
            % calculate effective number of particles
            weights = getParticleWeights(obj);
            % Normalized weight
            if sum(weights) == 0
                error('~~~~~~~~Particles all died~~~~~~~~');
            end
            weights = weights/sum(weights);
            weights = weights(weights>0);
            Neff = 1/sum(weights.^2);
            if Neff >= obj.resample_threshold*obj.ParticleNumber
                % no need to resample  
                if obj.BINARY_WEIGHT == false
                    % normalize particles' weight when it's not binary weight
                    obj.Particles(:,8) = obj.getParticleWeights/sum(obj.getParticleWeights);
                end
                return;
            else
                % resample takes place
                p = obj.Particles;
                if obj.BINARY_WEIGHT == true
                    % when is binary weight, dead particles are only
                    % replaced by survival ones
                    survivorIdx = find(p(:,1)>0);
                    deadIdx = p(:,1)==0;
                    survivorNum = length(survivorIdx);
%                     disp([num2str(survivorNum/obj.ParticleNumber*100) '% Survivor']);
                    duplicate_num = obj.ParticleNumber-survivorNum;
                    % rand select survivor particle to duplicate
                    to_duplicate_idx = survivorIdx(ceil(rand(duplicate_num,1)*survivorNum));
                    if obj.STEP_CALI == true && obj.IsTurn
                    % when turn occurs SL param is inherited from last generation
                        p(deadIdx,1:8) = p(to_duplicate_idx,1:8);
                    else
                    % SL param remain unchanged
                        p(deadIdx,[1:5,8]) = p(to_duplicate_idx,[1:5,8]);
                    end
                else
                    % this branch of function hasn't been tested and is unreliable
                    resmaple_idx = obj.resampleSystematic( getParticleWeights(obj) );
                    if obj.STEP_CALI == true && obj.IsTurn
                        % when turn occurs SL param is inherited from last generation
                        p = p(resmaple_idx,:);
                    else
                        % SL param is drawn from norm distribution rather than the survival generation if there is no turn
                        p = p(resmaple_idx,:);
                        duplicate_idx = obj.findDuplicate( resmaple_idx );
                        duplicate_num = length(duplicate_idx);
                        p(duplicate_idx,6:7) = obj.getSLParam(obj.SL_param,obj.SL_param_std,duplicate_num);
                    end
                end
                obj.Particles = p;
            end
        end
        function weights = getParticleWeights(obj)
            weights = obj.Particles(:,1);
        end
        function current_loc = getParticleCurrentLoc(obj)
            current_loc = obj.Particles(:,2:3);
        end
        function last_loc = getParticleLastLoc(obj)
            last_loc = obj.Particles(:,4:5);
        end
        function [loc,obj] = getWeightedLocation(obj)
            if obj.BINARY_WEIGHT == true
                if nnz(getParticleWeights(obj)) ~= obj.ParticleNumber
                    disp('Resampling takes place unexpected!');
                    obj = Resample(obj);
                end
%                     loc = sum(getParticleCurrentLoc(obj))/obj.ParticleNumber;
                    loc = median(getParticleCurrentLoc(obj));
            else
                if sum(getParticleWeights(obj)) ~= 1
                    disp('Resampling takes place unexpected!');
                    obj = Resample(obj);
                end
                loc = sum(getParticleCurrentLoc(obj).*...
                          [getParticleWeights(obj) getParticleWeights(obj)]);
            end
        end
        function [weighted_loc,obj] = Iterate(obj, heading_now, stride_freq)
            % include state update, resampling, & weighted location
            % acquiration
            obj = SateUpdate(obj, heading_now, stride_freq);
            obj = Resample(obj);
            [weighted_loc,obj] = getWeightedLocation(obj);
        end
        function plotlocation(obj)
            ori = obj.ori_location;
            cp = obj.getParticleCurrentLoc;
            scatter(cp(:,1),cp(:,2),10,'b');axis equal;hold on;
            hs = plot(ori(1),ori(2),'rp');legend(hs,'Start Point');hold off;
        end
        function plotSLParams(obj)
            SL_P = obj.Particles(:,6:7);
            freq = (1:0.05:2.7)';
            freq = [freq ones(size(freq))];
            SL_record = zeros(obj.ParticleNumber,length(freq));
            for i=1:obj.ParticleNumber
                SL_record(i,:) = SL_P(i,:)*freq';
            end
            plot(freq(:,1)',SL_record*100,'--b+');axis([1 2.8 40 110]);
            ylabel('SL/cm');xlabel('Freq/Hz');
        end
    end
    methods(Static = true, Access = private)
        function idx = resampleSystematic( weights )
            idx = zeros(size(weights));
            N = length(weights);
            Q = cumsum(weights);
            T = linspace(0,1-1/N,N) + rand(1)/N;
            T(N+1) = 1;
            i=1;j=1;
            while (i<=N && j<=N),
                if (T(i)<Q(j))
                    idx(i)=j;
                    i=i+1;
                else
                    j=j+1;
                end
            end
        end
        function idx = resampleResidual( weights )
            idx = zeros(length(weights),1);
            M = length(weights);
            Ns = floor(M .* weights);
            R = sum( Ns );
            M_rdn = M-R;
            Ws = (M .* weights - floor(M .* weights))/M_rdn;
            i=1;
            for j=1:M,
                for k=1:Ns(j),
                    idx(i)=j;
                    i = i +1;
                end
            end;
            Q = cumsum(Ws);Q(M)=1; % Just in case...
            while (i<=M),
                sampl = rand(1,1);  % (0,1]
                j=1;
                while (Q(j)<sampl),j=j+1;
                end;
                idx(i)=j;
                i=i+1;
            end
        end
        function duplicate_idx = findDuplicate( idx )
            [~,first_idx] = unique(idx,'first');
            duplicate_idx = setdiff(1:length(idx),first_idx);
        end
        function weight = RouteAccessable(PS,PE)
%             weight = 1;return;
            global map;
            % get knn line idx
            PM = (PS+PE)/2;
            if PM(1)<=map.Xmin || PM(1)>=map.Xmax ||  PM(2)<=map.Ymin || PM(2)>=map.Ymax
                weight = 0;
                return;
            else
                knnLineIdx = map.nnIdx{round((PM(1)-map.Xmin)/map.gridSize)+1,round((PM(2)-map.Ymin)/map.gridSize)+1};
                compareSegSp = map.segment.sp(knnLineIdx,:);
                compareSegEp = map.segment.ep(knnLineIdx,:);
                weight = 1;
                for i=1:length(knnLineIdx)
                    if IntersectForLine(PS,PE,compareSegSp(i,:),compareSegEp(i,:))
                        weight = 0;break;
                    end
                end
            end
        end
        function positives = getPositives( positive_mean, stdval, len )
            % u -  sigma ~ u +  simga 68.27%
            % u - 1.96sigma ~ u - 1.96sigma 95%
            % u - 2.58sigma ~ u - 2.58sigma 99%
            if positive_mean > 2.58*stdval, len1 = len*1.1;
            elseif positive_mean > 1.96*stdval, len1 = len*1.2;
            elseif positive_mean > stdval, len1 = len*2;
            end
            tmp = normrnd(positive_mean,stdval,len1,1);
            tmp = tmp(tmp>0);
            positives = tmp(1:len);        
        end
        function SL_p = getSLParam( param, dev, n)
            center = [1.85 0.7];    % get this from paper [1.8Hz,0.7m(SL)]
            SL_p = zeros(n,2);
            SL_p(:,1) = normrnd(param(1),dev(1),n,1);
            SL_p(:,2) = normrnd(center(2)+param(2)-center(1)*SL_p(:,1),dev(2)*ones(n,1),n,1);
        end
    end
 end