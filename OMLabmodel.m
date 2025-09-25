classdef OMLabmodel
    %OMLABMODEL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Property1
    end
    
    methods
        function obj = OMLabmodel(inputArg1,inputArg2)
            %OMLABMODEL Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end

        function derivativeStates = test(stackedStates, params)
            
        end

        function [stackedStates , stackMetaData] = StackSignals(states)

        end

        function states = UnstackSignals(stackedStates , stackMetadata)

        end

        function statesOut = Run(t, initialState, inputs, params)

            [stackedInitialState , stackMetaDataInitialState] = StackSignals(initialState);
            [stackedInputs , stackMetaDataInputs] = StackSignals(inputs);
            params.stackMetaDataInputs = stackMetaDataInputs;
            params.stackMetaDataInitialState = stackMetaDataInitialState;

            [t, stackedStates] = ode45(@(ti,xi)SOMETHING( ti, xi, interp1(t,stackedInputs,ti)', params), t, stackedInitialState);
        
            statesOut = UnstackSignals(stackedStates , stackMetaDataInitialState);
        end

    end

    methods(Static)
        function [xout, wout, wouti, xsim, Sout, Gout] = ModelRun( t, w, wn, A, T, x0, S0, G0, learningGain)

            m = height(A)-1;
            n = height(x0);


            % stack the inital states
            state0 = [x0(:); x0(:); S0(:); G0(:)];

            [t, out] = ode45(@(ti,xi)AttractorNetworkOnlineLearning( ti, xi, interp1(t,w,ti)', interp1(t,wn,ti)', A, T, n, learningGain), t, state0);





            % unstack the states trhough time
            xout = nan(n,length(t));
            for i=1:length(t)
                xout(:,i) = reshape(out(i,1:n),n,1);
            end
            xsim = nan(n,length(t));
            for i=1:length(t)
                xsim(:,i) = reshape(out(i,n+(1:n)),n,1);
            end

            Sout = nan(n,m,length(t));
            for i=1:length(t)
                Sout(:,:,i) = reshape(out(i,2*n+(1:(n*m))),n,m);
            end
            Gout = nan(m-1,m-1,length(t));
            for i=1:length(t)
                Gout(:,:,i) = reshape(out(i,2*n+n*m  + (1:((m-1)*(m-1)))),m-1,m-1);
            end

            % calculate the angular velocity as well, which will be useful for
            % plots
            wout = nan(m-1,length(t));
            wouti= nan(m-1,length(t));
            dxout = diff(xout,1,2)./repmat(diff(t),1, n)';

            for i=1:length(t)-1
                S = Sout(:,:,i);
                P = (S'*S)\S';
                Ti = T;
                Ti(2:end,2:end,:) = -Ti(2:end,2:end,:);
                EQ = tensorprod(T,P*xout(:,i),1,1)'; % for extrinsic velocity
                GQ = tensorprod(Ti,P*xout(:,i),1,1)'; % for intrinsic velocity
                wout(:,i) = EQ*P*dxout(:,i);
                wouti(:,i) = GQ*P*dxout(:,i);
            end
        end
    end
end

