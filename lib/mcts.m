%CLASS_INTERFACE Example MATLAB class wrapper to an underlying C++ class
classdef mcts < handle
    properties (SetAccess = private, Hidden = true)
        objectHandle; % Handle to the underlying C++ class instance
    end
    methods
        %% Constructor - Create a new C++ class instance 
        function this = mcts(varargin)
            this.objectHandle = mcts_interface_mex('new', varargin{:});
        end
        
        %% Destructor - Destroy the C++ class instance
        function delete(this)
            mcts_interface_mex('delete', this.objectHandle);
        end

        %% Get planned action
        function varargout = get_action(this, varargin)
            [varargout{1:nargout}] = mcts_interface_mex('get_action', this.objectHandle, varargin{:});
        end
    end
end