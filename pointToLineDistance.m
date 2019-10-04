function distance = pointToLineDistance(pt, v1, v2)
    % v1, v2 is 1x3
    % pt is n x 3
    % check inputs
    if nargin~=3
        error('HJW:point_to_line_distance:nargin',...
            'Incorrect number of inputs, expected 3.');
    end
    if ~isnumeric(pt) || ~any(size(pt,2)==[2 3]) || any(size(pt)==0)
        error('HJW:point_to_line_distance:pt_type_size',...
            'First input (pt) is not numeric or has an incorrect shape.')
    end
    if ~isnumeric(v1) || numel(v1)~=size(pt,2)
        error('HJW:point_to_line_distance:v_type_size',...
            ['Second input (v1) is not numeric or has an incorrect ',...
            'size.' char(10) 'Expected 1x3 or 1x2, which should match ',...
            'the first input.']) %#ok<CHARTEN>
    end
    if ~isnumeric(v2) || numel(v2)~=size(pt,2)
        error('HJW:point_to_line_distance:v_type_size',['Third input (v2) ',...
            'is not numeric or has an incorrect size.' char(10) 'Expected ',...
            '1x3 or 1x2, which should match the first input.']) %#ok<CHARTEN>
    end
    %prepare inputs
    v1=v1(:)';%force 1x3 or 1x2
    if length(v1)==2,v1(3)=0;end%extend 1x2 to 1x3 if needed
    v2=v2(:)';%force 1x3 or 1x2
    if length(v2)==2,v2(3)=0;end%extend 1x2 to 1x3 if needed
    if size(pt,2)==2,pt(1,3)=0;end%extend nx2 to nx3 if needed
    v1_ = repmat(v1,size(pt,1),1);
    v2_ = repmat(v2,size(pt,1),1);
    %actual calculation
    a = v1_ - v2_;
    b = pt - v2_;
    distance = sqrt(sum(cross(a,b,2).^2,2)) ./ sqrt(sum(a.^2,2));
    %this is equivalent to the following line for a single point
    %distance=norm(cross(v1-v2,pt-v2))/norm(v1-v2)
end