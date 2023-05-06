% % Define a non-convex polygon
% poly = [0 0; 2 0; 2 1; 1 1; 1 2; 0 2];
% 
% % Test line segment that intersects at a vertex
% p1 = [1 1];
% p2 = [1 2];
% [intersect, type] = line_segment_intersect(poly, p1, p2)
% 
% % Plot polygon and line segment
% figure;
% hold on;
% plot(poly(:,1), poly(:,2), 'k-');
% if intersect
%     plot([p1(1) p2(1)], [p1(2) p2(2)], 'ro-');
% else
%     plot([p1(1) p2(1)], [p1(2) p2(2)], 'bx-');
% end
% axis equal;
% title('Line segment intersects at vertex');
% 
% % Test line segment that intersects an edge
% p1 = [0.5 0.5];
% p2 = [2 2];
% [intersect, type] = line_segment_intersect(poly, p1, p2)
% 
% % Plot polygon and line segment
% figure;
% hold on;
% plot(poly(:,1), poly(:,2), 'k-');
% if intersect
%     plot([p1(1) p2(1)], [p1(2) p2(2)], 'ro-');
% else
%     plot([p1(1) p2(1)], [p1(2) p2(2)], 'bx-');
% end
% axis equal;
% title('Line segment intersects an edge');
% 
% % Test line segment that does not intersect polygon
% p1 = [0.5 0.5];
% p2 = [0.8 0.8];
% [intersect, type] = line_segment_intersect(poly, p1, p2)
% 
% % Plot polygon and line segment
% figure;
% hold on;
% plot(poly(:,1), poly(:,2), 'k-');
% if intersect
%     plot([p1(1) p2(1)], [p1(2) p2(2)], 'ro-');
% else
%     plot([p1(1) p2(1)], [p1(2) p2(2)], 'bx-');
% end
% axis equal;
% title('Line segment does not intersect polygon');
% 
% 
% function [intersect, type] = line_segment_intersect(poly, p1, p2)
%     type = '';
%     for i = 1:length(poly)
%         p3 = poly(i,:);
%         p4 = poly(mod(i,length(poly))+1,:);
%         [intersect, type] = is_intersect(p1, p2, p3, p4);
%         if intersect            
%             return;
%         end
%     end
%     intersect = false;
% end
% 
% function [intersect, type] = is_intersect(p1, p2, p3, p4)
%     % Make sure p1, p2, p3, and p4 are all column vectors with two elements
%     if ~isequal(size(p1), [2 1]) || ~isequal(size(p2), [2 1]) || ~isequal(size(p3), [2 1]) || ~isequal(size(p4), [2 1])
%         p1 = reshape(p1, [2,1]);
%         p2 = reshape(p2, [2,1]);
%         p3 = reshape(p3, [2,1]);
%         p4 = reshape(p4, [2,1]);
%     end
% 
%     % Calculate the denominator and numerator of the t and u parameters
%     den = (p4(2) - p3(2)) * (p2(1) - p1(1)) - (p4(1) - p3(1)) * (p2(2) - p1(2));
%     numt = (p4(1) - p3(1)) * (p1(2) - p3(2)) - (p4(2) - p3(2)) * (p1(1) - p3(1));
%     numu = (p2(1) - p1(1)) * (p1(2) - p3(2)) - (p2(2) - p1(2)) * (p1(1) - p3(1));
% 
%     % Check if the line segments intersect
%     if den == 0
%         intersect = false;
%         type = '';
%     else
%         t = numt / den;
%         u = numu / den;
%         if (t >= 0) && (t <= 1) && (u >= 0) && (u <= 1)
%             intersect = true;
% 
%             % Check Slope of the two line segments (p1,p2) and (p3, p4)
%             % If the slope of the two line segments are the same, then the
%             % line segments intersect at a vertex
%             if (p2(2) - p1(2)) / (p2(1) - p1(1)) == (p4(2) - p3(2)) / (p4(1) - p3(1))
%                 type = 'vertex';
%             else
%                 type = 'edge';
%             end
% 
%         else
%             intersect = false;
%             type = '';
%         end
%     end
% 
%     % Check if the line segment intersects at a vertex
%     if ~intersect && (is_vertex(p1, p2, p3, p4) || is_vertex(p3, p4, p1, p2))
%         intersect = true;
%     end
% end
% 
% 
% function d = direction(p1, p2, p3)
%     d = (p3(1) - p1(1)) * (p2(2) - p1(2)) - (p2(1) - p1(1)) * (p3(2) - p1(2));
% end
% 
% function on_seg = on_segment(p1, p2, p3)
%     % Make sure p1, p2, and p3 are all column vectors with two elements
%     if ~isequal(size(p1), [2 1]) || ~isequal(size(p2), [2 1]) || ~isequal(size(p3), [2 1])
%         p1 = reshape(p1, [2,1]);
%         p2 = reshape(p2, [2,1]);
%         p3 = reshape(p3, [2,1]);
%     end
% 
%     % Check if p3 is on the line segment between p1 and p2
%     on_seg = (p3(1) <= max(p1(1), p2(1))) && (p3(1) >= min(p1(1), p2(1))) ...
%              && (p3(2) <= max(p1(2), p2(2))) && (p3(2) >= min(p1(2), p2(2))) ...
%              && (~is_vertex(p1, p2, p3, []) && ~is_vertex(p1, p2, [], p3));
% end
% 
% function vertex = is_vertex(p1, p2, p3, p4)
%     % Make sure p1, p2, p3, and p4 are all column vectors with two elements
%     if ~isequal(size(p1), [2 1]) || ~isequal(size(p2), [2 1]) || ~isequal(size(p3), [2 1]) || ~isequal(size(p4), [2 1])
%         p1 = reshape(p1, [2,1]);
%         p2 = reshape(p2, [2,1]);
%         p3 = reshape(p3, [2,1]);
%         p4 = reshape(p4, [2,1]);
%     end
% 
%     % Check if p3 or p4 coincide with p1 or p2
%     vertex = all(p3 == p1) || all(p3 == p2) || all(p4 == p1) || all(p4 == p2);
% end


% % Define a polygon with vertices (0,0), (1,0), (1,1), and (0,1)
% polygon = [0 0; 1 0; 1 1; 0 1; 0 0];
% 
% % Define line segments to test
% segments = {[0.5 2], [2 0.5];
%             [0 -1], [0 2]; 
%             [0.5 0], [1 0.5]; 
%             [0.5 0], [0 0.5]}; 
% 
% % Plot polygon and line segments
% figure;
% hold on;
% colors = ['r', 'g'];  % Colors for line segments
% for i = 1:size(segments, 1)
%     p1 = segments{i, 1};
%     p2 = segments{i, 2};
%     is_intersecting = isLineSegmentIntersectingPolygon(p1, p2, polygon);
%     if is_intersecting
%         color = colors(2);
%     else
%         color = colors(1);
%     end
%     plot([p1(1) p2(1)], [p1(2) p2(2)], [color '-']);  % Plot line segment
% end
% axis equal;
% plot(polygon(:,1), polygon(:,2), 'k-');  % Plot polygon
% legend('Not intersecting', 'Intersecting');
% 
% function is_intersecting = isLineSegmentIntersectingPolygon(p1, p2, polygon)
%     % Check if the line segment is along any boundary
%     if any(all(bsxfun(@eq, p1, polygon), 2)) || any(all(bsxfun(@eq, p2, polygon), 2))
%         is_intersecting = false;
%         return
%     end
% 
%     % Check if the line segment passes through only one vertex of the polygon
%     idx1 = find(all(bsxfun(@eq, p1, polygon), 2));
%     idx2 = find(all(bsxfun(@eq, p2, polygon), 2));
%     if numel(idx1) == 1 && numel(idx2) == 1 && abs(idx1 - idx2) == 1
%         is_intersecting = false;
%         return
%     end
% 
%     % Check if the line segment intersects with any edge of the polygon
%     for i = 1:size(polygon, 1)
%         j = mod(i, size(polygon, 1)) + 1;
%         if isLineSegmentIntersectingLineSegment(p1, p2, polygon(i,:), polygon(j,:))
%             is_intersecting = true;
%             return
%         end
%     end
% 
%     % If the line segment does not intersect with any edge of the polygon
%     is_intersecting = false;
% end
% 
% function is_intersecting = isLineSegmentIntersectingLineSegment(p1, p2, q1, q2)
%     % Compute the cross products and check their signs
%     r = p2 - p1;
%     s = q2 - q1;
%     a = q1 - p1;
% 
%     if size(r, 2) == 2
%         r = [r, 0];  % Append 0 as the third component to make it a 3D vector
%     end
% 
%     if size(s, 2) == 2
%         s = [s, 0];  % Append 0 as the third component to make it a 3D vector
%     end
% 
%     if size(a, 2) == 2
%         a = [a, 0];  % Append 0 as the third component to make it a 3D vector
%     end
% 
%     num = cross(a, r);
%     den = cross(r, s);
%     if den == 0
%         % The line segments are parallel or collinear
%         is_intersecting = false;
%         return
%     end
%     t = cross(a, s) / den;
%     u = num / den;
%     if t >= 0 && t <= 1 && u >= 0 && u <= 1
%         % The line segments intersect
%         is_intersecting = true;
%     else
%         % The line segments do not intersect
%         is_intersecting = false;
%     end
% end
