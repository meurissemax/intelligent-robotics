% INFO0948-2 - Introduction to intelligent robotics
% University of Liege - Academic year 2019-2020

% Taken from a YouTube video.
% URL : https://www.youtube.com/watch?v=b7t6qkfPKuI

function newMesh = simplifyPolygon(polygon)
	% Returns a simplified polygon from 'polygon'
	%
	% 'polygon' is a list of coordinates
	% 'newMesh' is a list of coordinates
	
	number_it = 0;
	
	while true
		number_change = 0;
		newMesh = zeros(2, size(polygon, 2));
		polygon = horzcat(polygon, polygon(:, 2)); %#ok<AGROW>
		size_p = size(polygon, 2);
		
		i = 1;
		k = 1;
		
		while i < size_p - 1
			p1 = polygon(:, i);
			p2 = polygon(:, i + 1);
			
			if norm(p2 - p1) < 0.05
				i = i + 2;
			else
				p3 = polygon(:, i + 2);
				
				v1 = p2 - p1;
				v2 = p3 - p2;
				
				v1 = [v1(1) v1(2) 0] / norm(v1);
				v2 = [v2(1) v2(2) 0] / norm(v2);
				
				area = cross(v1, v2);
				area = abs(area(3));
				
				if area < 0.15
					i = i + 2;
					number_change = number_change + 1;
				else
					i = i + 1;
				end
			end

			newMesh(:, k) = p1;
			k = k + 1;
		end
		
		newMesh(:, k) = newMesh(:, 1);
		newMesh = newMesh(:, 1:k);
		
		polygon = newMesh;
		
		if number_change < 2 || number_it > 7
			break;
		end

		number_it = number_it + 1;
	end
end
