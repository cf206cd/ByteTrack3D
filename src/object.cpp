#include "object.h"
#include "lapjv.h"
#include <cmath>
#include <opencv2/opencv.hpp>
#include <algorithm>
using namespace cv;
using namespace std;
void iou_distance(vector<Object3D> tracks, vector<Object3D> detections, vector<vector<float>>& dists)
{
    dists.clear();
    for(int i=0;i<tracks.size();i++)
    {
        vector<float> dist;
        for(int j=0;j<detections.size();j++)
        {
            dist.push_back(1.0-RoGDIoU(tracks[i], detections[j]));
        }
        dists.push_back(dist);
    }
}

float RoGDIoU(const Object3D& ob1, const Object3D& ob2, float w1, float w2)
{
    // 1. Convert 3D bounding box to BEV view (assuming x,y are BEV plane coordinates, yaw is rotation angle)
    RotatedRect b1(Point2f(ob1.x, ob1.y), Size2f(ob1.w, ob1.l), ob1.yaw * 180.0 / CV_PI);
    RotatedRect b2(Point2f(ob2.x, ob2.y), Size2f(ob2.w, ob2.l), ob2.yaw * 180.0 / CV_PI);

    // 2. Calculate intersection area
    vector<Point2f> interSection;
    float intersection_area = 0.0;
    if (rotatedRectangleIntersection(b1, b2, interSection)) {
        if (interSection.size() >= 3) {
            // Calculate polygon area
            intersection_area = contourArea(interSection);
        }
    }

    // 3. Calculate union area
    float area1 = b1.size.area();
    float area2 = b2.size.area();
    float union_area = area1 + area2 - intersection_area;

    // 4. Calculate minimum enclosing rectangle
    vector<Point2f> points;
    points.push_back(b1.center);
    points.push_back(b2.center);
    // Add four vertices
    Point2f vertices1[4], vertices2[4];
    b1.points(vertices1);
    b2.points(vertices2);
    for (int i = 0; i < 4; i++) {
        points.push_back(vertices1[i]);
        points.push_back(vertices2[i]);
    }
    RotatedRect min_enclosing_rect = minAreaRect(points);
    float c_area = min_enclosing_rect.size.area();

    // 5. Calculate center distance
    float center_dist = norm(b1.center - b2.center);

    // 6. Calculate diagonal distance of enclosing rectangle
    float diag_dist = sqrt(pow(min_enclosing_rect.size.width, 2) + pow(min_enclosing_rect.size.height, 2));

    // 7. Calculate Ro_IoU
    float ro_iou = (union_area > 0) ? (intersection_area / union_area) : 0.0f;

    // 8. Calculate Ro_GDIoU
    float rogdiou = ro_iou - w1 * (c_area - union_area) / c_area - w2 * (center_dist * center_dist) / (diag_dist * diag_dist);

    return rogdiou; 
}


void linear_assignment(vector<vector<float>> cost_matrix, float thresh, vector<vector<int>> &matches, vector<int> &unmatched_a, vector<int> &unmatched_b)
{
    matches.clear();
    unmatched_a.clear();
    unmatched_b.clear();
    if (cost_matrix.size() == 0||cost_matrix[0].size() == 0)
	{
		return;
	}

	vector<int> rowsol; vector<int> colsol;
	float c = lapjv(cost_matrix, rowsol, colsol, true, thresh, true);
	for (int i = 0; i < rowsol.size(); i++)
	{
		if (rowsol[i] >= 0)
		{
			vector<int> match;
			match.push_back(i);
			match.push_back(rowsol[i]);
			matches.push_back(match);
		}
		else
		{
			unmatched_a.push_back(i);
		}
	}

	for (int i = 0; i < colsol.size(); i++)
	{
		if (colsol[i] < 0)
		{
			unmatched_b.push_back(i);
		}
	}
}

double lapjv(const vector<vector<float> > &cost, vector<int> &rowsol, vector<int> &colsol,
	bool extend_cost, float cost_limit, bool return_cost)
{
	vector<vector<float> > cost_c;
	cost_c.assign(cost.begin(), cost.end());

	vector<vector<float> > cost_c_extended;

	int n_rows = cost.size();
	int n_cols = cost[0].size();
	rowsol.resize(n_rows);
	colsol.resize(n_cols);

	int n = 0;
	if (n_rows == n_cols)
	{
		n = n_rows;
	}
	else
	{
		if (!extend_cost)
		{
			// pause and exit if cost matrix is not square and extend_cost is false
			(void)system("pause");
			exit(0);
		}
	}
		
	if (extend_cost || cost_limit < LONG_MAX)
	{
		n = n_rows + n_cols;
		cost_c_extended.resize(n);
		for (int i = 0; i < cost_c_extended.size(); i++)
			cost_c_extended[i].resize(n);

		if (cost_limit < LONG_MAX)
		{
			for (int i = 0; i < cost_c_extended.size(); i++)
			{
				for (int j = 0; j < cost_c_extended[i].size(); j++)
				{
					cost_c_extended[i][j] = cost_limit / 2.0;
				}
			}
		}
		else
		{
			float cost_max = -1;
			for (int i = 0; i < cost_c.size(); i++)
			{
				for (int j = 0; j < cost_c[i].size(); j++)
				{
					if (cost_c[i][j] > cost_max)
						cost_max = cost_c[i][j];
				}
			}
			for (int i = 0; i < cost_c_extended.size(); i++)
			{
				for (int j = 0; j < cost_c_extended[i].size(); j++)
				{
					cost_c_extended[i][j] = cost_max + 1;
				}
			}
		}

		for (int i = n_rows; i < cost_c_extended.size(); i++)
		{
			for (int j = n_cols; j < cost_c_extended[i].size(); j++)
			{
				cost_c_extended[i][j] = 0;
			}
		}
		for (int i = 0; i < n_rows; i++)
		{
			for (int j = 0; j < n_cols; j++)
			{
				cost_c_extended[i][j] = cost_c[i][j];
			}
		}

		cost_c.clear();
		cost_c.assign(cost_c_extended.begin(), cost_c_extended.end());
	}

	double **cost_ptr;
	cost_ptr = new double *[sizeof(double *) * n];
	for (int i = 0; i < n; i++)
		cost_ptr[i] = new double[sizeof(double) * n];

	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{
			cost_ptr[i][j] = cost_c[i][j];
		}
	}

	int* x_c = new int[sizeof(int) * n];
	int *y_c = new int[sizeof(int) * n];

	int ret = lapjv_internal(n, cost_ptr, x_c, y_c);
	if (ret != 0)
	{
		// pause and exit on error
		(void)system("pause");
		exit(0);
	}

	double opt = 0.0;

	if (n != n_rows)
	{
		for (int i = 0; i < n; i++)
		{
			if (x_c[i] >= n_cols)
				x_c[i] = -1;
			if (y_c[i] >= n_rows)
				y_c[i] = -1;
		}
		for (int i = 0; i < n_rows; i++)
		{
			rowsol[i] = x_c[i];
		}
		for (int i = 0; i < n_cols; i++)
		{
			colsol[i] = y_c[i];
		}

		if (return_cost)
		{
			for (int i = 0; i < rowsol.size(); i++)
			{
				if (rowsol[i] != -1)
				{
					//cout << i << "\t" << rowsol[i] << "\t" << cost_ptr[i][rowsol[i]] << endl;
					opt += cost_ptr[i][rowsol[i]];
				}
			}
		}
	}
	else if (return_cost)
	{
		for (int i = 0; i < rowsol.size(); i++)
		{
			opt += cost_ptr[i][rowsol[i]];
		}
	}

	for (int i = 0; i < n; i++)
	{
		delete[]cost_ptr[i];
	}
	delete[]cost_ptr;
	delete[]x_c;
	delete[]y_c;

	return opt;
}

void match(vector<Object3D>& tracks, vector<Object3D>& detections, vector<vector<int>>& matches, vector<int>& unmatched_tracks, vector<int>& unmatched_detections, float match_thresh)
{
    vector<vector<float>> cost_matrix;
    iou_distance(tracks, detections, cost_matrix);
    linear_assignment(cost_matrix, match_thresh, matches, unmatched_tracks, unmatched_detections);
}