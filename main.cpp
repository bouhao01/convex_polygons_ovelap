#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <sstream>

using namespace std;

typedef struct point
{
    int x;
    int y;
} Point;

static Point vect(const Point &p, const Point &q)
{
    Point res = {q.x - p.x, q.y - p.y};
    return res;
}

static int cross_product(const Point &p, const Point &q)
{
    return p.x * q.y - p.y * q.x;
};

static bool check_intersection(const Point &p_1, const Point &p_2, const Point &q_1, const Point &q_2)
{
    // if orientation_1 is positive => points are counterclockwise
    int orientation_1 = cross_product(vect(p_1, p_2), vect(p_1, q_1)); //p_1, p_2, q_1
    int orientation_2 = cross_product(vect(p_1, p_2), vect(p_1, q_2)); //p_1, p_2, q_2
    int orientation_3 = cross_product(vect(q_1, q_2), vect(q_1, p_1)); //q_1, q_2, p_1
    int orientation_4 = cross_product(vect(q_1, q_2), vect(q_1, p_2)); //q_1, q_2, p_2

    if ((0 > orientation_1 * orientation_2) && (0 > orientation_3 * orientation_4))
        return true;

    //check whether q_1 on the segment [p_1, p_2]
    if ((0 == orientation_1) &&
        (q_1.x <= max(p_1.x, p_2.x) && q_1.x >= min(p_1.x, p_2.x) && q_1.y <= max(p_1.y, p_2.y) && q_1.y >= min(p_1.y, p_2.y)))
        return true;
    //check whether q_2 on the segment [p_1, p_2]
    if ((0 == orientation_2) &&
        (q_2.x <= max(p_1.x, p_2.x) && q_2.x >= min(p_1.x, p_2.x) && q_2.y <= max(p_1.y, p_2.y) && q_2.y >= min(p_1.y, p_2.y)))
        return true;
    //check whether p_1 on the segment [q_1, q_2]
    if ((0 == orientation_3) &&
        (p_1.x <= max(q_1.x, q_2.x) && p_1.x >= min(q_1.x, q_2.x) && p_1.y <= max(q_1.y, q_2.y) && p_1.y >= min(q_1.y, q_2.y)))
        return true;
    //check whether p_2 on the segment [q_1, q_2]
    if ((0 == orientation_4) &&
        (p_2.x <= max(q_1.x, q_2.x) && p_2.x >= min(q_1.x, q_2.x) && p_2.y <= max(q_1.y, q_2.y) && p_2.y >= min(q_1.y, q_2.y)))
        return true;

    //The two segments do not intersect
    return false;
}

static bool pointInConvexPolygon(const vector<Point> &polygon, const Point &p)
{
    // p inside the polygon IFF p is in the left of all polygon's edges (counterclockwise direction)
    // complexity O(size(polygon))
    Point q_, q;
    int index = 1;
    while (index < polygon.size())
    {
        q_ = polygon[index - 1];
        q = polygon[index];
        if (0 > cross_product(vect(q_, q), vect(q_, p)))
            // p is on the right side of [q_, q] => p is outside the polygon.
            break;
        index++;
    }

    if (index >= polygon.size())
        //PolygonA is inside polygonB
        return true;

    return false;
}

static bool overlap(string polygonA, string polygonB)
{

    //Two polygons are overlapping IFF (They have intersecting edges) OR (One is inside the other)

    vector<Point> polygons[2];
    char point_delimiter = ',';

    //Store the two polygons vertices in two vector of points.
    for (int poly_id = 0; poly_id < 2; ++poly_id)
    {
        string polygon = (poly_id == 0) ? polygonA : polygonB;

        int point_end = polygon.find(point_delimiter);
        int x, y;
        while (-1 != point_end)
        {
            string point_str = polygon.substr(0, point_end);
            stringstream stream(point_str);
            stream >> x >> y;
            Point p = {x, y};
            polygons[poly_id].push_back(p);

            polygon = polygon.substr(point_end + 1, polygon.length());
            point_end = polygon.find(point_delimiter);
        }
        // Add the last vertex; the same as the first vertex.
        polygons[poly_id].push_back(polygons[poly_id].front());
        //reverse the order to be counterclockwise if needed
        if (0 > cross_product(vect(polygons[poly_id][0], polygons[poly_id][1]), vect(polygons[poly_id][0], polygons[poly_id][2])))
        {
            reverse(polygons[poly_id].begin(), polygons[poly_id].end());
        }
    }

    //Two vectors hunting each other until they intersect or each one of them make the polygon boundary tour two times.
    // Complexity O(size(polygonA) + size(polygonB))
    int p_A_index = 1, p_B_index = 1, p_A_round = 0, p_B_round = 0;
    while (p_A_round < 2 && p_B_round < 2)
    {
        Point p_ = polygons[0][p_A_index - 1];
        Point p = polygons[0][p_A_index];
        Point q_ = polygons[1][p_B_index - 1];
        Point q = polygons[1][p_B_index];

        if (check_intersection(p_, p, q_, q))
        {
            // Intersection found
            return true;
        }
        if (0 <= cross_product(vect(q_, q), vect(p_, p)))
        {
            if (0 < cross_product(vect(q_, q), vect(q_, p)))
            {
                p_B_index++;
            }
            else
            {
                p_A_index++;
            }
        }
        else
        {
            if (cross_product(vect(p_, p), vect(p_, q)))
            {
                p_A_index++;
            }
            else
            {
                p_B_index++;
            }
        }

        if (p_A_index >= polygons[0].size())
        {
            p_A_index = 1;
            p_A_round++;
        }

        if (p_B_index >= polygons[1].size())
        {
            p_B_index = 1;
            p_B_round++;
        }
    }

    //check whether polygonA is inside polygonB: Check only if the first vertex of A is inside B
    if (pointInConvexPolygon(polygons[1], polygons[0].front()))
    {
        //PolygonA is inside polygonB
        return true;
    }
    //check whether polygonB is inside polygonA: Check only if the first vertex of B is inside A
    if (pointInConvexPolygon(polygons[0], polygons[1].front()))
    {
        //PolygonB is inside polygonA
        return true;
    }

    //there is no polygons overlap
    return false;
}

int main()
{
    string polygon1; // = "0 0,0 1,2 0,0 0";
    string polygon2; // = "1 0,1 1,3 0,1 0";

    getline (cin,polygon1);
    getline (cin,polygon2);

    if (overlap(polygon1, polygon2))
    {
        cout << "OK" << endl;
    }
    else
    {
        cout<< "NOK" << endl;
    }
    return 0;
}
