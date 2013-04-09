#ifndef __POINT_H_INCLUDED__
#define __POINT_H_INCLUDED__

class Point {
	int x;
	int y;
	
public:
	Point() {}
	Point(int x, int y) {
		SetX(x);
		SetY(y);
	}

	int GetX() {
		return x;
	}

	void SetX(int x) {
		this->x = x;
	}

	int GetY() {
		return y;
	}

	void SetY(int y) {
		this->y = y;
	}
};

#endif