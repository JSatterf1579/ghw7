#ifndef COLOR_H
#define COLOR_H


class Color
{
public:
	double r,g,b;

	Color() { r = 0; g = 0; b = 0;}
	Color(double rr, double gg, double bb)
	{
		r = rr;
		g = gg;
		b = bb;
	}
	Color operator*(double num)
	{
		return Color(r * num, g * num, b * num);
	}
	Color operator*(Color c)
	{
		return Color(r * c.r, g * c.g, b * c.b);
	}

	Color operator+(Color c)
	{
		return Color(c.r + r, c.g + g, c.b + b);
	}
	Color operator-(Color c)
	{
		return Color(r - c.r, g - c.g, b - c.b);
	}
	Color Clamp(double min, double max){
		Color temp = Color(r,g,b);
		if(temp.r > max){
			temp.r = max;
		} else if (temp.r < min){
			temp.r = min;
		}
		if(temp.g > max){
			temp.g = max;
		} else if (temp.g < min){
			temp.g = min;
		}
		if(temp.b > max){
			temp.b = max;
		} else if (temp.b < min){
			temp.b = min;
		}
		return temp;
	}
};

#endif