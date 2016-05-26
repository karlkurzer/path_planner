#ifndef GRADIENT
#define GRADIENT

#include <vector>

using namespace std;

namespace HybridAStar {

/*!
   \brief A color gradient class for visualizing the cost of nodes

  It has been copied from http://www.andrewnoske.com/wiki/Code_-_heatmaps_and_color_gradients under the WTFPL. Please refer to the author in case of questions.
  \author Andrew Noske
*/
class ColorGradient {
 private:
  struct ColorPoint { // Internal class used to store colors at different points in the gradient.
    float r, g, b;    // Red, green and blue values of our color.
    float val;        // Position of our color along the gradient (between 0 and 1).
    ColorPoint(float red, float green, float blue, float value)
      : r(red), g(green), b(blue), val(value) {}
  };
  vector<ColorPoint> color;      // An array of color points in ascending value.

 public:
  //-- Default constructor:
  ColorGradient()  {  createDefaultHeatMapGradient();  }

  //-- Inserts a new color point into its correct position:
  void addColorPoint(float red, float green, float blue, float value) {
    for (unsigned int i = 0; i < color.size(); i++)  {
      if (value < color[i].val) {
        color.insert(color.begin() + i, ColorPoint(red, green, blue, value));
        return;
      }
    }

    color.push_back(ColorPoint(red, green, blue, value));
  }

  //-- Inserts a new color point into its correct position:
  void clearGradient() { color.clear(); }

  //-- Places a 5 color heapmap gradient into the "color" vector:
  void createDefaultHeatMapGradient() {
    color.clear();
    color.push_back(ColorPoint(0, 0, 1,   0.0f));      // Blue.
    color.push_back(ColorPoint(0, 1, 1,   0.25f));     // Cyan.
    color.push_back(ColorPoint(0, 1, 0,   0.5f));      // Green.
    color.push_back(ColorPoint(1, 1, 0,   0.75f));     // Yellow.
    color.push_back(ColorPoint(1, 0, 0,   1.0f));      // Red.
  }

  //-- Inputs a (value) between 0 and 1 and outputs the (red), (green) and (blue)
  //-- values representing that position in the gradient.
  void getColorAtValue(const float value, float& red, float& green, float& blue) {
    if (color.size() == 0)
    { return; }

    for (unsigned int i = 0; i < color.size(); i++) {
      ColorPoint& currC = color[i];

      if (value < currC.val) {
        ColorPoint& prevC  = color[ max(0, (int)i - 1) ];
        float valueDiff    = (prevC.val - currC.val);
        float fractBetween = (valueDiff == 0) ? 0 : (value - currC.val) / valueDiff;
        red   = (prevC.r - currC.r) * fractBetween + currC.r;
        green = (prevC.g - currC.g) * fractBetween + currC.g;
        blue  = (prevC.b - currC.b) * fractBetween + currC.b;
        return;
      }
    }

    red   = color.back().r;
    green = color.back().g;
    blue  = color.back().b;
    return;
  }
};
}
#endif // GRADIENT

