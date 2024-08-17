#pragma once
#include <algorithm>
#include <cmath>
#include <iostream>
#include <utility>
#include <vector>

namespace map {

template <typename T> class Map {

private:
  T default_value;
  const int local_map_size;
  const double res;
  const bool x_direction;
  const bool y_direction;

public:
  std::vector<T> map;
  int width;
  int height;

  Map(int local_map_size, double res, bool x_direction, bool y_direction,
      T default_value);

  void map_to_parts();

  void parts_to_map();

  void add_row(bool direction);
  void add_col(bool direction);

  void print_data();
  void print_all();

  void drawline(int x0, int y0, int x1, int y1, T color = 0);

  T &get_map_by_ind(int row, int col);

  std::pair<int, int> get_inds_by_coords(double x, double y) {
    int row = static_cast<int>(height / 2) - (y / res);
    int col = static_cast<int>(width / 2) + (x / res);
    return std::pair<int, int>(row, col);
  }

  std::pair<int, int> convert_inds_to_rviz_inds(int x, int y) {
    int row = x;
    int col = height - y;
    return std::pair<int, int>(row, col);
  }
  std::pair<int, int> convert_inds_to_rviz_inds(std::pair<int, int> xy) {
    return convert_inds_to_rviz_inds(xy.first, xy.second);
  }
};

/*





*/

template <typename T>
Map<T>::Map(int local_map_size, double res, bool x_direction, bool y_direction,
            T default_value)
    : local_map_size(local_map_size), default_value(default_value), res(res),
      x_direction(x_direction), y_direction(y_direction),
      map(std::pow(local_map_size * 2 / res, 2), default_value) {

  width = local_map_size * 2 / res;
  height = local_map_size * 2 / res;
}

// /**
//  * Adding column to matrix.
//  *
//  * @param direction 0 - left side, 1 - right side.
//  */
// template <typename T> void Map<T>::add_col(bool direction) {
//   map_to_parts();
//   print_all();
//   width += 1;
//   if (direction) {
//     for (int row = 0; row < height; ++row) {
//       int end_ind = row * width + (width - 1);
//       double origin_x;
//       if (x_direction == 0) {
//         origin_x = get_part(row, width - 2).origin_x + part_size;
//       } else {
//         origin_x = get_part(row, width - 2).origin_x - part_size;
//       }
//       double origin_y = get_part(row, width - 2).origin_y;
//       parts.insert(
//           parts.begin() + end_ind,
//           Part(part_size, part_res, default_value, origin_x, origin_y));
//     }
//   } else {
//     for (int row = 0; row < height; ++row) {
//       double origin_x;
//       if (x_direction == 0) {
//         origin_x = get_part(row, 0).origin_x - part_size;
//       } else {
//         origin_x = get_part(row, 0).origin_x + part_size;
//       }

//       double origin_y = get_part(row, 0).origin_y;
//       parts.insert(
//           parts.begin() + row * width,
//           Part(part_size, part_res, default_value, origin_x, origin_y));
//     }
//     // parts_to_map();
//     // map_to_parts();
//     mid_part_col += 1;
//     // parts_to_map();
//   }
//   // parts_to_map();
// }

// /**
//  * Adding row to matrix.
//  *
//  * @param direction 0 - top side, 1 - bottom side.
//  */
// template <typename T> void Map<T>::add_row(bool direction) {
//   map_to_parts();
//   if (direction) {
//     for (int col = 0; col < width; ++col) {
//       double origin_x = get_part(height - 1, col).origin_x;
//       double origin_y;
//       if (y_direction == 0) {
//         origin_y = get_part(height - 1, col).origin_y - part_size;
//       } else {
//         origin_y = get_part(height - 1, col).origin_y + part_size;
//       }
//       parts.push_back(
//           Part(part_size, part_res, default_value, origin_x, origin_y));
//     }
//   } else {
//     std::vector<Part> data_to_insert;
//     for (int col = 0; col < width; ++col) {
//       double origin_x = get_part(0, col).origin_x;
//       double origin_y;
//       if (y_direction == 0) {
//         origin_y = get_part(0, col).origin_y + part_size;
//       } else {
//         origin_y = get_part(0, col).origin_y - part_size;
//       }
//       data_to_insert.push_back(
//           Part(part_size, part_res, default_value, origin_x, origin_y));
//     }
//     parts.insert(parts.begin(), data_to_insert.begin(),
//     data_to_insert.end()); mid_part_row += 1;
//   }
//   height += 1;
// }

template <typename T> T &Map<T>::get_map_by_ind(int row, int col) {
  int map_row = width - 1 - col;
  int map_col = height - 1 - row;
  return map.at(map_row * width + map_col);
}

template <typename T>
void Map<T>::drawline(int x0, int y0, int x1, int y1, T color) {

  //   int size = local_part_count * part_size / part_res;
  int dx_convert = x1 - x0;
  int dy_convert = y1 - y0;

  if (dx_convert == 0) {
    if (y1 < 0) {
      y1 = 0;
    } else if (y1 >= height) {
      y1 = height - 1;
    }
  } else {
    if (x1 < 0) {
      x1 = 0;
      y1 = y0 + dy_convert * (x1 - x0) / dx_convert;
    } else if (x1 >= width) {
      x1 = width - 1;
      y1 = y0 + dy_convert * (x1 - x0) / dx_convert;
    }
  }
  if (dy_convert == 0) {
    if (x1 < 0) {
      x1 = 0;
    } else if (x1 >= width) {
      x1 = width - 1;
    }
  } else {
    if (y1 < 0) {
      y1 = 0;
      x1 = x0 + dx_convert * (y1 - y0) / dy_convert;
    } else if (y1 >= height) {
      y1 = height - 1;
      x1 = x0 + dx_convert * (y1 - y0) / dy_convert;
    }
  }

  int dx = abs(x1 - x0);
  int dy = abs(y1 - y0);
  int sx = x0 < x1 ? 1 : -1;
  int sy = y0 < y1 ? 1 : -1;
  int err = dx - dy;
  int e2;

  while (1) {
    if (x0 >= 0 && x0 < width && y0 >= 0 && y0 < height) {
      get_map_by_ind(y0, x0) = color;
    }

    if (x0 == x1 && y0 == y1) {
      break;
    }

    e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dx) {
      err += dx;
      y0 += sy;
    }
  }
}

template <typename T> void Map<T>::print_data() {
  // parts_to_map();

  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      std::cout << get_map_by_ind(row, col);
    }
    std::cout << "\n";
  }
  std::cout << "\n";
}

//"\033[31;1;4mHello\033[0m"
//\033[31;1;4m - red
//\033[42m - green
} // namespace map
