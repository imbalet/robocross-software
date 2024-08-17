// #pragma once

// #include <algorithm>
// #include <cmath>
// #include <iostream>
// #include <string>
// #include <utility>
// #include <vector>

// namespace gr {

// enum { TOP, BOTTOM };

// enum { LEFT, RIGHT };

// enum { LB, RB, RT, LT };

// template <typename T> class Grid {

// private:
//   class Part {
//   public:
//     double origin_x, origin_y;
//     int size;
//     double resolution;
//     std::vector<T> data;

//     Part(int size, double resolution, T default_value,
//          std::vector<double> origin)
//         : origin_x(origin[0]), origin_y(origin[1]), size(size),
//           resolution(resolution),
//           data(std::pow(size / resolution, 2), default_value) {}

//     Part(int size, double resolution, T default_value, double origin_x,
//          double origin_y)
//         : origin_x(origin_x), origin_y(origin_y), size(size),
//           resolution(resolution),
//           data(std::pow(size / resolution, 2), default_value) {}

//     T &operator()(int row, int col) { return data.at(row * size + col); }
//   };
//   T default_value;
//   const int local_part_count;
//   const int part_size;
//   const double part_res;
//   const int origin_pos;
//   int mid_part_row;
//   int mid_part_col;
//   std::vector<Part> parts;

// public:
//   std::vector<T> map;
//   int width;
//   int height;

//   Grid(int part_count, int part_size, double part_res, int origin_pos,
//        T default_value, double origin_x, double origin_y);

//   Grid(int part_count, int part_size, double part_res, int origin_pos,
//        T default_value)
//       : Grid(part_count, part_size, part_res, origin_pos, default_value,
//              -part_size / 2.0, -part_size / 2.0) {}

//   void map_to_parts();

//   void parts_to_map();

//   // TODO добавить проверки
//   void go(int row, int col) {
//     mid_part_col += col;
//     mid_part_row += row;
//     parts_to_map();
//   }
//   // TODO добавить проверки
//   void go_to(int row, int col) {
//     mid_part_col = col;
//     mid_part_row = row;
//     parts_to_map();
//   }

//   void add_row(bool direction);
//   void add_col(bool direction);

//   void print_data();
//   void print_all();

//   void drawline(int x0, int y0, int x1, int y1, T color = 0);

//   int size() { return local_part_count * part_size / part_res; }

//   int size_m() { return local_part_count * part_size; }

//   void add_col_go(bool direction) {
//     std::cout << "HERE" << std::endl;
//     map_to_parts();
//     print_all();
//     print_data();
//     add_col(direction);
//     // print_data();
//     print_all();
//     go(0, direction == 1 ? 1 : -1);
//   }

//   void add_row_go(bool direction) {
//     add_row(direction);
//     go(direction == 1 ? 1 : -1, 0);
//   }

//   std::pair<int, int> get_part_inds_by_map_inds(int row, int col) {
//     int part_row = row / (part_size / part_res);
//     int part_col = col / (part_size / part_res);
//     return std::pair<int, int>(part_row, part_col);
//   }

//   std::pair<int, int> get_part_inds_by_local_map_inds(int row, int col) {
//     int part_row = row / (part_size / part_res) +
//                    (mid_part_row - static_cast<int>(local_part_count / 2));
//     int part_col = col / (part_size / part_res) +
//                    (mid_part_col - static_cast<int>(local_part_count / 2));
//     return std::pair<int, int>(part_row, part_col);
//   }

//   // По глобальным кордам х и у
//   std::pair<int, int> get_part_inds_by_coords(int x, int y) {
//     Part &part = get_part(mid_part_row, mid_part_col);

//     double center_x = part.origin_x + part_size / 2.0,
//            center_y = part.origin_y + part_size / 2.0;

//     double offset_x = x - center_x;
//     double offset_y = y - center_y;

//     offset_x += offset_x > 0 ? part_size / 2.0 : -part_size / 2.0;
//     offset_y += offset_y > 0 ? part_size / 2.0 : -part_size / 2.0;

//     int part_offset_x = offset_x / part_size;
//     int part_offset_y = offset_y / part_size;

//     int part_col, part_row;

//     if (origin_pos == LB || origin_pos == RB) {
//       part_row = mid_part_row - part_offset_y;
//     } else {
//       part_row = mid_part_row + part_offset_y;
//     }

//     if (origin_pos == LB || origin_pos == LT) {
//       part_col = mid_part_col + part_offset_x;
//     } else {
//       part_col = mid_part_col - part_offset_x;
//     }

//     return std::pair<int, int>(part_row, part_col);
//   }

//   // центральная часть карты
//   std::pair<int, int> get_mid() {
//     return std::pair<int, int>(mid_part_row, mid_part_col);
//   };

//   std::pair<double, double> get_origin() {
//     int row = mid_part_row + (local_part_count - 1 - mid_part_row);
//     int col = mid_part_col - (local_part_count - 1 - mid_part_col);
//     Part &part = get_part(row, col);
//     return std::pair<double, double>(part.origin_x, part.origin_y);
//   }

//   std::pair<double, double> get_center_coord() {
//     double half_part_size = part_size / 2.0;
//     Part &part = get_part(mid_part_row, mid_part_col);
//     double center_x = half_part_size + part.origin_x;
//     double center_y = half_part_size + part.origin_y;
//     return std::pair<double, double>(center_x, center_y);
//   }

//   Part &get_part(int row, int col);

//   T &get_map_by_ind(int row, int col);

//   std::string print_part(Part part);

//   void print_origins();
// };

// /**/

// /*
// origin_pos -
// 3  2
// 0  1
// */
// template <typename T>
// Grid<T>::Grid(int _local_part_count, int part_size, double part_res,
//               int origin_pos, T default_value, double origin_x, double
//               origin_y)
//     : default_value(default_value), local_part_count(_local_part_count),
//       part_size(part_size), part_res(part_res), origin_pos(origin_pos),
//       parts(std::pow(local_part_count, 2),
//             Part(part_size, part_res, default_value, 0, 0)),
//       map(std::pow(local_part_count * (part_size / part_res), 2),
//           default_value) {

//   width = local_part_count;
//   height = local_part_count;
//   mid_part_row = local_part_count / 2;
//   mid_part_col = local_part_count / 2;

//   int mid_part = mid_part_col;
//   for (int row = 0; row < local_part_count; ++row) {
//     for (int col = 0; col < local_part_count; ++col) {
//       Part &part = get_part(row, col);
//       switch (origin_pos) {
//       case 0:
//         part.origin_x = (col - mid_part) * part_size + origin_x;
//         part.origin_y = (mid_part - row) * part_size + origin_y;
//         break;

//       case 1:
//         part.origin_x = (mid_part - col) * part_size + origin_x;
//         part.origin_y = (mid_part - row) * part_size + origin_y;
//         break;

//       case 2:
//         part.origin_x = (mid_part - col) * part_size + origin_x;
//         part.origin_y = (row - mid_part) * part_size + origin_y;
//         break;

//       case 3:
//         part.origin_x = (col - mid_part) * part_size + origin_x;
//         part.origin_y = (row - mid_part) * part_size + origin_y;
//         break;
//       }
//     }
//   }
// }

// template <typename T>
// typename Grid<T>::Part &Grid<T>::get_part(int row, int col) {
//   return parts.at(row * width + col);
// }

// template <typename T> T &Grid<T>::get_map_by_ind(int row, int col) {
//   return map.at(row * local_part_count * part_size / part_res + col);
// }

// /**
//  * Adding column to matrix.
//  *
//  * @param direction 0 - left side, 1 - right side.
//  */
// template <typename T> void Grid<T>::add_col(bool direction) {
//   map_to_parts();
//   print_all();
//   width += 1;
//   if (direction) {
//     for (int row = 0; row < height; ++row) {
//       int end_ind = row * width + (width - 1);
//       double origin_x;
//       if (origin_pos == LB || origin_pos == LT) {
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
//       if (origin_pos == LB || origin_pos == LT) {
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
// template <typename T> void Grid<T>::add_row(bool direction) {
//   map_to_parts();
//   if (direction) {
//     for (int col = 0; col < width; ++col) {
//       double origin_x = get_part(height - 1, col).origin_x;
//       double origin_y;
//       if (origin_pos == LB || origin_pos == RB) {
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
//       if (origin_pos == LB || origin_pos == RB) {
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

// template <typename T> std::string Grid<T>::print_part(Part part) {
//   std::string answ;
//   int size = part.size * part_res;
//   for (int row = 0; row < size; ++row) {
//     for (int col = 0; col < size; ++col) {
//       answ += std::to_string(part(row, col));
//     }
//     answ += '\n';
//   }
//   answ += '\n';
//   return answ;
// }

// template <typename T> void Grid<T>::map_to_parts() {
//   int part_elements = part_size / part_res;
//   int size = local_part_count * part_elements;
//   for (int row = 0; row < size; ++row) {
//     for (int col = 0; col < size; ++col) {
//       int part_row = row / part_elements;
//       int part_col = col / part_elements;
//       auto a = get_part_inds_by_local_map_inds(row, col);
//       Part &part = get_part(a.first, a.second);
//       part(row - part_row * part_elements, col - part_col * part_elements) =
//           get_map_by_ind(row, col);
//       // print_data();
//     }
//   }
// }

// // TODO хуйня переделать
// // старт индекс неправильно считается и вообще пиздец
// // нужно из роу и кол вычитать старт роу и старт кол
// template <typename T> void Grid<T>::parts_to_map() {
//   if (mid_part_col == 0) {
//     add_col(0);
//     std::cerr << "wrong col 0";
//   } else if (mid_part_col == (width - 1)) {
//     add_col(1);
//     std::cerr << "wrong col 1";
//   }
//   if (mid_part_row == 0) {
//     add_row(0);
//     std::cerr << "wrong row 0";
//   } else if (mid_part_row == (height - 1)) {
//     add_row(1);
//     std::cerr << "wrong row 1";
//   }
//   // начальные столбец и строка
//   int start_row = mid_part_row - local_part_count / 2;
//   int start_col = mid_part_col - local_part_count / 2;

//   double part_cells = part_size / part_res;
//   // double grid_cells = local_part_count * part_cells;

//   for (int row = start_row; row < start_row + local_part_count; ++row) {
//     for (int col = start_col; col < start_col + local_part_count; ++col) {

//       int start_ind =
//           (row - start_row) * (local_part_count * part_cells * part_cells) +
//           (col - start_col) * part_cells;
//       Part &part = get_part(row, col);
//       for (int part_row = 0; part_row < part_size / part_res; ++part_row) {
//         std::copy(part.data.begin() + part_row * (part_size / part_res),
//                   part.data.begin() + part_row * (part_size / part_res) +
//                       (part_size / part_res),
//                   map.begin() + start_ind);
//         start_ind += (local_part_count * part_size / part_res);
//       }
//     }
//   }
// }

// template <typename T>
// void Grid<T>::drawline(int x0, int y0, int x1, int y1, T color) {

//   int size = local_part_count * part_size / part_res;
//   int dx_convert = x1 - x0;
//   int dy_convert = y1 - y0;

//   if (dx_convert == 0) {
//     if (y1 < 0) {
//       y1 = 0;
//     } else if (y1 >= size) {
//       y1 = size - 1;
//     }
//   } else {
//     if (x1 < 0) {
//       x1 = 0;
//       y1 = y0 + dy_convert * (x1 - x0) / dx_convert;
//     } else if (x1 >= size) {
//       x1 = size - 1;
//       y1 = y0 + dy_convert * (x1 - x0) / dx_convert;
//     }
//   }
//   if (dy_convert == 0) {
//     if (x1 < 0) {
//       x1 = 0;
//     } else if (x1 >= size) {
//       x1 = size - 1;
//     }
//   } else {
//     if (y1 < 0) {
//       y1 = 0;
//       x1 = x0 + dx_convert * (y1 - y0) / dy_convert;
//     } else if (y1 >= size) {
//       y1 = size - 1;
//       x1 = x0 + dx_convert * (y1 - y0) / dy_convert;
//     }
//   }

//   int dx = abs(x1 - x0);
//   int dy = abs(y1 - y0);
//   int sx = x0 < x1 ? 1 : -1;
//   int sy = y0 < y1 ? 1 : -1;
//   int err = dx - dy;
//   int e2;

//   while (1) {
//     if (x0 >= 0 && x0 < size && y0 >= 0 && y0 < size) {
//       get_map_by_ind(y0, x0) = color;
//     }

//     if (x0 == x1 && y0 == y1) {
//       break;
//     }

//     e2 = 2 * err;
//     if (e2 > -dy) {
//       err -= dy;
//       x0 += sx;
//     }
//     if (e2 < dx) {
//       err += dx;
//       y0 += sy;
//     }
//   }
// }

// template <typename T> void Grid<T>::print_origins() {
//   auto mid = get_mid();
//   int start_row = mid.first - local_part_count / 2;
//   int start_col = mid.second - local_part_count / 2;

//   for (int row = 0; row < height; row++) {
//     for (int col = 0; col < width; col++) {
//       auto part = get_part(row, col);
//       if (row == mid.first && col == mid.second) {
//         std::cout << "{" << part.origin_x << ";" << part.origin_y << "}";
//       } else if (row >= start_row && row < (start_row + local_part_count) &&
//                  col >= start_col && col < (start_col + local_part_count)) {
//         std::cout << "[" << part.origin_x << ";" << part.origin_y << "]";
//       } else {
//         std::cout << "(" << part.origin_x << ";" << part.origin_y << ")";
//       }
//     }
//     std::cout << "\n";
//   }
//   std::cout << "\n";
// }

// template <typename T> void Grid<T>::print_data() {
//   // parts_to_map();
//   int size = local_part_count * part_size / part_res;

//   for (int row = 0; row < size; row++) {
//     for (int col = 0; col < size; col++) {
//       std::cout << get_map_by_ind(row, col);
//     }
//     std::cout << "\n";
//   }
//   std::cout << "\n";
// }

// //"\033[31;1;4mHello\033[0m"
// //\033[31;1;4m - red
// //\033[42m - green
// template <typename T> void Grid<T>::print_all() {
//   // map_to_parts();
//   auto mid = get_mid();
//   int start_row = mid.first - local_part_count / 2;
//   int start_col = mid.second - local_part_count / 2;

//   int size_w = width * part_size / part_res - 1;
//   int size_h = height * part_size / part_res - 1;

//   for (int row = 0; row < size_h; row++) {
//     for (int col = 0; col < size_w; col++) {

//       auto part_ = get_part_inds_by_map_inds(row, col);

//       auto cell = get_part(part_.first, part_.second)(
//           row - part_.first * part_size / part_res,
//           col - part_.second * part_size / part_res);

//       if (part_.first == mid.first && part_.second == mid.second) {
//         std::cout << "\033[31;1;4m" << cell << "\033[0m";
//       } else if (part_.first >= start_row &&
//                  part_.first < (start_row + local_part_count) &&
//                  part_.second >= start_col &&
//                  part_.second < (start_col + local_part_count)) {
//         std::cout << "\033[42m" << cell << "\033[0m";
//       } else {
//         std::cout << cell;
//       }
//     }
//     std::cout << "\n";
//   }
//   std::cout << "\n";
// }
// } // namespace gr

#pragma once
#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace gr {

enum { TOP, BOTTOM };

enum { LEFT, RIGHT };

enum { LB, RB, RT, LT };

template <typename T> class Grid {

private:
  class Part {
  public:
    double origin_x, origin_y;
    int size;
    double resolution;
    std::vector<T> data;

    Part(int size, double resolution, T default_value,
         std::vector<double> origin)
        : origin_x(origin[0]), origin_y(origin[1]), size(size),
          resolution(resolution),
          data(std::pow(size / resolution, 2), default_value) {}

    Part(int size, double resolution, T default_value, double origin_x,
         double origin_y)
        : origin_x(origin_x), origin_y(origin_y), size(size),
          resolution(resolution),
          data(std::pow(size / resolution, 2), default_value) {}

    T &operator()(int row, int col) { return data.at(row * size + col); }
  };
  T default_value;
  const int local_part_count;
  const int part_size;
  const double part_res;
  const bool x_direction;
  const bool y_direction;
  int mid_part_row;
  int mid_part_col;
  std::vector<Part> parts;

public:
  std::vector<T> map;
  int width;
  int height;

  Grid(int part_count, int part_size, double part_res, bool x_direction,
       bool y_direction, T default_value, double origin_x, double origin_y);

  Grid(int part_count, int part_size, double part_res, bool x_direction,
       bool y_direction, T default_value)
      : Grid(part_count, part_size, part_res, x_direction, y_direction,
             default_value, -part_size / 2.0, -part_size / 2.0) {}

  void map_to_parts();

  void parts_to_map();

  // TODO добавить проверки
  void go(int row, int col) {
    mid_part_col += col;
    mid_part_row += row;
    parts_to_map();
  }
  // TODO добавить проверки
  void go_to(int row, int col) {
    mid_part_col = col;
    mid_part_row = row;
    parts_to_map();
  }

  void add_row(bool direction);
  void add_col(bool direction);

  void print_data();
  void print_all();

  void drawline(int x0, int y0, int x1, int y1, T color = 0);

  int size() { return local_part_count * part_size / part_res; }

  int size_m() { return local_part_count * part_size; }

  void add_col_go(bool direction) {
    if ((direction == 1 && mid_part_col == width - 2) ||
        (direction == 0 && mid_part_col == 1)) {
      add_col(direction);
    }
    go(0, direction == 1 ? 1 : -1);
  }

  void add_row_go(bool direction) {
    if ((direction == 1 && mid_part_row == height - 2) ||
        (direction == 0 && mid_part_row == 1)) {
      add_row(direction);
    }

    go(direction == 1 ? 1 : -1, 0);
  }

  std::pair<int, int> get_part_inds_by_map_inds(int row, int col) {
    int part_row = row / (part_size / part_res);
    int part_col = col / (part_size / part_res);
    return std::pair<int, int>(part_row, part_col);
  }

  std::pair<int, int> get_part_inds_by_local_map_inds(int row, int col) {
    int part_row = row / (part_size / part_res) +
                   (mid_part_row - static_cast<int>(local_part_count / 2));
    int part_col = col / (part_size / part_res) +
                   (mid_part_col - static_cast<int>(local_part_count / 2));
    return std::pair<int, int>(part_row, part_col);
  }

  // По глобальным кордам х и у
  std::pair<int, int> get_part_inds_by_coords(int x, int y) {
    Part &part = get_part(mid_part_row, mid_part_col);

    double center_x = part.origin_x + part_size / 2.0,
           center_y = part.origin_y + part_size / 2.0;

    double offset_x = x - center_x;
    double offset_y = y - center_y;

    offset_x += offset_x > 0 ? part_size / 2.0 : -part_size / 2.0;
    offset_y += offset_y > 0 ? part_size / 2.0 : -part_size / 2.0;

    int part_offset_x = offset_x / part_size;
    int part_offset_y = offset_y / part_size;

    int part_col, part_row;

    if (x_direction == 0) {
      part_col = mid_part_col + part_offset_x;
    } else {
      part_col = mid_part_col - part_offset_x;
    }

    if (y_direction == 0) {
      part_row = mid_part_row - part_offset_y;
    } else {
      part_row = mid_part_row + part_offset_y;
    }

    return std::pair<int, int>(part_row, part_col);
  }

  // центральная часть карты
  std::pair<int, int> get_mid() {
    return std::pair<int, int>(mid_part_row, mid_part_col);
  };

  std::pair<double, double> get_origin() {
    int row, col;
    if (y_direction == 0) {
      row = mid_part_row + (local_part_count - 1 - mid_part_row);
    } else {
      row = mid_part_row - (local_part_count - 1 - mid_part_row);
    }

    if (x_direction == 0) {
      col = mid_part_col + (local_part_count - 1 - mid_part_col);
    } else {
      col = mid_part_col - (local_part_count - 1 - mid_part_col);
    }

    Part &part = get_part(row, col);
    return std::pair<double, double>(part.origin_x, part.origin_y);
  }

  std::pair<double, double> get_center_coord() {
    double half_part_size = part_size / 2.0;
    Part &part = get_part(mid_part_row, mid_part_col);
    double center_x = half_part_size + part.origin_x;
    double center_y = half_part_size + part.origin_y;
    return std::pair<double, double>(center_x, center_y);
  }

  Part &get_part(int row, int col);

  T &get_map_by_ind(int row, int col);

  std::string print_part(Part part);

  void print_origins();
};

/**/

/*
origin_pos -
3  2
0  1
*/
template <typename T>
Grid<T>::Grid(int _local_part_count, int part_size, double part_res,
              bool x_direction, bool y_direction, T default_value,
              double origin_x, double origin_y)
    : default_value(default_value), local_part_count(_local_part_count),
      part_size(part_size), part_res(part_res), x_direction(x_direction),
      y_direction(y_direction),
      parts(std::pow(local_part_count, 2),
            Part(part_size, part_res, default_value, 0, 0)),
      map(std::pow(local_part_count * (part_size / part_res), 2),
          default_value) {

  width = local_part_count;
  height = local_part_count;
  mid_part_row = local_part_count / 2;
  mid_part_col = local_part_count / 2;

  int mid_part = mid_part_col;
  for (int row = 0; row < local_part_count; ++row) {
    for (int col = 0; col < local_part_count; ++col) {
      Part &part = get_part(row, col);

      if (x_direction == 0) {
        part.origin_x = (mid_part - col) * part_size + origin_x;
      } else {
        part.origin_x = (col - mid_part) * part_size + origin_x;
      }

      if (y_direction == 0) {
        part.origin_y = (mid_part - row) * part_size + origin_y;
      } else {
        part.origin_y = (row - mid_part) * part_size + origin_y;
      }
    }
  }
}

template <typename T>
typename Grid<T>::Part &Grid<T>::get_part(int row, int col) {
  return parts.at(row * width + col);
}

template <typename T> T &Grid<T>::get_map_by_ind(int row, int col) {
  return map.at(row * local_part_count * part_size / part_res + col);
}

/**
 * Adding column to matrix.
 *
 * @param direction 0 - left side, 1 - right side.
 */
template <typename T> void Grid<T>::add_col(bool direction) {
  map_to_parts();
  print_all();
  width += 1;
  if (direction) {
    for (int row = 0; row < height; ++row) {
      int end_ind = row * width + (width - 1);
      double origin_x;
      if (x_direction == 0) {
        origin_x = get_part(row, width - 2).origin_x + part_size;
      } else {
        origin_x = get_part(row, width - 2).origin_x - part_size;
      }
      double origin_y = get_part(row, width - 2).origin_y;
      parts.insert(
          parts.begin() + end_ind,
          Part(part_size, part_res, default_value, origin_x, origin_y));
    }
  } else {
    for (int row = 0; row < height; ++row) {
      double origin_x;
      if (x_direction == 0) {
        origin_x = get_part(row, 0).origin_x - part_size;
      } else {
        origin_x = get_part(row, 0).origin_x + part_size;
      }

      double origin_y = get_part(row, 0).origin_y;
      parts.insert(
          parts.begin() + row * width,
          Part(part_size, part_res, default_value, origin_x, origin_y));
    }
    // parts_to_map();
    // map_to_parts();
    mid_part_col += 1;
    // parts_to_map();
  }
  // parts_to_map();
}

/**
 * Adding row to matrix.
 *
 * @param direction 0 - top side, 1 - bottom side.
 */
template <typename T> void Grid<T>::add_row(bool direction) {
  map_to_parts();
  if (direction) {
    for (int col = 0; col < width; ++col) {
      double origin_x = get_part(height - 1, col).origin_x;
      double origin_y;
      if (y_direction == 0) {
        origin_y = get_part(height - 1, col).origin_y - part_size;
      } else {
        origin_y = get_part(height - 1, col).origin_y + part_size;
      }
      parts.push_back(
          Part(part_size, part_res, default_value, origin_x, origin_y));
    }
  } else {
    std::vector<Part> data_to_insert;
    for (int col = 0; col < width; ++col) {
      double origin_x = get_part(0, col).origin_x;
      double origin_y;
      if (y_direction == 0) {
        origin_y = get_part(0, col).origin_y + part_size;
      } else {
        origin_y = get_part(0, col).origin_y - part_size;
      }
      data_to_insert.push_back(
          Part(part_size, part_res, default_value, origin_x, origin_y));
    }
    parts.insert(parts.begin(), data_to_insert.begin(), data_to_insert.end());
    mid_part_row += 1;
  }
  height += 1;
}

template <typename T> std::string Grid<T>::print_part(Part part) {
  std::string answ;
  int size = part.size * part_res;
  for (int row = 0; row < size; ++row) {
    for (int col = 0; col < size; ++col) {
      answ += std::to_string(part(row, col));
    }
    answ += '\n';
  }
  answ += '\n';
  return answ;
}

template <typename T> void Grid<T>::map_to_parts() {
  int part_elements = part_size / part_res;
  int size = local_part_count * part_elements;
  for (int row = 0; row < size; ++row) {
    for (int col = 0; col < size; ++col) {
      int part_row = row / part_elements;
      int part_col = col / part_elements;
      auto a = get_part_inds_by_local_map_inds(row, col);
      Part &part = get_part(a.first, a.second);
      part(row - part_row * part_elements, col - part_col * part_elements) =
          get_map_by_ind(row, col);
      // print_data();
    }
  }
}

// TODO хуйня переделать
// старт индекс неправильно считается и вообще пиздец
// нужно из роу и кол вычитать старт роу и старт кол
template <typename T> void Grid<T>::parts_to_map() {
  if (mid_part_col == 0) {
    add_col(0);
    std::cerr << "wrong col 0";
  } else if (mid_part_col == (width - 1)) {
    add_col(1);
    std::cerr << "wrong col 1";
  }
  if (mid_part_row == 0) {
    add_row(0);
    std::cerr << "wrong row 0";
  } else if (mid_part_row == (height - 1)) {
    add_row(1);
    std::cerr << "wrong row 1";
  }
  // начальные столбец и строка
  int start_row = mid_part_row - local_part_count / 2;
  int start_col = mid_part_col - local_part_count / 2;

  double part_cells = part_size / part_res;
  // double grid_cells = local_part_count * part_cells;

  for (int row = start_row; row < start_row + local_part_count; ++row) {
    for (int col = start_col; col < start_col + local_part_count; ++col) {

      int start_ind =
          (row - start_row) * (local_part_count * part_cells * part_cells) +
          (col - start_col) * part_cells;
      Part &part = get_part(row, col);
      for (int part_row = 0; part_row < part_size / part_res; ++part_row) {
        std::copy(part.data.begin() + part_row * (part_size / part_res),
                  part.data.begin() + part_row * (part_size / part_res) +
                      (part_size / part_res),
                  map.begin() + start_ind);
        start_ind += (local_part_count * part_size / part_res);
      }
    }
  }
}

template <typename T>
void Grid<T>::drawline(int x0, int y0, int x1, int y1, T color) {

  int size = local_part_count * part_size / part_res;
  int dx_convert = x1 - x0;
  int dy_convert = y1 - y0;

  if (dx_convert == 0) {
    if (y1 < 0) {
      y1 = 0;
    } else if (y1 >= size) {
      y1 = size - 1;
    }
  } else {
    if (x1 < 0) {
      x1 = 0;
      y1 = y0 + dy_convert * (x1 - x0) / dx_convert;
    } else if (x1 >= size) {
      x1 = size - 1;
      y1 = y0 + dy_convert * (x1 - x0) / dx_convert;
    }
  }
  if (dy_convert == 0) {
    if (x1 < 0) {
      x1 = 0;
    } else if (x1 >= size) {
      x1 = size - 1;
    }
  } else {
    if (y1 < 0) {
      y1 = 0;
      x1 = x0 + dx_convert * (y1 - y0) / dy_convert;
    } else if (y1 >= size) {
      y1 = size - 1;
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
    if (x0 >= 0 && x0 < size && y0 >= 0 && y0 < size) {
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

template <typename T> void Grid<T>::print_origins() {
  auto mid = get_mid();
  int start_row = mid.first - local_part_count / 2;
  int start_col = mid.second - local_part_count / 2;

  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      auto part = get_part(row, col);
      if (row == mid.first && col == mid.second) {
        std::cout << "{" << part.origin_x << ";" << part.origin_y << "}";
      } else if (row >= start_row && row < (start_row + local_part_count) &&
                 col >= start_col && col < (start_col + local_part_count)) {
        std::cout << "[" << part.origin_x << ";" << part.origin_y << "]";
      } else {
        std::cout << "(" << part.origin_x << ";" << part.origin_y << ")";
      }
    }
    std::cout << "\n";
  }
  std::cout << "\n";
}

template <typename T> void Grid<T>::print_data() {
  // parts_to_map();
  int size = local_part_count * part_size / part_res;

  for (int row = 0; row < size; row++) {
    for (int col = 0; col < size; col++) {
      std::cout << get_map_by_ind(row, col);
    }
    std::cout << "\n";
  }
  std::cout << "\n";
}

//"\033[31;1;4mHello\033[0m"
//\033[31;1;4m - red
//\033[42m - green
template <typename T> void Grid<T>::print_all() {
  // map_to_parts();
  auto mid = get_mid();
  int start_row = mid.first - local_part_count / 2;
  int start_col = mid.second - local_part_count / 2;

  int size_w = width * part_size / part_res - 1;
  int size_h = height * part_size / part_res - 1;

  for (int row = 0; row < size_h; row++) {
    for (int col = 0; col < size_w; col++) {

      auto part_ = get_part_inds_by_map_inds(row, col);

      auto cell = get_part(part_.first, part_.second)(
          row - part_.first * part_size / part_res,
          col - part_.second * part_size / part_res);

      if (part_.first == mid.first && part_.second == mid.second) {
        std::cout << "\033[31;1;4m" << (int)cell << "\033[0m";
      } else if (part_.first >= start_row &&
                 part_.first < (start_row + local_part_count) &&
                 part_.second >= start_col &&
                 part_.second < (start_col + local_part_count)) {
        std::cout << "\033[42m" << (int)cell << "\033[0m";
      } else {
        std::cout << (int)cell;
      }
    }
    std::cout << "\n";
  }
  std::cout << "\n";
}
} // namespace gr
