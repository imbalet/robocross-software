#pragma once
#include <algorithm>
#include <cmath>
#include <iostream>
#include <utility>
#include <vector>

namespace map {

template <typename T> class Map {

private:
  const int local_map_size;
  T default_value;
  const double res;
  const bool x_direction;
  const bool y_direction;
  int center_x, center_y;

public:
  std::vector<T> map;
  std::vector<T> local_map;
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
  void print_data_local(int x, int y);

  void drawline(int x0, int y0, int x1, int y1, T color = 0);

  T &get_map_by_ind(int row, int col);

  std::pair<int, int> get_inds_by_coords(double x, double y) {

    int row = center_y - (y / res);
    int col = center_x + (x / res);
    return std::pair<int, int>(row, col);
  }

  std::pair<int, int> convert_inds_to_rviz_inds(int row, int col) {
    int rv_row = width - 1 - col;
    int rv_col = height - 1 - row;
    return std::pair<int, int>(rv_row, rv_col);
  }
  std::pair<int, int> convert_inds_to_rviz_inds(std::pair<int, int> xy) {
    return convert_inds_to_rviz_inds(xy.first, xy.second);
  }

  std::pair<int, int> convert_inds_to_rviz_inds_local(int row, int col) {
    int rv_row = local_map_size - 1 - row;
    int rv_col = col;
    return std::pair<int, int>(rv_row, rv_col);
  }

  bool update(int x_center, int y_center) {
    int x_start = x_center - local_map_size / 2,
        y_start = y_center - local_map_size / 2;
    int x_stop = x_start + local_map_size, y_stop = y_start + local_map_size;

    if (x_start <= 1) {
      add_col(0);
      return true;
    }
    if (y_start <= 1) {
      add_row(0);
      return true;
    }
    if (x_stop >= width - 1) {
      add_col(1);
      return true;
    }
    if (y_stop >= height - 1) {
      add_row(1);
      return true;
    }
    return false;
  }

  void to_local_map(int x_center, int y_center) {
    if (update(x_center, y_center)) {
      return;
    }
    int x_start = x_center - local_map_size / 2,
        y_start = y_center - local_map_size / 2;
    int x_stop = x_start + local_map_size, y_stop = y_start + local_map_size;

    for (int row = y_start; row < y_stop; ++row) {
      for (int col = x_start; col < x_stop; ++col) {
        auto c_local =
            convert_inds_to_rviz_inds_local(row - y_start, col - x_start);
        local_map.at(c_local.first * local_map_size + c_local.second) =
            get_map_by_ind(row, col);
      }
    }
  }

  void draw_circle(std::pair<int, int> center, int radius, T color,
                   std::vector<T> ignore) {
    int y0 = center.first;
    int x0 = center.second;

    auto draw_horizontal_line = [&](int y, int x_start, int x_end, int color) {
      for (int i = x_start; i <= x_end; ++i) {
        if (y >= 0 && y < height && i >= 0 && i < width) {
          // matrix[y][i] = color;
          T &val = get_map_by_ind(y, i);
          if (std::find(ignore.begin(), ignore.end(), val) == ignore.end())
            val = color;
        }
      }
    };

    int x = radius;
    int y = 0;
    int radius_err = 1 - radius;

    while (x >= y) {
      // Рисуем 8 симметричных точек
      if (y0 + y < height) {
        draw_horizontal_line(y0 + y, x0 - x, x0 + x, color); // Нижняя часть
      }
      if (y0 - y >= 0) {
        draw_horizontal_line(y0 - y, x0 - x, x0 + x, color); // Верхняя часть
      }
      if (y0 + x < height) {
        draw_horizontal_line(y0 + x, x0 - y, x0 + y,
                             color); // Нижняя часть по вертикали
      }
      if (y0 - x >= 0) {
        draw_horizontal_line(y0 - x, x0 - y, x0 + y,
                             color); // Верхняя часть по вертикали
      }

      y += 1;
      if (radius_err < 0) {
        radius_err += 2 * y + 1;
      } else {
        x -= 1;
        radius_err += 2 * (y - x + 1);
      }
    }
  }
};

/*





*/

template <typename T>
Map<T>::Map(int local_map_size, double res, bool x_direction, bool y_direction,
            T default_value)
    : local_map_size(local_map_size / res), default_value(default_value),
      res(res), x_direction(x_direction), y_direction(y_direction),
      map(std::pow(local_map_size * 2 / res, 2), default_value),
      local_map(std::pow(local_map_size / res, 2), default_value) {

  width = local_map_size * 2 / res;
  height = local_map_size * 2 / res;
  center_x = width / 2;
  center_y = height / 2;
}

template <typename T> T &Map<T>::get_map_by_ind(int row, int col) {
  int map_row = height - 1 - row;
  int map_col = col;
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

template <typename T> void Map<T>::add_row(bool direction) {
  if (direction) {
    map.insert(map.begin(), map.size(), default_value);
  } else {
    map.insert(map.end(), map.size(), default_value);
    center_y += height;
  }
  height *= 2;
}

template <typename T> void Map<T>::add_col(bool direction) {
  // 1 - вперед
  auto old_width = width;
  width *= 2;
  if (direction) {
    // map.insert(map.begin() + old_width, old_width, default_value);
    for (int i = 0; i < height; i++) {
      auto pos = map.begin() + old_width + i * width;
      map.insert(pos, old_width, default_value);
    }

  } else {
    for (int i = 0; i < height; i++) {
      auto pos = map.begin() + i * width;
      map.insert(pos, old_width, default_value);
    }
    center_x += old_width;
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

template <typename T> void Map<T>::print_data_local(int x, int y) {

  auto c = get_inds_by_coords(x, y);

  to_local_map(c.second, c.first);

  int x_start = c.second - local_map_size / 2,
      y_start = c.first - local_map_size / 2;
  int x_stop = x_start + local_map_size, y_stop = y_start + local_map_size;

  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      if (row >= y_start && row <= y_stop && col >= x_start && col <= x_stop) {
        std::cout << "\033[42m" << map[row * width + col] << "\033[0m";
      } else {
        std::cout << map[row * width + col];
      }
    }
    std::cout << "\n";
  }
  std::cout << "\n";
}

//"\033[31;1;4mHello\033[0m"
//\033[31;1;4m - red
//\033[42m - green
} // namespace map