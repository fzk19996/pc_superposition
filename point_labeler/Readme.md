整个点云被分割成了280个tiles，每个tiles的数据结构

struct Tile {
    int32_t i, j;                   // tile coordinates
    std::vector<uint32_t> indexes;  // scan indexes
    float x, y, size;               // actual world coordinates.
};