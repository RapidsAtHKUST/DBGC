#ifndef COMMON_POLYLINES
#define COMMON_POLYLINES

#include <cassert>
#include <deque>
#include <stack>
#include <unordered_map>
#include <vector>


/**
 * @brief identity of a line in blocks
 * \c bid_ block id
 * \c pid_ polyline id in block
 * \c reversed_ whether the polyline is reversed
 */
struct LineId {
    int bid_;
    int pid_;
    bool reversed_;
    LineId () : bid_(0) , pid_(0) , reversed_(false) {}
    LineId (int bid, int pid, bool reversed)
    : bid_(bid) , pid_(pid) , reversed_(reversed) {}
};

/**
 * @brief find a polyline by the head or tail.
 * \b key: point index in point cloud
 * \b value: index of polylines, of \c stack type
 *     Each element is of \c LineId type
 */
using Point2PolylineIndex = std::unordered_map<int, std::stack<LineId>>;

/**
 * @brief get another endpoint of a line
 * @param[in] end id of one end point of a line
 * @param[in] pindex the Point2PolylineIndex structure
 * @param[in] blocks the collection of lines
 * @return another endpoint of a line
 */
inline int get_another_end (
    const int end, const Point2PolylineIndex& pindex, 
    const std::vector<std::vector<std::deque<int>>>& blocks
) {
    const int bid = pindex.at(end).top().bid_;
    const int pid = pindex.at(end).top().pid_;
    bool reversed = pindex.at(end).top().reversed_;

    const std::deque<int>& pre_pline = blocks[bid][pid];
    if (reversed)
    {
        assert(pre_pline.back() == end);
        return pre_pline.front();
    }
    else
    {
        assert(pre_pline.front() == end);
        return pre_pline.back();
    }
}


/**
 * @brief extend a polyline using index, each point only
 *      belongs to one polyline
 * @param[in] index point cloud or spherical point cloud index
 * @param[in] max_dis the max distance of adjacent points
 * @param[in] visited record if a point is visited
 * @param[out] polyline the result polyline
 * @param[in] with_dir record whether polyline is extended 
 *      with direction constrain
 */
template <typename T>
void extend_polyline (
    const T& index,
    const float max_dis,
    std::vector<bool>& visited,
    std::deque<int>& polyline,
    const bool with_dir
) {
    float min_dis = std::numeric_limits<float>::max();
    unsigned int last_idx = polyline.back(), cur_idx;

    while (
        cur_idx = index.get_nearest_nbr(
            last_idx, max_dis, visited, min_dis, with_dir, true
        ),
        min_dis < max_dis
    ) {
        visited[cur_idx] = true;
        polyline.push_back(cur_idx);
        last_idx = cur_idx;
    }
    
    last_idx = polyline[0];
    while (
        cur_idx = index.get_nearest_nbr(
            last_idx, max_dis, visited, min_dis, with_dir, false
        ),
        min_dis < max_dis
    ) {
        visited[cur_idx] = true;
        polyline.push_front(cur_idx);
        last_idx = cur_idx;
    }
}


/**
 * @brief extend a polyline with hierarchy manner
 * @param[in] index point cloud or sperical point cloud index
 * @param[in] max_dis the max distance of adjacent points
 * @param[in] visited record if a point is visited
 * @param[out] polyline the result polyline
 * @param[in] with_dir record whether polyline is extended 
 *      with direction constrain
 */
template <typename T>
void extend_polyline_hierarchy (
    const T& index,
    const Point2PolylineIndex& pindex,
    const float max_dis,
    std::vector<bool>& visited,
    std::deque<int>& polyline,
    const std::vector<std::vector<std::deque<int>>>& blocks
){
    float min_dis = std::numeric_limits<float>::max();
    unsigned int last_idx = polyline.back(), cur_idx;
    
    while (
        cur_idx = index.get_nearest_nbr(
            last_idx, max_dis, visited, min_dis, false, true
        ),
        min_dis < max_dis
    ) {
        // if the point is the start or end of another line
        if (pindex.find(cur_idx) != pindex.end())
        {
            int another_end = get_another_end(cur_idx, pindex, blocks);
            visited[another_end] = true;
        }
        visited[cur_idx] = true;
        polyline.push_back(cur_idx);
        last_idx = cur_idx;
    }

    last_idx = polyline[0];
    while (
        cur_idx = index.get_nearest_nbr(
            last_idx, max_dis, visited, min_dis, false, false
        ),
        min_dis < max_dis
    ) {
        if (pindex.find(cur_idx) != pindex.end())
        {
            int another_end = get_another_end(cur_idx, pindex, blocks);
            visited[another_end] = true;
        }
        visited[cur_idx] = true;
        polyline.push_front(cur_idx);
        last_idx = cur_idx;
    }
}

#endif