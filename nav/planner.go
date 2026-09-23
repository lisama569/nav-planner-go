package nav

import (
	"math"
	"time"
)

// GlobalPlanner 在栅格地图上做全局路径搜索（八邻域 A*）。
//
// 确定性：邻居按固定顺序枚举，开放集中代价相同时按栅格行优先顺序
// 打破平局，因此相同（地图, 起点, 目标）必然得到相同路径。
type GlobalPlanner struct {
	cfg Config
}

// NewGlobalPlanner 创建全局规划器。
func NewGlobalPlanner(cfg Config) *GlobalPlanner { return &GlobalPlanner{cfg: cfg} }

// neighborDirs 是固定顺序的八邻域方向（先四正交后对角，行优先）。
var neighborDirs = [8][3]float64{
	{-1, 0, 1}, // 上
	{0, -1, 1}, // 左
	{0, 1, 1},  // 右
	{1, 0, 1},  // 下
	{-1, -1, math.Sqrt2},
	{-1, 1, math.Sqrt2},
	{1, -1, math.Sqrt2},
	{1, 1, math.Sqrt2},
}

// Search 在给定地图上搜索从 start 到 goal 的全局路径。
//
// 失败分类：
//   - FailMapMissing：地图为空；
//   - FailStartInvalid/FailGoalInvalid：位姿为 NaN、越界或落入障碍；
//   - FailUnreachable：开放集耗尽仍未到达目标；
//   - FailTimeout：超过 deadline（周期性检查）。
func (p *GlobalPlanner) Search(m *GridMap, start, goal Pose, deadline *time.Time, clk Clock) (*Path, error) {
	if clk == nil {
		clk = WallClock{}
	}
	if m == nil {
		return nil, NewError(FailMapMissing, "global search requires a map")
	}
	if isNaNpose(start) {
		return nil, NewError(FailStartInvalid, "start pose contains NaN: %+v", start)
	}
	if isNaNpose(goal) {
		return nil, NewError(FailGoalInvalid, "goal pose contains NaN: %+v", goal)
	}
	sc := m.WorldToCell(start.X, start.Y)
	gc := m.WorldToCell(goal.X, goal.Y)
	if !m.InBounds(sc) || m.IsBlocked(sc) {
		return nil, NewError(FailStartInvalid,
			"start %v maps to cell %s which is out of bounds or blocked", start, sc)
	}
	if !m.InBounds(gc) || m.IsBlocked(gc) {
		return nil, NewError(FailGoalInvalid,
			"goal %v maps to cell %s which is out of bounds or blocked", goal, gc)
	}
	if sc == gc {
		return reconstructPath(map[Cell]Cell{}, sc, m), nil
	}

	gScore := map[Cell]float64{sc: 0}
	came := map[Cell]Cell{}
	closed := map[Cell]bool{}
	open := map[Cell]bool{sc: true}

	for len(open) > 0 {
		// 每轮检查时限（一次时间比较，开销可忽略），保证小地图也能及时超时。
		if deadline != nil && !clk.Now().Before(*deadline) {
			return nil, NewError(FailTimeout,
				"global search exceeded deadline while exploring %d cells", len(closed))
		}

		cur := pickLowest(open, gScore, func(c Cell) float64 {
			return gScore[c] + p.heuristic(c, gc)
		})
		if cur == gc {
			return reconstructPath(came, cur, m), nil
		}
		delete(open, cur)
		closed[cur] = true

		for _, d := range neighborDirs {
			nb := Cell{Row: cur.Row + int(d[0]), Col: cur.Col + int(d[1])}
			if closed[nb] || !m.InBounds(nb) || m.IsBlocked(nb) {
				continue
			}
			// 对角移动要求两个正交邻格均自由，防止贴角穿越。
			if d[2] == math.Sqrt2 {
				corner1 := Cell{Row: cur.Row + int(d[0]), Col: cur.Col}
				corner2 := Cell{Row: cur.Row, Col: cur.Col + int(d[1])}
				if m.IsBlocked(corner1) || m.IsBlocked(corner2) {
					continue
				}
			}
			tentative := gScore[cur] + float64(d[2])
			if old, ok := gScore[nb]; !ok || tentative < old {
				gScore[nb] = tentative
				came[nb] = cur
				open[nb] = true
			}
		}
	}
	return nil, NewError(FailUnreachable,
		"goal cell %s cannot be reached from start cell %s: open set exhausted", gc, sc)
}

// pickLowest 在开放集中按 (f=g+h, 行优先 Cell) 全序选取最优节点，
// 保证代价相同时结果与 map 遍历顺序无关。
func pickLowest(open map[Cell]bool, gScore map[Cell]float64, f func(Cell) float64) Cell {
	var best Cell
	bestF := math.Inf(1)
	for c := range open {
		fv := f(c)
		if fv < bestF || (fv == bestF && cellLess(c, best)) {
			best, bestF = c, fv
		}
	}
	return best
}

// cellLess 定义栅格行优先全序。
func cellLess(a, b Cell) bool {
	if a.Row != b.Row {
		return a.Row < b.Row
	}
	return a.Col < b.Col
}

// heuristic 是与八邻域移动代价一致的 octile 启发函数。
func (p *GlobalPlanner) heuristic(a, b Cell) float64 {
	dx := math.Abs(float64(a.Col - b.Col))
	dy := math.Abs(float64(a.Row - b.Row))
	if dx > dy {
		dx, dy = dy, dx
	}
	return (dy-dx)*1 + dx*math.Sqrt2
}

// reconstructPath 从 came 表回溯，并以栅格中心世界坐标生成路径。
func reconstructPath(came map[Cell]Cell, cur Cell, m *GridMap) *Path {
	var cells []Cell
	for {
		cells = append(cells, cur)
		prev, ok := came[cur]
		if !ok {
			break
		}
		cur = prev
	}
	// 反转为从起点到目标的顺序。
	for i, j := 0, len(cells)-1; i < j; i, j = i+1, j-1 {
		cells[i], cells[j] = cells[j], cells[i]
	}
	path := &Path{}
	for _, c := range cells {
		x, y := m.CellCenter(c)
		path.Nodes = append(path.Nodes, PathNode{C: c, X: x, Y: y})
	}
	return path
}

// isNaNpose 报告位姿中是否含有 NaN。
func isNaNpose(p Pose) bool {
	return math.IsNaN(p.X) || math.IsNaN(p.Y) || math.IsNaN(p.Theta)
}
